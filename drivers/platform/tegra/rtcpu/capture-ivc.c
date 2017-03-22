/*
 * Capture IVC driver
 *
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/tegra-capture-ivc.h>

#include <linux/completion.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>

/* Referred from capture-scheduler.c defined in rtcpu-fw */
#define NUM_CAPTURE_CHANNELS 64

/* Temporary ids for the clients whose channel-id is not yet allocated */
#define NUM_CAPTURE_TRANSACTION_IDS 64

#define TOTAL_CHANNELS (NUM_CAPTURE_CHANNELS + NUM_CAPTURE_TRANSACTION_IDS)
#define TRANS_ID_START_IDX NUM_CAPTURE_CHANNELS

struct tegra_capture_ivc_cb_ctx {
	struct list_head node;
	uint32_t idx; /* trans_id */
	tegra_capture_ivc_cb_func cb_func;
	const void *priv_context;
};

struct tegra_capture_ivc {
	struct device *dev;
	struct tegra_ivc_channel *chan;
	struct mutex cb_ctx_lock;
	struct mutex ivc_wr_lock;
	struct work_struct work;
	wait_queue_head_t write_q;
	struct tegra_capture_ivc_cb_ctx cb_ctx[TOTAL_CHANNELS];
	spinlock_t avl_ctx_list_lock;
	struct list_head avl_ctx_list;
};

/*
 * Referred from CAPTURE_MSG_HEADER structure defined
 * in camrtc-capture-messages.h, in rtcpu and UMD.
 */
struct tegra_capture_ivc_msg_header {
	uint32_t msg_id;
	union {
		uint32_t channel_id;
		uint32_t transaction;
	};
} __aligned(8);

/*
 * Referred from CAPTURE_CONTROL_MSG and CAPTURE_MSG structures defined
 * in camrtc-capture-messages.h, in rtcpu and UMD. Only exception is,
 * the msg-id specific structures are opaque here.
 */
struct tegra_capture_ivc_resp {
	struct tegra_capture_ivc_msg_header header;
	void *resp;
};

static int tegra_capture_ivc_tx(struct tegra_ivc_channel *chan,
				const void *req, size_t len)
{
	struct tegra_capture_ivc *civc = tegra_ivc_channel_get_drvdata(chan);
	int ret;

	if (WARN_ON(!chan->is_ready))
		return -EIO;

	if (!tegra_ivc_can_write(&chan->ivc)) {
		ret = wait_event_interruptible(civc->write_q,
				tegra_ivc_can_write(&chan->ivc));
		if (ret)
			return ret;
	}

	ret = mutex_lock_interruptible(&civc->ivc_wr_lock);
	if (ret)
		return -ERESTARTSYS;

	ret = tegra_ivc_write(&chan->ivc, req, len);
	if (ret < 0)
		dev_err(&chan->dev, "IVC write error: %d\n", ret);

	mutex_unlock(&civc->ivc_wr_lock);

	return ret;
}

static struct tegra_capture_ivc *__scivc_control;
static struct tegra_capture_ivc *__scivc_capture;

int tegra_capture_ivc_control_submit(const void *control_desc, size_t len)
{
	struct tegra_ivc_channel *chan;
	int err;

	/* Whether capture-ivc driver is probed? */
	if (!__scivc_control)
		return -ENODEV;

	chan = to_tegra_ivc_channel(__scivc_control->dev);
	err = tegra_capture_ivc_tx(chan, control_desc, len);

	return err;
}
EXPORT_SYMBOL(tegra_capture_ivc_control_submit);

int tegra_capture_ivc_capture_submit(const void *capture_desc, size_t len)
{
	struct tegra_ivc_channel *chan;
	int err;

	if (!__scivc_capture)
		return -ENODEV;

	chan = to_tegra_ivc_channel(__scivc_capture->dev);
	err = tegra_capture_ivc_tx(chan, capture_desc, len);

	return err;
}
EXPORT_SYMBOL(tegra_capture_ivc_capture_submit);

int tegra_capture_ivc_register_control_cb(
		tegra_capture_ivc_cb_func control_resp_cb,
		uint32_t *trans_id, const void *priv_context)
{
	struct tegra_capture_ivc *civc;
	struct tegra_capture_ivc_cb_ctx *cb_ctx;
	int ret;

	/* Check if inputs are valid */
	if (!control_resp_cb || !trans_id)
		return -EINVAL;

	if (!__scivc_control)
		return -ENODEV;

	civc = __scivc_control;

	spin_lock(&civc->avl_ctx_list_lock);
	if (list_empty(&civc->avl_ctx_list)) {
		spin_unlock(&civc->avl_ctx_list_lock);
		return -EAGAIN;
	}

	cb_ctx = list_first_entry(&civc->avl_ctx_list,
			struct tegra_capture_ivc_cb_ctx, node);

	list_del(&cb_ctx->node);
	spin_unlock(&civc->avl_ctx_list_lock);

	ret = mutex_lock_interruptible(&civc->cb_ctx_lock);
	if (ret)
		return -ERESTARTSYS;

	/* Check if trans_id is valid */
	if ((cb_ctx->idx < TRANS_ID_START_IDX) ||
		(cb_ctx->idx >= TOTAL_CHANNELS)) {
		ret = -EINVAL;
		goto fail;
	}

	/* Check if cb_func already exists */
	if (cb_ctx->cb_func) {
		ret = -EEXIST;
		goto fail;
	}

	*trans_id = cb_ctx->idx;

	cb_ctx->cb_func = control_resp_cb;
	cb_ctx->priv_context = priv_context;

fail:
	mutex_unlock(&civc->cb_ctx_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_register_control_cb);

int tegra_capture_ivc_notify_chan_id(uint32_t chan_id, uint32_t trans_id)
{
	struct tegra_capture_ivc *civc;

	/* Check if ids are valid */
	if ((chan_id >= NUM_CAPTURE_CHANNELS) ||
		(trans_id < NUM_CAPTURE_CHANNELS) ||
			(trans_id >= TOTAL_CHANNELS))
		return -EINVAL;

	if (!__scivc_control)
		return -ENODEV;

	civc = __scivc_control;

	if (mutex_lock_interruptible(&civc->cb_ctx_lock))
		return -ERESTARTSYS;

	/* Update cb_ctx index */
	civc->cb_ctx[chan_id].cb_func = civc->cb_ctx[trans_id].cb_func;
	civc->cb_ctx[chan_id].priv_context =
			civc->cb_ctx[trans_id].priv_context;
	civc->cb_ctx[chan_id].idx = 0; /* For invalid idx check later */

	/* Reset trans_id cb_ctx fields */
	civc->cb_ctx[trans_id].cb_func = NULL;
	civc->cb_ctx[trans_id].priv_context = NULL;
	civc->cb_ctx[trans_id].idx = trans_id;
	mutex_unlock(&civc->cb_ctx_lock);

	spin_lock(&civc->avl_ctx_list_lock);
	list_add_tail(&civc->cb_ctx[trans_id].node, &civc->avl_ctx_list);
	spin_unlock(&civc->avl_ctx_list_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_notify_chan_id);

int tegra_capture_ivc_register_capture_cb(
		tegra_capture_ivc_cb_func capture_status_ind_cb,
		uint32_t chan_id, const void *priv_context)
{
	struct tegra_capture_ivc *civc;
	int ret;

	if ((chan_id >= NUM_CAPTURE_CHANNELS) || !capture_status_ind_cb)
		return -EINVAL;

	if (!__scivc_capture)
		return -ENODEV;

	civc = __scivc_capture;

	ret = mutex_lock_interruptible(&civc->cb_ctx_lock);
	if (ret)
		return -ERESTARTSYS;

	if (civc->cb_ctx[chan_id].cb_func) {
		ret = -EEXIST;
		goto fail;
	}

	civc->cb_ctx[chan_id].cb_func = capture_status_ind_cb;
	civc->cb_ctx[chan_id].priv_context = priv_context;

fail:
	mutex_unlock(&civc->cb_ctx_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_register_capture_cb);

int tegra_capture_ivc_unregister_control_cb(uint32_t id)
{
	struct tegra_capture_ivc *civc;

	/* id could be temporary trans_id or rtcpu-allocated chan_id */
	if (id >= TOTAL_CHANNELS)
		return -EINVAL;

	if (!__scivc_control)
		return -ENODEV;

	civc = __scivc_control;

	if (mutex_lock_interruptible(&civc->cb_ctx_lock))
		return -ERESTARTSYS;

	civc->cb_ctx[id].cb_func = NULL;
	civc->cb_ctx[id].priv_context = NULL;

	mutex_unlock(&civc->cb_ctx_lock);

	/*
	 * If its trans_id, client encountered an error before or during
	 * chan_id update, in that case the corresponding cb_ctx
	 * needs to be added back in the avilable cb_ctx list.
	 *
	 * Check also if the cb_ctx already part of the list.
	 */
	if ((id >= TRANS_ID_START_IDX) &&
		(civc->cb_ctx[id].idx < TRANS_ID_START_IDX)) {
		civc->cb_ctx[id].idx = id;
		spin_lock(&civc->avl_ctx_list_lock);
		list_add_tail(&civc->cb_ctx[id].node, &civc->avl_ctx_list);
		spin_unlock(&civc->avl_ctx_list_lock);
	}

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_unregister_control_cb);

int tegra_capture_ivc_unregister_capture_cb(uint32_t chan_id)
{
	struct tegra_capture_ivc *civc;

	if (chan_id >= NUM_CAPTURE_CHANNELS)
		return -EINVAL;

	if (!__scivc_capture)
		return -ENODEV;

	civc = __scivc_capture;

	if (mutex_lock_interruptible(&civc->cb_ctx_lock))
		return -ERESTARTSYS;

	civc->cb_ctx[chan_id].cb_func = NULL;
	civc->cb_ctx[chan_id].priv_context = NULL;

	mutex_unlock(&civc->cb_ctx_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_unregister_capture_cb);

static void tegra_capture_ivc_worker(struct work_struct *work)
{
	struct tegra_capture_ivc *civc = container_of(work,
					struct tegra_capture_ivc, work);
	struct tegra_ivc_channel *chan = civc->chan;

	WARN_ON(!chan->is_ready);

	while (tegra_ivc_can_read(&chan->ivc)) {
		const struct tegra_capture_ivc_resp *msg =
			tegra_ivc_read_get_next_frame(&chan->ivc);
		size_t len = chan->ivc.frame_size;

		/* Check if message is valid */
		if (len < sizeof(*msg) ||
			msg->header.channel_id >= TOTAL_CHANNELS) {
			dev_warn(&chan->dev, "Invalid rtcpu response chan %d\n",
				msg->header.channel_id);
			goto skip;
		}

		/* Check if callback function available */
		if (!civc->cb_ctx[msg->header.channel_id].cb_func) {
			dev_info(&chan->dev, "No callback func for chan %d\n",
				msg->header.channel_id);
			goto skip;
		}

		/* Invoke client callback.*/
		civc->cb_ctx[msg->header.channel_id].cb_func(msg,
			civc->cb_ctx[msg->header.channel_id].priv_context);

skip:
		tegra_ivc_read_advance(&chan->ivc);
	}
}

static void tegra_capture_ivc_notify(struct tegra_ivc_channel *chan)
{
	struct tegra_capture_ivc *civc = tegra_ivc_channel_get_drvdata(chan);

	wake_up(&civc->write_q);
	schedule_work(&civc->work);
}

#define NV(x) "nvidia," #x

static int tegra_capture_ivc_probe(struct tegra_ivc_channel *chan)
{
	struct device *dev = &chan->dev;
	struct tegra_capture_ivc *civc;
	const char *service;
	int ret;
	uint32_t trans_id;

	civc = devm_kzalloc(dev, (sizeof(*civc)), GFP_KERNEL);
	if (unlikely(civc == NULL))
		return -ENOMEM;

	ret = of_property_read_string(dev->of_node, NV(service),
			&service);
	if (ret) {
		dev_err(dev, "missing <%s> property\n", NV(service));
		return ret;
	}

	chan->is_ready = false;
	civc->chan = chan;
	civc->dev = dev;

	mutex_init(&civc->cb_ctx_lock);
	mutex_init(&civc->ivc_wr_lock);

	/* Initialize ivc_work */
	INIT_WORK(&civc->work, tegra_capture_ivc_worker);

	/* Initialize wait queue */
	init_waitqueue_head(&civc->write_q);

	/* transaction-id list of available callback contexts */
	spin_lock_init(&civc->avl_ctx_list_lock);
	INIT_LIST_HEAD(&civc->avl_ctx_list);

	/* Add all the free cb-contexts to the transcation-id list */
	trans_id = TRANS_ID_START_IDX;
	while (trans_id < TOTAL_CHANNELS) {
		civc->cb_ctx[trans_id].idx = trans_id;
		list_add_tail(&civc->cb_ctx[trans_id].node,
				&civc->avl_ctx_list);
		trans_id++;
	}

	tegra_ivc_channel_set_drvdata(chan, civc);

	if (!strcmp("capture-control", service)) {
		if (WARN_ON(__scivc_control != NULL))
			return -EEXIST;
		__scivc_control = civc;
	} else if (!strcmp("capture", service)) {
		if (WARN_ON(__scivc_capture != NULL))
			return -EEXIST;
		__scivc_capture = civc;
	} else {
		dev_err(dev, "Unknown ivc channel %s\n", service);
		return -EINVAL;
	}

	return 0;
}

static int tegra_capture_ivc_ready(struct tegra_ivc_channel *chan)
{
	chan->is_ready = true;
	return 0;
}

static void tegra_capture_ivc_remove(struct tegra_ivc_channel *chan)
{
	struct tegra_capture_ivc *civc = tegra_ivc_channel_get_drvdata(chan);

	if (__scivc_control == civc)
		__scivc_control = NULL;
	else if (__scivc_capture == civc)
		__scivc_capture = NULL;
	else
		dev_err(civc->dev, "Unknown ivc channel\n");
}

static struct of_device_id tegra_capture_ivc_channel_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-capture-control" },
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-capture" },
	{ },
};

static const struct tegra_ivc_channel_ops tegra_capture_ivc_ops = {
	.probe	= tegra_capture_ivc_probe,
	.ready	= tegra_capture_ivc_ready,
	.remove	= tegra_capture_ivc_remove,
	.notify	= tegra_capture_ivc_notify,
};

static struct tegra_ivc_driver tegra_capture_ivc_driver = {
	.driver = {
		.name	= "tegra-capture-ivc",
		.bus	= &tegra_ivc_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = tegra_capture_ivc_channel_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_capture_ivc_ops,
};

tegra_ivc_subsys_driver_default(tegra_capture_ivc_driver);
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra Capture IVC driver");
MODULE_LICENSE("GPL");
