/*
 * VI NOTIFY driver for T186
 *
 * Copyright (c) 2015-2016 NVIDIA Corporation.  All rights reserved.
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

#include <linux/completion.h>
#include <linux/mailbox_client.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tegra_camera_rtcpu.h>

#include "vi/vi_notify.h"
#include "nvhost_acm.h"

struct mbox_vi_notify_msg {
	union {
		struct {
			u8 type;
			u8 channel;
			u8 pad1[2];
			union {
				u32 syncpt_ids[3];
				u32 mask;
			};
		};
		char size[128];
	};
};

enum {
	TEGRA_IVC_VI_CLASSIFY,
	TEGRA_IVC_VI_SET_SYNCPTS,
	TEGRA_IVC_VI_RESET_CHANNEL,
};

struct mbox_vi_notify {
	struct vi_notify_dev *vi_notify;
	struct platform_device *vi;
	struct mbox_chan *mbox;
	struct mbox_client client;
	u32 tags;
	u16 channels;
	struct completion ack;
};

/* Mailbox */
static void mbox_vi_notify_process(struct mbox_vi_notify *mvi,
					const struct vi_notify_msg *msg)
{
	switch (VI_NOTIFY_TAG_TAG(msg->tag)) {
	case 1: /* acknowledgement */
		complete(&mvi->ack);
		break;
	}
}

static void mbox_vi_notify_rx(struct mbox_client *cl, void *data)
{
	struct mbox_vi_notify *mvi = container_of(cl, struct mbox_vi_notify,
							client);
	struct tegra_ivc_mbox_msg *msg = data;
	const struct vi_notify_msg *entries = msg->data;
	int len = msg->length;

	/* Receive VI Notify events */
	while (len >= sizeof(*entries)) {
		if (VI_NOTIFY_TAG_VALID(entries->tag))
			vi_notify_dev_recv(mvi->vi_notify, entries);
		else
			mbox_vi_notify_process(mvi, entries);

		entries++;
		len -= sizeof(*entries);
	}
}

/* Vi Notify */
static int mbox_vi_notify_send(struct device *dev,
				struct mbox_vi_notify_msg *req)
{
	struct mbox_vi_notify *mvi = dev_get_drvdata(dev);
	struct tegra_ivc_mbox_msg msg = {
		sizeof(*req),
		req,
	};
	long ret;

	if (mvi->tags == 0 && mvi->channels == 0) {
		ret = nvhost_module_busy(mvi->vi);
		if (ret)
			return ret;
	}

	ret = mbox_send_message(mvi->mbox, &msg);
	if (ret < 0)
		return ret;

	/* Wait for RTCPU to acknowledge the request. This fixes races such as:
	 * - RTCPU attempting to use a powered-off VI,
	 * - VI emitting an event before the request is processed. */
	ret = wait_for_completion_killable_timeout(&mvi->ack, HZ);
	if (ret <= 0) {
		dev_err(dev, "no reply from camera processor\n");
		WARN_ON(1);
		ret = -EIO;
	}

	return ret;
}

static int mbox_vi_notify_probe(struct device *dev, struct vi_notify_dev *vnd)
{
	struct mbox_vi_notify *mvi = dev_get_drvdata(dev);

	mvi->vi_notify = vnd;
	mvi->client.dev = dev;
	mvi->client.tx_block = true;
	mvi->client.tx_tout = 500;
	mvi->client.knows_txdone = false;
	mvi->client.rx_callback = mbox_vi_notify_rx;
	mvi->client.tx_done = NULL;

	mvi->mbox = mbox_request_channel(&mvi->client, 0);
	if (IS_ERR(mvi->mbox))
		return PTR_ERR(mvi->mbox);

	return 0;
}

static void __exit mbox_vi_remove(struct device *dev)
{
	struct mbox_vi_notify *mvi = dev_get_drvdata(dev);

	mbox_free_channel(mvi->mbox);
}

static int mbox_vi_notify_classify(struct device *dev, u32 mask)
{
	struct mbox_vi_notify *mvi = dev_get_drvdata(dev);
	struct mbox_vi_notify_msg msg = {
		.type = TEGRA_IVC_VI_CLASSIFY,
		.mask = ~mask,
	};
	int err;

	if (mvi->tags == mask)
		return 0; /* nothing to do */

	err = mbox_vi_notify_send(dev, &msg);
	if (likely(err == 0))
		mvi->tags = mask;

	if (mvi->tags == 0 && mvi->channels == 0)
		nvhost_module_idle(mvi->vi);

	return err;
}

static int mbox_vi_notify_set_syncpts(struct device *dev, u8 ch,
					const u32 ids[3])
{
	struct mbox_vi_notify *mvi = dev_get_drvdata(dev);
	struct mbox_vi_notify_msg msg = {
		.type = TEGRA_IVC_VI_SET_SYNCPTS,
		.channel = ch,
	};
	int err;

	memcpy(msg.syncpt_ids, ids, sizeof(msg.syncpt_ids));
	mvi->channels |= 1u << ch;

	err = mbox_vi_notify_send(dev, &msg);
	if (mvi->tags == 0 && mvi->channels == 0)
		nvhost_module_idle(mvi->vi);

	return err;
}

static void mbox_vi_notify_reset_channel(struct device *dev, u8 ch)
{
	struct mbox_vi_notify *mvi = dev_get_drvdata(dev);
	struct mbox_vi_notify_msg msg = {
		.type = TEGRA_IVC_VI_RESET_CHANNEL,
		.channel = ch,
	};
	int err = mbox_vi_notify_send(dev, &msg);
	if (likely(err == 0))
		mvi->channels &= ~(1u << ch);

	if (mvi->tags == 0 && mvi->channels == 0)
		nvhost_module_idle(mvi->vi);
}

static struct vi_notify_driver mbox_vi_notify_driver = {
	.owner = THIS_MODULE,
	.probe = mbox_vi_notify_probe,
	.remove = __exit_p(mbox_vi_notify_remove),
	.classify = mbox_vi_notify_classify,
	.set_syncpts = mbox_vi_notify_set_syncpts,
	.reset_channel = mbox_vi_notify_reset_channel,
};

/* Platform device */
static struct platform_device *mbox_get_vi(struct platform_device *pdev)
{
	struct device_node *vi_node;
	struct platform_device *vi_pdev;

	vi_node = of_parse_phandle(pdev->dev.of_node, "device", 0);
	if (vi_node == NULL) {
		dev_err(&pdev->dev, "cannot get VI device");
		return ERR_PTR(-ENODEV);
	}

	vi_pdev = of_find_device_by_node(vi_node);
	of_node_put(vi_node);

	if (vi_pdev == NULL)
		return ERR_PTR(-EPROBE_DEFER);

	if (&vi_pdev->dev.driver == NULL) {
		platform_device_put(vi_pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return vi_pdev;
}

static int tegra_vi_mbox_probe(struct platform_device *pdev)
{
	struct mbox_vi_notify *mvi;
	int err;

	mvi = devm_kzalloc(&pdev->dev, sizeof(*mvi), GFP_KERNEL);
	if (unlikely(mvi == NULL))
		return -ENOMEM;

	mvi->tags = 0;
	mvi->channels = 0;
	init_completion(&mvi->ack);

	platform_set_drvdata(pdev, mvi);

	mvi->vi = mbox_get_vi(pdev);
	if (IS_ERR(mvi->vi))
		return PTR_ERR(mvi->vi);

	err = vi_notify_register(&mbox_vi_notify_driver, &pdev->dev, 12);
	if (err)
		platform_device_put(mvi->vi);
	return err;
}

static int __exit tegra_vi_mbox_remove(struct platform_device *pdev)
{
	struct mbox_vi_notify *mvi = platform_get_drvdata(pdev);

	vi_notify_unregister(&mbox_vi_notify_driver, &pdev->dev);
	platform_device_put(mvi->vi);
	return 0;
}

static struct of_device_id tegra_vi_mbox_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-vinotify" },
	{ },
};

static struct platform_driver tegra_vi_mbox_driver = {
	.probe = tegra_vi_mbox_probe,
	.remove = __exit_p(tegra_vi_mbox_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra-vi-mbox",
		.of_match_table = tegra_vi_mbox_of_match,
	},
};

module_platform_driver(tegra_vi_mbox_driver);
