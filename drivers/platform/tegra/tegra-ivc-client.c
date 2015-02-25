/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/tegra-ivc-client.h>

#define IVCC_MIN_FRAME_SIZE		4

#define IVCC_CHAN_PROPERTY	"nvidia,ivcc-channels"

struct chan_sync {
	int ready;
	wait_queue_head_t wq;
	tegra_ivcc_notify notify;
	int (*func)(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags);
};

struct tegra_ivcc_chan_group {
	int hsp_id;		/* HSP master id */
	int hsp_db;		/* HSP doorbell */
	struct device *dev;
	struct list_head channels;
};

struct tegra_ivcc_chan {
	char *name;		/* name of the DT node */
	u32 nframes;
	u32 framesz;

	void *tx_base;	/* tx buffer */
	void *rx_base;	/* rx buffer */

	spinlock_t tx_lock;
	spinlock_t rx_lock;

	struct chan_sync tx_sync;
	struct chan_sync rx_sync;

	struct list_head list;
	struct tegra_ivcc_chan_group *group;	/* parent group */

	struct ivc ivc;
};

static int check_params(struct tegra_ivcc_chan *chan, void *frame, u32 size)
{
	if (!chan || !frame)
		return -EINVAL;
	if (size > chan->framesz) {
		dev_err(chan->group->dev, "buffer size exceeds frame size\n");
		return -E2BIG;
	}
	return 0;
}

static void ivc_rx_handler(struct tegra_ivcc_chan *chan)
{
	/* Unblock readers */
	chan->rx_sync.ready = 1;
	wake_up_all(&chan->rx_sync.wq);

	/* Notify passive readers */
	if (chan->rx_sync.notify)
		chan->rx_sync.notify(chan);
}

static void ivc_tx_handler(struct tegra_ivcc_chan *chan)
{
	/* Unblock senders */
	chan->tx_sync.ready = 1;
	wake_up_all(&chan->tx_sync.wq);

	/* Notify passive senders */
	if (chan->tx_sync.notify)
		chan->tx_sync.notify(chan);
}

static int ipc_send(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags)
{
	u32 bytes;
	ulong irqf;
	int ret;
	bool locked = !(flags & TEGRA_IVCC_FLAG_NOLOCKS);

	if (locked)
		spin_lock_irqsave(&chan->tx_lock, irqf);
	bytes = tegra_ivc_write(&chan->ivc, frame, size);
	ret = (bytes != size) ? -ENOMEM : 0;
	chan->tx_sync.ready = ret ? 0 : 1; /* ack waitq */
	if (locked)
		spin_unlock_irqrestore(&chan->tx_lock, irqf);

	return ret;
}

static int ipc_recv(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags)
{
	u32 bytes;
	ulong irqf;
	int ret;
	bool locked = !(flags & TEGRA_IVCC_FLAG_NOLOCKS);

	if (locked)
		spin_lock_irqsave(&chan->rx_lock, irqf);
	bytes = tegra_ivc_read(&chan->ivc, frame, size);
	ret = (bytes != size) ? -ENOMEM : 0;
	chan->rx_sync.ready = ret ? 0 : 1; /* ack waitq */
	if (locked)
		spin_unlock_irqrestore(&chan->rx_lock, irqf);

	return ret;
}

/*
 * This function performs both TX and RX depending on which sync
 * object is passed in. The logics for both are almost identical
 * except for the IVC methods and state variables, all of which
 * are encapsulated in chan_sync. This avoids code duplication.
 */
static int ipc_exec(struct tegra_ivcc_chan *chan, void *frame,
		u32 size, u32 flags, u32 timeout, struct chan_sync *sync)
{
	int forever;
	ktime_t kt_start;		/* polling wait */
	long to_jiffies = 0;	/* blocking wait */
	struct device *dev;
	int ret;

	/* Fast path */
	ret = sync->func(chan, frame, size, flags);
	if (!ret || !timeout)
		return ret;

	/* Slow path for unsuccessful transaction */
	dev = chan->group->dev;
	forever = timeout == TEGRA_IVCC_NOTIMEOUT;
	if (!forever) {
		kt_start = ktime_get();
		to_jiffies = usecs_to_jiffies(timeout);
	}

	/* This loop retries the transaction until success or timeout
	 *  - atomic: check elaspsed ktime and break out right away
	 *  - blocking: use wait-queue to sleep and exit in footer
	 */
	do {
		if (flags & TEGRA_IVCC_FLAG_POLL) {
			/* Polling wait */
			if (!forever && ktime_us_delta(
				ktime_get(), kt_start) >= timeout)
				break;
		} else if (!forever) {
			/* Blocking wait with timeout */
			to_jiffies = wait_event_timeout(sync->wq,
				sync->ready, to_jiffies);
			if (!to_jiffies)
				break;
		} else {
			/* Blocking wait indefinitely */
			wait_event(sync->wq, sync->ready);
		}

		/* Retry */
		ret = sync->func(chan, frame, size, flags);
	} while (ret && (forever || to_jiffies));

	return ret;
}

/**
 * tegra_ivcc_send - Send data to remote
 *
 * @chan: channel handle
 * @frame: pointer to the frame
 * @size: size of the frame
 * @flags: send options
 *
 * This function will wait-and-retry repeatedly
 * until the operation is completed.
 *
 * Returns 0 for success.
 */
int tegra_ivcc_send(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags)
{
	if (check_params(chan, frame, size))
		return -EINVAL;
	return ipc_exec(chan, frame, size, flags,
		TEGRA_IVCC_NOTIMEOUT, &chan->tx_sync);
}
EXPORT_SYMBOL(tegra_ivcc_send);

/**
 * tegra_ivcc_send_timeout - Send data to remote w/ timeout
 * @chan: channel handle
 * @frame: pointer to the frame
 * @size: size of the frame
 * @flags: send options
 * @timeout: timeout in usecs
 *
 * Returns 0 for success.
 */
int tegra_ivcc_send_timeout(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags, u32 timeout)
{
	if (check_params(chan, frame, size))
		return -EINVAL;
	return ipc_exec(chan, frame, size, flags, timeout, &chan->tx_sync);
}
EXPORT_SYMBOL(tegra_ivcc_send_timeout);

/**
 * tegra_ivcc_recv - Receive data from remote
 *
 * @chan: channel handle
 * @frame: pointer to the frame
 * @size: size of the frame
 * @flags: send options
 *
 * This function will wait-and-retry repeatedly
 * until the operation is completed.
 *
 * Returns 0 for success.
 */
int tegra_ivcc_recv(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags)
{
	if (check_params(chan, frame, size))
		return -EINVAL;
	return ipc_exec(chan, frame, size, flags,
		TEGRA_IVCC_NOTIMEOUT, &chan->rx_sync);
}
EXPORT_SYMBOL(tegra_ivcc_recv);

/**
 * tegra_ivcc_recv_timeout - Receive data from remote w/ timeout
 * @chan: channel handle
 * @frame: pointer to the frame
 * @size: size of the frame
 * @flags: send options
 * @timeout: timeout in usecs
 *
 * Returns 0 for success.
 */
int tegra_ivcc_recv_timeout(struct tegra_ivcc_chan *chan,
		void *frame, u32 size, u32 flags, u32 timeout)
{
	if (check_params(chan, frame, size))
		return -EINVAL;
	return ipc_exec(chan, frame, size, flags, timeout, &chan->rx_sync);
}
EXPORT_SYMBOL(tegra_ivcc_recv_timeout);

/**
 * tegra_ivcc_set_send_notify - provide callback when TX is non-full
 * @chan: channel handle
 * @func: pointer to the callback
 */
void tegra_ivcc_set_send_notify(struct tegra_ivcc_chan *chan,
		tegra_ivcc_notify func)
{
	chan->tx_sync.notify = func;
}
EXPORT_SYMBOL(tegra_ivcc_set_send_notify);

/**
 * tegra_ivcc_set_recv_notify - provide callback when RX is non-empty
 * @chan: channel handle
 * @func: pointer to the callback
 */
void tegra_ivcc_set_recv_notify(struct tegra_ivcc_chan *chan,
		tegra_ivcc_notify func)
{
	chan->rx_sync.notify = func;
}
EXPORT_SYMBOL(tegra_ivcc_set_recv_notify);

static int check_parent_node(struct device_node *dn);

/**
 * tegra_ivcc_find_queue_by_phandle - lookup channel by DT phandle
 * @np: node pointer of the client device
 * @index: index of the channels property
 *
 * Returns channel handle if success, NULL if failed.
 */
struct tegra_ivcc_chan *tegra_ivcc_find_queue_by_phandle(
		struct device_node *np, int index)
{
	struct device_node *qdn;

	if (!np)
		return NULL;
	qdn = of_parse_phandle(np, IVCC_CHAN_PROPERTY, index);
	if (!qdn || check_parent_node(qdn))
		return NULL;
	return (struct tegra_ivcc_chan *)qdn->data;
}
EXPORT_SYMBOL(tegra_ivcc_find_queue_by_phandle);

/**
 * tegra_ivcc_find_queue_by_name - lookup channel by channel name
 * @name: channel name
 *
 * The channel name is the name of the DT node of the channel.
 *
 * Returns channel handle if success, NULL if failed.
 */
struct tegra_ivcc_chan *tegra_ivcc_find_queue_by_name(const char *name)
{
	struct device_node *p;

	if (!name)
		return NULL;
	p = of_find_node_by_name(NULL, name);
	if (!p || check_parent_node(p))
		return NULL;
	return (struct tegra_ivcc_chan *)p->data;
}
EXPORT_SYMBOL(tegra_ivcc_find_queue_by_name);

static irqreturn_t ivc_notification(struct tegra_ivcc_chan *chan)
{
	/* There are new frames in the RX channel */
	if (likely(tegra_ivc_can_read(&chan->ivc)))
		ivc_rx_handler(chan);

	/* There are vacant spaces in the TX channel */
	if (likely(tegra_ivc_can_write(&chan->ivc)))
		ivc_tx_handler(chan);

	return IRQ_HANDLED;
}

/* Notify remote by ringing its HSP doorbell */
static void hsp_raise_irq(struct ivc *ivc)
{
	struct tegra_ivcc_chan *chan;
	chan = container_of(ivc, struct tegra_ivcc_chan, ivc);
	tegra_hsp_db_ring(chan->group->hsp_db);
}

/* Handler for CCPLEX's doorbell */
static void hsp_irq_handler(int master, void *data)
{
	struct tegra_ivcc_chan_group *group;
	struct tegra_ivcc_chan *chan;
	group = (struct tegra_ivcc_chan_group *)data;
	list_for_each_entry(chan, &group->channels, list)
		ivc_notification(chan);
}

static int setup_doorbell(struct tegra_ivcc_chan_group *group)
{
	/* Listen to the remote's notification */
	if (tegra_hsp_db_add_handler(group->hsp_id, hsp_irq_handler, group))
		return -EINVAL;
	/* Allow remote to ring CCPLEX's doorbell */
	if (tegra_hsp_db_enable_master(group->hsp_id))
		return -EINVAL;
	return 0;
}

static const struct of_device_id tegra_ivcc_of_match[] = {
	{ .compatible = "nvidia,tegra186-ivcc", },
	{},
};

/* Ensure parent node is IVCC compatible */
static int check_parent_node(struct device_node *dn)
{
	struct device_node *pp = of_get_parent(dn);
	const char *compat = tegra_ivcc_of_match[0].compatible;
	return !pp || !of_device_is_compatible(pp, compat);
}

#define NV(p) "nvidia," p

static int
parse_channel(struct tegra_ivcc_chan_group *group, struct device_node *qdn)
{
	const __be32 *tx_reg, *rx_reg;
	u64 tx_size, rx_size;
	struct tegra_ivcc_chan *chan;

	chan = devm_kzalloc(group->dev, sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		dev_err(group->dev, "out of memory.\n");
		return -ENOMEM;
	}

	chan->name = kstrdup(qdn->name, GFP_KERNEL);
	if (!chan->name)
		goto free_chan;

	tx_reg = of_get_address(qdn, 0, &tx_size, NULL);
	rx_reg = of_get_address(qdn, 1, &rx_size, NULL);

	/* 1-1 mapping has been set up via /memreserve/. */
	chan->tx_base = phys_to_virt(of_translate_address(qdn, tx_reg));
	chan->rx_base = phys_to_virt(of_translate_address(qdn, rx_reg));

	if (!chan->tx_base || !chan->rx_base || (tx_size != rx_size)) {
		dev_err(group->dev, "invalid <reg> property.\n");
		goto free;
	}

	if (of_property_read_u32(qdn, NV("frame-size"), &chan->framesz)) {
		dev_err(group->dev, "missing <frame-sizee> property.\n");
		goto free;
	}

	if (chan->framesz < IVCC_MIN_FRAME_SIZE ||
		chan->framesz > tx_size) {
		dev_err(group->dev, "frame-size exceeds range (%d-%lld).\n",
			IVCC_MIN_FRAME_SIZE, tx_size);
		goto free;
	}

	chan->nframes = tx_size / chan->framesz;
	if (!chan->nframes) {
		dev_err(group->dev, "invalid <frame-size> property.\n");
		goto free;
	}

	memset(chan->tx_base, 0, tx_size);
	memset(chan->rx_base, 0, rx_size);

	spin_lock_init(&chan->tx_lock);
	spin_lock_init(&chan->rx_lock);

	init_waitqueue_head(&chan->tx_sync.wq);
	init_waitqueue_head(&chan->rx_sync.wq);

	chan->tx_sync.func = ipc_send;
	chan->rx_sync.func = ipc_recv;

	/* Allocate the IVC links */
	if (tegra_ivc_init(&chan->ivc,
			(uintptr_t)chan->rx_base,
			(uintptr_t)chan->tx_base,
			chan->nframes, chan->framesz,
			group->dev, hsp_raise_irq)) {
		dev_err(group->dev, "failed to instantiate IVC.\n");
		goto free;
	}

	chan->group = group;
	list_add(&chan->list, &group->channels);

	dev_dbg(group->dev, "%s: TX: 0x%lx-0x%lx\n", chan->name,
		(ulong)chan->tx_base, (ulong)chan->tx_base + (u32)tx_size);
	dev_dbg(group->dev, "%s: RX: 0x%lx-0x%lx\n", chan->name,
		(ulong)chan->rx_base, (ulong)chan->rx_base + (u32)rx_size);

	/* Cache chan in DT node for find_queue_by_phandle */
	qdn->data = chan;

	return 0;

free:
	kfree(chan->name);

free_chan:
	devm_kfree(group->dev, chan);
	return -EINVAL;
}

static int tegra_ivcc_of_init(struct device *dev)
{
	u32 hsp_data[3];
	struct device_node *qdn;
	struct device_node *dn = dev->of_node;
	struct tegra_ivcc_chan_group *group;

	group = devm_kzalloc(dev, sizeof(*group), GFP_KERNEL);
	if (!group) {
		dev_err(dev, "out of memory.\n");
		return -ENOMEM;
	}

	group->dev = dev;
	dev->platform_data = group;

	if (of_property_read_u32_array(dn, NV("notifications"), hsp_data, 3)) {
		dev_err(dev, "missing <notification> property\n");
		goto free;
	}

	/* [0] is the HSP phandle */
	group->hsp_id = hsp_data[1];
	group->hsp_db = hsp_data[2];

	if (setup_doorbell(group))
		goto free;

	INIT_LIST_HEAD(&group->channels);

	/* Parse out all channels from DT */
	for_each_child_of_node(dn, qdn) {
		if (parse_channel(group, qdn)) {
			dev_err(dev, "failed to parse a channel\n");
			goto free;
		}
	}

	return 0;

free:
	devm_kfree(dev, group);
	return -EINVAL;
}

static int tegra_ivcc_probe(struct platform_device *pdev)
{
	return tegra_ivcc_of_init(&pdev->dev);
}

static struct platform_driver tegra_ivcc_driver = {
	.probe	= tegra_ivcc_probe,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_ivcc",
		.of_match_table = of_match_ptr(tegra_ivcc_of_match),
	},
};

static int __init tegra_ivcc_init(void)
{
	return platform_driver_register(&tegra_ivcc_driver);
}
subsys_initcall(tegra_ivcc_init);
