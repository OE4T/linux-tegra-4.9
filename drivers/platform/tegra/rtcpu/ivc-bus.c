/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/mailbox_controller.h>
#include <linux/tegra_ast.h>
#include <linux/tegra-ivc-bus.h>

#define NV(p) "nvidia," #p

static int tegra_ivc_bus_match(struct device *dev, struct device_driver *drv)
{
	struct tegra_ivc_driver *ivcdrv = to_tegra_ivc_driver(drv);

	return ivcdrv->dev_type == dev->type;
}

static int tegra_ivc_bus_probe(struct device *dev)
{
	int ret = -ENXIO;

	if (dev->type == &tegra_hsp_type) {
		const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

		BUG_ON(ops == NULL || ops->probe == NULL);
		ret = ops->probe(dev);
	}

	return ret;
}

static int tegra_ivc_bus_remove(struct device *dev)
{
	if (dev->type == &tegra_hsp_type) {
		const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

		if (ops->remove != NULL)
			ops->remove(dev);
	}

	return 0;
}

struct bus_type tegra_ivc_bus_type = {
	.name	= "tegra-ivc",
	.match	= tegra_ivc_bus_match,
	.probe	= tegra_ivc_bus_probe,
	.remove	= tegra_ivc_bus_remove,
};
EXPORT_SYMBOL(tegra_ivc_bus_type);

int tegra_ivc_driver_register(struct tegra_ivc_driver *drv)
{
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL(tegra_ivc_driver_register);

void tegra_ivc_driver_unregister(struct tegra_ivc_driver *drv)
{
	return driver_unregister(&drv->driver);
}
EXPORT_SYMBOL(tegra_ivc_driver_unregister);

struct tegra_ivc_bus {
	struct device dev;
	struct mbox_controller mbox;
	int hsp_master;
	int hsp_db;
	struct work_struct rx_work;
	struct mbox_chan chans[];
};

static void tegra_hsp_ring(struct device *dev)
{
	const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

	BUG_ON(ops == NULL || ops->ring == NULL);
	ops->ring(dev);
}

static int tegra_hsp_enable(struct device *dev)
{
	const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

	if (ops == NULL || ops->enable == NULL)
		return -ENXIO;
	return ops->enable(dev);
}

static void tegra_hsp_disable(struct device *dev)
{
	const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

	if (ops != NULL && ops->disable != NULL)
		ops->disable(dev);
}

struct device_type tegra_hsp_type = {
	.name = "tegra-hsp",
};
EXPORT_SYMBOL(tegra_hsp_type);

void tegra_hsp_notify(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);

	schedule_work(&bus->rx_work);
}
EXPORT_SYMBOL(tegra_hsp_notify);

struct tegra_ivc_channel {
	struct device dev;
	struct ivc ivc;
	bool last_tx_done;
};

static void tegra_ivc_channel_release(struct device *dev)
{
	struct tegra_ivc_channel *chan =
		container_of(dev, struct tegra_ivc_channel, dev);

	of_node_put(dev->of_node);
	kfree(chan);
}

static void tegra_ivc_bus_release(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);

	of_node_put(dev->of_node);
	kfree(bus);
}

static void tegra_ivc_channel_tx_notify(struct ivc *ivc)
{
	struct tegra_ivc_channel *chan =
		container_of(ivc, struct tegra_ivc_channel, ivc);
	struct tegra_ivc_bus *bus =
		container_of(chan->dev.parent, struct tegra_ivc_bus, dev);

	tegra_hsp_ring(&bus->dev);
}

static int tegra_ivc_mbox_send_data(struct mbox_chan *mbox_chan, void *data)
{
	struct tegra_ivc_channel *chan = mbox_chan->con_priv;
	struct tegra_ivc_mbox_msg *msg = data;
	int ret = 0;

	if (tegra_ivc_write(&chan->ivc, msg->data, msg->length) != msg->length)
		ret = -EBUSY;

	chan->last_tx_done = ret == 0;
	return ret;
}

static int tegra_ivc_mbox_startup(struct mbox_chan *mbox_chan)
{
	return 0;
}

static void tegra_ivc_mbox_shutdown(struct mbox_chan *mbox_chan)
{
}

static bool tegra_ivc_mbox_last_tx_done(struct mbox_chan *mbox_chan)
{
	struct tegra_ivc_channel *chan = mbox_chan->con_priv;

	return chan->last_tx_done;
}

static struct mbox_chan_ops tegra_ivc_mbox_chan_ops = {
	.send_data = tegra_ivc_mbox_send_data,
	.startup = tegra_ivc_mbox_startup,
	.shutdown = tegra_ivc_mbox_shutdown,
	.last_tx_done = tegra_ivc_mbox_last_tx_done,
};

static void tegra_ivc_bus_rx_worker(struct work_struct *work)
{
	struct tegra_ivc_bus *bus =
			container_of(work, struct tegra_ivc_bus, rx_work);
	int i;

	for (i = 0; i < bus->mbox.num_chans; i++) {
		struct mbox_chan *mbox_chan = &bus->mbox.chans[i];
		struct tegra_ivc_channel *chan = mbox_chan->con_priv;
		struct ivc *ivc = &chan->ivc;

		while (tegra_ivc_can_read(ivc)) {
			struct tegra_ivc_mbox_msg msg;

			msg.data = tegra_ivc_read_get_next_frame(ivc);
			msg.length = ivc->frame_size;
			mbox_chan_received_data(mbox_chan, &msg);
			tegra_ivc_read_advance(ivc);
		}
	}
}

static int tegra_ivc_bus_parse_channel(struct device *dev, int idx,
		uintptr_t ivc_base, dma_addr_t ivc_dma, u32 ivc_size,
		struct mbox_chan *mbox_chan, struct device_node *ch_node)
{
	struct tegra_ivc_channel *chan;
	int ret;
	u32 nframes, frame_size;
	union {
		u32 tab[2];
		struct {
			u32 rx;
			u32 tx;
		};
	} start, end;

	ret = of_property_read_u32_array(ch_node, "reg", start.tab,
						ARRAY_SIZE(start.tab));
	if (ret) {
		dev_err(dev, "missing <%s> property\n", "reg");
		return ret;
	}

	ret = of_property_read_u32(ch_node, NV(frame-count), &nframes);
	if (ret) {
		dev_err(dev, "missing <%s> property\n", NV(frame-count));
		return ret;
	}

	ret = of_property_read_u32(ch_node, NV(frame-size), &frame_size);
	if (ret) {
		dev_err(dev, "missing <%s> property\n", NV(frame-size));
		return ret;
	}

	end.rx = start.rx + tegra_ivc_total_queue_size(nframes * frame_size);
	end.tx = start.tx + tegra_ivc_total_queue_size(nframes * frame_size);

	if (end.rx > ivc_size) {
		dev_err(dev, "%s buffer exceeds IVC size\n", "RX");
		return -ENOMEM;
	}

	if (end.tx > ivc_size) {
		dev_err(dev, "%s buffer exceeds IVC size\n", "TX");
		return -ENOMEM;
	}

	if (start.tx < start.rx ? end.tx > start.rx : end.rx > start.tx) {
		dev_err(dev, "RX and TX buffers overlap on channel %s\n",
			ch_node->name);
		return -ENOMEM;
	}

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL))
		return -ENOMEM;

	chan->dev.parent = dev;
	chan->dev.bus = &tegra_ivc_bus_type;
	chan->dev.of_node = of_node_get(ch_node);
	chan->dev.release = tegra_ivc_channel_release;
	dev_set_name(&chan->dev, "%s:%d", dev_name(dev), idx);
	device_initialize(&chan->dev);

	mbox_chan->con_priv = chan;

	/* FIXME: revisit if dev->parent actually needed */
	/* Init IVC */
	ret = tegra_ivc_init_with_dma_handle(&chan->ivc,
		ivc_base + start.rx, (u64)ivc_dma + start.rx,
		ivc_base + start.tx, (u64)ivc_dma + start.tx,
		nframes, frame_size, dev->parent, tegra_ivc_channel_tx_notify);
	if (ret) {
		dev_err(&chan->dev, "IVC initialization error: %d\n", ret);
		put_device(&chan->dev);
		return ret;
	}

	dev_dbg(&chan->dev, "%s: RX: 0x%x-0x%x TX: 0x%x-0x%x\n",
		ch_node->name, start.rx, end.rx, start.tx, end.tx);
	return 0;
}

static int tegra_ivc_bus_check_overlap(struct device *dev,
					const struct tegra_ivc_channel *ch0,
					const struct tegra_ivc_channel *ch1)
{
	unsigned s0, s1;
	uintptr_t tx0, rx0, tx1, rx1;

	if (ch0 == NULL || ch1 == NULL)
		return -EINVAL;

	tx0 = (uintptr_t)ch0->ivc.tx_channel;
	rx0 = (uintptr_t)ch0->ivc.rx_channel;
	s0 = ch0->ivc.nframes * ch0->ivc.frame_size;
	s0 = tegra_ivc_total_queue_size(s0);

	tx1 = (uintptr_t)ch1->ivc.tx_channel;
	rx1 = (uintptr_t)ch1->ivc.rx_channel;
	s1 = ch1->ivc.nframes * ch1->ivc.frame_size;
	s1 = tegra_ivc_total_queue_size(s1);

	if ((tx0 < tx1 ? tx0 + s0 > tx1 : tx1 + s1 > tx0) ||
		(rx0 < tx1 ? rx0 + s0 > tx1 : tx1 + s1 > rx0) ||
		(rx0 < rx1 ? rx0 + s0 > rx1 : rx1 + s1 > rx0) ||
		(tx0 < rx1 ? tx0 + s0 > rx1 : rx1 + s1 > tx0)) {
		dev_err(dev, "buffers overlap on channels %s and %s\n",
			dev_name(&ch0->dev), dev_name(&ch1->dev));
		return -EINVAL;
	}

	return 0;
}

static int tegra_ivc_bus_validate_channels(struct tegra_ivc_bus *bus)
{
	int i, j, ret;

	for (i = 0; i < bus->mbox.num_chans; i++) {
		struct tegra_ivc_channel *ch0 = bus->mbox.chans[i].con_priv;

		for (j = i + 1; j < bus->mbox.num_chans; j++) {
			struct tegra_ivc_channel *ch1 =
						bus->mbox.chans[j].con_priv;

			ret = tegra_ivc_bus_check_overlap(&bus->dev, ch0, ch1);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int tegra_ivc_bus_parse_channels(struct tegra_ivc_bus *bus, u32 sid)
{
	struct device_node *dev_node = bus->dev.of_node, *reg_node;
	void __iomem *ast[2];
	int ret, channel = 0, region = 2;

	/* Get AST handles */
	ret = tegra_ast_map(bus->dev.parent, NV(ast), 2, ast);
	if (ret) {
		dev_err(&bus->dev, "ASTs not found: %d\n", ret);
		return ret;
	}

	/* Parse out all nodes with a region */
	for_each_child_of_node(dev_node, reg_node) {
		struct device_node *ch_node;
		unsigned long base;
		dma_addr_t ivc_dma;
		struct {
			u32 va, size;
		} ivc = { 0, 0 };

		if (of_node_cmp(reg_node->name, "ivc-channels"))
			continue;

		/* IVC VA addr and size */
		if (of_property_read_u32_array(reg_node, "reg", &ivc.va, 2)) {
			dev_err(&bus->dev, "missing <%s> property\n", "reg");
			ret = -EINVAL;
			goto error;
		}

		/* IVC buffer size must be a power of 2 */
		if (unlikely(ivc.size & (ivc.size - 1))) {
			dev_err(&bus->dev, "invalid region size 0x%08X\n",
				ivc.size);
			ret = -EINVAL;
			goto error;
		}

		/* Allocate RAM for IVC */
		base = (unsigned long)dmam_alloc_coherent(bus->dev.parent,
				ivc.size, &ivc_dma, GFP_KERNEL | __GFP_ZERO);
		if (unlikely(base == 0)) {
			ret = -ENOMEM;
			goto error;
		}

		tegra_ast_region_enable(2, ast, region, ivc.va, ivc.size,
					ivc_dma, sid);
		region++;

		for_each_child_of_node(reg_node, ch_node) {
			ret = tegra_ivc_bus_parse_channel(&bus->dev, channel,
					base, ivc_dma, ivc.size,
					&bus->mbox.chans[channel], ch_node);
			if (ret)
				goto error;

			channel++;
		}
	}

	return tegra_ivc_bus_validate_channels(bus);

error:
	while (channel > 0) {
		struct mbox_chan *mbox_chan = &bus->mbox.chans[--channel];
		struct tegra_ivc_channel *chan = mbox_chan->con_priv;

		put_device(&chan->dev);
	}
	return ret;
}

static void tegra_ivc_bus_destroy_channels(struct tegra_ivc_bus *bus)
{
	int i;

	for (i = 0; i < bus->mbox.num_chans; i++) {
		struct mbox_chan *mbox_chan = &bus->mbox.chans[i];
		struct tegra_ivc_channel *chan = mbox_chan->con_priv;

		put_device(&chan->dev);
	}
}

static unsigned tegra_ivc_bus_count_channels(struct device_node *dev_node)
{
	struct device_node *bus_node;
	unsigned num = 0;

	for_each_child_of_node(dev_node, bus_node)
		if (of_node_cmp(bus_node->name, "ivc-channels") == 0) {
			struct device_node *ch_node;

			for_each_child_of_node(bus_node, ch_node)
				num++;
		}

	return num;
}

struct tegra_ivc_bus *tegra_ivc_bus_create(struct device *dev, u32 sid)
{
	struct tegra_ivc_bus *bus;
	unsigned count;
	int ret;

	count = tegra_ivc_bus_count_channels(dev->of_node);
	if (count == 0) {
		dev_err(dev, "no IVC channels\n");
		return ERR_PTR(-EINVAL);
	}

	bus = kzalloc(sizeof(*bus) + count * sizeof(bus->chans[0]),
			GFP_KERNEL);
	if (unlikely(bus == NULL))
		return ERR_PTR(-ENOMEM);

	bus->dev.parent = dev;
	bus->dev.type = &tegra_hsp_type;
	bus->dev.bus = &tegra_ivc_bus_type;
	bus->dev.of_node = of_node_get(dev->of_node);
	bus->dev.release = tegra_ivc_bus_release;
	dev_set_name(&bus->dev, "ivc-%s", dev_name(dev));

	INIT_WORK(&bus->rx_work, tegra_ivc_bus_rx_worker);

	bus->mbox.dev = dev; /* must be platform_device */
	bus->mbox.chans = bus->chans;
	bus->mbox.num_chans = count;
	bus->mbox.ops = &tegra_ivc_mbox_chan_ops;
	bus->mbox.txdone_poll = true;
	bus->mbox.txpoll_period = 1;

	device_initialize(&bus->dev);

	ret = tegra_ivc_bus_parse_channels(bus, sid);
	if (ret) {
		dev_err(&bus->dev, "IVC channels setup failed: %d\n", ret);
		goto error;
	}

	ret = device_add(&bus->dev);
	if (ret) {
		dev_err(&bus->dev, "IVC instance error: %d\n", ret);
		tegra_ivc_bus_destroy_channels(bus);
		goto error;
	}

	return bus;

error:
	put_device(&bus->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(tegra_ivc_bus_create);

static int tegra_ivc_bus_start(struct tegra_ivc_bus *bus)
{
	int ret, i;

	for (i = 0; i < bus->mbox.num_chans; i++) {
		struct mbox_chan *mbox_chan = &bus->mbox.chans[i];
		struct tegra_ivc_channel *chan = mbox_chan->con_priv;

		ret = device_add(&chan->dev);
		if (ret) {
			dev_err(&bus->dev, "channel device error: %d\n", ret);
			goto error;
		}
	}

	ret = mbox_controller_register(&bus->mbox);
	if (ret) {
		dev_err(&bus->dev, "mailbox controller error: %d\n", ret);
		goto error;
	}

	/* Listen to the remote's notification */
	ret = tegra_hsp_enable(&bus->dev);
	if (ret) {
		dev_err(&bus->dev, "HSP doorbell master error: %d\n", ret);
		flush_scheduled_work();
		goto error_mbox;
	}
	return 0;

error_mbox:
	mbox_controller_unregister(&bus->mbox);
error:
	while (i > 0) {
		struct mbox_chan *mbox_chan = &bus->mbox.chans[--i];
		struct tegra_ivc_channel *chan = mbox_chan->con_priv;

		device_del(&chan->dev);
	}
	return ret;
}

static void tegra_ivc_bus_stop(struct tegra_ivc_bus *bus)
{
	int i;

	tegra_hsp_disable(&bus->dev);
	flush_scheduled_work();
	mbox_controller_unregister(&bus->mbox);

	for (i = 0; i < bus->mbox.num_chans; i++) {
		struct mbox_chan *mbox_chan = &bus->mbox.chans[i];
		struct tegra_ivc_channel *chan = mbox_chan->con_priv;

		device_del(&chan->dev);
	}
}

void tegra_ivc_bus_destroy(struct tegra_ivc_bus *bus)
{
	tegra_ivc_bus_destroy_channels(bus);
	device_unregister(&bus->dev);
}
EXPORT_SYMBOL(tegra_ivc_bus_destroy);

static int tegra_ivc_bus_notify(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_BOUND_DRIVER:
		if (dev->type == &tegra_hsp_type) {
			struct tegra_ivc_bus *bus =
				container_of(dev, struct tegra_ivc_bus, dev);
			int ret;

			ret = tegra_ivc_bus_start(bus);
			WARN_ON(ret);
		}
		break;

	case BUS_NOTIFY_UNBIND_DRIVER:
		if (dev->type == &tegra_hsp_type) {
			struct tegra_ivc_bus *bus =
				container_of(dev, struct tegra_ivc_bus, dev);

			tegra_ivc_bus_stop(bus);
		}
		break;
	}

	return 0;
}

static struct notifier_block tegra_ivc_bus_nb = {
	.notifier_call = tegra_ivc_bus_notify,
};

static __init int tegra_ivc_bus_init(void)
{
	int ret;

	ret = bus_register(&tegra_ivc_bus_type);
	if (ret)
		return ret;

	ret = bus_register_notifier(&tegra_ivc_bus_type, &tegra_ivc_bus_nb);
	if (ret)
		bus_unregister(&tegra_ivc_bus_type);
	return ret;
}

static __exit void tegra_ivc_bus_exit(void)
{
	bus_unregister_notifier(&tegra_ivc_bus_type, &tegra_ivc_bus_nb);
	bus_unregister(&tegra_ivc_bus_type);
}

subsys_initcall(tegra_ivc_bus_init);
module_exit(tegra_ivc_bus_exit);
MODULE_DESCRIPTION("NVIDIA Tegra IVC generic bus driver");
MODULE_LICENSE("GPL");
