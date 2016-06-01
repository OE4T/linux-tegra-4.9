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
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra_ast.h>
#include <linux/tegra-ivc-bus.h>

#define NV(p) "nvidia," #p

static int tegra_ivc_bus_start(struct device *);
static void tegra_ivc_bus_stop(struct device *);

static int tegra_ivc_bus_match(struct device *dev, struct device_driver *drv)
{
	struct tegra_ivc_driver *ivcdrv = to_tegra_ivc_driver(drv);

	if (dev->type != ivcdrv->dev_type)
		return 0;
	return of_driver_match_device(dev, drv);
}

static int tegra_ivc_bus_probe(struct device *dev)
{
	struct tegra_ivc_driver *drv = to_tegra_ivc_driver(dev->driver);
	int ret = -ENXIO;

	if (dev->type == &tegra_ivc_channel_type) {
		struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
		const struct tegra_ivc_channel_ops *ops = drv->ops.channel;

		BUG_ON(ops == NULL);
		if (ops->probe != NULL) {
			ret = ops->probe(chan);
			if (ret)
				return ret;
		}

		rcu_assign_pointer(chan->ops, ops);
		ret = 0;

	} else if (dev->type == &tegra_hsp_type) {
		const struct tegra_hsp_ops *ops = drv->ops.hsp;

		BUG_ON(ops == NULL || ops->probe == NULL);
		ret = ops->probe(dev);
		if (ret)
			return ret;

		ret = tegra_ivc_bus_start(dev);
		if (ret && ops->remove != NULL)
			ops->remove(dev);
	}

	return ret;
}

static int tegra_ivc_bus_remove(struct device *dev)
{
	struct tegra_ivc_driver *drv = to_tegra_ivc_driver(dev->driver);

	if (dev->type == &tegra_ivc_channel_type) {
		struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
		const struct tegra_ivc_channel_ops *ops = drv->ops.channel;

		WARN_ON(rcu_access_pointer(chan->ops) != ops);
		RCU_INIT_POINTER(chan->ops, NULL);
		synchronize_rcu();

		if (ops->remove != NULL)
			ops->remove(chan);

	} else if (dev->type == &tegra_hsp_type) {
		const struct tegra_hsp_ops *ops = drv->ops.hsp;

		tegra_ivc_bus_stop(dev);

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
	int hsp_master;
	int hsp_db;
	struct tegra_ivc_channel *chans;
};

static void tegra_hsp_ring(struct device *dev)
{
	const struct tegra_hsp_ops *ops = tegra_hsp_dev_ops(dev);

	BUG_ON(ops == NULL || ops->ring == NULL);
	ops->ring(dev);
}

struct device_type tegra_hsp_type = {
	.name = "tegra-hsp",
};
EXPORT_SYMBOL(tegra_hsp_type);

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

static void tegra_ivc_channel_ring(struct ivc *ivc)
{
	struct tegra_ivc_channel *chan =
		container_of(ivc, struct tegra_ivc_channel, ivc);
	struct tegra_ivc_bus *bus =
		container_of(chan->dev.parent, struct tegra_ivc_bus, dev);

	tegra_hsp_ring(&bus->dev);
}

static void tegra_ivc_channel_notify(struct tegra_ivc_channel *chan)
{
	const struct tegra_ivc_channel_ops *ops;

	rcu_read_lock();
	ops = rcu_dereference(chan->ops);

	if (ops != NULL && ops->notify != NULL)
		ops->notify(chan);
	rcu_read_unlock();
}

struct device_type tegra_ivc_channel_type = {
	.name = "tegra-ivc-channel",
};
EXPORT_SYMBOL(tegra_ivc_channel_type);

void tegra_hsp_notify(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);
	struct tegra_ivc_channel *chan;

	for (chan = bus->chans; chan != NULL; chan = chan->next)
		tegra_ivc_channel_notify(chan);
}
EXPORT_SYMBOL(tegra_hsp_notify);

static struct tegra_ivc_channel *tegra_ivc_bus_parse_channel(
		struct device_node *ch_node, struct device *parent,
		uintptr_t ivc_base, dma_addr_t ivc_dma, u32 ivc_size)
{
	int ret;
	u32 nframes, frame_size;
	union {
		u32 tab[2];
		struct {
			u32 rx;
			u32 tx;
		};
	} start, end;

	struct tegra_ivc_channel *chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL))
		return ERR_PTR(-ENOMEM);

	chan->dev.parent = parent;
	chan->dev.type = &tegra_ivc_channel_type;
	chan->dev.bus = &tegra_ivc_bus_type;
	chan->dev.of_node = of_node_get(ch_node);
	chan->dev.release = tegra_ivc_channel_release;
	dev_set_name(&chan->dev, "%s:%s", dev_name(parent),
			kbasename(ch_node->full_name));
	device_initialize(&chan->dev);

	ret = of_property_read_u32_array(ch_node, "reg", start.tab,
						ARRAY_SIZE(start.tab));
	if (ret) {
		dev_err(&chan->dev, "missing <%s> property\n", "reg");
		goto error;
	}

	ret = of_property_read_u32(ch_node, NV(frame-count), &nframes);
	if (ret) {
		dev_err(&chan->dev, "missing <%s> property\n",
			NV(frame-count));
		goto error;
	}

	ret = of_property_read_u32(ch_node, NV(frame-size), &frame_size);
	if (ret) {
		dev_err(&chan->dev, "missing <%s> property\n", NV(frame-size));
		goto error;
	}

	ret = -EINVAL;
	end.rx = start.rx + tegra_ivc_total_queue_size(nframes * frame_size);
	end.tx = start.tx + tegra_ivc_total_queue_size(nframes * frame_size);

	if (end.rx > ivc_size) {
		dev_err(&chan->dev, "%s buffer exceeds IVC size\n", "RX");
		goto error;
	}

	if (end.tx > ivc_size) {
		dev_err(&chan->dev, "%s buffer exceeds IVC size\n", "TX");
		goto error;
	}

	if (start.tx < start.rx ? end.tx > start.rx : end.rx > start.tx) {
		dev_err(&chan->dev, "RX and TX buffers overlap\n");
		goto error;
	}

	/* FIXME: revisit if dev->parent actually needed */
	/* Init IVC */
	ret = tegra_ivc_init_with_dma_handle(&chan->ivc,
			ivc_base + start.rx, (u64)ivc_dma + start.rx,
			ivc_base + start.tx, (u64)ivc_dma + start.tx,
			nframes, frame_size, parent->parent,
			tegra_ivc_channel_ring);
	if (ret) {
		dev_err(&chan->dev, "IVC initialization error: %d\n", ret);
		goto error;
	}

	dev_dbg(&chan->dev, "%s: RX: 0x%x-0x%x TX: 0x%x-0x%x\n",
		ch_node->name, start.rx, end.rx, start.tx, end.tx);
	return chan;
error:
	put_device(&chan->dev);
	return ERR_PTR(ret);
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
	struct tegra_ivc_channel *ch0;

	for (ch0 = bus->chans; ch0 != NULL; ch0 = ch0->next) {
		struct tegra_ivc_channel *ch1;

		for (ch1 = ch0->next; ch1 != NULL; ch1 = ch1->next) {
			int ret;

			ret = tegra_ivc_bus_check_overlap(&bus->dev, ch0, ch1);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void tegra_ivc_bus_destroy_channels(struct tegra_ivc_bus *bus)
{
	while (bus->chans != NULL) {
		struct tegra_ivc_channel *chan = bus->chans;

		bus->chans = chan->next;
		put_device(&chan->dev);
	}
}

static int tegra_ivc_bus_parse_channels(struct tegra_ivc_bus *bus,
					struct device_node *dev_node, u32 sid)
{
	struct device_node *reg_node;
	void __iomem *ast[2];
	int ret, region = 2;

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
			struct tegra_ivc_channel *chan;

			chan = tegra_ivc_bus_parse_channel(ch_node, &bus->dev,
						base, ivc_dma, ivc.size);
			if (IS_ERR(chan)) {
				ret = PTR_ERR(chan);
				of_node_put(ch_node);
				goto error;
			}

			chan->next = bus->chans;
			bus->chans = chan;
		}
	}

	return tegra_ivc_bus_validate_channels(bus);

error:
	of_node_put(reg_node);
	tegra_ivc_bus_destroy_channels(bus);
	return ret;
}

struct tegra_ivc_bus *tegra_ivc_bus_create(struct device *dev, u32 sid)
{
	int ret;

	struct tegra_ivc_bus *bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (unlikely(bus == NULL))
		return ERR_PTR(-ENOMEM);

	bus->dev.parent = dev;
	bus->dev.type = &tegra_hsp_type;
	bus->dev.bus = &tegra_ivc_bus_type;
	bus->dev.of_node = of_get_child_by_name(dev->of_node, "hsp");
	bus->dev.release = tegra_ivc_bus_release;
	dev_set_name(&bus->dev, "ivc-%s", dev_name(dev));

	device_initialize(&bus->dev);

	ret = tegra_ivc_bus_parse_channels(bus, dev->of_node, sid);
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

static int tegra_ivc_bus_start(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);
	struct tegra_ivc_channel *chan;

	for (chan = bus->chans; chan != NULL; chan = chan->next) {
		int ret = device_add(&chan->dev);
		if (ret) {
			struct tegra_ivc_channel *cd;

			dev_err(&chan->dev, "channel device error: %d\n", ret);

			for (cd = bus->chans; cd != chan; cd = cd->next)
				device_del(&cd->dev);
			return ret;
		}
	}

	return 0;
}

static void tegra_ivc_bus_stop(struct device *dev)
{
	struct tegra_ivc_bus *bus =
		container_of(dev, struct tegra_ivc_bus, dev);
	struct tegra_ivc_channel *chan;

	for (chan = bus->chans; chan != NULL; chan = chan->next)
		device_del(&chan->dev);
}

void tegra_ivc_bus_destroy(struct tegra_ivc_bus *bus)
{
	tegra_ivc_bus_destroy_channels(bus);
	device_unregister(&bus->dev);
}
EXPORT_SYMBOL(tegra_ivc_bus_destroy);

static __init int tegra_ivc_bus_init(void)
{
	return bus_register(&tegra_ivc_bus_type);
}

static __exit void tegra_ivc_bus_exit(void)
{
	bus_unregister(&tegra_ivc_bus_type);
}

subsys_initcall(tegra_ivc_bus_init);
module_exit(tegra_ivc_bus_exit);
MODULE_AUTHOR("Remi Denis-Courmont <remid@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra IVC generic bus driver");
MODULE_LICENSE("GPL");
