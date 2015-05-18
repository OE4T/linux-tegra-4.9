/*
 * drivers/video/tegra/host/host1x/host1x.h
 *
 * Tegra Graphics Host Driver Entrypoint
 *
 * Copyright (c) 2010-2015, NVIDIA Corporation. All rights reserved.
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

#ifndef __NVHOST_HOST1X_H
#define __NVHOST_HOST1X_H

#include <linux/cdev.h>
#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>

#include "nvhost_syncpt.h"
#include "nvhost_channel.h"
#include "nvhost_intr.h"

#define TRACE_MAX_LENGTH	128U
#define IFACE_NAME		"nvhost"

struct nvhost_chip_support;
struct nvhost_channel;
struct mem_mgr;

extern long linsim_cl;

/*
 * Policy determines how do we store the syncpts,
 * i.e. either per channel (in struct nvhost_channel)
 * or per channel instance (in struct nvhost_channel_userctx)
 */
enum nvhost_syncpt_policy {
	SYNCPT_PER_CHANNEL = 0,
	SYNCPT_PER_CHANNEL_INSTANCE,
};

/*
 * Policy determines when to map HW channel to device,
 * i.e. either on channel device node open time
 * or on work submission time
 */
enum nvhost_channel_policy {
	MAP_CHANNEL_ON_OPEN = 0,
	MAP_CHANNEL_ON_SUBMIT,
};

struct host1x_device_info {
	/* Channel info */
	int		nb_channels;	/* host1x: num channels supported */
	int		ch_base;	/* host1x: channel base */
	int		ch_limit;	/* host1x: channel limit */
	enum nvhost_channel_policy channel_policy; /* host1x: channel policy */

	/* Syncpoint info */
	int		nb_hw_pts;	/* host1x: num syncpoints supported
					   in h/w */
	int		nb_pts;		/* host1x: num syncpoints supported
					   in s/w where nb_pts <= nb_hw_pts */
	int		pts_base;	/* host1x: syncpoint base */
	int		pts_limit;	/* host1x: syncpoint limit */
	enum nvhost_syncpt_policy syncpt_policy; /* host1x: syncpoint policy */
	int		nb_mlocks;	/* host1x: number of mlocks */
	int		(*initialize_chip_support)(struct nvhost_master *,
						struct nvhost_chip_support *);
	bool		allow_user_mappings; /* allow userspace mappings? */
};

struct nvhost_master {
	void __iomem *aperture;
	void __iomem *sync_aperture;
	struct class *nvhost_class;
	struct cdev cdev;
	struct device *ctrl;
	struct nvhost_syncpt syncpt;
	struct nvhost_intr intr;
	struct platform_device *dev;
	atomic_t clientid;
	struct host1x_device_info info;
	struct nvhost_characteristics nvhost_char;
	struct kobject *caps_kobj;
	struct nvhost_capability_node *caps_nodes;
	struct mutex timeout_mutex;

	struct nvhost_channel **chlist;	/* channel list */
	struct mutex chlist_mutex;	/* mutex for channel list */
	unsigned long allocated_channels[2];
	struct mutex priority_lock;	/* mutex for priority update */

	/* nvhost vm specific structures */
	struct list_head static_mappings_list;
	struct list_head vm_list;
	struct mutex vm_mutex;
};

extern struct nvhost_master *nvhost;

void nvhost_debug_init(struct nvhost_master *master);
void nvhost_device_debug_init(struct platform_device *dev);
void nvhost_device_debug_deinit(struct platform_device *dev);
void nvhost_debug_dump(struct nvhost_master *master);

int nvhost_host1x_finalize_poweron(struct platform_device *dev);
int nvhost_host1x_prepare_poweroff(struct platform_device *dev);

void nvhost_set_chanops(struct nvhost_channel *ch);

int nvhost_gather_filter_enabled(struct nvhost_syncpt *sp);

int nvhost_update_characteristics(struct platform_device *dev);

static inline void *nvhost_get_private_data(struct platform_device *_dev)
{
	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)platform_get_drvdata(_dev);
	BUG_ON(!pdata);
	return pdata ? pdata->private_data : NULL;
}

static inline void nvhost_set_private_data(struct platform_device *_dev,
	void *priv_data)
{
	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)platform_get_drvdata(_dev);
	WARN_ON(!pdata);
	pdata->private_data = priv_data;
}

static inline struct nvhost_master *nvhost_get_host(
	struct platform_device *_dev)
{
	struct platform_device *pdev;

	if (_dev->dev.parent && _dev->dev.parent != &platform_bus) {
		pdev = to_platform_device(_dev->dev.parent);
		return nvhost_get_private_data(pdev);
	} else
		return nvhost_get_private_data(_dev);
}

static inline struct platform_device *nvhost_get_parent(
	struct platform_device *_dev)
{
	return (_dev->dev.parent && _dev->dev.parent != &platform_bus)
		? to_platform_device(_dev->dev.parent) : NULL;
}

#endif
