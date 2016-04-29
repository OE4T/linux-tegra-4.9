/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_IVC_BUS_H
#define _LINUX_TEGRA_IVC_BUS_H

extern struct bus_type tegra_ivc_bus_type;
struct tegra_ivc_bus;
struct tegra_hsp_ops;

struct tegra_ivc_bus *tegra_ivc_bus_create(struct device *, u32 sid);
void tegra_ivc_bus_destroy(struct tegra_ivc_bus *ibus);

struct tegra_ivc_driver {
	struct device_driver driver;
	struct device_type *dev_type;
	const struct tegra_hsp_ops *ops;
};

static inline struct tegra_ivc_driver *to_tegra_ivc_driver(
						struct device_driver *drv)
{
	if (drv == NULL)
		return NULL;
	return container_of(drv, struct tegra_ivc_driver, driver);
}

int tegra_ivc_driver_register(struct tegra_ivc_driver *drv);
void tegra_ivc_driver_unregister(struct tegra_ivc_driver *drv);
#define tegra_ivc_module_driver(drv) \
	module_driver(drv, tegra_ivc_driver_register, \
			tegra_ivc_driver_unregister)

/* Tegra HSP driver support */
extern struct device_type tegra_hsp_type;

struct tegra_hsp_ops {
	int (*probe)(struct device *);
	void (*remove)(struct device *);
	int (*enable)(struct device *);
	void (*disable)(struct device *);
	void (*ring)(struct device *);
};
void tegra_hsp_notify(struct device *);

static inline const struct tegra_hsp_ops *tegra_hsp_dev_ops(struct device *dev)
{
	struct tegra_ivc_driver *drv = to_tegra_ivc_driver(dev->driver);
	const struct tegra_hsp_ops *ops = NULL;

	if (drv != NULL && drv->dev_type == &tegra_hsp_type)
		ops = drv->ops;
	return ops;
}

/* Legacy mailbox support */
struct tegra_ivc_mbox_msg {
	int length;
	void *data;
};

#endif
