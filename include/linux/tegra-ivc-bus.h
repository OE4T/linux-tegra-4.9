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

struct tegra_ivc_bus *tegra_ivc_bus_create(struct device *, u32 sid);
void tegra_ivc_bus_destroy(struct tegra_ivc_bus *ibus);

struct tegra_ivc_mbox_msg {
	int length;
	void *data;
};

#endif
