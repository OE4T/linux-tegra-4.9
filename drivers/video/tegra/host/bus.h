/*
 * drivers/video/tegra/host/bus.h
 *
 * Tegra Graphics Host bus API header
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#ifndef __NVHOST_BUS_H
#define __NVHOST_BUS_H

#include <linux/types.h>
#include <linux/device.h>

#include "chip_support.h"

struct nvhost_bus {
	struct nvhost_chip_support *nvhost_chip_ops;
	struct bus_type nvhost_bus_type;
};

struct nvhost_bus *nvhost_bus_get(void);

extern struct nvhost_bus *nvhost_bus_inst;

#endif
