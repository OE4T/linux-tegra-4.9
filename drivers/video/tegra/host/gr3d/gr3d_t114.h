/*
 * drivers/video/tegra/host/t30/3dctx_t114.h
 *
 * Tegra Graphics Host Context Switching for Tegra11x SOCs
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __NVHOST_3DCTX_T114_H
#define __NVHOST_3DCTX_T114_H

struct nvhost_hwctx_handler;
struct nvhost_device;
struct nvhost_channel;
struct nvhost_hwctx;

struct nvhost_hwctx_handler *nvhost_gr3d_t114_ctxhandler_init(
		u32 syncpt, u32 base,
		struct nvhost_channel *ch);

void nvhost_gr3d_t114_init(struct nvhost_device *dev);
void nvhost_gr3d_t114_deinit(struct nvhost_device *dev);
int nvhost_gr3d_t114_prepare_power_off(struct nvhost_device *dev);
void nvhost_gr3d_t114_finalize_power_on(struct nvhost_device *dev);

int nvhost_gr3d_t114_read_reg(
	struct nvhost_device *dev,
	struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset,
	u32 *value);

#endif
