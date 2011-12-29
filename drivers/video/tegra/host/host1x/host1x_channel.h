/*
 * drivers/video/tegra/host/host1x/host1x_channel.h
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef __NVHOST_HOST1X_CHANNEL_H
#define __NVHOST_HOST1X_CHANNEL_H

struct nvhost_job;
struct nvhost_channel;
struct nvhost_hwctx;
struct nvhost_device;

/*  Submit job to a host1x client */
int host1x_channel_submit(struct nvhost_job *job);

/*  Read 3d register via FIFO */
int host1x_channel_read_3d_reg(
	struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset,
	u32 *value);

/* Reads words from FIFO */
int host1x_drain_read_fifo(void __iomem *chan_regs,
		u32 *ptr, unsigned int count, unsigned int *pending);

int host1x_save_context(struct nvhost_device *dev, u32 syncpt_id);

#endif
