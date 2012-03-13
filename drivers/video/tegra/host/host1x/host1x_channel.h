/*
 * drivers/video/tegra/host/host1x/host1x_channel.h
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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
