/*
 * drivers/video/tegra/host/nvhost_channel.h
 *
 * Tegra Graphics Host Channel
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

#ifndef __NVHOST_CHANNEL_H
#define __NVHOST_CHANNEL_H

#include <linux/cdev.h>
#include <linux/io.h>
#include "nvhost_cdma.h"

#define NVHOST_MAX_WAIT_CHECKS 256
#define NVHOST_MAX_GATHERS 512
#define NVHOST_MAX_HANDLES 1280
#define NVHOST_MAX_POWERGATE_IDS 2

struct nvhost_master;
struct nvhost_waitchk;
struct nvhost_device;
struct nvhost_channel;
struct nvhost_hwctx;

struct nvhost_channel_gather {
	u32 words;
	phys_addr_t mem;
	u32 mem_id;
	int offset;
};

struct nvhost_channel {
	int refcount;
	int chid;
	u32 syncpt_id;
	struct mutex reflock;
	struct mutex submitlock;
	void __iomem *aperture;
	struct nvhost_hwctx *cur_ctx;
	struct device *node;
	struct nvhost_device *dev;
	struct cdev cdev;
	struct nvhost_hwctx_handler *ctxhandler;
	struct nvhost_cdma cdma;
};

int nvhost_channel_init(
	struct nvhost_channel *ch,
	struct nvhost_master *dev, int index);

int nvhost_channel_submit(struct nvhost_job *job);

struct nvhost_channel *nvhost_getchannel(struct nvhost_channel *ch);
void nvhost_putchannel(struct nvhost_channel *ch, struct nvhost_hwctx *ctx);
int nvhost_channel_suspend(struct nvhost_channel *ch);

#define channel_cdma_op(ch) (nvhost_get_host(ch->dev)->op.cdma)
#define channel_op(ch) (nvhost_get_host(ch->dev)->op.channel)
#define host_channel_op(host) (host->op.channel)

int nvhost_channel_drain_read_fifo(void __iomem *chan_regs,
			u32 *ptr, unsigned int count, unsigned int *pending);

int nvhost_channel_read_3d_reg(
	struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset,
	u32 *value);

#endif
