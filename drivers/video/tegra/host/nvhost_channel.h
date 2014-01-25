/*
 * drivers/video/tegra/host/nvhost_channel.h
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2014, NVIDIA Corporation.  All rights reserved.
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

#define NVHOST_MAX_WAIT_CHECKS		256
#define NVHOST_MAX_GATHERS		512
#define NVHOST_MAX_HANDLES		1280
#define NVHOST_MAX_POWERGATE_IDS	2

struct nvhost_master;
struct platform_device;
struct nvhost_channel;
struct nvhost_hwctx;

struct nvhost_channel_ops {
	const char *soc_name;
	int (*init)(struct nvhost_channel *,
		    struct nvhost_master *);
	int (*submit)(struct nvhost_job *job);
	int (*save_context)(struct nvhost_channel *channel);
	int (*init_gather_filter)(struct nvhost_channel *ch);
};

struct nvhost_channel {
	struct nvhost_channel_ops ops;
	int refcount;
	int chid;
	u32 syncpt_id;
	struct mutex reflock;
	struct mutex submitlock;
	void __iomem *aperture;
	struct nvhost_hwctx *cur_ctx;
	struct platform_device *dev;
	struct nvhost_hwctx_handler *ctxhandler;
	struct nvhost_cdma cdma;

	/* the address space block here
	 * belongs to the module. but for
	 * now just keep it here */
	struct device *as_node;
	struct cdev as_cdev;
	struct nvhost_as *as;
};

#define channel_op(ch)		(ch->ops)

int nvhost_channel_init(struct nvhost_channel *ch,
	struct nvhost_master *dev);

int nvhost_channel_submit(struct nvhost_job *job);

struct nvhost_channel *nvhost_getchannel(struct nvhost_channel *ch,
		bool force, bool init);
void nvhost_putchannel(struct nvhost_channel *ch, bool deinit);
int nvhost_channel_suspend(struct nvhost_channel *ch);

int nvhost_channel_read_reg(struct nvhost_channel *channel,
	struct nvhost_hwctx *hwctx,
	u32 offset, u32 *value);

struct nvhost_channel *nvhost_alloc_channel_internal(int chindex,
	int max_channels, int *current_channel_count);

void nvhost_free_channel_internal(struct nvhost_channel *ch,
	int *current_channel_count);

int nvhost_channel_save_context(struct nvhost_channel *ch);
void nvhost_channel_init_gather_filter(struct nvhost_channel *ch);

struct nvhost_hwctx *nvhost_channel_get_file_hwctx(int fd);

struct nvhost_hwctx_handler *nvhost_alloc_hwctx_handler(u32 syncpt,
	u32 waitbase, struct nvhost_channel *ch);

#endif
