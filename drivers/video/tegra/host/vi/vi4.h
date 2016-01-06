/*
 * drivers/video/tegra/host/vi/vi4.h
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2015-2016 NVIDIA Corporation.  All rights reserved.
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

#ifndef __TEGRA_VI4_H__
#define __TEGRA_VI4_H__

#include "mc_common.h"

struct nvhost_vi_notify_dev {
	struct vi_notify_dev *vnd;
	u32 mask;
	u32 ld_mask;
	int error_irq;
	int prio_irq;
	int norm_irq;
	atomic_t overflow;
	atomic_t notify_overflow;
	atomic_t fmlite_overflow;

	struct {
		spinlock_t lock;
		struct list_head list;
	} incr[12];
};

struct reset_control;

extern struct vi_notify_driver nvhost_vi_notify_driver;

struct nvhost_vi_dev {
#ifdef CONFIG_TEGRA_VI_NOTIFY
	struct nvhost_vi_notify_dev notify;
#endif
	struct reset_control *vi_reset;
	struct reset_control *vi_tsc_reset;
	struct dentry *debug_dir;
	struct tegra_mc_vi mc_vi;
};

void nvhost_vi4_reset(struct platform_device *);
extern const struct file_operations nvhost_vi4_ctrl_ops;

#endif
