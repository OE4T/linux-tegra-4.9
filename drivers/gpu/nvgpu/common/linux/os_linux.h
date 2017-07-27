/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_OS_LINUX_H
#define NVGPU_OS_LINUX_H

#include <linux/cdev.h>

#include "gk20a/gk20a.h"

struct nvgpu_os_linux {
	struct gk20a g;
	struct device *dev;

	struct {
		struct cdev cdev;
		struct device *node;
	} channel;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctrl;

	struct {
		struct cdev cdev;
		struct device *node;
	} as_dev;

	struct {
		struct cdev cdev;
		struct device *node;
	} dbg;

	struct {
		struct cdev cdev;
		struct device *node;
	} prof;

	struct {
		struct cdev cdev;
		struct device *node;
	} tsg;

	struct {
		struct cdev cdev;
		struct device *node;
	} ctxsw;

	struct {
		struct cdev cdev;
		struct device *node;
	} sched;

	dev_t cdev_region;

	struct devfreq *devfreq;

	struct device_dma_parameters dma_parms;

	atomic_t hw_irq_stall_count;
	atomic_t hw_irq_nonstall_count;

	wait_queue_head_t sw_irq_stall_last_handled_wq;
	atomic_t sw_irq_stall_last_handled;

	atomic_t nonstall_ops;

	wait_queue_head_t sw_irq_nonstall_last_handled_wq;
	atomic_t sw_irq_nonstall_last_handled;

	struct work_struct nonstall_fn_work;
	struct workqueue_struct *nonstall_work_queue;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
	struct dentry *debugfs_alias;

	struct dentry *debugfs_ltc_enabled;
	struct dentry *debugfs_timeouts_enabled;
	struct dentry *debugfs_gr_idle_timeout_default;
	struct dentry *debugfs_bypass_smmu;
	struct dentry *debugfs_disable_bigpage;
	struct dentry *debugfs_gr_default_attrib_cb_size;

	struct dentry *debugfs_timeslice_low_priority_us;
	struct dentry *debugfs_timeslice_medium_priority_us;
	struct dentry *debugfs_timeslice_high_priority_us;
	struct dentry *debugfs_runlist_interleave;
	struct dentry *debugfs_allocators;
	struct dentry *debugfs_xve;
	struct dentry *debugfs_kmem;

	struct dentry *debugfs_force_preemption_cilp;
	struct dentry *debugfs_force_preemption_gfxp;
	struct dentry *debugfs_dump_ctxsw_stats;
#endif
};

static inline struct nvgpu_os_linux *nvgpu_os_linux_from_gk20a(struct gk20a *g)
{
	return container_of(g, struct nvgpu_os_linux, g);
}

static inline struct device *dev_from_gk20a(struct gk20a *g)
{
	return nvgpu_os_linux_from_gk20a(g)->dev;
}

#define INTERFACE_NAME "nvhost%s-gpu"

#endif
