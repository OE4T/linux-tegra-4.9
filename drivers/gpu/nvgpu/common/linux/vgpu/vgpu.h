/*
 * Virtualized GPU Interfaces
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _VIRT_H_
#define _VIRT_H_

struct device;
struct tegra_vgpu_gr_intr_info;
struct tegra_vgpu_fifo_intr_info;
struct tegra_vgpu_cmd_msg;
struct gk20a_platform;
struct nvgpu_mem;

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/thread.h>

#include "gk20a/gk20a.h"
#include "common/linux/platform_gk20a.h"
#include "common/linux/os_linux.h"

struct vgpu_priv_data {
	u64 virt_handle;
	struct nvgpu_thread intr_handler;
	struct tegra_vgpu_constants_params constants;
};

static inline
struct vgpu_priv_data *vgpu_get_priv_data_from_dev(struct device *dev)
{
	struct gk20a_platform *plat = gk20a_get_platform(dev);

	return (struct vgpu_priv_data *)plat->vgpu_priv;
}

static inline struct vgpu_priv_data *vgpu_get_priv_data(struct gk20a *g)
{
	return vgpu_get_priv_data_from_dev(dev_from_gk20a(g));
}

static inline u64 vgpu_get_handle(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	if (unlikely(!priv)) {
		nvgpu_err(g, "invalid vgpu_priv_data in %s", __func__);
		return INT_MAX;
	}

	return priv->virt_handle;
}

int vgpu_pm_prepare_poweroff(struct device *dev);
int vgpu_pm_finalize_poweron(struct device *dev);
int vgpu_probe(struct platform_device *dev);
int vgpu_remove(struct platform_device *dev);
u64 vgpu_bar1_map(struct gk20a *g, struct nvgpu_mem *mem);
int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info);
int vgpu_gr_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_gr_nonstall_intr_info *info);
int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
			struct nvgpu_gr_ctx *gr_ctx,
			struct vm_gk20a *vm,
			u32 class,
			u32 flags);
void vgpu_gr_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
			struct nvgpu_gr_ctx *gr_ctx);
void vgpu_gr_handle_sm_esr_event(struct gk20a *g,
			struct tegra_vgpu_sm_esr_info *info);
int vgpu_gr_init_ctx_state(struct gk20a *g);
int vgpu_fifo_isr(struct gk20a *g, struct tegra_vgpu_fifo_intr_info *info);
int vgpu_fifo_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_fifo_nonstall_intr_info *info);
int vgpu_ce2_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_ce2_nonstall_intr_info *info);
u32 vgpu_ce_get_num_pce(struct gk20a *g);
int vgpu_init_mm_support(struct gk20a *g);
int vgpu_init_gr_support(struct gk20a *g);
int vgpu_init_fifo_support(struct gk20a *g);

int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value);
int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out);

int vgpu_gp10b_init_hal(struct gk20a *g);
int vgpu_gv11b_init_hal(struct gk20a *g);

int vgpu_init_gpu_characteristics(struct gk20a *g);

void vgpu_create_sysfs(struct device *dev);
void vgpu_remove_sysfs(struct device *dev);
int vgpu_read_ptimer(struct gk20a *g, u64 *value);
int vgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples);
#else
static inline int vgpu_pm_prepare_poweroff(struct device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_pm_finalize_poweron(struct device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_probe(struct platform_device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_remove(struct platform_device *dev)
{
	return -ENOSYS;
}
static inline u64 vgpu_bar1_map(struct gk20a *g, struct nvgpu_mem *mem)
{
	return 0;
}
static inline int vgpu_gr_isr(struct gk20a *g,
			struct tegra_vgpu_gr_intr_info *info)
{
	return 0;
}
static inline int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
				struct nvgpu_gr_ctx *gr_ctx,
				struct vm_gk20a *vm,
				u32 class,
				u32 flags)
{
	return -ENOSYS;
}
static inline void vgpu_gr_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
				struct nvgpu_gr_ctx *gr_ctx)
{
}
static inline int vgpu_gr_init_ctx_state(struct gk20a *g)
{
	return -ENOSYS;
}
static inline int vgpu_fifo_isr(struct gk20a *g,
			struct tegra_vgpu_fifo_intr_info *info)
{
	return 0;
}
static inline int vgpu_init_mm_support(struct gk20a *g)
{
	return -ENOSYS;
}
static inline int vgpu_init_gr_support(struct gk20a *g)
{
	return -ENOSYS;
}
static inline int vgpu_init_fifo_support(struct gk20a *g)
{
	return -ENOSYS;
}

static inline int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value)
{
	return -ENOSYS;
}
static inline int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out)
{
	return -ENOSYS;
}
#endif

#endif
