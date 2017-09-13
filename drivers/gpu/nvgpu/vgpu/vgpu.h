/*
 * Virtualized GPU Interfaces
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _VIRT_H_
#define _VIRT_H_

#include <linux/tegra_gr_comm.h>
#include <linux/tegra_vgpu.h>
#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "common/linux/os_linux.h"

#include <nvgpu/thread.h>

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION

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

static inline u64 vgpu_get_handle_from_dev(struct device *dev)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data_from_dev(dev);

	if (unlikely(!priv)) {
		dev_err(dev, "invalid vgpu_priv_data in %s\n", __func__);
		return INT_MAX;
	}

	return priv->virt_handle;
}

static inline u64 vgpu_get_handle(struct gk20a *g)
{
	return vgpu_get_handle_from_dev(dev_from_gk20a(g));
}

int vgpu_pm_prepare_poweroff(struct device *dev);
int vgpu_pm_finalize_poweron(struct device *dev);
int vgpu_probe(struct platform_device *dev);
int vgpu_remove(struct platform_device *dev);
u64 vgpu_bar1_map(struct gk20a *g, struct sg_table **sgt, u64 size);
int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info);
int vgpu_gr_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_gr_nonstall_intr_info *info);
int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
			struct gr_ctx_desc **__gr_ctx,
			struct vm_gk20a *vm,
			u32 class,
			u32 flags);
void vgpu_gr_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
			struct gr_ctx_desc *gr_ctx);
void vgpu_gr_handle_sm_esr_event(struct gk20a *g,
			struct tegra_vgpu_sm_esr_info *info);
int vgpu_gr_init_ctx_state(struct gk20a *g);
int vgpu_fifo_isr(struct gk20a *g, struct tegra_vgpu_fifo_intr_info *info);
int vgpu_fifo_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_fifo_nonstall_intr_info *info);
int vgpu_ce2_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_ce2_nonstall_intr_info *info);
void vgpu_init_fifo_ops(struct gpu_ops *gops);
void vgpu_init_gr_ops(struct gpu_ops *gops);
void vgpu_init_ltc_ops(struct gpu_ops *gops);
void vgpu_init_mm_ops(struct gpu_ops *gops);
void vgpu_init_debug_ops(struct gpu_ops *gops);
void vgpu_init_tsg_ops(struct gpu_ops *gops);
#if defined(CONFIG_GK20A_CYCLE_STATS)
void vgpu_init_css_ops(struct gpu_ops *gops);
#endif
void vgpu_init_ce2_ops(struct gpu_ops *gops);
int vgpu_init_mm_support(struct gk20a *g);
int vgpu_init_gr_support(struct gk20a *g);
int vgpu_init_fifo_support(struct gk20a *g);

int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value);
int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out);

void vgpu_init_hal_common(struct gk20a *g);
int vgpu_gm20b_init_hal(struct gk20a *g);
int vgpu_gp10b_init_hal(struct gk20a *g);

void vgpu_init_dbg_session_ops(struct gpu_ops *gops);
int vgpu_init_gpu_characteristics(struct gk20a *g);

void vgpu_create_sysfs(struct device *dev);
void vgpu_remove_sysfs(struct device *dev);
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
static inline u64 vgpu_bar1_map(struct gk20a *g, struct sg_table **sgt,
				u64 size)
{
	return 0;
}
static inline int vgpu_gr_isr(struct gk20a *g,
			struct tegra_vgpu_gr_intr_info *info)
{
	return 0;
}
static inline int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
				struct gr_ctx_desc **__gr_ctx,
				struct vm_gk20a *vm,
				u32 class,
				u32 flags)
{
	return -ENOSYS;
}
static inline void vgpu_gr_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
				struct gr_ctx_desc *gr_ctx)
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
static inline void vgpu_init_fifo_ops(struct gpu_ops *gops)
{
}
static inline void vgpu_init_gr_ops(struct gpu_ops *gops)
{
}
static inline void vgpu_init_ltc_ops(struct gpu_ops *gops)
{
}
static inline void vgpu_init_mm_ops(struct gpu_ops *gops)
{
}
static inline void vgpu_init_debug_ops(struct gpu_ops *gops)
{
}
#if defined(CONFIG_GK20A_CYCLE_STATS)
static inline void vgpu_init_css_ops(struct gpu_ops *gops)
{
}
#endif
static inline void vgpu_init_ce2_ops(struct gpu_ops *gops)
{
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
