/*
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

#ifndef __VGPU_COMMON_H__
#define __VGPU_COMMON_H__

#include <nvgpu/types.h>
#include <nvgpu/thread.h>
#include <nvgpu/log.h>
#include <nvgpu/vgpu/tegra_vgpu.h>

struct device;
struct tegra_vgpu_gr_intr_info;
struct tegra_vgpu_fifo_intr_info;
struct tegra_vgpu_cmd_msg;
struct nvgpu_mem;
struct gk20a;
struct vm_gk20a;
struct nvgpu_gr_ctx;
struct nvgpu_cpu_time_correlation_sample;

struct vgpu_priv_data {
	u64 virt_handle;
	struct nvgpu_thread intr_handler;
	struct tegra_vgpu_constants_params constants;
};

struct vgpu_priv_data *vgpu_get_priv_data(struct gk20a *g);

static inline u64 vgpu_get_handle(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	if (unlikely(!priv)) {
		nvgpu_err(g, "invalid vgpu_priv_data in %s", __func__);
		return INT_MAX;
	}

	return priv->virt_handle;
}

int vgpu_comm_init(struct gk20a *g);
void vgpu_comm_deinit(void);
int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out);
u64 vgpu_connect(void);
int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value);
int vgpu_intr_thread(void *dev_id);
void vgpu_remove_support_common(struct gk20a *g);
void vgpu_detect_chip(struct gk20a *g);
int vgpu_init_gpu_characteristics(struct gk20a *g);
int vgpu_read_ptimer(struct gk20a *g, u64 *value);
int vgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples);
int vgpu_init_hal(struct gk20a *g);
int vgpu_get_constants(struct gk20a *g);
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

int vgpu_gp10b_init_hal(struct gk20a *g);
int vgpu_gv11b_init_hal(struct gk20a *g);

int vgpu_read_ptimer(struct gk20a *g, u64 *value);
int vgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples);
bool vgpu_is_reduced_bar1(struct gk20a *g);

#endif
