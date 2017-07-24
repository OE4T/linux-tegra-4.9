/*
 * GP10B GPU GR
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _NVGPU_GR_GP10B_H_
#define _NVGPU_GR_GP10B_H_

#include <linux/version.h>

#include "gk20a/mm_gk20a.h"

struct gk20a;
struct gr_gk20a_isr_data;

enum {
	PASCAL_CHANNEL_GPFIFO_A  = 0xC06F,
	PASCAL_A                 = 0xC097,
	PASCAL_COMPUTE_A         = 0xC0C0,
	PASCAL_DMA_COPY_A        = 0xC0B5,
	PASCAL_DMA_COPY_B        = 0xC1B5,
};

#define NVC097_SET_GO_IDLE_TIMEOUT		0x022c
#define NVC097_SET_ALPHA_CIRCULAR_BUFFER_SIZE	0x02dc
#define NVC097_SET_COALESCE_BUFFER_SIZE		0x1028
#define NVC097_SET_RD_COALESCE			0x102c
#define NVC097_SET_CIRCULAR_BUFFER_SIZE		0x1280
#define NVC097_SET_SHADER_EXCEPTIONS		0x1528
#define NVC097_SET_BES_CROP_DEBUG3		0x10c4
#define NVC0C0_SET_SHADER_EXCEPTIONS		0x1528
#define NVC0C0_SET_RD_COALESCE			0x0228

void gp10b_init_gr(struct gk20a *g);
int gr_gp10b_init_fs_state(struct gk20a *g);
int gr_gp10b_alloc_buffer(struct vm_gk20a *vm, size_t size,
			struct nvgpu_mem *mem);
void gr_gp10b_create_sysfs(struct device *dev);
int gr_gp10b_handle_fecs_error(struct gk20a *g,
			struct channel_gk20a *__ch,
			struct gr_gk20a_isr_data *isr_data);
int gr_gp10b_set_cilp_preempt_pending(struct gk20a *g,
		struct channel_gk20a *fault_ch);

struct gr_t18x {
	struct {
		u32 preempt_image_size;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
		u32 force_preemption_gfxp;
		u32 force_preemption_cilp;
		u32 dump_ctxsw_stats_on_channel_close;
#else
		bool force_preemption_gfxp;
		bool force_preemption_cilp;
		bool dump_ctxsw_stats_on_channel_close;
#endif
		struct dentry *debugfs_force_preemption_cilp;
		struct dentry *debugfs_force_preemption_gfxp;
		struct dentry *debugfs_dump_ctxsw_stats;
	} ctx_vars;

	u32 fecs_feature_override_ecc_val;

	int cilp_preempt_pending_chid;
};

struct gr_ctx_desc_t18x {
	struct nvgpu_mem preempt_ctxsw_buffer;
	struct nvgpu_mem spill_ctxsw_buffer;
	struct nvgpu_mem betacb_ctxsw_buffer;
	struct nvgpu_mem pagepool_ctxsw_buffer;
	u32 ctx_id;
	bool ctx_id_valid;
	bool cilp_preempt_pending;
};

#endif
