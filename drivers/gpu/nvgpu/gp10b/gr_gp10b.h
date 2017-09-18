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

#include "gk20a/mm_gk20a.h"

struct gk20a;
struct gr_gk20a_isr_data;
struct channel_ctx_gk20a;
struct zbc_entry;
struct gr_ctx_desc;
struct nvgpu_preemption_modes_rec;

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

int gr_gp10b_init_fs_state(struct gk20a *g);
int gr_gp10b_alloc_buffer(struct vm_gk20a *vm, size_t size,
			struct nvgpu_mem *mem);
void gr_gp10b_create_sysfs(struct device *dev);
int gr_gp10b_handle_fecs_error(struct gk20a *g,
			struct channel_gk20a *__ch,
			struct gr_gk20a_isr_data *isr_data);
int gr_gp10b_set_cilp_preempt_pending(struct gk20a *g,
		struct channel_gk20a *fault_ch);

bool gr_gp10b_is_valid_class(struct gk20a *g, u32 class_num);
bool gr_gp10b_is_valid_gfx_class(struct gk20a *g, u32 class_num);
bool gr_gp10b_is_valid_compute_class(struct gk20a *g, u32 class_num);
int gr_gp10b_handle_sm_exception(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr);
int gr_gp10b_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event);
int gr_gp10b_commit_global_cb_manager(struct gk20a *g,
			struct channel_gk20a *c, bool patch);
void gr_gp10b_commit_global_pagepool(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u32 size, bool patch);
int gr_gp10b_add_zbc_color(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *color_val, u32 index);
int gr_gp10b_add_zbc_depth(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *depth_val, u32 index);
u32 gr_gp10b_pagepool_default_size(struct gk20a *g);
int gr_gp10b_calc_global_ctx_buffer_size(struct gk20a *g);
void gr_gp10b_set_bes_crop_debug3(struct gk20a *g, u32 data);
int gr_gp10b_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data);
void gr_gp10b_cb_size_default(struct gk20a *g);
void gr_gp10b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data);
void gr_gp10b_set_circular_buffer_size(struct gk20a *g, u32 data);
int gr_gp10b_init_ctx_state(struct gk20a *g);
int gr_gp10b_set_ctxsw_preemption_mode(struct gk20a *g,
				struct gr_ctx_desc *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode);
int gr_gp10b_alloc_gr_ctx(struct gk20a *g,
			  struct gr_ctx_desc **gr_ctx, struct vm_gk20a *vm,
			  u32 class,
			  u32 flags);
void gr_gp10b_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
			  struct gr_ctx_desc *gr_ctx);
void gr_gp10b_update_ctxsw_preemption_mode(struct gk20a *g,
		struct channel_ctx_gk20a *ch_ctx,
		struct nvgpu_mem *mem);
int gr_gp10b_dump_gr_status_regs(struct gk20a *g,
			   struct gk20a_debug_output *o);
int gr_gp10b_wait_empty(struct gk20a *g, unsigned long duration_ms,
			       u32 expect_delay);
void gr_gp10b_commit_global_attrib_cb(struct gk20a *g,
					     struct channel_ctx_gk20a *ch_ctx,
					     u64 addr, bool patch);
void gr_gp10b_commit_global_bundle_cb(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u64 size, bool patch);
int gr_gp10b_load_smid_config(struct gk20a *g);
void gr_gp10b_init_cyclestats(struct gk20a *g);
void gr_gp10b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index);
void gr_gp10b_get_access_map(struct gk20a *g,
				   u32 **whitelist, int *num_entries);
int gr_gp10b_pre_process_sm_exception(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm, u32 global_esr, u32 warp_esr,
		bool sm_debugger_attached, struct channel_gk20a *fault_ch,
		bool *early_exit, bool *ignore_debugger);
u32 gp10b_gr_get_sm_hww_warp_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm);
u32 get_ecc_override_val(struct gk20a *g);
int gr_gp10b_suspend_contexts(struct gk20a *g,
				struct dbg_session_gk20a *dbg_s,
				int *ctx_resident_ch_fd);
int gr_gp10b_set_boosted_ctx(struct channel_gk20a *ch,
				    bool boost);
void gr_gp10b_update_boosted_ctx(struct gk20a *g, struct nvgpu_mem *mem,
				       struct gr_ctx_desc *gr_ctx);
int gr_gp10b_set_preemption_mode(struct channel_gk20a *ch,
					u32 graphics_preempt_mode,
					u32 compute_preempt_mode);
int gr_gp10b_get_preemption_mode_flags(struct gk20a *g,
	struct nvgpu_preemption_modes_rec *preemption_modes_rec);
int gp10b_gr_fuse_override(struct gk20a *g);
int gr_gp10b_init_preemption_state(struct gk20a *g);
void gr_gp10b_set_preemption_buffer_va(struct gk20a *g,
			struct nvgpu_mem *mem, u64 gpu_va);
int gr_gp10b_set_czf_bypass(struct gk20a *g, struct channel_gk20a *ch);
void gr_gp10b_init_ctxsw_hdr_data(struct gk20a *g, struct nvgpu_mem *mem);

struct gr_t18x {
	struct {
		u32 preempt_image_size;
		bool force_preemption_gfxp;
		bool force_preemption_cilp;
		bool dump_ctxsw_stats_on_channel_close;
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
