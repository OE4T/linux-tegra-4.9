/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVHOST_GM20B_GR_MMU_H
#define _NVHOST_GM20B_GR_MMU_H

struct gk20a;

enum {
	MAXWELL_B		= 0xB197,
	MAXWELL_COMPUTE_B	= 0xB1C0,
	KEPLER_INLINE_TO_MEMORY_B= 0xA140,
	MAXWELL_DMA_COPY_A	= 0xB0B5,
	MAXWELL_CHANNEL_GPFIFO_A= 0xB06F,
};

#define NVB197_SET_ALPHA_CIRCULAR_BUFFER_SIZE	0x02dc
#define NVB197_SET_CIRCULAR_BUFFER_SIZE		0x1280
#define NVB197_SET_SHADER_EXCEPTIONS		0x1528
#define NVB197_SET_RD_COALESCE			0x102c
#define NVB1C0_SET_SHADER_EXCEPTIONS		0x1528
#define NVB1C0_SET_RD_COALESCE			0x0228

#define NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE 0

void gr_gm20b_commit_global_attrib_cb(struct gk20a *g,
				      struct channel_ctx_gk20a *ch_ctx,
				      u64 addr, bool patch);
int gr_gm20b_init_fs_state(struct gk20a *g);
int gm20b_gr_tpc_disable_override(struct gk20a *g, u32 mask);
void gr_gm20b_set_rd_coalesce(struct gk20a *g, u32 data);
void gm20a_gr_disable_rd_coalesce(struct gk20a *g);
void gr_gm20b_init_gpc_mmu(struct gk20a *g);
void gr_gm20b_bundle_cb_defaults(struct gk20a *g);
void gr_gm20b_cb_size_default(struct gk20a *g);
int gr_gm20b_calc_global_ctx_buffer_size(struct gk20a *g);
void gr_gm20b_commit_global_bundle_cb(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u64 size, bool patch);
int gr_gm20b_commit_global_cb_manager(struct gk20a *g,
			struct channel_gk20a *c, bool patch);
void gr_gm20b_commit_global_pagepool(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u32 size, bool patch);
int gr_gm20b_handle_sw_method(struct gk20a *g, u32 addr,
					  u32 class_num, u32 offset, u32 data);
void gr_gm20b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data);
void gr_gm20b_set_circular_buffer_size(struct gk20a *g, u32 data);
void gr_gm20b_set_hww_esr_report_mask(struct gk20a *g);
bool gr_gm20b_is_valid_class(struct gk20a *g, u32 class_num);
bool gr_gm20b_is_valid_gfx_class(struct gk20a *g, u32 class_num);
bool gr_gm20b_is_valid_compute_class(struct gk20a *g, u32 class_num);
void gr_gm20b_init_sm_dsm_reg_info(void);
void gr_gm20b_get_sm_dsm_perf_regs(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride);
void gr_gm20b_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_ctrl_regs,
					       u32 **sm_dsm_perf_ctrl_regs,
					       u32 *ctrl_register_stride);
u32 gr_gm20b_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index);
void gr_gm20b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index);
void gr_gm20b_load_tpc_mask(struct gk20a *g);
void gr_gm20b_program_sm_id_numbering(struct gk20a *g,
					     u32 gpc, u32 tpc, u32 smid);
int gr_gm20b_load_smid_config(struct gk20a *g);
int gr_gm20b_load_ctxsw_ucode_segments(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset);
bool gr_gm20b_is_tpc_addr(struct gk20a *g, u32 addr);
u32 gr_gm20b_get_tpc_num(struct gk20a *g, u32 addr);
int gr_gm20b_load_ctxsw_ucode(struct gk20a *g);
int gr_gm20b_load_ctxsw_ucode(struct gk20a *g);
void gr_gm20b_detect_sm_arch(struct gk20a *g);
u32 gr_gm20b_pagepool_default_size(struct gk20a *g);
int gr_gm20b_alloc_gr_ctx(struct gk20a *g,
			  struct gr_ctx_desc **gr_ctx, struct vm_gk20a *vm,
			  u32 class,
			  u32 flags);
void gr_gm20b_update_ctxsw_preemption_mode(struct gk20a *g,
		struct channel_ctx_gk20a *ch_ctx,
		struct nvgpu_mem *mem);
int gr_gm20b_dump_gr_status_regs(struct gk20a *g,
			   struct gk20a_debug_output *o);
int gr_gm20b_update_pc_sampling(struct channel_gk20a *c,
				       bool enable);
u32 gr_gm20b_get_fbp_en_mask(struct gk20a *g);
u32 gr_gm20b_get_max_ltc_per_fbp(struct gk20a *g);
u32 gr_gm20b_get_max_lts_per_ltc(struct gk20a *g);
u32 *gr_gm20b_rop_l2_en_mask(struct gk20a *g);
u32 gr_gm20b_get_max_fbps_count(struct gk20a *g);
void gr_gm20b_init_cyclestats(struct gk20a *g);
void gr_gm20b_enable_cde_in_fecs(struct gk20a *g, struct nvgpu_mem *mem);
void gr_gm20b_bpt_reg_info(struct gk20a *g, struct warpstate *w_state);
void gr_gm20b_get_access_map(struct gk20a *g,
				   u32 **whitelist, int *num_entries);
int gm20b_gr_record_sm_error_state(struct gk20a *g, u32 gpc, u32 tpc);
int gm20b_gr_update_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id,
		struct nvgpu_dbg_gpu_sm_error_state_record *sm_error_state);
int gm20b_gr_clear_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id);
int gr_gm20b_get_preemption_mode_flags(struct gk20a *g,
		struct nvgpu_preemption_modes_rec *preemption_modes_rec);
int gm20b_gr_fuse_override(struct gk20a *g);
bool gr_gm20b_is_ltcs_ltss_addr(struct gk20a *g, u32 addr);
bool gr_gm20b_is_ltcn_ltss_addr(struct gk20a *g, u32 addr);
void gr_gm20b_split_lts_broadcast_addr(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index);
void gr_gm20b_split_ltc_broadcast_addr(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index);
void gm20b_gr_clear_sm_hww(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
			u32 global_esr);
#endif
