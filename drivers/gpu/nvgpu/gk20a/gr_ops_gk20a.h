/*
 * GPK20A GPU graphics ops
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _GR_OPS_GK20A_H_
#define _GR_OPS_GK20A_H_

#include "gr_ops.h"

#define __gr_gk20a_op(X)           gr_gk20a_ ## X
#define __set_gr_gk20a_op(X) . X = gr_gk20a_ ## X

int  __gr_gk20a_op(init_fs_state)(struct gk20a *);
void __gr_gk20a_op(access_smpc_reg)(struct gk20a *, u32, u32);
void __gr_gk20a_op(bundle_cb_defaults)(struct gk20a *);
void __gr_gk20a_op(cb_size_default)(struct gk20a *);
int  __gr_gk20a_op(calc_global_ctx_buffer_size)(struct gk20a *);
void __gr_gk20a_op(commit_global_attrib_cb)(struct gk20a *,
		struct channel_ctx_gk20a *, u64 , bool);
void __gr_gk20a_op(commit_global_bundle_cb)(struct gk20a *,
		struct channel_ctx_gk20a *, u64, u64, bool);
int  __gr_gk20a_op(commit_global_cb_manager)(struct gk20a *,
		struct channel_gk20a *, bool);
void __gr_gk20a_op(commit_global_pagepool)(struct gk20a *,
		struct channel_ctx_gk20a *, u64 , u32, bool);
void __gr_gk20a_op(init_gpc_mmu)(struct gk20a *);
int  __gr_gk20a_op(handle_sw_method)(struct gk20a *, u32 , u32, u32, u32);
void __gr_gk20a_op(set_alpha_circular_buffer_size)(struct gk20a *, u32);
void __gr_gk20a_op(set_circular_buffer_size)(struct gk20a *, u32);
void __gr_gk20a_op(enable_hww_exceptions)(struct gk20a *);
bool __gr_gk20a_op(is_valid_class)(struct gk20a *, u32);
void __gr_gk20a_op(get_sm_dsm_perf_regs)(struct gk20a *, u32 *, u32 **, u32 *);
void __gr_gk20a_op(get_sm_dsm_perf_ctrl_regs)(struct gk20a *,
		u32 *, u32 **, u32 *);
void __gr_gk20a_op(set_hww_esr_report_mask)(struct gk20a *);
int  __gr_gk20a_op(setup_alpha_beta_tables)(struct gk20a *, struct gr_gk20a *);
int  __gr_gk20a_op(falcon_load_ucode)(struct gk20a *, u64,
		struct gk20a_ctxsw_ucode_segments *, u32);
int  __gr_gk20a_op(load_ctxsw_ucode)(struct gk20a *);
u32  __gr_gk20a_op(get_gpc_tpc_mask)(struct gk20a *, u32);
void __gr_gk20a_op(free_channel_ctx)(struct channel_gk20a *);
int  __gr_gk20a_op(alloc_obj_ctx)(struct channel_gk20a *c,
		struct nvgpu_alloc_obj_ctx_args *);
int  __gr_gk20a_op(free_obj_ctx)(struct channel_gk20a *c,
		struct nvgpu_free_obj_ctx_args *);
int  __gr_gk20a_op(bind_ctxsw_zcull)(struct gk20a *,
		struct gr_gk20a *, struct channel_gk20a *, u64, u32);
int  __gr_gk20a_op(get_zcull_info)(struct gk20a *,
		struct gr_gk20a *, struct gr_zcull_info *);

#endif
