/*
 * GM20B GPU graphics ops
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

#ifndef _GR_OPS_GM20B_H_
#define _GR_OPS_GM20B_H_

#include "gr_ops.h"

#define __gr_gm20b_op(X)            gr_gm20b_ ## X
#define __set_gr_gm20b_op(X)  . X = gr_gm20b_ ## X

void __gr_gm20b_op(init_gpc_mmu)(struct gk20a *);
void __gr_gm20b_op(bundle_cb_defaults)(struct gk20a *);
void __gr_gm20b_op(cb_size_default)(struct gk20a *);
int  __gr_gm20b_op(calc_global_ctx_buffer_size)(struct gk20a *);
void __gr_gm20b_op(commit_global_bundle_cb)(struct gk20a *,
		struct channel_ctx_gk20a *, u64, u64, bool);
int  __gr_gm20b_op(commit_global_cb_manager)(struct gk20a *,
		struct channel_gk20a *, bool);
void __gr_gm20b_op(commit_global_pagepool)(struct gk20a *,
		struct channel_ctx_gk20a *, u64 , u32, bool);
int  __gr_gm20b_op(handle_sw_method)(struct gk20a *, u32 , u32, u32, u32);
void __gr_gm20b_op(set_alpha_circular_buffer_size)(struct gk20a *, u32);
void __gr_gm20b_op(set_circular_buffer_size)(struct gk20a *, u32);
void __gr_gm20b_op(enable_hww_exceptions)(struct gk20a *);
bool __gr_gm20b_op(is_valid_class)(struct gk20a *, u32);
void __gr_gm20b_op(get_sm_dsm_perf_regs)(struct gk20a *, u32 *, u32 **, u32 *);
void __gr_gm20b_op(get_sm_dsm_perf_ctrl_regs)(struct gk20a *,
		u32 *, u32 **, u32 *);
int  __gr_gm20b_op(init_fs_state)(struct gk20a *);
void __gr_gm20b_op(set_hww_esr_report_mask)(struct gk20a *);
int  __gr_gm20b_op(falcon_load_ucode)(struct gk20a *,
		u64, struct gk20a_ctxsw_ucode_segments *, u32);
u32  __gr_gm20b_op(get_gpc_tpc_mask)(struct gk20a *, u32);
int  __gr_gm20b_op(load_ctxsw_ucode)(struct gk20a *);

#define __set_gr_gm20b_ops()				\
	/* newly defined for gm20b */			\
	__set_gr_gm20b_op(init_gpc_mmu),		\
	__set_gr_gm20b_op(bundle_cb_defaults),		\
	__set_gr_gm20b_op(cb_size_default),		\
	__set_gr_gm20b_op(calc_global_ctx_buffer_size),	\
	__set_gr_gm20b_op(commit_global_bundle_cb),	\
	__set_gr_gm20b_op(commit_global_cb_manager),	\
	__set_gr_gm20b_op(commit_global_pagepool),	\
	__set_gr_gm20b_op(handle_sw_method),		\
	__set_gr_gm20b_op(set_alpha_circular_buffer_size), \
	__set_gr_gm20b_op(set_circular_buffer_size),	\
	__set_gr_gm20b_op(enable_hww_exceptions),	\
	__set_gr_gm20b_op(is_valid_class),		\
	__set_gr_gm20b_op(get_sm_dsm_perf_regs),	\
	__set_gr_gm20b_op(get_sm_dsm_perf_ctrl_regs),	\
	__set_gr_gm20b_op(init_fs_state),		\
	__set_gr_gm20b_op(set_hww_esr_report_mask),	\
	__set_gr_gm20b_op(falcon_load_ucode),		\
	__set_gr_gm20b_op(get_gpc_tpc_mask),		\
							\
	/* reused from gk20a */				\
	__set_gr_gk20a_op(access_smpc_reg),		\
	__set_gr_gk20a_op(commit_global_attrib_cb),	\
	__set_gr_gk20a_op(free_channel_ctx),		\
	__set_gr_gk20a_op(alloc_obj_ctx),		\
	__set_gr_gk20a_op(free_obj_ctx),		\
	__set_gr_gk20a_op(bind_ctxsw_zcull),		\
	__set_gr_gk20a_op(get_zcull_info)

#endif
