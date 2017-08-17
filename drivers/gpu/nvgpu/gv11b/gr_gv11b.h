/*
 * GV11B GPU GR
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_GR_GV11B_H_
#define _NVGPU_GR_GV11B_H_

#define EGPC_PRI_BASE        0x580000
#define EGPC_PRI_SHARED_BASE 0x480000

#define GV11B_ZBC_TYPE_STENCIL            T19X_ZBC
#define ZBC_STENCIL_CLEAR_FMT_INVAILD     0
#define ZBC_STENCIL_CLEAR_FMT_U8          1

struct zbc_s_table {
	u32 stencil;
	u32 format;
	u32 ref_cnt;
};

struct gk20a;
struct zbc_entry;
struct zbc_query_params;
struct channel_ctx_gk20a;

enum {
	VOLTA_CHANNEL_GPFIFO_A  = 0xC36F,
	VOLTA_A                 = 0xC397,
	VOLTA_COMPUTE_A         = 0xC3C0,
	VOLTA_DMA_COPY_A        = 0xC3B5,
};

#define NVC397_SET_SHADER_EXCEPTIONS		0x1528
#define NVC397_SET_CIRCULAR_BUFFER_SIZE 	0x1280
#define NVC397_SET_ALPHA_CIRCULAR_BUFFER_SIZE 	0x02dc
#define NVC397_SET_GO_IDLE_TIMEOUT 		0x022c
#define NVC397_SET_TEX_IN_DBG			0x10bc
#define NVC397_SET_SKEDCHECK			0x10c0
#define NVC397_SET_BES_CROP_DEBUG3		0x10c4

#define NVC397_SET_TEX_IN_DBG_TSL1_RVCH_INVALIDATE		0x1
#define NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_LD	0x2
#define NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_ST	0x4

#define NVC397_SET_SKEDCHECK_18_MASK				0x3
#define NVC397_SET_SKEDCHECK_18_DEFAULT				0x0
#define NVC397_SET_SKEDCHECK_18_DISABLE				0x1
#define NVC397_SET_SKEDCHECK_18_ENABLE				0x2

#define NVC3C0_SET_SKEDCHECK			0x23c

#define NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE 0

int gr_gv11b_alloc_buffer(struct vm_gk20a *vm, size_t size,
                        struct nvgpu_mem *mem);
/*zcull*/
void gr_gv11b_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
					u32 *zcull_map_tiles);
void gr_gv11b_create_sysfs(struct device *dev);

bool gr_gv11b_is_valid_class(struct gk20a *g, u32 class_num);
bool gr_gv11b_is_valid_gfx_class(struct gk20a *g, u32 class_num);
bool gr_gv11b_is_valid_compute_class(struct gk20a *g, u32 class_num);
void gr_gv11b_enable_hww_exceptions(struct gk20a *g);
void gr_gv11b_enable_exceptions(struct gk20a *g);
int gr_gv11b_handle_tpc_sm_ecc_exception(struct gk20a *g,
		u32 gpc, u32 tpc,
		bool *post_event, struct channel_gk20a *fault_ch,
		u32 *hww_global_esr);
int gr_gv11b_handle_gcc_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr);
int gr_gv11b_handle_gpc_gpcmmu_exception(struct gk20a *g, u32 gpc,
							u32 gpc_exception);
int gr_gv11b_handle_gpc_gpccs_exception(struct gk20a *g, u32 gpc,
							u32 gpc_exception);
void gr_gv11b_enable_gpc_exceptions(struct gk20a *g);
int gr_gv11b_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event);
int gr_gv11b_zbc_s_query_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params);
bool gr_gv11b_add_zbc_type_s(struct gk20a *g, struct gr_gk20a *gr,
		     struct zbc_entry *zbc_val, int *ret_val);
int gr_gv11b_add_zbc_stencil(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *stencil_val, u32 index);
int gr_gv11b_load_stencil_default_tbl(struct gk20a *g,
		 struct gr_gk20a *gr);
int gr_gv11b_load_stencil_tbl(struct gk20a *g, struct gr_gk20a *gr);
u32 gr_gv11b_pagepool_default_size(struct gk20a *g);
int gr_gv11b_calc_global_ctx_buffer_size(struct gk20a *g);
int gr_gv11b_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data);
void gr_gv11b_bundle_cb_defaults(struct gk20a *g);
void gr_gv11b_cb_size_default(struct gk20a *g);
void gr_gv11b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data);
void gr_gv11b_set_circular_buffer_size(struct gk20a *g, u32 data);
int gr_gv11b_dump_gr_status_regs(struct gk20a *g,
			   struct gk20a_debug_output *o);
int gr_gv11b_wait_empty(struct gk20a *g, unsigned long duration_ms,
		       u32 expect_delay);
void gr_gv11b_commit_global_attrib_cb(struct gk20a *g,
					     struct channel_ctx_gk20a *ch_ctx,
					     u64 addr, bool patch);
void gr_gv11b_init_cyclestats(struct gk20a *g);
void gr_gv11b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index);
void gr_gv11b_get_access_map(struct gk20a *g,
				   u32 **whitelist, int *num_entries);
int gr_gv11b_pre_process_sm_exception(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm, u32 global_esr, u32 warp_esr,
		bool sm_debugger_attached, struct channel_gk20a *fault_ch,
		bool *early_exit, bool *ignore_debugger);
int gr_gv11b_handle_fecs_error(struct gk20a *g,
				struct channel_gk20a *__ch,
				struct gr_gk20a_isr_data *isr_data);
int gr_gv11b_setup_rop_mapping(struct gk20a *g, struct gr_gk20a *gr);
int gr_gv11b_init_sw_veid_bundle(struct gk20a *g);
void gr_gv11b_detect_sm_arch(struct gk20a *g);
void gr_gv11b_init_sm_id_table(struct gk20a *g);
void gr_gv11b_program_sm_id_numbering(struct gk20a *g,
					u32 gpc, u32 tpc, u32 smid);
int gr_gv11b_load_smid_config(struct gk20a *g);
int gr_gv11b_commit_inst(struct channel_gk20a *c, u64 gpu_va);
int gr_gv11b_commit_global_timeslice(struct gk20a *g,
					struct channel_gk20a *c, bool patch);
void gv11b_restore_context_header(struct gk20a *g,
					struct nvgpu_mem *ctxheader);
void gr_gv11b_write_zcull_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va);
void gr_gv11b_write_pm_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va);
void gr_gv11b_init_elcg_mode(struct gk20a *g, u32 mode, u32 engine);
void gr_gv11b_load_tpc_mask(struct gk20a *g);
void gr_gv11b_set_preemption_buffer_va(struct gk20a *g,
			struct nvgpu_mem *mem, u64 gpu_va);
int gr_gv11b_init_fs_state(struct gk20a *g);
void gv11b_gr_get_esr_sm_sel(struct gk20a *g, u32 gpc, u32 tpc,
				u32 *esr_sm_sel);
int gv11b_gr_sm_trigger_suspend(struct gk20a *g);
void gv11b_gr_bpt_reg_info(struct gk20a *g, struct warpstate *w_state);
int gv11b_gr_update_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id,
		struct nvgpu_dbg_gpu_sm_error_state_record *sm_error_state);
int gv11b_gr_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable);
int gv11b_gr_record_sm_error_state(struct gk20a *g, u32 gpc, u32 tpc);
void gv11b_gr_set_hww_esr_report_mask(struct gk20a *g);
bool gv11b_gr_sm_debugger_attached(struct gk20a *g);
void gv11b_gr_suspend_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors);
void gv11b_gr_suspend_all_sms(struct gk20a *g,
		u32 global_esr_mask, bool check_errors);
void gv11b_gr_resume_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm);
void gv11b_gr_resume_all_sms(struct gk20a *g);
int gv11b_gr_resume_from_pause(struct gk20a *g);
u32 gv11b_gr_get_sm_hww_warp_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm);
u32 gv11b_gr_get_sm_hww_global_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm);
u32 gv11b_gr_get_sm_no_lock_down_hww_global_esr_mask(struct gk20a *g);
int gv11b_gr_wait_for_sm_lock_down(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors);
int gv11b_gr_lock_down_sm(struct gk20a *g,
			 u32 gpc, u32 tpc, u32 sm, u32 global_esr_mask,
			 bool check_errors);
void gv11b_gr_clear_sm_hww(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
				u32 global_esr);
int gr_gv11b_handle_tpc_mpc_exception(struct gk20a *g,
		u32 gpc, u32 tpc, bool *post_event);
void gv11b_gr_init_ovr_sm_dsm_perf(void);
void gv11b_gr_init_sm_dsm_reg_info(void);
void gv11b_gr_get_sm_dsm_perf_regs(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride);
void gv11b_gr_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_ctrl_regs,
					       u32 **sm_dsm_perf_ctrl_regs,
					       u32 *ctrl_register_stride);
void gv11b_gr_get_ovr_perf_regs(struct gk20a *g, u32 *num_ovr_perf_regs,
					       u32 **ovr_perf_regs);
void gv11b_gr_access_smpc_reg(struct gk20a *g, u32 quad, u32 offset);
bool gv11b_gr_pri_is_egpc_addr(struct gk20a *g, u32 addr);
bool gv11b_gr_pri_is_etpc_addr(struct gk20a *g, u32 addr);
void gv11b_gr_get_egpc_etpc_num(struct gk20a *g, u32 addr,
			u32 *egpc_num, u32 *etpc_num);
int gv11b_gr_decode_egpc_addr(struct gk20a *g, u32 addr, int *addr_type,
	u32 *gpc_num, u32 *tpc_num, u32 *broadcast_flags);
void gv11b_gr_egpc_etpc_priv_addr_table(struct gk20a *g, u32 addr,
		 u32 gpc, u32 broadcast_flags, u32 *priv_addr_table, u32 *t);
u32 gv11b_gr_get_egpc_base(struct gk20a *g);
void gr_gv11b_init_gpc_mmu(struct gk20a *g);

#endif
