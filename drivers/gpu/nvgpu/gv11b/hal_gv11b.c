/*
 * GV11B Tegra HAL interface
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

#include <linux/types.h>
#include <linux/printk.h>

#include <linux/types.h>
#include <linux/tegra_gpu_t19x.h>

#include "gk20a/gk20a.h"
#include "gk20a/fifo_gk20a.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "gk20a/fecs_trace_gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "gk20a/mc_gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/bus_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "gk20a/fb_gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "gk20a/gr_gk20a.h"

#include "gm20b/ltc_gm20b.h"
#include "gm20b/gr_gm20b.h"
#include "gm20b/fb_gm20b.h"
#include "gm20b/fifo_gm20b.h"
#include "gm20b/mm_gm20b.h"
#include "gm20b/acr_gm20b.h"
#include "gm20b/pmu_gm20b.h"

#include "gp10b/ltc_gp10b.h"
#include "gp10b/therm_gp10b.h"
#include "gp10b/mc_gp10b.h"
#include "gp10b/ce_gp10b.h"
#include "gp10b/priv_ring_gp10b.h"
#include "gp10b/fifo_gp10b.h"
#include "gp10b/fecs_trace_gp10b.h"
#include "gp10b/fb_gp10b.h"
#include "gp10b/mm_gp10b.h"
#include "gp10b/pmu_gp10b.h"
#include "gp10b/gr_gp10b.h"

#include "gp106/pmu_gp106.h"
#include "gp106/acr_gp106.h"

#include "hal_gv11b.h"
#include "gr_gv11b.h"
#include "mc_gv11b.h"
#include "ltc_gv11b.h"
#include "gv11b.h"
#include "ce_gv11b.h"
#include "gr_ctx_gv11b.h"
#include "mm_gv11b.h"
#include "pmu_gv11b.h"
#include "acr_gv11b.h"
#include "fb_gv11b.h"
#include "fifo_gv11b.h"
#include "gv11b_gating_reglist.h"
#include "regops_gv11b.h"
#include "subctx_gv11b.h"

#include <nvgpu/bus.h>
#include <nvgpu/debug.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gv11b/hw_proj_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fuse_gv11b.h>

static int gv11b_get_litter_value(struct gk20a *g, int value)
{
	int ret = EINVAL;
	switch (value) {
	case GPU_LIT_NUM_GPCS:
		ret = proj_scal_litter_num_gpcs_v();
		break;
	case GPU_LIT_NUM_PES_PER_GPC:
		ret = proj_scal_litter_num_pes_per_gpc_v();
		break;
	case GPU_LIT_NUM_ZCULL_BANKS:
		ret = proj_scal_litter_num_zcull_banks_v();
		break;
	case GPU_LIT_NUM_TPC_PER_GPC:
		ret = proj_scal_litter_num_tpc_per_gpc_v();
		break;
	case GPU_LIT_NUM_SM_PER_TPC:
		ret = proj_scal_litter_num_sm_per_tpc_v();
		break;
	case GPU_LIT_NUM_FBPS:
		ret = proj_scal_litter_num_fbps_v();
		break;
	case GPU_LIT_GPC_BASE:
		ret = proj_gpc_base_v();
		break;
	case GPU_LIT_GPC_STRIDE:
		ret = proj_gpc_stride_v();
		break;
	case GPU_LIT_GPC_SHARED_BASE:
		ret = proj_gpc_shared_base_v();
		break;
	case GPU_LIT_TPC_IN_GPC_BASE:
		ret = proj_tpc_in_gpc_base_v();
		break;
	case GPU_LIT_TPC_IN_GPC_STRIDE:
		ret = proj_tpc_in_gpc_stride_v();
		break;
	case GPU_LIT_TPC_IN_GPC_SHARED_BASE:
		ret = proj_tpc_in_gpc_shared_base_v();
		break;
	case GPU_LIT_PPC_IN_GPC_BASE:
		ret = proj_ppc_in_gpc_base_v();
		break;
	case GPU_LIT_PPC_IN_GPC_SHARED_BASE:
		ret = proj_ppc_in_gpc_shared_base_v();
		break;
	case GPU_LIT_PPC_IN_GPC_STRIDE:
		ret = proj_ppc_in_gpc_stride_v();
		break;
	case GPU_LIT_ROP_BASE:
		ret = proj_rop_base_v();
		break;
	case GPU_LIT_ROP_STRIDE:
		ret = proj_rop_stride_v();
		break;
	case GPU_LIT_ROP_SHARED_BASE:
		ret = proj_rop_shared_base_v();
		break;
	case GPU_LIT_HOST_NUM_ENGINES:
		ret = proj_host_num_engines_v();
		break;
	case GPU_LIT_HOST_NUM_PBDMA:
		ret = proj_host_num_pbdma_v();
		break;
	case GPU_LIT_LTC_STRIDE:
		ret = proj_ltc_stride_v();
		break;
	case GPU_LIT_LTS_STRIDE:
		ret = proj_lts_stride_v();
		break;
	case GPU_LIT_SM_PRI_STRIDE:
		ret = proj_sm_stride_v();
		break;
	/* Even though GV11B doesn't have an FBPA unit, the HW reports one,
	 * and the microcode as a result leaves space in the context buffer
	 * for one, so make sure SW accounts for this also.
	 */
	case GPU_LIT_NUM_FBPAS:
		ret = proj_scal_litter_num_fbpas_v();
		break;
	/* Hardcode FBPA values other than NUM_FBPAS to 0. */
	case GPU_LIT_FBPA_STRIDE:
	case GPU_LIT_FBPA_BASE:
	case GPU_LIT_FBPA_SHARED_BASE:
		ret = 0;
		break;

	default:
		nvgpu_err(g, "Missing definition %d", value);
		BUG();
		break;
	}

	return ret;
}

static const struct gpu_ops gv11b_ops = {
	.ltc = {
		.determine_L2_size_bytes = gp10b_determine_L2_size_bytes,
		.set_zbc_s_entry = gv11b_ltc_set_zbc_stencil_entry,
		.set_zbc_color_entry = gm20b_ltc_set_zbc_color_entry,
		.set_zbc_depth_entry = gm20b_ltc_set_zbc_depth_entry,
		.init_cbc = NULL,
		.init_fs_state = gv11b_ltc_init_fs_state,
		.init_comptags = gp10b_ltc_init_comptags,
		.cbc_ctrl = gm20b_ltc_cbc_ctrl,
		.isr = gv11b_ltc_isr,
		.cbc_fix_config = gv11b_ltc_cbc_fix_config,
		.flush = gm20b_flush_ltc,
		.set_enabled = gp10b_ltc_set_enabled,
	},
	.ce2 = {
		.isr_stall = gv11b_ce_isr,
		.isr_nonstall = gp10b_ce_nonstall_isr,
		.get_num_pce = gv11b_ce_get_num_pce,
	},
	.gr = {
		.init_gpc_mmu = gr_gv11b_init_gpc_mmu,
		.bundle_cb_defaults = gr_gv11b_bundle_cb_defaults,
		.cb_size_default = gr_gv11b_cb_size_default,
		.calc_global_ctx_buffer_size =
			gr_gv11b_calc_global_ctx_buffer_size,
		.commit_global_attrib_cb = gr_gv11b_commit_global_attrib_cb,
		.commit_global_bundle_cb = gr_gp10b_commit_global_bundle_cb,
		.commit_global_cb_manager = gr_gp10b_commit_global_cb_manager,
		.commit_global_pagepool = gr_gp10b_commit_global_pagepool,
		.handle_sw_method = gr_gv11b_handle_sw_method,
		.set_alpha_circular_buffer_size =
			gr_gv11b_set_alpha_circular_buffer_size,
		.set_circular_buffer_size = gr_gv11b_set_circular_buffer_size,
		.enable_hww_exceptions = gr_gv11b_enable_hww_exceptions,
		.is_valid_class = gr_gv11b_is_valid_class,
		.is_valid_gfx_class = gr_gv11b_is_valid_gfx_class,
		.is_valid_compute_class = gr_gv11b_is_valid_compute_class,
		.get_sm_dsm_perf_regs = gv11b_gr_get_sm_dsm_perf_regs,
		.get_sm_dsm_perf_ctrl_regs = gv11b_gr_get_sm_dsm_perf_ctrl_regs,
		.init_fs_state = gr_gv11b_init_fs_state,
		.set_hww_esr_report_mask = gv11b_gr_set_hww_esr_report_mask,
		.falcon_load_ucode = gr_gm20b_load_ctxsw_ucode_segments,
		.load_ctxsw_ucode = gr_gk20a_load_ctxsw_ucode,
		.set_gpc_tpc_mask = gr_gv11b_set_gpc_tpc_mask,
		.get_gpc_tpc_mask = gr_gm20b_get_gpc_tpc_mask,
		.free_channel_ctx = gk20a_free_channel_ctx,
		.alloc_obj_ctx = gk20a_alloc_obj_ctx,
		.bind_ctxsw_zcull = gr_gk20a_bind_ctxsw_zcull,
		.get_zcull_info = gr_gk20a_get_zcull_info,
		.is_tpc_addr = gr_gm20b_is_tpc_addr,
		.get_tpc_num = gr_gm20b_get_tpc_num,
		.detect_sm_arch = gr_gv11b_detect_sm_arch,
		.add_zbc_color = gr_gp10b_add_zbc_color,
		.add_zbc_depth = gr_gp10b_add_zbc_depth,
		.zbc_set_table = gk20a_gr_zbc_set_table,
		.zbc_query_table = gr_gk20a_query_zbc,
		.pmu_save_zbc = gk20a_pmu_save_zbc,
		.add_zbc = gr_gk20a_add_zbc,
		.pagepool_default_size = gr_gv11b_pagepool_default_size,
		.init_ctx_state = gr_gp10b_init_ctx_state,
		.alloc_gr_ctx = gr_gp10b_alloc_gr_ctx,
		.free_gr_ctx = gr_gp10b_free_gr_ctx,
		.update_ctxsw_preemption_mode =
			gr_gp10b_update_ctxsw_preemption_mode,
		.dump_gr_regs = gr_gv11b_dump_gr_status_regs,
		.update_pc_sampling = gr_gm20b_update_pc_sampling,
		.get_fbp_en_mask = gr_gm20b_get_fbp_en_mask,
		.get_max_ltc_per_fbp = gr_gm20b_get_max_ltc_per_fbp,
		.get_max_lts_per_ltc = gr_gm20b_get_max_lts_per_ltc,
		.get_rop_l2_en_mask = gr_gm20b_rop_l2_en_mask,
		.get_max_fbps_count = gr_gm20b_get_max_fbps_count,
		.init_sm_dsm_reg_info = gv11b_gr_init_sm_dsm_reg_info,
		.wait_empty = gr_gv11b_wait_empty,
		.init_cyclestats = gr_gv11b_init_cyclestats,
		.set_sm_debug_mode = gv11b_gr_set_sm_debug_mode,
		.enable_cde_in_fecs = gr_gm20b_enable_cde_in_fecs,
		.bpt_reg_info = gv11b_gr_bpt_reg_info,
		.get_access_map = gr_gv11b_get_access_map,
		.handle_fecs_error = gr_gv11b_handle_fecs_error,
		.handle_sm_exception = gr_gk20a_handle_sm_exception,
		.handle_tex_exception = gr_gv11b_handle_tex_exception,
		.enable_gpc_exceptions = gr_gv11b_enable_gpc_exceptions,
		.enable_exceptions = gr_gv11b_enable_exceptions,
		.get_lrf_tex_ltc_dram_override = get_ecc_override_val,
		.update_smpc_ctxsw_mode = gr_gk20a_update_smpc_ctxsw_mode,
		.update_hwpm_ctxsw_mode = gr_gk20a_update_hwpm_ctxsw_mode,
		.record_sm_error_state = gv11b_gr_record_sm_error_state,
		.update_sm_error_state = gv11b_gr_update_sm_error_state,
		.clear_sm_error_state = gm20b_gr_clear_sm_error_state,
		.suspend_contexts = gr_gp10b_suspend_contexts,
		.resume_contexts = gr_gk20a_resume_contexts,
		.get_preemption_mode_flags = gr_gp10b_get_preemption_mode_flags,
		.fuse_override = gp10b_gr_fuse_override,
		.init_sm_id_table = gr_gv11b_init_sm_id_table,
		.load_smid_config = gr_gv11b_load_smid_config,
		.program_sm_id_numbering = gr_gv11b_program_sm_id_numbering,
		.is_ltcs_ltss_addr = gr_gm20b_is_ltcs_ltss_addr,
		.is_ltcn_ltss_addr = gr_gm20b_is_ltcn_ltss_addr,
		.split_lts_broadcast_addr = gr_gm20b_split_lts_broadcast_addr,
		.split_ltc_broadcast_addr = gr_gm20b_split_ltc_broadcast_addr,
		.setup_rop_mapping = gr_gv11b_setup_rop_mapping,
		.program_zcull_mapping = gr_gv11b_program_zcull_mapping,
		.commit_global_timeslice = gr_gv11b_commit_global_timeslice,
		.commit_inst = gr_gv11b_commit_inst,
		.write_zcull_ptr = gr_gv11b_write_zcull_ptr,
		.write_pm_ptr = gr_gv11b_write_pm_ptr,
		.init_elcg_mode = gr_gv11b_init_elcg_mode,
		.load_tpc_mask = gr_gv11b_load_tpc_mask,
		.inval_icache = gr_gk20a_inval_icache,
		.trigger_suspend = gv11b_gr_sm_trigger_suspend,
		.wait_for_pause = gr_gk20a_wait_for_pause,
		.resume_from_pause = gv11b_gr_resume_from_pause,
		.clear_sm_errors = gr_gk20a_clear_sm_errors,
		.tpc_enabled_exceptions = gr_gk20a_tpc_enabled_exceptions,
		.get_esr_sm_sel = gv11b_gr_get_esr_sm_sel,
		.sm_debugger_attached = gv11b_gr_sm_debugger_attached,
		.suspend_single_sm = gv11b_gr_suspend_single_sm,
		.suspend_all_sms = gv11b_gr_suspend_all_sms,
		.resume_single_sm = gv11b_gr_resume_single_sm,
		.resume_all_sms = gv11b_gr_resume_all_sms,
		.get_sm_hww_warp_esr = gv11b_gr_get_sm_hww_warp_esr,
		.get_sm_hww_global_esr = gv11b_gr_get_sm_hww_global_esr,
		.get_sm_no_lock_down_hww_global_esr_mask =
			gv11b_gr_get_sm_no_lock_down_hww_global_esr_mask,
		.lock_down_sm = gv11b_gr_lock_down_sm,
		.wait_for_sm_lock_down = gv11b_gr_wait_for_sm_lock_down,
		.clear_sm_hww = gv11b_gr_clear_sm_hww,
		.init_ovr_sm_dsm_perf =  gv11b_gr_init_ovr_sm_dsm_perf,
		.get_ovr_perf_regs = gv11b_gr_get_ovr_perf_regs,
		.disable_rd_coalesce = gm20a_gr_disable_rd_coalesce,
		.set_boosted_ctx = gr_gp10b_set_boosted_ctx,
		.set_preemption_mode = gr_gp10b_set_preemption_mode,
		.set_czf_bypass = NULL,
		.pre_process_sm_exception = gr_gv11b_pre_process_sm_exception,
		.set_preemption_buffer_va = gr_gv11b_set_preemption_buffer_va,
		.init_preemption_state = NULL,
		.update_boosted_ctx = gr_gp10b_update_boosted_ctx,
		.set_bes_crop_debug3 = gr_gp10b_set_bes_crop_debug3,
		.create_gr_sysfs = gr_gv11b_create_sysfs,
		.set_ctxsw_preemption_mode = gr_gp10b_set_ctxsw_preemption_mode,
		.is_etpc_addr = gv11b_gr_pri_is_etpc_addr,
		.egpc_etpc_priv_addr_table = gv11b_gr_egpc_etpc_priv_addr_table,
		.handle_tpc_mpc_exception = gr_gv11b_handle_tpc_mpc_exception,
		.zbc_s_query_table = gr_gv11b_zbc_s_query_table,
		.load_zbc_s_default_tbl = gr_gv11b_load_stencil_default_tbl,
		.restore_context_header = gv11b_restore_context_header,
		.handle_gpc_gpcmmu_exception =
			gr_gv11b_handle_gpc_gpcmmu_exception,
		.add_zbc_type_s = gr_gv11b_add_zbc_type_s,
		.get_egpc_base = gv11b_gr_get_egpc_base,
		.get_egpc_etpc_num = gv11b_gr_get_egpc_etpc_num,
		.handle_gpc_gpccs_exception =
			gr_gv11b_handle_gpc_gpccs_exception,
		.load_zbc_s_tbl = gr_gv11b_load_stencil_tbl,
		.access_smpc_reg = gv11b_gr_access_smpc_reg,
		.is_egpc_addr = gv11b_gr_pri_is_egpc_addr,
		.add_zbc_s = gr_gv11b_add_zbc_stencil,
		.handle_gcc_exception = gr_gv11b_handle_gcc_exception,
		.init_sw_veid_bundle = gr_gv11b_init_sw_veid_bundle,
		.handle_tpc_sm_ecc_exception =
			gr_gv11b_handle_tpc_sm_ecc_exception,
		.decode_egpc_addr = gv11b_gr_decode_egpc_addr,
	},
	.fb = {
		.reset = gv11b_fb_reset,
		.init_hw = gk20a_fb_init_hw,
		.init_fs_state = gv11b_fb_init_fs_state,
		.init_cbc = gv11b_fb_init_cbc,
		.set_mmu_page_size = gm20b_fb_set_mmu_page_size,
		.set_use_full_comp_tag_line =
			gm20b_fb_set_use_full_comp_tag_line,
		.compression_page_size = gp10b_fb_compression_page_size,
		.compressible_page_size = gp10b_fb_compressible_page_size,
		.vpr_info_fetch = gm20b_fb_vpr_info_fetch,
		.dump_vpr_wpr_info = gm20b_fb_dump_vpr_wpr_info,
		.is_debug_mode_enabled = gm20b_fb_debug_mode_enabled,
		.set_debug_mode = gm20b_fb_set_debug_mode,
		.tlb_invalidate = gk20a_fb_tlb_invalidate,
		.hub_isr = gv11b_fb_hub_isr,
	},
	.clock_gating = {
		.slcg_bus_load_gating_prod =
			gv11b_slcg_bus_load_gating_prod,
		.slcg_ce2_load_gating_prod =
			gv11b_slcg_ce2_load_gating_prod,
		.slcg_chiplet_load_gating_prod =
			gv11b_slcg_chiplet_load_gating_prod,
		.slcg_ctxsw_firmware_load_gating_prod =
			gv11b_slcg_ctxsw_firmware_load_gating_prod,
		.slcg_fb_load_gating_prod =
			gv11b_slcg_fb_load_gating_prod,
		.slcg_fifo_load_gating_prod =
			gv11b_slcg_fifo_load_gating_prod,
		.slcg_gr_load_gating_prod =
			gr_gv11b_slcg_gr_load_gating_prod,
		.slcg_ltc_load_gating_prod =
			ltc_gv11b_slcg_ltc_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gv11b_slcg_perf_load_gating_prod,
		.slcg_priring_load_gating_prod =
			gv11b_slcg_priring_load_gating_prod,
		.slcg_pmu_load_gating_prod =
			gv11b_slcg_pmu_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gv11b_slcg_therm_load_gating_prod,
		.slcg_xbar_load_gating_prod =
			gv11b_slcg_xbar_load_gating_prod,
		.blcg_bus_load_gating_prod =
			gv11b_blcg_bus_load_gating_prod,
		.blcg_ce_load_gating_prod =
			gv11b_blcg_ce_load_gating_prod,
		.blcg_ctxsw_firmware_load_gating_prod =
			gv11b_blcg_ctxsw_firmware_load_gating_prod,
		.blcg_fb_load_gating_prod =
			gv11b_blcg_fb_load_gating_prod,
		.blcg_fifo_load_gating_prod =
			gv11b_blcg_fifo_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gv11b_blcg_gr_load_gating_prod,
		.blcg_ltc_load_gating_prod =
			gv11b_blcg_ltc_load_gating_prod,
		.blcg_pwr_csb_load_gating_prod =
			gv11b_blcg_pwr_csb_load_gating_prod,
		.blcg_pmu_load_gating_prod =
			gv11b_blcg_pmu_load_gating_prod,
		.blcg_xbar_load_gating_prod =
			gv11b_blcg_xbar_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gv11b_pg_gr_load_gating_prod,
	},
	.fifo = {
		.init_fifo_setup_hw = gv11b_init_fifo_setup_hw,
		.bind_channel = channel_gm20b_bind,
		.unbind_channel = channel_gv11b_unbind,
		.disable_channel = gk20a_fifo_disable_channel,
		.enable_channel = gk20a_fifo_enable_channel,
		.alloc_inst = gk20a_fifo_alloc_inst,
		.free_inst = gk20a_fifo_free_inst,
		.setup_ramfc = channel_gv11b_setup_ramfc,
		.channel_set_priority = gk20a_fifo_set_priority,
		.channel_set_timeslice = gk20a_fifo_set_timeslice,
		.default_timeslice_us = gk20a_fifo_default_timeslice_us,
		.setup_userd = gk20a_fifo_setup_userd,
		.userd_gp_get = gv11b_userd_gp_get,
		.userd_gp_put = gv11b_userd_gp_put,
		.userd_pb_get = gv11b_userd_pb_get,
		.pbdma_acquire_val = gk20a_fifo_pbdma_acquire_val,
		.preempt_channel = gv11b_fifo_preempt_channel,
		.preempt_tsg = gv11b_fifo_preempt_tsg,
		.enable_tsg = gv11b_fifo_enable_tsg,
		.disable_tsg = gk20a_disable_tsg,
		.tsg_verify_status_ctx_reload = gm20b_fifo_tsg_verify_status_ctx_reload,
		.tsg_verify_status_faulted = gv11b_fifo_tsg_verify_status_faulted,
		.update_runlist = gk20a_fifo_update_runlist,
		.trigger_mmu_fault = NULL,
		.get_mmu_fault_info = NULL,
		.wait_engine_idle = gk20a_fifo_wait_engine_idle,
		.get_num_fifos = gv11b_fifo_get_num_fifos,
		.get_pbdma_signature = gp10b_fifo_get_pbdma_signature,
		.set_runlist_interleave = gk20a_fifo_set_runlist_interleave,
		.tsg_set_timeslice = gk20a_fifo_tsg_set_timeslice,
		.force_reset_ch = gk20a_fifo_force_reset_ch,
		.engine_enum_from_type = gp10b_fifo_engine_enum_from_type,
		.device_info_data_parse = gp10b_device_info_data_parse,
		.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v,
		.init_engine_info = gk20a_fifo_init_engine_info,
		.runlist_entry_size = ram_rl_entry_size_v,
		.get_tsg_runlist_entry = gv11b_get_tsg_runlist_entry,
		.get_ch_runlist_entry = gv11b_get_ch_runlist_entry,
		.is_fault_engine_subid_gpc = gv11b_is_fault_engine_subid_gpc,
		.dump_pbdma_status = gk20a_dump_pbdma_status,
		.dump_eng_status = gv11b_dump_eng_status,
		.dump_channel_status_ramfc = gv11b_dump_channel_status_ramfc,
		.intr_0_error_mask = gv11b_fifo_intr_0_error_mask,
		.is_preempt_pending = gv11b_fifo_is_preempt_pending,
		.init_pbdma_intr_descs = gv11b_fifo_init_pbdma_intr_descs,
		.reset_enable_hw = gv11b_init_fifo_reset_enable_hw,
		.teardown_ch_tsg = gv11b_fifo_teardown_ch_tsg,
		.handle_sched_error = gv11b_fifo_handle_sched_error,
		.handle_pbdma_intr_0 = gv11b_fifo_handle_pbdma_intr_0,
		.handle_pbdma_intr_1 = gv11b_fifo_handle_pbdma_intr_1,
		.init_eng_method_buffers = gv11b_fifo_init_eng_method_buffers,
		.deinit_eng_method_buffers =
			gv11b_fifo_deinit_eng_method_buffers,
		.tsg_bind_channel = gk20a_tsg_bind_channel,
		.tsg_unbind_channel = gk20a_tsg_unbind_channel,
#ifdef CONFIG_TEGRA_GK20A_NVHOST
		.alloc_syncpt_buf = gv11b_fifo_alloc_syncpt_buf,
		.free_syncpt_buf = gv11b_fifo_free_syncpt_buf,
		.add_syncpt_wait_cmd = gv11b_fifo_add_syncpt_wait_cmd,
		.get_syncpt_wait_cmd_size = gv11b_fifo_get_syncpt_wait_cmd_size,
		.add_syncpt_incr_cmd = gv11b_fifo_add_syncpt_incr_cmd,
		.get_syncpt_incr_cmd_size = gv11b_fifo_get_syncpt_incr_cmd_size,
#endif
		.resetup_ramfc = NULL,
		.device_info_fault_id = top_device_info_data_fault_id_enum_v,
		.free_channel_ctx_header = gv11b_free_subctx_header,
		.preempt_ch_tsg = gv11b_fifo_preempt_ch_tsg,
		.handle_ctxsw_timeout = gv11b_fifo_handle_ctxsw_timeout,
	},
	.gr_ctx = {
		.get_netlist_name = gr_gv11b_get_netlist_name,
		.is_fw_defined = gr_gv11b_is_firmware_defined,
	},
#ifdef CONFIG_GK20A_CTXSW_TRACE
	.fecs_trace = {
		.alloc_user_buffer = gk20a_ctxsw_dev_ring_alloc,
		.free_user_buffer = gk20a_ctxsw_dev_ring_free,
		.mmap_user_buffer = gk20a_ctxsw_dev_mmap_buffer,
		.init = gk20a_fecs_trace_init,
		.deinit = gk20a_fecs_trace_deinit,
		.enable = gk20a_fecs_trace_enable,
		.disable = gk20a_fecs_trace_disable,
		.is_enabled = gk20a_fecs_trace_is_enabled,
		.reset = gk20a_fecs_trace_reset,
		.flush = gp10b_fecs_trace_flush,
		.poll = gk20a_fecs_trace_poll,
		.bind_channel = gk20a_fecs_trace_bind_channel,
		.unbind_channel = gk20a_fecs_trace_unbind_channel,
		.max_entries = gk20a_gr_max_entries,
	},
#endif /* CONFIG_GK20A_CTXSW_TRACE */
	.mm = {
		.support_sparse = gm20b_mm_support_sparse,
		.gmmu_map = gk20a_locked_gmmu_map,
		.gmmu_unmap = gk20a_locked_gmmu_unmap,
		.vm_bind_channel = gk20a_vm_bind_channel,
		.fb_flush = gk20a_mm_fb_flush,
		.l2_invalidate = gk20a_mm_l2_invalidate,
		.l2_flush = gv11b_mm_l2_flush,
		.cbc_clean = gk20a_mm_cbc_clean,
		.set_big_page_size = gm20b_mm_set_big_page_size,
		.get_big_page_sizes = gm20b_mm_get_big_page_sizes,
		.get_default_big_page_size = gp10b_mm_get_default_big_page_size,
		.gpu_phys_addr = gv11b_gpu_phys_addr,
		.get_physical_addr_bits = gp10b_mm_get_physical_addr_bits,
		.get_mmu_levels = gp10b_mm_get_mmu_levels,
		.init_pdb = gp10b_mm_init_pdb,
		.init_mm_setup_hw = gv11b_init_mm_setup_hw,
		.is_bar1_supported = gv11b_mm_is_bar1_supported,
		.init_inst_block = gv11b_init_inst_block,
		.mmu_fault_pending = gv11b_mm_mmu_fault_pending,
		.init_bar2_vm = gb10b_init_bar2_vm,
		.init_bar2_mm_hw_setup = gv11b_init_bar2_mm_hw_setup,
		.remove_bar2_vm = gv11b_mm_remove_bar2_vm,
		.fault_info_mem_destroy = gv11b_mm_fault_info_mem_destroy,
	},
	.therm = {
		.init_therm_setup_hw = gp10b_init_therm_setup_hw,
		.elcg_init_idle_filters = gp10b_elcg_init_idle_filters,
	},
	.pmu = {
		.pmu_setup_elpg = gp10b_pmu_setup_elpg,
		.pmu_get_queue_head = pwr_pmu_queue_head_r,
		.pmu_get_queue_head_size = pwr_pmu_queue_head__size_1_v,
		.pmu_get_queue_tail = pwr_pmu_queue_tail_r,
		.pmu_get_queue_tail_size = pwr_pmu_queue_tail__size_1_v,
		.pmu_queue_head = gk20a_pmu_queue_head,
		.pmu_queue_tail = gk20a_pmu_queue_tail,
		.pmu_msgq_tail = gk20a_pmu_msgq_tail,
		.pmu_mutex_size = pwr_pmu_mutex__size_1_v,
		.pmu_mutex_acquire = gk20a_pmu_mutex_acquire,
		.pmu_mutex_release = gk20a_pmu_mutex_release,
		.write_dmatrfbase = gp10b_write_dmatrfbase,
		.pmu_elpg_statistics = gp106_pmu_elpg_statistics,
		.pmu_pg_init_param = gv11b_pg_gr_init,
		.pmu_pg_supported_engines_list = gk20a_pmu_pg_engines_list,
		.pmu_pg_engines_feature_list = gk20a_pmu_pg_feature_list,
		.dump_secure_fuses = pmu_dump_security_fuses_gp10b,
		.reset_engine = gp106_pmu_engine_reset,
		.is_engine_in_reset = gp106_pmu_is_engine_in_reset,
		.pmu_nsbootstrap = gv11b_pmu_bootstrap,
		.pmu_pg_set_sub_feature_mask = gv11b_pg_set_subfeature_mask,
		.is_pmu_supported = gv11b_is_pmu_supported,
	},
	.regops = {
		.get_global_whitelist_ranges =
			gv11b_get_global_whitelist_ranges,
		.get_global_whitelist_ranges_count =
			gv11b_get_global_whitelist_ranges_count,
		.get_context_whitelist_ranges =
			gv11b_get_context_whitelist_ranges,
		.get_context_whitelist_ranges_count =
			gv11b_get_context_whitelist_ranges_count,
		.get_runcontrol_whitelist = gv11b_get_runcontrol_whitelist,
		.get_runcontrol_whitelist_count =
			gv11b_get_runcontrol_whitelist_count,
		.get_runcontrol_whitelist_ranges =
			gv11b_get_runcontrol_whitelist_ranges,
		.get_runcontrol_whitelist_ranges_count =
			gv11b_get_runcontrol_whitelist_ranges_count,
		.get_qctl_whitelist = gv11b_get_qctl_whitelist,
		.get_qctl_whitelist_count = gv11b_get_qctl_whitelist_count,
		.get_qctl_whitelist_ranges = gv11b_get_qctl_whitelist_ranges,
		.get_qctl_whitelist_ranges_count =
			gv11b_get_qctl_whitelist_ranges_count,
		.apply_smpc_war = gv11b_apply_smpc_war,
	},
	.mc = {
		.intr_enable = mc_gv11b_intr_enable,
		.intr_unit_config = mc_gp10b_intr_unit_config,
		.isr_stall = mc_gp10b_isr_stall,
		.intr_stall = mc_gp10b_intr_stall,
		.intr_stall_pause = mc_gp10b_intr_stall_pause,
		.intr_stall_resume = mc_gp10b_intr_stall_resume,
		.intr_nonstall = mc_gp10b_intr_nonstall,
		.intr_nonstall_pause = mc_gp10b_intr_nonstall_pause,
		.intr_nonstall_resume = mc_gp10b_intr_nonstall_resume,
		.enable = gk20a_mc_enable,
		.disable = gk20a_mc_disable,
		.reset = gk20a_mc_reset,
		.boot_0 = gk20a_mc_boot_0,
		.is_intr1_pending = mc_gp10b_is_intr1_pending,
		.is_intr_hub_pending = gv11b_mc_is_intr_hub_pending,
	},
	.debug = {
		.show_dump = gk20a_debug_show_dump,
	},
	.dbg_session_ops = {
		.exec_reg_ops = exec_regops_gk20a,
		.dbg_set_powergate = dbg_set_powergate,
		.check_and_set_global_reservation =
			nvgpu_check_and_set_global_reservation,
		.check_and_set_context_reservation =
			nvgpu_check_and_set_context_reservation,
		.release_profiler_reservation =
			nvgpu_release_profiler_reservation,
		.perfbuffer_enable = gk20a_perfbuf_enable_locked,
		.perfbuffer_disable = gk20a_perfbuf_disable_locked,
	},
	.bus = {
		.init_hw = gk20a_bus_init_hw,
		.isr = gk20a_bus_isr,
		.read_ptimer = gk20a_read_ptimer,
		.get_timestamps_zipper = nvgpu_get_timestamps_zipper,
		.bar1_bind = NULL,
	},
#if defined(CONFIG_GK20A_CYCLE_STATS)
	.css = {
		.enable_snapshot = css_hw_enable_snapshot,
		.disable_snapshot = css_hw_disable_snapshot,
		.check_data_available = css_hw_check_data_available,
		.set_handled_snapshots = css_hw_set_handled_snapshots,
		.allocate_perfmon_ids = css_gr_allocate_perfmon_ids,
		.release_perfmon_ids = css_gr_release_perfmon_ids,
	},
#endif
	.falcon = {
		.falcon_hal_sw_init = gk20a_falcon_hal_sw_init,
	},
	.priv_ring = {
		.isr = gp10b_priv_ring_isr,
	},
	.chip_init_gpu_characteristics = gv11b_init_gpu_characteristics,
	.get_litter_value = gv11b_get_litter_value,
};

int gv11b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;
	u32 val;
	bool priv_security;

	gops->ltc = gv11b_ops.ltc;
	gops->ce2 = gv11b_ops.ce2;
	gops->gr = gv11b_ops.gr;
	gops->fb = gv11b_ops.fb;
	gops->clock_gating = gv11b_ops.clock_gating;
	gops->fifo = gv11b_ops.fifo;
	gops->gr_ctx = gv11b_ops.gr_ctx;
	gops->mm = gv11b_ops.mm;
	gops->fecs_trace = gv11b_ops.fecs_trace;
	gops->therm = gv11b_ops.therm;
	gops->pmu = gv11b_ops.pmu;
	gops->regops = gv11b_ops.regops;
	gops->mc = gv11b_ops.mc;
	gops->debug = gv11b_ops.debug;
	gops->dbg_session_ops = gv11b_ops.dbg_session_ops;
	gops->bus = gv11b_ops.bus;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gops->css = gv11b_ops.css;
#endif
	gops->falcon = gv11b_ops.falcon;
	gops->priv_ring = gv11b_ops.priv_ring;

	/* Lone functions */
	gops->chip_init_gpu_characteristics =
		gv11b_ops.chip_init_gpu_characteristics;
	gops->get_litter_value = gv11b_ops.get_litter_value;

	val = gk20a_readl(g, fuse_opt_priv_sec_en_r());
	if (val) {
		priv_security = true;
		pr_err("priv security is enabled\n");
	} else {
		priv_security = false;
		pr_err("priv security is disabled\n");
	}
	__nvgpu_set_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP, false);
	__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, priv_security);
	__nvgpu_set_enabled(g, NVGPU_SEC_SECUREGPCCS, priv_security);

	/* priv security dependent ops */
	if (nvgpu_is_enabled(g, NVGPU_SEC_PRIVSECURITY)) {
		/* Add in ops from gm20b acr */
		gops->pmu.prepare_ucode = gp106_prepare_ucode_blob,
		gops->pmu.pmu_setup_hw_and_bootstrap = gv11b_bootstrap_hs_flcn,
		gops->pmu.get_wpr = gm20b_wpr_info,
		gops->pmu.alloc_blob_space = gm20b_alloc_blob_space,
		gops->pmu.pmu_populate_loader_cfg =
			gp106_pmu_populate_loader_cfg,
		gops->pmu.flcn_populate_bl_dmem_desc =
			gp106_flcn_populate_bl_dmem_desc,
		gops->pmu.falcon_wait_for_halt = pmu_wait_for_halt,
		gops->pmu.falcon_clear_halt_interrupt_status =
			clear_halt_interrupt_status,
		gops->pmu.init_falcon_setup_hw = gv11b_init_pmu_setup_hw1,

		gops->pmu.init_wpr_region = gm20b_pmu_init_acr;
		gops->pmu.load_lsfalcon_ucode = gp10b_load_falcon_ucode;
		gops->pmu.is_lazy_bootstrap = gv11b_is_lazy_bootstrap,
		gops->pmu.is_priv_load = gv11b_is_priv_load,

		gops->gr.load_ctxsw_ucode = gr_gm20b_load_ctxsw_ucode;
	} else {
		/* Inherit from gk20a */
		gops->pmu.prepare_ucode = nvgpu_pmu_prepare_ns_ucode_blob,
		gops->pmu.pmu_setup_hw_and_bootstrap = gk20a_init_pmu_setup_hw1,

		gops->pmu.load_lsfalcon_ucode = NULL;
		gops->pmu.init_wpr_region = NULL;
		gops->pmu.pmu_setup_hw_and_bootstrap = gp10b_init_pmu_setup_hw1;

		gops->gr.load_ctxsw_ucode = gr_gk20a_load_ctxsw_ucode;
	}

	__nvgpu_set_enabled(g, NVGPU_PMU_FECS_BOOTSTRAP_DONE, false);
	gv11b_init_uncompressed_kind_map();
	gv11b_init_kind_attr();
	g->bootstrap_owner = LSF_BOOTSTRAP_OWNER_DEFAULT;

	g->name = "gv11b";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = VOLTA_A;
	c->compute_class = VOLTA_COMPUTE_A;
	c->gpfifo_class = VOLTA_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = VOLTA_DMA_COPY_A;

	return 0;
}
