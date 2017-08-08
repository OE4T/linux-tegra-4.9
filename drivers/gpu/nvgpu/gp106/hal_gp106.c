/*
 * GP106 HAL interface
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

#include "gk20a/gk20a.h"
#include "gk20a/fifo_gk20a.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "gk20a/fecs_trace_gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "gk20a/bus_gk20a.h"
#include "gk20a/pramin_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "gk20a/mc_gk20a.h"
#include "gk20a/fb_gk20a.h"
#include "gk20a/pmu_gk20a.h"
#include "gk20a/gr_gk20a.h"

#include "gp10b/ltc_gp10b.h"
#include "gp10b/gr_gp10b.h"
#include "gp10b/fecs_trace_gp10b.h"
#include "gp10b/mc_gp10b.h"
#include "gp10b/mm_gp10b.h"
#include "gp10b/ce_gp10b.h"
#include "gp10b/regops_gp10b.h"
#include "gp10b/cde_gp10b.h"
#include "gp10b/priv_ring_gp10b.h"
#include "gp10b/fifo_gp10b.h"
#include "gp10b/fb_gp10b.h"
#include "gp10b/pmu_gp10b.h"
#include "gp10b/gr_gp10b.h"

#include "gp106/fifo_gp106.h"
#include "gp106/regops_gp106.h"

#include "gm20b/ltc_gm20b.h"
#include "gm20b/gr_gm20b.h"
#include "gm20b/fifo_gm20b.h"
#include "gm20b/mm_gm20b.h"
#include "gm20b/pmu_gm20b.h"
#include "gm20b/fb_gm20b.h"
#include "gm20b/acr_gm20b.h"
#include "gm20b/gr_gm20b.h"

#include "gp106/acr_gp106.h"
#include "gp106/sec2_gp106.h"
#include "gp106/clk_gp106.h"
#include "gp106/clk_arb_gp106.h"
#include "gp106/mclk_gp106.h"
#include "gm206/bios_gm206.h"
#include "gp106/therm_gp106.h"
#include "gp106/xve_gp106.h"
#include "gp106/fifo_gp106.h"
#include "gp106/clk_gp106.h"
#include "gp106/mm_gp106.h"
#include "gp106/pmu_gp106.h"
#include "gp106/gr_ctx_gp106.h"
#include "gp106/gr_gp106.h"
#include "gp106/fb_gp106.h"
#include "gp106/gp106_gating_reglist.h"
#include "gp106/flcn_gp106.h"

#include "hal_gp106.h"

#include <nvgpu/debug.h>
#include <nvgpu/bug.h>
#include <nvgpu/bus.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gp106/hw_proj_gp106.h>
#include <nvgpu/hw/gp106/hw_fifo_gp106.h>
#include <nvgpu/hw/gp106/hw_ram_gp106.h>
#include <nvgpu/hw/gp106/hw_top_gp106.h>
#include <nvgpu/hw/gp106/hw_pram_gp106.h>
#include <nvgpu/hw/gp106/hw_pwr_gp106.h>


static int gp106_get_litter_value(struct gk20a *g, int value)
{
	int ret = -EINVAL;

	switch (value) {
	case GPU_LIT_NUM_GPCS:
		ret = proj_scal_litter_num_gpcs_v();
		break;
	case GPU_LIT_NUM_PES_PER_GPC:
		ret = proj_scal_litter_num_pes_per_gpc_v();
		break;
	case GPU_LIT_NUM_SM_PER_TPC:
		ret = proj_scal_litter_num_sm_per_tpc_v();
		break;
	case GPU_LIT_NUM_ZCULL_BANKS:
		ret = proj_scal_litter_num_zcull_banks_v();
		break;
	case GPU_LIT_NUM_TPC_PER_GPC:
		ret = proj_scal_litter_num_tpc_per_gpc_v();
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
	case GPU_LIT_PPC_IN_GPC_STRIDE:
		ret = proj_ppc_in_gpc_stride_v();
		break;
	case GPU_LIT_PPC_IN_GPC_SHARED_BASE:
		ret = proj_ppc_in_gpc_shared_base_v();
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
	case GPU_LIT_NUM_FBPAS:
		ret = proj_scal_litter_num_fbpas_v();
		break;
	case GPU_LIT_FBPA_SHARED_BASE:
		ret = proj_fbpa_shared_base_v();
		break;
	case GPU_LIT_FBPA_BASE:
		ret = proj_fbpa_base_v();
		break;
	case GPU_LIT_FBPA_STRIDE:
		ret = proj_fbpa_stride_v();
		break;
	default:
		BUG();
		break;
	}

	return ret;
}

static int gp106_init_gpu_characteristics(struct gk20a *g)
{
	struct nvgpu_gpu_characteristics *gpu = &g->gpu_characteristics;

	int err;

	err = gk20a_init_gpu_characteristics(g);
	if (err)
		return err;

	gpu->flags |= NVGPU_GPU_FLAGS_SUPPORT_GET_VOLTAGE |
			NVGPU_GPU_FLAGS_SUPPORT_GET_CURRENT |
			NVGPU_GPU_FLAGS_SUPPORT_GET_POWER |
			NVGPU_GPU_FLAGS_SUPPORT_GET_TEMPERATURE |
			NVGPU_GPU_FLAGS_SUPPORT_DEVICE_EVENTS |
			NVGPU_GPU_FLAGS_SUPPORT_SET_THERM_ALERT_LIMIT;

	/* WAR for missing INA3221 on HW2.5 RevA */
	if (g->power_sensor_missing) {
		gpu->flags &= ~(NVGPU_GPU_FLAGS_SUPPORT_GET_VOLTAGE |
				NVGPU_GPU_FLAGS_SUPPORT_GET_CURRENT |
				NVGPU_GPU_FLAGS_SUPPORT_GET_POWER);
	}

	return 0;
}

static const struct gpu_ops gp106_ops = {
	.ltc = {
		.determine_L2_size_bytes = gp10b_determine_L2_size_bytes,
		.set_zbc_color_entry = gm20b_ltc_set_zbc_color_entry,
		.set_zbc_depth_entry = gm20b_ltc_set_zbc_depth_entry,
		.init_cbc = NULL,
		.init_fs_state = gm20b_ltc_init_fs_state,
		.init_comptags = gp10b_ltc_init_comptags,
		.cbc_ctrl = gm20b_ltc_cbc_ctrl,
		.isr = gp10b_ltc_isr,
		.cbc_fix_config = NULL,
		.flush = gm20b_flush_ltc,
		.set_enabled = gp10b_ltc_set_enabled,
	},
	.ce2 = {
		.isr_stall = gp10b_ce_isr,
		.isr_nonstall = gp10b_ce_nonstall_isr,
	},
	.gr = {
		.init_gpc_mmu = gr_gm20b_init_gpc_mmu,
		.bundle_cb_defaults = gr_gm20b_bundle_cb_defaults,
		.cb_size_default = gr_gp106_cb_size_default,
		.calc_global_ctx_buffer_size =
			gr_gp10b_calc_global_ctx_buffer_size,
		.commit_global_attrib_cb = gr_gp10b_commit_global_attrib_cb,
		.commit_global_bundle_cb = gr_gp10b_commit_global_bundle_cb,
		.commit_global_cb_manager = gr_gp10b_commit_global_cb_manager,
		.commit_global_pagepool = gr_gp10b_commit_global_pagepool,
		.handle_sw_method = gr_gp106_handle_sw_method,
		.set_alpha_circular_buffer_size =
			gr_gp10b_set_alpha_circular_buffer_size,
		.set_circular_buffer_size = gr_gp10b_set_circular_buffer_size,
		.enable_hww_exceptions = gr_gk20a_enable_hww_exceptions,
		.is_valid_class = gr_gp106_is_valid_class,
		.is_valid_gfx_class = gr_gp10b_is_valid_gfx_class,
		.is_valid_compute_class = gr_gp10b_is_valid_compute_class,
		.get_sm_dsm_perf_regs = gr_gm20b_get_sm_dsm_perf_regs,
		.get_sm_dsm_perf_ctrl_regs = gr_gm20b_get_sm_dsm_perf_ctrl_regs,
		.init_fs_state = gr_gp10b_init_fs_state,
		.set_hww_esr_report_mask = gr_gm20b_set_hww_esr_report_mask,
		.falcon_load_ucode = gr_gm20b_load_ctxsw_ucode_segments,
		.set_gpc_tpc_mask = gr_gp10b_set_gpc_tpc_mask,
		.get_gpc_tpc_mask = gr_gm20b_get_gpc_tpc_mask,
		.free_channel_ctx = gk20a_free_channel_ctx,
		.alloc_obj_ctx = gk20a_alloc_obj_ctx,
		.bind_ctxsw_zcull = gr_gk20a_bind_ctxsw_zcull,
		.get_zcull_info = gr_gk20a_get_zcull_info,
		.is_tpc_addr = gr_gm20b_is_tpc_addr,
		.get_tpc_num = gr_gm20b_get_tpc_num,
		.detect_sm_arch = gr_gm20b_detect_sm_arch,
		.add_zbc_color = gr_gp10b_add_zbc_color,
		.add_zbc_depth = gr_gp10b_add_zbc_depth,
		.zbc_set_table = gk20a_gr_zbc_set_table,
		.zbc_query_table = gr_gk20a_query_zbc,
		.pmu_save_zbc = gk20a_pmu_save_zbc,
		.add_zbc = gr_gk20a_add_zbc,
		.pagepool_default_size = gr_gp106_pagepool_default_size,
		.init_ctx_state = gr_gp10b_init_ctx_state,
		.alloc_gr_ctx = gr_gp10b_alloc_gr_ctx,
		.free_gr_ctx = gr_gp10b_free_gr_ctx,
		.update_ctxsw_preemption_mode =
			gr_gp10b_update_ctxsw_preemption_mode,
		.dump_gr_regs = gr_gp10b_dump_gr_status_regs,
		.update_pc_sampling = gr_gm20b_update_pc_sampling,
		.get_fbp_en_mask = gr_gm20b_get_fbp_en_mask,
		.get_max_ltc_per_fbp = gr_gm20b_get_max_ltc_per_fbp,
		.get_max_lts_per_ltc = gr_gm20b_get_max_lts_per_ltc,
		.get_rop_l2_en_mask = gr_gm20b_rop_l2_en_mask,
		.get_max_fbps_count = gr_gm20b_get_max_fbps_count,
		.init_sm_dsm_reg_info = gr_gm20b_init_sm_dsm_reg_info,
		.wait_empty = gr_gp10b_wait_empty,
		.init_cyclestats = gr_gp10b_init_cyclestats,
		.set_sm_debug_mode = gr_gk20a_set_sm_debug_mode,
		.enable_cde_in_fecs = gr_gm20b_enable_cde_in_fecs,
		.bpt_reg_info = gr_gm20b_bpt_reg_info,
		.get_access_map = gr_gp10b_get_access_map,
		.handle_fecs_error = gr_gp10b_handle_fecs_error,
		.handle_sm_exception = gr_gp10b_handle_sm_exception,
		.handle_tex_exception = gr_gp10b_handle_tex_exception,
		.enable_gpc_exceptions = gk20a_gr_enable_gpc_exceptions,
		.enable_exceptions = gk20a_gr_enable_exceptions,
		.get_lrf_tex_ltc_dram_override = get_ecc_override_val,
		.update_smpc_ctxsw_mode = gr_gk20a_update_smpc_ctxsw_mode,
		.update_hwpm_ctxsw_mode = gr_gk20a_update_hwpm_ctxsw_mode,
		.record_sm_error_state = gm20b_gr_record_sm_error_state,
		.update_sm_error_state = gm20b_gr_update_sm_error_state,
		.clear_sm_error_state = gm20b_gr_clear_sm_error_state,
		.suspend_contexts = gr_gp10b_suspend_contexts,
		.resume_contexts = gr_gk20a_resume_contexts,
		.get_preemption_mode_flags = gr_gp10b_get_preemption_mode_flags,
		.fuse_override = gp10b_gr_fuse_override,
		.init_sm_id_table = gr_gk20a_init_sm_id_table,
		.load_smid_config = gr_gp10b_load_smid_config,
		.program_sm_id_numbering = gr_gm20b_program_sm_id_numbering,
		.is_ltcs_ltss_addr = gr_gm20b_is_ltcs_ltss_addr,
		.is_ltcn_ltss_addr = gr_gm20b_is_ltcn_ltss_addr,
		.split_lts_broadcast_addr = gr_gm20b_split_lts_broadcast_addr,
		.split_ltc_broadcast_addr = gr_gm20b_split_ltc_broadcast_addr,
		.setup_rop_mapping = gr_gk20a_setup_rop_mapping,
		.program_zcull_mapping = gr_gk20a_program_zcull_mapping,
		.commit_global_timeslice = gr_gk20a_commit_global_timeslice,
		.commit_inst = gr_gk20a_commit_inst,
		.write_zcull_ptr = gr_gk20a_write_zcull_ptr,
		.write_pm_ptr = gr_gk20a_write_pm_ptr,
		.init_elcg_mode = gr_gk20a_init_elcg_mode,
		.load_tpc_mask = gr_gm20b_load_tpc_mask,
		.inval_icache = gr_gk20a_inval_icache,
		.trigger_suspend = gr_gk20a_trigger_suspend,
		.wait_for_pause = gr_gk20a_wait_for_pause,
		.resume_from_pause = gr_gk20a_resume_from_pause,
		.clear_sm_errors = gr_gk20a_clear_sm_errors,
		.tpc_enabled_exceptions = gr_gk20a_tpc_enabled_exceptions,
		.get_esr_sm_sel = gk20a_gr_get_esr_sm_sel,
		.sm_debugger_attached = gk20a_gr_sm_debugger_attached,
		.suspend_single_sm = gk20a_gr_suspend_single_sm,
		.suspend_all_sms = gk20a_gr_suspend_all_sms,
		.resume_single_sm = gk20a_gr_resume_single_sm,
		.resume_all_sms = gk20a_gr_resume_all_sms,
		.get_sm_hww_warp_esr = gp10b_gr_get_sm_hww_warp_esr,
		.get_sm_hww_global_esr = gk20a_gr_get_sm_hww_global_esr,
		.get_sm_no_lock_down_hww_global_esr_mask =
			gk20a_gr_get_sm_no_lock_down_hww_global_esr_mask,
		.lock_down_sm = gk20a_gr_lock_down_sm,
		.wait_for_sm_lock_down = gk20a_gr_wait_for_sm_lock_down,
		.clear_sm_hww = gm20b_gr_clear_sm_hww,
		.init_ovr_sm_dsm_perf =  gk20a_gr_init_ovr_sm_dsm_perf,
		.get_ovr_perf_regs = gk20a_gr_get_ovr_perf_regs,
		.disable_rd_coalesce = gm20a_gr_disable_rd_coalesce,
		.set_boosted_ctx = NULL,
		.set_preemption_mode = gr_gp10b_set_preemption_mode,
		.set_czf_bypass = gr_gp10b_set_czf_bypass,
		.pre_process_sm_exception = gr_gp10b_pre_process_sm_exception,
		.set_preemption_buffer_va = gr_gp10b_set_preemption_buffer_va,
		.init_preemption_state = NULL,
		.update_boosted_ctx = NULL,
		.set_bes_crop_debug3 = gr_gp10b_set_bes_crop_debug3,
		.create_gr_sysfs = NULL,
		.set_ctxsw_preemption_mode = gr_gp106_set_ctxsw_preemption_mode,
		.load_ctxsw_ucode = gr_gm20b_load_ctxsw_ucode
	},
	.fb = {
		.reset = gp106_fb_reset,
		.init_hw = gk20a_fb_init_hw,
		.init_fs_state = NULL,
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
	},
	.clock_gating = {
		.slcg_bus_load_gating_prod =
			gp106_slcg_bus_load_gating_prod,
		.slcg_ce2_load_gating_prod =
			gp106_slcg_ce2_load_gating_prod,
		.slcg_chiplet_load_gating_prod =
			gp106_slcg_chiplet_load_gating_prod,
		.slcg_ctxsw_firmware_load_gating_prod =
			gp106_slcg_ctxsw_firmware_load_gating_prod,
		.slcg_fb_load_gating_prod =
			gp106_slcg_fb_load_gating_prod,
		.slcg_fifo_load_gating_prod =
			gp106_slcg_fifo_load_gating_prod,
		.slcg_gr_load_gating_prod =
			gr_gp106_slcg_gr_load_gating_prod,
		.slcg_ltc_load_gating_prod =
			ltc_gp106_slcg_ltc_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gp106_slcg_perf_load_gating_prod,
		.slcg_priring_load_gating_prod =
			gp106_slcg_priring_load_gating_prod,
		.slcg_pmu_load_gating_prod =
			gp106_slcg_pmu_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gp106_slcg_therm_load_gating_prod,
		.slcg_xbar_load_gating_prod =
			gp106_slcg_xbar_load_gating_prod,
		.blcg_bus_load_gating_prod =
			gp106_blcg_bus_load_gating_prod,
		.blcg_ce_load_gating_prod =
			gp106_blcg_ce_load_gating_prod,
		.blcg_fb_load_gating_prod =
			gp106_blcg_fb_load_gating_prod,
		.blcg_fifo_load_gating_prod =
			gp106_blcg_fifo_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gp106_blcg_gr_load_gating_prod,
		.blcg_ltc_load_gating_prod =
			gp106_blcg_ltc_load_gating_prod,
		.blcg_pmu_load_gating_prod =
			gp106_blcg_pmu_load_gating_prod,
		.blcg_xbar_load_gating_prod =
			gp106_blcg_xbar_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gp106_pg_gr_load_gating_prod,
	},
	.fifo = {
		.init_fifo_setup_hw = gk20a_init_fifo_setup_hw,
		.bind_channel = channel_gm20b_bind,
		.unbind_channel = gk20a_fifo_channel_unbind,
		.disable_channel = gk20a_fifo_disable_channel,
		.enable_channel = gk20a_fifo_enable_channel,
		.alloc_inst = gk20a_fifo_alloc_inst,
		.free_inst = gk20a_fifo_free_inst,
		.setup_ramfc = channel_gp10b_setup_ramfc,
		.channel_set_priority = gk20a_fifo_set_priority,
		.channel_set_timeslice = gk20a_fifo_set_timeslice,
		.default_timeslice_us = gk20a_fifo_default_timeslice_us,
		.setup_userd = gk20a_fifo_setup_userd,
		.userd_gp_get = gk20a_fifo_userd_gp_get,
		.userd_gp_put = gk20a_fifo_userd_gp_put,
		.userd_pb_get = gk20a_fifo_userd_pb_get,
		.pbdma_acquire_val = gk20a_fifo_pbdma_acquire_val,
		.preempt_channel = gk20a_fifo_preempt_channel,
		.preempt_tsg = gk20a_fifo_preempt_tsg,
		.update_runlist = gk20a_fifo_update_runlist,
		.trigger_mmu_fault = gm20b_fifo_trigger_mmu_fault,
		.get_mmu_fault_info = gp10b_fifo_get_mmu_fault_info,
		.wait_engine_idle = gk20a_fifo_wait_engine_idle,
		.get_num_fifos = gp106_fifo_get_num_fifos,
		.get_pbdma_signature = gp10b_fifo_get_pbdma_signature,
		.set_runlist_interleave = gk20a_fifo_set_runlist_interleave,
		.tsg_set_timeslice = gk20a_fifo_tsg_set_timeslice,
		.force_reset_ch = gk20a_fifo_force_reset_ch,
		.engine_enum_from_type = gp10b_fifo_engine_enum_from_type,
		.device_info_data_parse = gp10b_device_info_data_parse,
		.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v,
		.init_engine_info = gk20a_fifo_init_engine_info,
		.runlist_entry_size = ram_rl_entry_size_v,
		.get_tsg_runlist_entry = gk20a_get_tsg_runlist_entry,
		.get_ch_runlist_entry = gk20a_get_ch_runlist_entry,
		.is_fault_engine_subid_gpc = gk20a_is_fault_engine_subid_gpc,
		.dump_pbdma_status = gk20a_dump_pbdma_status,
		.dump_eng_status = gk20a_dump_eng_status,
		.dump_channel_status_ramfc = gk20a_dump_channel_status_ramfc,
		.intr_0_error_mask = gk20a_fifo_intr_0_error_mask,
		.is_preempt_pending = gk20a_fifo_is_preempt_pending,
		.init_pbdma_intr_descs = gp10b_fifo_init_pbdma_intr_descs,
		.reset_enable_hw = gk20a_init_fifo_reset_enable_hw,
		.teardown_ch_tsg = gk20a_fifo_teardown_ch_tsg,
		.handle_sched_error = gk20a_fifo_handle_sched_error,
		.handle_pbdma_intr_0 = gk20a_fifo_handle_pbdma_intr_0,
		.handle_pbdma_intr_1 = gk20a_fifo_handle_pbdma_intr_1,
		.tsg_bind_channel = gk20a_tsg_bind_channel,
		.tsg_unbind_channel = gk20a_tsg_unbind_channel,
#ifdef CONFIG_TEGRA_GK20A_NVHOST
		.alloc_syncpt_buf = gk20a_fifo_alloc_syncpt_buf,
		.free_syncpt_buf = gk20a_fifo_free_syncpt_buf,
		.add_syncpt_wait_cmd = gk20a_fifo_add_syncpt_wait_cmd,
		.get_syncpt_wait_cmd_size = gk20a_fifo_get_syncpt_wait_cmd_size,
		.add_syncpt_incr_cmd = gk20a_fifo_add_syncpt_incr_cmd,
		.get_syncpt_incr_cmd_size = gk20a_fifo_get_syncpt_incr_cmd_size,
#endif
		.resetup_ramfc = gp10b_fifo_resetup_ramfc,
		.device_info_fault_id = top_device_info_data_fault_id_enum_v,
	},
	.gr_ctx = {
		.get_netlist_name = gr_gp106_get_netlist_name,
		.is_fw_defined = gr_gp106_is_firmware_defined,
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
		.l2_flush = gk20a_mm_l2_flush,
		.cbc_clean = gk20a_mm_cbc_clean,
		.set_big_page_size = gm20b_mm_set_big_page_size,
		.get_big_page_sizes = gm20b_mm_get_big_page_sizes,
		.get_default_big_page_size = gp10b_mm_get_default_big_page_size,
		.gpu_phys_addr = gm20b_gpu_phys_addr,
		.get_physical_addr_bits = NULL,
		.get_mmu_levels = gp10b_mm_get_mmu_levels,
		.init_pdb = gp10b_mm_init_pdb,
		.init_mm_setup_hw = gp10b_init_mm_setup_hw,
		.is_bar1_supported = gm20b_mm_is_bar1_supported,
		.init_inst_block = gk20a_init_inst_block,
		.mmu_fault_pending = gk20a_fifo_mmu_fault_pending,
		.init_bar2_vm = gb10b_init_bar2_vm,
		.init_bar2_mm_hw_setup = gb10b_init_bar2_mm_hw_setup,
		.remove_bar2_vm = gp10b_remove_bar2_vm,
		.get_vidmem_size = gp106_mm_get_vidmem_size,
	},
	.pramin = {
		.enter = gk20a_pramin_enter,
		.exit = gk20a_pramin_exit,
		.data032_r = pram_data032_r,
	},
	.therm = {
#ifdef CONFIG_DEBUG_FS
		.therm_debugfs_init = gp106_therm_debugfs_init,
#endif /* CONFIG_DEBUG_FS */
		.elcg_init_idle_filters = gp106_elcg_init_idle_filters,
		.get_internal_sensor_curr_temp =
			gp106_get_internal_sensor_curr_temp,
		.get_internal_sensor_limits = gp106_get_internal_sensor_limits,
		.configure_therm_alert = gp106_configure_therm_alert,
	},
	.pmu = {
		.init_wpr_region = gm20b_pmu_init_acr,
		.load_lsfalcon_ucode = gp106_load_falcon_ucode,
		.is_lazy_bootstrap = gp106_is_lazy_bootstrap,
		.is_priv_load = gp106_is_priv_load,
		.prepare_ucode = gp106_prepare_ucode_blob,
		.pmu_setup_hw_and_bootstrap = gp106_bootstrap_hs_flcn,
		.get_wpr = gp106_wpr_info,
		.alloc_blob_space = gp106_alloc_blob_space,
		.pmu_populate_loader_cfg = gp106_pmu_populate_loader_cfg,
		.flcn_populate_bl_dmem_desc = gp106_flcn_populate_bl_dmem_desc,
		.falcon_wait_for_halt = sec2_wait_for_halt,
		.falcon_clear_halt_interrupt_status =
			sec2_clear_halt_interrupt_status,
		.init_falcon_setup_hw = init_sec2_setup_hw1,
		.pmu_queue_tail = gk20a_pmu_queue_tail,
		.pmu_get_queue_head = pwr_pmu_queue_head_r,
		.pmu_mutex_release = gk20a_pmu_mutex_release,
		.is_pmu_supported = gp106_is_pmu_supported,
		.pmu_pg_supported_engines_list = gp106_pmu_pg_engines_list,
		.pmu_elpg_statistics = gp106_pmu_elpg_statistics,
		.pmu_mutex_acquire = gk20a_pmu_mutex_acquire,
		.pmu_is_lpwr_feature_supported =
			gp106_pmu_is_lpwr_feature_supported,
		.pmu_msgq_tail = gk20a_pmu_msgq_tail,
		.pmu_pg_engines_feature_list = gp106_pmu_pg_feature_list,
		.pmu_get_queue_head_size = pwr_pmu_queue_head__size_1_v,
		.pmu_queue_head = gk20a_pmu_queue_head,
		.pmu_pg_param_post_init = nvgpu_lpwr_post_init,
		.pmu_get_queue_tail_size = pwr_pmu_queue_tail__size_1_v,
		.pmu_pg_init_param = gp106_pg_param_init,
		.reset_engine = gp106_pmu_engine_reset,
		.pmu_lpwr_disable_pg = nvgpu_lpwr_disable_pg,
		.write_dmatrfbase = gp10b_write_dmatrfbase,
		.pmu_mutex_size = pwr_pmu_mutex__size_1_v,
		.is_engine_in_reset = gp106_pmu_is_engine_in_reset,
		.pmu_get_queue_tail = pwr_pmu_queue_tail_r,
		.pmu_lpwr_enable_pg = nvgpu_lpwr_enable_pg,
	},
	.clk = {
		.init_clk_support = gp106_init_clk_support,
		.get_crystal_clk_hz = gp106_crystal_clk_hz,
		.measure_freq = gp106_clk_measure_freq,
		.suspend_clk_support = gp106_suspend_clk_support,
		.mclk_init = gp106_mclk_init,
		.mclk_change = gp106_mclk_change,
		.mclk_deinit = gp106_mclk_deinit,
	},
	.clk_arb = {
		.get_arbiter_clk_domains = gp106_get_arbiter_clk_domains,
		.get_arbiter_clk_range = gp106_get_arbiter_clk_range,
		.get_arbiter_clk_default = gp106_get_arbiter_clk_default,
		.get_current_pstate = nvgpu_clk_arb_get_current_pstate,
	},
	.regops = {
		.get_global_whitelist_ranges =
			gp106_get_global_whitelist_ranges,
		.get_global_whitelist_ranges_count =
			gp106_get_global_whitelist_ranges_count,
		.get_context_whitelist_ranges =
			gp106_get_context_whitelist_ranges,
		.get_context_whitelist_ranges_count =
			gp106_get_context_whitelist_ranges_count,
		.get_runcontrol_whitelist = gp106_get_runcontrol_whitelist,
		.get_runcontrol_whitelist_count =
			gp106_get_runcontrol_whitelist_count,
		.get_runcontrol_whitelist_ranges =
			gp106_get_runcontrol_whitelist_ranges,
		.get_runcontrol_whitelist_ranges_count =
			gp106_get_runcontrol_whitelist_ranges_count,
		.get_qctl_whitelist = gp106_get_qctl_whitelist,
		.get_qctl_whitelist_count = gp106_get_qctl_whitelist_count,
		.get_qctl_whitelist_ranges = gp106_get_qctl_whitelist_ranges,
		.get_qctl_whitelist_ranges_count =
			gp106_get_qctl_whitelist_ranges_count,
		.apply_smpc_war = gp106_apply_smpc_war,
	},
	.mc = {
		.intr_enable = mc_gp10b_intr_enable,
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
	.cde = {
		.get_program_numbers = gp10b_cde_get_program_numbers,
		.need_scatter_buffer = gp10b_need_scatter_buffer,
		.populate_scatter_buffer = gp10b_populate_scatter_buffer,
	},
	.bus = {
		.init_hw = gk20a_bus_init_hw,
		.isr = gk20a_bus_isr,
		.read_ptimer = gk20a_read_ptimer,
		.get_timestamps_zipper = nvgpu_get_timestamps_zipper,
		.bar1_bind = gk20a_bus_bar1_bind,
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
	.xve = {
		.get_speed        = xve_get_speed_gp106,
		.set_speed        = xve_set_speed_gp106,
		.available_speeds = xve_available_speeds_gp106,
		.xve_readl        = xve_xve_readl_gp106,
		.xve_writel       = xve_xve_writel_gp106,
		.disable_aspm     = xve_disable_aspm_gp106,
		.reset_gpu        = xve_reset_gpu_gp106,
#if defined(CONFIG_PCI_MSI)
		.rearm_msi        = xve_rearm_msi_gp106,
#endif
		.enable_shadow_rom = xve_enable_shadow_rom_gp106,
		.disable_shadow_rom = xve_disable_shadow_rom_gp106,
		.get_link_control_status = xve_get_link_control_status,
	},
	.falcon = {
		.falcon_hal_sw_init = gp106_falcon_hal_sw_init,
	},
	.priv_ring = {
		.isr = gp10b_priv_ring_isr,
	},
	.get_litter_value = gp106_get_litter_value,
	.chip_init_gpu_characteristics = gp106_init_gpu_characteristics,
	.bios_init = gm206_bios_init,
};

int gp106_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;

	gk20a_dbg_fn("");

	gops->ltc = gp106_ops.ltc;
	gops->ce2 = gp106_ops.ce2;
	gops->gr = gp106_ops.gr;
	gops->fb = gp106_ops.fb;
	gops->clock_gating = gp106_ops.clock_gating;
	gops->fifo = gp106_ops.fifo;
	gops->gr_ctx = gp106_ops.gr_ctx;
	gops->fecs_trace = gp106_ops.fecs_trace;
	gops->mm = gp106_ops.mm;
	gops->pramin = gp106_ops.pramin;
	gops->therm = gp106_ops.therm;
	gops->pmu = gp106_ops.pmu;
	/*
	 * clk must be assigned member by member
	 * since some clk ops are assigned during probe prior to HAL init
	 */
	gops->clk.init_clk_support = gp106_ops.clk.init_clk_support;
	gops->clk.get_crystal_clk_hz = gp106_ops.clk.get_crystal_clk_hz;
	gops->clk.measure_freq = gp106_ops.clk.measure_freq;
	gops->clk.suspend_clk_support = gp106_ops.clk.suspend_clk_support;
	gops->clk.mclk_init = gp106_ops.clk.mclk_init;
	gops->clk.mclk_change = gp106_ops.clk.mclk_change;
	gops->clk.mclk_deinit = gp106_ops.clk.mclk_deinit;

	gops->clk_arb = gp106_ops.clk_arb;
	gops->regops = gp106_ops.regops;
	gops->mc = gp106_ops.mc;
	gops->debug = gp106_ops.debug;
	gops->dbg_session_ops = gp106_ops.dbg_session_ops;
	gops->cde = gp106_ops.cde;
	gops->bus = gp106_ops.bus;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gops->css = gp106_ops.css;
#endif
	gops->xve = gp106_ops.xve;
	gops->falcon = gp106_ops.falcon;
	gops->priv_ring = gp106_ops.priv_ring;

	/* Lone functions */
	gops->get_litter_value = gp106_ops.get_litter_value;
	gops->chip_init_gpu_characteristics =
		gp106_ops.chip_init_gpu_characteristics;
	gops->bios_init = gp106_ops.bios_init;

	__nvgpu_set_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP, true);
	__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, true);
	__nvgpu_set_enabled(g, NVGPU_SEC_SECUREGPCCS, true);
	__nvgpu_set_enabled(g, NVGPU_PMU_PSTATE, true);
	__nvgpu_set_enabled(g, NVGPU_PMU_FECS_BOOTSTRAP_DONE, false);

	g->pmu_lsf_pmu_wpr_init_done = 0;
	g->bootstrap_owner = LSF_FALCON_ID_SEC2;

	gp10b_init_uncompressed_kind_map();
	gp10b_init_kind_attr();

	g->name = "gp10x";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = PASCAL_B;
	c->compute_class = PASCAL_COMPUTE_B;
	c->gpfifo_class = PASCAL_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = PASCAL_DMA_COPY_A;

	gk20a_dbg_fn("done");

	return 0;
}
