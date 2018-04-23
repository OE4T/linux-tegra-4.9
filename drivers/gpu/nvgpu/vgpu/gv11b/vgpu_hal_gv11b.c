/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <gk20a/gk20a.h>
#include <gv11b/hal_gv11b.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/error_notifier.h>

#include "vgpu/fifo_vgpu.h"
#include "vgpu/gr_vgpu.h"
#include "vgpu/ltc_vgpu.h"
#include "vgpu/mm_vgpu.h"
#include "vgpu/dbg_vgpu.h"
#include "vgpu/fecs_trace_vgpu.h"
#include "vgpu/css_vgpu.h"
#include "vgpu/gm20b/vgpu_gr_gm20b.h"
#include "vgpu/gp10b/vgpu_mm_gp10b.h"
#include "vgpu/gp10b/vgpu_gr_gp10b.h"

#include <gk20a/fb_gk20a.h>
#include <gk20a/flcn_gk20a.h>
#include <gk20a/bus_gk20a.h>
#include <gk20a/mc_gk20a.h>
#include "gk20a/dbg_gpu_gk20a.h"

#include <gm20b/gr_gm20b.h>
#include <gm20b/fb_gm20b.h>
#include <gm20b/fifo_gm20b.h>
#include <gm20b/pmu_gm20b.h>
#include <gm20b/mm_gm20b.h>
#include <gm20b/acr_gm20b.h>
#include <gm20b/ltc_gm20b.h>

#include <gp10b/fb_gp10b.h>
#include <gp10b/pmu_gp10b.h>
#include <gp10b/mm_gp10b.h>
#include <gp10b/mc_gp10b.h>
#include <gp10b/ce_gp10b.h>
#include "gp10b/gr_gp10b.h"
#include <gp10b/fifo_gp10b.h>
#include <gp10b/therm_gp10b.h>
#include <gp10b/priv_ring_gp10b.h>
#include <gp10b/ltc_gp10b.h>

#include <gp106/pmu_gp106.h>
#include <gp106/acr_gp106.h>

#include <gv11b/fb_gv11b.h>
#include <gv11b/pmu_gv11b.h>
#include <gv11b/acr_gv11b.h>
#include <gv11b/mm_gv11b.h>
#include <gv11b/mc_gv11b.h>
#include <gv11b/ce_gv11b.h>
#include <gv11b/fifo_gv11b.h>
#include <gv11b/therm_gv11b.h>
#include <gv11b/regops_gv11b.h>
#include <gv11b/gr_ctx_gv11b.h>
#include <gv11b/ltc_gv11b.h>
#include <gv11b/gv11b_gating_reglist.h>
#include <gv11b/gr_gv11b.h>

#include <nvgpu/enabled.h>

#include "vgpu_gv11b.h"
#include "vgpu_gr_gv11b.h"
#include "vgpu_fifo_gv11b.h"
#include "vgpu_subctx_gv11b.h"
#include "vgpu_tsg_gv11b.h"

#include <nvgpu/hw/gv11b/hw_fuse_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>

static const struct gpu_ops vgpu_gv11b_ops = {
	.ltc = {
		.determine_L2_size_bytes = vgpu_determine_L2_size_bytes,
		.set_zbc_s_entry = gv11b_ltc_set_zbc_stencil_entry,
		.set_zbc_color_entry = gm20b_ltc_set_zbc_color_entry,
		.set_zbc_depth_entry = gm20b_ltc_set_zbc_depth_entry,
		.init_cbc = NULL,
		.init_fs_state = vgpu_ltc_init_fs_state,
		.init_comptags = vgpu_ltc_init_comptags,
		.cbc_ctrl = NULL,
		.isr = gv11b_ltc_isr,
		.flush = gm20b_flush_ltc,
		.set_enabled = gp10b_ltc_set_enabled,
	},
	.ce2 = {
		.isr_stall = gv11b_ce_isr,
		.isr_nonstall = gp10b_ce_nonstall_isr,
		.get_num_pce = vgpu_ce_get_num_pce,
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
		.init_fs_state = vgpu_gr_init_fs_state,
		.set_hww_esr_report_mask = gv11b_gr_set_hww_esr_report_mask,
		.falcon_load_ucode = gr_gm20b_load_ctxsw_ucode_segments,
		.load_ctxsw_ucode = gr_gk20a_load_ctxsw_ucode,
		.set_gpc_tpc_mask = gr_gv11b_set_gpc_tpc_mask,
		.get_gpc_tpc_mask = vgpu_gr_get_gpc_tpc_mask,
		.alloc_obj_ctx = vgpu_gr_alloc_obj_ctx,
		.bind_ctxsw_zcull = vgpu_gr_bind_ctxsw_zcull,
		.get_zcull_info = vgpu_gr_get_zcull_info,
		.is_tpc_addr = gr_gm20b_is_tpc_addr,
		.get_tpc_num = gr_gm20b_get_tpc_num,
		.detect_sm_arch = vgpu_gr_detect_sm_arch,
		.add_zbc_color = gr_gp10b_add_zbc_color,
		.add_zbc_depth = gr_gp10b_add_zbc_depth,
		.zbc_set_table = vgpu_gr_add_zbc,
		.zbc_query_table = vgpu_gr_query_zbc,
		.pmu_save_zbc = gk20a_pmu_save_zbc,
		.add_zbc = gr_gk20a_add_zbc,
		.pagepool_default_size = gr_gv11b_pagepool_default_size,
		.init_ctx_state = vgpu_gr_gp10b_init_ctx_state,
		.alloc_gr_ctx = vgpu_gr_gp10b_alloc_gr_ctx,
		.free_gr_ctx = vgpu_gr_free_gr_ctx,
		.update_ctxsw_preemption_mode =
			gr_gv11b_update_ctxsw_preemption_mode,
		.dump_gr_regs = NULL,
		.update_pc_sampling = vgpu_gr_update_pc_sampling,
		.get_fbp_en_mask = vgpu_gr_get_fbp_en_mask,
		.get_max_ltc_per_fbp = vgpu_gr_get_max_ltc_per_fbp,
		.get_max_lts_per_ltc = vgpu_gr_get_max_lts_per_ltc,
		.get_rop_l2_en_mask = vgpu_gr_rop_l2_en_mask,
		.get_max_fbps_count = vgpu_gr_get_max_fbps_count,
		.init_sm_dsm_reg_info = gv11b_gr_init_sm_dsm_reg_info,
		.wait_empty = gr_gv11b_wait_empty,
		.init_cyclestats = vgpu_gr_gm20b_init_cyclestats,
		.set_sm_debug_mode = vgpu_gr_set_sm_debug_mode,
		.enable_cde_in_fecs = gr_gm20b_enable_cde_in_fecs,
		.bpt_reg_info = gv11b_gr_bpt_reg_info,
		.get_access_map = gr_gv11b_get_access_map,
		.handle_fecs_error = gr_gv11b_handle_fecs_error,
		.handle_sm_exception = gr_gk20a_handle_sm_exception,
		.handle_tex_exception = gr_gv11b_handle_tex_exception,
		.enable_gpc_exceptions = gr_gv11b_enable_gpc_exceptions,
		.enable_exceptions = gr_gv11b_enable_exceptions,
		.get_lrf_tex_ltc_dram_override = get_ecc_override_val,
		.update_smpc_ctxsw_mode = vgpu_gr_update_smpc_ctxsw_mode,
		.update_hwpm_ctxsw_mode = vgpu_gr_update_hwpm_ctxsw_mode,
		.record_sm_error_state = gv11b_gr_record_sm_error_state,
		.update_sm_error_state = gv11b_gr_update_sm_error_state,
		.clear_sm_error_state = vgpu_gr_clear_sm_error_state,
		.suspend_contexts = vgpu_gr_suspend_contexts,
		.resume_contexts = vgpu_gr_resume_contexts,
		.get_preemption_mode_flags = gr_gp10b_get_preemption_mode_flags,
		.init_sm_id_table = vgpu_gr_init_sm_id_table,
		.load_smid_config = gr_gv11b_load_smid_config,
		.program_sm_id_numbering = gr_gv11b_program_sm_id_numbering,
		.is_ltcs_ltss_addr = gr_gm20b_is_ltcs_ltss_addr,
		.is_ltcn_ltss_addr = gr_gm20b_is_ltcn_ltss_addr,
		.split_lts_broadcast_addr = gr_gm20b_split_lts_broadcast_addr,
		.split_ltc_broadcast_addr = gr_gm20b_split_ltc_broadcast_addr,
		.setup_rop_mapping = gr_gv11b_setup_rop_mapping,
		.program_zcull_mapping = gr_gv11b_program_zcull_mapping,
		.commit_global_timeslice = gr_gv11b_commit_global_timeslice,
		.commit_inst = vgpu_gr_gv11b_commit_inst,
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
		.set_boosted_ctx = NULL,
		.set_preemption_mode = vgpu_gr_gp10b_set_preemption_mode,
		.set_czf_bypass = NULL,
		.pre_process_sm_exception = gr_gv11b_pre_process_sm_exception,
		.set_preemption_buffer_va = gr_gv11b_set_preemption_buffer_va,
		.init_preemption_state = NULL,
		.update_boosted_ctx = NULL,
		.set_bes_crop_debug3 = gr_gp10b_set_bes_crop_debug3,
		.set_bes_crop_debug4 = gr_gp10b_set_bes_crop_debug4,
#ifdef CONFIG_SYSFS
		.create_gr_sysfs = gr_gv11b_create_sysfs,
#endif
		.set_ctxsw_preemption_mode = vgpu_gr_gp10b_set_ctxsw_preemption_mode,
		.is_etpc_addr = gv11b_gr_pri_is_etpc_addr,
		.egpc_etpc_priv_addr_table = gv11b_gr_egpc_etpc_priv_addr_table,
		.handle_tpc_mpc_exception = gr_gv11b_handle_tpc_mpc_exception,
		.zbc_s_query_table = gr_gv11b_zbc_s_query_table,
		.load_zbc_s_default_tbl = gr_gv11b_load_stencil_default_tbl,
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
		.init_ctxsw_hdr_data = gr_gp10b_init_ctxsw_hdr_data,
		.init_gfxp_wfi_timeout_count =
			gr_gv11b_init_gfxp_wfi_timeout_count,
		.get_max_gfxp_wfi_timeout_count =
			gr_gv11b_get_max_gfxp_wfi_timeout_count,
		.add_ctxsw_reg_pm_fbpa = gr_gk20a_add_ctxsw_reg_pm_fbpa,
		.add_ctxsw_reg_perf_pma = gr_gk20a_add_ctxsw_reg_perf_pma,
		.decode_priv_addr = gr_gv11b_decode_priv_addr,
		.create_priv_addr_table = gr_gv11b_create_priv_addr_table,
		.get_pmm_per_chiplet_offset =
			gr_gv11b_get_pmm_per_chiplet_offset,
		.split_fbpa_broadcast_addr = gr_gk20a_split_fbpa_broadcast_addr,
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
		.compression_align_mask = gm20b_fb_compression_align_mask,
		.vpr_info_fetch = gm20b_fb_vpr_info_fetch,
		.dump_vpr_wpr_info = gm20b_fb_dump_vpr_wpr_info,
		.read_wpr_info = gm20b_fb_read_wpr_info,
		.is_debug_mode_enabled = NULL,
		.set_debug_mode = vgpu_mm_mmu_set_debug_mode,
		.tlb_invalidate = vgpu_mm_tlb_invalidate,
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
		.init_fifo_setup_hw = vgpu_gv11b_init_fifo_setup_hw,
		.bind_channel = vgpu_channel_bind,
		.unbind_channel = vgpu_channel_unbind,
		.disable_channel = vgpu_channel_disable,
		.enable_channel = vgpu_channel_enable,
		.alloc_inst = vgpu_channel_alloc_inst,
		.free_inst = vgpu_channel_free_inst,
		.setup_ramfc = vgpu_channel_setup_ramfc,
		.default_timeslice_us = vgpu_fifo_default_timeslice_us,
		.setup_userd = gk20a_fifo_setup_userd,
		.userd_gp_get = gv11b_userd_gp_get,
		.userd_gp_put = gv11b_userd_gp_put,
		.userd_pb_get = gv11b_userd_pb_get,
		.pbdma_acquire_val = gk20a_fifo_pbdma_acquire_val,
		.preempt_channel = vgpu_fifo_preempt_channel,
		.preempt_tsg = vgpu_fifo_preempt_tsg,
		.enable_tsg = vgpu_enable_tsg,
		.disable_tsg = gk20a_disable_tsg,
		.tsg_verify_channel_status = NULL,
		.tsg_verify_status_ctx_reload = NULL,
		/* TODO: implement it for CE fault */
		.tsg_verify_status_faulted = NULL,
		.update_runlist = vgpu_fifo_update_runlist,
		.trigger_mmu_fault = NULL,
		.get_mmu_fault_info = NULL,
		.get_mmu_fault_desc = NULL,
		.get_mmu_fault_client_desc = NULL,
		.get_mmu_fault_gpc_desc = NULL,
		.wait_engine_idle = vgpu_fifo_wait_engine_idle,
		.get_num_fifos = gv11b_fifo_get_num_fifos,
		.get_pbdma_signature = gp10b_fifo_get_pbdma_signature,
		.set_runlist_interleave = vgpu_fifo_set_runlist_interleave,
		.tsg_set_timeslice = vgpu_tsg_set_timeslice,
		.tsg_open = vgpu_tsg_open,
		.tsg_release = vgpu_tsg_release,
		.force_reset_ch = vgpu_fifo_force_reset_ch,
		.engine_enum_from_type = gp10b_fifo_engine_enum_from_type,
		.device_info_data_parse = gp10b_device_info_data_parse,
		.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v,
		.init_engine_info = vgpu_fifo_init_engine_info,
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
		.tsg_bind_channel = vgpu_gv11b_tsg_bind_channel,
		.tsg_unbind_channel = vgpu_tsg_unbind_channel,
		.post_event_id = gk20a_tsg_event_id_post_event,
		.ch_abort_clean_up = gk20a_channel_abort_clean_up,
		.check_tsg_ctxsw_timeout = gk20a_fifo_check_tsg_ctxsw_timeout,
		.check_ch_ctxsw_timeout = gk20a_fifo_check_ch_ctxsw_timeout,
		.channel_suspend = gk20a_channel_suspend,
		.channel_resume = gk20a_channel_resume,
		.set_error_notifier = nvgpu_set_error_notifier,
		.setup_sw = gk20a_init_fifo_setup_sw,
#ifdef CONFIG_TEGRA_GK20A_NVHOST
		.alloc_syncpt_buf = vgpu_gv11b_fifo_alloc_syncpt_buf,
		.free_syncpt_buf = gv11b_fifo_free_syncpt_buf,
		.add_syncpt_wait_cmd = gv11b_fifo_add_syncpt_wait_cmd,
		.get_syncpt_wait_cmd_size = gv11b_fifo_get_syncpt_wait_cmd_size,
		.get_syncpt_incr_per_release =
                                gv11b_fifo_get_syncpt_incr_per_release,
		.add_syncpt_incr_cmd = gv11b_fifo_add_syncpt_incr_cmd,
		.get_syncpt_incr_cmd_size = gv11b_fifo_get_syncpt_incr_cmd_size,
		.get_sync_ro_map = vgpu_gv11b_fifo_get_sync_ro_map,
#endif
		.resetup_ramfc = NULL,
		.reschedule_runlist = NULL,
		.device_info_fault_id = top_device_info_data_fault_id_enum_v,
		.free_channel_ctx_header = vgpu_gv11b_free_subctx_header,
		.preempt_ch_tsg = gv11b_fifo_preempt_ch_tsg,
		.handle_ctxsw_timeout = gv11b_fifo_handle_ctxsw_timeout,
		.runlist_hw_submit = gk20a_fifo_runlist_hw_submit,
		.runlist_wait_pending = gk20a_fifo_runlist_wait_pending,
		.ring_channel_doorbell = gv11b_ring_channel_doorbell,
		.get_sema_wait_cmd_size = gk20a_fifo_get_sema_wait_cmd_size,
		.get_sema_incr_cmd_size = gk20a_fifo_get_sema_incr_cmd_size,
		.add_sema_cmd = gk20a_fifo_add_sema_cmd,
	},
	.gr_ctx = {
		.get_netlist_name = gr_gv11b_get_netlist_name,
		.is_fw_defined = gr_gv11b_is_firmware_defined,
	},
#ifdef CONFIG_GK20A_CTXSW_TRACE
	.fecs_trace = {
		.alloc_user_buffer = NULL,
		.free_user_buffer = NULL,
		.mmap_user_buffer = NULL,
		.init = NULL,
		.deinit = NULL,
		.enable = NULL,
		.disable = NULL,
		.is_enabled = NULL,
		.reset = NULL,
		.flush = NULL,
		.poll = NULL,
		.bind_channel = NULL,
		.unbind_channel = NULL,
		.max_entries = NULL,
	},
#endif /* CONFIG_GK20A_CTXSW_TRACE */
	.mm = {
		/* FIXME: add support for sparse mappings */
		.support_sparse = NULL,
		.gmmu_map = vgpu_gp10b_locked_gmmu_map,
		.gmmu_unmap = vgpu_locked_gmmu_unmap,
		.vm_bind_channel = vgpu_vm_bind_channel,
		.fb_flush = vgpu_mm_fb_flush,
		.l2_invalidate = vgpu_mm_l2_invalidate,
		.l2_flush = vgpu_mm_l2_flush,
		.cbc_clean = gk20a_mm_cbc_clean,
		.set_big_page_size = gm20b_mm_set_big_page_size,
		.get_big_page_sizes = gm20b_mm_get_big_page_sizes,
		.get_default_big_page_size = gp10b_mm_get_default_big_page_size,
		.gpu_phys_addr = gm20b_gpu_phys_addr,
		.get_iommu_bit = gk20a_mm_get_iommu_bit,
		.get_mmu_levels = gp10b_mm_get_mmu_levels,
		.init_pdb = gp10b_mm_init_pdb,
		.init_mm_setup_hw = vgpu_gp10b_init_mm_setup_hw,
		.is_bar1_supported = gv11b_mm_is_bar1_supported,
		.init_inst_block = gv11b_init_inst_block,
		.mmu_fault_pending = gv11b_mm_mmu_fault_pending,
		.get_kind_invalid = gm20b_get_kind_invalid,
		.get_kind_pitch = gm20b_get_kind_pitch,
		.init_bar2_vm = gp10b_init_bar2_vm,
		.init_bar2_mm_hw_setup = gv11b_init_bar2_mm_hw_setup,
		.remove_bar2_vm = gv11b_mm_remove_bar2_vm,
		.fault_info_mem_destroy = gv11b_mm_fault_info_mem_destroy,
	},
	.therm = {
		.init_therm_setup_hw = gp10b_init_therm_setup_hw,
		.elcg_init_idle_filters = gv11b_elcg_init_idle_filters,
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
		.pmu_init_perfmon = nvgpu_pmu_init_perfmon_rpc,
		.pmu_perfmon_start_sampling = nvgpu_pmu_perfmon_start_sampling_rpc,
		.pmu_perfmon_stop_sampling = nvgpu_pmu_perfmon_stop_sampling_rpc,
		.pmu_perfmon_get_samples_rpc = nvgpu_pmu_perfmon_get_samples_rpc,
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
		.isr_nonstall = mc_gk20a_isr_nonstall,
		.enable = gk20a_mc_enable,
		.disable = gk20a_mc_disable,
		.reset = gk20a_mc_reset,
		.boot_0 = gk20a_mc_boot_0,
		.is_intr1_pending = mc_gp10b_is_intr1_pending,
		.is_intr_hub_pending = gv11b_mc_is_intr_hub_pending,
	},
	.debug = {
		.show_dump = NULL,
	},
	.debugger = {
		.post_events = gk20a_dbg_gpu_post_events,
	},
	.dbg_session_ops = {
		.exec_reg_ops = vgpu_exec_regops,
		.dbg_set_powergate = vgpu_dbg_set_powergate,
		.check_and_set_global_reservation =
			vgpu_check_and_set_global_reservation,
		.check_and_set_context_reservation =
			vgpu_check_and_set_context_reservation,
		.release_profiler_reservation =
			vgpu_release_profiler_reservation,
		.perfbuffer_enable = vgpu_perfbuffer_enable,
		.perfbuffer_disable = vgpu_perfbuffer_disable,
	},
	.bus = {
		.init_hw = gk20a_bus_init_hw,
		.isr = gk20a_bus_isr,
		.read_ptimer = vgpu_read_ptimer,
		.get_timestamps_zipper = vgpu_get_timestamps_zipper,
		.bar1_bind = NULL,
		.set_ppriv_timeout_settings =
			gk20a_bus_set_ppriv_timeout_settings,
	},
#if defined(CONFIG_GK20A_CYCLE_STATS)
	.css = {
		.enable_snapshot = vgpu_css_enable_snapshot_buffer,
		.disable_snapshot = vgpu_css_release_snapshot_buffer,
		.check_data_available = vgpu_css_flush_snapshots,
		.detach_snapshot = vgpu_css_detach,
		.set_handled_snapshots = NULL,
		.allocate_perfmon_ids = NULL,
		.release_perfmon_ids = NULL,
	},
#endif
	.falcon = {
		.falcon_hal_sw_init = gk20a_falcon_hal_sw_init,
	},
	.priv_ring = {
		.isr = gp10b_priv_ring_isr,
	},
	.chip_init_gpu_characteristics = vgpu_gv11b_init_gpu_characteristics,
	.get_litter_value = gv11b_get_litter_value,
};

int vgpu_gv11b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;

	gops->ltc = vgpu_gv11b_ops.ltc;
	gops->ce2 = vgpu_gv11b_ops.ce2;
	gops->gr = vgpu_gv11b_ops.gr;
	gops->fb = vgpu_gv11b_ops.fb;
	gops->clock_gating = vgpu_gv11b_ops.clock_gating;
	gops->fifo = vgpu_gv11b_ops.fifo;
	gops->gr_ctx = vgpu_gv11b_ops.gr_ctx;
	gops->mm = vgpu_gv11b_ops.mm;
#ifdef CONFIG_GK20A_CTXSW_TRACE
	gops->fecs_trace = vgpu_gv11b_ops.fecs_trace;
#endif
	gops->therm = vgpu_gv11b_ops.therm;
	gops->pmu = vgpu_gv11b_ops.pmu;
	gops->regops = vgpu_gv11b_ops.regops;
	gops->mc = vgpu_gv11b_ops.mc;
	gops->debug = vgpu_gv11b_ops.debug;
	gops->debugger = vgpu_gv11b_ops.debugger;
	gops->dbg_session_ops = vgpu_gv11b_ops.dbg_session_ops;
	gops->bus = vgpu_gv11b_ops.bus;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gops->css = vgpu_gv11b_ops.css;
#endif
	gops->falcon = vgpu_gv11b_ops.falcon;
	gops->priv_ring = vgpu_gv11b_ops.priv_ring;

	/* Lone functions */
	gops->chip_init_gpu_characteristics =
		vgpu_gv11b_ops.chip_init_gpu_characteristics;
	gops->get_litter_value = vgpu_gv11b_ops.get_litter_value;
	gops->semaphore_wakeup = gk20a_channel_semaphore_wakeup;

	g->name = "gv11b";

	return 0;
}
