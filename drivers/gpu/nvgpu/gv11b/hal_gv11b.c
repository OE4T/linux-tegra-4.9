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
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/bus_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/regops_gk20a.h"

#include "gm20b/ltc_gm20b.h"
#include "gm20b/gr_gm20b.h"
#include "gm20b/fifo_gm20b.h"

#include "gp10b/ltc_gp10b.h"
#include "gp10b/therm_gp10b.h"
#include "gp10b/mc_gp10b.h"
#include "gp10b/ce_gp10b.h"
#include "gp10b/priv_ring_gp10b.h"
#include "gp10b/fifo_gp10b.h"
#include "gp10b/fecs_trace_gp10b.h"

#include "hal_gv11b.h"
#include "gr_gv11b.h"
#include "mc_gv11b.h"
#include "ltc_gv11b.h"
#include "gv11b.h"
#include "ce_gv11b.h"
#include "gr_ctx_gv11b.h"
#include "mm_gv11b.h"
#include "pmu_gv11b.h"
#include "fb_gv11b.h"
#include "fifo_gv11b.h"
#include "gv11b_gating_reglist.h"
#include "regops_gv11b.h"
#include "subctx_gv11b.h"

#include <nvgpu/debug.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gv11b/hw_proj_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>

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
	case GPU_LIT_NUM_FBPAS:
		ret = proj_scal_litter_num_fbpas_v();
		break;
	case GPU_LIT_FBPA_STRIDE:
		ret = proj_fbpa_stride_v();
		break;
	case GPU_LIT_SM_PRI_STRIDE:
		ret = proj_sm_stride_v();
		break;

	default:
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
	.therm = {
		.init_therm_setup_hw = gp10b_init_therm_setup_hw,
		.elcg_init_idle_filters = gp10b_elcg_init_idle_filters,
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

	gops->ltc = gv11b_ops.ltc;
	gops->ce2 = gv11b_ops.ce2;
	gops->clock_gating = gv11b_ops.clock_gating;
	gops->fifo = gv11b_ops.fifo;
	gops->gr_ctx = gv11b_ops.gr_ctx;
	gops->fecs_trace = gv11b_ops.fecs_trace;
	gops->therm = gv11b_ops.therm;
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

	/* boot in non-secure modes for time beeing */
	__nvgpu_set_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP, false);
	__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, false);
	__nvgpu_set_enabled(g, NVGPU_SEC_SECUREGPCCS, false);

	gv11b_init_gr(g);
	gv11b_init_fb(gops);
	gv11b_init_mm(gops);
	gv11b_init_pmu_ops(g);

	g->name = "gv11b";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = VOLTA_A;
	c->compute_class = VOLTA_COMPUTE_A;
	c->gpfifo_class = VOLTA_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = VOLTA_DMA_COPY_A;

	return 0;
}
