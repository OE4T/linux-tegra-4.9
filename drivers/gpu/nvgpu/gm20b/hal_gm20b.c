/*
 * GM20B Graphics
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

#include "gk20a/gk20a.h"
#include "gk20a/ce2_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/fifo_gk20a.h"
#include "gk20a/therm_gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "gk20a/mc_gk20a.h"
#include "gk20a/bus_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/priv_ring_gk20a.h"
#include "gk20a/regops_gk20a.h"

#include "ltc_gm20b.h"
#include "gr_gm20b.h"
#include "ltc_gm20b.h"
#include "fb_gm20b.h"
#include "gm20b_gating_reglist.h"
#include "fifo_gm20b.h"
#include "gr_ctx_gm20b.h"
#include "mm_gm20b.h"
#include "pmu_gm20b.h"
#include "clk_gm20b.h"
#include "regops_gm20b.h"
#include "cde_gm20b.h"
#include "therm_gm20b.h"
#include "bus_gm20b.h"
#include "hal_gm20b.h"

#include <nvgpu/debug.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/bus.h>

#include <nvgpu/hw/gm20b/hw_proj_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fuse_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fifo_gm20b.h>
#include <nvgpu/hw/gm20b/hw_ram_gm20b.h>
#include <nvgpu/hw/gm20b/hw_top_gm20b.h>

#define PRIV_SECURITY_DISABLE 0x01

static int gm20b_get_litter_value(struct gk20a *g, int value)
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
	/* GM20B does not have a FBPA unit, despite what's listed in the
	 * hw headers or read back through NV_PTOP_SCAL_NUM_FBPAS,
	 * so hardcode all values to 0.
	 */
	case GPU_LIT_NUM_FBPAS:
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

static const struct gpu_ops gm20b_ops = {
	.ltc = {
		.determine_L2_size_bytes = gm20b_determine_L2_size_bytes,
		.set_zbc_color_entry = gm20b_ltc_set_zbc_color_entry,
		.set_zbc_depth_entry = gm20b_ltc_set_zbc_depth_entry,
		.init_cbc = gm20b_ltc_init_cbc,
		.init_fs_state = gm20b_ltc_init_fs_state,
		.init_comptags = gm20b_ltc_init_comptags,
		.cbc_ctrl = gm20b_ltc_cbc_ctrl,
		.isr = gm20b_ltc_isr,
		.cbc_fix_config = gm20b_ltc_cbc_fix_config,
		.flush = gm20b_flush_ltc,
#ifdef CONFIG_DEBUG_FS
		.sync_debugfs = gm20b_ltc_sync_debugfs,
#endif
	},
	.ce2 = {
		.isr_stall = gk20a_ce2_isr,
		.isr_nonstall = gk20a_ce2_nonstall_isr,
	},
	.clock_gating = {
		.slcg_bus_load_gating_prod =
			gm20b_slcg_bus_load_gating_prod,
		.slcg_ce2_load_gating_prod =
			gm20b_slcg_ce2_load_gating_prod,
		.slcg_chiplet_load_gating_prod =
			gm20b_slcg_chiplet_load_gating_prod,
		.slcg_ctxsw_firmware_load_gating_prod =
			gm20b_slcg_ctxsw_firmware_load_gating_prod,
		.slcg_fb_load_gating_prod =
			gm20b_slcg_fb_load_gating_prod,
		.slcg_fifo_load_gating_prod =
			gm20b_slcg_fifo_load_gating_prod,
		.slcg_gr_load_gating_prod =
			gr_gm20b_slcg_gr_load_gating_prod,
		.slcg_ltc_load_gating_prod =
			ltc_gm20b_slcg_ltc_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gm20b_slcg_perf_load_gating_prod,
		.slcg_priring_load_gating_prod =
			gm20b_slcg_priring_load_gating_prod,
		.slcg_pmu_load_gating_prod =
			gm20b_slcg_pmu_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gm20b_slcg_therm_load_gating_prod,
		.slcg_xbar_load_gating_prod =
			gm20b_slcg_xbar_load_gating_prod,
		.blcg_bus_load_gating_prod =
			gm20b_blcg_bus_load_gating_prod,
		.blcg_ctxsw_firmware_load_gating_prod =
			gm20b_blcg_ctxsw_firmware_load_gating_prod,
		.blcg_fb_load_gating_prod =
			gm20b_blcg_fb_load_gating_prod,
		.blcg_fifo_load_gating_prod =
			gm20b_blcg_fifo_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gm20b_blcg_gr_load_gating_prod,
		.blcg_ltc_load_gating_prod =
			gm20b_blcg_ltc_load_gating_prod,
		.blcg_pwr_csb_load_gating_prod =
			gm20b_blcg_pwr_csb_load_gating_prod,
		.blcg_xbar_load_gating_prod =
			gm20b_blcg_xbar_load_gating_prod,
		.blcg_pmu_load_gating_prod =
			gm20b_blcg_pmu_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gm20b_pg_gr_load_gating_prod,
	},
	.fifo = {
		.init_fifo_setup_hw = gk20a_init_fifo_setup_hw,
		.bind_channel = channel_gm20b_bind,
		.unbind_channel = gk20a_fifo_channel_unbind,
		.disable_channel = gk20a_fifo_disable_channel,
		.enable_channel = gk20a_fifo_enable_channel,
		.alloc_inst = gk20a_fifo_alloc_inst,
		.free_inst = gk20a_fifo_free_inst,
		.setup_ramfc = gk20a_fifo_setup_ramfc,
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
		.get_mmu_fault_info = gk20a_fifo_get_mmu_fault_info,
		.wait_engine_idle = gk20a_fifo_wait_engine_idle,
		.get_num_fifos = gm20b_fifo_get_num_fifos,
		.get_pbdma_signature = gk20a_fifo_get_pbdma_signature,
		.set_runlist_interleave = gk20a_fifo_set_runlist_interleave,
		.tsg_set_timeslice = gk20a_fifo_tsg_set_timeslice,
		.force_reset_ch = gk20a_fifo_force_reset_ch,
		.engine_enum_from_type = gk20a_fifo_engine_enum_from_type,
		.device_info_data_parse = gm20b_device_info_data_parse,
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
		.init_pbdma_intr_descs = gm20b_fifo_init_pbdma_intr_descs,
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
	},
	.gr_ctx = {
		.get_netlist_name = gr_gm20b_get_netlist_name,
		.is_fw_defined = gr_gm20b_is_firmware_defined,
	},
	.therm = {
		.init_therm_setup_hw = gm20b_init_therm_setup_hw,
		.elcg_init_idle_filters = gk20a_elcg_init_idle_filters,
	},
	.clk = {
		.init_clk_support = gm20b_init_clk_support,
		.suspend_clk_support = gm20b_suspend_clk_support,
#ifdef CONFIG_DEBUG_FS
		.init_debugfs = gm20b_clk_init_debugfs,
#endif
		.get_voltage = gm20b_clk_get_voltage,
		.get_gpcclk_clock_counter = gm20b_clk_get_gpcclk_clock_counter,
		.pll_reg_write = gm20b_clk_pll_reg_write,
		.get_pll_debug_data = gm20b_clk_get_pll_debug_data,
	},
	.regops = {
		.get_global_whitelist_ranges =
			gm20b_get_global_whitelist_ranges,
		.get_global_whitelist_ranges_count =
			gm20b_get_global_whitelist_ranges_count,
		.get_context_whitelist_ranges =
			gm20b_get_context_whitelist_ranges,
		.get_context_whitelist_ranges_count =
			gm20b_get_context_whitelist_ranges_count,
		.get_runcontrol_whitelist = gm20b_get_runcontrol_whitelist,
		.get_runcontrol_whitelist_count =
			gm20b_get_runcontrol_whitelist_count,
		.get_runcontrol_whitelist_ranges =
			gm20b_get_runcontrol_whitelist_ranges,
		.get_runcontrol_whitelist_ranges_count =
			gm20b_get_runcontrol_whitelist_ranges_count,
		.get_qctl_whitelist = gm20b_get_qctl_whitelist,
		.get_qctl_whitelist_count = gm20b_get_qctl_whitelist_count,
		.get_qctl_whitelist_ranges = gm20b_get_qctl_whitelist_ranges,
		.get_qctl_whitelist_ranges_count =
			gm20b_get_qctl_whitelist_ranges_count,
		.apply_smpc_war = gm20b_apply_smpc_war,
	},
	.mc = {
		.intr_enable = mc_gk20a_intr_enable,
		.intr_unit_config = mc_gk20a_intr_unit_config,
		.isr_stall = mc_gk20a_isr_stall,
		.intr_stall = mc_gk20a_intr_stall,
		.intr_stall_pause = mc_gk20a_intr_stall_pause,
		.intr_stall_resume = mc_gk20a_intr_stall_resume,
		.intr_nonstall = mc_gk20a_intr_nonstall,
		.intr_nonstall_pause = mc_gk20a_intr_nonstall_pause,
		.intr_nonstall_resume = mc_gk20a_intr_nonstall_resume,
		.enable = gk20a_mc_enable,
		.disable = gk20a_mc_disable,
		.reset = gk20a_mc_reset,
		.boot_0 = gk20a_mc_boot_0,
		.is_intr1_pending = mc_gk20a_is_intr1_pending,
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
		.get_program_numbers = gm20b_cde_get_program_numbers,
	},
	.bus = {
		.init_hw = gk20a_bus_init_hw,
		.isr = gk20a_bus_isr,
		.read_ptimer = gk20a_read_ptimer,
		.get_timestamps_zipper = nvgpu_get_timestamps_zipper,
		.bar1_bind = gm20b_bus_bar1_bind,
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
		.isr = gk20a_priv_ring_isr,
	},
	.chip_init_gpu_characteristics = gk20a_init_gpu_characteristics,
	.get_litter_value = gm20b_get_litter_value,
};

int gm20b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;
	u32 val;

	gops->ltc = gm20b_ops.ltc;
	gops->ce2 = gm20b_ops.ce2;
	gops->clock_gating = gm20b_ops.clock_gating;
	gops->fifo = gm20b_ops.fifo;
	gops->gr_ctx = gm20b_ops.gr_ctx;
	gops->therm = gm20b_ops.therm;
	/*
	 * clk must be assigned member by member
	 * since some clk ops are assigned during probe prior to HAL init
	 */
	gops->clk.init_clk_support = gm20b_ops.clk.init_clk_support;
	gops->clk.suspend_clk_support = gm20b_ops.clk.suspend_clk_support;
	gops->clk.get_voltage = gm20b_ops.clk.get_voltage;
	gops->clk.get_gpcclk_clock_counter =
		gm20b_ops.clk.get_gpcclk_clock_counter;
	gops->clk.pll_reg_write = gm20b_ops.clk.pll_reg_write;
	gops->clk.get_pll_debug_data = gm20b_ops.clk.get_pll_debug_data;

	gops->regops = gm20b_ops.regops;
	gops->mc = gm20b_ops.mc;
	gops->dbg_session_ops = gm20b_ops.dbg_session_ops;
	gops->debug = gm20b_ops.debug;
	gops->cde = gm20b_ops.cde;
	gops->bus = gm20b_ops.bus;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gops->css = gm20b_ops.css;
#endif
	gops->falcon = gm20b_ops.falcon;

	gops->priv_ring = gm20b_ops.priv_ring;

	/* Lone functions */
	gops->chip_init_gpu_characteristics =
		gm20b_ops.chip_init_gpu_characteristics;
	gops->get_litter_value = gm20b_ops.get_litter_value;

	__nvgpu_set_enabled(g, NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP, true);
	__nvgpu_set_enabled(g, NVGPU_SEC_SECUREGPCCS, false);
	__nvgpu_set_enabled(g, NVGPU_PMU_PSTATE, false);

#ifdef CONFIG_TEGRA_ACR
	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, true);
	} else {
		val = gk20a_readl(g, fuse_opt_priv_sec_en_r());
		if (!val) {
			gk20a_dbg_info("priv security is disabled in HW");
			__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, false);
		} else {
			__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, true);
		}
	}
#else
	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		gk20a_dbg_info("running ASIM with PRIV security disabled");
		__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, false);
	} else {
		val = gk20a_readl(g, fuse_opt_priv_sec_en_r());
		if (!val) {
			__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, false);
		} else {
			gk20a_dbg_info("priv security is not supported but enabled");
			__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, true);
			return -EPERM;
		}
	}
#endif
	g->bootstrap_owner = LSF_BOOTSTRAP_OWNER_DEFAULT;
	gm20b_init_gr(g);
	gm20b_init_fb(gops);
	gm20b_init_mm(gops);
	gm20b_init_pmu_ops(g);

	g->name = "gm20b";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = MAXWELL_B;
	c->compute_class = MAXWELL_COMPUTE_B;
	c->gpfifo_class = MAXWELL_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = MAXWELL_DMA_COPY_A;

	return 0;
}
