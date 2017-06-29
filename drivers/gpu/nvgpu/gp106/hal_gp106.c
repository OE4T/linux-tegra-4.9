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
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "gk20a/bus_gk20a.h"
#include "gk20a/pramin_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "gk20a/mc_gk20a.h"

#include "gp10b/ltc_gp10b.h"
#include "gp10b/gr_gp10b.h"
#include "gp10b/fecs_trace_gp10b.h"
#include "gp10b/mc_gp10b.h"
#include "gp10b/mm_gp10b.h"
#include "gp10b/ce_gp10b.h"
#include "gp10b/regops_gp10b.h"
#include "gp10b/cde_gp10b.h"
#include "gp10b/priv_ring_gp10b.h"

#include "gp106/fifo_gp106.h"
#include "gp106/regops_gp106.h"

#include "gm20b/ltc_gm20b.h"
#include "gm20b/gr_gm20b.h"
#include "gm20b/fifo_gm20b.h"
#include "gm20b/pmu_gm20b.h"

#include "gp106/clk_gp106.h"
#include "gp106/clk_arb_gp106.h"
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

#include <nvgpu/hw/gp106/hw_proj_gp106.h>

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
#ifdef CONFIG_DEBUG_FS
		.sync_debugfs = gp10b_ltc_sync_debugfs,
#endif
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
		.sw_init          = xve_sw_init_gp106,
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
	gops->clock_gating = gp106_ops.clock_gating;
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

	gops->privsecurity = 1;
	gops->securegpccs = 1;
	gops->pmupstate = true;


	g->bootstrap_owner = LSF_FALCON_ID_SEC2;
	gp106_init_gr(gops);
	gp10b_init_fecs_trace_ops(gops);
	gp106_init_fb(gops);
	gp106_init_fifo(gops);
	gp10b_init_ce(gops);
	gp106_init_gr_ctx(gops);
	gp106_init_mm(gops);
	gp106_init_pmu_ops(gops);
	gp106_init_clk_ops(gops);
	gp106_init_clk_arb_ops(gops);
	gp106_init_regops(gops);
	gk20a_init_tsg_ops(gops);
	gk20a_init_pramin_ops(gops);
	gp106_init_therm_ops(gops);

	g->name = "gp10x";
	gops->gr_ctx.use_dma_for_fw_bootstrap = true;

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = PASCAL_B;
	c->compute_class = PASCAL_COMPUTE_B;
	c->gpfifo_class = PASCAL_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = PASCAL_DMA_COPY_A;

	gk20a_dbg_fn("done");

	return 0;
}
