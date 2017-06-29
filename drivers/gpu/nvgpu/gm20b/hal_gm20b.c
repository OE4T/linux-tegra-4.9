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
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/css_gr_gk20a.h"
#include "gk20a/bus_gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gk20a/priv_ring_gk20a.h"

#include "ltc_gm20b.h"
#include "ce2_gm20b.h"
#include "gr_gm20b.h"
#include "ltc_gm20b.h"
#include "fb_gm20b.h"
#include "gm20b_gating_reglist.h"
#include "fifo_gm20b.h"
#include "gr_ctx_gm20b.h"
#include "mm_gm20b.h"
#include "pmu_gm20b.h"
#include "clk_gm20b.h"
#include "mc_gm20b.h"
#include "regops_gm20b.h"
#include "cde_gm20b.h"
#include "therm_gm20b.h"
#include "hal_gm20b.h"

#include <nvgpu/debug.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gm20b/hw_proj_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fuse_gm20b.h>

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
	.cde = {
		.get_program_numbers = gm20b_cde_get_program_numbers,
	},
	.falcon = {
		.falcon_hal_sw_init = gk20a_falcon_hal_sw_init,
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
	gops->clock_gating = gm20b_ops.clock_gating;
	gops->cde = gm20b_ops.cde;
	gops->falcon = gm20b_ops.falcon;

	/* Lone functions */
	gops->chip_init_gpu_characteristics =
		gm20b_ops.chip_init_gpu_characteristics;
	gops->get_litter_value = gm20b_ops.get_litter_value;

	gops->securegpccs = false;
	gops->pmupstate = false;
#ifdef CONFIG_TEGRA_ACR
	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		gops->privsecurity = 1;
	} else {
		val = gk20a_readl(g, fuse_opt_priv_sec_en_r());
		if (!val) {
			gk20a_dbg_info("priv security is disabled in HW");
			gops->privsecurity = 0;
		} else {
			gops->privsecurity = 1;
		}
	}
#else
	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		gk20a_dbg_info("running ASIM with PRIV security disabled");
		gops->privsecurity = 0;
	} else {
		val = gk20a_readl(g, fuse_opt_priv_sec_en_r());
		if (!val) {
			gops->privsecurity = 0;
		} else {
			gk20a_dbg_info("priv security is not supported but enabled");
			gops->privsecurity = 1;
			return -EPERM;
		}
	}
#endif
	gk20a_init_bus(gops);
	gm20b_init_mc(gops);
	gk20a_init_priv_ring(gops);
	gm20b_init_gr(gops);
	gm20b_init_fb(gops);
	gm20b_init_fifo(gops);
	gm20b_init_ce2(gops);
	gm20b_init_gr_ctx(gops);
	gm20b_init_mm(gops);
	gm20b_init_pmu_ops(gops);
	gm20b_init_clk_ops(gops);
	gm20b_init_regops(gops);
	gk20a_init_debug_ops(gops);
	gk20a_init_dbg_session_ops(gops);
	gm20b_init_therm_ops(gops);
	gk20a_init_tsg_ops(gops);
#if defined(CONFIG_GK20A_CYCLE_STATS)
	gk20a_init_css_ops(gops);
#endif
	g->name = "gm20b";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = MAXWELL_B;
	c->compute_class = MAXWELL_COMPUTE_B;
	c->gpfifo_class = MAXWELL_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = MAXWELL_DMA_COPY_A;

	return 0;
}
