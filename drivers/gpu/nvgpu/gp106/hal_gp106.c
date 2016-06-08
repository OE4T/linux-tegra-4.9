/*
 * GP106 HAL interface
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

#include "gp10b/gr_gp10b.h"
#include "gp10b/mc_gp10b.h"
#include "gp10b/ltc_gp10b.h"
#include "gp10b/mm_gp10b.h"
#include "gp10b/ce_gp10b.h"
#include "gp10b/fb_gp10b.h"
#include "gp10b/fifo_gp10b.h"
#include "gp10b/gp10b_gating_reglist.h"
#include "gp10b/regops_gp10b.h"
#include "gp10b/cde_gp10b.h"
#include "gp10b/therm_gp10b.h"

#include "gm206/bios_gm206.h"

#include "gm20b/gr_gm20b.h"
#include "gm20b/fifo_gm20b.h"
#include "gm20b/pmu_gm20b.h"
#include "gm20b/clk_gm20b.h"

#include "gp106/mm_gp106.h"
#include "gp106/pmu_gp106.h"
#include "gp106/gr_ctx_gp106.h"
#include "gp106/gr_gp106.h"
#include "nvgpu_gpuid_t18x.h"
#include "hw_proj_gp106.h"

static struct gpu_ops gp106_ops = {
	.clock_gating = {
		.slcg_bus_load_gating_prod =
			gp10b_slcg_bus_load_gating_prod,
		.slcg_ce2_load_gating_prod =
			gp10b_slcg_ce2_load_gating_prod,
		.slcg_chiplet_load_gating_prod =
			gp10b_slcg_chiplet_load_gating_prod,
		.slcg_ctxsw_firmware_load_gating_prod =
			gp10b_slcg_ctxsw_firmware_load_gating_prod,
		.slcg_fb_load_gating_prod =
			gp10b_slcg_fb_load_gating_prod,
		.slcg_fifo_load_gating_prod =
			gp10b_slcg_fifo_load_gating_prod,
		.slcg_gr_load_gating_prod =
			gr_gp10b_slcg_gr_load_gating_prod,
		.slcg_ltc_load_gating_prod =
			ltc_gp10b_slcg_ltc_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gp10b_slcg_perf_load_gating_prod,
		.slcg_priring_load_gating_prod =
			gp10b_slcg_priring_load_gating_prod,
		.slcg_pmu_load_gating_prod =
			gp10b_slcg_pmu_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gp10b_slcg_therm_load_gating_prod,
		.slcg_xbar_load_gating_prod =
			gp10b_slcg_xbar_load_gating_prod,
		.blcg_bus_load_gating_prod =
			gp10b_blcg_bus_load_gating_prod,
		.blcg_ce_load_gating_prod =
			gp10b_blcg_ce_load_gating_prod,
		.blcg_ctxsw_firmware_load_gating_prod =
			gp10b_blcg_ctxsw_firmware_load_gating_prod,
		.blcg_fb_load_gating_prod =
			gp10b_blcg_fb_load_gating_prod,
		.blcg_fifo_load_gating_prod =
			gp10b_blcg_fifo_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gp10b_blcg_gr_load_gating_prod,
		.blcg_ltc_load_gating_prod =
			gp10b_blcg_ltc_load_gating_prod,
		.blcg_pwr_csb_load_gating_prod =
			gp10b_blcg_pwr_csb_load_gating_prod,
		.blcg_pmu_load_gating_prod =
			gp10b_blcg_pmu_load_gating_prod,
		.blcg_xbar_load_gating_prod =
			gp10b_blcg_xbar_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gp10b_pg_gr_load_gating_prod,
	}
};

static int gp106_get_litter_value(struct gk20a *g,
		enum nvgpu_litter_value value)
{
	int ret = -EINVAL;

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
	default:
		BUG();
		break;
	}

	return ret;
}

int gp106_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;
	u32 ver = g->gpu_characteristics.arch + g->gpu_characteristics.impl;

	gk20a_dbg_fn("");

	*gops = gp106_ops;

	gops->privsecurity = 1;
	gops->securegpccs = 1;

	gp10b_init_mc(gops);
	gp106_init_gr(gops);
	gp10b_init_ltc(gops);
	gp10b_init_fb(gops);
	gp10b_init_fifo(gops);
	gp10b_init_ce(gops);
	gp106_init_gr_ctx(gops);
	gp106_init_mm(gops);
	gp106_init_pmu_ops(gops);
	gk20a_init_debug_ops(gops);
	gp10b_init_regops(gops);
	gp10b_init_cde_ops(gops);
	gp10b_init_therm_ops(gops);
	gm206_init_bios(gops);
	switch(ver){
		case NVGPU_GPUID_GP106:
			gops->name = "gp106";
			break;
		case NVGPU_GPUID_GP104:
			gops->name = "gp104";
			break;
		default:
			gk20a_err(g->dev, "no support for %x", ver);
			BUG();
	}
	gops->get_litter_value = gp106_get_litter_value;
	gops->chip_init_gpu_characteristics = gk20a_init_gpu_characteristics;
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
