/*
 * GV11B Tegra HAL interface
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
#include <linux/tegra_gpu_t19x.h>
#include "gk20a/gk20a.h"

#include "gv11b/hal_gv11b.h"
#include "gv11b/gr_gv11b.h"
#include "gv11b/mc_gv11b.h"
#include "gv11b/ltc_gv11b.h"
#include "gv11b/fecs_trace_gv11b.h"
#include "gv11b/gv11b.h"
#include "gv11b/ce_gv11b.h"
#include "gv11b/gr_ctx_gv11b.h"
#include "gv11b/mm_gv11b.h"
#include "gv11b/pmu_gv11b.h"
#include "gv11b/therm_gv11b.h"
#include "gv11b/fb_gv11b.h"
#include "gv11b/fifo_gv11b.h"
#include "gv11b/hw_proj_gv11b.h"

#include "gm20b/gr_gm20b.h"
#include "gk20a/dbg_gpu_gk20a.h"

static struct gpu_ops gv11b_ops;

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
	case GPU_LIT_NUM_SUBCTX:
		ret = proj_litter_num_subctx_v();
		break;
	default:
		break;
	}

	return ret;
}

int gv11b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;

	*gops = gv11b_ops;

	/* boot in non-secure modes for time beeing */
	gops->privsecurity = 0;
	gops->securegpccs = 0;

	gv11b_init_mc(gops);
	gv11b_init_ltc(gops);
	gv11b_init_gr(gops);
	gv11b_init_fecs_trace_ops(gops);
	gv11b_init_fb(gops);
	gv11b_init_fifo(gops);
	gv11b_init_ce(gops);
	gv11b_init_gr_ctx(gops);
	gv11b_init_mm(gops);
	gv11b_init_pmu_ops(gops);
	gk20a_init_debug_ops(gops);
	gk20a_init_dbg_session_ops(gops);
	gv11b_init_therm_ops(gops);
	gk20a_init_tsg_ops(gops);
	gops->name = "gv11b";
	gops->chip_init_gpu_characteristics = gv11b_init_gpu_characteristics;
	gops->get_litter_value = gv11b_get_litter_value;
	gops->read_ptimer = gk20a_read_ptimer;

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = VOLTA_A;
	c->compute_class = VOLTA_COMPUTE_A;
	c->gpfifo_class = VOLTA_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = VOLTA_DMA_COPY_A;

	return 0;
}
