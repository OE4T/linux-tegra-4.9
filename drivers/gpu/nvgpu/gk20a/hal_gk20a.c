/*
 * drivers/video/tegra/host/gk20a/hal_gk20a.c
 *
 * GK20A Tegra HAL interface.
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include "hal_gk20a.h"
#include "ltc_gk20a.h"
#include "fb_gk20a.h"
#include "gk20a.h"
#include "gk20a_gating_reglist.h"
#include "channel_gk20a.h"
#include "gr_ctx_gk20a.h"
#include "mm_gk20a.h"
#include "mc_gk20a.h"
#include "pmu_gk20a.h"
#include "clk_gk20a.h"
#include "regops_gk20a.h"

static struct gpu_ops gk20a_ops = {
	.clock_gating = {
		.slcg_gr_load_gating_prod =
			gr_gk20a_slcg_gr_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gr_gk20a_slcg_perf_load_gating_prod,
		.slcg_ltc_load_gating_prod =
			ltc_gk20a_slcg_ltc_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gr_gk20a_blcg_gr_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gk20a_pg_gr_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gr_gk20a_slcg_therm_load_gating_prod,
	},
};

int gk20a_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;

	*gops = gk20a_ops;
	gops->privsecurity = 0;
	gops->securegpccs = 0;
	gk20a_init_mc(gops);
	gk20a_init_ltc(gops);
	gk20a_init_gr_ops(gops);
	gk20a_init_fb(gops);
	gk20a_init_fifo(gops);
	gk20a_init_ce2(gops);
	gk20a_init_gr_ctx(gops);
	gk20a_init_mm(gops);
	gk20a_init_pmu_ops(gops);
	gk20a_init_clk_ops(gops);
	gk20a_init_regops(gops);
	gk20a_init_debug_ops(gops);
	gops->name = "gk20a";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = KEPLER_C;
	c->compute_class = KEPLER_COMPUTE_A;
	c->gpfifo_class = KEPLER_CHANNEL_GPFIFO_C;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_A;
	c->dma_copy_class = KEPLER_DMA_COPY_A;

	return 0;
}
