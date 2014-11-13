/*
 * GP10B Tegra HAL interface
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include "gp10b/fb_gp10b.h"

#include "gm20b/gr_gm20b.h"
#include "gm20b/gm20b_gating_reglist.h"
#include "gm20b/fifo_gm20b.h"
#include "gm20b/gr_ctx_gm20b.h"
#include "gm20b/pmu_gm20b.h"
#include "gm20b/clk_gm20b.h"

struct gpu_ops gp10b_ops = {
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
		.blcg_pmu_load_gating_prod =
			gm20b_blcg_pmu_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gm20b_pg_gr_load_gating_prod,
	}
};

int gp10b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;

	*gops = gp10b_ops;
	gp10b_init_mc(gops);
	gp10b_init_gr(gops);
	gp10b_init_ltc(gops);
	gp10b_init_fb(gops);
	gm20b_init_fifo(gops);
	gm20b_init_gr_ctx(gops);
	gp10b_init_mm(gops);
	gm20b_init_pmu_ops(gops);
	gm20b_init_clk_ops(gops);
	gops->name = "gp10b";

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = PASCAL_A;
	c->compute_class = PASCAL_COMPUTE_A;
	c->gpfifo_class = MAXWELL_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = MAXWELL_DMA_COPY_A;

	return 0;
}
