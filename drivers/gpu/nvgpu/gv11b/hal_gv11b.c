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

#include "gk20a/gk20a.h"

#include "gv11b/hal_gv11b.h"
#include "gv11b/gr_gv11b.h"
#include "gv11b/mc_gv11b.h"
#include "gv11b/ltc_gv11b.h"
#include "gv11b/fecs_trace_gv11b.h"
#include "gv11b/gv11b.h"
#include "gv11b/ce2_gv11b.h"
#include "gv11b/gr_ctx_gv11b.h"
#include "gv11b/mm_gv11b.h"
#include "gv11b/pmu_gv11b.h"
#include "gv11b/therm_gv11b.h"

#include "gm20b/gr_gm20b.h"

int gv11b_init_hal(struct gk20a *g)
{
	struct gpu_ops *gops = &g->ops;
	struct nvgpu_gpu_characteristics *c = &g->gpu_characteristics;

	/* boot in non-secure modes for time beeing */
	gops->privsecurity = 0;
	gops->securegpccs = 0;

	gv11b_init_mc(gops);
	gv11b_init_ltc(gops);
	gv11b_init_gr(gops);
	gv11b_init_fecs_trace_ops(gops);
	gv11b_init_ce2(gops);
	gv11b_init_mm(gops);
	gv11b_init_gr_ctx(gops);
	gv11b_init_pmu_ops(gops);
        gk20a_init_debug_ops(gops);
	gv11b_init_therm_ops(gops);
	gops->name = "gv11b";
	gops->chip_init_gpu_characteristics = gv11b_init_gpu_characteristics;

	c->twod_class = FERMI_TWOD_A;
	c->threed_class = VOLTA_A;
	c->compute_class = VOLTA_COMPUTE_A;
	c->gpfifo_class = VOLTA_CHANNEL_GPFIFO_A;
	c->inline_to_memory_class = KEPLER_INLINE_TO_MEMORY_B;
	c->dma_copy_class = VOLTA_DMA_COPY_A;

	return 0;
}
