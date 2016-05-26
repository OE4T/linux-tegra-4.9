/*
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

#include <linux/delay.h>	/* for udelay */
#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"

#include "gp10b/pmu_gp10b.h"
#include "hw_mc_gp106.h"
#include "hw_pwr_gp106.h"

static int gp106_pmu_reset(struct gk20a *g)
{
	gk20a_dbg_fn("");

	gk20a_reset(g, mc_enable_pwr_enabled_f());

	gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_true_f());
	udelay(10);
	gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_false_f());

	gk20a_dbg_fn("done");
	return 0;
}

void gp106_init_pmu_ops(struct gpu_ops *gops)
{
	gk20a_dbg_fn("");

	gp10b_init_pmu_ops(gops);
	gops->pmu.reset = gp106_pmu_reset;

	gk20a_dbg_fn("done");
}
