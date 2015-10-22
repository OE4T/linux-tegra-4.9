/*
 * drivers/gpu/nvgpu/gm20b/therm_gk20a.c
 *
 * GP10B Therm
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#include "hw_therm_gp10b.h"

static int gp10b_init_therm_setup_hw(struct gk20a *g)
{
	gk20a_dbg_fn("");

	/* program NV_THERM registers */
	gk20a_writel(g, therm_use_a_r(), therm_use_a_ext_therm_0_enable_f() |
					therm_use_a_ext_therm_1_enable_f()  |
					therm_use_a_ext_therm_2_enable_f());
	gk20a_writel(g, therm_evt_ext_therm_0_r(),
		therm_evt_ext_therm_0_slow_factor_f(
			therm_evt_ext_therm_0_slow_factor_init_v()));
	gk20a_writel(g, therm_evt_ext_therm_1_r(),
		therm_evt_ext_therm_1_slow_factor_f(
			therm_evt_ext_therm_1_slow_factor_init_v()));
	gk20a_writel(g, therm_evt_ext_therm_2_r(),
		therm_evt_ext_therm_2_slow_factor_f(
			therm_evt_ext_therm_2_slow_factor_init_v()));

	return 0;
}

void gp10b_init_therm_ops(struct gpu_ops *gops)
{
	gops->therm.init_therm_setup_hw = gp10b_init_therm_setup_hw;

}
