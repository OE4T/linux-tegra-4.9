/*
 * drivers/video/tegra/host/gk20a/therm_gk20a.c
 *
 * GK20A Therm
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "gk20a.h"
#include "hw_gr_gk20a.h"
#include "hw_therm_gk20a.h"

static int gk20a_init_therm_reset_enable_hw(struct gk20a *g)
{
	return 0;
}

static int gk20a_init_therm_setup_sw(struct gk20a *g)
{
	return 0;
}

static int gk20a_init_therm_setup_hw(struct gk20a *g)
{
	u32 v;

	/* program NV_THERM registers */
	gk20a_writel(g, therm_use_a_r(), NV_THERM_USE_A_INIT);
	gk20a_writel(g, therm_evt_ext_therm_0_r(),
		NV_THERM_EVT_EXT_THERM_0_INIT);
	gk20a_writel(g, therm_evt_ext_therm_1_r(),
		NV_THERM_EVT_EXT_THERM_1_INIT);
	gk20a_writel(g, therm_evt_ext_therm_2_r(),
		NV_THERM_EVT_EXT_THERM_2_INIT);

	gk20a_writel(g, therm_grad_stepping_table_r(0),
		therm_grad_stepping_table_slowdown_factor0_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by1p5_f()) |
		therm_grad_stepping_table_slowdown_factor1_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by2_f()) |
		therm_grad_stepping_table_slowdown_factor2_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by4_f()) |
		therm_grad_stepping_table_slowdown_factor3_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor4_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()));
	gk20a_writel(g, therm_grad_stepping_table_r(1),
		therm_grad_stepping_table_slowdown_factor0_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor1_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor2_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor3_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()) |
		therm_grad_stepping_table_slowdown_factor4_f(therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()));

	v = gk20a_readl(g, therm_clk_timing_r(0));
	v |= therm_clk_timing_grad_slowdown_enabled_f();
	gk20a_writel(g, therm_clk_timing_r(0), v);

	v = gk20a_readl(g, therm_config2_r());
	v |= therm_config2_grad_enable_f(1);
	v |= therm_config2_slowdown_factor_extended_f(1);
	gk20a_writel(g, therm_config2_r(), v);

	gk20a_writel(g, therm_grad_stepping1_r(),
			therm_grad_stepping1_pdiv_duration_f(32));

	v = gk20a_readl(g, therm_grad_stepping0_r());
	v |= therm_grad_stepping0_feature_enable_f();
	gk20a_writel(g, therm_grad_stepping0_r(), v);

	return 0;
}

int gk20a_init_therm_support(struct gk20a *g)
{
	u32 err;

	gk20a_dbg_fn("");

	err = gk20a_init_therm_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_therm_setup_sw(g);
	if (err)
		return err;

	err = gk20a_init_therm_setup_hw(g);
	if (err)
		return err;

	return err;
}
