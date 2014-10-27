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
	/* program NV_THERM registers */
	gk20a_writel(g, therm_use_a_r(), NV_THERM_USE_A_INIT);
	gk20a_writel(g, therm_evt_ext_therm_0_r(),
		NV_THERM_EVT_EXT_THERM_0_INIT);
	gk20a_writel(g, therm_evt_ext_therm_1_r(),
		NV_THERM_EVT_EXT_THERM_1_INIT);
	gk20a_writel(g, therm_evt_ext_therm_2_r(),
		NV_THERM_EVT_EXT_THERM_2_INIT);

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
