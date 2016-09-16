/*
 * general p state infrastructure
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

#include "gk20a/gk20a.h"
#include "clk/clk.h"
#include "perf/perf.h"

/*sw setup for pstate components*/
int gk20a_init_pstate_support(struct gk20a *g)
{
	u32 err;

	gk20a_dbg_fn("");

	err = clk_vin_sw_setup(g);
	if (err)
		return err;

	err = clk_fll_sw_setup(g);
	if (err)
		return err;

	err = vfe_var_sw_setup(g);
	if (err)
		return err;

	err = vfe_equ_sw_setup(g);
	if (err)
		return err;

	err = clk_domain_sw_setup(g);
	if (err)
		return err;

	err = clk_vf_point_sw_setup(g);
	if (err)
		return err;

	err = clk_prog_sw_setup(g);
	return err;
}

/*sw setup for pstate components*/
int gk20a_init_pstate_pmu_support(struct gk20a *g)
{
	u32 err;

	gk20a_dbg_fn("");

	err = vfe_var_pmu_setup(g);
	if (err)
		return err;

	err = vfe_equ_pmu_setup(g);
	if (err)
		return err;

	err = clk_domain_pmu_setup(g);
	if (err)
		return err;

	err = clk_prog_pmu_setup(g);
	if (err)
		return err;

	err = clk_vin_pmu_setup(g);
	if (err)
		return err;

	err = clk_fll_pmu_setup(g);
	if (err)
		return err;

	err = clk_vf_point_pmu_setup(g);
	if (err)
		return err;

	err = clk_pmu_vin_load(g);
	if (err)
		return err;

	err = perf_pmu_vfe_load(g);
	if (err)
		return err;

	err = clk_vf_point_cache(g);
	if (err)
		return err;

	err = clk_set_boot_fll_clk(g);
	return err;
}

