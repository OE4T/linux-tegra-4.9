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

#include "gk20a/gk20a.h"

#include "clk/clk_arb.h"
#include "clk_arb_gp106.h"

static u32 gp106_get_arbiter_clk_domains(struct gk20a *g)
{
	(void)g;
	return (CTRL_CLK_DOMAIN_MCLK|CTRL_CLK_DOMAIN_GPC2CLK);
}

static int gp106_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u64 *min_hz, u64 *max_hz)
{
	enum nv_pmu_clk_clkwhich clkwhich;
	struct clk_set_info *p0_info;
	struct clk_set_info *p5_info;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_MCLK:
		clkwhich = clkwhich_mclk;
		break;

	case CTRL_CLK_DOMAIN_GPC2CLK:
		clkwhich = clkwhich_gpc2clk;
		break;

	default:
		return -EINVAL;
	}

	p5_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P5, clkwhich);
	if (!p5_info)
		return -EINVAL;

	p0_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P0, clkwhich);
	if (!p0_info)
		return -EINVAL;

	*min_hz = (u64)(p5_info->min_mhz) * (u64)MHZ;
	*max_hz = (u64)(p0_info->max_mhz) * (u64)MHZ;

	return 0;
}

static int gp106_get_arbiter_clk_default(struct gk20a *g, u32 api_domain,
		u64 *default_hz)
{
	enum nv_pmu_clk_clkwhich clkwhich;
	struct clk_set_info *p0_info;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_MCLK:
		clkwhich = clkwhich_mclk;
		break;

	case CTRL_CLK_DOMAIN_GPC2CLK:
		clkwhich = clkwhich_gpc2clk;
		break;

	default:
		return -EINVAL;
	}

	p0_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P0, clkwhich);
	if (!p0_info)
		return -EINVAL;

	*default_hz = (u64)p0_info->max_mhz * (u64)MHZ;

	return 0;
}

void gp106_init_clk_arb_ops(struct gpu_ops *gops)
{
	gops->clk_arb.get_arbiter_clk_domains = gp106_get_arbiter_clk_domains;
	gops->clk_arb.get_arbiter_clk_range = gp106_get_arbiter_clk_range;
	gops->clk_arb.get_arbiter_clk_default = gp106_get_arbiter_clk_default;
}
