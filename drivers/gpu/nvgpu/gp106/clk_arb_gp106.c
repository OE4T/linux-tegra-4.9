/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "gk20a/gk20a.h"

#include "clk/clk_arb.h"
#include "clk_arb_gp106.h"

u32 gp106_get_arbiter_clk_domains(struct gk20a *g)
{
	(void)g;
	return (CTRL_CLK_DOMAIN_MCLK|CTRL_CLK_DOMAIN_GPC2CLK);
}

int gp106_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	enum nv_pmu_clk_clkwhich clkwhich;
	struct clk_set_info *p0_info;
	struct clk_set_info *p5_info;
	struct avfsfllobjs *pfllobjs =  &(g->clk_pmu.avfs_fllobjs);

	u16 limit_min_mhz;

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

	limit_min_mhz = p5_info->min_mhz;
	/* WAR for DVCO min */
	if (api_domain == CTRL_CLK_DOMAIN_GPC2CLK)
		if ((pfllobjs->max_min_freq_mhz) &&
		(pfllobjs->max_min_freq_mhz > limit_min_mhz))
			limit_min_mhz = pfllobjs->max_min_freq_mhz;

	*min_mhz = limit_min_mhz;
	*max_mhz = p0_info->max_mhz;

	return 0;
}

int gp106_get_arbiter_clk_default(struct gk20a *g, u32 api_domain,
		u16 *default_mhz)
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

	*default_mhz = p0_info->max_mhz;

	return 0;
}
