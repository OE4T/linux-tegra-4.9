/*
 * Linux clock support
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>

#include "clk.h"
#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"

static unsigned long nvgpu_linux_clk_get_rate(struct gk20a *g, u32 api_domain)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	unsigned long ret;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		if (g->clk.tegra_clk)
			ret = clk_get_rate(g->clk.tegra_clk);
		else
			ret = clk_get_rate(platform->clk[0]);
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		ret = clk_get_rate(platform->clk[1]);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = 0;
		break;
	}

	return ret;
}

static int nvgpu_linux_clk_set_rate(struct gk20a *g,
				     u32 api_domain, unsigned long rate)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	int ret;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPCCLK:
		if (g->clk.tegra_clk)
			ret = clk_set_rate(g->clk.tegra_clk, rate);
		else
			ret = clk_set_rate(platform->clk[0], rate);
		break;
	case CTRL_CLK_DOMAIN_PWRCLK:
		ret = clk_set_rate(platform->clk[1], rate);
		break;
	default:
		nvgpu_err(g, "unknown clock: %u", api_domain);
		ret = -EINVAL;
		break;
	}

	return ret;
}

void nvgpu_linux_init_clk_support(struct gk20a *g)
{
	g->ops.clk.get_rate = nvgpu_linux_clk_get_rate;
	g->ops.clk.set_rate = nvgpu_linux_clk_set_rate;
}
