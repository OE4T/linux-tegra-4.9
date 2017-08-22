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

#include <soc/tegra/tegra-dvfs.h>

#include "clk.h"
#include "os_linux.h"

#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"

static unsigned long nvgpu_linux_clk_get_rate(struct gk20a *g, u32 api_domain)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
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
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
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

static unsigned long nvgpu_linux_get_fmax_at_vmin_safe(struct clk_gk20a *clk)
{
	/*
	 * On Tegra GPU clock exposed to frequency governor is a shared user on
	 * GPCPLL bus (gbus). The latter can be accessed as GPU clock parent.
	 * Respectively the grandparent is PLL reference clock.
	 */
	return tegra_dvfs_get_fmax_at_vmin_safe_t(
			clk_get_parent(clk->tegra_clk));
}

static u32 nvgpu_linux_get_ref_clock_rate(struct gk20a *g)
{
	struct clk *c;

	c = clk_get_sys("gpu_ref", "gpu_ref");
	if (IS_ERR(c)) {
		nvgpu_err(g, "failed to get GPCPLL reference clock");
		return 0;
	}

	return clk_get_rate(c);
}

static int nvgpu_linux_predict_mv_at_hz_cur_tfloor(struct clk_gk20a *clk,
	unsigned long rate)
{
	return tegra_dvfs_predict_mv_at_hz_cur_tfloor(
				clk_get_parent(clk->tegra_clk), rate);
}

static unsigned long nvgpu_linux_get_maxrate(struct clk_gk20a *clk)
{
	return tegra_dvfs_get_maxrate(clk_get_parent(clk->tegra_clk));
}

static int nvgpu_linux_prepare_enable(struct clk_gk20a *clk)
{
	return clk_prepare_enable(clk->tegra_clk);
}

static void nvgpu_linux_disable_unprepare(struct clk_gk20a *clk)
{
	clk_disable_unprepare(clk->tegra_clk);
}

void nvgpu_linux_init_clk_support(struct gk20a *g)
{
	g->ops.clk.get_rate = nvgpu_linux_clk_get_rate;
	g->ops.clk.set_rate = nvgpu_linux_clk_set_rate;
	g->ops.clk.get_fmax_at_vmin_safe = nvgpu_linux_get_fmax_at_vmin_safe;
	g->ops.clk.get_ref_clock_rate = nvgpu_linux_get_ref_clock_rate;
	g->ops.clk.predict_mv_at_hz_cur_tfloor = nvgpu_linux_predict_mv_at_hz_cur_tfloor;
	g->ops.clk.get_maxrate = nvgpu_linux_get_maxrate;
	g->ops.clk.prepare_enable = nvgpu_linux_prepare_enable;
	g->ops.clk.disable_unprepare = nvgpu_linux_disable_unprepare;
}
