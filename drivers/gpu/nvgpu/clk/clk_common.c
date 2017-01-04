/*
 * Copyright (c) 2011-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

struct clk *gk20a_clk_get(struct gk20a *g)
{
	if (!g->clk.tegra_clk) {
		struct clk *clk;
		char clk_dev_id[32];
		struct device *dev = dev_from_gk20a(g);

		snprintf(clk_dev_id, 32, "tegra_%s", dev_name(dev));

		clk = clk_get_sys(clk_dev_id, "gpu");
		if (IS_ERR(clk)) {
			gk20a_err(dev, "fail to get tegra gpu clk %s/gpu\n",
				  clk_dev_id);
			return NULL;
		}
		g->clk.tegra_clk = clk;
	}

	return g->clk.tegra_clk;
}

unsigned long gk20a_clk_get_rate(struct gk20a *g)
{
	struct clk_gk20a *clk = &g->clk;
	return rate_gpc2clk_to_gpu(clk->gpc_pll.freq);
}

long gk20a_clk_round_rate(struct gk20a *g, unsigned long rate)
{
	/* make sure the clock is available */
	if (!gk20a_clk_get(g))
		return rate;

	return clk_round_rate(clk_get_parent(g->clk.tegra_clk), rate);
}

int gk20a_clk_set_rate(struct gk20a *g, unsigned long rate)
{
	return clk_set_rate(g->clk.tegra_clk, rate);
}

