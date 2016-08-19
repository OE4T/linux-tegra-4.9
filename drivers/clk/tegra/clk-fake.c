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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include "clk-tegra.h"

struct fclk {
	struct clk_hw hw;
	unsigned long rate;
};

static int fclk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct fclk *fclk = container_of(hw, struct fclk, hw);

	fclk->rate = rate;

	return 0;
}

static unsigned long fclk_get_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct fclk *fclk = container_of(hw, struct fclk, hw);

	return fclk->rate;
}

static long fclk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	return rate;
}

static const struct clk_ops fclk_ops = {
	.set_rate = fclk_set_rate,
	.recalc_rate = fclk_get_rate,
	.round_rate = fclk_round_rate
};

static struct clk_onecell_data clk_data;

int tegra_fake_clks_init(struct device_node *np)
{
	const int num_clks = 1023;
	struct clk_init_data init;
	struct fclk *fclks;
	struct clk **pclks;
	char name[32];
	const size_t sz = sizeof(name);
	int i;
	int r;

	fclks = kcalloc(num_clks + 1, sizeof(*fclks), GFP_KERNEL);
	if (!fclks)
		return -ENOMEM;

	pclks = kcalloc(num_clks + 1, sizeof(*pclks), GFP_KERNEL);
	if (!pclks) {
		kfree(fclks);
		return -ENOMEM;
	}

	init.name = name;
	init.flags = 0;
	init.num_parents = 0;
	init.parent_names = NULL;
	init.ops = &fclk_ops;

	pclks[0] = ERR_PTR(-EINVAL);

	for (i = 1; i <= num_clks; i++) {
		snprintf(name, sz, "fclk.%d", i);
		name[sz - 1] = 0;
		fclks[i].hw.init = &init;

		pclks[i] = clk_register(NULL, &fclks[i].hw);
		if (IS_ERR_OR_NULL(pclks[i])) {
			r = PTR_ERR(pclks[i]);
			pr_err("failed to register clk %s (%d)\n", name, r);
			goto err_out;
		}
	}

	clk_data.clks = pclks;
	clk_data.clk_num = num_clks;

	r = of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
	if (!r)
		return 0;

err_out:
	kfree(fclks);
	kfree(pclks);

	return r;
}
