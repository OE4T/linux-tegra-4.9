/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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

#include "clk-group.h"

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/clk.h>

struct camrtc_clk_group {
	struct device *device;
	int nclocks;
	struct clk **clocks;
	struct {
		u32 slow;
		u32 fast;
	} *rates;
};

static void camrtc_clk_group_release(struct device *dev, void *res)
{
	const struct camrtc_clk_group *grp = res;
	int i;

	for (i = 0; i < grp->nclocks; i++) {
		if (grp->clocks[i])
			clk_put(grp->clocks[i]);
	}
}

struct camrtc_clk_group *camrtc_clk_group_get(
	struct device *dev)
{
	struct camrtc_clk_group *grp;
	struct device_node *np;
	int nclocks;
	int nrates;
	int index;
	int ret;

	if (!dev || !dev->of_node)
		return ERR_PTR(-EINVAL);

	np = dev->of_node;

	nclocks = of_property_count_strings(np, "clock-names");
	if (nclocks < 0)
		return ERR_PTR(-ENOENT);

	nrates = of_property_count_u64_elems(np, "nvidia,clock-rates");

	grp = devres_alloc(camrtc_clk_group_release,
			sizeof(*grp) +
			nclocks * sizeof(grp->clocks[0]) +
			nclocks * sizeof(grp->rates[0]),
			GFP_KERNEL);
	if (!grp)
		return ERR_PTR(-ENOMEM);

	grp->nclocks = nclocks;
	grp->device = dev;
	grp->clocks = (struct clk **)(grp + 1);
	grp->rates = (void *)(grp->clocks + nclocks);

	for (index = 0; index < grp->nclocks; index++) {
		struct clk *clk;

		clk = of_clk_get(np, index);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			goto error;
		}

		grp->clocks[index] = clk;

		if (index >= nrates)
			continue;

		of_property_read_u32_index(np, "nvidia,clock-rates",
					2 * index, &grp->rates[index].slow);
		of_property_read_u32_index(np, "nvidia,clock-rates",
					2 * index + 1, &grp->rates[index].fast);
	}

	devres_add(dev, grp);
	return grp;

error:
	devres_free(grp);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(camrtc_clk_group_get);

static void camrtc_clk_group_error(
	const struct camrtc_clk_group *grp,
	char const *op,
	int index,
	int error)
{
	const char *name = "unnamed";

	of_property_read_string_index(grp->device->of_node,
				"clock-names", index, &name);
	dev_WARN(grp->device, "%s clk %s (at [%d]): failed (%d)\n",
				op, name, index, error);
}

int camrtc_clk_group_enable(const struct camrtc_clk_group *grp)
{
	int index, err;

	if (IS_ERR_OR_NULL(grp))
		return -ENODEV;

	for (index = 0; index < grp->nclocks; index++) {
		err = clk_prepare_enable(grp->clocks[index]);
		if (err) {
			camrtc_clk_group_error(grp, "enable", index, err);
			return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(camrtc_clk_group_enable);

void camrtc_clk_group_disable(const struct camrtc_clk_group *grp)
{
	int index;

	if (IS_ERR_OR_NULL(grp))
		return;

	for (index = 0; index < grp->nclocks; index++)
		clk_disable_unprepare(grp->clocks[index]);
}
EXPORT_SYMBOL_GPL(camrtc_clk_group_disable);

int camrtc_clk_group_adjust_slow(const struct camrtc_clk_group *grp)
{
	int index;

	if (IS_ERR_OR_NULL(grp))
		return -ENODEV;

	for (index = 0; index < grp->nclocks; index++) {
		u32 slow = grp->rates[index].slow;

		if (slow != 0)
			clk_set_rate(grp->clocks[index], slow);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(camrtc_clk_group_adjust_slow);

int camrtc_clk_group_adjust_fast(const struct camrtc_clk_group *grp)
{
	int index;

	if (IS_ERR_OR_NULL(grp))
		return -ENODEV;

	for (index = 0; index < grp->nclocks; index++) {
		u32 fast = grp->rates[index].fast;

		if (fast != 0)
			clk_set_rate(grp->clocks[index], fast);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(camrtc_clk_group_adjust_fast);
