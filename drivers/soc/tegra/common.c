/*

 * Copyright (C) 2014-2017 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>

#include <soc/tegra/common.h>

/* Before T18x architecture */
static const struct of_device_id tegra210_le_machine_match[] = {
	{ .compatible = "nvidia,tegra20", },
	{ .compatible = "nvidia,tegra30", },
	{ .compatible = "nvidia,tegra114", },
	{ .compatible = "nvidia,tegra124", },
	{ .compatible = "nvidia,tegra132", },
	{ .compatible = "nvidia,tegra210", },
	{ }
};

/* T186 and later architecture */
static const struct of_device_id tegra186_ge_machine_match[] = {
	{ .compatible = "nvidia,tegra186", },
	{ .compatible = "nvidia,tegra194", },
	{ }
};

bool soc_is_tegra210_n_before(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/");
	if (!root)
		return false;

	return of_match_node(tegra210_le_machine_match, root) != NULL;
}

bool soc_is_tegra186_n_later(void)
{
	struct device_node *root;

	root = of_find_node_by_path("/");
	if (!root)
		return false;

	return of_match_node(tegra186_ge_machine_match, root) != NULL;
}
