/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <soc/tegra/tegra_bpmp.h>

#include "reset-t186.h"
#include "reset.h"

static int tegra18x_reset_probe(struct platform_device *pdev)
{
	return bpmp_register_reset(TEGRA186_RESET_SIZE, pdev);
}

static int tegra18x_reset_remove(struct platform_device *pdev)
{
	return bpmp_reset_remove(pdev);
}

static const struct of_device_id tegra18x_reset_match[] = {
	{ .compatible = "nvidia,tegra18x-car", },
	{ },
};

static struct platform_driver tegra18x_reset_driver = {
	.probe = tegra18x_reset_probe,
	.remove = tegra18x_reset_remove,
	.driver = {
		.name		= "tegra18x-reset",
		.owner		= THIS_MODULE,
		.of_match_table	= tegra18x_reset_match,
	},
};

module_platform_driver(tegra18x_reset_driver);

MODULE_AUTHOR("Peter De Schrijver <pdeschrijver@nvidia.com>");
MODULE_DESCRIPTION("Tegra18x Controller Driver");
MODULE_LICENSE("GPL");
