/*
 * drivers/video/tegra/dc/dsi2lvds.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "dc_priv.h"

static int tegra_dc_dsi2lvds_init(struct tegra_dc *dc)
{
	/* To be done */
	return 0;
}

static void tegra_dc_dsi2lvds_destroy(struct tegra_dc *dc)
{
	/* To be done */
}

static void tegra_dc_dsi2lvds_enable(struct tegra_dc *dc)
{
	/* To be done */
}

static void tegra_dc_dsi2lvds_disable(struct tegra_dc *dc)
{
	/* To be done */
}

#ifdef CONFIG_PM
static void tegra_dc_dsi2lvds_suspend(struct tegra_dc *dc)
{
	/* To be done */
}

static void tegra_dc_dsi2lvds_resume(struct tegra_dc *dc)
{
	/* To be done */
}
#endif

struct tegra_dc_out_ops tegra_dc_dsi2lvds_ops = {
	.init = tegra_dc_dsi2lvds_init,
	.destroy = tegra_dc_dsi2lvds_destroy,
	.enable = tegra_dc_dsi2lvds_enable,
	.disable = tegra_dc_dsi2lvds_disable,
#ifdef CONFIG_PM
	.suspend = tegra_dc_dsi2lvds_suspend,
	.resume = tegra_dc_dsi2lvds_resume,
#endif
};
