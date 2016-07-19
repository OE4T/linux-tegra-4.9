/*
 * drivers/video/tegra/host/tpg/tpg.c
 *
 * Tegra VI test pattern generator driver
 *
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>

#include "vi/vi.h"
#include "csi/csi.h"

#define TPG_CHANNELS 6

static struct tegra_csi_device tpg_csi;
static struct tegra_mc_vi tpg_mc_vi;

static int tpg_probe(struct vi *tegra_vi)
{
	struct tegra_csi_device *csi = &tpg_csi;
	struct tegra_mc_vi *mc_vi = &tpg_mc_vi;
	struct platform_device *pdev = tegra_vi->ndev;
	int ret;

	/* Init CSI related media controller interface */
	csi->num_ports = TPG_CHANNELS;
	csi->pg_mode = TEGRA_VI_PG_PATCH;
	csi->vi = tegra_vi;
	ret = tegra_csi_media_controller_init(csi, pdev);
	if (ret)
		return ret;

	/* Init Tegra VI TPG channels */
	mc_vi->vi = tegra_vi;
	mc_vi->csi = csi;
	mc_vi->reg = tegra_vi->reg;
	mc_vi->pg_mode = TEGRA_VI_PG_PATCH;
	mc_vi->num_channels = TPG_CHANNELS;
	mc_vi->fops = tegra_vi->data->vi_fops;
	ret = tegra_vi_media_controller_init(mc_vi, pdev);
	if (ret)
		goto vi_mc_error;
	return 0;

vi_mc_error:
	tegra_csi_media_controller_remove(csi);
	return ret;
}

static int __exit tpg_remove(struct vi *tegra_vi)
{
	struct tegra_csi_device *csi = &tpg_csi;
	struct tegra_mc_vi *mc_vi = &tpg_mc_vi;

	tegra_csi_media_controller_remove(csi);
	tegra_vi_channels_cleanup(mc_vi);
	tegra_vi_v4l2_cleanup(mc_vi);

	return 0;
}

static int __init tpg_init(void)
{
	struct vi *tegra_vi = tegra_vi_get();
	if (tegra_vi)
		return tpg_probe(tegra_vi);

	return 0;
}

static void __exit tpg_exit(void)
{
	struct vi *tegra_vi = tegra_vi_get();
	if (tegra_vi)
		tpg_remove(tegra_vi);
}

module_init(tpg_init);
module_exit(tpg_exit);
MODULE_LICENSE("GPL v2");
