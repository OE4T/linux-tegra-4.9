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

#include "camera/vi/mc_common.h"
#include "camera/csi/csi.h"
#include "vi/vi4.h"
#include "nvcsi/nvcsi.h"
#include "host1x/host1x.h"

/* PG generate 1 pixel per nvcsi_clk:
 * ((width + hblank) * height + vblank) * fps = nvcsi_clk_freq
 *
 */
const struct tpg_frmfmt tegra18x_csi_tpg_frmfmt[] = {
	{{1280, 720}, V4L2_PIX_FMT_SRGGB10, 120, 750, 0xffff},
	{{1920, 1080}, V4L2_PIX_FMT_SRGGB10, 60, 930, 0xffff},
	{{3840, 2160}, V4L2_PIX_FMT_SRGGB10, 20, 900, 0xffff},
	{{1280, 720}, V4L2_PIX_FMT_RGB32, 60, 3351, 0xffff},
	{{1920, 1080}, V4L2_PIX_FMT_RGB32, 30, 4315, 0xffff},
	{{3840, 2160}, V4L2_PIX_FMT_RGB32, 20, 851, 0xffff},
};

static int tpg_probe_t18x(void)
{
	struct tegra_csi_device *mc_csi = tegra_get_mc_csi();
	struct tegra_mc_vi *mc_vi = tegra_get_mc_vi();
	int err;

	dev_dbg(mc_csi->dev, "%s\n", __func__);
	mc_vi->csi = mc_csi;
	/* Init CSI related media controller interface */
	mc_csi->tpg_frmfmt_table = tegra18x_csi_tpg_frmfmt;
	mc_csi->tpg_frmfmt_table_size = ARRAY_SIZE(tegra18x_csi_tpg_frmfmt);
	err = tpg_csi_media_controller_init(mc_csi, TEGRA_VI_PG_PATCH);
	if (err)
		return -EINVAL;
	err = tpg_vi_media_controller_init(mc_vi, TEGRA_VI_PG_PATCH);
	if (err)
		goto vi_init_err;
	return err;
vi_init_err:
	tpg_csi_media_controller_cleanup(mc_csi);
	dev_err(mc_csi->dev, "%s error\n", __func__);
	return err;
}
static int __exit tpg_remove_t18x(void)
{
	struct tegra_csi_device *mc_csi = tegra_get_mc_csi();
	struct tegra_mc_vi *mc_vi = tegra_get_mc_vi();

	tpg_csi_media_controller_cleanup(mc_csi);
	tpg_vi_media_controller_cleanup(mc_vi);

	return 0;
}

static int __init tpg_init(void)
{
	return tpg_probe_t18x();
}

static void __exit tpg_exit(void)
{
	tpg_remove_t18x();
}

module_init(tpg_init);
module_exit(tpg_exit);
MODULE_LICENSE("GPL v2");
