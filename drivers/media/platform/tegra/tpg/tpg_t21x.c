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
#include "camera/csi/csi.h"
#include "host1x/host1x.h"
#define TPG_CHANNELS 6

/*
 * total_cycles_per_lane for one second = pll_d freq / 8
 * width_in_bytes = ((width * bpp) / 8)
 * cycles_per_line = width_in_bytes + hblank
 * cycles_per_image = (cycles_per_line * height) + vblank
 * image_cycles_per_lane = cycles_per_image / numlanes
 * framerate = total_cycles_per_lane / image_cycles_per_lane
 * As per IAS maximum overhead of ~15% can occur
 * hblank and vblank are tuned to consider overhead during capture
 * e.g. for 1920x1080, RAW 10 and two lane TPG
 * cycles_per_lane = (((((1920 * 10)/8) + 512) * 1080) + 8) / 2 ~ 1572480
 * framerate = ((927000000 / 8) / 1572480) ~ 73fps
 * Max overhead of 15% results in minimum of 62fps (max can be 73fps)
 * Note: with changing resolution, bpp and hblank overhead % varies.
 */

static struct tpg_frmfmt tegra21x_csi_tpg_frmfmt[] = {
	{{1280, 720}, V4L2_PIX_FMT_SRGGB10, 120, 512, 8},
	{{1920, 1080}, V4L2_PIX_FMT_SRGGB10, 60, 512, 8},
	{{3840, 2160}, V4L2_PIX_FMT_SRGGB10, 20, 8, 8},
	{{1280, 720}, V4L2_PIX_FMT_RGB32, 60, 512, 8},
	{{1920, 1080}, V4L2_PIX_FMT_RGB32, 30, 512, 8},
	{{3840, 2160}, V4L2_PIX_FMT_RGB32, 8, 8, 8},
};


static struct tegra_csi_device tpg_csi;
static struct tegra_mc_vi tpg_mc_vi;

static int tpg_probe(struct vi *tegra_vi)
{
	struct tegra_csi_device *csi = &tpg_csi;
	struct tegra_mc_vi *mc_vi = &tpg_mc_vi;
	struct platform_device *pdev = tegra_vi->ndev;
	int ret, i;

	/* Init CSI related media controller interface */
	csi->num_ports = TPG_CHANNELS;
	csi->pg_mode = TEGRA_VI_PG_PATCH;
	csi->tpg_frmfmt_table = tegra21x_csi_tpg_frmfmt;
	csi->tpg_frmfmt_table_size = ARRAY_SIZE(tegra21x_csi_tpg_frmfmt);
	ret = tegra_csi_media_controller_init(csi, pdev);
	if (ret)
		return ret;

	/* Init Tegra VI TPG channels */
	mc_vi->vi = tegra_vi;
	mc_vi->csi = csi;
	mc_vi->reg = tegra_vi->reg;
	mc_vi->pg_mode = TEGRA_VI_PG_PATCH;
	mc_vi->num_channels = TPG_CHANNELS;
	ret = tegra_vi_media_controller_init(mc_vi, pdev);
	if (ret)
		goto vi_mc_error;

	/* for TPG bind the subdev with v4l2 device explicitly */
	/* usually this happens during notifier registration */
	for (i = 0; i < csi->num_channels; i++) {
		ret = v4l2_device_register_subdev(&mc_vi->v4l2_dev,
				&csi->chans[i].subdev);
		if (ret)
			goto v4l2_reg_error;
	}

	ret = v4l2_device_register_subdev_nodes(&mc_vi->v4l2_dev);
	if (ret < 0)
		goto v4l2_reg_error;

	return 0;

v4l2_reg_error:
	tegra_vi_media_controller_cleanup(mc_vi);
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
