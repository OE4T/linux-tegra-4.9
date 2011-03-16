/*
 * drivers/video/tegra/dc/nvsd.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <mach/dc.h>
#include <linux/types.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "nvsd.h"

static atomic_t *sd_brightness = NULL;

void nvsd_init(struct tegra_dc *dc, struct tegra_dc_sd_settings *settings) {
	u32 i = 0, val = 0;
	/* TODO: check if HW says SD's available */

	/* If SD's not present or disabled, clear the register and return. */
	if (!settings || settings->enable == 0) {
		/* clear the brightness val, too. */
		if (sd_brightness)
			atomic_set(sd_brightness, 255);
		sd_brightness = NULL;

		tegra_dc_writel(dc, 0, DC_DISP_SD_CONTROL);
		return;
	}

	dev_dbg(&dc->ndev->dev, "===================\n");
	dev_dbg(&dc->ndev->dev, "**** NVSD_INIT ****\n");

	/* Write LUT */
	for (i = 0; i < DC_DISP_SD_LUT_NUM; i++) {
		val = 	SD_LUT_R(settings->lut[i].r) |
			SD_LUT_G(settings->lut[i].g) |
			SD_LUT_B(settings->lut[i].b);
		tegra_dc_writel(dc, val, DC_DISP_SD_LUT(i));

		dev_dbg(&dc->ndev->dev, "LUT(%d): 0x%08x\n", i, val);
	}

	/* Write BL TF */
	for (i = 0; i < DC_DISP_SD_BL_TF_NUM; i++) {
		val = 	SD_BL_TF_POINT_0(settings->bltf[i][0]) |
			SD_BL_TF_POINT_1(settings->bltf[i][1]) |
			SD_BL_TF_POINT_2(settings->bltf[i][2]) |
			SD_BL_TF_POINT_3(settings->bltf[i][3]);
		tegra_dc_writel(dc, val, DC_DISP_SD_BL_TF(i));

		dev_dbg(&dc->ndev->dev, "BL_TF(%d): 0x%08x\n", i, val);
	}

	/* Write Coeff */
	val = 	SD_CSC_COEFF_R(settings->coeff.r) |
		SD_CSC_COEFF_G(settings->coeff.g) |
		SD_CSC_COEFF_B(settings->coeff.b);
	tegra_dc_writel(dc, val, DC_DISP_SD_CSC_COEFF);
	dev_dbg(&dc->ndev->dev, "COEFF: 0x%08x\n", val);

	/* Write BL Params */
	val = 	SD_BLP_TIME_CONSTANT(settings->blp.time_constant) |
		SD_BLP_STEP(settings->blp.step);
	tegra_dc_writel(dc, val, DC_DISP_SD_BL_PARAMETERS);
	dev_dbg(&dc->ndev->dev, "BLP: 0x%08x\n", val);

	/* Write Auto/Manual PWM */
	val = (settings->use_auto_pwm) ? SD_BLC_MODE_AUTO : SD_BLC_MODE_MAN;
	tegra_dc_writel(dc, val, DC_DISP_SD_BL_CONTROL);
	dev_dbg(&dc->ndev->dev, "BL_CONTROL: 0x%08x\n", val);

	/* Write Flicker Control */
	val = 	SD_FC_TIME_LIMIT(settings->fc.time_limit) |
		SD_FC_THRESHOLD(settings->fc.threshold);
	tegra_dc_writel(dc, val, DC_DISP_SD_FLICKER_CONTROL);
	dev_dbg(&dc->ndev->dev, "FLICKER_CONTROL: 0x%08x\n", val);

	/* Manage SD Control */
	val = 0;
	/* Enable / One-Shot */
	val |= (settings->enable == 2) ?
			(SD_ENABLE_ONESHOT | SD_ONESHOT_ENABLE) :
			SD_ENABLE_NORMAL;
	/* HW Update Delay */
	val |= SD_HW_UPDATE_DLY(settings->hw_update_delay);
	/* Video Luma */
	val |= (settings->use_vid_luma) ? SD_USE_VID_LUMA : 0;
	/* Aggressiveness */
	val |= SD_AGGRESSIVENESS(settings->aggressiveness);
	/* Bin Width */
	switch (settings->bin_width) {
		default: case 0:
			/* A 0 bin-width indicates 'automatic'
			   based upon aggressiveness. */
			switch (settings->aggressiveness) {
				default: case 0: case 1:
					val |= SD_BIN_WIDTH_ONE;
					break;
				case 2: case 3: case 4:
					val |= SD_BIN_WIDTH_TWO;
					break;
				case 5:
					val |= SD_BIN_WIDTH_FOUR;
					break;
			}
			break;
		case 1: val |= SD_BIN_WIDTH_ONE; break;
		case 2: val |= SD_BIN_WIDTH_TWO; break;
		case 4: val |= SD_BIN_WIDTH_FOUR; break;
		case 8: val |= SD_BIN_WIDTH_EIGHT; break;
	}

	/* TODO: histogram reset WAR? */

	/* Finally, Write SD Control */
	tegra_dc_writel(dc, val, DC_DISP_SD_CONTROL);
	dev_dbg(&dc->ndev->dev, "SD_CONTROL: 0x%08x\n", val);

	/* set the brightness pointer */
	sd_brightness = settings->sd_brightness;
	dev_dbg(&dc->ndev->dev, "sd_brightness: 0x%08x\n", (u32)sd_brightness);

	dev_dbg(&dc->ndev->dev, "*******************\n");
	dev_dbg(&dc->ndev->dev, "===================\n");
}

bool nvsd_update_brightness(struct tegra_dc *dc) {
	u32 val = 0;
	int cur_sd_brightness;

	if (sd_brightness) {
		/* TODO: histogram reset WAR? */

		cur_sd_brightness = atomic_read(sd_brightness);

		/* read brightness value */
		val = tegra_dc_readl(dc, DC_DISP_SD_BL_CONTROL);
		val = SD_BLC_BRIGHTNESS(val);

		if (val != (u32)cur_sd_brightness)
		{
			/* set brightness value and note the update */
			atomic_set(sd_brightness, (int)val);
			return true;
		}
		/* TODO: log? */
	}

	/* No update needed. */
	return false;
}
