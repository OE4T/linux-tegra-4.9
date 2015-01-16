/*
 * FPDLink Serializer driver
 *
 * Copyright (C) 2014-2015 NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DRIVERS_VIDEO_TEGRA_DC_HDMI2FPD_H
#define __DRIVERS_VIDEO_TEGRA_DC_HDMI2FPD_H

#include <linux/types.h>

#define DS90UH949_SER_REG_RESET			0x01
#define DS90UH949_SER_REG_RESET_DIGRESET0		BIT(0)
#define DS90UH949_SER_REG_RESET_DIGRESET1		BIT(1)

struct tegra_dc_hdmi2fpd_data {
	int en_gpio; /* GPIO */
	int en_gpio_flags;
	int power_on_delay;
	int power_off_delay;
	bool hdmi2fpd_enabled;
	struct tegra_dc_hdmi_data *hdmi;
	struct regmap *regmap;
	struct mutex lock;
/* TODO: have configuration parameters like HDCP support etc.. */
};

#endif /* __DRIVERS_VIDEO_TEGRA_DC_HDMI2FPD_H */
