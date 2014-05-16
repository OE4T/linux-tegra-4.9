/*
 * drivers/video/tegra/dc/hdmi2.0.h
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_HDMI2_0_H__
#define __DRIVERS_VIDEO_TEGRA_DC_HDMI2_0_H__

struct tegra_hdmi {
	struct tegra_dc *dc;
	struct tegra_hdmi_out *pdata;
	bool enabled;
	struct tegra_dc_sor_data *sor;
};

#endif

