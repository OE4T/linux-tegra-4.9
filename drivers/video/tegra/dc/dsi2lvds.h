/*
 * drivers/video/tegra/dc/dsi2lvds.h
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DSI2LVDS_H
#define __DRIVERS_VIDEO_TEGRA_DC_DSI2LVDS_H

struct tegra_dc_dsi2lvds_data {
	struct tegra_dc *dc;
	struct tegra_dc_dsi_data *dsi;
	struct i2c_client *client_i2c;

	struct mutex lock;
};

extern inline void *tegra_dc_dsi_get_outdata(struct tegra_dc *);
extern inline void tegra_dc_dsi_set_outdata(struct tegra_dc *, void*);

#endif
