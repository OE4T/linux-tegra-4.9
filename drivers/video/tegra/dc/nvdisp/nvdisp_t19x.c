/*
 * drivers/video/tegra/dc/nvdisp/nvdisp_t19x.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
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

void tegra_dc_populate_t19x_hw_data(struct tegra_dc_hw_data *hw_data)
{
	if (!hw_data)
		return;

	hw_data->nheads = 4;
	hw_data->nwins = 6;
	hw_data->nsors = 4;
	hw_data->valid = true;
	hw_data->version = TEGRA_DC_HW_T19x;
}
