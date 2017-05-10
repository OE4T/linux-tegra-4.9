/*
 * crc.c: CRC functions for tegradc EXT device
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include "dc.h"
#include <linux/errno.h>

long tegra_dc_crc_enable(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	return -ENOTSUPP;
}

long tegra_dc_crc_disable(struct tegra_dc *dc,
			  struct tegra_dc_ext_crc_arg *arg)
{
	return -ENOTSUPP;
}

long tegra_dc_crc_get(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	return -ENOTSUPP;
}
