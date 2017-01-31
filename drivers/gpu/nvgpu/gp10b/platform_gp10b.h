/*
 * GP10B Platform (SoC) Interface
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _GP10B_PLATFORM_H_
#define _GP10B_PLATFORM_H_

struct device;

int gp10b_tegra_get_clocks(struct device *dev);
int gp10b_tegra_reset_assert(struct device *dev);
int gp10b_tegra_reset_deassert(struct device *dev);

#endif
