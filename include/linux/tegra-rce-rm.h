/*
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_RCE_RM_H_
#define _LINUX_TEGRA_RCE_RM_H_

#include <linux/types.h>
#include <linux/scatterlist.h>

int rce_rm_map_carveout_for_device(struct platform_device *pdev,
				struct device *dev, struct sg_table *sgt);

#endif
