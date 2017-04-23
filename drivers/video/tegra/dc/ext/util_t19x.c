/*
 * drivers/video/tegra/dc/ext/util_t19x.c
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

#include <linux/dma-buf.h>
#include <linux/dma-override.h>
#include "dc_priv.h"

u32 tegra_dc_ext_get_map_buffer_flag(u32 flags)
{
	if (flags & TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR)
		return DMA_TO_DEVICE | IOMMU_USE_BL_FORMAT;
	else
		return DMA_TO_DEVICE;
}

