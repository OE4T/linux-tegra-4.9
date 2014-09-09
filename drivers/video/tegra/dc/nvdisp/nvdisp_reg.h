/*
 * drivers/video/tegra/dc/nvdisplay/nvdis_reg.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __TEGRA_NVDISP_REG_H
#define __TEGRA_NVDISP_REG_H

#define DC_WIN_BLEND_LAYER_CONTROL 0x716
#define    BLEND_BYPASS            (0x1 << 24)

#define WIN_PCALC_WINDOW_SET_CROPPED_SIZE_IN 0x706
#define    PCALC_HEIGHT            0
#define    PCALC_WIDTH             16

#define    WIN_INVERT_V            (0x1 << 2)
#define    WIN_INVERT_H            (0x1)

#define WIN_CORE_WINDOWGROUP_SET_CONTROL 0x702
#define    SET_CONTROL_NONE        (0xf)

#endif
