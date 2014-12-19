/*
 * drivers/video/tegra/dc/nvdisplay/nvdis.h
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVER_VIDEO_TEGRA_DC_NVDISP_H
#define __DRIVER_VIDEO_TEGRA_DC_NVDISP_H

extern struct mutex tegra_nvdisp_lock;


int tegra_nvdisp_assign_win(struct tegra_dc *dc, unsigned idx);
int tegra_nvdisp_detach_win(struct tegra_dc *dc, unsigned idx);

int tegra_nvdisp_set_csc(struct tegra_dc_win *win, struct tegra_dc_csc_v2 *csc);
void tegra_nvdisp_init_csc_defaults(struct tegra_dc_csc_v2 *csc);
#endif
