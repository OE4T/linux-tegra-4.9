/*
 * drivers/video/tegra/camera/tegra_camera_common.h
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _TEGRA_CAMERA_PLATFORM_H_
#define _TEGRA_CAMERA_PLATFORM_H_

#include <linux/ioctl.h>

#define TEGRA_CAMERA_IOCTL_SET_BW _IOW('o', 1, struct bw_info)

struct bw_info {
	u8 is_iso;
	u64 bw;
};

int vi_v4l2_update_isobw(u32 vi_kbyteps, u32 is_iotcl);
int tegra_camera_emc_clk_enable(void);
int tegra_camera_emc_clk_disable(void);

#endif

