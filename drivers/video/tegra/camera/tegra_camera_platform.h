/*
 * drivers/video/tegra/camera/tegra_camera_common.h
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

/* TODO: Add ISO_EMC */
enum clock_id {
	EMC,
	NUM_CLKS
};

/* TODO: Add iso.emc */
static char *clk_names[NUM_CLKS] = {
	"emc",
};

struct tegra_camera_info {
	char devname[64];
	atomic_t in_use;
	struct device *dev;
	struct clk *clks[NUM_CLKS];
};

struct bw_info {
	u8 is_iso;
	u64 bw;
};

#endif

