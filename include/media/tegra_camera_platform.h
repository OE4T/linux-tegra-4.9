/*
 * drivers/video/tegra/camera/tegra_camera_common.h
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
#define TEGRA_CAMERA_IOCTL_GET_BW _IOR('o', 2, u64)

struct bw_info {
	u8 is_iso;
	u64 bw;
};

/**
 * enum tegra_camera_hw_type - camera hw engines
 */
enum tegra_camera_hw_type {
	HWTYPE_NONE = 0,
	HWTYPE_CSI,
	HWTYPE_SLVSEC,
	HWTYPE_VI,
	HWTYPE_ISPA,
	HWTYPE_ISPB,
	HWTYPE_MAX,
};

/**
 * enum tegra_camera_sensor_type - camera sensor types
 */
enum tegra_camera_sensor_type {
	SENSORTYPE_NONE = 0,
	SENSORTYPE_DPHY,
	SENSORTYPE_CPHY,
	SENSORTYPE_SLVSEC,
	SENSORTYPE_VIRTUAL,
	SENSORTYPE_MAX,
};

/**
 * struct tegra_camera_dev_info - camera devices information
 * @priv: a unique identifier assigned during registration
 * @hw_type: type of HW engine as defined by the enum above
 * @sensor_type: type of sensor as defined by the enum above
 * @pixel_rate: pixel rate coming out of the sensor
 * @pixel_bit_depth: bits per pixel
 * @bpp: bytes per pixel
 * @bus_width: csi bus width for clock calculation
 * @overhead: hw/ sw overhead considered while calculations
 * @stream_on: stream enabled on the channel
 * @clk_rate: calculated clk rate for this node
 * @bw: calculated bw for this node
 * @chan: pointer to tegra_channel struct for sensor info
 * @pdev: pointer to platform_data
 * @device_node: list node
 */
struct tegra_camera_dev_info {
	void *priv;
	u32 hw_type;
	u32 sensor_type;
	u64 pixel_rate;
	u32 pixel_bit_depth;
	u32 bpp;
	u32 bus_width;
	u32 overhead;
	bool stream_on;
	u64 clk_rate;
	u64 bw;
	struct tegra_channel *chan;
	struct platform_device *pdev;
	struct list_head device_node;
};

int vi_v4l2_update_isobw(u32 vi_kbyteps, u32 is_iotcl);
int tegra_camera_emc_clk_enable(void);
int tegra_camera_emc_clk_disable(void);
int tegra_camera_device_register(struct tegra_camera_dev_info *cdev_info,
					void *priv);
int tegra_camera_device_unregister(void *priv);

#endif

