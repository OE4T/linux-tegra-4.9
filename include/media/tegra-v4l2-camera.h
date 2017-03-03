/**
 * TEGRA_V4L2_CAMERA.h - utilities for tegra camera driver
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA_V4L2_CAMERA__
#define __TEGRA_V4L2_CAMERA__

#include <linux/v4l2-controls.h>

#define TEGRA_CAMERA_CID_BASE	(V4L2_CTRL_CLASS_CAMERA | 0x2000)

#define TEGRA_CAMERA_CID_FRAME_LENGTH		(TEGRA_CAMERA_CID_BASE+0)
#define TEGRA_CAMERA_CID_COARSE_TIME		(TEGRA_CAMERA_CID_BASE+1)
#define TEGRA_CAMERA_CID_COARSE_TIME_SHORT	(TEGRA_CAMERA_CID_BASE+2)
#define TEGRA_CAMERA_CID_GROUP_HOLD		(TEGRA_CAMERA_CID_BASE+3)
#define TEGRA_CAMERA_CID_HDR_EN			(TEGRA_CAMERA_CID_BASE+4)
#define TEGRA_CAMERA_CID_EEPROM_DATA		(TEGRA_CAMERA_CID_BASE+5)
#define TEGRA_CAMERA_CID_OTP_DATA		(TEGRA_CAMERA_CID_BASE+6)
#define TEGRA_CAMERA_CID_FUSE_ID		(TEGRA_CAMERA_CID_BASE+7)
#define TEGRA_CAMERA_CID_TEGRA_CAMERA_LAST	(TEGRA_CAMERA_CID_BASE+8)
#define TEGRA_CAMERA_CID_SENSOR_MODE_ID		(TEGRA_CAMERA_CID_BASE+10)

#define TEGRA_CAMERA_CID_GAIN			(TEGRA_CAMERA_CID_BASE+11)
#define TEGRA_CAMERA_CID_EXPOSURE		(TEGRA_CAMERA_CID_BASE+12)
#define TEGRA_CAMERA_CID_FRAME_RATE		(TEGRA_CAMERA_CID_BASE+13)

#define TEGRA_CAMERA_CID_VI_BYPASS_MODE		(TEGRA_CAMERA_CID_BASE+100)
#define TEGRA_CAMERA_CID_OVERRIDE_ENABLE	(TEGRA_CAMERA_CID_BASE+101)
#define TEGRA_CAMERA_CID_VI_HEIGHT_ALIGN	(TEGRA_CAMERA_CID_BASE+102)
#define TEGRA_CAMERA_CID_VI_SIZE_ALIGN		(TEGRA_CAMERA_CID_BASE+103)
#define TEGRA_CAMERA_CID_WRITE_ISPFORMAT	(TEGRA_CAMERA_CID_BASE+104)

#define MAX_BUFFER_SIZE			32
#define MAX_CID_CONTROLS		16
#define MAX_NUM_SENSOR_MODES		30
#define OF_MAX_STR_LEN			256
#define OF_SENSORMODE_PREFIX ("mode")

/*
 * Scaling factor for converting a Q10.22 fixed point value
 * back to its original floating point value
 */
#define FIXED_POINT_SCALING_FACTOR (1ULL << 22)

struct unpackedU64 {
	__u32 high;
	__u32 low;
};

union __u64val {
	struct unpackedU64 unpacked;
	__u64 val;
};

struct sensor_signal_properties {
	__u32 readout_orientation;
	__u32 num_lanes;
	__u32 mclk_freq;
	union __u64val pixel_clock;
	__u32 cil_settletime;
	__u32 discontinuous_clk;
	__u32 dpcm_enable;
	__u32 tegra_sinterface;
};

struct sensor_image_properties {
	__u32 width;
	__u32 height;
	__u32 line_length;
	__u32 pixel_format;
	__u32 embedded_metadata_height;
};

struct sensor_dv_timings {
	__u32 hfrontporch;
	__u32 hsync;
	__u32 hbackporch;
	__u32 vfrontporch;
	__u32 vsync;
	__u32 vbackporch;
};

struct sensor_control_properties {
	__u32 gain_factor;
	__u32 framerate_factor;
	__u32 inherent_gain;
	__u32 min_gain_val;
	__u32 max_gain_val;
	__u32 min_hdr_ratio;
	__u32 max_hdr_ratio;
	__u32 min_framerate;
	__u32 max_framerate;
	union __u64val min_exp_time;
	union __u64val max_exp_time;
};

struct sensor_mode_properties {
	struct sensor_signal_properties signal_properties;
	struct sensor_image_properties image_properties;
	struct sensor_control_properties control_properties;
	struct sensor_dv_timings dv_timings;
};

#define SENSOR_SIGNAL_PROPERTIES_CID_SIZE \
	(sizeof(struct sensor_signal_properties) / sizeof(__u32))
#define SENSOR_IMAGE_PROPERTIES_CID_SIZE \
	(sizeof(struct sensor_image_properties) / sizeof(__u32))
#define SENSOR_CONTROL_PROPERTIES_CID_SIZE \
	(sizeof(struct sensor_control_properties) / sizeof(__u32))
#define SENSOR_DV_TIMINGS_CID_SIZE \
	(sizeof(struct sensor_dv_timings) / sizeof(__u32))
#define SENSOR_MODE_PROPERTIES_CID_SIZE \
	(sizeof(struct sensor_mode_properties) / sizeof(__u32))

#endif /* __TEGRA_V4L2_CAMERA__ */
