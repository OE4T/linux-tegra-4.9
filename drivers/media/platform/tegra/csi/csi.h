/*
 * NVIDIA Tegra CSI Device Header
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CSI_H_
#define __CSI_H_

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/platform_device.h>

#include <media/camera_common.h>
#include "../camera/registers.h"
#include "../camera/mc_common.h"

#define NVCSI_PHY_0_BASE			0x18000
#define NVCSI_CIL_PHY_CTRL_0			0x0
#define		CFG_PHY_MODE			0x1
#define		CFG_PHY_MODE_SHIFT		0
#define NVCSI_CIL_CONFIG_0			0x4
#define		DATA_LANE_A			0x7
#define		DATA_LANE_A_SHIFT		0
#define		DATA_LANE_B			0x7
#define		DATA_LANE_B_SHIFT		8
#define	NVCSI_CIL_CLKEN_OVERRIDE_CTRL_0		0x8
#define	NVCSI_CIL_PAD_CONFIG_0			0xc
#define NVCSI_CIL_LANE_SWIZZLE_CTRL_0		0x10

#define	NVCSI_CIL_A_BASE                        0x18
#define	SW_RESET_0				0x0
#define CLKEN_OVERRIDE_CTRL_0			0x4
#define PAD_CONFIG_0				0x8
#define		E_INPUT_LP_IO1_SHIFT		22
#define		E_INPUT_LP_IO0_SHIFT		21
#define		E_INPUT_LP_CLK			20
#define		PD_CLK				18
#define		PD_IO1				17
#define		PD_IO0				16
#define	CLK_DESKEW_CTRL_0			0x14
#define DATA_DESKEW_CTRL_0			0x18
#define POLARITY_SWIZZLE_CTRL_0			0x40
#define CONTROL_0				0x44
#define NVCSI_CIL_B_BASE			0x7c

#define NVCSI_PHY_1_BASE			0x28000
#define NVCSI_PHY_2_BASE			0x38000

enum tegra_csi_port_num {
	PORT_A = 0,
	PORT_B = 1,
	PORT_C = 2,
	PORT_D = 3,
	PORT_E = 4,
	PORT_F = 5,
};

#define csi_port_is_valid(port) \
	(port < PORT_A ? 0 : (port > PORT_F ? 0 : 1))

enum camera_gang_mode {
	CAMERA_NO_GANG_MODE = 0,
	CAMERA_GANG_L_R = 1,
	CAMERA_GANG_T_B,
	CAMERA_GANG_R_L,
	CAMERA_GANG_B_T
};

struct tegra_channel;

struct tegra_csi_port {
	void __iomem *pixel_parser;
	void __iomem *cil;
	void __iomem *tpg;

	/* One pair of sink/source pad has one format */
	struct v4l2_mbus_framefmt format;
	const struct tegra_video_format *core_format;
	unsigned int lanes;

	enum tegra_csi_port_num num;
};

struct tegra_csi_device {
	struct v4l2_subdev subdev;
	struct vi *vi;
	struct device *dev;
	struct platform_device *pdev;
	void __iomem *iomem[3];
	struct clk *clk;
	struct clk *tpg_clk;
	struct clk *cil[3];

	struct camera_common_data s_data[6];
	struct tegra_csi_port *ports;
	struct media_pad *pads;

	unsigned int clk_freq;
	int num_ports;
	int pg_mode;

	struct tegra_csi_fops *fops;
};

struct tegra_csi_fops {
	void (*soc_tpg_start_streaming)(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
	void (*soc_start_streaming)(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
	int (*soc_error)(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
	void (*soc_status)(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
	void (*soc_error_recover)(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
	void (*soc_stop_streaming)(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
	int (*soc_init)(struct tegra_csi_device *csi,
			struct platform_device *pdev);
};

static inline struct tegra_csi_device *to_csi(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct tegra_csi_device, subdev);
}

void set_csi_portinfo(struct tegra_csi_device *csi,
	unsigned int port, unsigned int numlanes);
void tegra_csi_status(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
int tegra_csi_error(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num);
void tegra_csi_tpg_start_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num);
void tegra_csi_start_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num);
void tegra_csi_stop_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num);
void tegra_csi_error_recover(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num);
void tegra_csi_pad_control(struct tegra_csi_device *csi,
				unsigned char *port_num, int enable);
int tegra_csi_channel_power(struct tegra_csi_device *csi,
				unsigned char *port, int enable);
#define tegra_csi_channel_power_on(csi, port) \
	tegra_csi_channel_power(csi, port, 1)
#define tegra_csi_channel_power_off(csi, port) \
	tegra_csi_channel_power(csi, port, 0)
int tegra_csi_power(struct tegra_csi_device *csi, int enable);
#define tegra_csi_power_on(csi) tegra_csi_power(csi, 1)
#define tegra_csi_power_off(csi) tegra_csi_power(csi, 0)
int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev);
int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				struct platform_device *pdev);
int tegra_csi_media_controller_remove(struct tegra_csi_device *csi);
int csi_mipi_cal(struct tegra_channel *chan, char is_bypass);
#endif
