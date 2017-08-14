/*
 * NVIDIA Tegra CSI Device Header
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <media/camera_common.h>
#include <media/vi2_registers.h>
#include <media/csi4_registers.h>

#include <linux/platform_device.h>

#define MAX_CSI_BLOCK_LANES 4

enum tegra_csi_port_num {
	PORT_A = 0,
	PORT_B = 1,
	PORT_C = 2,
	PORT_D = 3,
	PORT_E = 4,
	PORT_F = 5,
};

#define csi_port_is_valid(port) (port > PORT_F ? 0 : 1)

enum camera_gang_mode {
	CAMERA_NO_GANG_MODE = 0,
	CAMERA_GANG_L_R = 1,
	CAMERA_GANG_T_B,
	CAMERA_GANG_R_L,
	CAMERA_GANG_B_T
};

struct tegra_channel;

struct tpg_frmfmt {
	struct v4l2_frmsize_discrete frmsize;
	int pixel_format;
	int framerate;
	int h_blank;
	int v_blank;
};

struct tegra_csi_port {
	void __iomem *pixel_parser;
	void __iomem *cil;
	void __iomem *tpg;

	/* One pair of sink/source pad has one format */
	struct v4l2_mbus_framefmt format;
	const struct tegra_video_format *core_format;
	unsigned int lanes;
	unsigned int framerate;
	unsigned int h_blank;
	unsigned int v_blank;

	enum tegra_csi_port_num num;
};

struct tegra_csi_device {
	struct device *dev;
	struct platform_device *pdev;
	char devname[32];
	void __iomem *iomem_base;
	void __iomem *iomem[3];
	struct clk *plld_dsi;
	struct clk *plld;

	struct camera_common_data s_data[6];
	struct tegra_csi_port *ports;
	struct media_pad *pads;

	unsigned int clk_freq;
	int num_ports;
	int num_channels;
	struct list_head csi_chans;
	struct tegra_csi_channel *tpg_start;
	const struct tegra_csi_fops *fops;
	const struct tpg_frmfmt *tpg_frmfmt_table;
	unsigned int tpg_frmfmt_table_size;
	atomic_t power_ref;

	struct dentry *debugdir;
	struct mutex source_update;
	int tpg_active;
	int sensor_active;
	/* num_tpg_channels is a fixed number per soc*/
	int num_tpg_channels;
};

/*
 * subdev: channel subdev
 * numports: Number of CSI ports in use for this channel
 * numlanes: Number of CIL lanes in use
 */
struct tegra_csi_channel {
	struct list_head list;
	struct v4l2_subdev subdev;
	struct media_pad *pads;
	struct media_pipeline pipe;

	struct tegra_csi_device *csi;
	struct tegra_csi_port *ports;
	unsigned char port[TEGRA_CSI_BLOCKS];
	struct mutex format_lock;
	unsigned int numports;
	unsigned int numlanes;
	unsigned int pg_mode;
	struct camera_common_data *s_data;
	unsigned int id;
	atomic_t is_streaming;
};

static inline struct tegra_csi_channel *to_csi_chan(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct tegra_csi_channel, subdev);
}

static inline struct tegra_csi_device *to_csi(struct v4l2_subdev *subdev)
{
	struct tegra_csi_channel *chan = to_csi_chan(subdev);

	return chan->csi;
}

void set_csi_portinfo(struct tegra_csi_device *csi,
	unsigned int port, unsigned int numlanes);
void tegra_csi_status(struct tegra_csi_channel *chan,
			enum tegra_csi_port_num port_num);
int tegra_csi_error(struct tegra_csi_channel *chan,
			enum tegra_csi_port_num port_num);
int tegra_csi_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num);
void tegra_csi_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num);
void tegra_csi_error_recover(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num);
int tegra_csi_power(struct tegra_csi_device *csi, int enable);
#define tegra_csi_power_on(csi) tegra_csi_power(csi, 1)
#define tegra_csi_power_off(csi) tegra_csi_power(csi, 0)
int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev);
int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				struct platform_device *pdev);
int tegra_csi_media_controller_remove(struct tegra_csi_device *csi);
struct tegra_csi_device *tegra_get_mc_csi(void);
int tpg_csi_media_controller_init(struct tegra_csi_device *csi, int pg_mode);
void tpg_csi_media_controller_cleanup(struct tegra_csi_device *csi);
#endif
