/*
 * Tegra CSI5 device common APIs
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <media/csi.h>
#include <media/mc_common.h>
#include <media/csi5_registers.h>
#include "nvhost_acm.h"
#include "nvcsi/nvcsi.h"
#include "csi5_fops.h"
#include <linux/tegra-capture-ivc.h>
#include "soc/tegra/camrtc-capture-messages.h"

#include "mipical/mipi_cal.h"

/* Referred from capture-scheduler.c defined in rtcpu-fw */
#define NUM_CAPTURE_CHANNELS 64

/* Temporary ids for the clients whose channel-id is not yet allocated */
#define NUM_CAPTURE_TRANSACTION_IDS 64

#define TOTAL_CHANNELS (NUM_CAPTURE_CHANNELS + NUM_CAPTURE_TRANSACTION_IDS)

#define NVCSI_CIL_CLOCK_RATE 204000

#define TEMP_CHANNEL_ID (NUM_CAPTURE_CHANNELS + 1)
#define TPG_HBLANK 0
#define TPG_VBLANK 40800


static void csi5_phy_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;

	writel(val, csi->iomem_base +
		CSI5_BASE_ADDRESS + (CSI5_PHY_OFFSET * index) + addr);
}

static int csi5_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	dev_dbg(csi->dev, "%s\n", __func__);

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:cannot enable csi\n", __func__);

	return err;
}

static int csi5_power_off(struct tegra_csi_device *csi)
{
	dev_dbg(csi->dev, "%s\n", __func__);

	nvhost_module_idle(csi->pdev);

	return 0;
}

static int csi5_stream_open(struct tegra_csi_channel *chan, int csi_port)
{
	struct tegra_csi_device *csi = chan->csi;

	struct CAPTURE_CONTROL_MSG msg;

	dev_dbg(csi->dev, "%s: stream_id=%d\n", __func__, csi_port);

	/* Open NvCsi stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_PHY_STREAM_OPEN_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.phy_stream_open_req.stream_id = csi_port;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	return 0;
}

static void csi5_stream_close(struct tegra_csi_channel *chan, int csi_port)
{
	struct tegra_csi_device *csi = chan->csi;

	struct CAPTURE_CONTROL_MSG msg;

	dev_dbg(csi->dev, "%s: stream_id=%d\n", __func__, csi_port);

	/* Close NvCsi stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_PHY_STREAM_CLOSE_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.phy_stream_close_req.stream_id = csi_port;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));
}

static int csi5_stream_set_config(struct tegra_csi_channel *chan, int csi_port,
	int csi_lanes)
{
	struct tegra_csi_device *csi = chan->csi;

	struct camera_common_data *s_data = chan->s_data;
	const struct sensor_mode_properties *mode = NULL;

	unsigned int cil_settletime = 0;

	struct CAPTURE_CONTROL_MSG msg;
	struct nvcsi_brick_config brick_config;
	struct nvcsi_cil_config cil_config;
	bool is_cphy = (csi_lanes == 3);

	dev_dbg(csi->dev, "%s: stream_id=%d\n", __func__, csi_port);

	/* Attempt to find the cil_settingtime from the device tree */
	if (s_data) {
		int idx = s_data->mode_prop_idx;

		dev_dbg(csi->dev, "cil_settingtime is pulled from device");
		if (idx < s_data->sensor_props.num_modes) {
			mode = &s_data->sensor_props.sensor_modes[idx];
			cil_settletime = mode->signal_properties.cil_settletime;
		} else {
			dev_dbg(csi->dev, "mode not listed in DT, use default");
			cil_settletime = 0;
		}
	} else if (chan->of_node) {
		int err = 0;
		const char *str;

		dev_dbg(csi->dev,
			"cil_settletime is pulled from device of_node");
		err = of_property_read_string(chan->of_node, "cil_settletime",
			&str);
		if (!err) {
			err = kstrtou32(str, 10, &cil_settletime);
			if (err) {
				dev_dbg(csi->dev,
					"no cil_settletime in of_node");
				cil_settletime = 0;
			}
		}
	}

	/* Brick config */
	memset(&brick_config, 0, sizeof(brick_config));
	brick_config.phy_mode = (!is_cphy) ?
		NVCSI_PHY_TYPE_DPHY : NVCSI_PHY_TYPE_CPHY;

	/* CIL config */
	memset(&cil_config, 0, sizeof(cil_config));
	cil_config.num_lanes = csi_lanes;
	cil_config.lp_bypass_mode = is_cphy ? 0 : 1;
	cil_config.t_clk_settle = is_cphy ? 1 : 33;
	cil_config.t_hs_settle = cil_settletime;
	cil_config.cil_clock_rate = NVCSI_CIL_CLOCK_RATE; /* hard-coding */
	cil_config.mipi_clock_rate = csi->clk_freq / 1000;

	/* Set NvCsi stream config */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_SET_CONFIG_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.csi_stream_set_config_req.stream_id = csi_port;
	msg.csi_stream_set_config_req.brick_config = brick_config;
	msg.csi_stream_set_config_req.cil_config = cil_config;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	return 0;
}

static int csi5_stream_tpg_start(struct tegra_csi_channel *chan,
	enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[port_num];

	int csi_port = chan->ports[port_num].num;

	/* TPG native resolution */
	const size_t px_max = 0x4000;
	const size_t py_max = 0x2000;
	size_t hfreq = 0;
	size_t vfreq = 0;

	struct CAPTURE_CONTROL_MSG msg;
	union nvcsi_tpg_config *tpg_config = NULL;

	dev_dbg(csi->dev, "%s: stream_id=%d\n", __func__, csi_port);

	/* Set TPG config for a virtual channel */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_TPG_SET_CONFIG_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	hfreq = px_max / port->format.width;
	vfreq = py_max / port->format.height;

	tpg_config = &(msg.csi_stream_tpg_set_config_req.tpg_config);

	tpg_config->t194.virtual_channel_id = 0;
	tpg_config->t194.datatype = port->core_format->img_dt;

	tpg_config->t194.lane_count = chan->numlanes;
	tpg_config->t194.flags	= NVCSI_TPG_FLAG_PATCH_MODE;

	tpg_config->t194.initial_frame_number = 1;
	tpg_config->t194.maximum_frame_number = 32768;
	tpg_config->t194.image_width = port->format.width;
	tpg_config->t194.image_height = port->format.height;

	tpg_config->t194.red_horizontal_init_freq = hfreq;
	tpg_config->t194.red_vertical_init_freq = vfreq;

	tpg_config->t194.green_horizontal_init_freq = hfreq;
	tpg_config->t194.green_vertical_init_freq = vfreq;

	tpg_config->t194.blue_horizontal_init_freq = hfreq;
	tpg_config->t194.blue_vertical_init_freq = vfreq;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	/* Enable TPG on a stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_TPG_START_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.csi_stream_tpg_start_req.stream_id = csi_port;
	msg.csi_stream_tpg_start_req.virtual_channel_id = 0;
	msg.csi_stream_tpg_start_req.tpg_rate_config.hblank = TPG_HBLANK;
	msg.csi_stream_tpg_start_req.tpg_rate_config.vblank = TPG_VBLANK;
	msg.csi_stream_tpg_start_req.tpg_rate_config.pixel_interval = 0;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	return 0;
}

static void csi5_stream_tpg_stop(struct tegra_csi_channel *chan, int csi_port)
{
	struct tegra_csi_device *csi = chan->csi;

	struct CAPTURE_CONTROL_MSG msg;

	dev_dbg(csi->dev, "%s: stream_id=%d\n", __func__, csi_port);

	/* Disable TPG on a stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_TPG_STOP_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.csi_stream_tpg_stop_req.stream_id = csi_port;
	msg.csi_stream_tpg_stop_req.virtual_channel_id = 0;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));
}

static int csi5_start_streaming(struct tegra_csi_channel *chan,
	enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	int csi_port = chan->ports[port_num].num;
	int num_lanes = chan->ports[port_num].lanes;

	dev_dbg(csi->dev, "%s: stream %d, pg_mode=0x%x\n",
		__func__, csi_port, chan->pg_mode);

	if (!chan->pg_mode)
		csi5_stream_set_config(chan, csi_port, num_lanes);

	csi5_stream_open(chan, csi_port);

	if (chan->pg_mode)
		csi5_stream_tpg_start(chan, port_num);

	return 0;
}

static void csi5_stop_streaming(struct tegra_csi_channel *chan,
	enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	int csi_port = chan->ports[port_num].num;

	dev_dbg(csi->dev, "%s: stream %d, pg_mode=0x%x\n",
		__func__, csi_port, chan->pg_mode);

	if (chan->pg_mode)
		csi5_stream_tpg_stop(chan, csi_port);

	csi5_stream_close(chan, csi_port);
}

static int csi5_mipi_cal(struct tegra_csi_channel *chan)
{
	unsigned int lanes, num_ports, port, addr;
	unsigned int cila, cilb;
	struct tegra_csi_device *csi = chan->csi;
	u32 phy_mode = read_phy_mode_from_dt(chan);
	bool is_cphy = (phy_mode == CSI_PHY_MODE_CPHY);

	dev_dbg(csi->dev, "%s\n", __func__);

	lanes = 0;
	num_ports = 0;
	port = 0;
	while (num_ports < chan->numports) {
		port = chan->ports[num_ports].num;
		dev_dbg(csi->dev, "csi port:%d\n", port);

		if (chan->numlanes <= 2) {
			lanes |= CSIA << port;
			addr = (port % 2 == 0 ?
				CSI5_NVCSI_CIL_A_SW_RESET :
				CSI5_NVCSI_CIL_B_SW_RESET);
			csi5_phy_write(chan, port >> 1, addr,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
		} else if (chan->numlanes == 3) {
			lanes |= (CSIA | CSIB) << port;
			cila =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x00 << CSI5_E_INPUT_LP_CLK_SHIFT) |
				(0x01 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x00 << CSI5_PD_IO1_SHIFT);
			cilb =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x00 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x00 << CSI5_E_INPUT_LP_CLK_SHIFT) |
				(0x01 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x01 << CSI5_PD_IO1_SHIFT);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_A_BASE + CSI5_PAD_CONFIG_0,
					cila);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_B_BASE + CSI5_PAD_CONFIG_0,
					cilb);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_A_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_B_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
		} else {
			lanes |= (CSIA | CSIB) << port;
			cila =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_CLK_SHIFT) |
				(0x00 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x00 << CSI5_PD_IO1_SHIFT);
			cilb =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x01 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x00 << CSI5_PD_IO1_SHIFT);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_A_BASE + CSI5_PAD_CONFIG_0,
					cila);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_B_BASE + CSI5_PAD_CONFIG_0,
					cilb);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_A_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
			csi5_phy_write(chan, port >> 1,
				CSI5_NVCSI_CIL_B_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
		}
		num_ports++;
	}
	if (!lanes) {
		dev_err(csi->dev,
			"Selected no CSI lane, cannot do calibration");
		return -EINVAL;
	}
	lanes |= is_cphy ? CPHY_MASK : 0;
	return tegra_mipi_calibration(lanes);
}

static int csi5_hw_init(struct tegra_csi_device *csi)
{
	dev_dbg(csi->dev, "%s\n", __func__);

	csi->iomem[0] = csi->iomem_base + CSI5_TEGRA_CSI_STREAM_0_BASE;
	csi->iomem[1] = csi->iomem_base + CSI5_TEGRA_CSI_STREAM_2_BASE;
	csi->iomem[2] = csi->iomem_base + CSI5_TEGRA_CSI_STREAM_4_BASE;

	return 0;
}

struct tegra_csi_fops csi5_fops = {
	.csi_power_on = csi5_power_on,
	.csi_power_off = csi5_power_off,
	.csi_start_streaming = csi5_start_streaming,
	.csi_stop_streaming = csi5_stop_streaming,
	.mipical = csi5_mipi_cal,
	.hw_init = csi5_hw_init,
};
