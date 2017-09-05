/*
 * Tegra CSI5 device common APIs
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <media/csi.h>
#include <media/mc_common.h>
#include "nvhost_acm.h"
#include "nvcsi/nvcsi.h"
#include "csi5_fops.h"
#include <linux/tegra-capture-ivc.h>
#include "soc/tegra/camrtc-capture-messages.h"

/* Referred from capture-scheduler.c defined in rtcpu-fw */
#define NUM_CAPTURE_CHANNELS 64

/* Temporary ids for the clients whose channel-id is not yet allocated */
#define NUM_CAPTURE_TRANSACTION_IDS 64

#define TOTAL_CHANNELS (NUM_CAPTURE_CHANNELS + NUM_CAPTURE_TRANSACTION_IDS)

#define TEMP_CHANNEL_ID (NUM_CAPTURE_CHANNELS + 1)
#define TPG_HBLANK 0
#define TPG_VBLANK 40800

static int csi5_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	dev_info(csi->dev, "csi5_power_on\n");

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:cannot enable csi\n", __func__);

	return err;
}

static int csi5_power_off(struct tegra_csi_device *csi)
{
	dev_info(csi->dev, "csi5_power_off\n");

	nvhost_module_idle(csi->pdev);

	return 0;
}

static int csi5_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[port_num];
	int err = 0;

	/* PG native resolution */
	const size_t px_max = 0x4000;
	const size_t py_max = 0x2000;
	size_t hfreq = 0;
	size_t vfreq = 0;

	union nvcsi_tpg_config *pgc = NULL;
	struct CAPTURE_CONTROL_MSG msg;
	const size_t messageSize = sizeof(msg);

	memset(&msg, 0, messageSize);
	dev_info(csi->dev, "csi5_start_streaming\n");
	if (chan->pg_mode) {
		pgc = &msg.tpg_setup_req.tpg_config;

		hfreq = px_max / port->format.width;
		vfreq = py_max / port->format.height;

		pgc->t194.virtual_channel = 0;
		/* hardcode CSI_DT_RAW_16 */
		pgc->t194.data_type = 46;
		pgc->t194.lane_count = chan->numlanes;
		pgc->t194.flags	= NVCSI_TPG_FLAG_PATCH_MODE;

		pgc->t194.initial_frame_number = 1;
		pgc->t194.maximum_frame_number = 32768;
		pgc->t194.image_width = port->format.width;
		pgc->t194.image_height = port->format.height;

		pgc->t194.red_horizontal_init_freq = hfreq;
		pgc->t194.red_vertical_init_freq = vfreq;

		pgc->t194.green_horizontal_init_freq = hfreq;
		pgc->t194.green_vertical_init_freq = vfreq;

		pgc->t194.blue_horizontal_init_freq = hfreq;
		pgc->t194.blue_vertical_init_freq = vfreq;

		msg.header.msg_id = CAPTURE_CHANNEL_TPG_SETUP_REQ;
		/* use a free channel to send control */
		msg.header.channel_id = TEMP_CHANNEL_ID;

		err = tegra_capture_ivc_control_submit(&msg, messageSize);
		if (err < 0) {
			dev_err(csi->dev, "IVC control submit failed\n");
			return  err;
		}

		/* start streaming */
		memset(&msg, 0, messageSize);
		msg.header.msg_id = CAPTURE_CHANNEL_TPG_START_REQ;
		msg.header.channel_id = TEMP_CHANNEL_ID;
		msg.tpg_start_req.stream = chan->port[0];
		msg.tpg_start_req.channel = 0;
		msg.tpg_start_req.tpg_rate_config.hblank = TPG_HBLANK;
		msg.tpg_start_req.tpg_rate_config.vblank = TPG_VBLANK;
		msg.tpg_start_req.tpg_rate_config.pixel_interval = 0;
		err = tegra_capture_ivc_control_submit(&msg, messageSize);
		if (err < 0) {
			dev_err(csi->dev, "IVC control submit failed\n");
			return  err;
		}
	}

	return err;
}

static void csi5_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int err = 0;

	struct CAPTURE_CONTROL_MSG msg;
	const size_t messageSize = sizeof(msg);

	memset(&msg, 0, messageSize);
	dev_info(csi->dev, "csi5_stop_streaming\n");

	if (chan->pg_mode) {
		msg.header.msg_id = CAPTURE_CHANNEL_TPG_STOP_REQ;
		msg.header.channel_id = TEMP_CHANNEL_ID;

		msg.tpg_stop_req.stream = chan->port[0];
		msg.tpg_stop_req.channel = 0;

		err = tegra_capture_ivc_control_submit(&msg, messageSize);
		if (err < 0) {
			dev_err(csi->dev, "IVC control submit failed\n");
			return;
		}
	}
}

static int csi5_mipi_cal(struct tegra_csi_channel *chan)
{
	struct tegra_csi_device *csi = chan->csi;

	dev_info(csi->dev, "csi5_mipi_cal\n");

	return 0;
}

static int csi5_hw_init(struct tegra_csi_device *csi)
{
	dev_info(csi->dev, "csi5_hw_init\n");

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
