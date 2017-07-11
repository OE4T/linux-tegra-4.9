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

	dev_info(csi->dev, "csi5_start_streaming\n");

	return 0;
}

static void csi5_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	dev_info(csi->dev, "csi5_stop_streaming\n");
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
