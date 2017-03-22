/*
 * Tegra CSI4 device common APIs
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CSI4_H__
#define __CSI4_H__

#include "csi.h"

int csi4_power_on(struct tegra_csi_device *csi);
int csi4_power_off(struct tegra_csi_device *csi);
int csi4_start_streaming(struct tegra_csi_channel *chacsin,
		enum tegra_csi_port_num port_num);
void csi4_stop_streaming(struct tegra_csi_channel *chan,
		enum tegra_csi_port_num port_num);
void csi4_override_format(struct tegra_csi_channel *chan,
		enum tegra_csi_port_num port_num);
int csi4_mipi_cal(struct tegra_csi_channel *chan);
int csi4_hw_init(struct tegra_csi_device *csi);

struct tegra_csi_fops csi4_fops = {
	.csi_power_on = csi4_power_on,
	.csi_power_off = csi4_power_off,
	.csi_start_streaming = csi4_start_streaming,
	.csi_stop_streaming = csi4_stop_streaming,
	.csi_override_format = csi4_override_format,
	.mipical = csi4_mipi_cal,
	.hw_init = csi4_hw_init,
};

#endif
