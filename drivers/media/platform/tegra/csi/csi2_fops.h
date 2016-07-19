/*
 * Tegra CSI2 device common APIs
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CSI2_H__
#define __CSI2_H__

#include "csi.h"

void csi2_tpg_start_streaming(struct tegra_csi_device *csi,
		enum tegra_csi_port_num port_num);
void csi2_start_streaming(struct tegra_csi_device *csi,
		enum tegra_csi_port_num port_num);
int csi2_error(struct tegra_csi_device *csi,
		enum tegra_csi_port_num port_num);
void csi2_status(struct tegra_csi_device *csi,
		enum tegra_csi_port_num port_num);
void csi2_error_recover(struct tegra_csi_device *csi,
		enum tegra_csi_port_num port_num);
void csi2_stop_streaming(struct tegra_csi_device *csi,
		enum tegra_csi_port_num port_num);

struct tegra_csi_fops csi2_fops = {
	.soc_tpg_start_streaming = csi2_tpg_start_streaming,
	.soc_start_streaming = csi2_start_streaming,
	.soc_error = csi2_error,
	.soc_status = csi2_status,
	.soc_error_recover = csi2_error_recover,
	.soc_stop_streaming = csi2_stop_streaming,
};

#endif
