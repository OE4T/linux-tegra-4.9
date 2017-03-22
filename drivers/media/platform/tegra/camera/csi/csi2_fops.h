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

int csi2_start_streaming(struct tegra_csi_channel *chan,
		enum tegra_csi_port_num port_num);
void csi2_stop_streaming(struct tegra_csi_channel *chan,
		enum tegra_csi_port_num port_num);

extern struct tegra_csi_fops csi2_fops;

#endif
