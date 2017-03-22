/*
 * Tegra Video Input 2 device common APIs
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

#ifndef __T210_VI_H__
#define __T210_VI_H__

int vi2_power_on(struct tegra_channel *chan);
void vi2_power_off(struct tegra_channel *chan);
int vi2_channel_start_streaming(struct vb2_queue *vq, u32 count);
int vi2_channel_stop_streaming(struct vb2_queue *vq);

struct tegra_vi_fops vi2_fops = {
	.vi_power_on = vi2_power_on,
	.vi_power_off = vi2_power_off,
	.vi_start_streaming = vi2_channel_start_streaming,
	.vi_stop_streaming = vi2_channel_stop_streaming,
};

#endif
