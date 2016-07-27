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

#include "vi/vi.h"

int vi2_power_on(struct tegra_mc_vi *vi);
void vi2_power_off(struct tegra_mc_vi *vi);
void vi2_channel_ec_init(struct tegra_channel *chan);
void vi2_channel_ec_recover(struct tegra_channel *chan);
int vi2_channel_capture_setup(struct tegra_channel *chan);
void vi2_channel_capture_frame_init(struct tegra_channel *chan,
		struct tegra_channel_buffer *buf, u32 *thresh);
void vi2_channel_capture_frame_enable(struct tegra_channel *chan);
int vi2_channel_capture_frame(struct tegra_channel *chan,
		struct timespec *ts, u32 *thresh);
int vi2_channel_capture_done(struct tegra_channel *chan,
		struct tegra_channel_buffer *buf,
		struct timespec *ts);
int vi2_channel_error_status(struct tegra_channel *chan);
int vi2_channel_stop_streaming(struct tegra_channel *chan);

struct tegra_vi_fops vi2_fops = {
	.soc_power_on = vi2_power_on,
	.soc_power_off = vi2_power_off,
};

struct tegra_vi_channel_fops vi2_channel_fops = {
	.soc_channel_ec_init = vi2_channel_ec_init,
	.soc_channel_ec_recover = vi2_channel_ec_recover,
	.soc_channel_capture_setup = vi2_channel_capture_setup,
	.soc_channel_capture_frame_init = vi2_channel_capture_frame_init,
	.soc_channel_capture_frame_enable = vi2_channel_capture_frame_enable,
	.soc_channel_capture_frame = vi2_channel_capture_frame,
	.soc_channel_capture_done = vi2_channel_capture_done,
	.soc_channel_error_status = vi2_channel_error_status,
	.soc_channel_stop_streaming = vi2_channel_stop_streaming,
};


#endif
