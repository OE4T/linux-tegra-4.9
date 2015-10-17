/*
 * Tegra Video Input device common APIs
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VI_COMMON_H_
#define __VI_COMMON_H_

void tegra_vi_v4l2_cleanup(struct vi *vi);
int tegra_vi_v4l2_init(struct vi *vi);

int tegra_vi_graph_init(struct vi *vi);
void tegra_vi_graph_cleanup(struct vi *vi);
int tegra_vi_channels_init(struct vi *vi);
int tegra_vi_channels_cleanup(struct vi *vi);
void tegra_channel_fmts_bitmap_init(struct tegra_channel *chan,
				    struct tegra_vi_graph_entity *entity);

#endif
