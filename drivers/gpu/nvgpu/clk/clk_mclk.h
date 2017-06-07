/*
* Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*/

#ifndef _CLKMCLK_H_
#define _CLKMCLK_H_

#include <nvgpu/lock.h>

#define GP106_MCLK_LOW_SPEED	0
#define GP106_MCLK_MID_SPEED	1
#define GP106_MCLK_HIGH_SPEED	2
#define GP106_MCLK_NUM_SPEED	3

enum gk20a_mclk_speed {
	gk20a_mclk_low_speed,
	gk20a_mclk_mid_speed,
	gk20a_mclk_high_speed,
};

struct clk_mclk_state {
	u32 speed;
	struct nvgpu_mutex mclk_lock;
	struct nvgpu_mutex data_lock;

	u16 p5_min;
	u16 p0_min;

	void *vreg_buf;
	bool init;

#ifdef CONFIG_DEBUG_FS
	s64 switch_max;
	s64 switch_min;
	u64 switch_num;
	s64 switch_avg;
	s64 switch_std;
	bool debugfs_set;
#endif
};

#endif
