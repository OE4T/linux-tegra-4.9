/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __NVGPU_XVE_H__
#define __NVGPU_XVE_H__

#include <nvgpu/types.h>
#include <nvgpu/log2.h>

/*
 * For the available speeds bitmap.
 */
#define GPU_XVE_SPEED_2P5	(1 << 0)
#define GPU_XVE_SPEED_5P0	(1 << 1)
#define GPU_XVE_SPEED_8P0	(1 << 2)
#define GPU_XVE_NR_SPEEDS	3

#define GPU_XVE_SPEED_MASK	(GPU_XVE_SPEED_2P5 |	\
				 GPU_XVE_SPEED_5P0 |	\
				 GPU_XVE_SPEED_8P0)

/*
 * The HW uses a 2 bit field where speed is defined by a number:
 *
 *   NV_XVE_LINK_CONTROL_STATUS_LINK_SPEED_2P5 = 1
 *   NV_XVE_LINK_CONTROL_STATUS_LINK_SPEED_5P0 = 2
 *   NV_XVE_LINK_CONTROL_STATUS_LINK_SPEED_8P0 = 3
 *
 * This isn't ideal for a bitmap with available speeds. So the external
 * APIs think about speeds as a bit in a bitmap and this function converts
 * from those bits to the actual HW speed setting.
 *
 * @speed_bit must have only 1 bit set and must be one of the 3 available
 * HW speeds. Not all chips support all speeds so use available_speeds() to
 * determine what a given chip supports.
 */
static inline const char *xve_speed_to_str(u32 speed)
{
	if (!speed || !is_power_of_2(speed) ||
	    !(speed & GPU_XVE_SPEED_MASK))
		return "Unknown ???";

	return speed & GPU_XVE_SPEED_2P5 ? "Gen1" :
	       speed & GPU_XVE_SPEED_5P0 ? "Gen2" :
	       speed & GPU_XVE_SPEED_8P0 ? "Gen3" :
	       "Unknown ???";
}

#endif
