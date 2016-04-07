/*
 * GV11B GPU GR
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_GR_GV11B_H_
#define _NVGPU_GR_GV11B_H_

struct gpu_ops;

enum {
	VOLTA_CHANNEL_GPFIFO_A  = 0xC36F,
	VOLTA_A                 = 0xC397,
	VOLTA_COMPUTE_A         = 0xC3C0,
	VOLTA_DMA_COPY_A        = 0xC3B5,
};

void gv11b_init_gr(struct gpu_ops *ops);

#endif
