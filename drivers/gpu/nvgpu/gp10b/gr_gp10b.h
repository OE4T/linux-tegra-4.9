/*
 * GM20B GPU GR
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_GR_GP10B_H_
#define _NVGPU_GR_GP10B_H_

struct gk20a;

enum {
	PASCAL_CHANNEL_GPFIFO_A  = 0xC06F,
	PASCAL_A                 = 0xC097,
	PASCAL_COMPUTE_A         = 0xC0C0,
	PASCAL_DMA_COPY_A        = 0xC0B5,
};

#define NVC097_SET_ALPHA_CIRCULAR_BUFFER_SIZE	0x02dc
#define NVC097_SET_CIRCULAR_BUFFER_SIZE		0x1280
#define NVC097_SET_SHADER_EXCEPTIONS		0x1528
#define NVC0C0_SET_SHADER_EXCEPTIONS		0x1528

void gp10b_init_gr(struct gpu_ops *ops);

#endif
