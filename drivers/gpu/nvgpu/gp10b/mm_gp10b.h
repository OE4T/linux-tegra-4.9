/*
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

#ifndef MM_GP10B_H
#define MM_GP10B_H

#include <linux/version.h>

#define NVGPU_MM_GET_IO_COHERENCE_BIT	35

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
#define NVGPU_USE_NEW_ALLOCATOR	1
#else
#define NVGPU_USE_NEW_ALLOCATOR	0
#endif

struct gpu_ops;

void gp10b_init_mm(struct gpu_ops *gops);
#endif
