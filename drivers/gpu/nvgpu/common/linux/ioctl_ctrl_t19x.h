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

#ifndef _NVGPU_IOCTL_CTRL_T19X
#define _NVGPU_IOCTL_CTRL_T19X

#include <nvgpu/types.h>

struct gk20a;

u64 nvgpu_ctrl_ioctl_gpu_characteristics_flags_t19x(struct gk20a *g);

#endif
