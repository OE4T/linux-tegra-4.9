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
#ifndef __NVGPU_FUSE_H__
#define __NVGPU_FUSE_H__

int nvgpu_tegra_fuse_read(unsigned long offset, u32 *value);
void nvgpu_tegra_fuse_write(u32 value, unsigned long offset);

int nvgpu_tegra_get_gpu_speedo_id(void);

#endif
