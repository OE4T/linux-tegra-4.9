/*
 * GM20B BUS
 *
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

#ifndef _NVGPU_GM20B_BUS
#define _NVGPU_GM20B_BUS

struct gk20a;
struct nvgpu_mem;

int gm20b_bus_bar1_bind(struct gk20a *g, struct nvgpu_mem *bar1_inst);

#endif
