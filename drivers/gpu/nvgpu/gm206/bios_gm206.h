/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_BIOS_GM206_H
#define NVGPU_BIOS_GM206_H

struct gk20a;
struct gpu_ops;

int gm206_bios_init(struct gk20a *g);
void gm206_init_bios_ops(struct gpu_ops *gops);

#endif
