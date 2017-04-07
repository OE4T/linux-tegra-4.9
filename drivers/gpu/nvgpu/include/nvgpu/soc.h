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
#ifndef __NVGPU_SOC_H__
#define __NVGPU_SOC_H__

struct gk20a;

bool nvgpu_platform_is_silicon(struct gk20a *g);
bool nvgpu_platform_is_simulation(struct gk20a *g);
bool nvgpu_platform_is_fpga(struct gk20a *g);
bool nvgpu_is_hypervisor_mode(struct gk20a *g);

#endif
