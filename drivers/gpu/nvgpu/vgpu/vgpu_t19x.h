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

#ifndef _VGPU_T19X_H_
#define _VGPU_T19X_H_

struct gk20a;

int vgpu_gv11b_init_hal(struct gk20a *g);

#define vgpu_t19x_init_hal(g) vgpu_gv11b_init_hal(g)

#define TEGRA_19x_VGPU_COMPAT_TEGRA "nvidia,gv11b-vgpu"
extern struct gk20a_platform gv11b_vgpu_tegra_platform;
#define t19x_vgpu_tegra_platform gv11b_vgpu_tegra_platform

#endif
