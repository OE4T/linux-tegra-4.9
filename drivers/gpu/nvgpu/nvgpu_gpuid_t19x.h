/*
 * NVIDIA GPU ID functions, definitions.
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
#ifndef _NVGPU_GPUID_T19X_H_
#define _NVGPU_GPUID_T19X_H_

#define NVGPU_GPUID_GV11B \
	GK20A_GPUID(NVGPU_GPU_ARCH_GV110, NVGPU_GPU_IMPL_GV11B)
#define NVGPU_GPUID_GV100 \
	GK20A_GPUID(NVGPU_GPU_ARCH_GV100, NVGPU_GPU_IMPL_GV100)


#define NVGPU_COMPAT_TEGRA_GV11B "nvidia,gv11b"
#define NVGPU_COMPAT_GENERIC_GV11B "nvidia,generic-gv11b"


#define TEGRA_19x_GPUID NVGPU_GPUID_GV11B
#define TEGRA_19x_GPUID_HAL gv11b_init_hal
#define TEGRA_19x_GPU_COMPAT_TEGRA NVGPU_COMPAT_TEGRA_GV11B
#define TEGRA_19x_GPU_COMPAT_GENERIC NVGPU_COMPAT_GENERIC_GV11B

#define BIGGPU_19x_GPUID NVGPU_GPUID_GV100
#define BIGGPU_19x_GPUID_HAL gv100_init_hal

struct gpu_ops;
extern int gv11b_init_hal(struct gk20a *);
extern int gv100_init_hal(struct gk20a *);
extern struct gk20a_platform t19x_gpu_tegra_platform;

#endif
