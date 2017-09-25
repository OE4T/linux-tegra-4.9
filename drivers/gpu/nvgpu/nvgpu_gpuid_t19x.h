/*
 * NVIDIA GPU ID functions, definitions.
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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
