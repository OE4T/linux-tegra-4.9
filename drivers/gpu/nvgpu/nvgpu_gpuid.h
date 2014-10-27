/*
 * NVIDIA GPU ID functions, definitions.
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
#ifndef _NVGPU_GPUID_H_
#define _NVGPU_GPUID_H_

#define NVGPU_GPU_ARCHITECTURE_SHIFT 4

/* constructs unique and compact GPUID from nvgpu_gpu_characteristics
 * arch/impl fields */
#define NVGPU_GPUID(arch, impl) ((u32) ((arch) | (impl)))

#define NVGPU_GPUID_GK20A \
	NVGPU_GPUID(NVGPU_GPU_ARCH_GK100, NVGPU_GPU_IMPL_GK20A)

#define NVGPU_GPUID_GM20B \
	NVGPU_GPUID(NVGPU_GPU_ARCH_GM200, NVGPU_GPU_IMPL_GM20B)

#endif /* _NVGPU_GPU_ID_H_ */
