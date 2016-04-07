/*
 * NVGPU Public Interface Header
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

/* This file is meant to extend nvgpu.h, not replace it
 * as such, be sure that nvgpu.h is actually the file performing the
 * inclusion, to the extent that's possible.
 */
#ifndef _UAPI__LINUX_NVGPU_IOCTL_H
#    error "This file is to be included within nvgpu.h only."
#endif

#ifndef _UAPI__LINUX_NVGPU_T19X_IOCTL_H_
#define _UAPI__LINUX_NVGPU_T19X_IOCTL_H_

#define NVGPU_GPU_ARCH_GV110 0x00000150
#define NVGPU_GPU_IMPL_GV11B 0x0000000B

#endif /* _UAPI__LINUX_NVGPU_T19X_IOCTL_H_ */
