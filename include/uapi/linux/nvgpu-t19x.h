/*
 * NVGPU Public Interface Header
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
#define NVGPU_GPU_ARCH_GV100 0x00000140
#define NVGPU_GPU_IMPL_GV11B 0x0000000B
#define NVGPU_GPU_IMPL_GV100 0x00000000

/*
 * this flag is used in struct nvgpu_as_map_buffer_ex_args
 * to provide L3 cache allocation hint
 */
#define NVGPU_AS_MAP_BUFFER_FLAGS_L3_ALLOC	(1 << 7)

/*  subcontexts are available */
#define NVGPU_GPU_FLAGS_SUPPORT_TSG_SUBCONTEXTS         (1ULL << 22)

struct nvgpu_tsg_bind_channel_ex_args {
	/* in: channel fd */
	__s32 channel_fd;

	/* in: VEID in Volta */
	__u32 subcontext_id;

	__u64 reserved[2];
};

#define NVGPU_TSG_IOCTL_BIND_CHANNEL_EX \
	_IOWR(NVGPU_TSG_IOCTL_MAGIC, 11, struct nvgpu_tsg_bind_channel_ex_args)

#define NVGPU_TSG_IOCTL_MAX NVGPU_TSG_IOCTL_BIND_CHANNEL_EX

#define NVGPU_TSG_IOCTL_MAX_ARG	sizeof(struct nvgpu_tsg_bind_channel_ex_args)

#endif /* _UAPI__LINUX_NVGPU_T19X_IOCTL_H_ */
