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

#include <nvgpu/types.h>
#include <nvgpu/enabled.h>
#include <nvgpu/enabled_t19x.h>

#include "ioctl_ctrl_t19x.h"
#include "common/linux/os_linux.h"
#include "gk20a/gk20a.h"

u64 nvgpu_ctrl_ioctl_gpu_characteristics_flags_t19x(struct gk20a *g)
{
	u64 ioctl_flags = 0;

	if (nvgpu_is_enabled(g, NVGPU_SUPPORT_TSG_SUBCONTEXTS))
		ioctl_flags |= NVGPU_GPU_FLAGS_SUPPORT_TSG_SUBCONTEXTS;

	return ioctl_flags;
}

