/*
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

#include <linux/string.h>
#include "gk20a/gk20a.h"
#include "fecs_trace_vgpu.h"

void vgpu_init_fecs_trace_ops(struct gpu_ops *ops)
{
	memset(&ops->fecs_trace, 0, sizeof(ops->fecs_trace));
}
