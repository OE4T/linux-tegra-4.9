/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include "vgpu_gr_gp10b.h"
#include "vgpu/gm20b/vgpu_gr_gm20b.h"

void vgpu_gp10b_init_gr_ops(struct gpu_ops *gops)
{
	vgpu_gm20b_init_gr_ops(gops);
}
