/*
 * GP10B MMU
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

#include <linux/pm_runtime.h>
#include "gk20a/gk20a.h"

u32 gp10b_mm_get_physical_addr_bits(struct gk20a *g)
{
	return 36;
}

void gp10b_init_mm(struct gpu_ops *gops)
{
	gm20b_init_mm(gops);
	gops->mm.get_physical_addr_bits = gk20a_mm_get_physical_addr_bits;
}
