/*
 * GV100 memory management
 *
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

#include "gk20a/gk20a.h"
#include "gv11b/mm_gv11b.h"
#include "gv100/mm_gv100.h"

#include <nvgpu/hw/gv100/hw_fb_gv100.h>

static size_t gv100_mm_get_vidmem_size(struct gk20a *g)
{
	u32 range = gk20a_readl(g, fb_mmu_local_memory_range_r());
	u32 mag = fb_mmu_local_memory_range_lower_mag_v(range);
	u32 scale = fb_mmu_local_memory_range_lower_scale_v(range);
	u32 ecc = fb_mmu_local_memory_range_ecc_mode_v(range);
	size_t bytes = ((size_t)mag << scale) * SZ_1M;

	if (ecc)
		bytes = bytes / 16 * 15;

	return bytes;
}

void gv100_init_mm(struct gpu_ops *gops)
{
	gv11b_init_mm(gops);
	gops->mm.get_vidmem_size = gv100_mm_get_vidmem_size;
	gops->mm.get_physical_addr_bits = NULL;
}
