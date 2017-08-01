/*
 * GM20B GMMU
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVHOST_GM20B_MM
#define _NVHOST_GM20B_MM
struct gk20a;

#define PDE_ADDR_START(x, y)	((x) &  ~((0x1UL << (y)) - 1))
#define PDE_ADDR_END(x, y)	((x) | ((0x1UL << (y)) - 1))

void gm20b_mm_set_big_page_size(struct gk20a *g,
				struct nvgpu_mem *mem, int size);
u32 gm20b_mm_get_big_page_sizes(void);
u32 gm20b_mm_get_default_big_page_size(void);
bool gm20b_mm_support_sparse(struct gk20a *g);
bool gm20b_mm_is_bar1_supported(struct gk20a *g);
int gm20b_mm_mmu_vpr_info_fetch(struct gk20a *g);
u64 gm20b_gpu_phys_addr(struct gk20a *g,
			struct nvgpu_gmmu_attrs *attrs, u64 phys);
#endif
