/*
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

#ifndef MM_GP10B_H
#define MM_GP10B_H

struct gk20a;
struct gk20a_mmu_level;
struct nvgpu_mem;
struct vm_gk20a;

u32 gp10b_mm_get_default_big_page_size(void);
u32 gp10b_mm_get_physical_addr_bits(struct gk20a *g);
int gp10b_init_mm_setup_hw(struct gk20a *g);
int gb10b_init_bar2_vm(struct gk20a *g);
int gb10b_init_bar2_mm_hw_setup(struct gk20a *g);
const struct gk20a_mmu_level *gp10b_mm_get_mmu_levels(struct gk20a *g,
	u32 big_page_size);
void gp10b_mm_init_pdb(struct gk20a *g, struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm);
void gp10b_remove_bar2_vm(struct gk20a *g);

#endif
