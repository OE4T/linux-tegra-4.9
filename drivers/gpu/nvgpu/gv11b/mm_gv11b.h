/*
 * GV11B MM
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

#ifndef MM_GV11B_H
#define MM_GV11B_H

#define HW_FAULT_BUF_STATUS_ALLOC_TRUE	1
#define HW_FAULT_BUF_STATUS_ALLOC_FALSE	0

struct gk20a;
struct nvgpu_mem;
struct vm_gk20a;

bool gv11b_mm_is_bar1_supported(struct gk20a *g);
void gv11b_init_inst_block(struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm, u32 big_page_size);
bool gv11b_mm_mmu_fault_pending(struct gk20a *g);
void gv11b_mm_remove_bar2_vm(struct gk20a *g);
int gv11b_init_mm_setup_hw(struct gk20a *g);
int gv11b_init_bar2_mm_hw_setup(struct gk20a *g);
void gv11b_mm_l2_flush(struct gk20a *g, bool invalidate);
u64 gv11b_gpu_phys_addr(struct gk20a *g,
			struct nvgpu_gmmu_attrs *attrs, u64 phys);
void gv11b_mm_fault_info_mem_destroy(struct gk20a *g);

#endif
