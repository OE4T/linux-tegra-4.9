/*
 * GV11B MMU
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

#include <linux/pm_runtime.h>

#include "gk20a/gk20a.h"

#include "gp10b/mm_gp10b.h"

#include "mm_gv11b.h"

#include <nvgpu/hw/gv11b/hw_fb_gv11b.h>

static bool gv11b_mm_is_bar1_supported(struct gk20a *g)
{
	return false;
}

static void gv11b_init_inst_block(struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm, u32 big_page_size)
{
	struct gk20a *g = gk20a_from_vm(vm);

	gk20a_dbg_info("inst block phys = 0x%llx, kv = 0x%p",
		gk20a_mm_inst_block_addr(g, inst_block), inst_block->cpu_va);

	g->ops.mm.init_pdb(g, inst_block, vm);

	if (big_page_size && g->ops.mm.set_big_page_size)
		g->ops.mm.set_big_page_size(g, inst_block, big_page_size);
}

static bool gv11b_mm_mmu_fault_pending(struct gk20a *g)
{
	if (gk20a_readl(g, fb_niso_intr_r()) &
		(fb_niso_intr_mmu_nonreplayable_fault_notify_pending_f() |
		fb_niso_intr_mmu_nonreplayable_fault_overflow_pending_f()))
		return true;

	return false;
}

void gv11b_init_mm(struct gpu_ops *gops)
{
	gp10b_init_mm(gops);
	gops->mm.is_bar1_supported = gv11b_mm_is_bar1_supported;
	gops->mm.init_inst_block = gv11b_init_inst_block;
	gops->mm.init_mm_setup_hw = gk20a_init_mm_setup_hw;
	gops->mm.mmu_fault_pending = gv11b_mm_mmu_fault_pending;
}
