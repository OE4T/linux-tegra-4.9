/*
 * GP10B MMU
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/dma-mapping.h>
#include "gk20a/gk20a.h"
#include "mm_gp10b.h"
#include "rpfb_gp10b.h"
#include "hw_ram_gp10b.h"
#include "hw_bus_gp10b.h"

u32 gp10b_mm_get_physical_addr_bits(struct gk20a *g)
{
	return 36;
}

static int gp10b_init_mm_setup_hw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct inst_desc *inst_block = &mm->bar1.inst_block;
	phys_addr_t inst_pa = inst_block->cpu_pa;
	int err = 0;

	gk20a_dbg_fn("");

	g->ops.fb.set_mmu_page_size(g);

	inst_pa = (u32)(inst_pa >> bar1_instance_block_shift_gk20a());
	gk20a_dbg_info("bar1 inst block ptr: 0x%08x",  (u32)inst_pa);

	gk20a_writel(g, bus_bar1_block_r(),
			bus_bar1_block_target_vid_mem_f() |
			bus_bar1_block_mode_virtual_f() |
			bus_bar1_block_ptr_f(inst_pa));

	if (g->ops.mm.init_bar2_mm_hw_setup) {
		err = g->ops.mm.init_bar2_mm_hw_setup(g);
		if (err)
			return err;
	}

	if (gk20a_mm_fb_flush(g) || gk20a_mm_fb_flush(g))
		return -EBUSY;

	err = gp10b_replayable_pagefault_buffer_init(g);

	gk20a_dbg_fn("done");
	return err;

}

static int gb10b_init_bar2_vm(struct gk20a *g)
{
	int err;
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = &mm->bar2.vm;
	struct inst_desc *inst_block = &mm->bar2.inst_block;
	u32 big_page_size = gk20a_get_platform(g->dev)->default_big_page_size;

	/* BAR2 aperture size is 32MB */
	mm->bar2.aperture_size = 32 << 20;
	gk20a_dbg_info("bar2 vm size = 0x%x", mm->bar2.aperture_size);
	gk20a_init_vm(mm, vm, big_page_size, SZ_4K,
		mm->bar2.aperture_size, false, "bar2");

	/* allocate instance mem for bar2 */
	err = gk20a_alloc_inst_block(g, inst_block);
	if (err)
		goto clean_up_va;

	gk20a_init_inst_block(inst_block, vm, big_page_size);

	return 0;

clean_up_va:
	gk20a_deinit_vm(vm);
	return err;
}


static int gb10b_init_bar2_mm_hw_setup(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct inst_desc *inst_block = &mm->bar2.inst_block;
	phys_addr_t inst_pa = inst_block->cpu_pa;

	gk20a_dbg_fn("");

	g->ops.fb.set_mmu_page_size(g);

	inst_pa = (u32)(inst_pa >> bus_bar2_block_ptr_shift_v());
	gk20a_dbg_info("bar2 inst block ptr: 0x%08x",  (u32)inst_pa);

	gk20a_writel(g, bus_bar2_block_r(),
		bus_bar2_block_target_vid_mem_f() |
		bus_bar2_block_mode_virtual_f() |
		bus_bar2_block_ptr_f(inst_pa));

	gk20a_dbg_fn("done");
	return 0;
}
void gp10b_init_mm(struct gpu_ops *gops)
{
	gm20b_init_mm(gops);
	gops->mm.get_physical_addr_bits = gk20a_mm_get_physical_addr_bits;
	gops->mm.init_mm_setup_hw = gp10b_init_mm_setup_hw;
	gops->mm.init_bar2_vm = gb10b_init_bar2_vm;
	gops->mm.init_bar2_mm_hw_setup = gb10b_init_bar2_mm_hw_setup;
}
