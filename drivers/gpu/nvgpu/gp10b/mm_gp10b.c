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
#include "hw_gmmu_gp10b.h"
#include "gk20a/semaphore_gk20a.h"

static u32 gp10b_mm_get_physical_addr_bits(struct gk20a *g)
{
	return 36;
}

static int gp10b_init_mm_setup_hw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct mem_desc *inst_block = &mm->bar1.inst_block;
	phys_addr_t inst_pa = gk20a_mem_phys(inst_block);
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
	struct mem_desc *inst_block = &mm->bar2.inst_block;
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
	struct mem_desc *inst_block = &mm->bar2.inst_block;
	phys_addr_t inst_pa = gk20a_mem_phys(inst_block);

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

static u64 gp10b_mm_phys_addr_translate(struct gk20a *g, u64 phys_addr,
		u32 flags)
{
	if (!device_is_iommuable(dev_from_gk20a(g)))
		if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_IO_COHERENT)
			return phys_addr |
				1ULL << NVGPU_MM_GET_IO_COHERENCE_BIT;

	return phys_addr;
}

static u64 gp10b_mm_iova_addr(struct gk20a *g, struct scatterlist *sgl,
		u32 flags)
{
	if (!device_is_iommuable(dev_from_gk20a(g)))
		return gp10b_mm_phys_addr_translate(g, sg_phys(sgl), flags);

	if (sg_dma_address(sgl) == 0)
		return gp10b_mm_phys_addr_translate(g, sg_phys(sgl), flags);

	if (sg_dma_address(sgl) == DMA_ERROR_CODE)
		return 0;

	return gk20a_mm_smmu_vaddr_translate(g, sg_dma_address(sgl));
}

static u32 *pde3_from_index(struct gk20a_mm_entry *entry, u32 i)
{
	return (u32 *) (((u8 *)entry->cpu_va) + i*gmmu_new_pde__size_v());
}

static int update_gmmu_pde3_locked(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *parent,
			   u32 i, u32 gmmu_pgsz_idx,
			   struct scatterlist **sgl,
			   u64 *offset,
			   u64 *iova,
			   u32 kind_v, u32 *ctag,
			   bool cacheable, bool unmapped_pte,
			   int rw_flag, bool sparse, bool priv)
{
	u64 pte_addr = 0;
	u64 pde_addr = 0;
	struct gk20a_mm_entry *pte = parent->entries + i;
	u32 pde_v[2] = {0, 0};
	u32 *pde;

	gk20a_dbg_fn("");

	pte_addr = virt_to_phys(pte->cpu_va)
		   >> gmmu_new_pde_address_shift_v();
	pde_addr = virt_to_phys(parent->cpu_va);

	pde_v[0] |= gmmu_new_pde_aperture_video_memory_f();
	pde_v[0] |= gmmu_new_pde_address_sys_f(u64_lo32(pte_addr));
	pde_v[0] |= gmmu_new_pde_vol_true_f();
	pde_v[1] |= pte_addr >> 24;
	pde = pde3_from_index(parent, i);

	gk20a_mem_wr32(pde, 0, pde_v[0]);
	gk20a_mem_wr32(pde, 1, pde_v[1]);

	gk20a_dbg(gpu_dbg_pte, "pde:%d,sz=%d = 0x%x,0x%08x",
		  i, gmmu_pgsz_idx, pde_v[1], pde_v[0]);
	gk20a_dbg_fn("done");
	return 0;
}

static u32 *pde0_from_index(struct gk20a_mm_entry *entry, u32 i)
{
	return (u32 *) (((u8 *)entry->cpu_va) + i*gmmu_new_dual_pde__size_v());
}

static int update_gmmu_pde0_locked(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *pte,
			   u32 i, u32 gmmu_pgsz_idx,
			   struct scatterlist **sgl,
			   u64 *offset,
			   u64 *iova,
			   u32 kind_v, u32 *ctag,
			   bool cacheable, bool unmapped_pte,
			   int rw_flag, bool sparse, bool priv)
{
	bool small_valid, big_valid;
	u32 pte_addr_small = 0, pte_addr_big = 0;
	struct gk20a_mm_entry *entry = pte->entries + i;
	u32 pde_v[4] = {0, 0, 0, 0};
	u32 *pde;

	gk20a_dbg_fn("");

	small_valid = entry->size && entry->pgsz == gmmu_page_size_small;
	big_valid = entry->size && entry->pgsz == gmmu_page_size_big;

	if (small_valid)
		pte_addr_small = virt_to_phys(entry->cpu_va)
				 >> gmmu_new_dual_pde_address_shift_v();

	if (big_valid)
		pte_addr_big = virt_to_phys(entry->cpu_va)
			       >> gmmu_new_dual_pde_address_big_shift_v();

	if (small_valid) {
		pde_v[2] |= gmmu_new_dual_pde_address_small_sys_f(pte_addr_small);
		pde_v[2] |= gmmu_new_dual_pde_aperture_small_video_memory_f();
		pde_v[2] |= gmmu_new_dual_pde_vol_small_true_f();
		pde_v[3] |= pte_addr_small >> 24;
	}

	if (big_valid) {
		pde_v[0] |= gmmu_new_dual_pde_address_big_sys_f(pte_addr_big);
		pde_v[0] |= gmmu_new_dual_pde_vol_big_true_f();
		pde_v[0] |= gmmu_new_dual_pde_aperture_big_video_memory_f();
		pde_v[1] |= pte_addr_big >> 28;
	}

	pde = pde0_from_index(pte, i);

	gk20a_mem_wr32(pde, 0, pde_v[0]);
	gk20a_mem_wr32(pde, 1, pde_v[1]);
	gk20a_mem_wr32(pde, 2, pde_v[2]);
	gk20a_mem_wr32(pde, 3, pde_v[3]);

	gk20a_dbg(gpu_dbg_pte, "pde:%d,sz=%d [0x%08x, 0x%08x, 0x%x, 0x%08x]",
		  i, gmmu_pgsz_idx, pde_v[3], pde_v[2], pde_v[1], pde_v[0]);
	gk20a_dbg_fn("done");
	return 0;
}

static int update_gmmu_pte_locked(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *pte,
			   u32 i, u32 gmmu_pgsz_idx,
			   struct scatterlist **sgl,
			   u64 *offset,
			   u64 *iova,
			   u32 kind_v, u32 *ctag,
			   bool cacheable, bool unmapped_pte,
			   int rw_flag, bool sparse, bool priv)
{
	struct gk20a *g = vm->mm->g;
	u32 page_size  = vm->gmmu_page_sizes[gmmu_pgsz_idx];
	u32 ctag_granularity = g->ops.fb.compression_page_size(g);
	u32 pte_w[2] = {0, 0}; /* invalid pte */

	gk20a_dbg_fn("");

	if (*iova) {
		if (unmapped_pte)
			pte_w[0] = gmmu_new_pte_valid_false_f();
		else
			pte_w[0] = gmmu_new_pte_valid_true_f();
		pte_w[0] |= gmmu_new_pte_aperture_video_memory_f() |
			    gmmu_new_pte_address_sys_f(*iova
			      >> gmmu_new_pte_address_shift_v());

		if (priv)
			pte_w[0] |= gmmu_new_pte_privilege_true_f();

		pte_w[1] = *iova >> (24 + gmmu_new_pte_address_shift_v()) |
			   gmmu_new_pte_kind_f(kind_v) |
			   gmmu_new_pte_comptagline_f(*ctag / ctag_granularity);

		if (rw_flag == gk20a_mem_flag_read_only)
			pte_w[0] |= gmmu_new_pte_read_only_true_f();
		if (unmapped_pte && !cacheable)
			pte_w[0] |= gmmu_new_pte_read_only_true_f();
		else if (!cacheable)
			pte_w[0] |= gmmu_new_pte_vol_true_f();

		gk20a_dbg(gpu_dbg_pte, "pte=%d iova=0x%llx kind=%d"
			   " ctag=%d vol=%d"
			   " [0x%08x, 0x%08x]",
			   i, *iova,
			   kind_v, *ctag / ctag_granularity, !cacheable,
			   pte_w[1], pte_w[0]);

		if (*ctag)
			*ctag += page_size;
	} else if (sparse) {
		pte_w[0] = gmmu_new_pte_valid_false_f();
		pte_w[0] |= gmmu_new_pte_vol_true_f();
	} else {
		gk20a_dbg(gpu_dbg_pte, "pte_cur=%d [0x0,0x0]", i);
	}

	gk20a_mem_wr32(pte->cpu_va + i*8, 0, pte_w[0]);
	gk20a_mem_wr32(pte->cpu_va + i*8, 1, pte_w[1]);

	if (*iova) {
		*iova += page_size;
		*offset += page_size;
		if (*sgl && *offset + page_size > (*sgl)->length) {
			u64 new_iova;
			*sgl = sg_next(*sgl);
			if (*sgl) {
				new_iova = sg_phys(*sgl);
				gk20a_dbg(gpu_dbg_pte, "chunk address %llx, size %d",
					  new_iova, (*sgl)->length);
				if (new_iova) {
					*offset = 0;
					*iova = new_iova;
				}
			}
		}
	}
	gk20a_dbg_fn("done");
	return 0;
}

static const struct gk20a_mmu_level gp10b_mm_levels[] = {
	{.hi_bit = {48, 48},
	 .lo_bit = {47, 47},
	 .update_entry = update_gmmu_pde3_locked,
	 .entry_size = 8},
	{.hi_bit = {46, 46},
	 .lo_bit = {38, 38},
	 .update_entry = update_gmmu_pde3_locked,
	 .entry_size = 8},
	{.hi_bit = {37, 37},
	 .lo_bit = {29, 29},
	 .update_entry = update_gmmu_pde3_locked,
	 .entry_size = 8},
	{.hi_bit = {28, 28},
	 .lo_bit = {21, 21},
	 .update_entry = update_gmmu_pde0_locked,
	 .entry_size = 16},
	{.hi_bit = {20, 20},
	 .lo_bit = {12, 16},
	 .update_entry = update_gmmu_pte_locked,
	 .entry_size = 8},
	{.update_entry = NULL}
};

static const struct gk20a_mmu_level *gp10b_mm_get_mmu_levels(struct gk20a *g,
	u32 big_page_size)
{
	return gp10b_mm_levels;
}

static void gp10b_mm_init_pdb(struct gk20a *g, void *inst_ptr, u64 pdb_addr)
{
	u32 pdb_addr_lo = u64_lo32(pdb_addr >> ram_in_base_shift_v());
	u32 pdb_addr_hi = u64_hi32(pdb_addr);

	gk20a_mem_wr32(inst_ptr, ram_in_page_dir_base_lo_w(),
		ram_in_page_dir_base_target_vid_mem_f() |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(pdb_addr_lo) |
		1 << 10);

	gk20a_mem_wr32(inst_ptr, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(pdb_addr_hi));
}

void gp10b_init_mm(struct gpu_ops *gops)
{
	gm20b_init_mm(gops);
	gops->mm.get_physical_addr_bits = gp10b_mm_get_physical_addr_bits;
	gops->mm.init_mm_setup_hw = gp10b_init_mm_setup_hw;
	gops->mm.init_bar2_vm = gb10b_init_bar2_vm;
	gops->mm.init_bar2_mm_hw_setup = gb10b_init_bar2_mm_hw_setup;
	gops->mm.get_iova_addr = gp10b_mm_iova_addr;
	gops->mm.get_mmu_levels = gp10b_mm_get_mmu_levels;
	gops->mm.init_pdb = gp10b_mm_init_pdb;
}
