/*
 * GM20B MMU
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
#include "mm_gm20b.h"
#include "hw_gmmu_gm20b.h"
#include "hw_fb_gm20b.h"
#include "hw_gr_gm20b.h"
#include "hw_ram_gm20b.h"

static int allocate_gmmu_ptes_sparse(struct vm_gk20a *vm,
				enum gmmu_pgsz_gk20a pgsz_idx,
				u64 first_vaddr, u64 last_vaddr,
				bool clear, bool refplus)
{
	int err;
	u32 pte_lo, pte_hi;
	u32 pde_lo, pde_hi;
	u32 pte_w[2] = {0, 0}; /* invalid pte */
	u64 addr = 0;
	u32 pte_cur;
	void *pte_kv_cur;
	struct page_table_gk20a *pte;

	gk20a_dbg_fn("");

	pde_range_from_vaddr_range(vm, first_vaddr, last_vaddr,
					&pde_lo, &pde_hi);

	gk20a_dbg(gpu_dbg_pte, "size_idx=%d, pde_lo=%d, pde_hi=%d",
			pgsz_idx, pde_lo, pde_hi);

	/* Expect ptes of the same pde */
	BUG_ON(pde_lo != pde_hi);

	pte = vm->pdes.ptes[pgsz_idx] + pde_lo;
	if (refplus)
		pte->ref_cnt++;

	pte_lo = pte_index_from_vaddr(vm, first_vaddr, pgsz_idx);
	pte_hi = pte_index_from_vaddr(vm, last_vaddr, pgsz_idx);

	/* get cpu access to the ptes */
	err = map_gmmu_pages(pte->ref, pte->sgt, &pte_kv_cur, pte->size);
	if (err)
		goto fail;

	gk20a_dbg(gpu_dbg_pte, "pte_lo=%d, pte_hi=%d", pte_lo, pte_hi);
	for (pte_cur = pte_lo; pte_cur <= pte_hi; pte_cur++) {
		pte_w[0] = gmmu_pte_valid_false_f();
		pte_w[1] = clear ? 0 : gmmu_pte_vol_true_f();

		gk20a_dbg(gpu_dbg_pte,
			   "pte_cur=%d addr=%llx refs=%d"
			   " [0x%08x,0x%08x]",
			   pte_cur, addr,
			   pte->ref_cnt, pte_w[1], pte_w[0]);

		gk20a_mem_wr32(pte_kv_cur + pte_cur*8, 0, pte_w[0]);
		gk20a_mem_wr32(pte_kv_cur + pte_cur*8, 1, pte_w[1]);
	}

	unmap_gmmu_pages(pte->ref, pte->sgt, pte_kv_cur);

	smp_mb();
	vm->tlb_dirty = true;
	gk20a_dbg_fn("set tlb dirty");

	return 0;
fail:
	return err;

}

static bool gm20b_vm_is_pde_in_range(struct vm_gk20a *vm, u64 vaddr_lo,
					u64 vaddr_hi, u32 pde)
{
	u64 pde_vaddr_lo, pde_vaddr_hi;

	gk20a_dbg_fn("");

	pde_vaddr_lo = (u64)pde << vm->pde_stride_shift;
	pde_vaddr_hi = pde_vaddr_lo |
			((0x1UL << (vm->pde_stride_shift)) - 1);

	return ((vaddr_lo <= pde_vaddr_lo) && (vaddr_hi) >= pde_vaddr_hi);
}

static int gm20b_vm_put_sparse(struct vm_gk20a *vm, u64 vaddr,
			       u32 num_pages, u32 pgsz_idx, bool refplus)
{
	struct mm_gk20a *mm = vm->mm;
	u32 pgsz = vm->gmmu_page_sizes[pgsz_idx];
	u32 pde_shift = vm->pde_stride_shift;
	u64 vaddr_hi;
	u64 vaddr_pde_start;
	u32 i;
	u32 pde_lo, pde_hi;
	int err;

	gk20a_dbg_fn("");

	vaddr_hi = vaddr + pgsz * num_pages - 1;
	pde_range_from_vaddr_range(vm,
				   vaddr,
				   vaddr_hi,
				   &pde_lo, &pde_hi);

	gk20a_dbg_info("vaddr: 0x%llx, vaddr_hi: 0x%llx, pde_lo: 0x%x, "
			"pde_hi: 0x%x, pgsz: %d, pde_stride_shift: %d",
			vaddr, vaddr_hi, pde_lo, pde_hi, pgsz,
			vm->pde_stride_shift);

	for (i = pde_lo; i <= pde_hi; i++) {
		/* Mark all ptes as sparse. */
		err = validate_gmmu_page_table_gk20a_locked(vm, i,
						    pgsz_idx);
		if (err) {
			gk20a_err(dev_from_vm(vm),
				"failed to validate page table %d: %d",
				i, err);
			goto fail;
		}

		if (gm20b_vm_is_pde_in_range(vm, vaddr, vaddr_hi, i)) {
			/* entire pde is marked as sparse */
			vaddr_pde_start = (u64)i << pde_shift;
			allocate_gmmu_ptes_sparse(vm, pgsz_idx,
				vaddr_pde_start,
				PDE_ADDR_END(vaddr_pde_start,
				pde_shift), false, refplus);
		} else {
			/* Check leading and trailing spaces which doesn't fit
			 * into entire pde. */
			if (pde_lo == pde_hi)
				allocate_gmmu_ptes_sparse(vm, pgsz_idx, vaddr,
						vaddr_hi, false, refplus);
			else if (i == pde_lo)
				allocate_gmmu_ptes_sparse(vm, pgsz_idx, vaddr,
					PDE_ADDR_END(vaddr, pde_shift), false,
					refplus);
			else
				allocate_gmmu_ptes_sparse(vm, pgsz_idx,
					PDE_ADDR_START(vaddr_hi, pde_shift),
							vaddr_hi, false,
							refplus);
		}
	}

	gk20a_mm_l2_flush(mm->g, true);

	return 0;

fail:
	WARN_ON(1);

	return err;
}

static int gm20b_mm_mmu_vpr_info_fetch_wait(struct gk20a *g,
		const unsigned int msec)
{
	unsigned long timeout;

	if (tegra_platform_is_silicon())
		timeout = jiffies + msecs_to_jiffies(msec);
	else
		timeout = msecs_to_jiffies(msec);

	while (1) {
		u32 val;
		val = gk20a_readl(g, fb_mmu_vpr_info_r());
		if (fb_mmu_vpr_info_fetch_v(val) ==
				fb_mmu_vpr_info_fetch_false_v())
			break;
		if (tegra_platform_is_silicon()) {
			if (WARN_ON(time_after(jiffies, timeout)))
				return -ETIME;
		} else if (--timeout == 0)
			return -ETIME;
	}
	return 0;
}

int gm20b_mm_mmu_vpr_info_fetch(struct gk20a *g)
{
	int ret = 0;

	gk20a_busy_noresume(g->dev);
#ifdef CONFIG_PM_RUNTIME
	if (!pm_runtime_active(&g->dev->dev))
		goto fail;
#endif

	if (gm20b_mm_mmu_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT)) {
		ret = -ETIME;
		goto fail;
	}

	gk20a_writel(g, fb_mmu_vpr_info_r(),
			fb_mmu_vpr_info_fetch_true_v());

	ret = gm20b_mm_mmu_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT);

fail:
	pm_runtime_put(&g->dev->dev);
	return ret;
}

static void gm20b_vm_clear_sparse(struct vm_gk20a *vm, u64 vaddr,
			       u64 size, u32 pgsz_idx) {
	u64 vaddr_hi;
	u32 pde_lo, pde_hi, pde_i;

	gk20a_dbg_fn("");
	vaddr_hi = vaddr + size - 1;
	pde_range_from_vaddr_range(vm,
				   vaddr,
				   vaddr_hi,
				   &pde_lo, &pde_hi);

	gk20a_dbg_info("vaddr: 0x%llx, vaddr_hi: 0x%llx, pde_lo: 0x%x, "
			"pde_hi: 0x%x, pgsz_idx: %d, pde_stride_shift: %d",
			vaddr, vaddr_hi, pde_lo, pde_hi, pgsz_idx,
			vm->pde_stride_shift);

	for (pde_i = pde_lo; pde_i <= pde_hi; pde_i++) {
		struct page_table_gk20a *pte = vm->pdes.ptes[pgsz_idx] + pde_i;
		pte->ref_cnt--;

		if (pte->ref_cnt == 0) {
			free_gmmu_pages(vm, pte->ref, pte->sgt,
				vm->page_table_sizing[pgsz_idx].order,
				pte->size);
			pte->ref = NULL;
			update_gmmu_pde_locked(vm, pde_i);
		}
	}

	return;
}

static bool gm20b_mm_mmu_debug_mode_enabled(struct gk20a *g)
{
	u32 debug_ctrl = gk20a_readl(g, gr_gpcs_pri_mmu_debug_ctrl_r());
	return gr_gpcs_pri_mmu_debug_ctrl_debug_v(debug_ctrl) ==
		gr_gpcs_pri_mmu_debug_ctrl_debug_enabled_v();
}

static void gm20b_mm_set_big_page_size(struct gk20a *g,
				void *inst_ptr, int size)
{
	u32 val;

	gk20a_dbg_fn("");

	gk20a_dbg_info("big page size %d\n", size);
	val = gk20a_mem_rd32(inst_ptr, ram_in_big_page_size_w());
	val &= ~ram_in_big_page_size_m();

	if (size == SZ_64K)
		val |= ram_in_big_page_size_64kb_f();
	else
		val |= ram_in_big_page_size_128kb_f();

	gk20a_mem_wr32(inst_ptr, ram_in_big_page_size_w(), val);
	gk20a_dbg_fn("done");
}

void gm20b_init_mm(struct gpu_ops *gops)
{
	gops->mm.set_sparse = gm20b_vm_put_sparse;
	gops->mm.clear_sparse = gm20b_vm_clear_sparse;
	gops->mm.is_debug_mode_enabled = gm20b_mm_mmu_debug_mode_enabled;
	gops->mm.gmmu_map = gk20a_locked_gmmu_map;
	gops->mm.gmmu_unmap = gk20a_locked_gmmu_unmap;
	gops->mm.vm_remove = gk20a_vm_remove_support;
	gops->mm.vm_alloc_share = gk20a_vm_alloc_share;
	gops->mm.vm_bind_channel = gk20a_vm_bind_channel;
	gops->mm.fb_flush = gk20a_mm_fb_flush;
	gops->mm.l2_invalidate = gk20a_mm_l2_invalidate;
	gops->mm.l2_flush = gk20a_mm_l2_flush;
	gops->mm.tlb_invalidate = gk20a_mm_tlb_invalidate;
	gops->mm.set_big_page_size = gm20b_mm_set_big_page_size;
}
