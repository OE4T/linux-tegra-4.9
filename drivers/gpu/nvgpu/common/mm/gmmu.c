/*
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <nvgpu/log.h>
#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/nvgpu_mem.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

static int alloc_gmmu_phys_pages(struct vm_gk20a *vm, u32 order,
				 struct gk20a_mm_entry *entry)
{
	u32 num_pages = 1 << order;
	u32 len = num_pages * PAGE_SIZE;
	int err;
	struct page *pages;
	struct gk20a *g = vm->mm->g;

	/* note: mem_desc slightly abused (wrt. alloc_gmmu_pages) */

	pages = alloc_pages(GFP_KERNEL, order);
	if (!pages) {
		nvgpu_log(g, gpu_dbg_pte, "alloc_pages failed");
		goto err_out;
	}
	entry->mem.priv.sgt = nvgpu_kzalloc(g, sizeof(*entry->mem.priv.sgt));
	if (!entry->mem.priv.sgt) {
		nvgpu_log(g, gpu_dbg_pte, "cannot allocate sg table");
		goto err_alloced;
	}
	err = sg_alloc_table(entry->mem.priv.sgt, 1, GFP_KERNEL);
	if (err) {
		nvgpu_log(g, gpu_dbg_pte, "sg_alloc_table failed");
		goto err_sg_table;
	}
	sg_set_page(entry->mem.priv.sgt->sgl, pages, len, 0);
	entry->mem.cpu_va = page_address(pages);
	memset(entry->mem.cpu_va, 0, len);
	entry->mem.size = len;
	entry->mem.aperture = APERTURE_SYSMEM;
	FLUSH_CPU_DCACHE(entry->mem.cpu_va,
			 sg_phys(entry->mem.priv.sgt->sgl), len);

	return 0;

err_sg_table:
	nvgpu_kfree(vm->mm->g, entry->mem.priv.sgt);
err_alloced:
	__free_pages(pages, order);
err_out:
	return -ENOMEM;
}

static int nvgpu_alloc_gmmu_pages(struct vm_gk20a *vm, u32 order,
				  struct gk20a_mm_entry *entry)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u32 num_pages = 1 << order;
	u32 len = num_pages * PAGE_SIZE;
	int err;

	if (g->is_fmodel)
		return alloc_gmmu_phys_pages(vm, order, entry);

	/*
	 * On arm32 we're limited by vmalloc space, so we do not map pages by
	 * default.
	 */
	if (IS_ENABLED(CONFIG_ARM64))
		err = nvgpu_dma_alloc(g, len, &entry->mem);
	else
		err = nvgpu_dma_alloc_flags(g, NVGPU_DMA_NO_KERNEL_MAPPING,
				len, &entry->mem);


	if (err) {
		nvgpu_err(g, "memory allocation failed");
		return -ENOMEM;
	}

	return 0;
}

/*
 * Allocate a phys contig region big enough for a full
 * sized gmmu page table for the given gmmu_page_size.
 * the whole range is zeroed so it's "invalid"/will fault.
 *
 * If a previous entry is supplied, its memory will be used for
 * suballocation for this next entry too, if there is space.
 */
int nvgpu_zalloc_gmmu_page_table(struct vm_gk20a *vm,
				 enum gmmu_pgsz_gk20a pgsz_idx,
				 const struct gk20a_mmu_level *l,
				 struct gk20a_mm_entry *entry,
				 struct gk20a_mm_entry *prev_entry)
{
	int err = -ENOMEM;
	int order;
	struct gk20a *g = gk20a_from_vm(vm);
	u32 bytes;

	/* allocate enough pages for the table */
	order = l->hi_bit[pgsz_idx] - l->lo_bit[pgsz_idx] + 1;
	order += ilog2(l->entry_size);
	bytes = 1 << order;
	order -= PAGE_SHIFT;
	if (order < 0 && prev_entry) {
		/* try to suballocate from previous chunk */
		u32 capacity = prev_entry->mem.size / bytes;
		u32 prev = prev_entry->woffset * sizeof(u32) / bytes;
		u32 free = capacity - prev - 1;

		nvgpu_log(g, gpu_dbg_pte, "cap %d prev %d free %d bytes %d",
				capacity, prev, free, bytes);

		if (free) {
			memcpy(&entry->mem, &prev_entry->mem,
					sizeof(entry->mem));
			entry->woffset = prev_entry->woffset
				+ bytes / sizeof(u32);
			err = 0;
		}
	}

	if (err) {
		/* no suballoc space */
		order = max(0, order);
		err = nvgpu_alloc_gmmu_pages(vm, order, entry);
		entry->woffset = 0;
	}

	nvgpu_log(g, gpu_dbg_pte, "entry = 0x%p, addr=%08llx, size %d, woff %x",
		  entry,
		  (entry->mem.priv.sgt &&
		   entry->mem.aperture == APERTURE_SYSMEM) ?
		  g->ops.mm.get_iova_addr(g, entry->mem.priv.sgt->sgl, 0) : 0,
		  order, entry->woffset);
	if (err)
		return err;
	entry->pgsz = pgsz_idx;
	entry->mem.skip_wmb = true;

	return err;
}

/*
 * Core GMMU map function for the kernel to use. If @addr is 0 then the GPU
 * VA will be allocated for you. If addr is non-zero then the buffer will be
 * mapped at @addr.
 */
static u64 __nvgpu_gmmu_map(struct vm_gk20a *vm,
			    struct nvgpu_mem *mem,
			    u64 addr,
			    u64 size,
			    u32 flags,
			    int rw_flag,
			    bool priv,
			    enum nvgpu_aperture aperture)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u64 vaddr;

	struct sg_table *sgt = mem->priv.sgt;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	vaddr = g->ops.mm.gmmu_map(vm, addr,
				   sgt,    /* sg table */
				   0,      /* sg offset */
				   size,
				   gmmu_page_size_kernel,
				   0,      /* kind */
				   0,      /* ctag_offset */
				   flags, rw_flag,
				   false,  /* clear_ctags */
				   false,  /* sparse */
				   priv,   /* priv */
				   NULL,   /* mapping_batch handle */
				   aperture);
	nvgpu_mutex_release(&vm->update_gmmu_lock);
	if (!vaddr) {
		nvgpu_err(g, "failed to allocate va space");
		return 0;
	}

	return vaddr;
}

u64 nvgpu_gmmu_map(struct vm_gk20a *vm,
		   struct nvgpu_mem *mem,
		   u64 size,
		   u32 flags,
		   int rw_flag,
		   bool priv,
		   enum nvgpu_aperture aperture)
{
	return __nvgpu_gmmu_map(vm, mem, 0, size, flags, rw_flag, priv,
			aperture);
}

/*
 * Like nvgpu_gmmu_map() except it can work on a fixed address instead.
 */
u64 nvgpu_gmmu_map_fixed(struct vm_gk20a *vm,
			 struct nvgpu_mem *mem,
			 u64 addr,
			 u64 size,
			 u32 flags,
			 int rw_flag,
			 bool priv,
			 enum nvgpu_aperture aperture)
{
	return __nvgpu_gmmu_map(vm, mem, addr, size, flags, rw_flag, priv,
			aperture);
}

void nvgpu_gmmu_unmap(struct vm_gk20a *vm, struct nvgpu_mem *mem, u64 gpu_va)
{
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	g->ops.mm.gmmu_unmap(vm,
			     gpu_va,
			     mem->size,
			     gmmu_page_size_kernel,
			     true, /*va_allocated */
			     gk20a_mem_flag_none,
			     false,
			     NULL);

	nvgpu_mutex_release(&vm->update_gmmu_lock);
}
