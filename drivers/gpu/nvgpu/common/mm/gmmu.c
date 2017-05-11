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
#include <nvgpu/list.h>
#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/enabled.h>
#include <nvgpu/page_allocator.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"

#define gmmu_dbg(g, fmt, args...)			\
	nvgpu_log(g, gpu_dbg_map, fmt, ##args)
#define gmmu_dbg_v(g, fmt, args...)			\
	nvgpu_log(g, gpu_dbg_map_v, fmt, ##args)

static int map_gmmu_phys_pages(struct gk20a_mm_entry *entry)
{
	FLUSH_CPU_DCACHE(entry->mem.cpu_va,
			 sg_phys(entry->mem.priv.sgt->sgl),
			 entry->mem.priv.sgt->sgl->length);
	return 0;
}

static void unmap_gmmu_phys_pages(struct gk20a_mm_entry *entry)
{
	FLUSH_CPU_DCACHE(entry->mem.cpu_va,
			 sg_phys(entry->mem.priv.sgt->sgl),
			 entry->mem.priv.sgt->sgl->length);
}

static int map_gmmu_pages(struct gk20a *g, struct gk20a_mm_entry *entry)
{
	gk20a_dbg_fn("");

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		return map_gmmu_phys_pages(entry);

	if (IS_ENABLED(CONFIG_ARM64)) {
		if (entry->mem.aperture == APERTURE_VIDMEM)
			return 0;

		FLUSH_CPU_DCACHE(entry->mem.cpu_va,
				 sg_phys(entry->mem.priv.sgt->sgl),
				 entry->mem.size);
	} else {
		int err = nvgpu_mem_begin(g, &entry->mem);

		if (err)
			return err;
	}

	return 0;
}

static void unmap_gmmu_pages(struct gk20a *g, struct gk20a_mm_entry *entry)
{
	gk20a_dbg_fn("");

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		unmap_gmmu_phys_pages(entry);
		return;
	}

	if (IS_ENABLED(CONFIG_ARM64)) {
		if (entry->mem.aperture == APERTURE_VIDMEM)
			return;

		FLUSH_CPU_DCACHE(entry->mem.cpu_va,
				 sg_phys(entry->mem.priv.sgt->sgl),
				 entry->mem.size);
	} else {
		nvgpu_mem_end(g, &entry->mem);
	}
}

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

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
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

static void free_gmmu_phys_pages(struct vm_gk20a *vm,
			    struct gk20a_mm_entry *entry)
{
	gk20a_dbg_fn("");

	/* note: mem_desc slightly abused (wrt. nvgpu_free_gmmu_pages) */

	free_pages((unsigned long)entry->mem.cpu_va, get_order(entry->mem.size));
	entry->mem.cpu_va = NULL;

	sg_free_table(entry->mem.priv.sgt);
	nvgpu_kfree(vm->mm->g, entry->mem.priv.sgt);
	entry->mem.priv.sgt = NULL;
	entry->mem.size = 0;
	entry->mem.aperture = APERTURE_INVALID;
}

void nvgpu_free_gmmu_pages(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *entry)
{
	struct gk20a *g = gk20a_from_vm(vm);

	gk20a_dbg_fn("");

	if (!entry->mem.size)
		return;

	if (entry->woffset) /* fake shadow mem */
		return;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		free_gmmu_phys_pages(vm, entry);
		return;
	}

	nvgpu_dma_free(g, &entry->mem);
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

/*
 * Convenience wrapper over __nvgpu_gmmu_map() for non-fixed mappings.
 */
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

static int update_gmmu_level_locked(struct vm_gk20a *vm,
				    struct gk20a_mm_entry *pte,
				    enum gmmu_pgsz_gk20a pgsz_idx,
				    struct scatterlist **sgl,
				    u64 *offset,
				    u64 *iova,
				    u64 gpu_va, u64 gpu_end,
				    u8 kind_v, u64 *ctag,
				    bool cacheable, bool unmapped_pte,
				    int rw_flag,
				    bool sparse,
				    int lvl,
				    bool priv,
				    enum nvgpu_aperture aperture)
{
	struct gk20a *g = gk20a_from_vm(vm);
	const struct gk20a_mmu_level *l = &vm->mmu_levels[lvl];
	const struct gk20a_mmu_level *next_l = &vm->mmu_levels[lvl+1];
	int err = 0;
	u32 pde_i;
	u64 pde_size = 1ULL << (u64)l->lo_bit[pgsz_idx];
	struct gk20a_mm_entry *next_pte = NULL, *prev_pte = NULL;

	gk20a_dbg_fn("");

	pde_i = (gpu_va & ((1ULL << ((u64)l->hi_bit[pgsz_idx]+1)) - 1ULL))
		>> (u64)l->lo_bit[pgsz_idx];

	gk20a_dbg(gpu_dbg_pte, "size_idx=%d, l: %d, [%llx,%llx], iova=%llx",
		  pgsz_idx, lvl, gpu_va, gpu_end-1, *iova);

	while (gpu_va < gpu_end) {
		u64 next = min((gpu_va + pde_size) & ~(pde_size-1), gpu_end);

		/* Allocate next level */
		if (next_l->update_entry) {
			if (!pte->entries) {
				int num_entries =
					1 <<
					 (l->hi_bit[pgsz_idx]
					  - l->lo_bit[pgsz_idx] + 1);
				pte->entries =
					nvgpu_vzalloc(g,
						sizeof(struct gk20a_mm_entry) *
						num_entries);
				if (!pte->entries)
					return -ENOMEM;
				pte->pgsz = pgsz_idx;
				pte->num_entries = num_entries;
			}
			prev_pte = next_pte;
			next_pte = pte->entries + pde_i;

			if (!next_pte->mem.size) {
				err = nvgpu_zalloc_gmmu_page_table(vm,
					pgsz_idx, next_l, next_pte, prev_pte);
				if (err)
					return err;
			}
		}

		err = l->update_entry(vm, pte, pde_i, pgsz_idx,
				sgl, offset, iova,
				kind_v, ctag, cacheable, unmapped_pte,
				rw_flag, sparse, priv, aperture);
		if (err)
			return err;

		if (next_l->update_entry) {
			/* get cpu access to the ptes */
			err = map_gmmu_pages(g, next_pte);
			if (err) {
				nvgpu_err(g,
					   "couldn't map ptes for update as=%d",
					   vm_aspace_id(vm));
				return err;
			}
			err = update_gmmu_level_locked(vm, next_pte,
				pgsz_idx,
				sgl,
				offset,
				iova,
				gpu_va,
				next,
				kind_v, ctag, cacheable, unmapped_pte,
				rw_flag, sparse, lvl+1, priv, aperture);
			unmap_gmmu_pages(g, next_pte);

			if (err)
				return err;
		}

		pde_i++;
		gpu_va = next;
	}

	gk20a_dbg_fn("done");

	return 0;
}

/*
 * This is the true top level GMMU mapping logic. This breaks down the incoming
 * scatter gather table and does actual programming of GPU virtual address to
 * physical* address.
 *
 * The update of each level of the page tables is farmed out to chip specific
 * implementations. But the logic around that is generic to all chips. Every chip
 * has some number of PDE levels and then a PTE level.
 *
 * Each chunk of the incoming SGT is sent to the chip specific implementation
 * of page table update.
 *
 * [*] Note: the "physical" address may actually be an IO virtual address in the
 *     case of SMMU usage.
 */
static int update_gmmu_ptes_locked(struct vm_gk20a *vm,
				   enum gmmu_pgsz_gk20a pgsz_idx,
				   struct sg_table *sgt,
				   u64 buffer_offset,
				   u64 gpu_va, u64 gpu_end,
				   u8 kind_v, u32 ctag_offset,
				   bool cacheable, bool unmapped_pte,
				   int rw_flag,
				   bool sparse,
				   bool priv,
				   enum nvgpu_aperture aperture)
{
	struct gk20a *g = gk20a_from_vm(vm);
	int ctag_granularity = g->ops.fb.compression_page_size(g);
	u64 ctag = (u64)ctag_offset * (u64)ctag_granularity;
	u64 iova = 0;
	u64 space_to_skip = buffer_offset;
	u64 map_size = gpu_end - gpu_va;
	u32 page_size  = vm->gmmu_page_sizes[pgsz_idx];
	int err;
	struct scatterlist *sgl = NULL;
	struct nvgpu_page_alloc *alloc = NULL;
	struct page_alloc_chunk *chunk = NULL;
	u64 length;

	/* note: here we need to map kernel to small, since the
	 * low-level mmu code assumes 0 is small and 1 is big pages */
	if (pgsz_idx == gmmu_page_size_kernel)
		pgsz_idx = gmmu_page_size_small;

	if (space_to_skip & (page_size - 1))
		return -EINVAL;

	err = map_gmmu_pages(g, &vm->pdb);
	if (err) {
		nvgpu_err(g,
			   "couldn't map ptes for update as=%d",
			   vm_aspace_id(vm));
		return err;
	}

	if (aperture == APERTURE_VIDMEM) {
		gmmu_dbg_v(g, "vidmem map size_idx=%d, gpu_va=[%llx,%llx]",
			   pgsz_idx, gpu_va, gpu_end-1);

		if (sgt) {
			alloc = get_vidmem_page_alloc(sgt->sgl);

			nvgpu_list_for_each_entry(chunk, &alloc->alloc_chunks,
						 page_alloc_chunk, list_entry) {
				if (space_to_skip &&
				    space_to_skip > chunk->length) {
					space_to_skip -= chunk->length;
				} else {
					iova = chunk->base + space_to_skip;
					length = chunk->length - space_to_skip;
					length = min(length, map_size);
					space_to_skip = 0;

					err = update_gmmu_level_locked(vm,
						&vm->pdb, pgsz_idx,
						&sgl,
						&space_to_skip,
						&iova,
						gpu_va, gpu_va + length,
						kind_v, &ctag,
						cacheable, unmapped_pte,
						rw_flag, sparse, 0, priv,
						aperture);
					if (err)
						break;

					/* need to set explicit zero here */
					space_to_skip = 0;
					gpu_va += length;
					map_size -= length;

					if (!map_size)
						break;
				}
			}
		} else {
			err = update_gmmu_level_locked(vm, &vm->pdb, pgsz_idx,
					&sgl,
					&space_to_skip,
					&iova,
					gpu_va, gpu_end,
					kind_v, &ctag,
					cacheable, unmapped_pte, rw_flag,
					sparse, 0, priv,
					aperture);
		}
	} else {
		gmmu_dbg_v(g,
			   "pgsz=%-6d, gpu_va: %#-12llx +%#-6llx  phys: %#-12llx "
			   "buffer offset: %-4lld, nents: %d",
			   page_size,
			   gpu_va, gpu_end - gpu_va,
			   sgt ? g->ops.mm.get_iova_addr(g, sgt->sgl, 0) : 0ULL,
			   buffer_offset,
			   sgt ? sgt->nents : 0);

		if (sgt) {
			iova = g->ops.mm.get_iova_addr(vm->mm->g, sgt->sgl, 0);
			if (!vm->mm->bypass_smmu && iova) {
				iova += space_to_skip;
			} else {
				sgl = sgt->sgl;

				gk20a_dbg(gpu_dbg_pte, "chunk address %llx, size %d",
						(u64)sg_phys(sgl),
						sgl->length);

				while (space_to_skip && sgl &&
				      space_to_skip + page_size > sgl->length) {
					space_to_skip -= sgl->length;
					sgl = sg_next(sgl);
					gk20a_dbg(gpu_dbg_pte, "chunk address %llx, size %d",
							(u64)sg_phys(sgl),
							sgl->length);
				}

				iova = sg_phys(sgl) + space_to_skip;
			}
		}

		err = update_gmmu_level_locked(vm, &vm->pdb, pgsz_idx,
				&sgl,
				&space_to_skip,
				&iova,
				gpu_va, gpu_end,
				kind_v, &ctag,
				cacheable, unmapped_pte, rw_flag,
				sparse, 0, priv,
				aperture);
	}

	unmap_gmmu_pages(g, &vm->pdb);

	mb();

	gk20a_dbg_fn("done");

	return err;
}

/**
 * gk20a_locked_gmmu_map - Map a buffer into the GMMU
 *
 * This is for non-vGPU chips. It's part of the HAL at the moment but really
 * should not be. Chip specific stuff is handled at the PTE/PDE programming
 * layer. The rest of the logic is essentially generic for all chips.
 *
 * To call this function you must have locked the VM lock: vm->update_gmmu_lock.
 * However, note: this function is not called directly. It's used through the
 * mm.gmmu_lock() HAL. So before calling the mm.gmmu_lock() HAL make sure you
 * have the update_gmmu_lock aquired.
 */
u64 gk20a_locked_gmmu_map(struct vm_gk20a *vm,
			u64 map_offset,
			struct sg_table *sgt,
			u64 buffer_offset,
			u64 size,
			int pgsz_idx,
			u8 kind_v,
			u32 ctag_offset,
			u32 flags,
			int rw_flag,
			bool clear_ctags,
			bool sparse,
			bool priv,
			struct vm_gk20a_mapping_batch *batch,
			enum nvgpu_aperture aperture)
{
	int err = 0;
	bool allocated = false;
	struct gk20a *g = gk20a_from_vm(vm);
	int ctag_granularity = g->ops.fb.compression_page_size(g);
	u32 ctag_lines = DIV_ROUND_UP_ULL(size, ctag_granularity);

	/* Allocate (or validate when map_offset != 0) the virtual address. */
	if (!map_offset) {
		map_offset = __nvgpu_vm_alloc_va(vm, size,
					  pgsz_idx);
		if (!map_offset) {
			nvgpu_err(g, "failed to allocate va space");
			err = -ENOMEM;
			goto fail_alloc;
		}
		allocated = true;
	}

	gmmu_dbg(g,
		 "gv: 0x%04x_%08x + 0x%-7llx "
		 "[dma: 0x%02x_%08x, pa: 0x%02x_%08x] "
		 "pgsz=%-3dKb as=%-2d ctags=%d start=%d "
		 "kind=0x%x flags=0x%x apt=%s",
		 u64_hi32(map_offset), u64_lo32(map_offset), size,
		 sgt ? u64_hi32((u64)sg_dma_address(sgt->sgl)) : 0,
		 sgt ? u64_lo32((u64)sg_dma_address(sgt->sgl)) : 0,
		 sgt ? u64_hi32((u64)sg_phys(sgt->sgl)) : 0,
		 sgt ? u64_lo32((u64)sg_phys(sgt->sgl)) : 0,
		 vm->gmmu_page_sizes[pgsz_idx] >> 10, vm_aspace_id(vm),
		 ctag_lines, ctag_offset,
		 kind_v, flags, nvgpu_aperture_str(aperture));

	err = update_gmmu_ptes_locked(vm, pgsz_idx,
				      sgt,
				      buffer_offset,
				      map_offset, map_offset + size,
				      kind_v,
				      ctag_offset,
				      flags &
				      NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
				      flags &
				      NVGPU_AS_MAP_BUFFER_FLAGS_UNMAPPED_PTE,
				      rw_flag,
				      sparse,
				      priv,
				      aperture);
	if (err) {
		nvgpu_err(g, "failed to update ptes on map");
		goto fail_validate;
	}

	if (!batch)
		g->ops.fb.tlb_invalidate(g, &vm->pdb.mem);
	else
		batch->need_tlb_invalidate = true;

	return map_offset;
fail_validate:
	if (allocated)
		__nvgpu_vm_free_va(vm, map_offset, pgsz_idx);
fail_alloc:
	nvgpu_err(g, "%s: failed with err=%d", __func__, err);
	return 0;
}

void gk20a_locked_gmmu_unmap(struct vm_gk20a *vm,
			u64 vaddr,
			u64 size,
			int pgsz_idx,
			bool va_allocated,
			int rw_flag,
			bool sparse,
			struct vm_gk20a_mapping_batch *batch)
{
	int err = 0;
	struct gk20a *g = gk20a_from_vm(vm);

	if (va_allocated) {
		err = __nvgpu_vm_free_va(vm, vaddr, pgsz_idx);
		if (err) {
			nvgpu_err(g, "failed to free va");
			return;
		}
	}

	/* unmap here needs to know the page size we assigned at mapping */
	err = update_gmmu_ptes_locked(vm,
				pgsz_idx,
				NULL, /* n/a for unmap */
				0,
				vaddr,
				vaddr + size,
				0, 0, false /* n/a for unmap */,
				false, rw_flag,
				sparse, 0,
				APERTURE_INVALID); /* don't care for unmap */
	if (err)
		nvgpu_err(g, "failed to update gmmu ptes on unmap");

	/* flush l2 so any dirty lines are written out *now*.
	 *  also as we could potentially be switching this buffer
	 * from nonvolatile (l2 cacheable) to volatile (l2 non-cacheable) at
	 * some point in the future we need to invalidate l2.  e.g. switching
	 * from a render buffer unmap (here) to later using the same memory
	 * for gmmu ptes.  note the positioning of this relative to any smmu
	 * unmapping (below). */

	if (!batch) {
		gk20a_mm_l2_flush(g, true);
		g->ops.fb.tlb_invalidate(g, &vm->pdb.mem);
	} else {
		if (!batch->gpu_l2_flushed) {
			gk20a_mm_l2_flush(g, true);
			batch->gpu_l2_flushed = true;
		}
		batch->need_tlb_invalidate = true;
	}
}
