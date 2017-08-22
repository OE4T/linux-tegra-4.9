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

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>

#include <nvgpu/log.h>
#include <nvgpu/lock.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/page_allocator.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/kind_gk20a.h"
#include "gk20a/platform_gk20a.h"

#include "vm_priv.h"
#include "os_linux.h"

static struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_reverse(
	struct vm_gk20a *vm, struct dma_buf *dmabuf, u32 kind)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_enum_start(0, &node, root);

	while (node) {
		struct nvgpu_mapped_buf *mapped_buffer =
				mapped_buffer_from_rbtree_node(node);

		if (mapped_buffer->dmabuf == dmabuf &&
		    kind == mapped_buffer->kind)
			return mapped_buffer;

		nvgpu_rbtree_enum_next(&node, node);
	}

	return NULL;
}

/*
 * Determine alignment for a passed buffer. Necessary since the buffer may
 * appear big to map with large pages but the SGL may have chunks that are not
 * aligned on a 64/128kB large page boundary.
 */
static u64 nvgpu_get_buffer_alignment(struct gk20a *g, struct scatterlist *sgl,
				      enum nvgpu_aperture aperture)
{
	u64 align = 0, chunk_align = 0;
	u64 buf_addr;

	if (aperture == APERTURE_VIDMEM) {
		struct nvgpu_page_alloc *alloc = get_vidmem_page_alloc(sgl);
		struct page_alloc_chunk *chunk = NULL;

		nvgpu_list_for_each_entry(chunk, &alloc->alloc_chunks,
					page_alloc_chunk, list_entry) {
			chunk_align = 1ULL << __ffs(chunk->base |
						    chunk->length);

			if (align)
				align = min(align, chunk_align);
			else
				align = chunk_align;
		}

		return align;
	}

	buf_addr = (u64)sg_dma_address(sgl);

	if (g->mm.bypass_smmu || buf_addr == DMA_ERROR_CODE || !buf_addr) {
		while (sgl) {
			buf_addr = (u64)sg_phys(sgl);
			chunk_align = 1ULL << __ffs(buf_addr |
						    (u64)sgl->length);

			if (align)
				align = min(align, chunk_align);
			else
				align = chunk_align;
			sgl = sg_next(sgl);
		}

		return align;
	}

	align = 1ULL << __ffs(buf_addr);

	return align;
}

/*
 * vm->update_gmmu_lock must be held. This checks to see if we already have
 * mapped the passed buffer into this VM. If so, just return the existing
 * mapping address.
 */
static u64 __nvgpu_vm_find_mapping(struct vm_gk20a *vm,
				   struct dma_buf *dmabuf,
				   u64 offset_align,
				   u32 flags,
				   int kind,
				   bool user_mapped,
				   int rw_flag)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct nvgpu_mapped_buf *mapped_buffer = NULL;

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET) {
		mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, offset_align);
		if (!mapped_buffer)
			return 0;

		if (mapped_buffer->dmabuf != dmabuf ||
		    mapped_buffer->kind != (u32)kind)
			return 0;
	} else {
		mapped_buffer =
			__nvgpu_vm_find_mapped_buf_reverse(vm, dmabuf, kind);
		if (!mapped_buffer)
			return 0;
	}

	if (mapped_buffer->flags != flags)
		return 0;

	/* mark the buffer as used */
	if (user_mapped) {
		if (mapped_buffer->user_mapped == 0)
			vm->num_user_mapped_buffers++;
		mapped_buffer->user_mapped++;

		/* If the mapping comes from user space, we own
		 * the handle ref. Since we reuse an
		 * existing mapping here, we need to give back those
		 * refs once in order not to leak.
		 */
		if (mapped_buffer->own_mem_ref)
			dma_buf_put(mapped_buffer->dmabuf);
		else
			mapped_buffer->own_mem_ref = true;
	}
	nvgpu_ref_get(&mapped_buffer->ref);

	nvgpu_log(g, gpu_dbg_map,
		  "gv: 0x%04x_%08x + 0x%-7zu "
		  "[dma: 0x%02x_%08x, pa: 0x%02x_%08x] "
		  "pgsz=%-3dKb as=%-2d ctags=%d start=%d "
		  "flags=0x%x apt=%s (reused)",
		  u64_hi32(mapped_buffer->addr), u64_lo32(mapped_buffer->addr),
		  dmabuf->size,
		  u64_hi32((u64)sg_dma_address(mapped_buffer->sgt->sgl)),
		  u64_lo32((u64)sg_dma_address(mapped_buffer->sgt->sgl)),
		  u64_hi32((u64)sg_phys(mapped_buffer->sgt->sgl)),
		  u64_lo32((u64)sg_phys(mapped_buffer->sgt->sgl)),
		  vm->gmmu_page_sizes[mapped_buffer->pgsz_idx] >> 10,
		  vm_aspace_id(vm),
		  mapped_buffer->ctag_lines, mapped_buffer->ctag_offset,
		  mapped_buffer->flags,
		  nvgpu_aperture_str(gk20a_dmabuf_aperture(g, dmabuf)));

	return mapped_buffer->addr;
}

static int setup_bfr_kind_fields(struct buffer_attrs *bfr, s16 compr_kind,
				 s16 incompr_kind, u32 flags)
{
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL) {
		/* were we supplied with a kind in either parameter? */
		if ((compr_kind < 0 || compr_kind >= NV_KIND_ATTR_SIZE) &&
		    (incompr_kind < 0 || incompr_kind >= NV_KIND_ATTR_SIZE))
			return -EINVAL;

		if (compr_kind != NV_KIND_INVALID) {
			bfr->use_kind_v = true;
			bfr->kind_v = (u8)compr_kind;
		}

		if (incompr_kind != NV_KIND_INVALID) {
			bfr->use_uc_kind_v = true;
			bfr->uc_kind_v = (u8)incompr_kind;
		}
	} else {
		if (compr_kind < 0 || compr_kind >= NV_KIND_ATTR_SIZE)
			return -EINVAL;

		bfr->use_kind_v = true;
		bfr->kind_v = (u8)compr_kind;

		/*
		 * Note: setup_buffer_kind_and_compression() will
		 * figure out uc_kind_v or return an error
		 */
	}

	return 0;
}

u64 nvgpu_vm_map(struct vm_gk20a *vm,
		 struct dma_buf *dmabuf,
		 u64 offset_align,
		 u32 flags,
		 s16 compr_kind,
		 s16 incompr_kind,
		 bool user_mapped,
		 int rw_flag,
		 u64 buffer_offset,
		 u64 mapping_size,
		 struct vm_gk20a_mapping_batch *batch)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_comptag_allocator *ctag_allocator = &g->gr.comp_tags;
	struct nvgpu_mapped_buf *mapped_buffer = NULL;
	bool va_allocated = false;
	u64 map_offset = 0;
	int err = 0;
	struct buffer_attrs bfr = {NULL};
	struct gk20a_comptags comptags;
	bool clear_ctags = false;
	struct scatterlist *sgl;
	struct nvgpu_vm_area *vm_area = NULL;
	u32 ctag_offset;
	enum nvgpu_aperture aperture;

	/*
	 * The kind used as part of the key for map caching. HW may
	 * actually be programmed with the fallback kind in case the
	 * key kind is compressible but we're out of comptags.
	 */
	s16 map_key_kind;

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL) {
		if (compr_kind != NV_KIND_INVALID)
			map_key_kind = compr_kind;
		else
			map_key_kind = incompr_kind;
	} else {
		map_key_kind = compr_kind;
	}

	if (user_mapped && vm->userspace_managed &&
	    !(flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)) {
		nvgpu_err(g, "non-fixed-offset mapping not available on "
			  "userspace managed address spaces");
		return -EFAULT;
	}

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	/* check if this buffer is already mapped */
	if (!vm->userspace_managed) {
		map_offset = __nvgpu_vm_find_mapping(
			vm, dmabuf, offset_align,
			flags, map_key_kind,
			user_mapped, rw_flag);
		if (map_offset) {
			nvgpu_mutex_release(&vm->update_gmmu_lock);
			return map_offset;
		}
	}

	/* pin buffer to get phys/iovmm addr */
	bfr.sgt = gk20a_mm_pin(dev, dmabuf);
	if (IS_ERR(bfr.sgt)) {
		/* Falling back to physical is actually possible
		 * here in many cases if we use 4K phys pages in the
		 * gmmu.  However we have some regions which require
		 * contig regions to work properly (either phys-contig
		 * or contig through smmu io_vaspace).  Until we can
		 * track the difference between those two cases we have
		 * to fail the mapping when we run out of SMMU space.
		 */
		nvgpu_warn(g, "oom allocating tracking buffer");
		goto clean_up;
	}

	err = setup_bfr_kind_fields(&bfr, compr_kind, incompr_kind, flags);
	if (err)
		goto clean_up;

	bfr.size = dmabuf->size;
	sgl = bfr.sgt->sgl;

	aperture = gk20a_dmabuf_aperture(g, dmabuf);
	if (aperture == APERTURE_INVALID) {
		err = -EINVAL;
		goto clean_up;
	}

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)
		map_offset = offset_align;

	bfr.align = nvgpu_get_buffer_alignment(g, sgl, aperture);
	if (g->mm.disable_bigpage)
		bfr.pgsz_idx = gmmu_page_size_small;
	else
		bfr.pgsz_idx = __get_pte_size(vm, map_offset,
				      min_t(u64, bfr.size, bfr.align));
	mapping_size = mapping_size ? mapping_size : bfr.size;
	mapping_size = ALIGN(mapping_size, SZ_4K);

	if ((mapping_size > bfr.size) ||
	    (buffer_offset > (bfr.size - mapping_size))) {
		err = -EINVAL;
		goto clean_up;
	}

	/* Check if we should use a fixed offset for mapping this buffer */
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)  {
		err = nvgpu_vm_area_validate_buffer(vm, offset_align, mapping_size,
						    bfr.pgsz_idx, &vm_area);
		if (err)
			goto clean_up;

		map_offset = offset_align;
		va_allocated = false;
	} else
		va_allocated = true;

	err = setup_buffer_kind_and_compression(vm, flags, &bfr, bfr.pgsz_idx);
	if (unlikely(err)) {
		nvgpu_err(g, "failure setting up kind and compression");
		goto clean_up;
	}

	/* bar1 and pmu vm don't need ctag */
	if (!vm->enable_ctag)
		bfr.ctag_lines = 0;

	gk20a_get_comptags(dev, dmabuf, &comptags);

	/* ensure alignment to compression page size if compression enabled */
	if (bfr.ctag_offset)
		mapping_size = ALIGN(mapping_size,
				     g->ops.fb.compression_page_size(g));

	if (bfr.ctag_lines && !comptags.lines) {
		/* allocate compression resources if needed */
		err = gk20a_alloc_comptags(g, dev, dmabuf,
					   ctag_allocator,
					   bfr.ctag_lines);
		if (unlikely(err)) {
			/* TBD: we can partially alloc ctags as well... */
			if (bfr.use_uc_kind_v) {
				/* no comptags, but fallback kind available */
				bfr.kind_v = bfr.uc_kind_v;
			} else {
				nvgpu_err(g, "comptag alloc failed and no fallback kind specified");
				goto clean_up;
			}
		} else {
			gk20a_get_comptags(dev,
					   dmabuf, &comptags);

			if (g->ops.ltc.cbc_ctrl)
				g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_clear,
						    comptags.offset,
						    comptags.offset +
							comptags.allocated_lines - 1);
			else
				clear_ctags = true;
		}
	}

	/* store the comptag info */
	bfr.ctag_offset = comptags.offset;
	bfr.ctag_lines = comptags.lines;
	bfr.ctag_allocated_lines = comptags.allocated_lines;
	bfr.ctag_user_mappable = comptags.user_mappable;

	/*
	 * Calculate comptag index for this mapping. Differs in
	 * case of partial mapping.
	 */
	ctag_offset = comptags.offset;
	if (ctag_offset)
		ctag_offset += buffer_offset >>
			       ilog2(g->ops.fb.compression_page_size(g));

	/* update gmmu ptes */
	map_offset = g->ops.mm.gmmu_map(vm, map_offset,
					bfr.sgt,
					buffer_offset, /* sg offset */
					mapping_size,
					bfr.pgsz_idx,
					bfr.kind_v,
					ctag_offset,
					flags, rw_flag,
					clear_ctags,
					false,
					false,
					batch,
					aperture);
	if (!map_offset)
		goto clean_up;

	mapped_buffer = nvgpu_kzalloc(g, sizeof(*mapped_buffer));
	if (!mapped_buffer) {
		nvgpu_warn(g, "oom allocating tracking buffer");
		goto clean_up;
	}
	mapped_buffer->dmabuf      = dmabuf;
	mapped_buffer->sgt         = bfr.sgt;
	mapped_buffer->addr        = map_offset;
	mapped_buffer->size        = mapping_size;
	mapped_buffer->pgsz_idx    = bfr.pgsz_idx;
	mapped_buffer->ctag_offset = bfr.ctag_offset;
	mapped_buffer->ctag_lines  = bfr.ctag_lines;
	mapped_buffer->ctag_allocated_lines = bfr.ctag_allocated_lines;
	mapped_buffer->vm          = vm;
	mapped_buffer->flags       = flags;
	mapped_buffer->kind        = map_key_kind;
	mapped_buffer->va_allocated = va_allocated;
	mapped_buffer->user_mapped = user_mapped ? 1 : 0;
	mapped_buffer->own_mem_ref = user_mapped;
	nvgpu_init_list_node(&mapped_buffer->buffer_list);
	nvgpu_ref_init(&mapped_buffer->ref);

	err = nvgpu_insert_mapped_buf(vm, mapped_buffer);
	if (err) {
		nvgpu_err(g, "failed to insert into mapped buffer tree");
		goto clean_up;
	}
	if (user_mapped)
		vm->num_user_mapped_buffers++;

	if (vm_area) {
		nvgpu_list_add_tail(&mapped_buffer->buffer_list,
			      &vm_area->buffer_list_head);
		mapped_buffer->vm_area = vm_area;
	}

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return map_offset;

clean_up:
	nvgpu_kfree(g, mapped_buffer);
	if (va_allocated)
		__nvgpu_vm_free_va(vm, map_offset, bfr.pgsz_idx);
	if (!IS_ERR(bfr.sgt))
		gk20a_mm_unpin(dev, dmabuf, bfr.sgt);

	nvgpu_mutex_release(&vm->update_gmmu_lock);
	nvgpu_log_info(g, "err=%d", err);
	return 0;
}

void nvgpu_vm_unmap(struct vm_gk20a *vm, u64 offset)
{
	struct gk20a *g = vm->mm->g;
	struct nvgpu_mapped_buf *mapped_buffer;

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, offset);
	if (!mapped_buffer) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		nvgpu_err(g, "invalid addr to unmap 0x%llx", offset);
		return;
	}

	nvgpu_ref_put(&mapped_buffer->ref, gk20a_vm_unmap_locked_ref);
	nvgpu_mutex_release(&vm->update_gmmu_lock);
}
