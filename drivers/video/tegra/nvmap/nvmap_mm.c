/*
 * drivers/video/tegra/nvmap/nvmap_mm.c
 *
 * Some MM related functionality specific to nvmap.
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

void nvmap_zap_handle(struct nvmap_handle *handle, u32 offset, u32 size)
{
	struct list_head *vmas;
	struct nvmap_vma_list *vma_list;
	struct vm_area_struct *vma;

	if (!handle->heap_pgalloc)
		return;

	/* if no dirty page is present, no need to zap */
	if (nvmap_handle_track_dirty(handle) && !atomic_read(&handle->pgalloc.ndirty))
		return;

	if (!size) {
		offset = 0;
		size = handle->size;
	}

	size = PAGE_ALIGN((offset & ~PAGE_MASK) + size);

	mutex_lock(&handle->lock);
	vmas = &handle->vmas;
	list_for_each_entry(vma_list, vmas, list) {
		struct nvmap_vma_priv *priv;
		u32 vm_size = size;

		vma = vma_list->vma;
		priv = vma->vm_private_data;
		if ((offset + size) > (vma->vm_end - vma->vm_start))
			vm_size = vma->vm_end - vma->vm_start - offset;
		if (priv->offs || vma->vm_pgoff)
			/* vma mapping starts in the middle of handle memory.
			 * zapping needs special care. zap entire range for now.
			 * FIXME: optimze zapping.
			 */
			zap_page_range(vma, vma->vm_start,
				vma->vm_end - vma->vm_start, NULL);
		else
			zap_page_range(vma, vma->vm_start + offset,
				vm_size, NULL);
	}
	mutex_unlock(&handle->lock);
}

static void nvmap_zap_handles(struct nvmap_handle **handles, u32 *offsets,
		       u32 *sizes, u32 nr)
{
	int i;

	for (i = 0; i < nr; i++)
		nvmap_zap_handle(handles[i], offsets[i], sizes[i]);
}

static
void nvmap_vm_insert_handle(struct nvmap_handle *handle, u32 offset, u32 size)
{
	struct list_head *vmas;
	struct nvmap_vma_list *vma_list;
	struct vm_area_struct *vma;

	if (!handle->heap_pgalloc)
		return;

	if (!size) {
		offset = 0;
		size = handle->size;
	}

	mutex_lock(&handle->lock);
	vmas = &handle->vmas;
	list_for_each_entry(vma_list, vmas, list) {
		struct nvmap_vma_priv *priv;
		u32 vm_size = size;
		int end;
		int i;

		vma = vma_list->vma;
		down_write(&vma->vm_mm->mmap_sem);
		priv = vma->vm_private_data;
		if ((offset + size) > (vma->vm_end - vma->vm_start))
			vm_size = vma->vm_end - vma->vm_start - offset;

		end = PAGE_ALIGN(offset + vm_size) >> PAGE_SHIFT;
		offset >>= PAGE_SHIFT;
		for (i = offset; i < end; i++) {
			struct page *page;
			pte_t *pte;
			spinlock_t *ptl;

			page = nvmap_to_page(handle->pgalloc.pages[i]);
			pte = get_locked_pte(vma->vm_mm,
					vma->vm_start + (i << PAGE_SHIFT),
					&ptl);
			if (!pte) {
				pr_err("nvmap: %s get_locked_pte failed\n",
					__func__);
				up_write(&vma->vm_mm->mmap_sem);
				mutex_unlock(&handle->lock);
				return;
			}
			/*
			 * page->_map_count gets incremented while mapping here.
			 * If _count is not incremented, zap code will see that
			 * page as a bad page and throws lot of warnings.
			 */
			atomic_inc(&page->_count);
			do_set_pte(vma, vma->vm_start + (i << PAGE_SHIFT), page,
					pte, true, false);
			pte_unmap_unlock(pte, ptl);
		}

		up_write(&vma->vm_mm->mmap_sem);
	}
	mutex_unlock(&handle->lock);
}

static void nvmap_vm_insert_handles(struct nvmap_handle **handles, u32 *offsets,
		       u32 *sizes, u32 nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		nvmap_vm_insert_handle(handles[i], offsets[i], sizes[i]);
		nvmap_handle_mkdirty(handles[i], offsets[i],
				     sizes[i] ? sizes[i] : handles[i]->size);
	}
}

int nvmap_reserve_pages(struct nvmap_handle **handles, u32 *offsets, u32 *sizes,
			u32 nr, u32 op)
{
	int i;

	for (i = (op == NVMAP_PAGES_ZAP_AND_CLEAN) ? nr : 0; i < nr; i++) {
		u32 size = sizes[i] ? sizes[i] : handles[i]->size;
		u32 offset = sizes[i] ? offsets[i] : 0;

		if ((op == NVMAP_PAGES_RESERVE) || (op == NVMAP_PAGES_UNRESERVE))
			if ((offset != 0) || (size != handles[i]->size))
				return -EINVAL;

		/*
		 * NOTE: This unreserves the handle even when
		 * NVMAP_PAGES_INSERT_ON_UNRESERVE is called on some portion
		 * of the handle
		 */
		atomic_set(&handles[i]->pgalloc.reserved,
				(op == NVMAP_PAGES_RESERVE) ? 1 : 0);
	}

	if (op == NVMAP_PAGES_ZAP_AND_CLEAN)
		op = NVMAP_PAGES_RESERVE;

	if (op == NVMAP_PAGES_RESERVE)
		nvmap_zap_handles(handles, offsets, sizes, nr);
	else if (op == NVMAP_INSERT_PAGES_ON_UNRESERVE)
		nvmap_vm_insert_handles(handles, offsets, sizes, nr);

	if (!(handles[0]->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE))
		return 0;

	if (op == NVMAP_PAGES_RESERVE) {
		nvmap_do_cache_maint_list(handles, offsets, sizes,
					  NVMAP_CACHE_OP_WB, nr);
		for (i = 0; i < nr; i++)
			nvmap_handle_mkclean(handles[i], offsets[i],
					     sizes[i] ? sizes[i] : handles[i]->size);
	} else if ((op == NVMAP_PAGES_UNRESERVE) && handles[0]->heap_pgalloc) {
		/* Do nothing */
	} else {
		nvmap_do_cache_maint_list(handles, offsets, sizes,
					  NVMAP_CACHE_OP_WB_INV, nr);
	}
	return 0;
}

