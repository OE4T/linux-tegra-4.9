/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <soc/tegra/chip-id.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

enum NVMAP_PROT_OP {
	NVMAP_HANDLE_PROT_NONE = 1,
	NVMAP_HANDLE_PROT_RESTORE = 2,
};

static int nvmap_list_prot_none(struct nvmap_vma_list *vma_list,
				struct vm_area_struct *vma,
				struct vm_area_struct *prev,
				size_t vm_size,
				int handle_is_dirty)
{
	int err = 0;

	vma->vm_flags = vma_list->save_vm_flags;
	(void)vma_set_page_prot(vma);

	if (!handle_is_dirty)
		return 0;

	err = mprotect_fixup(vma, &prev, vma->vm_start,
			vma->vm_start + vm_size, VM_NONE);
	if (err)
		return err;

	vma->vm_flags = vma_list->save_vm_flags;
	(void)vma_set_page_prot(vma);

	return err;
}

static int nvmap_list_prot_restore(struct nvmap_vma_list *vma_list,
				struct vm_area_struct *vma,
				struct vm_area_struct *prev,
				size_t vm_size)
{
	int err = 0;

	vma->vm_flags = VM_NONE;
	(void)vma_set_page_prot(vma);

	err = mprotect_fixup(vma, &prev, vma->vm_start,
			vma->vm_start + vm_size,
			vma_list->save_vm_flags);
	return err;
}

static int nvmap_list_prot(struct nvmap_vma_list *vma_list, u64 offset,
					u64 size, int handle_is_dirty, int op)
{
	struct vm_area_struct *vma = vma_list->vma;
	struct nvmap_vma_priv *priv = vma->vm_private_data;
	struct vm_area_struct *prev = vma->vm_prev;
	size_t vm_size;
	int err = 0;

	vm_size = size;

	if ((offset + size) > (vma->vm_end - vma->vm_start))
		vm_size = vma->vm_end - vma->vm_start - offset;

	if ((priv->offs || vma->vm_pgoff) ||
			(size > (vma->vm_end - vma->vm_start)))
		vm_size = vma->vm_end - vma->vm_start;

	if (vma->vm_mm != current->mm)
		down_write(&vma->vm_mm->mmap_sem);

	switch (op) {
		case NVMAP_HANDLE_PROT_NONE:
			err = nvmap_list_prot_none(vma_list, vma,
							prev, vm_size,
							handle_is_dirty);
			break;
		case NVMAP_HANDLE_PROT_RESTORE:
			err = nvmap_list_prot_restore(vma_list, vma,
							prev, vm_size);
			break;
		default:
			BUG();
	};

	if (vma->vm_mm != current->mm)
		up_write(&vma->vm_mm->mmap_sem);

	return err;
}

static int handle_prot(struct nvmap_handle *handle, u64 offset,
							u64 size, int op)
{
	struct nvmap_vma_list *vma_list;
	int err = -EINVAL;

	if (!handle->heap_pgalloc)
		return err;

	if ((offset >= handle->size) || (offset > handle->size - size) ||
	    (size > handle->size)) {
		pr_debug("%s offset: %lld h->size: %zu size: %lld\n", __func__,
				offset, handle->size, size);
		return err;
	}

	if (!size)
		size = handle->size;

	size = PAGE_ALIGN((offset & ~PAGE_MASK) + size);

	mutex_lock(&handle->lock);

	list_for_each_entry(vma_list, &handle->vmas, list) {
		int handle_is_dirty = (nvmap_handle_track_dirty(handle) &&
				atomic_read(&handle->pgalloc.ndirty));

		err = nvmap_list_prot(vma_list, offset, size,
							handle_is_dirty, op);
		if (err)
			break;

		if(op == NVMAP_HANDLE_PROT_RESTORE)
			_nvmap_handle_mkdirty(handle, 0, size);
	}

	mutex_unlock(&handle->lock);

	return err;
}

static int handles_prot(struct nvmap_handle **handles, u64 *offsets,
		       u64 *sizes, int op, int nr)
{
	int i, err = 0;

	down_write(&current->mm->mmap_sem);
	for (i = 0; i < nr; i++) {
		err = handle_prot(handles[i], offsets[i], sizes[i], op);
		if (err) {
			pr_debug("%s nvmap_prot_handle failed [%d]\n",
					__func__, err);
			goto finish;
		}
	}
finish:
	up_write(&current->mm->mmap_sem);
	return err;
}

int NVMAP2_handles_reserve(struct nvmap_handle **handles, u64 *offsets,
						u64 *sizes, int op, int nr)
{
	int i, err;
	int cache_op;

	for (i = 0; i < nr; i++) {
		if ((offsets[i] != 0) || (sizes[i] != handles[i]->size)) {
			pr_debug("%s offset: %lld size: %lld h->size %zu\n",
					__func__, offsets[i], sizes[i],
					handles[i]->size);
			return -EINVAL;
		}

		if (op == NVMAP_PAGES_PROT_AND_CLEAN)
			continue;

		/*
		 * NOTE: This unreserves the handle even when
		 * NVMAP_PAGES_INSERT_ON_UNRESERVE is called on some portion
		 * of the handle
		 */
		atomic_set(&handles[i]->pgalloc.reserved,
				(op == NVMAP_PAGES_RESERVE) ? 1 : 0);
	}

	err = 0;
	switch (op) {
		case NVMAP_PAGES_PROT_AND_CLEAN:
			/* Fall through */
		case NVMAP_PAGES_RESERVE:
			err = handles_prot(handles, offsets, sizes,
					NVMAP_HANDLE_PROT_NONE, nr);
			break;
		case NVMAP_INSERT_PAGES_ON_UNRESERVE:
			err = handles_prot(handles, offsets, sizes,
					NVMAP_HANDLE_PROT_RESTORE, nr);
			break;
		case NVMAP_PAGES_UNRESERVE:
			for (i = 0; i < nr; i++)
				if (nvmap_handle_track_dirty(handles[i]))
					atomic_set(&handles[i]->pgalloc.ndirty, 0);
			break;
		default:
			return -EINVAL;
	}

	if (!(handles[0]->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE))
		return 0;

	if ((op == NVMAP_PAGES_UNRESERVE) && handles[0]->heap_pgalloc)
		return 0;

	if (op == NVMAP_PAGES_RESERVE) {
		cache_op = NVMAP_CACHE_OP_WB;
	} else {
		cache_op = NVMAP_CACHE_OP_WB_INV;
	}

	err = NVMAP2_handles_cache_maint(handles, offsets, sizes, cache_op, nr);
	if (err)
		return err;

	if (op == NVMAP_PAGES_RESERVE) {
		for (i = 0; i < nr; i++)
			nvmap_handle_mkclean(handles[i], offsets[i], sizes[i]);
	}

	return 0;
}
