/*
 * drivers/video/tegra/nvmap/nvmap_mm.c
 *
 * Some MM related functionality specific to nvmap.
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

void inner_flush_cache_all(void)
{
#if defined(CONFIG_ARM64) && defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS_ON_ONE_CPU)
	__flush_dcache_all(NULL);
#elif defined(CONFIG_ARM64)
	on_each_cpu(__flush_dcache_all, NULL, 1);
#elif defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS_ON_ONE_CPU)
	v7_flush_kern_cache_all();
#else
	on_each_cpu(v7_flush_kern_cache_all, NULL, 1);
#endif
}

void inner_clean_cache_all(void)
{
#if defined(CONFIG_ARM64) && \
	defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS_ON_ONE_CPU)
	__clean_dcache_all(NULL);
#elif defined(CONFIG_ARM64)
	on_each_cpu(__clean_dcache_all, NULL, 1);
#elif defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS_ON_ONE_CPU)
	v7_clean_kern_cache_all(NULL);
#else
	on_each_cpu(v7_clean_kern_cache_all, NULL, 1);
#endif
}

void nvmap_flush_cache(struct page **pages, int numpages)
{
	unsigned int i;
	bool flush_inner = true;
	__attribute__((unused)) unsigned long base;

	nvmap_stats_inc(NS_CFLUSH_RQ, numpages << PAGE_SHIFT);
#if defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS)
	if (numpages >= (cache_maint_inner_threshold >> PAGE_SHIFT)) {
		nvmap_stats_inc(NS_CFLUSH_DONE, cache_maint_inner_threshold);
		inner_flush_cache_all();
		flush_inner = false;
	}
#endif
	if (flush_inner)
		nvmap_stats_inc(NS_CFLUSH_DONE, numpages << PAGE_SHIFT);
	trace_nvmap_cache_flush(numpages << PAGE_SHIFT,
		nvmap_stats_read(NS_ALLOC),
		nvmap_stats_read(NS_CFLUSH_RQ),
		nvmap_stats_read(NS_CFLUSH_DONE));

	for (i = 0; i < numpages; i++) {
		struct page *page = nvmap_to_page(pages[i]);
#ifdef CONFIG_ARM64 //__flush_dcache_page flushes inner and outer on ARM64
		if (flush_inner)
			__flush_dcache_page(page);
#else
		if (flush_inner)
			__flush_dcache_page(page_mapping(page), page);
		base = page_to_phys(page);
		outer_flush_range(base, base + PAGE_SIZE);
#endif
	}
}

/*
 * Perform cache op on the list of passed handles.
 * This will optimze the op if it can.
 * In the case that all the handles together are larger than the inner cache
 * maint threshold it is possible to just do an entire inner cache flush.
 */
int nvmap_do_cache_maint_list(struct nvmap_handle **handles, int op, int nr)
{
	int i, err = 0;
	u64 total = 0;

	for (i = 0; i < nr; i++)
		total += handles[i]->size;

	/* Full flush in the case the passed list is bigger than our
	 * threshold. */
	if (total >= cache_maint_inner_threshold) {
		if (op == NVMAP_CACHE_OP_WB) {
			inner_clean_cache_all();
			outer_clean_all();
		} else {
			inner_flush_cache_all();
			outer_flush_all();
		}
		nvmap_stats_inc(NS_CFLUSH_RQ, total);
		nvmap_stats_inc(NS_CFLUSH_DONE, cache_maint_inner_threshold);
		trace_nvmap_cache_flush(total,
					nvmap_stats_read(NS_ALLOC),
					nvmap_stats_read(NS_CFLUSH_RQ),
					nvmap_stats_read(NS_CFLUSH_DONE));
	} else {
		for (i = 0; i < nr; i++) {
			err = __nvmap_do_cache_maint(handles[i]->owner,
						     handles[i], 0,
						     handles[i]->size,
						     op, false);
			if (err)
				break;
		}
	}

	return err;
}

void nvmap_zap_handle(struct nvmap_handle *handle,
		      u64 offset,
		      u64 size)
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

	vmas = &handle->pgalloc.vmas;
	mutex_lock(&handle->lock);
	list_for_each_entry(vma_list, vmas, list) {
		vma = vma_list->vma;
		zap_page_range(vma, vma->vm_start + offset,
				offset + size - vma->vm_start,
				NULL);
	}
	mutex_unlock(&handle->lock);
}

void nvmap_zap_handles(struct nvmap_handle **handles,
		       u64 *offsets,
		       u64 *sizes,
		       u32 nr)
{
	int i;

	for (i = 0; i < nr; i++)
		nvmap_zap_handle(handles[i], offsets[i], sizes[i]);
}

