/*
 * drivers/video/tegra/nvmap/nvmap_mm.c
 *
 * Some MM related functionality specific to nvmap.
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION. All rights reserved.
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

inline static void nvmap_flush_dcache_all(void *dummy)
{
#if defined(CONFIG_DENVER_CPU)
	u64 id_afr0;
	u64 midr;

	asm volatile ("mrs %0, MIDR_EL1" : "=r"(midr));
	/* check if current core is a Denver processor */
	if ((midr & 0xFF8FFFF0) == 0x4e0f0000) {
		asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
		/* check if complete cache flush through msr is supported */
		if (likely((id_afr0 & 0xf00) == 0x100)) {
			asm volatile ("msr s3_0_c15_c13_0, %0" : : "r" (0));
			asm volatile ("dsb sy");
			return;
		}
	}
#endif
	__flush_dcache_all(NULL);
}

void inner_flush_cache_all(void)
{
#if defined(CONFIG_ARM64) && defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS_ON_ONE_CPU)
	nvmap_flush_dcache_all(NULL);
#elif defined(CONFIG_ARM64)
	on_each_cpu(nvmap_flush_dcache_all, NULL, 1);
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

/*
 * FIXME:
 *
 *   __clean_dcache_page() is only available on ARM64 (well, we haven't
 *   implemented it on ARMv7).
 */
#if defined(CONFIG_ARM64)
void nvmap_clean_cache(struct page **pages, int numpages)
{
	int i;

	/* Not technically a flush but that's what nvmap knows about. */
	nvmap_stats_inc(NS_CFLUSH_DONE, numpages << PAGE_SHIFT);
	trace_nvmap_cache_flush(numpages << PAGE_SHIFT,
		nvmap_stats_read(NS_ALLOC),
		nvmap_stats_read(NS_CFLUSH_RQ),
		nvmap_stats_read(NS_CFLUSH_DONE));

	for (i = 0; i < numpages; i++)
		__clean_dcache_page(pages[i]);
}
#endif

void nvmap_clean_cache_page(struct page *page)
{
#if defined(CONFIG_ARM64)
	__clean_dcache_page(page);
#else
	__flush_dcache_page(page_mapping(page), page);
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
			__flush_dcache_area(page_address(page), PAGE_SIZE);
#else
		if (flush_inner)
			__flush_dcache_page(page_mapping(page), page);

		base = page_to_phys(page);
		outer_flush_range(base, base + PAGE_SIZE);
#endif
	}
}

/*
 * Perform cache op on the list of memory regions within passed handles.
 * A memory region within handle[i] is identified by offsets[i], sizes[i]
 *
 * sizes[i] == 0  is a special case which causes handle wide operation,
 * this is done by replacing offsets[i] = 0, sizes[i] = handles[i]->size.
 * So, the input arrays sizes, offsets  are not guaranteed to be read-only
 *
 * This will optimze the op if it can.
 * In the case that all the handles together are larger than the inner cache
 * maint threshold it is possible to just do an entire inner cache flush.
 */
int nvmap_do_cache_maint_list(struct nvmap_handle **handles, u32 *offsets,
			      u32 *sizes, int op, int nr)
{
	int i;
	u64 total = 0;
	u64 thresh = ~0;

#if defined(CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS)
	thresh = cache_maint_inner_threshold;
#endif

	for (i = 0; i < nr; i++)
		total += sizes[i] ? sizes[i] : handles[i]->size;

	/* Full flush in the case the passed list is bigger than our
	 * threshold. */
	if (total >= thresh) {
		for (i = 0; i < nr; i++) {
			if (handles[i]->userflags &
			    NVMAP_HANDLE_CACHE_SYNC) {
				nvmap_handle_mkclean(handles[i], 0,
						     handles[i]->size);
				nvmap_zap_handle(handles[i], 0,
						 handles[i]->size);
			}
		}

		if (op == NVMAP_CACHE_OP_WB) {
			inner_clean_cache_all();
			outer_clean_all();
		} else {
			inner_flush_cache_all();
			outer_flush_all();
		}
		nvmap_stats_inc(NS_CFLUSH_RQ, total);
		nvmap_stats_inc(NS_CFLUSH_DONE, thresh);
		trace_nvmap_cache_flush(total,
					nvmap_stats_read(NS_ALLOC),
					nvmap_stats_read(NS_CFLUSH_RQ),
					nvmap_stats_read(NS_CFLUSH_DONE));
	} else {
		for (i = 0; i < nr; i++) {
			u32 size = sizes[i] ? sizes[i] : handles[i]->size;
			u32 offset = sizes[i] ? offsets[i] : 0;
			int err = __nvmap_do_cache_maint(handles[i]->owner,
							 handles[i], offset,
							 offset + size,
							 op, false);
			if (err)
				return err;
		}
	}

	return 0;
}

void nvmap_zap_handle(struct nvmap_handle *handle, u32 offset, u32 size)
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

void nvmap_zap_handles(struct nvmap_handle **handles, u32 *offsets,
		       u32 *sizes, u32 nr)
{
	int i;

	for (i = 0; i < nr; i++)
		nvmap_zap_handle(handles[i], offsets[i], sizes[i]);
}

int nvmap_reserve_pages(struct nvmap_handle **handles, u32 *offsets, u32 *sizes,
			u32 nr, u32 op)
{
	int i;

	for (i = 0; i < nr; i++) {
		u32 size = sizes[i] ? sizes[i] : handles[i]->size;
		u32 offset = sizes[i] ? offsets[i] : 0;

		if (op == NVMAP_PAGES_RESERVE)
			nvmap_handle_mkreserved(handles[i], offset, size);
		else
			nvmap_handle_mkunreserved(handles[i],offset, size);
	}

	if (op == NVMAP_PAGES_RESERVE)
		nvmap_zap_handles(handles, offsets, sizes, nr);
	return 0;
}

