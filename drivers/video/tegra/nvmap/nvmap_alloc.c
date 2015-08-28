/*
 * drivers/video/tegra/nvmap/nvmap_alloc.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/moduleparam.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

#ifdef CONFIG_NVMAP_FORCE_ZEROED_USER_PAGES
bool zero_memory = true;
#define ZERO_MEMORY_PERMS 0444
#else
bool zero_memory;
#define ZERO_MEMORY_PERMS 0644
#endif

static int zero_memory_set(const char *arg, const struct kernel_param *kp)
{
#ifdef CONFIG_NVMAP_FORCE_ZEROED_USER_PAGES
	return -EPERM;
#else
	param_set_bool(arg, kp);
#ifdef CONFIG_NVMAP_PAGE_POOLS
	nvmap_page_pool_clear();
#endif
	return 0;
#endif
}

static struct kernel_param_ops zero_memory_ops = {
	.get = param_get_bool,
	.set = zero_memory_set,
};

module_param_cb(zero_memory, &zero_memory_ops, &zero_memory, ZERO_MEMORY_PERMS);

u32 nvmap_max_handle_count;

/* handles may be arbitrarily large (16+MiB), and any handle allocated from
 * the kernel (i.e., not a carveout handle) includes its array of pages. to
 * preserve kmalloc space, if the array of pages exceeds PAGELIST_VMALLOC_MIN,
 * the array is allocated using vmalloc. */
#define PAGELIST_VMALLOC_MIN	(PAGE_SIZE)

void *nvmap_altalloc(size_t len)
{
	if (len > PAGELIST_VMALLOC_MIN)
		return vmalloc(len);
	else
		return kmalloc(len, GFP_KERNEL);
}

void nvmap_altfree(void *ptr, size_t len)
{
	if (!ptr)
		return;

	if (len > PAGELIST_VMALLOC_MIN)
		vfree(ptr);
	else
		kfree(ptr);
}

static struct page *nvmap_alloc_pages_exact(gfp_t gfp, size_t size)
{
	struct page *page, *p, *e;
	unsigned int order;

	size = PAGE_ALIGN(size);
	order = get_order(size);
	page = alloc_pages(gfp, order);

	if (!page)
		return NULL;

	split_page(page, order);
	e = page + (1 << order);
	for (p = page + (size >> PAGE_SHIFT); p < e; p++)
		__free_page(p);

	return page;
}

static int handle_page_alloc(struct nvmap_client *client,
			     struct nvmap_handle *h, bool contiguous)
{
	size_t size = PAGE_ALIGN(h->size);
	unsigned int nr_page = size >> PAGE_SHIFT;
	pgprot_t prot;
	unsigned int i = 0, page_index = 0;
	struct page **pages;
	gfp_t gfp = GFP_NVMAP;

	if (zero_memory)
		gfp |= __GFP_ZERO;

	pages = nvmap_altalloc(nr_page * sizeof(*pages));
	if (!pages)
		return -ENOMEM;

	prot = nvmap_pgprot(h, PG_PROT_KERNEL);

	if (contiguous) {
		struct page *page;
		page = nvmap_alloc_pages_exact(gfp, size);
		if (!page)
			goto fail;

		for (i = 0; i < nr_page; i++)
			pages[i] = nth_page(page, i);

	} else {
#ifdef CONFIG_NVMAP_PAGE_POOLS
		/*
		 * Get as many pages from the pools as possible.
		 */
		page_index = nvmap_page_pool_alloc_lots(&nvmap_dev->pool, pages,
								 nr_page);
#endif
		for (i = page_index; i < nr_page; i++) {
			pages[i] = nvmap_alloc_pages_exact(gfp,	PAGE_SIZE);
			if (!pages[i])
				goto fail;
		}
	}

	/*
	 * Make sure any data in the caches is cleaned out before
	 * passing these pages to userspace. otherwise, It can lead to
	 * corruption in pages that get mapped as something other than WB in
	 * userspace and leaked kernel data structures.
	 *
	 * FIXME: For ARMv7 we don't have __clean_dcache_page() so we continue
	 * to use the flush cache version.
	 */
	if (page_index < nr_page)
#ifdef ARM64
		nvmap_clean_cache(&pages[page_index], nr_page - page_index);
#else
		nvmap_flush_cache(&pages[page_index], nr_page - page_index);
#endif

	h->size = size;
	h->pgalloc.pages = pages;
	h->pgalloc.contig = contiguous;
	atomic_set(&h->pgalloc.ndirty, 0);
	return 0;

fail:
	while (i--)
		__free_page(pages[i]);
	nvmap_altfree(pages, nr_page * sizeof(*pages));
	wmb();
	return -ENOMEM;
}

static void alloc_handle(struct nvmap_client *client,
			 struct nvmap_handle *h, unsigned int type)
{
	unsigned int carveout_mask = NVMAP_HEAP_CARVEOUT_MASK;
	unsigned int iovmm_mask = NVMAP_HEAP_IOVMM;

	BUG_ON(type & (type - 1));

	BUILD_BUG_ON(config_enabled(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM) &&
		     config_enabled(CONFIG_NVMAP_CONVERT_IOVMM_TO_CARVEOUT));

#ifdef CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM
	carveout_mask &= ~NVMAP_HEAP_CARVEOUT_GENERIC;
	iovmm_mask |= NVMAP_HEAP_CARVEOUT_GENERIC;
#elif defined(CONFIG_NVMAP_CONVERT_IOVMM_TO_CARVEOUT)
	if (type & NVMAP_HEAP_IOVMM) {
		type &= ~NVMAP_HEAP_IOVMM;
		type |= NVMAP_HEAP_CARVEOUT_GENERIC;
	}
#endif

	if (type & carveout_mask) {
		struct nvmap_heap_block *b;

		b = nvmap_carveout_alloc(client, h, type, NULL);
		if (b) {
			h->heap_type = type;
			h->heap_pgalloc = false;
			/* barrier to ensure all handle alloc data
			 * is visible before alloc is seen by other
			 * processors.
			 */
			mb();
			h->alloc = true;
		}
	} else if (type & iovmm_mask) {
		int ret;

		ret = handle_page_alloc(client, h,
			h->userflags & NVMAP_HANDLE_PHYS_CONTIG);
		if (ret)
			return;
		h->heap_type = NVMAP_HEAP_IOVMM;
		h->heap_pgalloc = true;
		mb();
		h->alloc = true;
	}
}

/* small allocations will try to allocate from generic OS memory before
 * any of the limited heaps, to increase the effective memory for graphics
 * allocations, and to reduce fragmentation of the graphics heaps with
 * sub-page splinters */
static const unsigned int heap_policy_small[] = {
	NVMAP_HEAP_CARVEOUT_VPR,
	NVMAP_HEAP_CARVEOUT_IRAM,
	NVMAP_HEAP_CARVEOUT_IVM,
	NVMAP_HEAP_CARVEOUT_MASK,
	NVMAP_HEAP_IOVMM,
	0,
};

static const unsigned int heap_policy_large[] = {
	NVMAP_HEAP_CARVEOUT_VPR,
	NVMAP_HEAP_CARVEOUT_IRAM,
	NVMAP_HEAP_IOVMM,
	NVMAP_HEAP_CARVEOUT_IVM,
	NVMAP_HEAP_CARVEOUT_MASK,
	0,
};

int nvmap_alloc_handle(struct nvmap_client *client,
		       struct nvmap_handle *h, unsigned int heap_mask,
		       size_t align,
		       u8 kind,
		       unsigned int flags,
		       int peer)
{
	const unsigned int *alloc_policy;
	int nr_page;
	int err = -ENOMEM;
	int tag;

	h = nvmap_handle_get(h);

	if (!h)
		return -EINVAL;

	if (h->alloc) {
		nvmap_handle_put(h);
		return -EEXIST;
	}

	nvmap_stats_inc(NS_TOTAL, PAGE_ALIGN(h->orig_size));
	nvmap_stats_inc(NS_ALLOC, PAGE_ALIGN(h->size));
	trace_nvmap_alloc_handle(client, h,
		h->size, heap_mask, align, flags,
		nvmap_stats_read(NS_TOTAL),
		nvmap_stats_read(NS_ALLOC));
	h->userflags = flags;
	nr_page = ((h->size + PAGE_SIZE - 1) >> PAGE_SHIFT);
	h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);
	h->align = max_t(size_t, align, L1_CACHE_BYTES);
	h->kind = kind;
	h->peer = peer;
	tag = flags >> 16;

	if (!tag && client && !client->tag_warned) {
		char task_comm[TASK_COMM_LEN];
		client->tag_warned = 1;
		get_task_comm(task_comm, client->task);
		pr_err("PID %d: %s: WARNING: "
			"All NvMap Allocations must have a tag "
			"to identify the subsystem allocating memory."
			"Plase pass the tag to the API call"
			" NvRmMemHanldeAllocAttr() or relevant. \n",
			client->task->pid, task_comm);
	}

	/* convert iovmm requests to generic carveout. */
	if (heap_mask & NVMAP_HEAP_IOVMM) {
		heap_mask = (heap_mask & ~NVMAP_HEAP_IOVMM) |
			    NVMAP_HEAP_CARVEOUT_GENERIC;
	}

	/* If user specifies IVM carveout, allocation from no other heap should
	 * be allowed.
	 */
	if (heap_mask & NVMAP_HEAP_CARVEOUT_IVM)
		if (heap_mask & ~(NVMAP_HEAP_CARVEOUT_IVM)) {
			pr_err("%s alloc mixes IVM and other heaps\n",
			       current->group_leader->comm);
			err = -EINVAL;
			goto out;
		}

	if (!heap_mask) {
		err = -EINVAL;
		goto out;
	}

	alloc_policy = (nr_page == 1) ? heap_policy_small : heap_policy_large;

	while (!h->alloc && *alloc_policy) {
		unsigned int heap_type;

		heap_type = *alloc_policy++;
		heap_type &= heap_mask;

		if (!heap_type)
			continue;

		heap_mask &= ~heap_type;

		while (heap_type && !h->alloc) {
			unsigned int heap;

			/* iterate possible heaps MSB-to-LSB, since higher-
			 * priority carveouts will have higher usage masks */
			heap = 1 << __fls(heap_type);
			alloc_handle(client, h, heap);
			heap_type &= ~heap;
		}
	}

out:
	if (h->alloc) {
		if (client->kernel_client)
			nvmap_stats_inc(NS_KALLOC, h->size);
		else
			nvmap_stats_inc(NS_UALLOC, h->size);
	} else {
		nvmap_stats_dec(NS_TOTAL, PAGE_ALIGN(h->orig_size));
		nvmap_stats_dec(NS_ALLOC, PAGE_ALIGN(h->orig_size));
	}

	err = (h->alloc) ? 0 : err;
	nvmap_handle_put(h);
	return err;
}

void _nvmap_handle_free(struct nvmap_handle *h)
{
	unsigned int i, nr_page, page_index = 0;

	if (h->nvhost_priv)
		h->nvhost_priv_delete(h->nvhost_priv);

	if (nvmap_handle_remove(nvmap_dev, h) != 0)
		return;

	if (!h->alloc)
		goto out;

	nvmap_stats_inc(NS_RELEASE, h->size);
	nvmap_stats_dec(NS_TOTAL, PAGE_ALIGN(h->orig_size));
	if (!h->heap_pgalloc) {
		nvmap_heap_free(h->carveout);
		goto out;
	}

	nr_page = DIV_ROUND_UP(h->size, PAGE_SIZE);

	BUG_ON(h->size & ~PAGE_MASK);
	BUG_ON(!h->pgalloc.pages);

#ifdef NVMAP_LAZY_VFREE
	if (h->vaddr) {
		nvmap_kmaps_dec(h);
		vm_unmap_ram(h->vaddr, h->size >> PAGE_SHIFT);
	}
#endif

	for (i = 0; i < nr_page; i++)
		h->pgalloc.pages[i] = nvmap_to_page(h->pgalloc.pages[i]);

#ifdef CONFIG_NVMAP_PAGE_POOLS
	page_index = nvmap_page_pool_fill_lots(&nvmap_dev->pool,
				h->pgalloc.pages, nr_page);
#endif

	for (i = page_index; i < nr_page; i++)
		__free_page(h->pgalloc.pages[i]);

	nvmap_altfree(h->pgalloc.pages, nr_page * sizeof(struct page *));

out:
	kfree(h);
}

void nvmap_free_handle(struct nvmap_client *client,
		       struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *h;
	int pins;

	nvmap_ref_lock(client);

	ref = __nvmap_validate_locked(client, handle);
	if (!ref) {
		nvmap_ref_unlock(client);
		return;
	}

	trace_nvmap_free_handle(client, handle);
	BUG_ON(!ref->handle);
	h = ref->handle;

	if (atomic_dec_return(&ref->dupes)) {
		nvmap_ref_unlock(client);
		goto out;
	}

	smp_rmb();
	pins = atomic_read(&ref->pin);
	rb_erase(&ref->node, &client->handle_refs);
	client->handle_count--;
	atomic_dec(&ref->handle->share_count);

	nvmap_ref_unlock(client);

	if (pins)
		pr_debug("%s freeing pinned handle %p\n",
			    current->group_leader->comm, h);

	while (atomic_read(&ref->pin))
		__nvmap_unpin(ref);

	if (h->owner == client)
		h->owner = NULL;

	dma_buf_put(ref->handle->dmabuf);
	kfree(ref);

out:
	BUG_ON(!atomic_read(&h->ref));
	nvmap_handle_put(h);
}
EXPORT_SYMBOL(nvmap_free_handle);

void nvmap_free_handle_fd(struct nvmap_client *client,
			       int fd)
{
	struct nvmap_handle *handle = nvmap_handle_get_from_fd(fd);
	if (handle) {
		nvmap_free_handle(client, handle);
		nvmap_handle_put(handle);
	}
}
