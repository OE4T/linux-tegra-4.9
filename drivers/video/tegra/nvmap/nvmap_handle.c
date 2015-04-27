/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2009-2015, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <linux/tegra-soc.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

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

static void add_handle_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref)
{
	struct rb_node **p, *parent = NULL;

	nvmap_ref_lock(client);
	p = &client->handle_refs.rb_node;
	while (*p) {
		struct nvmap_handle_ref *node;
		parent = *p;
		node = rb_entry(parent, struct nvmap_handle_ref, node);
		if (ref->handle > node->handle)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&ref->node, parent, p);
	rb_insert_color(&ref->node, &client->handle_refs);
	client->handle_count++;
	if (client->handle_count > nvmap_max_handle_count)
		nvmap_max_handle_count = client->handle_count;
	atomic_inc(&ref->handle->share_count);
	nvmap_ref_unlock(client);
}

struct nvmap_handle_ref *nvmap_create_handle(struct nvmap_client *client,
					     size_t size)
{
	void *err = ERR_PTR(-ENOMEM);
	struct nvmap_handle *h;
	struct nvmap_handle_ref *ref = NULL;

	if (!client)
		return ERR_PTR(-EINVAL);

	if (!size)
		return ERR_PTR(-EINVAL);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return ERR_PTR(-ENOMEM);

	ref = kzalloc(sizeof(*ref), GFP_KERNEL);
	if (!ref)
		goto ref_alloc_fail;

	atomic_set(&h->ref, 1);
	atomic_set(&h->pin, 0);
	h->owner = client;
	BUG_ON(!h->owner);
	h->size = h->orig_size = size;
	h->flags = NVMAP_HANDLE_WRITE_COMBINE;
	h->peer = NVMAP_IVM_INVALID_PEER;
	mutex_init(&h->lock);
	INIT_LIST_HEAD(&h->vmas);
	INIT_LIST_HEAD(&h->lru);

	/*
	 * This takes out 1 ref on the dambuf. This corresponds to the
	 * handle_ref that gets automatically made by nvmap_create_handle().
	 */
	h->dmabuf = __nvmap_make_dmabuf(client, h);
	if (IS_ERR(h->dmabuf)) {
		err = h->dmabuf;
		goto make_dmabuf_fail;
	}

	/*
	 * Pre-attach nvmap to this new dmabuf. This gets unattached during the
	 * dma_buf_release() operation.
	 */
	h->attachment = dma_buf_attach(h->dmabuf, nvmap_dev->dev_user.parent);
	if (IS_ERR(h->attachment)) {
		err = h->attachment;
		goto dma_buf_attach_fail;
	}

	nvmap_handle_add(nvmap_dev, h);

	/*
	 * Major assumption here: the dma_buf object that the handle contains
	 * is created with a ref count of 1.
	 */
	atomic_set(&ref->dupes, 1);
	ref->handle = h;
	atomic_set(&ref->pin, 0);
	add_handle_ref(client, ref);
	trace_nvmap_create_handle(client, client->name, h, size, ref);
	return ref;

dma_buf_attach_fail:
	dma_buf_put(h->dmabuf);
make_dmabuf_fail:
	kfree(ref);
ref_alloc_fail:
	kfree(h);
	return err;
}

struct nvmap_handle_ref *nvmap_duplicate_handle(struct nvmap_client *client,
					struct nvmap_handle *h, bool skip_val)
{
	struct nvmap_handle_ref *ref = NULL;

	BUG_ON(!client);
	/* on success, the reference count for the handle should be
	 * incremented, so the success paths will not call nvmap_handle_put */
	h = nvmap_validate_get(h);

	if (!h) {
		pr_debug("%s duplicate handle failed\n",
			    current->group_leader->comm);
		return ERR_PTR(-EPERM);
	}

	if (!h->alloc) {
		pr_err("%s duplicating unallocated handle\n",
			current->group_leader->comm);
		nvmap_handle_put(h);
		return ERR_PTR(-EINVAL);
	}

	nvmap_ref_lock(client);
	ref = __nvmap_validate_locked(client, h);

	if (ref) {
		/* handle already duplicated in client; just increment
		 * the reference count rather than re-duplicating it */
		atomic_inc(&ref->dupes);
		nvmap_ref_unlock(client);
		return ref;
	}

	nvmap_ref_unlock(client);

	ref = kzalloc(sizeof(*ref), GFP_KERNEL);
	if (!ref) {
		nvmap_handle_put(h);
		return ERR_PTR(-ENOMEM);
	}

	atomic_set(&ref->dupes, 1);
	ref->handle = h;
	atomic_set(&ref->pin, 0);
	add_handle_ref(client, ref);

	/*
	 * Ref counting on the dma_bufs follows the creation and destruction of
	 * nvmap_handle_refs. That is every time a handle_ref is made the
	 * dma_buf ref count goes up and everytime a handle_ref is destroyed
	 * the dma_buf ref count goes down.
	 */
	get_dma_buf(h->dmabuf);

	trace_nvmap_duplicate_handle(client, h, ref);
	return ref;
}

struct nvmap_handle_ref *nvmap_create_handle_from_fd(
			struct nvmap_client *client, int fd)
{
	struct nvmap_handle *handle;
	struct nvmap_handle_ref *ref;

	BUG_ON(!client);

	handle = nvmap_handle_get_from_dmabuf_fd(client, fd);
	if (IS_ERR(handle))
		return ERR_CAST(handle);
	ref = nvmap_duplicate_handle(client, handle, 1);
	nvmap_handle_put(handle);
	return ref;
}

int __nvmap_get_handle_param(struct nvmap_client *client,
			     struct nvmap_handle *h, u32 param, u64 *result)
{
	int err = 0;

	if (WARN_ON(!virt_addr_valid(h)))
		return -EINVAL;

	switch (param) {
	case NVMAP_HANDLE_PARAM_SIZE:
		*result = h->orig_size;
		break;
	case NVMAP_HANDLE_PARAM_ALIGNMENT:
		*result = h->align;
		break;
	case NVMAP_HANDLE_PARAM_BASE:
		if (!h->alloc || !atomic_read(&h->pin))
			*result = -EINVAL;
		else if (!h->heap_pgalloc) {
			mutex_lock(&h->lock);
			*result = h->carveout->base;
			mutex_unlock(&h->lock);
		} else if (h->attachment->priv)
			*result = sg_dma_address(
				((struct sg_table *)h->attachment->priv)->sgl);
		else
			*result = -EINVAL;
		break;
	case NVMAP_HANDLE_PARAM_HEAP:
		if (!h->alloc)
			*result = 0;
		else if (!h->heap_pgalloc) {
			mutex_lock(&h->lock);
			*result = nvmap_carveout_usage(client, h->carveout);
			mutex_unlock(&h->lock);
		} else
			*result = NVMAP_HEAP_IOVMM;
		break;
	case NVMAP_HANDLE_PARAM_KIND:
		*result = h->kind;
		break;
	case NVMAP_HANDLE_PARAM_COMPR:
		/* ignored, to be removed */
		break;
	default:
		err = -EINVAL;
		break;
	}
	return err;
}

int nvmap_get_handle_param(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref, u32 param, u64 *result)
{
	if (WARN_ON(!virt_addr_valid(ref)) ||
	    WARN_ON(!virt_addr_valid(client)) ||
	    WARN_ON(!result))
		return -EINVAL;

	return __nvmap_get_handle_param(client, ref->handle, param, result);
}
