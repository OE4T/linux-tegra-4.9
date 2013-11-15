/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2009-2013, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/shrinker.h>
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <linux/tegra-soc.h>

#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <asm/tlbflush.h>
#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"
#include "nvmap_ioctl.h"

u32 nvmap_max_handle_count;

#define NVMAP_SECURE_HEAPS	(NVMAP_HEAP_CARVEOUT_IRAM | NVMAP_HEAP_IOVMM | \
				 NVMAP_HEAP_CARVEOUT_VPR)
#ifdef CONFIG_NVMAP_HIGHMEM_ONLY
#define GFP_NVMAP		(__GFP_HIGHMEM | __GFP_NOWARN)
#else
#define GFP_NVMAP		(GFP_KERNEL | __GFP_HIGHMEM | __GFP_NOWARN)
#endif
/* handles may be arbitrarily large (16+MiB), and any handle allocated from
 * the kernel (i.e., not a carveout handle) includes its array of pages. to
 * preserve kmalloc space, if the array of pages exceeds PAGELIST_VMALLOC_MIN,
 * the array is allocated using vmalloc. */
#define PAGELIST_VMALLOC_MIN	(PAGE_SIZE)

#ifdef CONFIG_NVMAP_PAGE_POOLS

#define NVMAP_TEST_PAGE_POOL_SHRINKER 1
static bool enable_pp = 1;
static int pool_size[NVMAP_NUM_POOLS];

static char *s_memtype_str[] = {
	"uc",
	"wc",
	"iwb",
	"wb",
};

static inline void nvmap_page_pool_lock(struct nvmap_page_pool *pool)
{
	mutex_lock(&pool->lock);
}

static inline void nvmap_page_pool_unlock(struct nvmap_page_pool *pool)
{
	mutex_unlock(&pool->lock);
}

static struct page *nvmap_page_pool_alloc_locked(struct nvmap_page_pool *pool)
{
	struct page *page = NULL;

	if (pool->npages > 0) {
		page = pool->page_array[--pool->npages];
		pool->page_array[pool->npages] = NULL;
		atomic_dec(&page->_count);
		BUG_ON(atomic_read(&page->_count) != 1);
	}
	return page;
}

static struct page *nvmap_page_pool_alloc(struct nvmap_page_pool *pool)
{
	struct page *page = NULL;

	if (pool) {
		nvmap_page_pool_lock(pool);
		page = nvmap_page_pool_alloc_locked(pool);
		nvmap_page_pool_unlock(pool);
	}
	return page;
}

static bool nvmap_page_pool_release_locked(struct nvmap_page_pool *pool,
					    struct page *page)
{
	int ret = false;

	if (enable_pp && pool->npages < pool->max_pages) {
		atomic_inc(&page->_count);
		BUG_ON(atomic_read(&page->_count) != 2);
		BUG_ON(pool->page_array[pool->npages] != NULL);
		pool->page_array[pool->npages++] = page;
		ret = true;
	}
	return ret;
}

static bool nvmap_page_pool_release(struct nvmap_page_pool *pool,
					  struct page *page)
{
	int ret = false;

	if (pool) {
		nvmap_page_pool_lock(pool);
		ret = nvmap_page_pool_release_locked(pool, page);
		nvmap_page_pool_unlock(pool);
	}
	return ret;
}

static int nvmap_page_pool_get_available_count(struct nvmap_page_pool *pool)
{
	return pool->npages;
}

static int nvmap_page_pool_free(struct nvmap_page_pool *pool, int nr_free)
{
	int err;
	int i = nr_free;
	int idx = 0;
	struct page *page;

	if (!nr_free)
		return nr_free;
	nvmap_page_pool_lock(pool);
	while (i) {
		page = nvmap_page_pool_alloc_locked(pool);
		if (!page)
			break;
		pool->shrink_array[idx++] = page;
		i--;
	}

	if (idx) {
		/* This op should never fail. */
		err = nvmap_set_pages_array_wb(pool->shrink_array, idx);
		BUG_ON(err);
	}

	while (idx--)
		__free_page(pool->shrink_array[idx]);
	nvmap_page_pool_unlock(pool);
	return i;
}

ulong nvmap_page_pool_get_unused_pages(void)
{
	unsigned int i;
	int total = 0;
	struct nvmap_share *share;

	if (!nvmap_dev)
		return 0;

	share = nvmap_get_share_from_dev(nvmap_dev);
	if (!share)
		return 0;

	for (i = 0; i < NVMAP_NUM_POOLS; i++)
		total += nvmap_page_pool_get_available_count(&share->pools[i]);

	return total;
}

static void nvmap_page_pool_resize(struct nvmap_page_pool *pool, int size)
{
	int available_pages;
	int pages_to_release = 0;
	struct page **page_array = NULL;
	struct page **shrink_array = NULL;

	if (size == pool->max_pages)
		return;
repeat:
	nvmap_page_pool_free(pool, pages_to_release);
	nvmap_page_pool_lock(pool);
	available_pages = nvmap_page_pool_get_available_count(pool);
	if (available_pages > size) {
		nvmap_page_pool_unlock(pool);
		pages_to_release = available_pages - size;
		goto repeat;
	}

	if (size == 0) {
		vfree(pool->page_array);
		vfree(pool->shrink_array);
		pool->page_array = pool->shrink_array = NULL;
		goto out;
	}

	page_array = vzalloc(sizeof(struct page *) * size);
	shrink_array = vzalloc(sizeof(struct page *) * size);
	if (!page_array || !shrink_array)
		goto fail;

	memcpy(page_array, pool->page_array,
		pool->npages * sizeof(struct page *));
	vfree(pool->page_array);
	vfree(pool->shrink_array);
	pool->page_array = page_array;
	pool->shrink_array = shrink_array;
out:
	pr_debug("%s pool resized to %d from %d pages",
		s_memtype_str[pool->flags], size, pool->max_pages);
	pool->max_pages = size;
	goto exit;
fail:
	vfree(page_array);
	vfree(shrink_array);
	pr_err("failed");
exit:
	nvmap_page_pool_unlock(pool);
}

static int nvmap_page_pool_shrink(struct shrinker *shrinker,
				  struct shrink_control *sc)
{
	unsigned int i;
	unsigned int pool_offset;
	struct nvmap_page_pool *pool;
	int shrink_pages = sc->nr_to_scan;
	static atomic_t start_pool = ATOMIC_INIT(-1);
	struct nvmap_share *share = nvmap_get_share_from_dev(nvmap_dev);

	if (!shrink_pages)
		goto out;

	pr_debug("sh_pages=%d", shrink_pages);

	for (i = 0; i < NVMAP_NUM_POOLS && shrink_pages; i++) {
		pool_offset = atomic_add_return(1, &start_pool) %
				NVMAP_NUM_POOLS;
		pool = &share->pools[pool_offset];
		shrink_pages = nvmap_page_pool_free(pool, shrink_pages);
	}
out:
	return nvmap_page_pool_get_unused_pages();
}

static struct shrinker nvmap_page_pool_shrinker = {
	.shrink = nvmap_page_pool_shrink,
	.seeks = 1,
};

static void shrink_page_pools(int *total_pages, int *available_pages)
{
	struct shrink_control sc;

	if (*total_pages == 0) {
		sc.gfp_mask = GFP_KERNEL;
		sc.nr_to_scan = 0;
		*total_pages = nvmap_page_pool_shrink(NULL, &sc);
	}
	sc.nr_to_scan = *total_pages;
	*available_pages = nvmap_page_pool_shrink(NULL, &sc);
}

#if NVMAP_TEST_PAGE_POOL_SHRINKER
static int shrink_pp;
static int shrink_set(const char *arg, const struct kernel_param *kp)
{
	int cpu = smp_processor_id();
	unsigned long long t1, t2;
	int total_pages, available_pages;

	param_set_int(arg, kp);

	if (shrink_pp) {
		total_pages = shrink_pp;
		t1 = cpu_clock(cpu);
		shrink_page_pools(&total_pages, &available_pages);
		t2 = cpu_clock(cpu);
		pr_debug("shrink page pools: time=%lldns, "
			"total_pages_released=%d, free_pages_available=%d",
			t2-t1, total_pages, available_pages);
	}
	return 0;
}

static int shrink_get(char *buff, const struct kernel_param *kp)
{
	return param_get_int(buff, kp);
}

static struct kernel_param_ops shrink_ops = {
	.get = shrink_get,
	.set = shrink_set,
};

module_param_cb(shrink_page_pools, &shrink_ops, &shrink_pp, 0644);
#endif

static int enable_pp_set(const char *arg, const struct kernel_param *kp)
{
	int total_pages, available_pages, ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (!enable_pp) {
		total_pages = 0;
		shrink_page_pools(&total_pages, &available_pages);
		pr_info("disabled page pools and released pages, "
			"total_pages_released=%d, free_pages_available=%d",
			total_pages, available_pages);
	}
	return 0;
}

static int enable_pp_get(char *buff, const struct kernel_param *kp)
{
	return param_get_int(buff, kp);
}

static struct kernel_param_ops enable_pp_ops = {
	.get = enable_pp_get,
	.set = enable_pp_set,
};

module_param_cb(enable_page_pools, &enable_pp_ops, &enable_pp, 0644);

#define POOL_SIZE_SET(m, i) \
static int pool_size_##m##_set(const char *arg, const struct kernel_param *kp) \
{ \
	struct nvmap_share *share = nvmap_get_share_from_dev(nvmap_dev); \
	param_set_int(arg, kp); \
	nvmap_page_pool_resize(&share->pools[i], pool_size[i]); \
	return 0; \
}

#define POOL_SIZE_GET(m) \
static int pool_size_##m##_get(char *buff, const struct kernel_param *kp) \
{ \
	return param_get_int(buff, kp); \
}

#define POOL_SIZE_OPS(m) \
static struct kernel_param_ops pool_size_##m##_ops = { \
	.get = pool_size_##m##_get, \
	.set = pool_size_##m##_set, \
};

#define POOL_SIZE_MOUDLE_PARAM_CB(m, i) \
module_param_cb(m##_pool_size, &pool_size_##m##_ops, &pool_size[i], 0644)

POOL_SIZE_SET(uc, NVMAP_HANDLE_UNCACHEABLE);
POOL_SIZE_GET(uc);
POOL_SIZE_OPS(uc);
POOL_SIZE_MOUDLE_PARAM_CB(uc, NVMAP_HANDLE_UNCACHEABLE);

POOL_SIZE_SET(wc, NVMAP_HANDLE_WRITE_COMBINE);
POOL_SIZE_GET(wc);
POOL_SIZE_OPS(wc);
POOL_SIZE_MOUDLE_PARAM_CB(wc, NVMAP_HANDLE_WRITE_COMBINE);

POOL_SIZE_SET(iwb, NVMAP_HANDLE_INNER_CACHEABLE);
POOL_SIZE_GET(iwb);
POOL_SIZE_OPS(iwb);
POOL_SIZE_MOUDLE_PARAM_CB(iwb, NVMAP_HANDLE_INNER_CACHEABLE);

POOL_SIZE_SET(wb, NVMAP_HANDLE_CACHEABLE);
POOL_SIZE_GET(wb);
POOL_SIZE_OPS(wb);
POOL_SIZE_MOUDLE_PARAM_CB(wb, NVMAP_HANDLE_CACHEABLE);

int nvmap_page_pool_init(struct nvmap_page_pool *pool, int flags)
{
	static int reg = 1;
	struct sysinfo info;
#ifdef CONFIG_NVMAP_PAGE_POOLS_INIT_FILLUP
	int i;
	int err;
	struct page *page;
	int pages_to_fill;
	int highmem_pages = 0;
	typedef int (*set_pages_array) (struct page **pages, int addrinarray);
	set_pages_array s_cpa[] = {
		nvmap_set_pages_array_uc,
		nvmap_set_pages_array_wc,
		nvmap_set_pages_array_iwb,
		nvmap_set_pages_array_wb
	};
#endif

	BUG_ON(flags >= NVMAP_NUM_POOLS);
	memset(pool, 0x0, sizeof(*pool));
	mutex_init(&pool->lock);
	pool->flags = flags;

	/* No default pool for cached memory. */
	if (flags == NVMAP_HANDLE_CACHEABLE)
		return 0;

#if !defined(CONFIG_OUTER_CACHE)
	/* If outer cache is not enabled or don't exist, cacheable and
	 * inner cacheable memory are same. For cacheable memory, there
	 * is no need of page pool as there is no need to flush cache and
	 * change page attributes.
	 */
	if (flags == NVMAP_HANDLE_INNER_CACHEABLE)
		return 0;
#endif

	si_meminfo(&info);
	if (!pool_size[flags] && !CONFIG_NVMAP_PAGE_POOL_SIZE)
		/* Use 3/8th of total ram for page pools.
		 * 1/8th for uc, 1/8th for wc and 1/8th for iwb.
		 */
		pool->max_pages = info.totalram >> 3;
	else
		pool->max_pages = CONFIG_NVMAP_PAGE_POOL_SIZE;

	if (pool->max_pages <= 0 || pool->max_pages >= info.totalram)
		goto fail;
	pool_size[flags] = pool->max_pages;
	pr_info("nvmap %s page pool size=%d pages\n",
		s_memtype_str[flags], pool->max_pages);
	pool->page_array = vzalloc(sizeof(struct page *) * pool->max_pages);
	pool->shrink_array = vzalloc(sizeof(struct page *) * pool->max_pages);
	if (!pool->page_array || !pool->shrink_array)
		goto fail;

	if (reg) {
		reg = 0;
		register_shrinker(&nvmap_page_pool_shrinker);
	}

#ifdef CONFIG_NVMAP_PAGE_POOLS_INIT_FILLUP
	pages_to_fill = CONFIG_NVMAP_PAGE_POOLS_INIT_FILLUP_SIZE * SZ_1M /
			PAGE_SIZE;
	pages_to_fill = pages_to_fill ? : pool->max_pages;

	nvmap_page_pool_lock(pool);
	for (i = 0; i < pages_to_fill; i++) {
		page = alloc_page(GFP_NVMAP);
		if (!page)
			goto do_cpa;
		if (!nvmap_page_pool_release_locked(pool, page)) {
			__free_page(page);
			goto do_cpa;
		}
		if (PageHighMem(page))
			highmem_pages++;
	}
	si_meminfo(&info);
	pr_info("nvmap pool = %s, highmem=%d, pool_size=%d,"
		"totalram=%lu, freeram=%lu, totalhigh=%lu, freehigh=%lu\n",
		s_memtype_str[flags], highmem_pages, pool->max_pages,
		info.totalram, info.freeram, info.totalhigh, info.freehigh);
do_cpa:
	if (pool->npages) {
		err = (*s_cpa[flags])(pool->page_array, pool->npages);
		BUG_ON(err);
	}
	nvmap_page_pool_unlock(pool);
#endif
	return 0;
fail:
	pool->max_pages = 0;
	vfree(pool->shrink_array);
	vfree(pool->page_array);
	return -ENOMEM;
}
#endif

static inline void *altalloc(size_t len)
{
	if (len > PAGELIST_VMALLOC_MIN)
		return vmalloc(len);
	else
		return kmalloc(len, GFP_KERNEL);
}

static inline void altfree(void *ptr, size_t len)
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
	int err;
	struct nvmap_share *share = nvmap_get_share_from_dev(h->dev);
	unsigned int i, nr_page, page_index = 0;
#ifdef CONFIG_NVMAP_PAGE_POOLS
	struct nvmap_page_pool *pool = NULL;
#endif

	if (h->nvhost_priv)
		h->nvhost_priv_delete(h->nvhost_priv);

	if (nvmap_handle_remove(h->dev, h) != 0)
		return;

	if (!h->alloc)
		goto out;

	if (!h->heap_pgalloc) {
		nvmap_heap_free(h->carveout);
		goto out;
	}

	nr_page = DIV_ROUND_UP(h->size, PAGE_SIZE);

	BUG_ON(h->size & ~PAGE_MASK);
	BUG_ON(!h->pgalloc.pages);

#ifdef CONFIG_NVMAP_PAGE_POOLS
	if (h->flags < NVMAP_NUM_POOLS)
		pool = &share->pools[h->flags];

	while (page_index < nr_page) {
		if (!nvmap_page_pool_release(pool,
		    h->pgalloc.pages[page_index]))
			break;
		page_index++;
	}
#endif

	if (page_index == nr_page)
		goto skip_attr_restore;

	/* Restore page attributes. */
	if (h->flags == NVMAP_HANDLE_WRITE_COMBINE ||
	    h->flags == NVMAP_HANDLE_UNCACHEABLE ||
	    h->flags == NVMAP_HANDLE_INNER_CACHEABLE) {
		/* This op should never fail. */
		err = nvmap_set_pages_array_wb(&h->pgalloc.pages[page_index],
				nr_page - page_index);
		BUG_ON(err);
	}

skip_attr_restore:
	for (i = page_index; i < nr_page; i++)
		__free_page(h->pgalloc.pages[i]);

	altfree(h->pgalloc.pages, nr_page * sizeof(struct page *));

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
	int err = 0;
	size_t size = PAGE_ALIGN(h->size);
	unsigned int nr_page = size >> PAGE_SHIFT;
	pgprot_t prot;
	unsigned int i = 0, page_index = 0;
	struct page **pages;
#ifdef CONFIG_NVMAP_PAGE_POOLS
	struct nvmap_page_pool *pool = NULL;
	struct nvmap_share *share = nvmap_get_share_from_dev(h->dev);
	phys_addr_t paddr;
#endif
	gfp_t gfp = GFP_NVMAP;
	unsigned long kaddr;
	pte_t **pte = NULL;

	if (h->userflags & NVMAP_HANDLE_ZEROED_PAGES) {
		gfp |= __GFP_ZERO;
		prot = nvmap_pgprot(h, pgprot_kernel);
		pte = nvmap_alloc_pte(nvmap_dev, (void **)&kaddr);
		if (IS_ERR(pte))
			return -ENOMEM;
	}

	pages = altalloc(nr_page * sizeof(*pages));
	if (!pages)
		return -ENOMEM;

	prot = nvmap_pgprot(h, pgprot_kernel);

	if (contiguous) {
		struct page *page;
		page = nvmap_alloc_pages_exact(gfp, size);
		if (!page)
			goto fail;

		for (i = 0; i < nr_page; i++)
			pages[i] = nth_page(page, i);

	} else {
#ifdef CONFIG_NVMAP_PAGE_POOLS
		if (h->flags < NVMAP_NUM_POOLS)
			pool = &share->pools[h->flags];
		else
			BUG();

		for (i = 0; i < nr_page; i++) {
			/* Get pages from pool, if available. */
			pages[i] = nvmap_page_pool_alloc(pool);
			if (!pages[i])
				break;
			if (h->userflags & NVMAP_HANDLE_ZEROED_PAGES) {
				/*
				 * Just memset low mem pages; they will for
				 * sure have a virtual address. Otherwise, build
				 * a mapping for the page in the kernel.
				 */
				if (!PageHighMem(pages[i])) {
					memset(page_address(pages[i]), 0,
					       PAGE_SIZE);
				} else {
					paddr = page_to_phys(pages[i]);
					set_pte_at(&init_mm, kaddr, *pte,
						   pfn_pte(__phys_to_pfn(paddr),
							   prot));
					nvmap_flush_tlb_kernel_page(kaddr);
					memset((char *)kaddr, 0, PAGE_SIZE);
				}
			}
			page_index++;
		}
#endif
		for (; i < nr_page; i++) {
			pages[i] = nvmap_alloc_pages_exact(gfp,	PAGE_SIZE);
			if (!pages[i])
				goto fail;
		}
	}

	if (nr_page == page_index)
		goto skip_attr_change;

	/* Update the pages mapping in kernel page table. */
	if (h->flags == NVMAP_HANDLE_WRITE_COMBINE)
		err = nvmap_set_pages_array_wc(&pages[page_index],
					nr_page - page_index);
	else if (h->flags == NVMAP_HANDLE_UNCACHEABLE)
		err = nvmap_set_pages_array_uc(&pages[page_index],
					nr_page - page_index);
	else if (h->flags == NVMAP_HANDLE_INNER_CACHEABLE)
		err = nvmap_set_pages_array_iwb(&pages[page_index],
					nr_page - page_index);

	if (err)
		goto fail;

skip_attr_change:
	if (h->userflags & NVMAP_HANDLE_ZEROED_PAGES)
		nvmap_free_pte(nvmap_dev, pte);
	h->size = size;
	h->pgalloc.pages = pages;
	h->pgalloc.contig = contiguous;
	return 0;

fail:
	if (h->userflags & NVMAP_HANDLE_ZEROED_PAGES)
		nvmap_free_pte(nvmap_dev, pte);
	if (i) {
		err = nvmap_set_pages_array_wb(pages, i);
		BUG_ON(err);
	}
	while (i--)
		__free_page(pages[i]);
	altfree(pages, nr_page * sizeof(*pages));
	wmb();
	return -ENOMEM;
}

static void alloc_handle(struct nvmap_client *client,
			 struct nvmap_handle *h, unsigned int type)
{
	unsigned int carveout_mask = NVMAP_HEAP_CARVEOUT_MASK;
	unsigned int iovmm_mask = NVMAP_HEAP_IOVMM;

	BUG_ON(type & (type - 1));

#ifdef CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM
	/* Convert generic carveout requests to iovmm requests. */
	carveout_mask &= ~NVMAP_HEAP_CARVEOUT_GENERIC;
	iovmm_mask |= NVMAP_HEAP_CARVEOUT_GENERIC;
#endif

	if (type & carveout_mask) {
		struct nvmap_heap_block *b;

		b = nvmap_carveout_alloc(client, h, type);
		if (b) {
			h->heap_pgalloc = false;
			/* barrier to ensure all handle alloc data
			 * is visible before alloc is seen by other
			 * processors.
			 */
			mb();
			h->alloc = true;
			nvmap_carveout_commit_add(client,
				nvmap_heap_to_arg(nvmap_block_to_heap(b)),
				h->size);
		}
	} else if (type & iovmm_mask) {
		int ret;
		size_t reserved = PAGE_ALIGN(h->size);

		atomic_add_return(reserved, &client->iovm_commit);
		ret = handle_page_alloc(client, h, false);
		if (ret) {
			atomic_sub(reserved, &client->iovm_commit);
			return;
		}
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
	NVMAP_HEAP_CARVEOUT_MASK,
	NVMAP_HEAP_IOVMM,
	0,
};

static const unsigned int heap_policy_large[] = {
	NVMAP_HEAP_CARVEOUT_VPR,
	NVMAP_HEAP_CARVEOUT_IRAM,
	NVMAP_HEAP_IOVMM,
	NVMAP_HEAP_CARVEOUT_MASK,
	0,
};

int nvmap_alloc_handle_id(struct nvmap_client *client,
			  unsigned long id, unsigned int heap_mask,
			  size_t align,
			  u8 kind,
			  unsigned int flags)
{
	struct nvmap_handle *h = NULL;
	const unsigned int *alloc_policy;
	int nr_page;
	int err = -ENOMEM;

	h = nvmap_get_handle_id(client, id);

	if (!h)
		return -EINVAL;

	if (h->alloc) {
		nvmap_handle_put(h);
		return -EEXIST;
	}

	trace_nvmap_alloc_handle_id(client, id, heap_mask, align, flags);
	h->userflags = flags;
	nr_page = ((h->size + PAGE_SIZE - 1) >> PAGE_SHIFT);
	h->secure = !!(flags & NVMAP_HANDLE_SECURE);
	h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);
	h->align = max_t(size_t, align, L1_CACHE_BYTES);
	h->kind = kind;
	h->map_resources = 0;

#ifndef CONFIG_TEGRA_IOVMM
	/* convert iovmm requests to generic carveout. */
	if (heap_mask & NVMAP_HEAP_IOVMM) {
		heap_mask = (heap_mask & ~NVMAP_HEAP_IOVMM) |
			    NVMAP_HEAP_CARVEOUT_GENERIC;
	}
#endif
	/* secure allocations can only be served from secure heaps */
	if (h->secure)
		heap_mask &= NVMAP_SECURE_HEAPS;

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
	err = (h->alloc) ? 0 : err;
	nvmap_handle_put(h);
	return err;
}

void nvmap_free_handle_id(struct nvmap_client *client, unsigned long id)
{
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *h;
	int pins;

	nvmap_ref_lock(client);

	ref = __nvmap_validate_id_locked(client, id);
	if (!ref) {
		nvmap_ref_unlock(client);
		return;
	}

	trace_nvmap_free_handle_id(client, id);
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

	if (h->alloc && h->heap_pgalloc && !h->pgalloc.contig)
		atomic_sub_return(h->size, &client->iovm_commit);

	if (h->alloc && !h->heap_pgalloc) {
		mutex_lock(&h->lock);
		nvmap_carveout_commit_subtract(client,
			nvmap_heap_to_arg(nvmap_block_to_heap(h->carveout)),
			h->size);
		mutex_unlock(&h->lock);
	}

	nvmap_ref_unlock(client);

	if (pins)
		nvmap_debug(client, "%s freeing pinned handle %p\n",
			    current->group_leader->comm, h);

	while (atomic_read(&ref->pin))
		__nvmap_unpin(ref);

	if (h->owner == client) {
		h->owner = NULL;
		h->owner_ref = NULL;
	}

	dma_buf_put(ref->handle->dmabuf);
	kfree(ref);

out:
	BUG_ON(!atomic_read(&h->ref));
	if (nvmap_find_cache_maint_op(h->dev, h))
		nvmap_cache_maint_ops_flush(h->dev, h);
	nvmap_handle_put(h);
}
EXPORT_SYMBOL(nvmap_free_handle_id);

void nvmap_free_handle_user_id(struct nvmap_client *client,
			       unsigned long user_id)
{
	nvmap_free_handle_id(client, unmarshal_user_id(user_id));
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
	h->owner_ref = ref;
	h->dev = nvmap_dev;
	BUG_ON(!h->owner);
	h->size = h->orig_size = size;
	h->flags = NVMAP_HANDLE_WRITE_COMBINE;
	mutex_init(&h->lock);

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
	h->attachment = dma_buf_attach(h->dmabuf, &nvmap_pdev->dev);
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

struct nvmap_handle_ref *nvmap_duplicate_handle_id(struct nvmap_client *client,
					unsigned long id, bool skip_val)
{
	struct nvmap_handle_ref *ref = NULL;
	struct nvmap_handle *h = NULL;

	BUG_ON(!client);
	/* on success, the reference count for the handle should be
	 * incremented, so the success paths will not call nvmap_handle_put */
	h = nvmap_validate_get(client, id, skip_val);

	if (!h) {
		nvmap_debug(client, "%s duplicate handle failed\n",
			    current->group_leader->comm);
		return ERR_PTR(-EPERM);
	}

	if (!h->alloc) {
		nvmap_err(client, "%s duplicating unallocated handle\n",
			  current->group_leader->comm);
		nvmap_handle_put(h);
		return ERR_PTR(-EINVAL);
	}

	nvmap_ref_lock(client);
	ref = __nvmap_validate_id_locked(client, (unsigned long)h);

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

	if (!h->heap_pgalloc) {
		mutex_lock(&h->lock);
		nvmap_carveout_commit_add(client,
			nvmap_heap_to_arg(nvmap_block_to_heap(h->carveout)),
			h->size);
		mutex_unlock(&h->lock);
	} else if (!h->pgalloc.contig) {
		atomic_add(h->size, &client->iovm_commit);
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

	trace_nvmap_duplicate_handle_id(client, id, ref);
	return ref;
}

struct nvmap_handle_ref *nvmap_create_handle_from_fd(
			struct nvmap_client *client, int fd)
{
	unsigned long id;
	struct nvmap_handle_ref *ref;

	BUG_ON(!client);

	id = nvmap_get_id_from_dmabuf_fd(client, fd);
	if (IS_ERR_VALUE(id))
		return ERR_PTR(id);
	ref = nvmap_duplicate_handle_id(client, id, 1);
	return ref;
}

unsigned long nvmap_duplicate_handle_id_ex(struct nvmap_client *client,
						unsigned long id)
{
	struct nvmap_handle_ref *ref = nvmap_duplicate_handle_id(client, id, 0);

	if (IS_ERR(ref))
		return 0;

	return __nvmap_ref_to_id(ref);
}
EXPORT_SYMBOL(nvmap_duplicate_handle_id_ex);

int nvmap_get_page_list_info(struct nvmap_client *client,
				unsigned long id, u32 *size, u32 *flags,
				u32 *nr_page, bool *contig)
{
	struct nvmap_handle *h;

	BUG_ON(!size || !flags || !nr_page || !contig);
	BUG_ON(!client);

	*size = 0;
	*flags = 0;
	*nr_page = 0;

	h = nvmap_validate_get(client, id, 0);

	if (!h) {
		nvmap_err(client, "%s query invalid handle %p\n",
			  current->group_leader->comm, (void *)id);
		return -EINVAL;
	}

	if (!h->alloc || !h->heap_pgalloc) {
		nvmap_err(client, "%s query unallocated handle %p\n",
			  current->group_leader->comm, (void *)id);
		nvmap_handle_put(h);
		return -EINVAL;
	}

	*flags = h->flags;
	*size = h->orig_size;
	*nr_page = PAGE_ALIGN(h->size) >> PAGE_SHIFT;
	*contig = h->pgalloc.contig;

	nvmap_handle_put(h);
	return 0;
}
EXPORT_SYMBOL(nvmap_get_page_list_info);

int nvmap_acquire_page_list(struct nvmap_client *client,
			unsigned long id, struct page **pages, u32 nr_page)
{
	struct nvmap_handle *h;
	struct nvmap_handle_ref *ref;
	int idx;
	phys_addr_t dummy;

	BUG_ON(!client);

	h = nvmap_validate_get(client, id, 0);

	if (!h) {
		nvmap_err(client, "%s query invalid handle %p\n",
			  current->group_leader->comm, (void *)id);
		return -EINVAL;
	}

	if (!h->alloc || !h->heap_pgalloc) {
		nvmap_err(client, "%s query unallocated handle %p\n",
			  current->group_leader->comm, (void *)id);
		nvmap_handle_put(h);
		return -EINVAL;
	}

	BUG_ON(nr_page != PAGE_ALIGN(h->size) >> PAGE_SHIFT);

	for (idx = 0; idx < nr_page; idx++)
		pages[idx] = h->pgalloc.pages[idx];

	nvmap_ref_lock(client);
	ref = __nvmap_validate_id_locked(client, id);
	if (ref)
		__nvmap_pin(ref, &dummy);
	nvmap_ref_unlock(client);

	return 0;
}
EXPORT_SYMBOL(nvmap_acquire_page_list);

int nvmap_release_page_list(struct nvmap_client *client, unsigned long id)
{
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *h = NULL;

	BUG_ON(!client);

	nvmap_ref_lock(client);

	ref = __nvmap_validate_id_locked(client, id);
	if (ref)
		__nvmap_unpin(ref);

	nvmap_ref_unlock(client);

	if (ref)
		h = ref->handle;
	if (h)
		nvmap_handle_put(h);

	return 0;
}
EXPORT_SYMBOL(nvmap_release_page_list);

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
		} else if (h->pgalloc.contig)
			*result = page_to_phys(h->pgalloc.pages[0]);
		else if (h->attachment->priv)
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
