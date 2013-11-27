/*
 * drivers/video/tegra/nvmap/nvmap_pp.c
 *
 * Manage page pools to speed up page allocation.
 *
 * Copyright (c) 2009-2014, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/moduleparam.h>
#include <linux/shrinker.h>

#include "nvmap_priv.h"

#define NVMAP_TEST_PAGE_POOL_SHRINKER 1
static bool enable_pp = 1;
static int pool_size;

#ifdef CONFIG_NVMAP_PAGE_POOL_DEBUG
static inline void __pp_dbg_var_add(u64 *dbg_var, u32 nr)
{
	*dbg_var += nr;
}
#else
#define __pp_dbg_var_add(dbg_var, nr)
#endif

#define pp_alloc_add(pool, nr) __pp_dbg_var_add(&(pool)->allocs, nr)
#define pp_fill_add(pool, nr)  __pp_dbg_var_add(&(pool)->fills, nr)
#define pp_hit_add(pool, nr)   __pp_dbg_var_add(&(pool)->hits, nr)
#define pp_miss_add(pool, nr)  __pp_dbg_var_add(&(pool)->misses, nr)

static inline void nvmap_page_pool_lock(struct nvmap_page_pool *pool)
{
	mutex_lock(&pool->lock);
}

static inline void nvmap_page_pool_unlock(struct nvmap_page_pool *pool)
{
	mutex_unlock(&pool->lock);
}

/*
 * This removes a page from the page pool.
 */
static struct page *nvmap_page_pool_alloc_locked(struct nvmap_page_pool *pool)
{
	struct page *page;

	if (pp_empty(pool)) {
		pp_miss_add(pool, 1);
		return NULL;
	}

	if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG))
		BUG_ON(pool->count == 0);

	page = pool->page_array[pool->alloc];
	pool->page_array[pool->alloc] = NULL;
	nvmap_pp_alloc_inc(pool);
	pool->count--;

	/* Sanity check. */
	if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
		atomic_dec(&page->_count);
		BUG_ON(atomic_read(&page->_count) != 1);
	}

	pp_alloc_add(pool, 1);
	pp_hit_add(pool, 1);

	return page;
}

/*
 * Alloc a bunch of pages from the page pool. This will alloc as many as it can
 * and return the number of pages allocated. Pages are placed into the passed
 * array in a linear fashion starting from index 0.
 *
 * You must lock the page pool before using this.
 */
int __nvmap_page_pool_alloc_lots_locked(struct nvmap_page_pool *pool,
					struct page **pages, u32 nr)
{
	u32 real_nr;
	u32 ind = 0;

	real_nr = min(nr, pool->count);

	while (real_nr--) {
		if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
			BUG_ON(pp_empty(pool));
			BUG_ON(!pool->page_array[pool->alloc]);
		}
		pages[ind++] = pool->page_array[pool->alloc];
		pool->page_array[pool->alloc] = NULL;
		nvmap_pp_alloc_inc(pool);
		if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
			atomic_dec(&pages[ind - 1]->_count);
			BUG_ON(atomic_read(&pages[ind - 1]->_count) != 1);
		}
	}

	pool->count -= ind;
	pp_alloc_add(pool, ind);
	pp_hit_add(pool, ind);
	pp_miss_add(pool, nr - ind);

	return ind;
}

struct page *nvmap_page_pool_alloc(struct nvmap_page_pool *pool)
{
	struct page *page = NULL;

	if (pool) {
		nvmap_page_pool_lock(pool);
		page = nvmap_page_pool_alloc_locked(pool);
		nvmap_page_pool_unlock(pool);
	}
	return page;
}

/*
 * This adds a page to the pool. Returns true iff the passed page is added.
 * That means if the pool is full this operation will fail.
 */
static bool nvmap_page_pool_fill_locked(struct nvmap_page_pool *pool,
					struct page *page)
{
	if (pp_full(pool))
		return false;

	/* Sanity check. */
	if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
		atomic_inc(&page->_count);
		BUG_ON(atomic_read(&page->_count) != 2);
		BUG_ON(pool->count > pool->length);
		BUG_ON(pool->page_array[pool->fill] != NULL);
	}

	pool->page_array[pool->fill] = page;
	nvmap_pp_fill_inc(pool);
	pool->count++;
	pp_fill_add(pool, 1);

	return true;
}

/*
 * Fill a bunch of pages into the page pool. This will fill as many as it can
 * and return the number of pages filled. Pages are used from the start of the
 * passed page pointer array in a linear fashion.
 *
 * You must lock the page pool before using this.
 */
int __nvmap_page_pool_fill_lots_locked(struct nvmap_page_pool *pool,
				       struct page **pages, u32 nr)
{
	u32 real_nr;
	u32 ind = 0;

	real_nr = min(pool->length - pool->count, nr);
	if (real_nr == 0)
		return 0;

	while (real_nr--) {
		if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
			BUG_ON(pp_full(pool));
			BUG_ON(pool->page_array[pool->fill]);
			atomic_inc(&pages[ind]->_count);
			BUG_ON(atomic_read(&pages[ind]->_count) != 2);
		}
		pool->page_array[pool->fill] = pages[ind++];
		nvmap_pp_fill_inc(pool);
	}

	pool->count += ind;
	pp_fill_add(pool, ind);

	return ind;
}

bool nvmap_page_pool_fill(struct nvmap_page_pool *pool, struct page *page)
{
	bool ret = false;

	if (pool) {
		nvmap_page_pool_lock(pool);
		ret = nvmap_page_pool_fill_locked(pool, page);
		nvmap_page_pool_unlock(pool);
	}

	return ret;
}

static int nvmap_page_pool_get_available_count(struct nvmap_page_pool *pool)
{
	return pool->count;
}

static int nvmap_page_pool_free(struct nvmap_page_pool *pool, int nr_free)
{
	int i = nr_free;
	struct page *page;

	if (!nr_free)
		return nr_free;

	nvmap_page_pool_lock(pool);
	while (i) {
		page = nvmap_page_pool_alloc_locked(pool);
		if (!page)
			break;
		__free_page(page);
		i--;
	}
	nvmap_page_pool_unlock(pool);

	return i;
}

ulong nvmap_page_pool_get_unused_pages(void)
{
	int total = 0;
	struct nvmap_share *share;

	if (!nvmap_dev)
		return 0;

	share = nvmap_get_share_from_dev(nvmap_dev);
	if (!share)
		return 0;

	total = nvmap_page_pool_get_available_count(&share->pool);

	return total;
}

static void nvmap_page_pool_resize(struct nvmap_page_pool *pool, int size)
{
	int ind;
	struct page **page_array = NULL;

	if (size == pool->length)
		return;

	nvmap_page_pool_lock(pool);
	if (size == 0) {
		/* TODO: fix this! */
		vfree(pool->page_array);
		pool->page_array = NULL;
		goto out;
	}

	page_array = vzalloc(sizeof(struct page *) * size);
	if (!page_array)
		goto fail;

	/*
	 * Reuse what pages we can.
	 */
	ind = __nvmap_page_pool_alloc_lots_locked(pool, page_array, size);

	/*
	 * And free anything that might be left over.
	 */
	while (!pp_empty(pool))
		__free_page(nvmap_page_pool_alloc_locked(pool));

	swap(page_array, pool->page_array);
	pool->alloc = 0;
	pool->fill = (ind == size ? 0 : ind);
	pool->count = ind;
	pool->length = size;

	vfree(page_array);

out:
	pr_debug("page pool resized to %d from %d pages\n", size, pool->length);
	pool->length = size;
	goto exit;
fail:
	vfree(page_array);
	pr_err("page pool resize failed\n");
exit:
	nvmap_page_pool_unlock(pool);
}

static int nvmap_page_pool_shrink(struct shrinker *shrinker,
				  struct shrink_control *sc)
{
	int shrink_pages = sc->nr_to_scan;
	struct nvmap_share *share = nvmap_get_share_from_dev(nvmap_dev);

	if (!shrink_pages)
		goto out;

	pr_debug("sh_pages=%d", shrink_pages);

	shrink_pages = nvmap_page_pool_free(&share->pool, shrink_pages);
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

static int pool_size_set(const char *arg, const struct kernel_param *kp)
{
	struct nvmap_share *share = nvmap_get_share_from_dev(nvmap_dev);
	param_set_int(arg, kp);
	nvmap_page_pool_resize(&share->pool, pool_size);
	return 0;
}

static int pool_size_get(char *buff, const struct kernel_param *kp)
{
	return param_get_int(buff, kp);
}

static struct kernel_param_ops pool_size_ops = {
	.get = pool_size_get,
	.set = pool_size_set,
};

module_param_cb(pool_size, &pool_size_ops, &pool_size, 0644);

int nvmap_page_pool_init(struct nvmap_device *dev)
{
	static int reg = 1;
	struct sysinfo info;
	struct nvmap_page_pool *pool = &dev->iovmm_master.pool;
#ifdef CONFIG_NVMAP_PAGE_POOLS_INIT_FILLUP
	int i;
	struct page *page;
	int pages_to_fill;
	int highmem_pages = 0;
#endif

	memset(pool, 0x0, sizeof(*pool));
	mutex_init(&pool->lock);

	si_meminfo(&info);
	if (!CONFIG_NVMAP_PAGE_POOL_SIZE)
		/* Use 3/8th of total ram for page pools.
		 * 1/8th for uc, 1/8th for wc and 1/8th for iwb.
		 */
		pool->length = (info.totalram >> 3) * 3;
	else
		pool->length = CONFIG_NVMAP_PAGE_POOL_SIZE;

	if (pool->length <= 0 || pool->length >= info.totalram)
		goto fail;

	pr_info("nvmap page pool size: %d pages\n", pool->length);
	pool->page_array = vzalloc(sizeof(struct page *) * pool->length);
	if (!pool->page_array)
		goto fail;

	if (reg) {
		reg = 0;
		register_shrinker(&nvmap_page_pool_shrinker);
	}

#ifdef CONFIG_NVMAP_PAGE_POOLS_INIT_FILLUP
	pages_to_fill = CONFIG_NVMAP_PAGE_POOLS_INIT_FILLUP_SIZE * SZ_1M /
			PAGE_SIZE;
	pages_to_fill = pages_to_fill ? : pool->length;

	nvmap_page_pool_lock(pool);
	for (i = 0; i < pages_to_fill; i++) {
		page = alloc_page(GFP_NVMAP);
		if (!page)
			goto done;
		if (!nvmap_page_pool_fill_locked(pool, page)) {
			__free_page(page);
			goto done;
		}
		if (PageHighMem(page))
			highmem_pages++;
	}
	si_meminfo(&info);
	pr_info("highmem=%d, pool_size=%d,"
		"totalram=%lu, freeram=%lu, totalhigh=%lu, freehigh=%lu\n",
		highmem_pages, pool->length,
		info.totalram, info.freeram, info.totalhigh, info.freehigh);
done:
	nvmap_page_pool_unlock(pool);
#endif
	return 0;
fail:
	pool->length = 0;
	vfree(pool->page_array);
	return -ENOMEM;
}
