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

#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/moduleparam.h>
#include <linux/shrinker.h>

#include "nvmap_priv.h"

#define NVMAP_TEST_PAGE_POOL_SHRINKER 1
static bool enable_pp = 1;
static int pool_size;

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

bool nvmap_page_pool_release(struct nvmap_page_pool *pool, struct page *page)
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
	int available_pages;
	int pages_to_release = 0;
	struct page **page_array = NULL;

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
		pool->page_array = NULL;
		goto out;
	}

	page_array = vzalloc(sizeof(struct page *) * size);
	if (!page_array)
		goto fail;

	memcpy(page_array, pool->page_array,
		pool->npages * sizeof(struct page *));
	vfree(pool->page_array);
	pool->page_array = page_array;
out:
	pr_debug("page pool resized to %d from %d pages", size,
							  pool->max_pages);
	pool->max_pages = size;
	goto exit;
fail:
	vfree(page_array);
	pr_err("failed");
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
		pool->max_pages = (info.totalram >> 3) * 3;
	else
		pool->max_pages = CONFIG_NVMAP_PAGE_POOL_SIZE;

	if (pool->max_pages <= 0 || pool->max_pages >= info.totalram)
		goto fail;

	pool_size = pool->max_pages;
	pr_info("nvmap page pool size: %d pages\n", pool->max_pages);
	pool->page_array = vzalloc(sizeof(struct page *) * pool->max_pages);
	if (!pool->page_array)
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
			goto done;
		if (!nvmap_page_pool_release_locked(pool, page)) {
			__free_page(page);
			goto done;
		}
		if (PageHighMem(page))
			highmem_pages++;
	}
	si_meminfo(&info);
	pr_info("highmem=%d, pool_size=%d,"
		"totalram=%lu, freeram=%lu, totalhigh=%lu, freehigh=%lu\n",
		highmem_pages, pool->max_pages,
		info.totalram, info.freeram, info.totalhigh, info.freehigh);
done:
	nvmap_page_pool_unlock(pool);
#endif
	return 0;
fail:
	pool->max_pages = 0;
	vfree(pool->page_array);
	return -ENOMEM;
}
