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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/atomic.h>

#include <nvgpu/kmem.h>

/*
 * Statically declared because this needs to be shared across all nvgpu driver
 * instances. This makes sure that all kmem caches are _definitely_ uniquely
 * named.
 */
static atomic_t kmem_cache_id;

/*
 * Linux specific version of the nvgpu_kmem_cache struct. This type is
 * completely opaque to the rest of the driver.
 */
struct nvgpu_kmem_cache {
	struct gk20a *g;
	struct kmem_cache *cache;

	/*
	 * Memory to hold the kmem_cache unique name. Only necessary on our
	 * k3.10 kernel when not using the SLUB allocator but it's easier to
	 * just carry this on to newer kernels.
	 */
	char name[128];
};

struct nvgpu_kmem_cache *nvgpu_kmem_cache_create(struct gk20a *g, size_t size)
{
	struct nvgpu_kmem_cache *cache =
		kzalloc(sizeof(struct nvgpu_kmem_cache), GFP_KERNEL);

	if (!cache)
		return NULL;

	cache->g = g;

	snprintf(cache->name, sizeof(cache->name),
		 "nvgpu-cache-0x%p-%d-%d", g, (int)size,
		 atomic_inc_return(&kmem_cache_id));
	cache->cache = kmem_cache_create(cache->name,
					 size, size, 0, NULL);
	if (!cache->cache) {
		kfree(cache);
		return NULL;
	}

	return cache;
}

void nvgpu_kmem_cache_destroy(struct nvgpu_kmem_cache *cache)
{
	kmem_cache_destroy(cache->cache);
	kfree(cache);
}

void *nvgpu_kmem_cache_alloc(struct nvgpu_kmem_cache *cache)
{
	return kmem_cache_alloc(cache->cache, GFP_KERNEL);
}

void nvgpu_kmem_cache_free(struct nvgpu_kmem_cache *cache, void *ptr)
{
	kmem_cache_free(cache->cache, ptr);
}
