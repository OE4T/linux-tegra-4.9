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

#ifndef NVGPU_KMEM_H
#define NVGPU_KMEM_H

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <asm/page.h>

struct gk20a;

/*
 * In Linux there is support for the notion of a kmem_cache. It gives better
 * memory usage characteristics for lots of allocations of the same size. Think
 * structs that get allocated over and over. Normal kmalloc() type routines
 * typically round to the next power-of-2 since that's easy.
 *
 * But if we know the size ahead of time the packing for the allocations can be
 * much better. This is the benefit of a slab allocator. This type hides the
 * underlying kmem_cache (or absense thereof).
 */
struct nvgpu_kmem_cache;

struct nvgpu_kmem_cache *nvgpu_kmem_cache_create(struct gk20a *g, size_t size);
void nvgpu_kmem_cache_destroy(struct nvgpu_kmem_cache *cache);

void *nvgpu_kmem_cache_alloc(struct nvgpu_kmem_cache *cache);
void nvgpu_kmem_cache_free(struct nvgpu_kmem_cache *cache, void *ptr);

static inline void *__nvgpu_big_alloc(size_t size, bool clear)
{
	void *p;

	if (size > PAGE_SIZE) {
		if (clear)
			p = vzalloc(size);
		else
			p = vmalloc(size);
	} else {
		if (clear)
			p = kzalloc(size, GFP_KERNEL);
		else
			p = kmalloc(size, GFP_KERNEL);
	}

	return p;
}

/**
 * nvgpu_big_malloc - Pick virtual or physical alloc based on @size
 *
 * @size - Size of the allocation.
 *
 * On some platforms (i.e Linux) it is possible to allocate memory directly
 * mapped into the kernel's address space (kmalloc) or allocate discontiguous
 * pages which are then mapped into a special kernel address range. Each type
 * of allocation has pros and cons. kmalloc() for instance lets you allocate
 * small buffers more space efficiently but vmalloc() allows you to successfully
 * allocate much larger buffers without worrying about fragmentation as much
 * (but will allocate in multiples of page size).
 *
 * This function aims to provide the right allocation for when buffers are of
 * variable size. In some cases the code doesn't know ahead of time if the
 * buffer is going to be big or small so this does the check for you and
 * provides the right type of memory allocation.
 *
 * Returns a pointer to a virtual address range that the kernel can access or
 * %NULL on failure.
 */
static inline void *nvgpu_big_malloc(size_t size)
{
	return __nvgpu_big_alloc(size, false);
}

/**
 * nvgpu_big_malloc - Pick virtual or physical alloc based on @size
 *
 * @size - Size of the allocation.
 *
 * Zeroed memory version of nvgpu_big_malloc().
 */
static inline void *nvgpu_big_zalloc(size_t size)
{
	return __nvgpu_big_alloc(size, true);
}

/**
 * nvgpu_big_free - Free and alloc from nvgpu_big_zalloc() or
 *                  nvgpu_big_malloc().
 *
 * @p - A pointer allocated by nvgpu_big_zalloc() or nvgpu_big_malloc().
 */
static inline void nvgpu_big_free(void *p)
{
	/*
	 * This will have to be fixed eventually. Allocs that use
	 * nvgpu_big_[mz]alloc() will need to remember the size of the alloc
	 * when freeing.
	 */
	if (virt_addr_valid(p))
		kfree(p);
	else
		vfree(p);
}

#endif
