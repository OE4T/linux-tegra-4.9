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

#endif
