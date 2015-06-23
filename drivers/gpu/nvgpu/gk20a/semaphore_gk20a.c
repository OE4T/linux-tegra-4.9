/*
 * drivers/video/tegra/host/gk20a/semaphore_gk20a.c
 *
 * GK20A Semaphores
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "semaphore_gk20a.h"
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include "gk20a.h"
#include "mm_gk20a.h"

static const int SEMAPHORE_SIZE = 16;

struct gk20a_semaphore_pool *gk20a_semaphore_pool_alloc(struct device *d,
		const char *unique_name, size_t capacity)
{
	struct gk20a_semaphore_pool *p;
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	kref_init(&p->ref);
	INIT_LIST_HEAD(&p->maps);
	mutex_init(&p->maps_mutex);
	p->dev = d;

	/* Alloc one 4k page of semaphore per channel. */
	p->size = roundup(capacity * SEMAPHORE_SIZE, PAGE_SIZE);
	p->cpu_va = dma_alloc_coherent(d, p->size, &p->iova, GFP_KERNEL);
	if (!p->cpu_va)
		goto clean_up;
	if (gk20a_get_sgtable(d, &p->sgt, p->cpu_va, p->iova, p->size))
		goto clean_up;

	/* Sacrifice one semaphore in the name of returning error codes. */
	if (gk20a_allocator_init(&p->alloc, unique_name,
				 SEMAPHORE_SIZE, p->size - SEMAPHORE_SIZE,
				 SEMAPHORE_SIZE))
		goto clean_up;

	gk20a_dbg_info("cpuva=%p iova=%llx phys=%llx", p->cpu_va,
		(u64)sg_dma_address(p->sgt->sgl), (u64)sg_phys(p->sgt->sgl));
	return p;
clean_up:
	if (p->cpu_va)
		dma_free_coherent(d, p->size, p->cpu_va, p->iova);
	if (p->sgt)
		gk20a_free_sgtable(&p->sgt);
	kfree(p);
	return NULL;
}

static void gk20a_semaphore_pool_free(struct kref *ref)
{
	struct gk20a_semaphore_pool *p =
		container_of(ref, struct gk20a_semaphore_pool, ref);
	mutex_lock(&p->maps_mutex);
	WARN_ON(!list_empty(&p->maps));
	mutex_unlock(&p->maps_mutex);
	gk20a_free_sgtable(&p->sgt);
	dma_free_coherent(p->dev, p->size, p->cpu_va, p->iova);
	gk20a_allocator_destroy(&p->alloc);
	kfree(p);
}

static void gk20a_semaphore_pool_get(struct gk20a_semaphore_pool *p)
{
	kref_get(&p->ref);
}

void gk20a_semaphore_pool_put(struct gk20a_semaphore_pool *p)
{
	kref_put(&p->ref, gk20a_semaphore_pool_free);
}

static struct gk20a_semaphore_pool_map *
gk20a_semaphore_pool_find_map_locked(struct gk20a_semaphore_pool *p,
				     struct vm_gk20a *vm)
{
	struct gk20a_semaphore_pool_map *map, *found = NULL;
	list_for_each_entry(map, &p->maps, list) {
		if (map->vm == vm) {
			found = map;
			break;
		}
	}
	return found;
}

int gk20a_semaphore_pool_map(struct gk20a_semaphore_pool *p,
			     struct vm_gk20a *vm,
			     enum gk20a_mem_rw_flag rw_flag)
{
	struct gk20a_semaphore_pool_map *map;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;
	map->vm = vm;
	map->rw_flag = rw_flag;
	map->gpu_va = gk20a_gmmu_map(vm, &p->sgt, p->size,
				     0/*uncached*/, rw_flag,
				     false);
	if (!map->gpu_va) {
		kfree(map);
		return -ENOMEM;
	}
	gk20a_vm_get(vm);

	mutex_lock(&p->maps_mutex);
	WARN_ON(gk20a_semaphore_pool_find_map_locked(p, vm));
	list_add(&map->list, &p->maps);
	mutex_unlock(&p->maps_mutex);
	return 0;
}

void gk20a_semaphore_pool_unmap(struct gk20a_semaphore_pool *p,
		struct vm_gk20a *vm)
{
	struct gk20a_semaphore_pool_map *map;
	WARN_ON(!vm);

	mutex_lock(&p->maps_mutex);
	map = gk20a_semaphore_pool_find_map_locked(p, vm);
	if (map) {
		gk20a_gmmu_unmap(vm, map->gpu_va, p->size, map->rw_flag);
		gk20a_vm_put(vm);
		list_del(&map->list);
		kfree(map);
	}
	mutex_unlock(&p->maps_mutex);
}

u64 gk20a_semaphore_pool_gpu_va(struct gk20a_semaphore_pool *p,
		struct vm_gk20a *vm)
{
	struct gk20a_semaphore_pool_map *map;
	u64 gpu_va = 0;

	mutex_lock(&p->maps_mutex);
	map = gk20a_semaphore_pool_find_map_locked(p, vm);
	if (map)
		gpu_va = map->gpu_va;
	mutex_unlock(&p->maps_mutex);

	return gpu_va;
}

struct gk20a_semaphore *gk20a_semaphore_alloc(struct gk20a_semaphore_pool *pool)
{
	struct gk20a_semaphore *s;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return NULL;

	s->offset = gk20a_balloc(&pool->alloc, SEMAPHORE_SIZE);
	if (!s->offset) {
		gk20a_err(pool->dev, "failed to allocate semaphore");
		kfree(s);
		return NULL;
	}

	gk20a_semaphore_pool_get(pool);
	s->pool = pool;

	kref_init(&s->ref);
	s->value = (volatile u32 *)((uintptr_t)pool->cpu_va + s->offset);
	*s->value = 0; /* Initially acquired. */
	gk20a_dbg_info("created semaphore offset=%d, value_cpu=%p, value=%d",
			s->offset, s->value, *s->value);
	return s;
}

static void gk20a_semaphore_free(struct kref *ref)
{
	struct gk20a_semaphore *s =
		container_of(ref, struct gk20a_semaphore, ref);

	gk20a_bfree(&s->pool->alloc, s->offset);
	gk20a_semaphore_pool_put(s->pool);
	kfree(s);
}

void gk20a_semaphore_put(struct gk20a_semaphore *s)
{
	kref_put(&s->ref, gk20a_semaphore_free);
}

void gk20a_semaphore_get(struct gk20a_semaphore *s)
{
	kref_get(&s->ref);
}
