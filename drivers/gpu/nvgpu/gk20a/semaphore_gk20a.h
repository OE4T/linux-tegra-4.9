/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef SEMAPHORE_GK20A_H
#define SEMAPHORE_GK20A_H

#include <linux/kref.h>
#include "gk20a_allocator.h"
#include "mm_gk20a.h"

/* A memory pool for holding semaphores. */
struct gk20a_semaphore_pool {
	void *cpu_va;
	dma_addr_t iova;
	size_t size;
	struct device *dev;
	struct sg_table *sgt;
	struct list_head maps;
	struct mutex maps_mutex;
	struct kref ref;
	struct gk20a_allocator alloc;
};

enum gk20a_mem_rw_flag {
	gk20a_mem_flag_none = 0,
	gk20a_mem_flag_read_only = 1,
	gk20a_mem_flag_write_only = 2,
};

/* A semaphore pool can be mapped to multiple GPU address spaces. */
struct gk20a_semaphore_pool_map {
	u64 gpu_va;
	enum gk20a_mem_rw_flag rw_flag;
	struct vm_gk20a *vm;
	struct list_head list;
};

/* A semaphore that lives inside a semaphore pool. */
struct gk20a_semaphore {
	struct gk20a_semaphore_pool *pool;
	u32 offset; /* byte offset within pool */
	struct kref ref;
	/* value is a pointer within the pool's coherent cpu_va.
	 * It is shared between CPU and GPU, hence volatile. */
	volatile u32 *value; /* 0=acquired, 1=released */
};

/* Create a semaphore pool that can hold at most 'capacity' semaphores. */
struct gk20a_semaphore_pool *
gk20a_semaphore_pool_alloc(struct device *, const char *unique_name,
			   size_t capacity);
void gk20a_semaphore_pool_put(struct gk20a_semaphore_pool *);
int gk20a_semaphore_pool_map(struct gk20a_semaphore_pool *,
			     struct vm_gk20a *,
			     enum gk20a_mem_rw_flag);
void gk20a_semaphore_pool_unmap(struct gk20a_semaphore_pool *,
				struct vm_gk20a *);
u64 gk20a_semaphore_pool_gpu_va(struct gk20a_semaphore_pool *,
				struct vm_gk20a *);

/* Allocate a semaphore from the semaphore pool. The newly allocated
 * semaphore will be in acquired state (value=0). */
struct gk20a_semaphore *
gk20a_semaphore_alloc(struct gk20a_semaphore_pool *);
void gk20a_semaphore_put(struct gk20a_semaphore *);
void gk20a_semaphore_get(struct gk20a_semaphore *);

static inline u64 gk20a_semaphore_gpu_va(struct gk20a_semaphore *s,
					 struct vm_gk20a *vm)
{
	return gk20a_semaphore_pool_gpu_va(s->pool, vm) + s->offset;
}

static inline bool gk20a_semaphore_is_acquired(struct gk20a_semaphore *s)
{
	u32 v = *s->value;

	/* When often block on value reaching a certain threshold. We must make
	 * sure that if we get unblocked, we haven't read anything too early. */
	smp_rmb();
	return v == 0;
}

static inline void gk20a_semaphore_release(struct gk20a_semaphore *s)
{
	smp_wmb();
	*s->value = 1;
}
#endif
