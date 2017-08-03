/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/kref.h>
#include <nvgpu/list.h>
#include <nvgpu/nvgpu_mem.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/channel_gk20a.h"

#define gpu_sema_dbg(g, fmt, args...)		\
	nvgpu_log(g, gpu_dbg_sema, fmt, ##args)
#define gpu_sema_verbose_dbg(g, fmt, args...)	\
	nvgpu_log(g, gpu_dbg_sema_v, fmt, ##args)

/*
 * Max number of channels that can be used is 512. This of course needs to be
 * fixed to be dynamic but still fast.
 */
#define SEMAPHORE_POOL_COUNT		512
#define SEMAPHORE_SIZE			16
#define SEMAPHORE_SEA_GROWTH_RATE	32

struct nvgpu_semaphore_sea;

/*
 * Underlying semaphore data structure. This semaphore can be shared amongst
 * other semaphore instances.
 */
struct nvgpu_semaphore_int {
	int idx;			/* Semaphore index. */
	u32 offset;			/* Offset into the pool. */
	nvgpu_atomic_t next_value;	/* Next available value. */
	u32 nr_incrs;			/* Number of increments programmed. */
	struct nvgpu_semaphore_pool *p;	/* Pool that owns this sema. */
	struct channel_gk20a *ch;	/* Channel that owns this sema. */
	struct nvgpu_list_node hw_sema_list;	/* List of HW semaphores. */
};

static inline struct nvgpu_semaphore_int *
nvgpu_semaphore_int_from_hw_sema_list(struct nvgpu_list_node *node)
{
	return (struct nvgpu_semaphore_int *)
	 ((uintptr_t)node - offsetof(struct nvgpu_semaphore_int, hw_sema_list));
};

/*
 * A semaphore which the rest of the driver actually uses. This consists of a
 * pointer to a real semaphore and a value to wait for. This allows one physical
 * semaphore to be shared among an essentially infinite number of submits.
 */
struct nvgpu_semaphore {
	struct nvgpu_semaphore_int *hw_sema;

	nvgpu_atomic_t value;
	int incremented;

	struct kref ref;
};

/*
 * A semaphore pool. Each address space will own exactly one of these.
 */
struct nvgpu_semaphore_pool {
	struct nvgpu_list_node pool_list_entry;	/* Node for list of pools. */
	u64 gpu_va;				/* GPU access to the pool. */
	u64 gpu_va_ro;				/* GPU access to the pool. */
	int page_idx;				/* Index into sea bitmap. */

	struct nvgpu_list_node hw_semas;	/* List of HW semas. */
	DECLARE_BITMAP(semas_alloced, PAGE_SIZE / SEMAPHORE_SIZE);

	struct nvgpu_semaphore_sea *sema_sea;	/* Sea that owns this pool. */

	struct nvgpu_mutex pool_lock;

	/*
	 * This is the address spaces's personal RW table. Other channels will
	 * ultimately map this page as RO. This is a sub-nvgpu_mem from the
	 * sea's mem.
	 */
	struct nvgpu_mem rw_mem;

	int mapped;

	/*
	 * Sometimes a channel can be released before other channels are
	 * done waiting on it. This ref count ensures that the pool doesn't
	 * go away until all semaphores using this pool are cleaned up first.
	 */
	struct kref ref;
};

static inline struct nvgpu_semaphore_pool *
nvgpu_semaphore_pool_from_pool_list_entry(struct nvgpu_list_node *node)
{
	return (struct nvgpu_semaphore_pool *)
		((uintptr_t)node -
		offsetof(struct nvgpu_semaphore_pool, pool_list_entry));
};

/*
 * A sea of semaphores pools. Each pool is owned by a single VM. Since multiple
 * channels can share a VM each channel gets it's own HW semaphore from the
 * pool. Channels then allocate regular semaphores - basically just a value that
 * signifies when a particular job is done.
 */
struct nvgpu_semaphore_sea {
	struct nvgpu_list_node pool_list;	/* List of pools in this sea. */
	struct gk20a *gk20a;

	size_t size;			/* Number of pages available. */
	u64 gpu_va;			/* GPU virtual address of sema sea. */
	u64 map_size;			/* Size of the mapping. */

	/*
	 * TODO:
	 * List of pages that we use to back the pools. The number of pages
	 * can grow dynamically since allocating 512 pages for all channels at
	 * once would be a tremendous waste.
	 */
	int page_count;			/* Pages allocated to pools. */

	/*
	 * The read-only memory for the entire semaphore sea. Each semaphore
	 * pool needs a sub-nvgpu_mem that will be mapped as RW in its address
	 * space. This sea_mem cannot be freed until all semaphore_pools have
	 * been freed.
	 */
	struct nvgpu_mem sea_mem;

	/*
	 * Can't use a regular allocator here since the full range of pools are
	 * not always allocated. Instead just use a bitmap.
	 */
	DECLARE_BITMAP(pools_alloced, SEMAPHORE_POOL_COUNT);

	struct nvgpu_mutex sea_lock;		/* Lock alloc/free calls. */
};

/*
 * Semaphore sea functions.
 */
struct nvgpu_semaphore_sea *nvgpu_semaphore_sea_create(struct gk20a *gk20a);
void gk20a_semaphore_sea_destroy(struct gk20a *g);
int nvgpu_semaphore_sea_map(struct nvgpu_semaphore_pool *sea,
			    struct vm_gk20a *vm);
void nvgpu_semaphore_sea_unmap(struct nvgpu_semaphore_pool *sea,
			       struct vm_gk20a *vm);
struct nvgpu_semaphore_sea *nvgpu_semaphore_get_sea(struct gk20a *g);

/*
 * Semaphore pool functions.
 */
struct nvgpu_semaphore_pool *nvgpu_semaphore_pool_alloc(
	struct nvgpu_semaphore_sea *sea);
int nvgpu_semaphore_pool_map(struct nvgpu_semaphore_pool *pool,
			     struct vm_gk20a *vm);
void nvgpu_semaphore_pool_unmap(struct nvgpu_semaphore_pool *pool,
				struct vm_gk20a *vm);
u64 __nvgpu_semaphore_pool_gpu_va(struct nvgpu_semaphore_pool *p, bool global);
void nvgpu_semaphore_pool_get(struct nvgpu_semaphore_pool *p);
void nvgpu_semaphore_pool_put(struct nvgpu_semaphore_pool *p);

/*
 * Semaphore functions.
 */
struct nvgpu_semaphore *nvgpu_semaphore_alloc(struct channel_gk20a *ch);
void nvgpu_semaphore_put(struct nvgpu_semaphore *s);
void nvgpu_semaphore_get(struct nvgpu_semaphore *s);
void nvgpu_semaphore_free_hw_sema(struct channel_gk20a *ch);

/*
 * Return the address of a specific semaphore.
 *
 * Don't call this on a semaphore you don't own - the VA returned will make no
 * sense in your specific channel's VM.
 */
static inline u64 nvgpu_semaphore_gpu_rw_va(struct nvgpu_semaphore *s)
{
	return __nvgpu_semaphore_pool_gpu_va(s->hw_sema->p, false) +
		s->hw_sema->offset;
}

/*
 * Get the global RO address for the semaphore. Can be called on any semaphore
 * regardless of whether you own it.
 */
static inline u64 nvgpu_semaphore_gpu_ro_va(struct nvgpu_semaphore *s)
{
	return __nvgpu_semaphore_pool_gpu_va(s->hw_sema->p, true) +
		s->hw_sema->offset;
}

static inline u64 nvgpu_hw_sema_addr(struct nvgpu_semaphore_int *hw_sema)
{
	return __nvgpu_semaphore_pool_gpu_va(hw_sema->p, true) +
		hw_sema->offset;
}

static inline u32 __nvgpu_semaphore_read(struct nvgpu_semaphore_int *hw_sema)
{
	return nvgpu_mem_rd(hw_sema->ch->g,
			    &hw_sema->p->rw_mem, hw_sema->offset);
}

/*
 * Read the underlying value from a semaphore.
 */
static inline u32 nvgpu_semaphore_read(struct nvgpu_semaphore *s)
{
	return __nvgpu_semaphore_read(s->hw_sema);
}

/*
 * TODO: handle wrap around... Hmm, how to do this?
 */
static inline bool nvgpu_semaphore_is_released(struct nvgpu_semaphore *s)
{
	u32 sema_val = nvgpu_semaphore_read(s);

	/*
	 * If the underlying semaphore value is greater than or equal to
	 * the value of the semaphore then the semaphore has been signaled
	 * (a.k.a. released).
	 */
	return (int)sema_val >= nvgpu_atomic_read(&s->value);
}

static inline bool nvgpu_semaphore_is_acquired(struct nvgpu_semaphore *s)
{
	return !nvgpu_semaphore_is_released(s);
}

static inline u32 nvgpu_semaphore_get_value(struct nvgpu_semaphore *s)
{
	return (u32)nvgpu_atomic_read(&s->value);
}

static inline u32 nvgpu_semaphore_next_value(struct nvgpu_semaphore *s)
{
	return (u32)nvgpu_atomic_read(&s->hw_sema->next_value);
}

/*
 * If @force is set then this will not wait for the underlying semaphore to
 * catch up to the passed semaphore.
 */
static inline void __nvgpu_semaphore_release(struct nvgpu_semaphore *s,
					     bool force)
{
	struct nvgpu_semaphore_int *hw_sema = s->hw_sema;
	u32 current_val;
	u32 val = nvgpu_semaphore_get_value(s);
	int attempts = 0;

	/*
	 * Wait until the sema value is 1 less than the write value. That
	 * way this function is essentially an increment.
	 *
	 * TODO: tune the wait a little better.
	 */
	while ((current_val = nvgpu_semaphore_read(s)) < (val - 1)) {
		if (force)
			break;
		nvgpu_msleep(100);
		attempts += 1;
		if (attempts > 100) {
			WARN(1, "Stall on sema release!");
			return;
		}
	}

	/*
	 * If the semaphore has already passed the value we would write then
	 * this is really just a NO-OP.
	 */
	if (current_val >= val)
		return;

	nvgpu_mem_wr(hw_sema->ch->g, &hw_sema->p->rw_mem, hw_sema->offset, val);

	gpu_sema_verbose_dbg(hw_sema->p->sema_sea->gk20a,
			     "(c=%d) WRITE %u", hw_sema->ch->chid, val);
}

static inline void nvgpu_semaphore_release(struct nvgpu_semaphore *s)
{
	__nvgpu_semaphore_release(s, false);
}

/*
 * Configure a software based increment on this semaphore. This is useful for
 * when we want the GPU to wait on a SW event before processing a channel.
 * Another way to describe this is when the GPU needs to wait on a SW pre-fence.
 * The pre-fence signals SW which in turn calls nvgpu_semaphore_release() which
 * then allows the GPU to continue.
 *
 * Also used to prep a semaphore for an INCR by the GPU.
 */
static inline void nvgpu_semaphore_incr(struct nvgpu_semaphore *s)
{
	BUG_ON(s->incremented);

	nvgpu_atomic_set(&s->value, nvgpu_atomic_add_return(1, &s->hw_sema->next_value));
	s->incremented = 1;

	gpu_sema_verbose_dbg(s->hw_sema->p->sema_sea->gk20a,
			     "INCR sema for c=%d (%u)",
			     s->hw_sema->ch->chid,
			     nvgpu_semaphore_next_value(s));
}
#endif
