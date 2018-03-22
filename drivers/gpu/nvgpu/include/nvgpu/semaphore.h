/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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

struct nvgpu_semaphore_loc {
	struct nvgpu_semaphore_pool *pool; /* Pool that owns this sema. */
	u32 offset;			   /* Byte offset into the pool. */
};

/*
 * Underlying semaphore data structure. This semaphore can be shared amongst
 * other semaphore instances.
 */
struct nvgpu_semaphore_int {
	struct nvgpu_semaphore_loc location;
	nvgpu_atomic_t next_value;	/* Next available value. */
	struct channel_gk20a *ch;	/* Channel that owns this sema. */
};

/*
 * A semaphore which the rest of the driver actually uses. This consists of a
 * pointer to a real semaphore and a value to wait for. This allows one physical
 * semaphore to be shared among an essentially infinite number of submits.
 */
struct nvgpu_semaphore {
	struct gk20a *g;
	struct nvgpu_semaphore_loc location;

	nvgpu_atomic_t value;
	int incremented;

	struct nvgpu_ref ref;
};

/*
 * A semaphore pool. Each address space will own exactly one of these.
 */
struct nvgpu_semaphore_pool {
	struct nvgpu_list_node pool_list_entry;	/* Node for list of pools. */
	u64 gpu_va;				/* GPU access to the pool. */
	u64 gpu_va_ro;				/* GPU access to the pool. */
	int page_idx;				/* Index into sea bitmap. */

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
	struct nvgpu_ref ref;
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
void nvgpu_semaphore_sea_destroy(struct gk20a *g);
int nvgpu_semaphore_sea_map(struct nvgpu_semaphore_pool *sea,
			    struct vm_gk20a *vm);
void nvgpu_semaphore_sea_unmap(struct nvgpu_semaphore_pool *sea,
			       struct vm_gk20a *vm);
struct nvgpu_semaphore_sea *nvgpu_semaphore_get_sea(struct gk20a *g);

/*
 * Semaphore pool functions.
 */
int nvgpu_semaphore_pool_alloc(struct nvgpu_semaphore_sea *sea,
			       struct nvgpu_semaphore_pool **pool);
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
	return __nvgpu_semaphore_pool_gpu_va(s->location.pool, false) +
		s->location.offset;
}

/*
 * Get the global RO address for the semaphore. Can be called on any semaphore
 * regardless of whether you own it.
 */
static inline u64 nvgpu_semaphore_gpu_ro_va(struct nvgpu_semaphore *s)
{
	return __nvgpu_semaphore_pool_gpu_va(s->location.pool, true) +
		s->location.offset;
}

static inline u64 nvgpu_hw_sema_addr(struct nvgpu_semaphore_int *hw_sema)
{
	return __nvgpu_semaphore_pool_gpu_va(hw_sema->location.pool, true) +
		hw_sema->location.offset;
}

static inline u32 __nvgpu_semaphore_read(struct nvgpu_semaphore_int *hw_sema)
{
	return nvgpu_mem_rd(hw_sema->ch->g, &hw_sema->location.pool->rw_mem,
			hw_sema->location.offset);
}

/*
 * Read the underlying value from a semaphore.
 */
static inline u32 nvgpu_semaphore_read(struct nvgpu_semaphore *s)
{
	return nvgpu_mem_rd(s->g, &s->location.pool->rw_mem,
			s->location.offset);
}

/*
 * Check if "racer" is over "goal" with wraparound handling.
 */
static inline bool __nvgpu_semaphore_value_released(u32 goal, u32 racer)
{
	/*
	 * Handle wraparound with the same heuristic as the hardware does:
	 * although the integer will eventually wrap around, consider a sema
	 * released against a threshold if its value has passed that threshold
	 * but has not wrapped over half of the u32 range over that threshold;
	 * such wrapping is unlikely to happen during a sema lifetime.
	 *
	 * Values for [goal, goal + 0x7fffffff] are considered signaled; that's
	 * precisely half of the 32-bit space. If racer == goal + 0x80000000,
	 * then it needs 0x80000000 increments to wrap again and signal.
	 *
	 * Unsigned arithmetic is used because it's well-defined. This is
	 * effectively the same as: signed_racer - signed_goal > 0.
	 */

	return racer - goal < 0x80000000;
}

static inline u32 nvgpu_semaphore_get_value(struct nvgpu_semaphore *s)
{
	return (u32)nvgpu_atomic_read(&s->value);
}

static inline bool nvgpu_semaphore_is_released(struct nvgpu_semaphore *s)
{
	u32 sema_val = nvgpu_semaphore_read(s);
	u32 wait_payload = nvgpu_semaphore_get_value(s);

	return __nvgpu_semaphore_value_released(wait_payload, sema_val);
}

static inline bool nvgpu_semaphore_is_acquired(struct nvgpu_semaphore *s)
{
	return !nvgpu_semaphore_is_released(s);
}

/*
 * Fast-forward the hw sema to its tracked max value.
 *
 * Return true if the sema wasn't at the max value and needed updating, false
 * otherwise.
 */
static inline bool nvgpu_semaphore_reset(struct nvgpu_semaphore_int *hw_sema)
{
	u32 threshold = (u32)nvgpu_atomic_read(&hw_sema->next_value);
	u32 current_val = __nvgpu_semaphore_read(hw_sema);

	/*
	 * If the semaphore has already reached the value we would write then
	 * this is really just a NO-OP. However, the sema value shouldn't be
	 * more than what we expect to be the max.
	 */

	if (WARN_ON(__nvgpu_semaphore_value_released(threshold + 1,
						     current_val)))
		return false;

	if (current_val == threshold)
		return false;

	nvgpu_mem_wr(hw_sema->ch->g, &hw_sema->location.pool->rw_mem,
			hw_sema->location.offset, threshold);

	gpu_sema_verbose_dbg(hw_sema->ch->g, "(c=%d) RESET %u -> %u",
			hw_sema->ch->chid, current_val, threshold);

	return true;
}

/*
 * Update nvgpu-tracked shadow of the value in "hw_sema" and mark the threshold
 * value to "s" which represents the increment that the caller must write in a
 * pushbuf. The same nvgpu_semaphore will also represent an output fence; when
 * nvgpu_semaphore_is_released(s) == true, the gpu is done with this increment.
 */
static inline void nvgpu_semaphore_prepare(struct nvgpu_semaphore *s,
		struct nvgpu_semaphore_int *hw_sema)
{
	int next = nvgpu_atomic_add_return(1, &hw_sema->next_value);

	/* "s" should be an uninitialized sema. */
	WARN_ON(s->incremented);

	nvgpu_atomic_set(&s->value, next);
	s->incremented = 1;

	gpu_sema_verbose_dbg(s->g, "INCR sema for c=%d (%u)",
			     hw_sema->ch->chid, next);
}
#endif
