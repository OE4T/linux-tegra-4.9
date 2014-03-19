/*
 * gk20a allocator
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVHOST_ALLOCATOR_H__
#define __NVHOST_ALLOCATOR_H__

#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

/* #define ALLOCATOR_DEBUG */

struct allocator_block;

/* main struct */
struct gk20a_allocator {

	char name[32];			/* name for allocator */
	struct rb_root rb_root;		/* rb tree root for blocks */

	u32 base;			/* min value of this linear space */
	u32 limit;			/* max value = limit - 1 */
	u32 align;			/* alignment size, power of 2 */

	struct gk20a_alloc_block *block_first;	/* first block in list */
	struct gk20a_alloc_block *block_recent; /* last visited block */

	u32 first_free_addr;		/* first free addr, non-contigous
					   allocation preferred start,
					   in order to pick up small holes */
	u32 last_free_addr;		/* last free addr, contiguous
					   allocation preferred start */
	u32 cached_hole_size;		/* max free hole size up to
					   last_free_addr */
	u32 block_count;		/* number of blocks */

	struct rw_semaphore rw_sema;	/* lock */
	struct kmem_cache *block_cache;	/* slab cache */

	/* if enabled, constrain to [base, limit) */
	struct {
		bool enable;
		u32 base;
		u32 limit;
	} constraint;

	int (*alloc)(struct gk20a_allocator *allocator,
		u32 *addr, u32 len);
	int (*alloc_nc)(struct gk20a_allocator *allocator,
		u32 *addr, u32 len,
		struct gk20a_alloc_block **pblock);
	int (*free)(struct gk20a_allocator *allocator,
		u32 addr, u32 len);
	void (*free_nc)(struct gk20a_allocator *allocator,
		struct gk20a_alloc_block *block);

	int (*constrain)(struct gk20a_allocator *a,
			 bool enable,
			 u32 base, u32 limit);
};

/* a block of linear space range [start, end) */
struct gk20a_alloc_block {
	struct gk20a_allocator *allocator;	/* parent allocator */
	struct rb_node rb;			/* rb tree node */

	u32 start;				/* linear space range
						   [start, end) */
	u32 end;

	void *priv;				/* backing structure for this
						   linear space block
						   page table, comp tag, etc */

	struct gk20a_alloc_block *prev;	/* prev block with lower address */
	struct gk20a_alloc_block *next;	/* next block with higher address */

	bool nc_block;
	struct gk20a_alloc_block *nc_prev;	/* prev block for
						   non-contiguous allocation */
	struct gk20a_alloc_block *nc_next;	/* next block for
						   non-contiguous allocation */
};

int gk20a_allocator_init(struct gk20a_allocator *allocator,
			const char *name, u32 base, u32 size, u32 align);
void gk20a_allocator_destroy(struct gk20a_allocator *allocator);

int gk20a_allocator_block_alloc(struct gk20a_allocator *allocator,
			u32 *addr, u32 len);
int gk20a_allocator_block_alloc_nc(struct gk20a_allocator *allocator,
			u32 *addr, u32 len,
			struct gk20a_alloc_block **pblock);

int gk20a_allocator_block_free(struct gk20a_allocator *allocator,
			u32 addr, u32 len);
void gk20a_allocator_block_free_nc(struct gk20a_allocator *allocator,
			struct gk20a_alloc_block *block);

#if defined(ALLOCATOR_DEBUG)

#define allocator_dbg(alloctor, format, arg...)				\
do {								\
	if (1)							\
		pr_debug("gk20a_allocator (%s) %s: " format "\n",\
			alloctor->name, __func__, ##arg);\
} while (0)

static inline void
gk20a_allocator_dump(struct gk20a_allocator *allocator) {
	struct gk20a_alloc_block *block;
	u32 count = 0;

	down_read(&allocator->rw_sema);
	for (block = allocator->block_first; block; block = block->next) {
		allocator_dbg(allocator, "block %d - %d:%d, nc %d",
			count++, block->start, block->end, block->nc_block);

		if (block->prev)
			BUG_ON(block->prev->end > block->start);
		if (block->next)
			BUG_ON(block->next->start < block->end);
	}
	allocator_dbg(allocator, "tracked count %d, actual count %d",
		allocator->block_count, count);
	allocator_dbg(allocator, "first block %d:%d",
		allocator->block_first ? allocator->block_first->start : -1,
		allocator->block_first ? allocator->block_first->end : -1);
	allocator_dbg(allocator, "first free addr %d",
		allocator->first_free_addr);
	allocator_dbg(allocator, "last free addr %d",
		allocator->last_free_addr);
	allocator_dbg(allocator, "cached hole size %d",
		allocator->cached_hole_size);
	up_read(&allocator->rw_sema);

	BUG_ON(count != allocator->block_count);
}

static inline void
gk20a_allocator_dump_nc_list(
		struct gk20a_allocator *allocator,
		struct gk20a_alloc_block *block)
{
	down_read(&allocator->rw_sema);
	while (block) {
		pr_debug("non-contiguous block %d:%d\n",
			block->start, block->end);
		block = block->nc_next;
	}
	up_read(&allocator->rw_sema);
}

void gk20a_allocator_test(void);

#else /* ALLOCATOR_DEBUG */

#define allocator_dbg(format, arg...)

#endif /* ALLOCATOR_DEBUG */

#endif /*__NVHOST_ALLOCATOR_H__ */
