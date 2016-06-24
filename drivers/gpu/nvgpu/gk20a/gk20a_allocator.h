/*
 * Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef GK20A_ALLOCATOR_H
#define GK20A_ALLOCATOR_H

#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>

/* #define ALLOCATOR_DEBUG */

struct gk20a_allocator;
struct vm_gk20a;

/*
 * Operations for an allocator to implement.
 */
struct gk20a_allocator_ops {
	u64  (*alloc)(struct gk20a_allocator *allocator, u64 len);
	void (*free)(struct gk20a_allocator *allocator, u64 addr);

	/*
	 * Special interface to allocate a memory region with a specific
	 * starting address. Yikes. Note: if free() works for freeing both
	 * regular and fixed allocations then free_fixed() does not need to
	 * be implemented. This behavior exists for legacy reasons and should
	 * not be propagated to new allocators.
	 */
	u64  (*alloc_fixed)(struct gk20a_allocator *allocator,
			     u64 base, u64 len);
	void (*free_fixed)(struct gk20a_allocator *allocator,
			    u64 base, u64 len);

	/*
	 * Returns info about the allocator.
	 */
	u64  (*base)(struct gk20a_allocator *allocator);
	u64  (*length)(struct gk20a_allocator *allocator);
	u64  (*end)(struct gk20a_allocator *allocator);
	int  (*inited)(struct gk20a_allocator *allocator);

	/* Destructor. */
	void (*fini)(struct gk20a_allocator *allocator);

	/* Debugging. */
	void (*print_stats)(struct gk20a_allocator *allocator,
			    struct seq_file *s, int lock);
};

/*
 * Each buddy is an element in a binary tree.
 */
struct gk20a_buddy {
	struct gk20a_buddy *parent;	/* Parent node. */
	struct gk20a_buddy *buddy;	/* This node's buddy. */
	struct gk20a_buddy *left;	/* Lower address sub-node. */
	struct gk20a_buddy *right;	/* Higher address sub-node. */

	struct list_head buddy_entry;	/* List entry for various lists. */
	struct rb_node alloced_entry;	/* RB tree of allocations. */

	u64 start;			/* Start address of this buddy. */
	u64 end;			/* End address of this buddy. */
	u64 order;			/* Buddy order. */

#define BALLOC_BUDDY_ALLOCED	0x1
#define BALLOC_BUDDY_SPLIT	0x2
#define BALLOC_BUDDY_IN_LIST	0x4
	int flags;			/* List of associated flags. */

	/*
	 * Size of the PDE this buddy is using. This allows for grouping like
	 * sized allocations into the same PDE.
	 */
#define BALLOC_PTE_SIZE_ANY	0x0
#define BALLOC_PTE_SIZE_SMALL	0x1
#define BALLOC_PTE_SIZE_BIG	0x2
	int pte_size;
};

#define __buddy_flag_ops(flag, flag_up)					\
	static inline int buddy_is_ ## flag(struct gk20a_buddy *b)	\
	{								\
		return b->flags & BALLOC_BUDDY_ ## flag_up;		\
	}								\
	static inline void buddy_set_ ## flag(struct gk20a_buddy *b)	\
	{								\
		b->flags |= BALLOC_BUDDY_ ## flag_up;			\
	}								\
	static inline void buddy_clr_ ## flag(struct gk20a_buddy *b)	\
	{								\
		b->flags &= ~BALLOC_BUDDY_ ## flag_up;			\
	}

/*
 * int  buddy_is_alloced(struct gk20a_buddy *b);
 * void buddy_set_alloced(struct gk20a_buddy *b);
 * void buddy_clr_alloced(struct gk20a_buddy *b);
 *
 * int  buddy_is_split(struct gk20a_buddy *b);
 * void buddy_set_split(struct gk20a_buddy *b);
 * void buddy_clr_split(struct gk20a_buddy *b);
 *
 * int  buddy_is_in_list(struct gk20a_buddy *b);
 * void buddy_set_in_list(struct gk20a_buddy *b);
 * void buddy_clr_in_list(struct gk20a_buddy *b);
 */
__buddy_flag_ops(alloced, ALLOCED);
__buddy_flag_ops(split,   SPLIT);
__buddy_flag_ops(in_list, IN_LIST);

/*
 * Keeps info for a fixed allocation.
 */
struct gk20a_fixed_alloc {
	struct list_head buddies;	/* List of buddies. */
	struct rb_node alloced_entry;	/* RB tree of fixed allocations. */

	u64 start;			/* Start of fixed block. */
	u64 end;			/* End address. */
};

/*
 * GPU buddy allocator for the various GPU address spaces. Each addressable unit
 * doesn't have to correspond to a byte. In some cases each unit is a more
 * complex object such as a comp_tag line or the like.
 *
 * The max order is computed based on the size of the minimum order and the size
 * of the address space.
 *
 * order_size is the size of an order 0 buddy.
 */
struct gk20a_buddy_allocator {
	struct gk20a_allocator *owner;	/* Owner of this buddy allocator. */
	struct vm_gk20a *vm;		/* Parent VM - can be NULL. */

	u64 base;			/* Base address of the space. */
	u64 length;			/* Length of the space. */
	u64 blk_size;			/* Size of order 0 allocation. */
	u64 blk_shift;			/* Shift to divide by blk_size. */

	int init;			/* Non-zero if initialized. */

	/* Internal stuff. */
	u64 start;			/* Real start (aligned to blk_size). */
	u64 end;			/* Real end, trimmed if needed. */
	u64 count;			/* Count of objects in space. */
	u64 blks;			/* Count of blks in the space. */
	u64 max_order;			/* Specific maximum order. */

	struct rb_root alloced_buddies;	/* Outstanding allocations. */
	struct rb_root fixed_allocs;	/* Outstanding fixed allocations. */

	/*
	 * Impose an upper bound on the maximum order.
	 */
#define GPU_BALLOC_MAX_ORDER		31
#define GPU_BALLOC_ORDER_LIST_LEN	(GPU_BALLOC_MAX_ORDER + 1)

	struct list_head buddy_list[GPU_BALLOC_ORDER_LIST_LEN];
	u64 buddy_list_len[GPU_BALLOC_ORDER_LIST_LEN];
	u64 buddy_list_split[GPU_BALLOC_ORDER_LIST_LEN];
	u64 buddy_list_alloced[GPU_BALLOC_ORDER_LIST_LEN];

	/*
	 * This is for when the allocator is managing a GVA space (the
	 * GPU_BALLOC_GVA_SPACE bit is set in @flags). This requires
	 * that we group like sized allocations into PDE blocks.
	 */
	u64 pte_blk_order;

	int inited;

#define GPU_BALLOC_GVA_SPACE		0x1
	u64 flags;

	u64 bytes_alloced;
	u64 bytes_alloced_real;
	u64 bytes_freed;
};

struct gk20a_allocator {
	char name[32];
	struct mutex lock;

	void *priv;
	const struct gk20a_allocator_ops *ops;

	struct dentry *debugfs_entry;
};

static inline void alloc_lock(struct gk20a_allocator *a)
{
	mutex_lock(&a->lock);
}

static inline void alloc_unlock(struct gk20a_allocator *a)
{
	mutex_unlock(&a->lock);
}

static inline struct gk20a_buddy_allocator *buddy_allocator(
	struct gk20a_allocator *a)
{
	return (struct gk20a_buddy_allocator *)a->priv;
}

static inline struct list_head *balloc_get_order_list(
	struct gk20a_buddy_allocator *a, int order)
{
	return &a->buddy_list[order];
}

static inline u64 balloc_order_to_len(struct gk20a_buddy_allocator *a,
				      int order)
{
	return (1 << order) * a->blk_size;
}

static inline u64 balloc_base_shift(struct gk20a_buddy_allocator *a,
				    u64 base)
{
	return base - a->start;
}

static inline u64 balloc_base_unshift(struct gk20a_buddy_allocator *a,
				      u64 base)
{
	return base + a->start;
}

static inline struct gk20a_allocator *balloc_owner(
	struct gk20a_buddy_allocator *a)
{
	return a->owner;
}

/*
 * Buddy allocator specific initializers.
 */
int  __gk20a_buddy_allocator_init(struct gk20a_allocator *a,
				  struct vm_gk20a *vm, const char *name,
				  u64 base, u64 size, u64 blk_size,
				  u64 max_order, u64 flags);
int  gk20a_buddy_allocator_init(struct gk20a_allocator *allocator,
				const char *name, u64 base, u64 size,
				u64 blk_size, u64 flags);

/*
 * Allocator APIs.
 */
u64  gk20a_alloc(struct gk20a_allocator *allocator, u64 len);
void gk20a_free(struct gk20a_allocator *allocator, u64 addr);

u64  gk20a_alloc_fixed(struct gk20a_allocator *allocator, u64 base, u64 len);
void gk20a_free_fixed(struct gk20a_allocator *allocator, u64 base, u64 len);

u64  gk20a_alloc_base(struct gk20a_allocator *a);
u64  gk20a_alloc_length(struct gk20a_allocator *a);
u64  gk20a_alloc_end(struct gk20a_allocator *a);
u64  gk20a_alloc_initialized(struct gk20a_allocator *a);

void gk20a_alloc_destroy(struct gk20a_allocator *allocator);

void gk20a_alloc_print_stats(struct gk20a_allocator *a,
			     struct seq_file *s, int lock);

/*
 * Debug stuff.
 */
void gk20a_alloc_debugfs_init(struct platform_device *pdev);

#define __alloc_pstat(seq, allocator, fmt, arg...)		\
	do {							\
		if (s)						\
			seq_printf(seq, fmt, ##arg);		\
		else						\
			alloc_dbg(allocator, fmt, ##arg);	\
	} while (0)

#if defined(ALLOCATOR_DEBUG)
#define alloc_dbg(allocator, format, arg...)		\
	pr_info("%-25s %25s() " format,			\
		allocator->name, __func__, ##arg)
#else
#define alloc_dbg(allocator, format, arg...)
#endif

#endif /* GK20A_ALLOCATOR_H */
