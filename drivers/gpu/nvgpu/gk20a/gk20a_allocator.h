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

struct gk20a_allocator {
	char name[32];
	struct mutex lock;

	void *priv;
	const struct gk20a_allocator_ops *ops;

	struct dentry *debugfs_entry;
};

/*
 * Allocator flags.
 */
#define GPU_BALLOC_GVA_SPACE		0x1

static inline void alloc_lock(struct gk20a_allocator *a)
{
	mutex_lock(&a->lock);
}

static inline void alloc_unlock(struct gk20a_allocator *a)
{
	mutex_unlock(&a->lock);
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

#define GPU_BALLOC_MAX_ORDER		31

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
 * Common functionality for the internals of the allocators.
 */
void gk20a_init_alloc_debug(struct gk20a_allocator *a);
void gk20a_fini_alloc_debug(struct gk20a_allocator *a);
int  __gk20a_alloc_common_init(struct gk20a_allocator *a,
			       const char *name, void *priv,
			       const struct gk20a_allocator_ops *ops);

/*
 * Debug stuff.
 */
extern u32 gk20a_alloc_tracing_on;

void gk20a_alloc_debugfs_init(struct platform_device *pdev);

#define gk20a_alloc_trace_func()			\
	do {						\
		if (gk20a_alloc_tracing_on)		\
			trace_printk("%s\n", __func__);	\
	} while (0)

#define gk20a_alloc_trace_func_done()				\
	do {							\
		if (gk20a_alloc_tracing_on)			\
			trace_printk("%s_done\n", __func__);	\
	} while (0)

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
