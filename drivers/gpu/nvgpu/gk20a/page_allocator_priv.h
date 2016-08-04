/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef PAGE_ALLOCATOR_PRIV_H
#define PAGE_ALLOCATOR_PRIV_H

#include <linux/list.h>
#include <linux/rbtree.h>
#include <gk20a/gk20a_allocator.h>

#include "gk20a_allocator.h"

struct gk20a_allocator;

struct page_alloc_chunk {
	struct list_head list_entry;

	u64 base;
	u64 length;
};

/*
 * Struct to handle internal management of page allocation. It holds a list
 * of the chunks of page that make up the overall allocation - much like a
 * scatter gather table.
 */
struct gk20a_page_alloc {
	struct list_head alloc_chunks;

	int nr_chunks;
	u64 length;

	/*
	 * Only useful for the RB tree - since the alloc will have discontiguous
	 * pages the base is essentially irrelevant except for the fact that it
	 * is guarenteed to be unique.
	 */
	u64 base;

	struct rb_node tree_entry;
};

struct gk20a_page_allocator {
	struct gk20a_allocator *owner;	/* Owner of this allocator. */

	/*
	 * Use a buddy allocator to manage the allocation of the underlying
	 * pages. This lets us abstract the discontiguous allocation handling
	 * out of the annoyingly complicated buddy allocator.
	 */
	struct gk20a_allocator source_allocator;

	/*
	 * Page params.
	 */
	u64 base;
	u64 length;
	u64 page_size;
	u32 page_shift;

	struct rb_root allocs;		/* Outstanding allocations. */

	u64 flags;

	/*
	 * Stat tracking.
	 */
	u64 nr_allocs;
	u64 nr_frees;
	u64 nr_fixed_allocs;
	u64 nr_fixed_frees;
	u64 pages_alloced;
	u64 pages_freed;
};

static inline struct gk20a_page_allocator *page_allocator(
	struct gk20a_allocator *a)
{
	return (struct gk20a_page_allocator *)(a)->priv;
}

static inline struct gk20a_allocator *palloc_owner(
	struct gk20a_page_allocator *a)
{
	return a->owner;
}

#endif
