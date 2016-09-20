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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/bitops.h>

#include "gk20a_allocator.h"
#include "buddy_allocator_priv.h"
#include "page_allocator_priv.h"

#define palloc_dbg(a, fmt, arg...)			\
	alloc_dbg(palloc_owner(a), fmt, ##arg)

static struct kmem_cache *page_alloc_cache;
static struct kmem_cache *page_alloc_chunk_cache;
static DEFINE_MUTEX(meta_data_cache_lock);

static u64 gk20a_page_alloc_length(struct gk20a_allocator *a)
{
	struct gk20a_page_allocator *va = a->priv;

	return gk20a_alloc_length(&va->source_allocator);
}

static u64 gk20a_page_alloc_base(struct gk20a_allocator *a)
{
	struct gk20a_page_allocator *va = a->priv;

	return gk20a_alloc_base(&va->source_allocator);
}

static int gk20a_page_alloc_inited(struct gk20a_allocator *a)
{
	struct gk20a_page_allocator *va = a->priv;

	return gk20a_alloc_initialized(&va->source_allocator);
}

static u64 gk20a_page_alloc_end(struct gk20a_allocator *a)
{
	struct gk20a_page_allocator *va = a->priv;

	return gk20a_alloc_end(&va->source_allocator);
}

static u64 gk20a_page_alloc_space(struct gk20a_allocator *a)
{
	struct gk20a_page_allocator *va = a->priv;

	return gk20a_alloc_space(&va->source_allocator);
}

static int gk20a_page_reserve_co(struct gk20a_allocator *a,
				 struct gk20a_alloc_carveout *co)
{
	struct gk20a_page_allocator *va = a->priv;

	return gk20a_alloc_reserve_carveout(&va->source_allocator, co);
}

static void gk20a_page_release_co(struct gk20a_allocator *a,
				  struct gk20a_alloc_carveout *co)
{
	struct gk20a_page_allocator *va = a->priv;

	gk20a_alloc_release_carveout(&va->source_allocator, co);
}

static int __insert_page_alloc(struct gk20a_page_allocator *a,
			       struct gk20a_page_alloc *alloc)
{
	struct rb_node **new = &a->allocs.rb_node;
	struct rb_node *parent = NULL;

	while (*new) {
		struct gk20a_page_alloc *tmp =
			container_of(*new, struct gk20a_page_alloc,
				     tree_entry);

		parent = *new;
		if (alloc->base < tmp->base) {
			new = &((*new)->rb_left);
		} else if (alloc->base > tmp->base) {
			new = &((*new)->rb_right);
		} else {
			WARN(1, "Duplicate entries in allocated list!\n");
			return 0;
		}
	}

	rb_link_node(&alloc->tree_entry, parent, new);
	rb_insert_color(&alloc->tree_entry, &a->allocs);

	return 0;
}

static struct gk20a_page_alloc *__find_page_alloc(
	struct gk20a_page_allocator *a,
	u64 addr)
{
	struct rb_node *node = a->allocs.rb_node;
	struct gk20a_page_alloc *alloc;

	while (node) {
		alloc = container_of(node, struct gk20a_page_alloc, tree_entry);

		if (addr < alloc->base)
			node = node->rb_left;
		else if (addr > alloc->base)
			node = node->rb_right;
		else
			break;
	}

	if (!node)
		return NULL;

	rb_erase(node, &a->allocs);

	return alloc;
}

/*
 * Allocate physical pages. Since the underlying allocator is a buddy allocator
 * the returned pages are always contiguous. However, since there could be
 * fragmentation in the space this allocator will collate smaller non-contiguous
 * allocations together if necessary.
 */
static struct gk20a_page_alloc *__gk20a_alloc_pages(
	struct gk20a_page_allocator *a, u64 pages)
{
	struct gk20a_page_alloc *alloc;
	struct page_alloc_chunk *c;
	u64 max_chunk_len = pages << a->page_shift;
	int i = 0;

	alloc = kmem_cache_alloc(page_alloc_cache, GFP_KERNEL);
	if (!alloc)
		goto fail;

	INIT_LIST_HEAD(&alloc->alloc_chunks);
	alloc->length = pages << a->page_shift;

	while (pages) {
		u64 chunk_addr = 0;
		u64 chunk_pages = 1 << __fls(pages);
		u64 chunk_len = chunk_pages << a->page_shift;

		/*
		 * Take care of the possibility that the allocation must be
		 * contiguous. If this is not the first iteration then that
		 * means the first iteration failed to alloc the entire
		 * requested size. The buddy allocator guarantees any given
		 * single alloc is contiguous.
		 */
		if (a->flags & GPU_ALLOC_FORCE_CONTIG && i != 0)
			goto fail_cleanup;

		if (chunk_len > max_chunk_len)
			chunk_len = max_chunk_len;

		/*
		 * Keep attempting to allocate in smaller chunks until the alloc
		 * either succeeds or is smaller than the page_size of the
		 * allocator (i.e the allocator is OOM).
		 */
		do {
			chunk_addr = gk20a_alloc(&a->source_allocator,
						 chunk_len);

			/* Divide by 2 and try again */
			if (!chunk_addr) {
				palloc_dbg(a, "balloc failed: 0x%llx\n",
					   chunk_len);
				chunk_len >>= 1;
				max_chunk_len = chunk_len;
			}
		} while (!chunk_addr && chunk_len >= a->page_size);

		chunk_pages = chunk_len >> a->page_shift;

		if (!chunk_addr) {
			palloc_dbg(a, "bailing @ 0x%llx\n", chunk_len);
			goto fail_cleanup;
		}

		c = kmem_cache_alloc(page_alloc_chunk_cache, GFP_KERNEL);
		if (!c) {
			gk20a_free(&a->source_allocator, chunk_addr);
			goto fail_cleanup;
		}

		pages -= chunk_pages;

		c->base = chunk_addr;
		c->length = chunk_len;
		list_add(&c->list_entry, &alloc->alloc_chunks);

		i++;
	}

	alloc->nr_chunks = i;
	c = list_first_entry(&alloc->alloc_chunks,
			     struct page_alloc_chunk, list_entry);
	alloc->base = c->base;

	return alloc;

fail_cleanup:
	while (!list_empty(&alloc->alloc_chunks)) {
		c = list_first_entry(&alloc->alloc_chunks,
				     struct page_alloc_chunk, list_entry);
		list_del(&c->list_entry);
		gk20a_free(&a->source_allocator, c->base);
		kfree(c);
	}
	kfree(alloc);
fail:
	return ERR_PTR(-ENOMEM);
}

/*
 * Allocate enough pages to satisfy @len. Page size is determined at
 * initialization of the allocator.
 *
 * The return is actually a pointer to a struct gk20a_page_alloc pointer. This
 * is because it doesn't make a lot of sense to return the address of the first
 * page in the list of pages (since they could be discontiguous). This has
 * precedent in the dma_alloc APIs, though, it's really just an annoying
 * artifact of the fact that the gk20a_alloc() API requires a u64 return type.
 */
static u64 gk20a_page_alloc(struct gk20a_allocator *__a, u64 len)
{
	struct gk20a_page_allocator *a = page_allocator(__a);
	struct gk20a_page_alloc *alloc = NULL;
	struct page_alloc_chunk *c;
	u64 real_len;
	u64 pages;
	int i = 0;

	/*
	 * If we want contig pages we have to round up to a power of two. It's
	 * easier to do that here than in the buddy allocator.
	 */
	real_len = a->flags & GPU_ALLOC_FORCE_CONTIG ?
		roundup_pow_of_two(len) : len;

	pages = ALIGN(real_len, a->page_size) >> a->page_shift;

	alloc_lock(__a);

	alloc = __gk20a_alloc_pages(a, pages);
	if (IS_ERR(alloc)) {
		alloc_unlock(__a);
		palloc_dbg(a, "Alloc 0x%llx (%llu) (failed)\n",
			   pages << a->page_shift, pages);
		return 0;
	}

	__insert_page_alloc(a, alloc);
	alloc_unlock(__a);

	palloc_dbg(a, "Alloc 0x%llx (%llu) id=0x%010llx\n",
		   pages << a->page_shift, pages, alloc->base);
	list_for_each_entry(c, &alloc->alloc_chunks, list_entry) {
		palloc_dbg(a, "  Chunk %2d: 0x%010llx + 0x%llx\n",
			   i++, c->base, c->length);
	}

	a->nr_allocs++;
	a->pages_alloced += pages;

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER)
		return alloc->base;
	else
		return (u64) (uintptr_t) alloc;
}

static void __gk20a_free_pages(struct gk20a_page_allocator *a,
			       struct gk20a_page_alloc *alloc)
{
	struct page_alloc_chunk *chunk;

	while (!list_empty(&alloc->alloc_chunks)) {
		chunk = list_first_entry(&alloc->alloc_chunks,
					 struct page_alloc_chunk,
					 list_entry);
		list_del(&chunk->list_entry);

		gk20a_free(&a->source_allocator, chunk->base);
		kfree(chunk);
	}

	kfree(alloc);
}

/*
 * Note: this will remove the gk20a_page_alloc struct from the RB tree
 * if it's found.
 */
static void gk20a_page_free(struct gk20a_allocator *__a, u64 base)
{
	struct gk20a_page_allocator *a = page_allocator(__a);
	struct gk20a_page_alloc *alloc;

	alloc_lock(__a);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER)
		alloc = __find_page_alloc(a, base);
	else
		alloc = __find_page_alloc(a,
			((struct gk20a_page_alloc *)(uintptr_t)base)->base);

	if (!alloc) {
		palloc_dbg(a, "Hrm, found no alloc?\n");
		goto done;
	}

	a->nr_frees++;
	a->pages_freed += (alloc->length >> a->page_shift);

	/*
	 * Frees *alloc.
	 */
	__gk20a_free_pages(a, alloc);

	palloc_dbg(a, "Free  0x%010llx id=0x%010llx\n",
		   alloc->length, alloc->base);

done:
	alloc_unlock(__a);
}

static struct gk20a_page_alloc *__gk20a_alloc_pages_fixed(
	struct gk20a_page_allocator *a, u64 base, u64 length)
{
	struct gk20a_page_alloc *alloc;
	struct page_alloc_chunk *c;

	alloc = kmem_cache_alloc(page_alloc_cache, GFP_KERNEL);
	c = kmem_cache_alloc(page_alloc_chunk_cache, GFP_KERNEL);
	if (!alloc || !c)
		goto fail;

	alloc->base = gk20a_alloc_fixed(&a->source_allocator, base, length);
	if (!alloc->base) {
		WARN(1, "gk20a: failed to fixed alloc pages @ 0x%010llx", base);
		goto fail;
	}

	alloc->nr_chunks = 1;
	alloc->length = length;
	INIT_LIST_HEAD(&alloc->alloc_chunks);

	c->base = alloc->base;
	c->length = length;
	list_add(&c->list_entry, &alloc->alloc_chunks);

	return alloc;

fail:
	kfree(c);
	kfree(alloc);
	return ERR_PTR(-ENOMEM);
}

static u64 gk20a_page_alloc_fixed(struct gk20a_allocator *__a,
				  u64 base, u64 len)
{
	struct gk20a_page_allocator *a = page_allocator(__a);
	struct gk20a_page_alloc *alloc = NULL;
	struct page_alloc_chunk *c;
	u64 aligned_len, pages;
	int i = 0;

	aligned_len = ALIGN(len, a->page_size);
	pages = aligned_len >> a->page_shift;

	alloc_lock(__a);

	alloc = __gk20a_alloc_pages_fixed(a, base, aligned_len);
	if (IS_ERR(alloc)) {
		alloc_unlock(__a);
		return 0;
	}

	__insert_page_alloc(a, alloc);
	alloc_unlock(__a);

	palloc_dbg(a, "Alloc [fixed] @ 0x%010llx + 0x%llx (%llu)\n",
		   alloc->base, aligned_len, pages);
	list_for_each_entry(c, &alloc->alloc_chunks, list_entry) {
		palloc_dbg(a, "  Chunk %2d: 0x%010llx + 0x%llx\n",
			   i++, c->base, c->length);
	}

	a->nr_fixed_allocs++;
	a->pages_alloced += pages;

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER)
		return alloc->base;
	else
		return (u64) (uintptr_t) alloc;
}

static void gk20a_page_free_fixed(struct gk20a_allocator *__a,
				  u64 base, u64 len)
{
	struct gk20a_page_allocator *a = page_allocator(__a);
	struct gk20a_page_alloc *alloc;

	alloc_lock(__a);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER) {
		alloc = __find_page_alloc(a, base);
		if (!alloc)
			goto done;
	} else {
		alloc = (struct gk20a_page_alloc *) (uintptr_t) base;
	}

	/*
	 * This works for the time being since the buddy allocator
	 * uses the same free function for both fixed and regular
	 * allocs. This would have to be updated if the underlying
	 * allocator were to change.
	 */
	__gk20a_free_pages(a, alloc);

	palloc_dbg(a, "Free  [fixed] 0x%010llx + 0x%llx\n",
		   alloc->base, alloc->length);
	a->nr_fixed_frees++;
	a->pages_freed += (alloc->length >> a->page_shift);

done:
	alloc_unlock(__a);
}

static void gk20a_page_allocator_destroy(struct gk20a_allocator *__a)
{
	struct gk20a_page_allocator *a = page_allocator(__a);

	alloc_lock(__a);
	kfree(a);
	__a->priv = NULL;
	alloc_unlock(__a);
}

static void gk20a_page_print_stats(struct gk20a_allocator *__a,
				   struct seq_file *s, int lock)
{
	struct gk20a_page_allocator *a = page_allocator(__a);

	if (lock)
		alloc_lock(__a);

	__alloc_pstat(s, __a, "Page allocator:\n");
	__alloc_pstat(s, __a, "  allocs         %lld\n", a->nr_allocs);
	__alloc_pstat(s, __a, "  frees          %lld\n", a->nr_frees);
	__alloc_pstat(s, __a, "  fixed_allocs   %lld\n", a->nr_fixed_allocs);
	__alloc_pstat(s, __a, "  fixed_frees    %lld\n", a->nr_fixed_frees);
	__alloc_pstat(s, __a, "  pages alloced  %lld\n", a->pages_alloced);
	__alloc_pstat(s, __a, "  pages freed    %lld\n", a->pages_freed);
	__alloc_pstat(s, __a, "\n");
	__alloc_pstat(s, __a, "Source alloc: %s\n",
		      a->source_allocator.name);

	gk20a_alloc_print_stats(&a->source_allocator, s, lock);

	if (lock)
		alloc_unlock(__a);
}

static const struct gk20a_allocator_ops page_ops = {
	.alloc		= gk20a_page_alloc,
	.free		= gk20a_page_free,

	.alloc_fixed	= gk20a_page_alloc_fixed,
	.free_fixed	= gk20a_page_free_fixed,

	.reserve_carveout	= gk20a_page_reserve_co,
	.release_carveout	= gk20a_page_release_co,

	.base		= gk20a_page_alloc_base,
	.length		= gk20a_page_alloc_length,
	.end		= gk20a_page_alloc_end,
	.inited		= gk20a_page_alloc_inited,
	.space		= gk20a_page_alloc_space,

	.fini		= gk20a_page_allocator_destroy,

	.print_stats	= gk20a_page_print_stats,
};

int gk20a_page_allocator_init(struct gk20a_allocator *__a,
				const char *name, u64 base, u64 length,
				u64 blk_size, u64 flags)
{
	struct gk20a_page_allocator *a;
	char buddy_name[sizeof(__a->name)];
	int err;

	mutex_lock(&meta_data_cache_lock);
	if (!page_alloc_cache)
		page_alloc_cache = KMEM_CACHE(gk20a_page_alloc, 0);
	if (!page_alloc_chunk_cache)
		page_alloc_chunk_cache = KMEM_CACHE(page_alloc_chunk, 0);
	mutex_unlock(&meta_data_cache_lock);

	if (!page_alloc_cache || !page_alloc_chunk_cache)
		return -ENOMEM;

	a = kzalloc(sizeof(struct gk20a_page_allocator), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	err = __gk20a_alloc_common_init(__a, name, a, false, &page_ops);
	if (err)
		goto fail;

	a->base = base;
	a->length = length;
	a->page_size = blk_size;
	a->page_shift = __ffs(blk_size);
	a->allocs = RB_ROOT;
	a->owner = __a;
	a->flags = flags;

	snprintf(buddy_name, sizeof(buddy_name), "%s-src", name);

	err = gk20a_buddy_allocator_init(&a->source_allocator, buddy_name, base,
					 length, blk_size, 0);
	if (err)
		goto fail;

	gk20a_init_alloc_debug(__a);
	palloc_dbg(a, "New allocator: type      page\n");
	palloc_dbg(a, "               base      0x%llx\n", a->base);
	palloc_dbg(a, "               size      0x%llx\n", a->length);
	palloc_dbg(a, "               page_size 0x%llx\n", a->page_size);
	palloc_dbg(a, "               flags     0x%llx\n", a->flags);

	return 0;

fail:
	kfree(a);
	return err;
}
