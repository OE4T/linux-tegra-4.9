/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bitops.h>
#include <nvgpu/allocator.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/log2.h>

#include "buddy_allocator_priv.h"

#define palloc_dbg(a, fmt, arg...)			\
	alloc_dbg(palloc_owner(a), fmt, ##arg)

/*
 * Since some Linux headers are still leaked into common code this is necessary
 * for some builds.
 */
#ifdef PAGE_SIZE
#undef PAGE_SIZE
#undef PAGE_ALIGN
#endif

/*
 * VIDMEM page size is 4k.
 */
#define PAGE_SIZE		0x1000
#define PAGE_ALIGN(addr)	((addr + (PAGE_SIZE - 1)) &		\
				 ((typeof(addr)) ~(PAGE_SIZE - 1)))

/*
 * Handle the book-keeping for these operations.
 */
static inline void add_slab_page_to_empty(struct page_alloc_slab *slab,
					  struct page_alloc_slab_page *page)
{
	BUG_ON(page->state != SP_NONE);
	nvgpu_list_add(&page->list_entry, &slab->empty);
	slab->nr_empty++;
	page->state = SP_EMPTY;
}
static inline void add_slab_page_to_partial(struct page_alloc_slab *slab,
					    struct page_alloc_slab_page *page)
{
	BUG_ON(page->state != SP_NONE);
	nvgpu_list_add(&page->list_entry, &slab->partial);
	slab->nr_partial++;
	page->state = SP_PARTIAL;
}
static inline void add_slab_page_to_full(struct page_alloc_slab *slab,
					 struct page_alloc_slab_page *page)
{
	BUG_ON(page->state != SP_NONE);
	nvgpu_list_add(&page->list_entry, &slab->full);
	slab->nr_full++;
	page->state = SP_FULL;
}

static inline void del_slab_page_from_empty(struct page_alloc_slab *slab,
					    struct page_alloc_slab_page *page)
{
	nvgpu_list_del(&page->list_entry);
	slab->nr_empty--;
	page->state = SP_NONE;
}
static inline void del_slab_page_from_partial(struct page_alloc_slab *slab,
					      struct page_alloc_slab_page *page)
{
	nvgpu_list_del(&page->list_entry);
	slab->nr_partial--;
	page->state = SP_NONE;
}
static inline void del_slab_page_from_full(struct page_alloc_slab *slab,
					   struct page_alloc_slab_page *page)
{
	nvgpu_list_del(&page->list_entry);
	slab->nr_full--;
	page->state = SP_NONE;
}

static u64 nvgpu_page_alloc_length(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_length(&va->source_allocator);
}

static u64 nvgpu_page_alloc_base(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_base(&va->source_allocator);
}

static int nvgpu_page_alloc_inited(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_initialized(&va->source_allocator);
}

static u64 nvgpu_page_alloc_end(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_end(&va->source_allocator);
}

static u64 nvgpu_page_alloc_space(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_space(&va->source_allocator);
}

static int nvgpu_page_reserve_co(struct nvgpu_allocator *a,
				 struct nvgpu_alloc_carveout *co)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_reserve_carveout(&va->source_allocator, co);
}

static void nvgpu_page_release_co(struct nvgpu_allocator *a,
				  struct nvgpu_alloc_carveout *co)
{
	struct nvgpu_page_allocator *va = a->priv;

	nvgpu_alloc_release_carveout(&va->source_allocator, co);
}

static void __nvgpu_free_pages(struct nvgpu_page_allocator *a,
			       struct nvgpu_page_alloc *alloc,
			       bool free_buddy_alloc)
{
	struct page_alloc_chunk *chunk;

	while (!nvgpu_list_empty(&alloc->alloc_chunks)) {
		chunk = nvgpu_list_first_entry(&alloc->alloc_chunks,
					 page_alloc_chunk,
					 list_entry);
		nvgpu_list_del(&chunk->list_entry);

		if (free_buddy_alloc)
			nvgpu_free(&a->source_allocator, chunk->base);
		nvgpu_kmem_cache_free(a->chunk_cache, chunk);
	}

	nvgpu_kmem_cache_free(a->alloc_cache, alloc);
}

static int __insert_page_alloc(struct nvgpu_page_allocator *a,
			       struct nvgpu_page_alloc *alloc)
{
	alloc->tree_entry.key_start = alloc->base;
	alloc->tree_entry.key_end = alloc->base + alloc->length;

	nvgpu_rbtree_insert(&alloc->tree_entry, &a->allocs);
	return 0;
}

static struct nvgpu_page_alloc *__find_page_alloc(
	struct nvgpu_page_allocator *a,
	u64 addr)
{
	struct nvgpu_page_alloc *alloc;
	struct nvgpu_rbtree_node *node = NULL;

	nvgpu_rbtree_search(addr, &node, a->allocs);
	if (!node)
		return NULL;

	alloc = nvgpu_page_alloc_from_rbtree_node(node);

	nvgpu_rbtree_unlink(node, &a->allocs);

	return alloc;
}

static struct page_alloc_slab_page *alloc_slab_page(
	struct nvgpu_page_allocator *a,
	struct page_alloc_slab *slab)
{
	struct page_alloc_slab_page *slab_page;

	slab_page = nvgpu_kmem_cache_alloc(a->slab_page_cache);
	if (!slab_page) {
		palloc_dbg(a, "OOM: unable to alloc slab_page struct!\n");
		return NULL;
	}

	memset(slab_page, 0, sizeof(*slab_page));

	slab_page->page_addr = nvgpu_alloc(&a->source_allocator, a->page_size);
	if (!slab_page->page_addr) {
		nvgpu_kmem_cache_free(a->slab_page_cache, slab_page);
		palloc_dbg(a, "OOM: vidmem is full!\n");
		return NULL;
	}

	nvgpu_init_list_node(&slab_page->list_entry);
	slab_page->slab_size = slab->slab_size;
	slab_page->nr_objects = (u32)a->page_size / slab->slab_size;
	slab_page->nr_objects_alloced = 0;
	slab_page->owner = slab;
	slab_page->state = SP_NONE;

	a->pages_alloced++;

	palloc_dbg(a, "Allocated new slab page @ 0x%012llx size=%u\n",
		   slab_page->page_addr, slab_page->slab_size);

	return slab_page;
}

static void free_slab_page(struct nvgpu_page_allocator *a,
			   struct page_alloc_slab_page *slab_page)
{
	palloc_dbg(a, "Freeing slab page @ 0x%012llx\n", slab_page->page_addr);

	BUG_ON((slab_page->state != SP_NONE && slab_page->state != SP_EMPTY) ||
	       slab_page->nr_objects_alloced != 0 ||
	       slab_page->bitmap != 0);

	nvgpu_free(&a->source_allocator, slab_page->page_addr);
	a->pages_freed++;

	nvgpu_kmem_cache_free(a->slab_page_cache, slab_page);
}

/*
 * This expects @alloc to have 1 empty page_alloc_chunk already added to the
 * alloc_chunks list.
 */
static int __do_slab_alloc(struct nvgpu_page_allocator *a,
			   struct page_alloc_slab *slab,
			   struct nvgpu_page_alloc *alloc)
{
	struct page_alloc_slab_page *slab_page = NULL;
	struct page_alloc_chunk *chunk;
	unsigned long offs;

	/*
	 * Check the partial and empty lists to see if we have some space
	 * readily available. Take the slab_page out of what ever list it
	 * was in since it may be put back into a different list later.
	 */
	if (!nvgpu_list_empty(&slab->partial)) {
		slab_page = nvgpu_list_first_entry(&slab->partial,
					     page_alloc_slab_page,
					     list_entry);
		del_slab_page_from_partial(slab, slab_page);
	} else if (!nvgpu_list_empty(&slab->empty)) {
		slab_page = nvgpu_list_first_entry(&slab->empty,
					     page_alloc_slab_page,
					     list_entry);
		del_slab_page_from_empty(slab, slab_page);
	}

	if (!slab_page) {
		slab_page = alloc_slab_page(a, slab);
		if (!slab_page)
			return -ENOMEM;
	}

	/*
	 * We now have a slab_page. Do the alloc.
	 */
	offs = bitmap_find_next_zero_area(&slab_page->bitmap,
					  slab_page->nr_objects,
					  0, 1, 0);
	if (offs >= slab_page->nr_objects) {
		WARN(1, "Empty/partial slab with no free objects?");

		/* Add the buggy page to the full list... This isn't ideal. */
		add_slab_page_to_full(slab, slab_page);
		return -ENOMEM;
	}

	bitmap_set(&slab_page->bitmap, offs, 1);
	slab_page->nr_objects_alloced++;

	if (slab_page->nr_objects_alloced < slab_page->nr_objects)
		add_slab_page_to_partial(slab, slab_page);
	else if (slab_page->nr_objects_alloced == slab_page->nr_objects)
		add_slab_page_to_full(slab, slab_page);
	else
		BUG(); /* Should be impossible to hit this. */

	/*
	 * Handle building the nvgpu_page_alloc struct. We expect one
	 * page_alloc_chunk to be present.
	 */
	alloc->slab_page = slab_page;
	alloc->nr_chunks = 1;
	alloc->length = slab_page->slab_size;
	alloc->base = slab_page->page_addr + (offs * slab_page->slab_size);

	chunk = nvgpu_list_first_entry(&alloc->alloc_chunks,
				page_alloc_chunk, list_entry);
	chunk->base = alloc->base;
	chunk->length = alloc->length;

	return 0;
}

/*
 * Allocate from a slab instead of directly from the page allocator.
 */
static struct nvgpu_page_alloc *__nvgpu_alloc_slab(
	struct nvgpu_page_allocator *a, u64 len)
{
	int err, slab_nr;
	struct page_alloc_slab *slab;
	struct nvgpu_page_alloc *alloc = NULL;
	struct page_alloc_chunk *chunk = NULL;

	/*
	 * Align the length to a page and then divide by the page size (4k for
	 * this code). ilog2() of that then gets us the correct slab to use.
	 */
	slab_nr = (int)ilog2(PAGE_ALIGN(len) >> 12);
	slab = &a->slabs[slab_nr];

	alloc = nvgpu_kmem_cache_alloc(a->alloc_cache);
	if (!alloc) {
		palloc_dbg(a, "OOM: could not alloc page_alloc struct!\n");
		goto fail;
	}
	chunk = nvgpu_kmem_cache_alloc(a->chunk_cache);
	if (!chunk) {
		palloc_dbg(a, "OOM: could not alloc alloc_chunk struct!\n");
		goto fail;
	}

	nvgpu_init_list_node(&alloc->alloc_chunks);
	nvgpu_list_add(&chunk->list_entry, &alloc->alloc_chunks);

	err = __do_slab_alloc(a, slab, alloc);
	if (err)
		goto fail;

	palloc_dbg(a, "Alloc 0x%04llx sr=%d id=0x%010llx [slab]\n",
		   len, slab_nr, alloc->base);
	a->nr_slab_allocs++;

	return alloc;

fail:
	if (alloc)
		nvgpu_kmem_cache_free(a->alloc_cache, alloc);
	if (chunk)
		nvgpu_kmem_cache_free(a->chunk_cache, chunk);
	return NULL;
}

static void __nvgpu_free_slab(struct nvgpu_page_allocator *a,
			      struct nvgpu_page_alloc *alloc)
{
	struct page_alloc_slab_page *slab_page = alloc->slab_page;
	struct page_alloc_slab *slab = slab_page->owner;
	enum slab_page_state new_state;
	int offs;

	offs = (u32)(alloc->base - slab_page->page_addr) / slab_page->slab_size;
	bitmap_clear(&slab_page->bitmap, offs, 1);

	slab_page->nr_objects_alloced--;

	if (slab_page->nr_objects_alloced == 0)
		new_state = SP_EMPTY;
	else
		new_state = SP_PARTIAL;

	/*
	 * Need to migrate the page to a different list.
	 */
	if (new_state != slab_page->state) {
		/* Delete - can't be in empty. */
		if (slab_page->state == SP_PARTIAL)
			del_slab_page_from_partial(slab, slab_page);
		else
			del_slab_page_from_full(slab, slab_page);

		/* And add. */
		if (new_state == SP_EMPTY) {
			if (nvgpu_list_empty(&slab->empty))
				add_slab_page_to_empty(slab, slab_page);
			else
				free_slab_page(a, slab_page);
		} else {
			add_slab_page_to_partial(slab, slab_page);
		}
	}

	/*
	 * Now handle the page_alloc.
	 */
	__nvgpu_free_pages(a, alloc, false);
	a->nr_slab_frees++;

	return;
}

/*
 * Allocate physical pages. Since the underlying allocator is a buddy allocator
 * the returned pages are always contiguous. However, since there could be
 * fragmentation in the space this allocator will collate smaller non-contiguous
 * allocations together if necessary.
 */
static struct nvgpu_page_alloc *__do_nvgpu_alloc_pages(
	struct nvgpu_page_allocator *a, u64 pages)
{
	struct nvgpu_page_alloc *alloc;
	struct page_alloc_chunk *c;
	u64 max_chunk_len = pages << a->page_shift;
	int i = 0;

	alloc = nvgpu_kmem_cache_alloc(a->alloc_cache);
	if (!alloc)
		goto fail;

	memset(alloc, 0, sizeof(*alloc));

	nvgpu_init_list_node(&alloc->alloc_chunks);
	alloc->length = pages << a->page_shift;

	while (pages) {
		u64 chunk_addr = 0;
		u64 chunk_pages = (u64)1 << __fls(pages);
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
			chunk_addr = nvgpu_alloc(&a->source_allocator,
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

		c = nvgpu_kmem_cache_alloc(a->chunk_cache);
		if (!c) {
			nvgpu_free(&a->source_allocator, chunk_addr);
			goto fail_cleanup;
		}

		pages -= chunk_pages;

		c->base = chunk_addr;
		c->length = chunk_len;
		nvgpu_list_add(&c->list_entry, &alloc->alloc_chunks);

		i++;
	}

	alloc->nr_chunks = i;
	c = nvgpu_list_first_entry(&alloc->alloc_chunks,
				page_alloc_chunk, list_entry);
	alloc->base = c->base;

	return alloc;

fail_cleanup:
	while (!nvgpu_list_empty(&alloc->alloc_chunks)) {
		c = nvgpu_list_first_entry(&alloc->alloc_chunks,
				     page_alloc_chunk, list_entry);
		nvgpu_list_del(&c->list_entry);
		nvgpu_free(&a->source_allocator, c->base);
		nvgpu_kmem_cache_free(a->chunk_cache, c);
	}
	nvgpu_kmem_cache_free(a->alloc_cache, alloc);
fail:
	return NULL;
}

static struct nvgpu_page_alloc *__nvgpu_alloc_pages(
	struct nvgpu_page_allocator *a, u64 len)
{
	struct nvgpu_page_alloc *alloc = NULL;
	struct page_alloc_chunk *c;
	u64 pages;
	int i = 0;

	pages = ALIGN(len, a->page_size) >> a->page_shift;

	alloc = __do_nvgpu_alloc_pages(a, pages);
	if (!alloc) {
		palloc_dbg(a, "Alloc 0x%llx (%llu) (failed)\n",
			   pages << a->page_shift, pages);
		return NULL;
	}

	palloc_dbg(a, "Alloc 0x%llx (%llu) id=0x%010llx\n",
		   pages << a->page_shift, pages, alloc->base);
	nvgpu_list_for_each_entry(c, &alloc->alloc_chunks,
				  page_alloc_chunk, list_entry) {
		palloc_dbg(a, "  Chunk %2d: 0x%010llx + 0x%llx\n",
			   i++, c->base, c->length);
	}

	return alloc;
}

/*
 * Allocate enough pages to satisfy @len. Page size is determined at
 * initialization of the allocator.
 *
 * The return is actually a pointer to a struct nvgpu_page_alloc pointer. This
 * is because it doesn't make a lot of sense to return the address of the first
 * page in the list of pages (since they could be discontiguous). This has
 * precedent in the dma_alloc APIs, though, it's really just an annoying
 * artifact of the fact that the nvgpu_alloc() API requires a u64 return type.
 */
static u64 nvgpu_page_alloc(struct nvgpu_allocator *__a, u64 len)
{
	struct nvgpu_page_allocator *a = page_allocator(__a);
	struct nvgpu_page_alloc *alloc = NULL;
	u64 real_len;

	/*
	 * If we want contig pages we have to round up to a power of two. It's
	 * easier to do that here than in the buddy allocator.
	 */
	real_len = a->flags & GPU_ALLOC_FORCE_CONTIG ?
		roundup_pow_of_two(len) : len;

	alloc_lock(__a);
	if (a->flags & GPU_ALLOC_4K_VIDMEM_PAGES &&
	    real_len <= (a->page_size / 2))
		alloc = __nvgpu_alloc_slab(a, real_len);
	else
		alloc = __nvgpu_alloc_pages(a, real_len);

	if (!alloc) {
		alloc_unlock(__a);
		return 0;
	}

	__insert_page_alloc(a, alloc);

	a->nr_allocs++;
	if (real_len > a->page_size / 2)
		a->pages_alloced += alloc->length >> a->page_shift;
	alloc_unlock(__a);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER)
		return alloc->base;
	else
		return (u64) (uintptr_t) alloc;
}

/*
 * Note: this will remove the nvgpu_page_alloc struct from the RB tree
 * if it's found.
 */
static void nvgpu_page_free(struct nvgpu_allocator *__a, u64 base)
{
	struct nvgpu_page_allocator *a = page_allocator(__a);
	struct nvgpu_page_alloc *alloc;

	alloc_lock(__a);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER)
		alloc = __find_page_alloc(a, base);
	else
		alloc = __find_page_alloc(a,
			((struct nvgpu_page_alloc *)(uintptr_t)base)->base);

	if (!alloc) {
		palloc_dbg(a, "Hrm, found no alloc?\n");
		goto done;
	}

	a->nr_frees++;

	palloc_dbg(a, "Free  0x%llx id=0x%010llx\n",
		   alloc->length, alloc->base);

	/*
	 * Frees *alloc.
	 */
	if (alloc->slab_page) {
		__nvgpu_free_slab(a, alloc);
	} else {
		a->pages_freed += (alloc->length >> a->page_shift);
		__nvgpu_free_pages(a, alloc, true);
	}

done:
	alloc_unlock(__a);
}

static struct nvgpu_page_alloc *__nvgpu_alloc_pages_fixed(
	struct nvgpu_page_allocator *a, u64 base, u64 length, u32 unused)
{
	struct nvgpu_page_alloc *alloc;
	struct page_alloc_chunk *c;

	alloc = nvgpu_kmem_cache_alloc(a->alloc_cache);
	c = nvgpu_kmem_cache_alloc(a->chunk_cache);
	if (!alloc || !c)
		goto fail;

	alloc->base = nvgpu_alloc_fixed(&a->source_allocator, base, length, 0);
	if (!alloc->base) {
		WARN(1, "nvgpu: failed to fixed alloc pages @ 0x%010llx", base);
		goto fail;
	}

	alloc->nr_chunks = 1;
	alloc->length = length;
	nvgpu_init_list_node(&alloc->alloc_chunks);

	c->base = alloc->base;
	c->length = length;
	nvgpu_list_add(&c->list_entry, &alloc->alloc_chunks);

	return alloc;

fail:
	if (c)
		nvgpu_kmem_cache_free(a->chunk_cache, c);
	if (alloc)
		nvgpu_kmem_cache_free(a->alloc_cache, alloc);
	return NULL;
}

/*
 * @page_size is ignored.
 */
static u64 nvgpu_page_alloc_fixed(struct nvgpu_allocator *__a,
				  u64 base, u64 len, u32 page_size)
{
	struct nvgpu_page_allocator *a = page_allocator(__a);
	struct nvgpu_page_alloc *alloc = NULL;
	struct page_alloc_chunk *c;
	u64 aligned_len, pages;
	int i = 0;

	aligned_len = ALIGN(len, a->page_size);
	pages = aligned_len >> a->page_shift;

	alloc_lock(__a);

	alloc = __nvgpu_alloc_pages_fixed(a, base, aligned_len, 0);
	if (!alloc) {
		alloc_unlock(__a);
		return 0;
	}

	__insert_page_alloc(a, alloc);
	alloc_unlock(__a);

	palloc_dbg(a, "Alloc [fixed] @ 0x%010llx + 0x%llx (%llu)\n",
		   alloc->base, aligned_len, pages);
	nvgpu_list_for_each_entry(c, &alloc->alloc_chunks,
				  page_alloc_chunk, list_entry) {
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

static void nvgpu_page_free_fixed(struct nvgpu_allocator *__a,
				  u64 base, u64 len)
{
	struct nvgpu_page_allocator *a = page_allocator(__a);
	struct nvgpu_page_alloc *alloc;

	alloc_lock(__a);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER) {
		alloc = __find_page_alloc(a, base);
		if (!alloc)
			goto done;
	} else {
		alloc = (struct nvgpu_page_alloc *) (uintptr_t) base;
	}

	palloc_dbg(a, "Free  [fixed] 0x%010llx + 0x%llx\n",
		   alloc->base, alloc->length);

	a->nr_fixed_frees++;
	a->pages_freed += (alloc->length >> a->page_shift);

	/*
	 * This works for the time being since the buddy allocator
	 * uses the same free function for both fixed and regular
	 * allocs. This would have to be updated if the underlying
	 * allocator were to change.
	 */
	__nvgpu_free_pages(a, alloc, true);

done:
	alloc_unlock(__a);
}

static void nvgpu_page_allocator_destroy(struct nvgpu_allocator *__a)
{
	struct nvgpu_page_allocator *a = page_allocator(__a);

	alloc_lock(__a);
	nvgpu_kfree(nvgpu_alloc_to_gpu(__a), a);
	__a->priv = NULL;
	alloc_unlock(__a);
}

#ifdef CONFIG_DEBUG_FS
static void nvgpu_page_print_stats(struct nvgpu_allocator *__a,
				   struct seq_file *s, int lock)
{
	struct nvgpu_page_allocator *a = page_allocator(__a);
	int i;

	if (lock)
		alloc_lock(__a);

	__alloc_pstat(s, __a, "Page allocator:\n");
	__alloc_pstat(s, __a, "  allocs         %lld\n", a->nr_allocs);
	__alloc_pstat(s, __a, "  frees          %lld\n", a->nr_frees);
	__alloc_pstat(s, __a, "  fixed_allocs   %lld\n", a->nr_fixed_allocs);
	__alloc_pstat(s, __a, "  fixed_frees    %lld\n", a->nr_fixed_frees);
	__alloc_pstat(s, __a, "  slab_allocs    %lld\n", a->nr_slab_allocs);
	__alloc_pstat(s, __a, "  slab_frees     %lld\n", a->nr_slab_frees);
	__alloc_pstat(s, __a, "  pages alloced  %lld\n", a->pages_alloced);
	__alloc_pstat(s, __a, "  pages freed    %lld\n", a->pages_freed);
	__alloc_pstat(s, __a, "\n");

	__alloc_pstat(s, __a, "Page size:       %lld KB\n",
		      a->page_size >> 10);
	__alloc_pstat(s, __a, "Total pages:     %lld (%lld MB)\n",
		      a->length / a->page_size,
		      a->length >> 20);
	__alloc_pstat(s, __a, "Available pages: %lld (%lld MB)\n",
		      nvgpu_alloc_space(&a->source_allocator) / a->page_size,
		      nvgpu_alloc_space(&a->source_allocator) >> 20);
	__alloc_pstat(s, __a, "\n");

	/*
	 * Slab info.
	 */
	if (a->flags & GPU_ALLOC_4K_VIDMEM_PAGES) {
		__alloc_pstat(s, __a, "Slabs:\n");
		__alloc_pstat(s, __a, "  size      empty     partial   full\n");
		__alloc_pstat(s, __a, "  ----      -----     -------   ----\n");

		for (i = 0; i < a->nr_slabs; i++) {
			struct page_alloc_slab *slab = &a->slabs[i];

			__alloc_pstat(s, __a, "  %-9u %-9d %-9u %u\n",
				      slab->slab_size,
				      slab->nr_empty, slab->nr_partial,
				      slab->nr_full);
		}
		__alloc_pstat(s, __a, "\n");
	}

	__alloc_pstat(s, __a, "Source alloc: %s\n",
		      a->source_allocator.name);
	nvgpu_alloc_print_stats(&a->source_allocator, s, lock);

	if (lock)
		alloc_unlock(__a);
}
#endif

static const struct nvgpu_allocator_ops page_ops = {
	.alloc		= nvgpu_page_alloc,
	.free		= nvgpu_page_free,

	.alloc_fixed	= nvgpu_page_alloc_fixed,
	.free_fixed	= nvgpu_page_free_fixed,

	.reserve_carveout	= nvgpu_page_reserve_co,
	.release_carveout	= nvgpu_page_release_co,

	.base		= nvgpu_page_alloc_base,
	.length		= nvgpu_page_alloc_length,
	.end		= nvgpu_page_alloc_end,
	.inited		= nvgpu_page_alloc_inited,
	.space		= nvgpu_page_alloc_space,

	.fini		= nvgpu_page_allocator_destroy,

#ifdef CONFIG_DEBUG_FS
	.print_stats	= nvgpu_page_print_stats,
#endif
};

/*
 * nr_slabs is computed as follows: divide page_size by 4096 to get number of
 * 4k pages in page_size. Then take the base 2 log of that to get number of
 * slabs. For 64k page_size that works on like:
 *
 *   1024*64 / 1024*4 = 16
 *   ilog2(16) = 4
 *
 * That gives buckets of 1, 2, 4, and 8 pages (i.e 4k, 8k, 16k, 32k).
 */
static int nvgpu_page_alloc_init_slabs(struct nvgpu_page_allocator *a)
{
	size_t nr_slabs = ilog2(a->page_size >> 12);
	unsigned int i;

	a->slabs = nvgpu_kcalloc(nvgpu_alloc_to_gpu(a->owner),
				 nr_slabs,
				 sizeof(struct page_alloc_slab));
	if (!a->slabs)
		return -ENOMEM;
	a->nr_slabs = nr_slabs;

	for (i = 0; i < nr_slabs; i++) {
		struct page_alloc_slab *slab = &a->slabs[i];

		slab->slab_size = SZ_4K * (1 << i);
		nvgpu_init_list_node(&slab->empty);
		nvgpu_init_list_node(&slab->partial);
		nvgpu_init_list_node(&slab->full);
		slab->nr_empty = 0;
		slab->nr_partial = 0;
		slab->nr_full = 0;
	}

	return 0;
}

int nvgpu_page_allocator_init(struct gk20a *g, struct nvgpu_allocator *__a,
			      const char *name, u64 base, u64 length,
			      u64 blk_size, u64 flags)
{
	struct nvgpu_page_allocator *a;
	char buddy_name[sizeof(__a->name)];
	int err;

	if (blk_size < SZ_4K)
		return -EINVAL;

	a = nvgpu_kzalloc(g, sizeof(struct nvgpu_page_allocator));
	if (!a)
		return -ENOMEM;

	err = __nvgpu_alloc_common_init(__a, g, name, a, false, &page_ops);
	if (err)
		goto fail;

	a->alloc_cache = nvgpu_kmem_cache_create(g,
					sizeof(struct nvgpu_page_alloc));
	a->chunk_cache = nvgpu_kmem_cache_create(g,
					sizeof(struct page_alloc_chunk));
	a->slab_page_cache = nvgpu_kmem_cache_create(g,
					sizeof(struct page_alloc_slab_page));
	if (!a->alloc_cache || !a->chunk_cache || !a->slab_page_cache) {
		err = -ENOMEM;
		goto fail;
	}

	a->base = base;
	a->length = length;
	a->page_size = blk_size;
	a->page_shift = __ffs(blk_size);
	a->allocs = NULL;
	a->owner = __a;
	a->flags = flags;

	if (flags & GPU_ALLOC_4K_VIDMEM_PAGES && blk_size > SZ_4K) {
		err = nvgpu_page_alloc_init_slabs(a);
		if (err)
			goto fail;
	}

	snprintf(buddy_name, sizeof(buddy_name), "%s-src", name);

	err = nvgpu_buddy_allocator_init(g, &a->source_allocator, buddy_name,
					 base, length, blk_size, 0);
	if (err)
		goto fail;

#ifdef CONFIG_DEBUG_FS
	nvgpu_init_alloc_debug(g, __a);
#endif
	palloc_dbg(a, "New allocator: type      page\n");
	palloc_dbg(a, "               base      0x%llx\n", a->base);
	palloc_dbg(a, "               size      0x%llx\n", a->length);
	palloc_dbg(a, "               page_size 0x%llx\n", a->page_size);
	palloc_dbg(a, "               flags     0x%llx\n", a->flags);
	palloc_dbg(a, "               slabs:    %d\n", a->nr_slabs);

	return 0;

fail:
	if (a->alloc_cache)
		nvgpu_kmem_cache_destroy(a->alloc_cache);
	if (a->chunk_cache)
		nvgpu_kmem_cache_destroy(a->chunk_cache);
	if (a->slab_page_cache)
		nvgpu_kmem_cache_destroy(a->slab_page_cache);
	nvgpu_kfree(g, a);
	return err;
}
