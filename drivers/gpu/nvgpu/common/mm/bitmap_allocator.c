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
#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/barrier.h>

#include "bitmap_allocator_priv.h"

static u64 nvgpu_bitmap_alloc_length(struct nvgpu_allocator *a)
{
	struct nvgpu_bitmap_allocator *ba = a->priv;

	return ba->length;
}

static u64 nvgpu_bitmap_alloc_base(struct nvgpu_allocator *a)
{
	struct nvgpu_bitmap_allocator *ba = a->priv;

	return ba->base;
}

static int nvgpu_bitmap_alloc_inited(struct nvgpu_allocator *a)
{
	struct nvgpu_bitmap_allocator *ba = a->priv;
	int inited = ba->inited;

	nvgpu_smp_rmb();
	return inited;
}

static u64 nvgpu_bitmap_alloc_end(struct nvgpu_allocator *a)
{
	struct nvgpu_bitmap_allocator *ba = a->priv;

	return ba->base + ba->length;
}

/*
 * @page_size is ignored.
 */
static u64 nvgpu_bitmap_alloc_fixed(struct nvgpu_allocator *__a,
				    u64 base, u64 len, u32 page_size)
{
	struct nvgpu_bitmap_allocator *a = bitmap_allocator(__a);
	u64 blks, offs, ret;

	/* Compute the bit offset and make sure it's aligned to a block.  */
	offs = base >> a->blk_shift;
	if (offs * a->blk_size != base)
		return 0;

	offs -= a->bit_offs;

	blks = len >> a->blk_shift;
	if (blks * a->blk_size != len)
		blks++;

	alloc_lock(__a);

	/* Check if the space requested is already occupied. */
	ret = bitmap_find_next_zero_area(a->bitmap, a->num_bits, offs, blks, 0);
	if (ret != offs)
		goto fail;

	bitmap_set(a->bitmap, offs, blks);

	a->bytes_alloced += blks * a->blk_size;
	a->nr_fixed_allocs++;
	alloc_unlock(__a);

	alloc_dbg(__a, "Alloc-fixed 0x%-10llx 0x%-5llx [bits=0x%llx (%llu)]\n",
		  base, len, blks, blks);
	return base;

fail:
	alloc_unlock(__a);
	alloc_dbg(__a, "Alloc-fixed failed! (0x%llx)\n", base);
	return 0;
}

/*
 * Two possibilities for this function: either we are freeing a fixed allocation
 * or we are freeing a regular alloc but with GPU_ALLOC_NO_ALLOC_PAGE defined.
 *
 * Note: this function won't do much error checking. Thus you could really
 * confuse the allocator if you misuse this function.
 */
static void nvgpu_bitmap_free_fixed(struct nvgpu_allocator *__a,
				    u64 base, u64 len)
{
	struct nvgpu_bitmap_allocator *a = bitmap_allocator(__a);
	u64 blks, offs;

	offs = base >> a->blk_shift;
	if (WARN_ON(offs * a->blk_size != base))
		return;

	offs -= a->bit_offs;

	blks = len >> a->blk_shift;
	if (blks * a->blk_size != len)
		blks++;

	alloc_lock(__a);
	bitmap_clear(a->bitmap, offs, blks);
	a->bytes_freed += blks * a->blk_size;
	alloc_unlock(__a);

	alloc_dbg(__a, "Free-fixed 0x%-10llx 0x%-5llx [bits=0x%llx (%llu)]\n",
		  base, len, blks, blks);
}

/*
 * Add the passed alloc to the tree of stored allocations.
 */
static void insert_alloc_metadata(struct nvgpu_bitmap_allocator *a,
				  struct nvgpu_bitmap_alloc *alloc)
{
	alloc->alloc_entry.key_start = alloc->base;
	alloc->alloc_entry.key_end = alloc->base + alloc->length;

	nvgpu_rbtree_insert(&alloc->alloc_entry, &a->allocs);
}

/*
 * Find and remove meta-data from the outstanding allocations.
 */
static struct nvgpu_bitmap_alloc *find_alloc_metadata(
	struct nvgpu_bitmap_allocator *a, u64 addr)
{
	struct nvgpu_bitmap_alloc *alloc;
	struct nvgpu_rbtree_node *node = NULL;

	nvgpu_rbtree_search(addr, &node, a->allocs);
	if (!node)
		return NULL;

	alloc = nvgpu_bitmap_alloc_from_rbtree_node(node);

	nvgpu_rbtree_unlink(node, &a->allocs);

	return alloc;
}

/*
 * Tree of alloc meta data stores the address of the alloc not the bit offset.
 */
static int __nvgpu_bitmap_store_alloc(struct nvgpu_bitmap_allocator *a,
				      u64 addr, u64 len)
{
	struct nvgpu_bitmap_alloc *alloc =
		nvgpu_kmem_cache_alloc(a->meta_data_cache);

	if (!alloc)
		return -ENOMEM;

	alloc->base = addr;
	alloc->length = len;

	insert_alloc_metadata(a, alloc);

	return 0;
}

/*
 * @len is in bytes. This routine will figure out the right number of bits to
 * actually allocate. The return is the address in bytes as well.
 */
static u64 nvgpu_bitmap_alloc(struct nvgpu_allocator *__a, u64 len)
{
	u64 blks, addr;
	unsigned long offs, adjusted_offs, limit;
	struct nvgpu_bitmap_allocator *a = bitmap_allocator(__a);

	blks = len >> a->blk_shift;

	if (blks * a->blk_size != len)
		blks++;

	alloc_lock(__a);

	/*
	 * First look from next_blk and onwards...
	 */
	offs = bitmap_find_next_zero_area(a->bitmap, a->num_bits,
					  a->next_blk, blks, 0);
	if (offs >= a->num_bits) {
		/*
		 * If that didn't work try the remaining area. Since there can
		 * be available space that spans across a->next_blk we need to
		 * search up to the first set bit after that.
		 */
		limit = find_next_bit(a->bitmap, a->num_bits, a->next_blk);
		offs = bitmap_find_next_zero_area(a->bitmap, limit,
						  0, blks, 0);
		if (offs >= a->next_blk)
			goto fail;
	}

	bitmap_set(a->bitmap, offs, blks);
	a->next_blk = offs + blks;

	adjusted_offs = offs + a->bit_offs;
	addr = ((u64)adjusted_offs) * a->blk_size;

	/*
	 * Only do meta-data storage if we are allowed to allocate storage for
	 * that meta-data. The issue with using malloc and friends is that
	 * in latency and success critical paths an alloc_page() call can either
	 * sleep for potentially a long time or fail. Since we might not want
	 * either of these possibilities assume that the caller will keep what
	 * data it needs around to successfully free this allocation.
	 */
	if (!(a->flags & GPU_ALLOC_NO_ALLOC_PAGE) &&
	    __nvgpu_bitmap_store_alloc(a, addr, blks * a->blk_size))
		goto fail_reset_bitmap;

	alloc_dbg(__a, "Alloc 0x%-10llx 0x%-5llx [bits=0x%llx (%llu)]\n",
		  addr, len, blks, blks);

	a->nr_allocs++;
	a->bytes_alloced += (blks * a->blk_size);
	alloc_unlock(__a);

	return addr;

fail_reset_bitmap:
	bitmap_clear(a->bitmap, offs, blks);
fail:
	a->next_blk = 0;
	alloc_unlock(__a);
	alloc_dbg(__a, "Alloc failed!\n");
	return 0;
}

static void nvgpu_bitmap_free(struct nvgpu_allocator *__a, u64 addr)
{
	struct nvgpu_bitmap_allocator *a = bitmap_allocator(__a);
	struct nvgpu_bitmap_alloc *alloc = NULL;
	u64 offs, adjusted_offs, blks;

	alloc_lock(__a);

	if (a->flags & GPU_ALLOC_NO_ALLOC_PAGE) {
		WARN(1, "Using wrong free for NO_ALLOC_PAGE bitmap allocator");
		goto done;
	}

	alloc = find_alloc_metadata(a, addr);
	if (!alloc)
		goto done;

	/*
	 * Address comes from adjusted offset (i.e the bit offset with
	 * a->bit_offs added. So start with that and then work out the real
	 * offs into the bitmap.
	 */
	adjusted_offs = addr >> a->blk_shift;
	offs = adjusted_offs - a->bit_offs;
	blks = alloc->length >> a->blk_shift;

	bitmap_clear(a->bitmap, offs, blks);
	alloc_dbg(__a, "Free  0x%-10llx\n", addr);

	a->bytes_freed += alloc->length;

done:
	if (a->meta_data_cache && alloc)
		nvgpu_kmem_cache_free(a->meta_data_cache, alloc);
	alloc_unlock(__a);
}

static void nvgpu_bitmap_alloc_destroy(struct nvgpu_allocator *__a)
{
	struct nvgpu_bitmap_allocator *a = bitmap_allocator(__a);
	struct nvgpu_bitmap_alloc *alloc;
	struct nvgpu_rbtree_node *node;

	/*
	 * Kill any outstanding allocations.
	 */
	nvgpu_rbtree_enum_start(0, &node, a->allocs);
	while (node) {
		alloc = nvgpu_bitmap_alloc_from_rbtree_node(node);

		nvgpu_rbtree_unlink(node, &a->allocs);
		nvgpu_kmem_cache_free(a->meta_data_cache, alloc);

		nvgpu_rbtree_enum_start(0, &node, a->allocs);
	}

	nvgpu_kmem_cache_destroy(a->meta_data_cache);
	nvgpu_kfree(nvgpu_alloc_to_gpu(__a), a->bitmap);
	nvgpu_kfree(nvgpu_alloc_to_gpu(__a), a);
}

#ifdef CONFIG_DEBUG_FS
static void nvgpu_bitmap_print_stats(struct nvgpu_allocator *__a,
				     struct seq_file *s, int lock)
{
	struct nvgpu_bitmap_allocator *a = bitmap_allocator(__a);

	__alloc_pstat(s, __a, "Bitmap allocator params:\n");
	__alloc_pstat(s, __a, "  start = 0x%llx\n", a->base);
	__alloc_pstat(s, __a, "  end   = 0x%llx\n", a->base + a->length);
	__alloc_pstat(s, __a, "  blks  = 0x%llx\n", a->num_bits);

	/* Actual stats. */
	__alloc_pstat(s, __a, "Stats:\n");
	__alloc_pstat(s, __a, "  Number allocs = 0x%llx\n", a->nr_allocs);
	__alloc_pstat(s, __a, "  Number fixed  = 0x%llx\n", a->nr_fixed_allocs);
	__alloc_pstat(s, __a, "  Bytes alloced = 0x%llx\n", a->bytes_alloced);
	__alloc_pstat(s, __a, "  Bytes freed   = 0x%llx\n", a->bytes_freed);
	__alloc_pstat(s, __a, "  Outstanding   = 0x%llx\n",
		      a->bytes_alloced - a->bytes_freed);
}
#endif

static const struct nvgpu_allocator_ops bitmap_ops = {
	.alloc		= nvgpu_bitmap_alloc,
	.free		= nvgpu_bitmap_free,

	.alloc_fixed	= nvgpu_bitmap_alloc_fixed,
	.free_fixed	= nvgpu_bitmap_free_fixed,

	.base		= nvgpu_bitmap_alloc_base,
	.length		= nvgpu_bitmap_alloc_length,
	.end		= nvgpu_bitmap_alloc_end,
	.inited		= nvgpu_bitmap_alloc_inited,

	.fini		= nvgpu_bitmap_alloc_destroy,

#ifdef CONFIG_DEBUG_FS
	.print_stats	= nvgpu_bitmap_print_stats,
#endif
};


int nvgpu_bitmap_allocator_init(struct gk20a *g, struct nvgpu_allocator *__a,
				const char *name, u64 base, u64 length,
				u64 blk_size, u64 flags)
{
	int err;
	struct nvgpu_bitmap_allocator *a;

	if (WARN_ON(blk_size & (blk_size - 1)))
		return -EINVAL;

	/*
	 * blk_size must be a power-of-2; base length also need to be aligned
	 * to blk_size.
	 */
	if (blk_size & (blk_size - 1) ||
	    base & (blk_size - 1) || length & (blk_size - 1))
		return -EINVAL;

	if (base == 0) {
		base = blk_size;
		length -= blk_size;
	}

	a = nvgpu_kzalloc(g, sizeof(struct nvgpu_bitmap_allocator));
	if (!a)
		return -ENOMEM;

	err = __nvgpu_alloc_common_init(__a, g, name, a, false, &bitmap_ops);
	if (err)
		goto fail;

	if (!(flags & GPU_ALLOC_NO_ALLOC_PAGE)) {
		a->meta_data_cache = nvgpu_kmem_cache_create(g,
					sizeof(struct nvgpu_bitmap_alloc));
		if (!a->meta_data_cache) {
			err = -ENOMEM;
			goto fail;
		}
	}

	a->base = base;
	a->length = length;
	a->blk_size = blk_size;
	a->blk_shift = __ffs(a->blk_size);
	a->num_bits = length >> a->blk_shift;
	a->bit_offs = a->base >> a->blk_shift;
	a->flags = flags;
	a->allocs = NULL;

	a->bitmap = nvgpu_kcalloc(g, BITS_TO_LONGS(a->num_bits),
				  sizeof(*a->bitmap));
	if (!a->bitmap) {
		err = -ENOMEM;
		goto fail;
	}

	nvgpu_smp_wmb();
	a->inited = true;

#ifdef CONFIG_DEBUG_FS
	nvgpu_init_alloc_debug(g, __a);
#endif
	alloc_dbg(__a, "New allocator: type      bitmap\n");
	alloc_dbg(__a, "               base      0x%llx\n", a->base);
	alloc_dbg(__a, "               bit_offs  0x%llx\n", a->bit_offs);
	alloc_dbg(__a, "               size      0x%llx\n", a->length);
	alloc_dbg(__a, "               blk_size  0x%llx\n", a->blk_size);
	alloc_dbg(__a, "               flags     0x%llx\n", a->flags);

	return 0;

fail:
	if (a->meta_data_cache)
		nvgpu_kmem_cache_destroy(a->meta_data_cache);
	nvgpu_kfree(g, a);
	return err;
}
