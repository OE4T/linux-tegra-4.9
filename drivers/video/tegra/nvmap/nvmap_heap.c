/*
 * drivers/video/tegra/nvmap/nvmap_heap.c
 *
 * GPU heap allocator.
 *
 * Copyright (c) 2011-2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/bug.h>
#include <linux/stat.h>

#include <linux/nvmap.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>

#include "nvmap_priv.h"
#include "nvmap_heap.h"

/*
 * "carveouts" are platform-defined regions of physically contiguous memory
 * which are not managed by the OS. A platform may specify multiple carveouts,
 * for either small special-purpose memory regions (like IRAM on Tegra SoCs)
 * or reserved regions of main system memory.
 *
 * The carveout allocator returns allocations which are physically contiguous.
 */

static struct kmem_cache *heap_block_cache;

struct list_block {
	struct nvmap_heap_block block;
	struct list_head all_list;
	unsigned int mem_prot;
	phys_addr_t orig_addr;
	size_t size;
	size_t align;
	struct nvmap_heap *heap;
	struct list_head free_list;
};

struct nvmap_heap {
	struct list_head all_list;
	struct mutex lock;
	const char *name;
	void *arg;
	/* heap base */
	phys_addr_t base;
	/* heap size */
	size_t len;
	struct device *cma_dev;
	struct device *dma_dev;
	struct device dev;
};

void nvmap_heap_debugfs_init(struct dentry *heap_root, struct nvmap_heap *heap)
{
	if (sizeof(heap->base) == sizeof(u64))
		debugfs_create_x64("base", S_IRUGO,
			heap_root, (u64 *)&heap->base);
	else
		debugfs_create_x32("base", S_IRUGO,
			heap_root, (u32 *)&heap->base);
	if (sizeof(heap->len) == sizeof(u64))
		debugfs_create_x64("size", S_IRUGO,
			heap_root, (u64 *)&heap->len);
	else
		debugfs_create_x32("size", S_IRUGO,
			heap_root, (u32 *)&heap->len);
}

static phys_addr_t nvmap_alloc_mem(struct nvmap_heap *h, size_t len)
{
	phys_addr_t pa;
	DEFINE_DMA_ATTRS(attrs);
	struct device *dev = h->dma_dev;

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, &attrs);

	(void)dma_alloc_attrs(dev, len, &pa,
		DMA_MEMORY_NOMAP, &attrs);
	if (!dma_mapping_error(dev, pa))
		dev_dbg(dev, "Allocated addr (%pa) len(%zu)\n",
			&pa, len);

	return pa;
}

static void nvmap_free_mem(struct nvmap_heap *h, phys_addr_t base,
				size_t len)
{
	struct device *dev = h->dma_dev;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, &attrs);
	dev_dbg(dev,
		"Free base (%pa) size (%zu)\n", &base, len);
	dma_free_attrs(dev, len,
		(void *)(uintptr_t)base, (dma_addr_t)base, &attrs);
}

/*
 * base_max limits position of allocated chunk in memory.
 * if base_max is 0 then there is no such limitation.
 */
static struct nvmap_heap_block *do_heap_alloc(struct nvmap_heap *heap,
					      size_t len, size_t align,
					      unsigned int mem_prot,
					      phys_addr_t base_max)
{
	struct list_block *heap_block = NULL;
	dma_addr_t dev_base;
	struct device *dev = heap->dma_dev;

	/* since pages are only mappable with one cache attribute,
	 * and most allocations from carveout heaps are DMA coherent
	 * (i.e., non-cacheable), round cacheable allocations up to
	 * a page boundary to ensure that the physical pages will
	 * only be mapped one way. */
	if (mem_prot == NVMAP_HANDLE_CACHEABLE ||
	    mem_prot == NVMAP_HANDLE_INNER_CACHEABLE) {
		align = max_t(size_t, align, PAGE_SIZE);
		len = PAGE_ALIGN(len);
	}

	heap_block = kmem_cache_zalloc(heap_block_cache, GFP_KERNEL);
	if (!heap_block) {
		dev_err(dev, "%s: failed to alloc heap block %s\n",
			__func__, dev_name(dev));
		goto fail_heap_block_alloc;
	}

	dev_base = nvmap_alloc_mem(heap, len);
	if (dma_mapping_error(dev, dev_base)) {
		dev_err(dev, "failed to alloc mem of size (%zu)\n",
			len);
		goto fail_dma_alloc;
	}

	heap_block->block.base = dev_base;
	heap_block->orig_addr = dev_base;
	heap_block->size = len;

	list_add_tail(&heap_block->all_list, &heap->all_list);
	heap_block->heap = heap;
	heap_block->mem_prot = mem_prot;
	heap_block->align = align;
	return &heap_block->block;

fail_dma_alloc:
	kmem_cache_free(heap_block_cache, heap_block);
fail_heap_block_alloc:
	return NULL;
}

static struct list_block *do_heap_free(struct nvmap_heap_block *block)
{
	struct list_block *b = container_of(block, struct list_block, block);
	struct nvmap_heap *heap = b->heap;

	list_del(&b->all_list);

	nvmap_free_mem(heap, block->base, b->size);
	kmem_cache_free(heap_block_cache, b);

	return b;
}

/* nvmap_heap_alloc: allocates a block of memory of len bytes, aligned to
 * align bytes. */
struct nvmap_heap_block *nvmap_heap_alloc(struct nvmap_heap *h,
					  struct nvmap_handle *handle)
{
	struct nvmap_heap_block *b;
	size_t len        = handle->size;
	size_t align      = handle->align;
	unsigned int prot = handle->flags;

	mutex_lock(&h->lock);

	align = max_t(size_t, align, L1_CACHE_BYTES);
	b = do_heap_alloc(h, len, align, prot, 0);

	if (b) {
		b->handle = handle;
		handle->carveout = b;
	}
	mutex_unlock(&h->lock);
	return b;
}

struct nvmap_heap *nvmap_block_to_heap(struct nvmap_heap_block *b)
{
	struct list_block *lb;
	lb = container_of(b, struct list_block, block);
	return lb->heap;
}

/* nvmap_heap_free: frees block b*/
void nvmap_heap_free(struct nvmap_heap_block *b)
{
	struct nvmap_heap *h = nvmap_block_to_heap(b);
	struct list_block *lb;

	mutex_lock(&h->lock);

	lb = container_of(b, struct list_block, block);
	nvmap_flush_heap_block(NULL, b, lb->size, lb->mem_prot);
	do_heap_free(b);

	mutex_unlock(&h->lock);
}

/* nvmap_heap_create: create a heap object of len bytes, starting from
 * address base.
 */
struct nvmap_heap *nvmap_heap_create(struct device *parent,
				     const struct nvmap_platform_carveout *co,
				     phys_addr_t base, size_t len, void *arg)
{
	struct nvmap_heap *h;
	DEFINE_DMA_ATTRS(attrs);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h) {
		dev_err(parent, "%s: out of memory\n", __func__);
		return NULL;
	}

	if (co->cma_dev) {
#ifdef CONFIG_CMA
		struct dma_contiguous_stats stats;

		dma_get_contiguous_stats(co->cma_dev, &stats);
		base = stats.base;
		len = stats.size;
		h->cma_dev = co->cma_dev;
		h->dma_dev = co->dma_dev;
#else
		goto fail;
#endif
	} else {
		int err;

		if (co->dma_dev)
			h->dma_dev = co->dma_dev;
		else
			/* To continue working with bsp that doesn't
			 * pass dma_dev ptr.
			 */
			h->dma_dev = &h->dev;
		dev_set_name(h->dma_dev, "%s", co->name);
		dma_set_coherent_mask(h->dma_dev, DMA_BIT_MASK(64));
		/* declare Non-CMA heap */
		err = dma_declare_coherent_memory(h->dma_dev, 0, base, len,
				DMA_MEMORY_NOMAP | DMA_MEMORY_EXCLUSIVE);
		if (err & DMA_MEMORY_NOMAP) {
			dev_info(h->dma_dev, "dma coherent mem declare %pa,%zu\n",
				&base, len);
		} else {
			dev_dbg(h->dma_dev, "dma coherent declare fail %pa,%zu\n",
				&base, len);
			goto fail;
		}
	}

	h->name = co->name;
	h->arg = arg;
	h->base = base;
	h->len = len;
	INIT_LIST_HEAD(&h->all_list);
	mutex_init(&h->lock);
	inner_flush_cache_all();
	outer_flush_range(base, base + len);
	wmb();

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, &attrs);
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	dma_map_linear_attrs(parent->parent, base, len, DMA_TO_DEVICE,
				&attrs);
#endif
	dev_info(parent, "created heap %s base 0x%p size (%zuKiB)\n",
		co->name, (void *)(uintptr_t)base, len/1024);
	return h;
fail:
	kfree(h);
	return NULL;
}

void *nvmap_heap_to_arg(struct nvmap_heap *heap)
{
	return heap->arg;
}

/* nvmap_heap_destroy: frees all resources in heap */
void nvmap_heap_destroy(struct nvmap_heap *heap)
{
	WARN_ON(!list_is_singular(&heap->all_list));
	while (!list_empty(&heap->all_list)) {
		struct list_block *l;
		l = list_first_entry(&heap->all_list, struct list_block,
				     all_list);
		list_del(&l->all_list);
		kmem_cache_free(heap_block_cache, l);
	}
	kfree(heap);
}

int nvmap_heap_init(void)
{
	heap_block_cache = KMEM_CACHE(list_block, 0);
	if (!heap_block_cache) {
		pr_err("%s: unable to create heap block cache\n", __func__);
		return -ENOMEM;
	}
	pr_info("%s: created heap block cache\n", __func__);
	return 0;
}

void nvmap_heap_deinit(void)
{
	if (heap_block_cache)
		kmem_cache_destroy(heap_block_cache);

	heap_block_cache = NULL;
}
