/*
 * drivers/video/tegra/nvmap/nvmap_heap.c
 *
 * GPU heap allocator.
 *
 * Copyright (c) 2011-2013, NVIDIA Corporation. All rights reserved.
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
#include "nvmap_priv.h"
#include "nvmap_heap.h"

#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include <linux/dma-mapping.h>

/*
 * "carveouts" are platform-defined regions of physically contiguous memory
 * which are not managed by the OS. a platform may specify multiple carveouts,
 * for either small special-purpose memory regions (like IRAM on Tegra SoCs)
 * or reserved regions of main system memory.
 *
 * the carveout allocator returns allocations which are physically contiguous.
 */

enum block_type {
	BLOCK_FIRST_FIT,	/* block was allocated directly from the heap */
	BLOCK_EMPTY,
};

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
	struct list_head free_list;
	struct mutex lock;
	const char *name;
	void *arg;
	struct device dev;
};

static struct kmem_cache *heap_block_cache;

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
	void *dev_addr = NULL;
	dma_addr_t dev_base;
	DEFINE_DMA_ATTRS(attrs);

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
		dev_err(&heap->dev, "%s: failed to alloc heap block %s\n",
			__func__, dev_name(&heap->dev));
		goto fail_heap_block_alloc;
	}

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, &attrs);
	dev_addr = dma_alloc_attrs(&heap->dev, len, &dev_base,
					DMA_MEMORY_NOMAP, &attrs);
	if (dev_base == DMA_ERROR_CODE) {
		dev_err(&heap->dev, "%s: failed to alloc DMA coherent mem %s\n",
			__func__, dev_name(&heap->dev));
		goto fail_dma_alloc;
	}
	pr_debug("dma_alloc_coherent base (%pa) size (%d) heap (%s)\n",
		&dev_base, len, heap->name);

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

#ifdef DEBUG_FREE_LIST
static void freelist_debug(struct nvmap_heap *heap, const char *title,
			   struct list_block *token)
{
	int i;
	struct list_block *n;

	dev_debug(&heap->dev, "%s\n", title);
	i = 0;
	list_for_each_entry(n, &heap->free_list, free_list) {
		dev_debug(&heap->dev, "\t%d [%p..%p]%s\n", i, (void *)n->orig_addr,
			  (void *)(n->orig_addr + n->size),
			  (n == token) ? "<--" : "");
		i++;
	}
}
#else
#define freelist_debug(_heap, _title, _token)	do { } while (0)
#endif

static struct list_block *do_heap_free(struct nvmap_heap_block *block)
{
	struct list_block *b = container_of(block, struct list_block, block);
	struct nvmap_heap *heap = b->heap;
	DEFINE_DMA_ATTRS(attrs);

	list_del(&b->all_list);

	pr_debug("dma_free_coherent base (0x%pa) size (%d) heap (%s)\n",
		&block->base, b->size, heap->name);
	/* assumes dev_alloc_coherent() returns same offset for phys_addr_t */
	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, &attrs);
	dma_free_attrs(&heap->dev, b->size, (void *)(uintptr_t)block->base,
				block->base, &attrs);

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
struct nvmap_heap *nvmap_heap_create(struct device *parent, const char *name,
				     phys_addr_t base, size_t len, void *arg)
{
	struct nvmap_heap *h = NULL;
	int err = 0;
	DEFINE_DMA_ATTRS(attrs);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h) {
		dev_err(parent, "%s: out of memory\n", __func__);
		goto fail_alloc;
	}

	dev_set_name(&h->dev, "heap-%s", name);
	h->name = name;
	h->arg = arg;

	INIT_LIST_HEAD(&h->free_list);
	INIT_LIST_HEAD(&h->all_list);
	mutex_init(&h->lock);

	err = dma_declare_coherent_memory(&h->dev, 0, base, len,
		DMA_MEMORY_NOMAP | DMA_MEMORY_EXCLUSIVE);
	if (!(err & DMA_MEMORY_NOMAP) || (base == 0)) {
		dev_err(&h->dev, "%s: Unable to declare dma coherent memory\n",
			__func__);
		goto fail_dma_declare;
	}

	inner_flush_cache_all();
	outer_flush_range(base, base + len);
	wmb();

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, &attrs);
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	dma_map_linear_attrs(parent->parent, base, len, DMA_TO_DEVICE, &attrs);
#endif

	return h;

fail_dma_declare:
	kfree(h);
fail_alloc:
	return NULL;
}

void *nvmap_heap_device_to_arg(struct device *dev)
{
	struct nvmap_heap *heap = container_of(dev, struct nvmap_heap, dev);
	return heap->arg;
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
