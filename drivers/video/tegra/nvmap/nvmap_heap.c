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

struct heap_stat {
	size_t free;		/* total free size */
	size_t free_largest;	/* largest free block */
	size_t free_count;	/* number of free blocks */
	size_t total;		/* total size */
	size_t largest;		/* largest unique block */
	size_t count;		/* total number of blocks */
	/* fast compaction attempt counter */
	unsigned int compaction_count_fast;
	/* full compaction attempt counter */
	unsigned int compaction_count_full;
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

static struct kmem_cache *block_cache;

/* returns the free size of the heap (must be called while holding the parent
 * heap's lock. */
static phys_addr_t heap_stat(struct nvmap_heap *heap, struct heap_stat *stat)
{
	struct list_block *l = NULL;
	phys_addr_t base = -1ul;

	memset(stat, 0, sizeof(*stat));
	mutex_lock(&heap->lock);
	list_for_each_entry(l, &heap->all_list, all_list) {
		stat->total += l->size;
		stat->largest = max(l->size, stat->largest);
		stat->count++;
		base = min(base, l->orig_addr);
	}

	list_for_each_entry(l, &heap->free_list, free_list) {
		stat->free += l->size;
		stat->free_count++;
		stat->free_largest = max(l->size, stat->free_largest);
	}
	mutex_unlock(&heap->lock);

	return base;
}

static ssize_t heap_name_show(struct device *dev,
			      struct device_attribute *attr, char *buf);

static ssize_t heap_stat_show(struct device *dev,
			      struct device_attribute *attr, char *buf);

static struct device_attribute heap_stat_total_max =
	__ATTR(total_max, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_stat_total_count =
	__ATTR(total_count, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_stat_total_size =
	__ATTR(total_size, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_stat_free_max =
	__ATTR(free_max, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_stat_free_count =
	__ATTR(free_count, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_stat_free_size =
	__ATTR(free_size, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_stat_base =
	__ATTR(base, S_IRUGO, heap_stat_show, NULL);

static struct device_attribute heap_attr_name =
	__ATTR(name, S_IRUGO, heap_name_show, NULL);

static struct attribute *heap_stat_attrs[] = {
	&heap_stat_total_max.attr,
	&heap_stat_total_count.attr,
	&heap_stat_total_size.attr,
	&heap_stat_free_max.attr,
	&heap_stat_free_count.attr,
	&heap_stat_free_size.attr,
	&heap_stat_base.attr,
	&heap_attr_name.attr,
	NULL,
};

static struct attribute_group heap_stat_attr_group = {
	.attrs	= heap_stat_attrs,
};

static ssize_t heap_name_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{

	struct nvmap_heap *heap = container_of(dev, struct nvmap_heap, dev);
	return sprintf(buf, "%s\n", heap->name);
}

static ssize_t heap_stat_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct nvmap_heap *heap = container_of(dev, struct nvmap_heap, dev);
	struct heap_stat stat;
	phys_addr_t base;

	base = heap_stat(heap, &stat);

	if (attr == &heap_stat_total_max)
		return sprintf(buf, "%zu\n", stat.largest);
	else if (attr == &heap_stat_total_count)
		return sprintf(buf, "%zu\n", stat.count);
	else if (attr == &heap_stat_total_size)
		return sprintf(buf, "%zu\n", stat.total);
	else if (attr == &heap_stat_free_max)
		return sprintf(buf, "%zu\n", stat.free_largest);
	else if (attr == &heap_stat_free_count)
		return sprintf(buf, "%zu\n", stat.free_count);
	else if (attr == &heap_stat_free_size)
		return sprintf(buf, "%zu\n", stat.free);
	else if (attr == &heap_stat_base)
		return sprintf(buf, "%08llx\n", (unsigned long long)base);
	else
		return -EINVAL;
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
	struct list_block *b = NULL;
	struct list_block *i = NULL;
	struct list_block *rem = NULL;
	phys_addr_t fix_base;

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

	list_for_each_entry(i, &heap->free_list, free_list) {
		size_t fix_size;
		fix_base = ALIGN(i->block.base, align);
		if (!fix_base || fix_base >= i->block.base + i->size)
			continue;

		fix_size = i->size - (fix_base - i->block.base);

		/* needed for compaction. relocated chunk
		 * should never go up */
		if (base_max && fix_base > base_max)
			break;

		if (fix_size >= len) {
			b = i;
			break;
		}
	}

	if (!b)
		return NULL;

	b->block.type = BLOCK_FIRST_FIT;

	/* split free block */
	if (b->block.base != fix_base) {
		/* insert a new free block before allocated */
		rem = kmem_cache_zalloc(block_cache, GFP_KERNEL);
		if (!rem) {
			b->orig_addr = b->block.base;
			b->block.base = fix_base;
			b->size -= (b->block.base - b->orig_addr);
			goto out;
		}

		rem->block.type = BLOCK_EMPTY;
		rem->block.base = b->block.base;
		rem->orig_addr = rem->block.base;
		rem->size = fix_base - rem->block.base;
		b->block.base = fix_base;
		b->orig_addr = fix_base;
		b->size -= rem->size;
		list_add_tail(&rem->all_list,  &b->all_list);
		list_add_tail(&rem->free_list, &b->free_list);
	}

	b->orig_addr = b->block.base;

	if (b->size > len) {
		/* insert a new free block after allocated */
		rem = kmem_cache_zalloc(block_cache, GFP_KERNEL);
		if (!rem)
			goto out;

		rem->block.type = BLOCK_EMPTY;
		rem->block.base = b->block.base + len;
		rem->size = b->size - len;
		BUG_ON(rem->size > b->size);
		rem->orig_addr = rem->block.base;
		b->size = len;
		list_add(&rem->all_list,  &b->all_list);
		list_add(&rem->free_list, &b->free_list);
	}

out:
	list_del(&b->free_list);
	b->heap = heap;
	b->mem_prot = mem_prot;
	b->align = align;
	return &b->block;
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
	struct list_block *n = NULL;
	struct nvmap_heap *heap = b->heap;

	BUG_ON(b->block.base > b->orig_addr);
	b->size += (b->block.base - b->orig_addr);
	b->block.base = b->orig_addr;

	freelist_debug(heap, "free list before", b);

	/* Find position of first free block to the right of freed one */
	list_for_each_entry(n, &heap->free_list, free_list) {
		if (n->block.base > b->block.base)
			break;
	}

	/* Add freed block before found free one */
	list_add_tail(&b->free_list, &n->free_list);
	BUG_ON(list_empty(&b->all_list));

	freelist_debug(heap, "free list pre-merge", b);

	/* merge freed block with next if they connect
	 * freed block becomes bigger, next one is destroyed */
	if (!list_is_last(&b->free_list, &heap->free_list)) {
		n = list_first_entry(&b->free_list, struct list_block, free_list);
		if (n->block.base == b->block.base + b->size) {
			list_del(&n->all_list);
			list_del(&n->free_list);
			BUG_ON(b->orig_addr >= n->orig_addr);
			b->size += n->size;
			kmem_cache_free(block_cache, n);
		}
	}

	/* merge freed block with prev if they connect
	 * previous free block becomes bigger, freed one is destroyed */
	if (b->free_list.prev != &heap->free_list) {
		n = list_entry(b->free_list.prev, struct list_block, free_list);
		if (n->block.base + n->size == b->block.base) {
			list_del(&b->all_list);
			list_del(&b->free_list);
			BUG_ON(n->orig_addr >= b->orig_addr);
			n->size += b->size;
			kmem_cache_free(block_cache, b);
			b = n;
		}
	}

	freelist_debug(heap, "free list after", b);
	b->block.type = BLOCK_EMPTY;
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


static void heap_release(struct device *heap)
{
}

/* nvmap_heap_create: create a heap object of len bytes, starting from
 * address base.
 */
struct nvmap_heap *nvmap_heap_create(struct device *parent, const char *name,
				     phys_addr_t base, size_t len,
				     size_t buddy_size, void *arg)
{
	struct nvmap_heap *h = NULL;
	struct list_block *l = NULL;
	DEFINE_DMA_ATTRS(attrs);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h) {
		dev_err(parent, "%s: out of memory\n", __func__);
		goto fail_alloc;
	}

	l = kmem_cache_zalloc(block_cache, GFP_KERNEL);
	if (!l) {
		dev_err(parent, "%s: out of memory\n", __func__);
		goto fail_alloc;
	}

	dev_set_name(&h->dev, "heap-%s", name);
	h->name = name;
	h->arg = arg;
	h->dev.parent = parent;
	h->dev.driver = NULL;
	h->dev.release = heap_release;
	if (device_register(&h->dev)) {
		dev_err(parent, "%s: failed to register %s\n", __func__,
			dev_name(&h->dev));
		goto fail_alloc;
	}
	if (sysfs_create_group(&h->dev.kobj, &heap_stat_attr_group)) {
		dev_err(&h->dev, "%s: failed to create attributes\n", __func__);
		goto fail_register;
	}
	INIT_LIST_HEAD(&h->free_list);
	INIT_LIST_HEAD(&h->all_list);
	mutex_init(&h->lock);
	l->block.base = base;
	l->block.type = BLOCK_EMPTY;
	l->size = len;
	l->orig_addr = base;
	list_add_tail(&l->free_list, &h->free_list);
	list_add_tail(&l->all_list, &h->all_list);

	inner_flush_cache_all();
	outer_flush_range(base, base + len);
	wmb();

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
#ifdef CONFIG_PLATFORM_ENABLE_IOMMU
	dma_map_linear_attrs(parent->parent, base, len, DMA_TO_DEVICE, &attrs);
#endif
	return h;

fail_register:
	device_unregister(&h->dev);
fail_alloc:
	if (l)
		kmem_cache_free(block_cache, l);
	kfree(h);
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

	sysfs_remove_group(&heap->dev.kobj, &heap_stat_attr_group);
	device_unregister(&heap->dev);

	WARN_ON(!list_is_singular(&heap->all_list));
	while (!list_empty(&heap->all_list)) {
		struct list_block *l;
		l = list_first_entry(&heap->all_list, struct list_block,
				     all_list);
		list_del(&l->all_list);
		kmem_cache_free(block_cache, l);
	}

	kfree(heap);
}

/* nvmap_heap_create_group: adds the attribute_group grp to the heap kobject */
int nvmap_heap_create_group(struct nvmap_heap *heap,
			    const struct attribute_group *grp)
{
	return sysfs_create_group(&heap->dev.kobj, grp);
}

/* nvmap_heap_remove_group: removes the attribute_group grp  */
void nvmap_heap_remove_group(struct nvmap_heap *heap,
			     const struct attribute_group *grp)
{
	sysfs_remove_group(&heap->dev.kobj, grp);
}

int nvmap_heap_init(void)
{
	block_cache = KMEM_CACHE(list_block, 0);
	if (!block_cache) {
		pr_err("%s: unable to create block cache\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

void nvmap_heap_deinit(void)
{
	if (block_cache)
		kmem_cache_destroy(block_cache);

	block_cache = NULL;
}
