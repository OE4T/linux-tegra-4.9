/*
 * Coherent per-device memory handling.
 * Borrowed from i386
 */
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dma-contiguous.h>
#include <linux/debugfs.h>
#include <linux/highmem.h>
#include <asm/cacheflush.h>

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "%s:%d: " fmt, __func__, __LINE__

#define RESIZE_MAGIC 0xC11A900d
struct heap_info {
	int magic;
	char *name;
	/* number of chunks memory to manage in */
	unsigned int num_chunks;
	/* dev to manage cma/coherent memory allocs, if resize allowed */
	struct device dev;
	/* device to allocate memory from cma */
	struct device *cma_dev;
	/* lock to synchronise heap resizing */
	struct mutex resize_lock;
	/* CMA chunk size if resize supported */
	size_t cma_chunk_size;
	/* heap current base */
	phys_addr_t curr_base;
	/* heap current length */
	size_t curr_len;
	/* heap lowest base */
	phys_addr_t cma_base;
	/* heap max length */
	size_t cma_len;
	size_t rem_chunk_size;
	struct dentry *dma_debug_root;
	int (*update_resize_cfg)(phys_addr_t , size_t);
};

#ifdef CONFIG_ARM_DMA_IOMMU_ALIGNMENT
#define DMA_BUF_ALIGNMENT CONFIG_ARM_DMA_IOMMU_ALIGNMENT
#else
#define DMA_BUF_ALIGNMENT 8
#endif

struct dma_coherent_mem {
	void		*virt_base;
	dma_addr_t	device_base;
	unsigned long	pfn_base;
	int		size;
	int		flags;
	unsigned long	*bitmap;
	spinlock_t	spinlock;
};

static bool dma_is_coherent_dev(struct device *dev)
{
	struct heap_info *h;

	if (!dev)
		return false;
	h = dev_get_drvdata(dev);
	if (!h)
		return false;
	if (h->magic != RESIZE_MAGIC)
		return false;
	return true;
}
static void dma_debugfs_init(struct device *dev, struct heap_info *heap)
{
	if (!heap->dma_debug_root) {
		heap->dma_debug_root = debugfs_create_dir(dev_name(dev), NULL);
		if (IS_ERR_OR_NULL(heap->dma_debug_root)) {
			dev_err(dev, "couldn't create debug files\n");
			return;
		}
	}

	debugfs_create_x32("curr_base", S_IRUGO,
		heap->dma_debug_root, (u32 *)&heap->curr_base);
	debugfs_create_x32("curr_len", S_IRUGO,
		heap->dma_debug_root, (u32 *)&heap->curr_len);
	debugfs_create_x32("cma_base", S_IRUGO,
		heap->dma_debug_root, (u32 *)&heap->cma_base);
	debugfs_create_x32("cma_size", S_IRUGO,
		heap->dma_debug_root, (u32 *)&heap->cma_len);
	debugfs_create_x32("cma_chunk_size", S_IRUGO,
		heap->dma_debug_root, (u32 *)&heap->cma_chunk_size);
	debugfs_create_x32("num_cma_chunks", S_IRUGO,
		heap->dma_debug_root, (u32 *)&heap->num_chunks);
}

static bool dma_init_coherent_memory(
	phys_addr_t phys_addr, dma_addr_t device_addr, size_t size, int flags,
	struct dma_coherent_mem **mem)
{
	struct dma_coherent_mem *dma_mem = NULL;
	void __iomem *mem_base = NULL;
	int pages = size >> PAGE_SHIFT;
	int bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);

	if ((flags &
		(DMA_MEMORY_MAP | DMA_MEMORY_IO | DMA_MEMORY_NOMAP)) == 0)
		goto out;
	if (!size)
		goto out;

	if (flags & DMA_MEMORY_NOMAP)
		goto skip_mapping;

	if (flags & DMA_MEMORY_MAP)
		mem_base = memremap(phys_addr, size, MEMREMAP_WC);
	else
		mem_base = ioremap(phys_addr, size);
	if (!mem_base)
		goto out;

skip_mapping:
	dma_mem = kzalloc(sizeof(struct dma_coherent_mem), GFP_KERNEL);
	if (!dma_mem)
		goto out;
	dma_mem->bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!dma_mem->bitmap)
		goto out;

	dma_mem->virt_base = mem_base;
	dma_mem->device_base = device_addr;
	dma_mem->pfn_base = PFN_DOWN(phys_addr);
	dma_mem->size = pages;
	dma_mem->flags = flags;
	spin_lock_init(&dma_mem->spinlock);

	*mem = dma_mem;
	return true;

out:
	kfree(dma_mem);
	if (mem_base) {
		if (flags & DMA_MEMORY_MAP)
			memunmap(mem_base);
		else
			iounmap(mem_base);
	}
	return false;
}

static void dma_release_coherent_memory(struct dma_coherent_mem *mem)
{
	if (!mem)
		return;

	if (!(mem->flags & DMA_MEMORY_NOMAP))
		goto skip_unmapping;

	if (mem->flags & DMA_MEMORY_MAP)
		memunmap(mem->virt_base);
	else
		iounmap(mem->virt_base);
skip_unmapping:
	kfree(mem->bitmap);
	kfree(mem);
}

static int declare_coherent_heap(struct device *dev, phys_addr_t base,
					size_t size)
{
	int err;

	BUG_ON(dev->dma_mem);
	dma_set_coherent_mask(dev,  DMA_BIT_MASK(64));
	err = dma_declare_coherent_memory(dev, 0,
			base, size, DMA_MEMORY_NOMAP);
	if (err & DMA_MEMORY_NOMAP) {
		dev_dbg(dev, "dma coherent mem base (0x%pa) size (0x%zx)\n",
			&base, size);
		return 0;
	}
	dev_err(dev, "declare dma coherent_mem fail 0x%pa 0x%zx\n",
		&base, size);
	return -ENOMEM;
}

int dma_declare_coherent_resizable_cma_memory(struct device *dev,
					struct dma_declare_info *dma_info)
{
#ifdef CONFIG_DMA_CMA
	int err = 0;
	struct heap_info *heap_info = NULL;
	struct dma_contiguous_stats stats;

	if (!dev || !dma_info || !dma_info->name || !dma_info->cma_dev)
		return -EINVAL;

	heap_info = kzalloc(sizeof(*heap_info), GFP_KERNEL);
	if (!heap_info)
		return -ENOMEM;

	heap_info->magic = RESIZE_MAGIC;
	heap_info->name = kmalloc(strlen(dma_info->name) + 1, GFP_KERNEL);
	if (!heap_info->name) {
		kfree(heap_info);
		return -ENOMEM;
	}

	dma_get_contiguous_stats(dma_info->cma_dev, &stats);
	pr_info("resizable heap=%s, base=0x%pa, size=0x%zx\n",
		dma_info->name, &stats.base, stats.size);
	strcpy(heap_info->name, dma_info->name);
	dev_set_name(dev, "dma-%s", heap_info->name);
	heap_info->cma_dev = dma_info->cma_dev;
	heap_info->cma_chunk_size = dma_info->size ? : stats.size;
	heap_info->cma_base = stats.base;
	heap_info->cma_len = stats.size;
	heap_info->curr_base = stats.base;
	dev_set_name(heap_info->cma_dev, "cma-%s-heap", heap_info->name);
	mutex_init(&heap_info->resize_lock);

	if (heap_info->cma_len < heap_info->cma_chunk_size) {
		dev_err(dev, "error cma_len(0x%zx) < cma_chunk_size(0x%zx)\n",
			heap_info->cma_len, heap_info->cma_chunk_size);
		err = -EINVAL;
		goto fail;
	}

	heap_info->num_chunks = div64_u64_rem(heap_info->cma_len,
		(u64)heap_info->cma_chunk_size, (u64 *)&heap_info->rem_chunk_size);
	if (heap_info->rem_chunk_size) {
		heap_info->num_chunks++;
		dev_info(dev, "heap size is not multiple of cma_chunk_size "
			"heap_info->num_chunks (%d) rem_chunk_size(0x%zx)\n",
			heap_info->num_chunks, heap_info->rem_chunk_size);
	} else
		heap_info->rem_chunk_size = heap_info->cma_chunk_size;

	dev_set_name(&heap_info->dev, "%s-heap", heap_info->name);

	if (dma_info->notifier.ops)
		heap_info->update_resize_cfg =
			dma_info->notifier.ops->resize;

	dev_set_drvdata(dev, heap_info);
	dma_debugfs_init(dev, heap_info);

	if (declare_coherent_heap(&heap_info->dev,
				  heap_info->cma_base, heap_info->cma_len))
		goto declare_fail;
	heap_info->dev.dma_mem->size = 0;

	pr_info("resizable cma heap=%s create successful", heap_info->name);
	return 0;
declare_fail:
	kfree(heap_info->name);
fail:
	kfree(heap_info);
	return err;
#else
	return -EINVAL;
#endif
}
EXPORT_SYMBOL(dma_declare_coherent_resizable_cma_memory);

static int dma_assign_coherent_memory(struct device *dev,
				      struct dma_coherent_mem *mem)
{
	if (dev->dma_mem)
		return -EBUSY;

	dev->dma_mem = mem;
	/* FIXME: this routine just ignores DMA_MEMORY_INCLUDES_CHILDREN */

	return 0;
}

int dma_declare_coherent_memory(struct device *dev, phys_addr_t phys_addr,
				dma_addr_t device_addr, size_t size, int flags)
{
	struct dma_coherent_mem *mem;

	if (!dma_init_coherent_memory(phys_addr, device_addr, size, flags,
				      &mem))
		return 0;

	if (dma_assign_coherent_memory(dev, mem) == 0)
		return flags & DMA_MEMORY_NOMAP ? DMA_MEMORY_NOMAP :
		       flags & DMA_MEMORY_MAP ? DMA_MEMORY_MAP : DMA_MEMORY_IO;

	dma_release_coherent_memory(mem);
	return 0;
}
EXPORT_SYMBOL(dma_declare_coherent_memory);


static phys_addr_t alloc_from_contiguous_heap(
				struct heap_info *h,
				phys_addr_t base, size_t len)
{
	size_t count;
	struct page *page;
	unsigned long order;

	dev_dbg(h->cma_dev, "req at base (0x%pa) size (0x%zx)\n",
		&base, len);
	order = get_order(len);
	count = PAGE_ALIGN(len) >> PAGE_SHIFT;
	page = dma_alloc_at_from_contiguous(h->cma_dev, count, order, base);
	if (!page) {
		dev_err(h->cma_dev, "dma_alloc_at_from_contiguous failed\n");
		goto dma_alloc_err;
	}

	base = page_to_phys(page);
	dev_dbg(h->cma_dev, "allocated at base (0x%pa) size (0x%zx)\n",
		&base, len);
	BUG_ON(base < h->cma_base ||
		base - h->cma_base + len > h->cma_len);
	return base;

dma_alloc_err:
	return DMA_ERROR_CODE;
}

static void release_from_contiguous_heap(
				struct heap_info *h,
				phys_addr_t base, size_t len)
{
	struct page *page = phys_to_page(base);
	size_t count = PAGE_ALIGN(len) >> PAGE_SHIFT;

	dma_release_from_contiguous(h->cma_dev, page, count);
	dev_dbg(h->cma_dev, "released at base (0x%pa) size (0x%zx)\n",
		&base, len);
}

static void get_first_and_last_idx(struct heap_info *h,
				   int *first_alloc_idx, int *last_alloc_idx)
{
	if (!h->curr_len) {
		*first_alloc_idx = -1;
		*last_alloc_idx = h->num_chunks;
	} else {
		*first_alloc_idx = div_u64(h->curr_base - h->cma_base,
					   h->cma_chunk_size);
		*last_alloc_idx = div_u64(h->curr_base - h->cma_base +
					  h->curr_len + h->cma_chunk_size -
					  h->rem_chunk_size,
					  h->cma_chunk_size) - 1;
	}
}

static void update_alloc_range(struct heap_info *h)
{
	if (!h->curr_len)
		h->dev.dma_mem->size = 0;
	else
		h->dev.dma_mem->size = (h->curr_base - h->cma_base +
					h->curr_len) >> PAGE_SHIFT;
}

static int heap_resize_locked(struct heap_info *h)
{
	int i;
	int err = 0;
	phys_addr_t base = -1;
	size_t len = h->cma_chunk_size;
	phys_addr_t prev_base = h->curr_base;
	size_t prev_len = h->curr_len;
	int alloc_at_idx = 0;
	int first_alloc_idx;
	int last_alloc_idx;
	phys_addr_t start_addr = 0;

	get_first_and_last_idx(h, &first_alloc_idx, &last_alloc_idx);
	pr_debug("req resize, fi=%d,li=%d\n", first_alloc_idx, last_alloc_idx);

	/* All chunks are in use. Can't grow it. */
	if (first_alloc_idx == 0 && last_alloc_idx == h->num_chunks - 1)
		return -ENOMEM;

	/* All chunks are free. Can allocate anywhere in CMA with
	 * cma_chunk_size alignment.
	 */
	if (first_alloc_idx == -1) {
		base = alloc_from_contiguous_heap(h, start_addr, len);
		if (!dma_mapping_error(h->cma_dev, base))
			goto alloc_success;
	}

	/* Free chunk before previously allocated chunk. Attempt
	 * to allocate only immediate previous chunk.
	 */
	if (first_alloc_idx > 0) {
		alloc_at_idx = first_alloc_idx - 1;
		start_addr = alloc_at_idx * h->cma_chunk_size + h->cma_base;
		base = alloc_from_contiguous_heap(h, start_addr, len);
		if (base == start_addr)
			goto alloc_success;
		BUG_ON(!dma_mapping_error(h->cma_dev, base));
	}

	/* Free chunk after previously allocated chunk. */
	if (last_alloc_idx < h->num_chunks - 1) {
		alloc_at_idx = last_alloc_idx + 1;
		len = (alloc_at_idx == h->num_chunks - 1) ?
				h->rem_chunk_size : h->cma_chunk_size;
		start_addr = alloc_at_idx * h->cma_chunk_size + h->cma_base;
		base = alloc_from_contiguous_heap(h, start_addr, len);
		if (base == start_addr)
			goto alloc_success;
		BUG_ON(!dma_mapping_error(h->cma_dev, base));
	}

	if (dma_mapping_error(h->cma_dev, base))
		dev_err(&h->dev,
		"Failed to allocate contiguous memory on heap grow req\n");

	return -ENOMEM;

alloc_success:
	if (!h->curr_len || h->curr_base > base)
		h->curr_base = base;
	h->curr_len += len;

	for (i = 0; i < (len >> PAGE_SHIFT); i++) {
		struct page *page = phys_to_page(i + base);

		if (PageHighMem(page)) {
			void *ptr = kmap_atomic(page);
//			__dma_flush_area(ptr, PAGE_SIZE);
			kunmap_atomic(ptr);
		} else {
			void *ptr = page_address(page);
//			__dma_flush_area(ptr, PAGE_SIZE);
		}
	}

	/* Handle VPR configuration updates*/
	if (h->update_resize_cfg) {
		err = h->update_resize_cfg(h->curr_base, h->curr_len);
		if (err) {
			dev_err(&h->dev, "Failed to update heap resize\n");
			goto fail_update;
		}
		dev_dbg(&h->dev, "update vpr base to %pa, size=%zx\n",
			&h->curr_base, h->curr_len);
	}

	update_alloc_range(h);
	dev_dbg(&h->dev,
		"grow heap base from=0x%pa to=0x%pa,"
		" len from=0x%zx to=0x%zx\n",
		&prev_base, &h->curr_base, prev_len, h->curr_len);
	return 0;

fail_update:
	release_from_contiguous_heap(h, base, len);
	h->curr_base = prev_base;
	h->curr_len = prev_len;
	return -ENOMEM;
}

/* retval: !0 on success, 0 on failure */
static int dma_alloc_from_coherent_dev_at(struct device *dev, ssize_t size,
				       dma_addr_t *dma_handle, void **ret,
				       unsigned long attrs, ulong start)
{
	struct dma_coherent_mem *mem;
	int order = get_order(size);
	unsigned long flags;
	int pageno;
	int dma_memory_map;
	unsigned int count;
	unsigned long align;

	if (!dev)
		return 0;
	mem = dev->dma_mem;
	if (!mem)
		return 0;

	*dma_handle = DMA_ERROR_CODE;
	*ret = NULL;
	spin_lock_irqsave(&mem->spinlock, flags);

	if (unlikely(size > (mem->size << PAGE_SHIFT)))
		goto err;

	if (attrs & DMA_ATTR_ALLOC_EXACT_SIZE) {
		align = 0;
		count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	} else  {
		if (order > DMA_BUF_ALIGNMENT)
			align = (1 << DMA_BUF_ALIGNMENT) - 1;
		else
			align = (1 << order) - 1;
	}

	if (DMA_ATTR_ALLOC_EXACT_SIZE & attrs)
		count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	else
		count = 1 << order;

	pageno = bitmap_find_next_zero_area(mem->bitmap, mem->size,
			start, count, align);

	if (pageno >= mem->size)
		goto err;

	bitmap_set(mem->bitmap, pageno, count);

	/*
	 * Memory was found in the per-device area.
	 */
	*dma_handle = mem->device_base + (pageno << PAGE_SHIFT);
	if (!(mem->flags & DMA_MEMORY_NOMAP)) {
		*ret = mem->virt_base + (pageno << PAGE_SHIFT);
		dma_memory_map = (mem->flags & DMA_MEMORY_MAP);
	}
	spin_unlock_irqrestore(&mem->spinlock, flags);
	if (dma_memory_map)
		memset(*ret, 0, size);
	else if (*ret)
		memset_io(*ret, 0, size);

	return 1;

err:
	spin_unlock_irqrestore(&mem->spinlock, flags);
	/*
	 * In the case where the allocation can not be satisfied from the
	 * per-device area, try to fall back to generic memory if the
	 * constraints allow it.
	 */
	return mem->flags & DMA_MEMORY_EXCLUSIVE;
}

/**
 * dma_release_from_coherent_dev() - try to free the memory allocated from
 * per-device coherent memory pool
 * @dev:	device from which the memory was allocated
 * @size:	size of the memory area to free
 * @vaddr:	virtual address of allocated pages
 * @attrs:	DMA Attribute
 *
 * This checks whether the memory was allocated from the per-device
 * coherent memory pool and if so, releases that memory.
 *
 * Returns 1 if we correctly released the memory, or 0 if
 * dma_release_coherent_attr() should proceed with releasing memory from
 * generic pools.
 */
int dma_release_from_coherent_dev(struct device *dev, size_t size, void *vaddr,
				unsigned long attrs)
{
	struct dma_coherent_mem *mem = dev ? dev->dma_mem : NULL;
	void *mem_addr;
	unsigned int count;
	unsigned int pageno;

	if (!mem)
		return 0;

	if (mem->flags & DMA_MEMORY_NOMAP)
		mem_addr =  (void *)(uintptr_t)mem->device_base;
	else
		mem_addr =  mem->virt_base;

	if (mem && vaddr >= mem_addr &&
	    vaddr - mem_addr < mem->size << PAGE_SHIFT) {

		unsigned long flags;

		if (DMA_ATTR_ALLOC_EXACT_SIZE & attrs)
			count = PAGE_ALIGN(size) >> PAGE_SHIFT;
		else
			count = 1 << get_order(size);

		spin_lock_irqsave(&mem->spinlock, flags);
		bitmap_clear(mem->bitmap, pageno, count);
		spin_unlock_irqrestore(&mem->spinlock, flags);

		return 1;
	}
	return 0;
}

static int dma_alloc_from_coherent_dev(struct device *dev, ssize_t size,
				       dma_addr_t *dma_handle, void **ret,
				       unsigned long attrs)
{
	return dma_alloc_from_coherent_dev_at(dev, size, dma_handle,
					      ret, attrs, 0);
}

/* retval: !0 on success, 0 on failure */
static int dma_alloc_from_coherent_heap_dev(struct device *dev, size_t len,
					dma_addr_t *dma_handle, void **ret,
					unsigned long attrs)
{
	struct heap_info *h = NULL;

	*dma_handle = DMA_ERROR_CODE;
	if (!dma_is_coherent_dev(dev))
		return 0;

	h = dev_get_drvdata(dev);
	BUG_ON(!h);
	if (!h)
		return DMA_MEMORY_EXCLUSIVE;
	attrs |= DMA_ATTR_ALLOC_EXACT_SIZE;

	mutex_lock(&h->resize_lock);
retry_alloc:
	/* Try allocation from already existing CMA chunks */
	if (dma_alloc_from_coherent_dev_at(
		&h->dev, len, dma_handle, ret, attrs,
		(h->curr_base - h->cma_base) >> PAGE_SHIFT)) {
		dev_dbg(&h->dev, "allocated addr 0x%pa len 0x%zx\n",
			dma_handle, len);
		goto out;
	}

	if (!heap_resize_locked(h))
		goto retry_alloc;
out:
	mutex_unlock(&h->resize_lock);
	return DMA_MEMORY_EXCLUSIVE;
}

/* retval: !0 on success, 0 on failure */
static int dma_release_from_coherent_heap_dev(struct device *dev, size_t len,
					void *base, unsigned long attrs)
{
	int idx = 0;
	int err = 0;
	int resize_err = 0;
	void *ret = NULL;
	dma_addr_t dev_base;
	struct heap_info *h = NULL;
	size_t chunk_size;
	int first_alloc_idx;
	int last_alloc_idx;

	if (!dma_is_coherent_dev(dev))
		return 0;

	h = dev_get_drvdata(dev);
	BUG_ON(!h);
	if (!h)
		return 1;
	if ((uintptr_t)base < h->cma_base ||
	    len > h->cma_chunk_size ||
	    (uintptr_t)base - h->cma_base > h->cma_len - len) {
		BUG();
		return 1;
	}

	attrs |= DMA_ATTR_ALLOC_EXACT_SIZE;

	mutex_lock(&h->resize_lock);

	idx = div_u64((uintptr_t)base - h->cma_base, h->cma_chunk_size);
	dev_dbg(&h->dev, "req free addr (%p) size (0x%zx) idx (%d)\n",
		base, len, idx);
	err = dma_release_from_coherent_dev(&h->dev, len, base, attrs);

	if (!err)
		goto out_unlock;

check_next_chunk:
	get_first_and_last_idx(h, &first_alloc_idx, &last_alloc_idx);

	/* Check if heap can be shrinked */
	if (idx == first_alloc_idx || idx == last_alloc_idx) {
		/* check if entire chunk is free */
		chunk_size = (idx == h->num_chunks - 1) ? h->rem_chunk_size :
							  h->cma_chunk_size;
		resize_err = dma_alloc_from_coherent_dev_at(&h->dev,
					chunk_size, &dev_base, &ret, attrs,
					idx * h->cma_chunk_size >> PAGE_SHIFT);
		if (!resize_err) {
			goto out_unlock;
		} else if (dev_base != h->cma_base + idx * h->cma_chunk_size) {
			resize_err = dma_release_from_coherent_dev(
					&h->dev, chunk_size,
					(void *)(uintptr_t)dev_base, attrs);
			BUG_ON(!resize_err);
			goto out_unlock;
		} else {
			dev_dbg(&h->dev,
				"prep to remove chunk b=0x%pa, s=0x%zx\n",
				&dev_base, chunk_size);
			resize_err = dma_release_from_coherent_dev(
					&h->dev, chunk_size,
					(void *)(uintptr_t)dev_base, attrs);
			BUG_ON(!resize_err);
			if (!resize_err) {
				dev_err(&h->dev, "failed to rel mem\n");
				goto out_unlock;
			}

			/* Handle VPR configuration updates */
			if (h->update_resize_cfg) {
				phys_addr_t new_base = h->curr_base;
				size_t new_len = h->curr_len - chunk_size;
				if (h->curr_base == dev_base)
					new_base += chunk_size;
				dev_dbg(&h->dev, "update vpr base to %pa, size=%zx\n",
					&new_base, new_len);
				resize_err =
					h->update_resize_cfg(new_base, new_len);
				if (resize_err) {
					dev_err(&h->dev,
						"update resize failed\n");
					goto out_unlock;
				}
			}

			if (h->curr_base == dev_base)
				h->curr_base += chunk_size;
			h->curr_len -= chunk_size;
			update_alloc_range(h);
			idx == first_alloc_idx ? ++idx : --idx;
			release_from_contiguous_heap(h, dev_base, chunk_size);
			dev_dbg(&h->dev, "removed chunk b=0x%pa, s=0x%zx"
				" new heap b=0x%pa, s=0x%zx\n", &dev_base,
				chunk_size, &h->curr_base, h->curr_len);
		}
		if (idx < h->num_chunks)
			goto check_next_chunk;
	}
out_unlock:
	mutex_unlock(&h->resize_lock);
	return err;
}

void dma_release_declared_memory(struct device *dev)
{
	struct dma_coherent_mem *mem = dev->dma_mem;

	if (!mem)
		return;
	dev->dma_mem = NULL;

	if (!(mem->flags & DMA_MEMORY_NOMAP))
		iounmap(mem->virt_base);

	kfree(mem->bitmap);
	kfree(mem);
}
EXPORT_SYMBOL(dma_release_declared_memory);

void *dma_mark_declared_memory_occupied(struct device *dev,
					dma_addr_t device_addr, size_t size)
{
	struct dma_coherent_mem *mem = dev->dma_mem;
	int pos, err;

	size += device_addr & ~PAGE_MASK;

	if (!mem)
		return ERR_PTR(-EINVAL);

	pos = (device_addr - mem->device_base) >> PAGE_SHIFT;
	err = bitmap_allocate_region(mem->bitmap, pos, get_order(size));
	if (err != 0)
		return ERR_PTR(err);
	return mem->virt_base + (pos << PAGE_SHIFT);
}
EXPORT_SYMBOL(dma_mark_declared_memory_occupied);

/**
 * dma_alloc_from_coherent_attr() - try to allocate memory from the per-device
 * coherent area
 *
 * @dev:	device from which we allocate memory
 * @size:	size of requested memory area
 * @dma_handle:	This will be filled with the correct dma handle
 * @ret:	This pointer will be filled with the virtual address
 *		to allocated area.
 * @attrs:	DMA Attribute
 * This function should be only called from per-arch dma_alloc_coherent()
 * to support allocation from per-device coherent memory pools.
 *
 * Returns 0 if dma_alloc_coherent_attr should continue with allocating from
 * generic memory areas, or !0 if dma_alloc_coherent should return @ret.
 */
int dma_alloc_from_coherent_attr(struct device *dev, ssize_t size,
				       dma_addr_t *dma_handle, void **ret,
				       unsigned long attrs)
{
	if (!dev)
		return 0;

	if (dev->dma_mem)
		return dma_alloc_from_coherent_dev(dev, size, dma_handle, ret,
							attrs);
	else
		return dma_alloc_from_coherent_heap_dev(dev, size, dma_handle,
							ret, attrs);
}
EXPORT_SYMBOL(dma_alloc_from_coherent_attr);

/**
 * dma_release_from_coherent_attr() - try to free the memory allocated from
 * per-device coherent memory pool
 * @dev:	device from which the memory was allocated
 * @size:	size of the memory area to free
 * @vaddr:	virtual address of allocated pages
 * @attrs:	DMA Attribute
 *
 * This checks whether the memory was allocated from the per-device
 * coherent memory pool and if so, releases that memory.
 *
 * Returns 1 if we correctly released the memory, or 0 if
 * dma_release_coherent_attr() should proceed with releasing memory from
 * generic pools.
 */
int dma_release_from_coherent_attr(struct device *dev, size_t size, void *vaddr,
				unsigned long attrs)
{
	if (!dev)
		return 0;

	if (dev->dma_mem)
		return dma_release_from_coherent_dev(dev, size, vaddr, attrs);
	else
		return dma_release_from_coherent_heap_dev(dev, size, vaddr,
			attrs);
}
EXPORT_SYMBOL(dma_release_from_coherent_attr);

/**
 * dma_mmap_from_coherent() - try to mmap the memory allocated from
 * per-device coherent memory pool to userspace
 * @dev:	device from which the memory was allocated
 * @vma:	vm_area for the userspace memory
 * @vaddr:	cpu address returned by dma_alloc_from_coherent
 * @size:	size of the memory buffer allocated by dma_alloc_from_coherent
 * @ret:	result from remap_pfn_range()
 *
 * This checks whether the memory was allocated from the per-device
 * coherent memory pool and if so, maps that memory to the provided vma.
 *
 * Returns 1 if we correctly mapped the memory, or 0 if the caller should
 * proceed with mapping memory from generic pools.
 */
int dma_mmap_from_coherent(struct device *dev, struct vm_area_struct *vma,
			   void *vaddr, size_t size, int *ret)
{
	struct dma_coherent_mem *mem = dev ? dev->dma_mem : NULL;
	void *mem_addr;

	if (!mem)
		return 0;

	if (mem->flags & DMA_MEMORY_NOMAP)
		mem_addr =  (void *)(uintptr_t)mem->device_base;
	else
		mem_addr =  mem->virt_base;

	if (mem && vaddr >= mem_addr && vaddr + size <=
		   (mem_addr + (mem->size << PAGE_SHIFT))) {
		unsigned long off = vma->vm_pgoff;
		int start = (vaddr - mem_addr) >> PAGE_SHIFT;
		int user_count = vma_pages(vma);
		int count = PAGE_ALIGN(size) >> PAGE_SHIFT;

		*ret = -ENXIO;
		if (off < count && user_count <= count - off) {
			unsigned long pfn = mem->pfn_base + start + off;
			*ret = remap_pfn_range(vma, vma->vm_start, pfn,
					       user_count << PAGE_SHIFT,
					       vma->vm_page_prot);
		}
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(dma_mmap_from_coherent);

/*
 * Support for reserved memory regions defined in device tree
 */
#ifdef CONFIG_OF_RESERVED_MEM
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>

static int rmem_dma_device_init(struct reserved_mem *rmem, struct device *dev)
{
	struct dma_coherent_mem *mem = rmem->priv;

	if (!mem &&
	    !dma_init_coherent_memory(rmem->base, rmem->base, rmem->size,
				      DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE,
				      &mem)) {
		pr_err("Reserved memory: failed to init DMA memory pool at %pa, size %ld MiB\n",
			&rmem->base, (unsigned long)rmem->size / SZ_1M);
		return -ENODEV;
	}
	rmem->priv = mem;
	dma_assign_coherent_memory(dev, mem);
	return 0;
}

static void rmem_dma_device_release(struct reserved_mem *rmem,
				    struct device *dev)
{
	dev->dma_mem = NULL;
}

static const struct reserved_mem_ops rmem_dma_ops = {
	.device_init	= rmem_dma_device_init,
	.device_release	= rmem_dma_device_release,
};

static int __init rmem_dma_setup(struct reserved_mem *rmem)
{
	unsigned long node = rmem->fdt_node;

	if (of_get_flat_dt_prop(node, "reusable", NULL))
		return -EINVAL;

#ifdef CONFIG_ARM
	if (!of_get_flat_dt_prop(node, "no-map", NULL)) {
		pr_err("Reserved memory: regions without no-map are not yet supported\n");
		return -EINVAL;
	}
#endif

	rmem->ops = &rmem_dma_ops;
	pr_info("Reserved memory: created DMA memory pool at %pa, size %ld MiB\n",
		&rmem->base, (unsigned long)rmem->size / SZ_1M);
	return 0;
}
RESERVEDMEM_OF_DECLARE(dma, "shared-dma-pool", rmem_dma_setup);
#endif
