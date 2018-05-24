/*
 * drivers/video/tegra/nvmap/nvmap.c
 *
 * Memory manager for Tegra GPU
 *
 * Copyright (c) 2009-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/rbtree.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/export.h>

#include <asm/pgtable.h>
#include <asm/tlbflush.h>

#include <linux/nvmap.h>
#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

static void *handle_pgalloc_mmap(struct nvmap_handle *h)
{
	struct page **pages;
	void *vaddr;

	pgprot_t prot = nvmap_pgprot(h, PG_PROT_KERNEL);

	pages = nvmap_pages(h->pgalloc.pages, h->size >> PAGE_SHIFT);
	if (!pages)
		return NULL;

	vaddr = vm_map_ram(pages, h->size >> PAGE_SHIFT, -1, prot);
	nvmap_altfree(pages, (h->size >> PAGE_SHIFT) * sizeof(*pages));
	if (!vaddr && !h->vaddr)
		return NULL;

	if (vaddr && atomic_long_cmpxchg(&h->vaddr, 0, (long)vaddr)) {
		vm_unmap_ram(vaddr, h->size >> PAGE_SHIFT);
	}
	return h->vaddr;
}

static void *handle_carveout_mmap(struct nvmap_handle *h)
{
	pgprot_t prot = nvmap_pgprot(h, PG_PROT_KERNEL);
	unsigned long adj_size;
	struct vm_struct *v;
	void *vaddr;

	/* carveout - explicitly map the pfns into a vmalloc area */
	adj_size = h->carveout->base & ~PAGE_MASK;
	adj_size += h->size;
	adj_size = PAGE_ALIGN(adj_size);

	v = alloc_vm_area(adj_size, NULL);
	if (!v)
		return NULL;

	vaddr = v->addr + (h->carveout->base & ~PAGE_MASK);
	ioremap_page_range((ulong)v->addr, (ulong)v->addr + adj_size,
			h->carveout->base & PAGE_MASK, prot);

	if (vaddr && atomic_long_cmpxchg(&h->vaddr, 0, (long)vaddr)) {
		struct vm_struct *vm;

		vaddr -= (h->carveout->base & ~PAGE_MASK);
		vm = find_vm_area(vaddr);
		BUG_ON(!vm);
		free_vm_area(vm);
	}

	return h->vaddr;
}

void *NVMAP2_handle_mmap(struct nvmap_handle *h)
{
	void *vaddr;

	if (!virt_addr_valid(h))
		return NULL;

	h = nvmap_handle_get(h);
	if (!h)
		return NULL;

	if (!h->alloc)
		goto put_handle;

	if (!(h->heap_type & nvmap_dev->cpu_access_mask))
		goto put_handle;

	if (h->vaddr)
		return h->vaddr;

	nvmap_kmaps_inc(h);

	if (h->heap_pgalloc) {
		vaddr = handle_pgalloc_mmap(h);
	} else {
		vaddr = handle_carveout_mmap(h);
	}

	/* leave the handle ref count incremented by 1, so that
	 * the handle will not be freed while the kernel mapping exists.
	 * nvmap_handle_put will be called by unmapping this address */
	if (vaddr)
		return vaddr;

	/* If we fail to map then set kmaps back to original value */
	nvmap_kmaps_dec(h);
put_handle:
	NVMAP2_handle_put(h);
	return NULL;
}

void NVMAP2_handle_munmap(struct nvmap_handle *h, void *addr)
{
	if (!h || !h->alloc ||
	    WARN_ON(!virt_addr_valid(h)) ||
	    WARN_ON(!addr) ||
	    !(h->heap_type & nvmap_dev->cpu_access_mask))
		return;

	NVMAP2_handle_put(h);
}
