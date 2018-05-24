/*
 * drivers/video/tegra/nvmap/nvmap_ioctl.c
 *
 * User-space interface to nvmap
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/nvmap.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>

#include <asm/io.h>
#include <asm/memory.h>
#include <asm/uaccess.h>
#include <soc/tegra/common.h>
#include <trace/events/nvmap.h>

#include "nvmap_ioctl.h"
#include "nvmap_priv.h"
#include "nvmap_heap.h"


extern struct device tegra_vpr_dev;

/* NOTE: Callers of this utility function must invoke nvmap_handle_put after
 * using the returned nvmap_handle.
 */
struct nvmap_handle *nvmap_handle_get_from_fd(int fd)
{
	struct nvmap_handle *h;

	h = NVMAP2_handle_from_fd(fd);
	NVMAP2_handle_get(h);
	if (!IS_ERR(h))
		return h;
	return NULL;
}

int nvmap_ioctl_gup_test(struct file *filp, void __user *arg)
{
	int i, err = -EINVAL;
	struct nvmap_gup_test op;
	struct vm_area_struct *vma;
	struct nvmap_handle *handle;
	int nr_page;
	struct page **pages;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	op.result = 1;
	vma = find_vma(current->mm, op.va);
	if (unlikely(!vma) || (unlikely(op.va < vma->vm_start )) ||
	    unlikely(op.va >= vma->vm_end))
		goto exit;

	handle = nvmap_handle_get_from_fd(op.handle);
	if (!handle)
		goto exit;

	if (vma->vm_end - vma->vm_start != handle->size) {
		pr_err("handle size(0x%zx) and vma size(0x%lx) don't match\n",
			 handle->size, vma->vm_end - vma->vm_start);
		goto put_handle;
	}

	err = -ENOMEM;
	nr_page = handle->size >> PAGE_SHIFT;
	pages = nvmap_altalloc(nr_page * sizeof(*pages));
	if (IS_ERR_OR_NULL(pages)) {
		err = PTR_ERR(pages);
		goto put_handle;
	}

	err = nvmap_get_user_pages(op.va & PAGE_MASK, nr_page, pages);
	if (err)
		goto put_user_pages;

	for (i = 0; i < nr_page; i++) {
		if (handle->pgalloc.pages[i] != pages[i]) {
			pr_err("page pointers don't match, %p %p\n",
			       handle->pgalloc.pages[i], pages[i]);
			op.result = 0;
		}
	}

	if (op.result)
		err = 0;

	if (copy_to_user(arg, &op, sizeof(op)))
		err = -EFAULT;

put_user_pages:
	nvmap_altfree(pages, nr_page * sizeof(*pages));
put_handle:
	NVMAP2_handle_put(handle);
exit:
	pr_info("GUP Test %s\n", err ? "failed" : "passed");
	return err;
}

int nvmap_ioctl_set_tag_label(struct file *filp, void __user *arg)
{
	struct nvmap_set_tag_label op;
	struct nvmap_device *dev = nvmap_dev;
	int err;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (op.len > NVMAP_TAG_LABEL_MAXLEN)
		op.len = NVMAP_TAG_LABEL_MAXLEN;

	if (op.len)
		err = nvmap_define_tag(dev, op.tag,
			(const char __user *)op.addr, op.len);
	else
		err = nvmap_remove_tag(dev, op.tag);

	return err;
}

int nvmap_ioctl_get_available_heaps(struct file *filp, void __user *arg)
{
	struct nvmap_available_heaps op;
	int i;

	memset(&op, 0, sizeof(op));

	for (i = 0; i < nvmap_dev->nr_carveouts; i++)
		op.heaps |= nvmap_dev->heaps[i].heap_bit;

	if (copy_to_user(arg, &op, sizeof(op))) {
		pr_err("copy_to_user failed\n");
		return -EINVAL;
	}

	return 0;
}

int nvmap_ioctl_get_heap_size(struct file *filp, void __user *arg)
{
	struct nvmap_heap_size op;
	struct nvmap_heap *heap;
	int i;
	memset(&op, 0, sizeof(op));

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	for (i = 0; i < nvmap_dev->nr_carveouts; i++) {
		if (op.heap & nvmap_dev->heaps[i].heap_bit) {
			heap = nvmap_dev->heaps[i].carveout;
			op.size = nvmap_query_heap_size(heap);
			if (copy_to_user(arg, &op, sizeof(op)))
				return -EFAULT;
			return 0;
		}
	}
	return -ENODEV;

}
