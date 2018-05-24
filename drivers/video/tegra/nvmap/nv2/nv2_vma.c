/*
 * drivers/video/tegra/nvmap/nvmap_vma.c
 *
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <soc/tegra/chip-id.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

int NVMAP2_vma_belongs_to_handle(struct vm_area_struct *vma,
					struct nvmap_handle *h)
{
	struct nvmap_vma_priv *priv;

	priv = (struct nvmap_vma_priv *) vma->vm_private_data;

	return (priv->handle == h);
}

static void NVMAP2_zap_page_range(struct vm_area_struct *vma,
		unsigned long start, unsigned long size)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	zap_page_range(vma, start, size);
#else
	zap_page_range(vma, start, size, NULL);
#endif
}

void NVMAP2_vma_zap(struct list_head *vmas, u64 offset, u64 size)
{
	struct nvmap_vma_list *vma_list;
	struct vm_area_struct *vma;

	list_for_each_entry(vma_list, vmas, list) {
		struct nvmap_vma_priv *priv;
		size_t vm_size = size;

		vma = vma_list->vma;
		priv = vma->vm_private_data;
		if ((offset + size) > (vma->vm_end - vma->vm_start))
			vm_size = vma->vm_end - vma->vm_start - offset;

		if (priv->offs || vma->vm_pgoff) {
			/* vma mapping starts in the middle of handle memory.
			 * zapping needs special care. zap entire range for now.
			 * FIXME: optimze zapping.
			 */
			NVMAP2_zap_page_range(vma, vma->vm_start,
					vma->vm_end - vma->vm_start);
		} else {
			NVMAP2_zap_page_range(vma, vma->vm_start + offset,
						vm_size);
		}
	}
}
