/*
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

#ifndef __NVMAP2_VMA_H
#define __NVMAP2_VMA_H

int NVMAP2_vma_belongs_to_handle(struct vm_area_struct *vma,
					struct nvmap_handle *h);
void NVMAP2_vma_zap(struct list_head *vmas, u64 offset, u64 size);

#endif /* __NVMAP2_VMA_H */
