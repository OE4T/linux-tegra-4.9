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

#include "nvmap_structs.h"

// TODO pretty sure list and priv can be merged into one structure
struct nvmap_vma_list {
	struct list_head list;
	struct vm_area_struct *vma;
	unsigned long save_vm_flags;
	pid_t pid;
	atomic_t ref;
};

struct nvmap_vma_priv {
	struct nvmap_handle *handle;
	size_t		offs;
	atomic_t	count;	/* number of processes cloning the VMA */
};

int NVMAP2_vma_is_nvmap(struct vm_area_struct *vma);

int NVMAP2_vma_belongs_to_handle(struct vm_area_struct *vma,
					struct nvmap_handle *h);
void NVMAP2_vma_zap(struct list_head *vmas, u64 offset, u64 size);

int NVMAP2_vma_list_prot(struct nvmap_vma_list *vma_list, u64 offset,
					u64 size, int handle_is_dirty, int op);

#endif /* __NVMAP2_VMA_H */
