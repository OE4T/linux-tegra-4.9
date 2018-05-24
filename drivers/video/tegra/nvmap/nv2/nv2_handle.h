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

#ifndef __NVMAP2_HANDLE_H
#define __NVMAP2_HANDLE_H


struct nvmap_handle *NVMAP2_handle_create(size_t size);
struct nvmap_handle *NVMAP2_handle_create_from_dmabuf(
			struct nvmap_client * client, struct dma_buf *dmabuf);
void NVMAP2_handle_add_owner(struct nvmap_handle *handle,
					struct nvmap_client *client);
void NVMAP2_handle_destroy(struct nvmap_handle *handle);
void NVMAP2_handle_install_fd(struct nvmap_handle *handle, int fd);

struct nvmap_handle *NVMAP2_handle_get(struct nvmap_handle *h);
void NVMAP2_handle_put(struct nvmap_handle *h);

int NVMAP2_handle_alloc(struct nvmap_handle *h,
				unsigned int heap_mask,
				size_t align,
				u8 kind,
				unsigned int flags,
				int peer);

int NVMAP2_handle_alloc_from_ivmid(struct nvmap_handle *handle, u64 ivm_id);

int NVMAP2_handle_alloc_carveout(struct nvmap_handle *handle,
					      unsigned long type,
					      phys_addr_t *start);
int NVMAP2_handle_alloc_from_va(struct nvmap_handle *h,
			       ulong addr,
			       unsigned int flags);
int NVMAP2_handle_alloc_from_ivmid(struct nvmap_handle *handle, u64 ivm_id);

struct nvmap_handle *NVMAP2_handle_from_fd(int fd);
struct nvmap_handle *NVMAP2_handle_from_ivmid(u64 ivm_id);

int NVMAP2_handle_cache_maint(struct nvmap_handle *handle, unsigned long start,
		unsigned long end, unsigned int op);
int NVMAP2_handles_cache_maint(struct nvmap_handle **handles,
				u64 *offsets, u64 *sizes, int op, int nr);
void NVMAP2_handle_zap(struct nvmap_handle *handle, u64 offset, u64 size);

void *NVMAP2_handle_mmap(struct nvmap_handle *h);
void NVMAP2_handle_munmap(struct nvmap_handle *h, void *addr);

int NVMAP2_handles_reserve(struct nvmap_handle **handles, u64 *offsets,
						u64 *sizes, int op, int nr);
ssize_t NVMAP2_handle_rw(struct nvmap_handle *h,
			 unsigned long h_offs, unsigned long h_stride,
			 unsigned long sys_addr, unsigned long sys_stride,
			 unsigned long elem_size, unsigned long count,
			 int is_read);
int NVMAP2_handle_owns_vma(struct nvmap_handle *h, struct vm_area_struct *vma);

#endif /* __NVMAP2_HANDLE_H */
