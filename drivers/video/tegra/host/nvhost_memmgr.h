/*
 * drivers/video/tegra/host/nvhost_memmgr.h
 *
 * Tegra Graphics Host Memory Management Abstraction header
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _NVHOST_MEM_MGR_H_
#define _NVHOST_MEM_MGR_H_

struct nvhost_chip_support;
struct mem_mgr;
struct mem_handle;
struct platform_device;

struct nvhost_job_unpin {
	struct mem_handle *h;
	struct sg_table *mem;
};

enum mem_mgr_flag {
	mem_mgr_flag_uncacheable = 0,
	mem_mgr_flag_write_combine = 1,
};

enum mem_mgr_type {
	mem_mgr_type_nvmap = 0,
	mem_mgr_type_dmabuf = 1,
};

#define MEMMGR_TYPE_MASK	0x3
#define MEMMGR_ID_MASK		~0x3

int nvhost_memmgr_init(struct nvhost_chip_support *chip);
struct mem_mgr *nvhost_memmgr_alloc_mgr(void);
void nvhost_memmgr_put_mgr(struct mem_mgr *);
struct mem_mgr *nvhost_memmgr_get_mgr(struct mem_mgr *);
struct mem_mgr *nvhost_memmgr_get_mgr_file(int fd);
struct mem_handle *nvhost_memmgr_alloc(struct mem_mgr *,
		size_t size, size_t align,
		int flags);
struct mem_handle *nvhost_memmgr_get(struct mem_mgr *,
		u32 id, struct platform_device *dev);
void nvhost_memmgr_put(struct mem_mgr *mgr, struct mem_handle *handle);
struct sg_table *nvhost_memmgr_pin(struct mem_mgr *,
		struct mem_handle *handle,
		struct device *dev);
void nvhost_memmgr_unpin(struct mem_mgr *mgr,
		struct mem_handle *handle, struct device *dev,
		struct sg_table *sgt);
void *nvhost_memmgr_mmap(struct mem_handle *handle);
void nvhost_memmgr_munmap(struct mem_handle *handle, void *addr);
void *nvhost_memmgr_kmap(struct mem_handle *handle, unsigned int pagenum);
void nvhost_memmgr_kunmap(struct mem_handle *handle, unsigned int pagenum,
		void *addr);
static inline int nvhost_memmgr_type(u32 id) { return id & MEMMGR_TYPE_MASK; }
static inline int nvhost_memmgr_id(u32 id) { return id & MEMMGR_ID_MASK; }
u32 nvhost_memmgr_handle_to_id(struct mem_handle *handle);

int nvhost_memmgr_pin_array_ids(struct mem_mgr *mgr,
		struct platform_device *dev,
		u32 *ids,
		dma_addr_t *phys_addr,
		u32 count,
		struct nvhost_job_unpin *unpin_data);

#endif
