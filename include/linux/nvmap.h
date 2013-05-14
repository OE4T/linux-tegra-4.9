/*
 * include/linux/nvmap.h
 *
 * structure declarations for nvmem and nvmap user-space ioctls
 *
 * Copyright (c) 2009-2013, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/ioctl.h>
#include <linux/file.h>
#include <linux/rbtree.h>
#if defined(__KERNEL__)
#include <linux/dma-buf.h>
#endif

#ifndef _LINUX_NVMAP_H
#define _LINUX_NVMAP_H

#define NVMAP_HEAP_IOVMM   (1ul<<30)

/* common carveout heaps */
#define NVMAP_HEAP_CARVEOUT_IRAM    (1ul<<29)
#define NVMAP_HEAP_CARVEOUT_VPR     (1ul<<28)
#define NVMAP_HEAP_CARVEOUT_TSEC    (1ul<<27)
#define NVMAP_HEAP_CARVEOUT_GENERIC (1ul<<0)

#define NVMAP_HEAP_CARVEOUT_MASK    (NVMAP_HEAP_IOVMM - 1)

/* allocation flags */
#define NVMAP_HANDLE_UNCACHEABLE     (0x0ul << 0)
#define NVMAP_HANDLE_WRITE_COMBINE   (0x1ul << 0)
#define NVMAP_HANDLE_INNER_CACHEABLE (0x2ul << 0)
#define NVMAP_HANDLE_CACHEABLE       (0x3ul << 0)
#define NVMAP_HANDLE_CACHE_FLAG      (0x3ul << 0)

#define NVMAP_HANDLE_SECURE          (0x1ul << 2)
#define NVMAP_HANDLE_ZEROED_PAGES    (0x1ul << 3)

struct nvmap_handle;

#if defined(__KERNEL__)

#if defined(CONFIG_TEGRA_NVMAP)
struct nvmap_client;
struct nvmap_device;

#define nvmap_ref_to_handle(_ref) (*(struct nvmap_handle **)(_ref))
/* Convert User space handle to Kernel. */
#define nvmap_convert_handle_u2k(h) (h)

/* handle_ref objects are client-local references to an nvmap_handle;
 * they are distinct objects so that handles can be unpinned and
 * unreferenced the correct number of times when a client abnormally
 * terminates */
struct nvmap_handle_ref {
	struct nvmap_handle *handle;
	struct rb_node	node;
	atomic_t	dupes;	/* number of times to free on file close */
	atomic_t	pin;	/* number of times to unpin on free */
};

#elif defined(CONFIG_ION_TEGRA)
/* For Ion Mem Manager support through nvmap_* API's. */
#include "../../../../../drivers/gpu/ion/ion_priv.h"
#define nvmap_client ion_client
#define nvmap_device ion_device
#define nvmap_handle ion_handle
#define nvmap_handle_ref ion_handle
#define nvmap_ref_to_handle(_ref) (struct ion_handle *)_ref
/* Convert User space handle to Kernel. */
#define nvmap_convert_handle_u2k(h) ({ \
	if ((u32)h >= TASK_SIZE) { \
		pr_err("Invalid user space handle."); \
		BUG(); \
	} \
	(*((u32 *)h)); })

#endif /* CONFIG_ION_TEGRA */

struct nvmap_client *nvmap_create_client(struct nvmap_device *dev,
					 const char *name);

struct nvmap_handle_ref *nvmap_alloc(struct nvmap_client *client, size_t size,
				     size_t align, unsigned int flags,
				     unsigned int heap_mask);

phys_addr_t _nvmap_get_addr_from_id(ulong user_id);

void nvmap_free(struct nvmap_client *client, struct nvmap_handle_ref *r);

void _nvmap_free(struct nvmap_client *client, struct nvmap_handle_ref *r);

void *nvmap_mmap(struct nvmap_handle_ref *r);

void nvmap_munmap(struct nvmap_handle_ref *r, void *addr);

void *nvmap_kmap(struct nvmap_handle_ref *r, unsigned int pagenum);

void nvmap_kunmap(struct nvmap_handle_ref *r, unsigned int pagenum,
		void *addr);

struct nvmap_client *nvmap_client_get_file(int fd);

struct nvmap_client *nvmap_client_get(struct nvmap_client *client);

void nvmap_client_put(struct nvmap_client *c);

phys_addr_t nvmap_pin(struct nvmap_client *c, struct nvmap_handle_ref *r);

phys_addr_t nvmap_handle_address(struct nvmap_client *c, unsigned long id);

phys_addr_t nvmap_handle_address_user_id(struct nvmap_client *c,
					 unsigned long user_id);

void nvmap_unpin(struct nvmap_client *client, struct nvmap_handle_ref *r);

struct nvmap_handle_ref *nvmap_duplicate_handle_user_id(
						struct nvmap_client *client,
						unsigned long user_id);

struct nvmap_handle_ref *_nvmap_duplicate_handle_user_id(
						struct nvmap_client *client,
						unsigned long user_id);

int nvmap_pin_array(struct nvmap_client *client,
		unsigned long	 *user_ids,
		long unsigned id_type_mask,
		long unsigned id_type,
		int nr,
		struct nvmap_handle **unique_arr,
		struct nvmap_handle_ref **unique_arr_refs);

struct nvmap_handle *nvmap_get_handle_user_id(struct nvmap_client *client,
					 unsigned long id);

void nvmap_handle_put(struct nvmap_handle *h);

struct nvmap_handle_ref *nvmap_alloc_iovm(struct nvmap_client *client,
	size_t size, size_t align, unsigned int flags, unsigned int iova_start);

void nvmap_free_iovm(struct nvmap_client *client, struct nvmap_handle_ref *r);

struct nvmap_platform_carveout {
	const char *name;
	unsigned int usage_mask;
	phys_addr_t base;
	size_t size;
	size_t buddy_size;
};

struct nvmap_platform_data {
	const struct nvmap_platform_carveout *carveouts;
	unsigned int nr_carveouts;
};

extern struct nvmap_device *nvmap_dev;

#endif /* __KERNEL__ */

/*
 * DOC: NvMap Userspace API
 *
 * create a client by opening /dev/nvmap
 * most operations handled via following ioctls
 *
 */
enum {
	NVMAP_HANDLE_PARAM_SIZE = 1,
	NVMAP_HANDLE_PARAM_ALIGNMENT,
	NVMAP_HANDLE_PARAM_BASE,
	NVMAP_HANDLE_PARAM_HEAP,
};

enum {
	NVMAP_CACHE_OP_WB = 0,
	NVMAP_CACHE_OP_INV,
	NVMAP_CACHE_OP_WB_INV,
};

struct nvmap_create_handle {
#ifdef CONFIG_COMPAT
	union {
		__u32 id;	/* FromId */
		__u32 size;	/* CreateHandle */
		__s32 fd;	/* DmaBufFd or FromFd */
	};
	__u32 handle;		/* returns nvmap handle */
#else
	union {
		unsigned long id;	/* FromId */
		__u32 size;	/* CreateHandle */
		__s32 fd;	/* DmaBufFd or FromFd */
	};
	struct nvmap_handle *handle; /* returns nvmap handle */
#endif
};

struct nvmap_alloc_handle {
#ifdef CONFIG_COMPAT
	__u32 handle;		/* nvmap handle */
#else
	struct nvmap_handle *handle; /* nvmap handle */
#endif
	__u32 heap_mask;	/* heaps to allocate from */
	__u32 flags;		/* wb/wc/uc/iwb etc. */
	__u32 align;		/* min alignment necessary */
};

struct nvmap_map_caller {
#ifdef CONFIG_COMPAT
	__u32 handle;		/* nvmap handle */
#else
	struct nvmap_handle *handle; /* nvmap handle */
#endif
	__u32 offset;		/* offset into hmem; should be page-aligned */
	__u32 length;		/* number of bytes to map */
	__u32 flags;		/* maps as wb/iwb etc. */
#ifdef CONFIG_COMPAT
	__u32 addr;		/* user pointer*/
#else
	unsigned long addr;	/* user pointer */
#endif
};

struct nvmap_rw_handle {
#ifdef CONFIG_COMPAT
	__u32 addr;		/* user pointer */
	__u32 handle;		/* nvmap handle */
#else
	unsigned long addr;	/* user pointer*/
	struct nvmap_handle *handle; /* nvmap handle */
#endif
	__u32 offset;		/* offset into hmem */
	__u32 elem_size;	/* individual atom size */
	__u32 hmem_stride;	/* delta in bytes between atoms in hmem */
	__u32 user_stride;	/* delta in bytes between atoms in user */
	__u32 count;		/* number of atoms to copy */
};

struct nvmap_pin_handle {
#ifdef CONFIG_COMPAT
	__u32 *handles;		/* array of handles to pin/unpin */
	__u32 *addr;		/*  array of addresses to return */
#else
	struct nvmap_handle **handles;	/* array of handles to pin/unpin */
	unsigned long *addr;	/* array of addresses to return */
#endif
	__u32 count;		/* number of entries in handles */
};

struct nvmap_handle_param {
#ifdef CONFIG_COMPAT
	__u32 handle;		/* nvmap handle */
#else
	struct nvmap_handle *handle;	/* nvmap handle */
#endif
	__u32 param;		/* size/align/base/heap etc. */
#ifdef CONFIG_COMPAT
	__u32 result;		/* returnes requested info*/
#else
	unsigned long result;	/* returnes requested info*/
#endif
};

struct nvmap_cache_op {
#ifdef CONFIG_COMPAT
	__u32 addr;		/* user pointer*/
	__u32 handle;		/* nvmap handle */
#else
	unsigned long addr;	/* user pointer*/
	struct nvmap_handle *handle;	/* nvmap handle */
#endif
	__u32 len;		/* bytes to flush */
	__s32 op;		/* wb/wb_inv/inv */
};

#define NVMAP_IOC_MAGIC 'N'

/* Creates a new memory handle. On input, the argument is the size of the new
 * handle; on return, the argument is the name of the new handle
 */
#define NVMAP_IOC_CREATE  _IOWR(NVMAP_IOC_MAGIC, 0, struct nvmap_create_handle)
#define NVMAP_IOC_CLAIM   _IOWR(NVMAP_IOC_MAGIC, 1, struct nvmap_create_handle)
#define NVMAP_IOC_FROM_ID _IOWR(NVMAP_IOC_MAGIC, 2, struct nvmap_create_handle)

/* Actually allocates memory for the specified handle */
#define NVMAP_IOC_ALLOC    _IOW(NVMAP_IOC_MAGIC, 3, struct nvmap_alloc_handle)

/* Frees a memory handle, unpinning any pinned pages and unmapping any mappings
 */
#define NVMAP_IOC_FREE       _IO(NVMAP_IOC_MAGIC, 4)

/* Maps the region of the specified handle into a user-provided virtual address
 * that was previously created via an mmap syscall on this fd */
#define NVMAP_IOC_MMAP       _IOWR(NVMAP_IOC_MAGIC, 5, struct nvmap_map_caller)

/* Reads/writes data (possibly strided) from a user-provided buffer into the
 * hmem at the specified offset */
#define NVMAP_IOC_WRITE      _IOW(NVMAP_IOC_MAGIC, 6, struct nvmap_rw_handle)
#define NVMAP_IOC_READ       _IOW(NVMAP_IOC_MAGIC, 7, struct nvmap_rw_handle)

#define NVMAP_IOC_PARAM _IOWR(NVMAP_IOC_MAGIC, 8, struct nvmap_handle_param)

/* Pins a list of memory handles into IO-addressable memory (either IOVMM
 * space or physical memory, depending on the allocation), and returns the
 * address. Handles may be pinned recursively. */
#define NVMAP_IOC_PIN_MULT   _IOWR(NVMAP_IOC_MAGIC, 10, struct nvmap_pin_handle)
#define NVMAP_IOC_UNPIN_MULT _IOW(NVMAP_IOC_MAGIC, 11, struct nvmap_pin_handle)

#define NVMAP_IOC_CACHE      _IOW(NVMAP_IOC_MAGIC, 12, struct nvmap_cache_op)

/* Returns a global ID usable to allow a remote process to create a handle
 * reference to the same handle */
#define NVMAP_IOC_GET_ID  _IOWR(NVMAP_IOC_MAGIC, 13, struct nvmap_create_handle)

/* Returns a dma-buf fd usable to allow a remote process to create a handle
 * reference to the same handle */
#define NVMAP_IOC_SHARE  _IOWR(NVMAP_IOC_MAGIC, 14, struct nvmap_create_handle)

/* Returns a file id that allows a remote process to create a handle
 * reference to the same handle */
#define NVMAP_IOC_GET_FD  _IOWR(NVMAP_IOC_MAGIC, 15, struct nvmap_create_handle)

/* Create a new memory handle from file id passed */
#define NVMAP_IOC_FROM_FD _IOWR(NVMAP_IOC_MAGIC, 16, struct nvmap_create_handle)

#define NVMAP_IOC_MAXNR (_IOC_NR(NVMAP_IOC_FROM_FD))

#endif /* _LINUX_NVMAP_H */
