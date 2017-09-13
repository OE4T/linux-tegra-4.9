/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __NVGPU_VM_H__
#define __NVGPU_VM_H__

#include <nvgpu/kref.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/types.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/allocator.h>

struct vm_gk20a;
struct nvgpu_vm_area;
struct buffer_attrs;
struct gk20a_comptag_allocator;

/**
 * This header contains the OS agnostic APIs for dealing with VMs. Most of the
 * VM implementation is system specific - it must translate from a platform's
 * representation of DMA'able memory to our nvgpu_mem notion.
 *
 * However, some stuff is platform agnostic. VM ref-counting and the VM struct
 * itself are platform agnostic. Also, the initialization and destruction of
 * VMs is the same across all platforms (for now).
 *
 * VM Architecture:
 * ----------------
 *
 *   The VM managment in nvgpu is split up as follows: a vm_gk20a struct which
 * defines an address space. Each address space is a set of page tables and a
 * GPU Virtual Address (GVA) allocator. Any number of channels may bind to a VM.
 *
 *     +----+  +----+     +----+     +-----+     +-----+
 *     | C1 |  | C2 | ... | Cn |     | VM1 | ... | VMn |
 *     +-+--+  +-+--+     +-+--+     +--+--+     +--+--+
 *       |       |          |           |           |
 *       |       |          +----->-----+           |
 *       |       +---------------->-----+           |
 *       +------------------------>-----------------+
 *
 *   Each VM also manages a set of mapped buffers (struct nvgpu_mapped_buf)
 * which corresponds to _user space_ buffers which have been mapped into this VM.
 * Kernel space mappings (created by nvgpu_gmmu_map()) are not tracked by VMs.
 * This may be an architectural bug, but for now it seems to be OK. VMs can be
 * closed in various ways - refs counts hitting zero, direct calls to the remove
 * routine, etc. Note: this is going to change. VM cleanup is going to be
 * homogonized around ref-counts. When a VM is closed all mapped buffers in the
 * VM are unmapped from the GMMU. This means that those mappings will no longer
 * be valid and any subsequent access by the GPU will fault. That means one must
 * ensure the VM is not in use before closing it.
 *
 *   VMs may also contain VM areas (struct nvgpu_vm_area) which are created for
 * the purpose of sparse and/or fixed mappings. If userspace wishes to create a
 * fixed mapping it must first create a VM area - either with a fixed address or
 * not. VM areas are reserved - other mapping operations will not use the space.
 * Userspace may then create fixed mappings within that VM area.
 */

/* map/unmap batch state */
struct vm_gk20a_mapping_batch {
	bool gpu_l2_flushed;
	bool need_tlb_invalidate;
};

struct nvgpu_mapped_buf {
	struct vm_gk20a *vm;
	struct nvgpu_vm_area *vm_area;

	struct nvgpu_rbtree_node node;
	struct nvgpu_list_node buffer_list;
	u64 addr;
	u64 size;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	struct nvgpu_ref ref;
	u32 user_mapped;
	bool own_mem_ref;
	u32 pgsz_idx;
	u32 ctag_offset;
	u32 ctag_lines;
	u32 ctag_allocated_lines;

	u32 flags;
	u32 kind;
	bool va_allocated;
};

static inline struct nvgpu_mapped_buf *
nvgpu_mapped_buf_from_buffer_list(struct nvgpu_list_node *node)
{
	return (struct nvgpu_mapped_buf *)
		((uintptr_t)node - offsetof(struct nvgpu_mapped_buf,
					    buffer_list));
}

static inline struct nvgpu_mapped_buf *
mapped_buffer_from_rbtree_node(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_mapped_buf *)
		  ((uintptr_t)node - offsetof(struct nvgpu_mapped_buf, node));
}

struct vm_gk20a {
	struct mm_gk20a *mm;
	struct gk20a_as_share *as_share; /* as_share this represents */
	char name[20];

	u64 va_start;
	u64 va_limit;

	int num_user_mapped_buffers;

	bool big_pages;   /* enable large page support */
	bool enable_ctag;

	u32 big_page_size;

	bool userspace_managed;

	const struct gk20a_mmu_level *mmu_levels;

	struct nvgpu_ref ref;

	struct nvgpu_mutex update_gmmu_lock;

	struct nvgpu_gmmu_pd pdb;

	/*
	 * These structs define the address spaces. In some cases it's possible
	 * to merge address spaces (user and user_lp) and in other cases it's
	 * not. vma[] allows the code to be agnostic to this by always using
	 * address spaces through this pointer array.
	 */
	struct nvgpu_allocator *vma[gmmu_nr_page_sizes];
	struct nvgpu_allocator kernel;
	struct nvgpu_allocator user;
	struct nvgpu_allocator user_lp;

	struct nvgpu_rbtree_node *mapped_buffers;

	struct nvgpu_list_node vm_area_list;

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	u64 handle;
#endif
	u32 gmmu_page_sizes[gmmu_nr_page_sizes];

	/* if non-NULL, kref_put will use this batch when
	   unmapping. Must hold vm->update_gmmu_lock. */
	struct vm_gk20a_mapping_batch *kref_put_batch;

	/*
	 * Each address space needs to have a semaphore pool.
	 */
	struct nvgpu_semaphore_pool *sema_pool;
};

void nvgpu_vm_get(struct vm_gk20a *vm);
void nvgpu_vm_put(struct vm_gk20a *vm);

int vm_aspace_id(struct vm_gk20a *vm);
int nvgpu_big_pages_possible(struct vm_gk20a *vm, u64 base, u64 size);

/* batching eliminates redundant cache flushes and invalidates */
void nvgpu_vm_mapping_batch_start(struct vm_gk20a_mapping_batch *batch);
void nvgpu_vm_mapping_batch_finish(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *batch);
/* called when holding vm->update_gmmu_lock */
void nvgpu_vm_mapping_batch_finish_locked(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *batch);

/* get reference to all currently mapped buffers */
int nvgpu_vm_get_buffers(struct vm_gk20a *vm,
			 struct nvgpu_mapped_buf ***mapped_buffers,
			 int *num_buffers);
/* put references on the given buffers */
void nvgpu_vm_put_buffers(struct vm_gk20a *vm,
			  struct nvgpu_mapped_buf **mapped_buffers,
			  int num_buffers);

/* Note: batch may be NULL if unmap op is not part of a batch */
int nvgpu_vm_unmap_buffer(struct vm_gk20a *vm, u64 offset,
			  struct vm_gk20a_mapping_batch *batch);

void nvgpu_vm_unmap_locked(struct nvgpu_mapped_buf *mapped_buffer,
			   struct vm_gk20a_mapping_batch *batch);

/*
 * These all require the VM update lock to be held.
 */
struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf(
	struct vm_gk20a *vm, u64 addr);
struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_range(
	struct vm_gk20a *vm, u64 addr);
struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_less_than(
	struct vm_gk20a *vm, u64 addr);

int nvgpu_vm_find_buf(struct vm_gk20a *vm, u64 gpu_va,
		      struct dma_buf **dmabuf,
		      u64 *offset);
int nvgpu_insert_mapped_buf(struct vm_gk20a *vm,
			    struct nvgpu_mapped_buf *mapped_buffer);
void nvgpu_remove_mapped_buf(struct vm_gk20a *vm,
			     struct nvgpu_mapped_buf *mapped_buffer);

struct vm_gk20a *nvgpu_vm_init(struct gk20a *g,
			       u32 big_page_size,
			       u64 low_hole,
			       u64 kernel_reserved,
			       u64 aperture_size,
			       bool big_pages,
			       bool userspace_managed,
			       char *name);

/*
 * These are private to the VM code but are unfortunately used by the vgpu code.
 * It appears to be used for an optimization in reducing the number of server
 * requests to the vgpu server. Basically the vgpu implementation of
 * map_global_ctx_buffers() sends a bunch of VA ranges over to the RM server.
 * Ideally the RM server can just batch mappings but until such a time this
 * will be used by the vgpu code.
 */
u64 __nvgpu_vm_alloc_va(struct vm_gk20a *vm, u64 size,
			enum gmmu_pgsz_gk20a pgsz_idx);
int __nvgpu_vm_free_va(struct vm_gk20a *vm, u64 addr,
		       enum gmmu_pgsz_gk20a pgsz_idx);

#endif
