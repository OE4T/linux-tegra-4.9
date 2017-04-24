/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
struct vm_reserved_va_node;
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
 */

/* map/unmap batch state */
struct vm_gk20a_mapping_batch {
	bool gpu_l2_flushed;
	bool need_tlb_invalidate;
};

struct nvgpu_mapped_buf {
	struct vm_gk20a *vm;
	struct nvgpu_rbtree_node node;
	struct nvgpu_list_node buffer_list;
	struct vm_reserved_va_node *va_node;
	u64 addr;
	u64 size;
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	struct kref ref;
	u32 user_mapped;
	bool own_mem_ref;
	u32 pgsz_idx;
	u32 ctag_offset;
	u32 ctag_lines;
	u32 ctag_allocated_lines;

	/* For comptag mapping, these are the mapping window parameters */
	bool ctags_mappable;
	u64 ctag_map_win_addr; /* non-zero if mapped */
	u64 ctag_map_win_size; /* non-zero if ctags_mappable */
	u32 ctag_map_win_ctagline; /* ctagline at win start, set if
				    * ctags_mappable */

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

	u64 va_start;
	u64 va_limit;

	int num_user_mapped_buffers;

	bool big_pages;   /* enable large page support */
	bool enable_ctag;
	bool mapped;

	u32 big_page_size;

	bool userspace_managed;

	const struct gk20a_mmu_level *mmu_levels;

	struct kref ref;

	struct nvgpu_mutex update_gmmu_lock;

	struct gk20a_mm_entry pdb;

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

	struct nvgpu_list_node reserved_va_list;

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

void nvgpu_vm_remove_support_nofree(struct vm_gk20a *vm);
void nvgpu_vm_remove_support(struct vm_gk20a *vm);

void nvgpu_remove_vm(struct vm_gk20a *vm, struct nvgpu_mem *inst_block);

int nvgpu_init_vm(struct mm_gk20a *mm,
		struct vm_gk20a *vm,
		u32 big_page_size,
		u64 low_hole,
		u64 kernel_reserved,
		u64 aperture_size,
		bool big_pages,
		bool userspace_managed,
		char *name);
void nvgpu_deinit_vm(struct vm_gk20a *vm);

#endif
