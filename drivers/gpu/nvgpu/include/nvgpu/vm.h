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
struct mapped_buffer_node;

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

/* batching eliminates redundant cache flushes and invalidates */
void nvgpu_vm_mapping_batch_start(struct vm_gk20a_mapping_batch *batch);
void nvgpu_vm_mapping_batch_finish(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *batch);
/* called when holding vm->update_gmmu_lock */
void nvgpu_vm_mapping_batch_finish_locked(
	struct vm_gk20a *vm, struct vm_gk20a_mapping_batch *batch);

/* get reference to all currently mapped buffers */
int nvgpu_vm_get_buffers(struct vm_gk20a *vm,
			 struct mapped_buffer_node ***mapped_buffers,
			 int *num_buffers);

/* put references on the given buffers */
void nvgpu_vm_put_buffers(struct vm_gk20a *vm,
			  struct mapped_buffer_node **mapped_buffers,
			  int num_buffers);

/* Note: batch may be NULL if unmap op is not part of a batch */
int nvgpu_vm_unmap_buffer(struct vm_gk20a *vm, u64 offset,
			  struct vm_gk20a_mapping_batch *batch);

void nvgpu_vm_unmap_locked(struct mapped_buffer_node *mapped_buffer,
			   struct vm_gk20a_mapping_batch *batch);

void nvgpu_vm_remove_support_nofree(struct vm_gk20a *vm);
void nvgpu_vm_remove_support(struct vm_gk20a *vm);

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
