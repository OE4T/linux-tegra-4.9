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

#ifndef __NVGPU_VM_AREA_H__
#define __NVGPU_VM_AREA_H__

#include <nvgpu/list.h>
#include <nvgpu/types.h>

struct vm_gk20a;
struct gk20a_as_share;
struct nvgpu_as_alloc_space_args;
struct nvgpu_as_free_space_args;

struct nvgpu_vm_area {
	/*
	 * Entry into the list of VM areas owned by a VM.
	 */
	struct nvgpu_list_node vm_area_list;

	/*
	 * List of buffers mapped into this vm_area.
	 */
	struct nvgpu_list_node buffer_list_head;

	u32 flags;
	u32 pgsz_idx;
	u64 addr;
	u64 size;
	bool sparse;
};

static inline struct nvgpu_vm_area *
nvgpu_vm_area_from_vm_area_list(struct nvgpu_list_node *node)
{
	return (struct nvgpu_vm_area *)
		((uintptr_t)node - offsetof(struct nvgpu_vm_area,
					    vm_area_list));
};

int nvgpu_vm_area_alloc(struct vm_gk20a *vm, u32 pages, u32 page_size,
			u64 *addr, u32 flags);
int nvgpu_vm_area_free(struct vm_gk20a *vm, u64 addr);

struct nvgpu_vm_area *nvgpu_vm_area_find(struct vm_gk20a *vm, u64 addr);
int nvgpu_vm_area_validate_buffer(struct vm_gk20a *vm,
				  u64 map_offset, u64 map_size, int pgsz_idx,
				  struct nvgpu_vm_area **pvm_area);

#endif
