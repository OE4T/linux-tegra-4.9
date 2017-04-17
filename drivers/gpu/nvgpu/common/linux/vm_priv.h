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

#ifndef __COMMON_LINUX_VM_PRIV_H__
#define __COMMON_LINUX_VM_PRIV_H__

#include <nvgpu/types.h>

struct sg_table;
struct dma_buf;

struct vm_gk20a;
struct vm_gk20a_mapping_batch;

u64 nvgpu_vm_map(struct vm_gk20a *vm,
		 struct dma_buf *dmabuf,
		 u64 offset_align,
		 u32 flags /*NVGPU_AS_MAP_BUFFER_FLAGS_*/,
		 int kind,
		 struct sg_table **sgt,
		 bool user_mapped,
		 int rw_flag,
		 u64 buffer_offset,
		 u64 mapping_size,
		 struct vm_gk20a_mapping_batch *mapping_batch);

int nvgpu_vm_map_compbits(struct vm_gk20a *vm,
			  u64 mapping_gva,
			  u64 *compbits_win_gva,
			  u64 *mapping_iova,
			  u32 flags);

/* Note: batch may be NULL if map op is not part of a batch */
int nvgpu_vm_map_buffer(struct vm_gk20a *vm,
			int dmabuf_fd,
			u64 *offset_align,
			u32 flags, /* NVGPU_AS_MAP_BUFFER_FLAGS_ */
			int kind,
			u64 buffer_offset,
			u64 mapping_size,
			struct vm_gk20a_mapping_batch *batch);

void nvgpu_vm_unmap(struct vm_gk20a *vm, u64 offset);

/* find buffer corresponding to va */
int nvgpu_vm_find_buffer(struct vm_gk20a *vm, u64 gpu_va,
			 struct dma_buf **dmabuf,
			 u64 *offset);
#endif
