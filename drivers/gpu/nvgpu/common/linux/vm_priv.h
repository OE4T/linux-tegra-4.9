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

struct buffer_attrs {
	struct sg_table *sgt;
	u64 size;
	u64 align;
	u32 ctag_offset;
	u32 ctag_lines;
	u32 ctag_allocated_lines;
	int pgsz_idx;
	u8 kind_v;
	u8 uc_kind_v;
	bool ctag_user_mappable;
};

u64 nvgpu_vm_map(struct vm_gk20a *vm,
		 struct dma_buf *dmabuf,
		 u64 offset_align,
		 u32 flags,
		 int kind,
		 bool user_mapped,
		 int rw_flag,
		 u64 buffer_offset,
		 u64 mapping_size,
		 struct vm_gk20a_mapping_batch *mapping_batch);

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

enum nvgpu_aperture gk20a_dmabuf_aperture(struct gk20a *g,
					  struct dma_buf *dmabuf);
int validate_fixed_buffer(struct vm_gk20a *vm,
			  struct buffer_attrs *bfr,
			  u64 map_offset, u64 map_size,
			  struct nvgpu_vm_area **pva_node);
int setup_buffer_kind_and_compression(struct vm_gk20a *vm,
				      u32 flags,
				      struct buffer_attrs *bfr,
				      enum gmmu_pgsz_gk20a pgsz_idx);
int gk20a_alloc_comptags(struct gk20a *g,
			 struct device *dev,
			 struct dma_buf *dmabuf,
			 struct gk20a_comptag_allocator *allocator,
			 u32 lines);

#endif
