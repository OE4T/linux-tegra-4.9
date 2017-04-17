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

#ifndef __NVGPU_GMMU_H__
#define __NVGPU_GMMU_H__

#include <nvgpu/types.h>
#include <nvgpu/nvgpu_mem.h>

struct scatterlist;

/*
 * This is the GMMU API visible to blocks outside of the GMMU. Basically this
 * API supports all the different types of mappings that might be done in the
 * GMMU.
 */

struct vm_gk20a;
struct nvgpu_mem;

enum gmmu_pgsz_gk20a {
	gmmu_page_size_small  = 0,
	gmmu_page_size_big    = 1,
	gmmu_page_size_kernel = 2,
	gmmu_nr_page_sizes    = 3,
};

struct gk20a_mm_entry {
	/* backing for */
	struct nvgpu_mem mem;
	u32 woffset; /* if >0, mem is a shadow copy, owned by another entry */
	int pgsz;
	struct gk20a_mm_entry *entries;
	int num_entries;
};

struct gk20a_mmu_level {
	int hi_bit[2];
	int lo_bit[2];
	int (*update_entry)(struct vm_gk20a *vm,
			   struct gk20a_mm_entry *pte,
			   u32 i, u32 gmmu_pgsz_idx,
			   struct scatterlist **sgl,
			   u64 *offset,
			   u64 *iova,
			   u32 kind_v, u64 *ctag,
			   bool cacheable, bool unmapped_pte,
			   int rw_flag, bool sparse, bool priv,
			   enum nvgpu_aperture aperture);
	size_t entry_size;
};

/**
 * nvgpu_gmmu_map - Map memory into the GMMU.
 *
 * Kernel space.
 */
u64 nvgpu_gmmu_map(struct vm_gk20a *vm,
		   struct nvgpu_mem *mem,
		   u64 size,
		   u32 flags,
		   int rw_flag,
		   bool priv,
		   enum nvgpu_aperture aperture);

/**
 * nvgpu_gmmu_map_fixed - Map memory into the GMMU.
 *
 * Kernel space.
 */
u64 nvgpu_gmmu_map_fixed(struct vm_gk20a *vm,
			 struct nvgpu_mem *mem,
			 u64 addr,
			 u64 size,
			 u32 flags,
			 int rw_flag,
			 bool priv,
			 enum nvgpu_aperture aperture);

/**
 * nvgpu_gmmu_unmap - Unmap a buffer.
 *
 * Kernel space.
 */
void nvgpu_gmmu_unmap(struct vm_gk20a *vm,
		      struct nvgpu_mem *mem,
		      u64 gpu_va);

#endif
