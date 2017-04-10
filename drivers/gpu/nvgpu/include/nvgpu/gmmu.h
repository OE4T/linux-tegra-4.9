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

/*
 * This is the GMMU API visible to blocks outside of the GMMU. Basically this
 * API supports all the different types of mappings that might be done in the
 * GMMU.
 */

struct vm_gk20a;
struct nvgpu_mem;

enum nvgpu_aperture;

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
