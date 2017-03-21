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

#ifndef __NVGPU_DMA_H__
#define __NVGPU_DMA_H__

#include <nvgpu/types.h>

struct gk20a;
struct vm_gk20a;
struct nvgpu_mem;

/*
 * Flags for the below gk20a_gmmu_{alloc,alloc_map}_flags*
 */

/*
 * Don't create a virtual kernel mapping for the buffer but only allocate it;
 * this may save some resources. The buffer can be mapped later explicitly.
 */
#define NVGPU_DMA_NO_KERNEL_MAPPING	(1 << 0)

/*
 * Don't allow building the buffer from individual pages but require a
 * physically contiguous block.
 */
#define NVGPU_DMA_FORCE_CONTIGUOUS	(1 << 1)

/*
 * Make the mapping read-only.
 */
#define NVGPU_DMA_READ_ONLY		(1 << 2)

/**
 * gk20a_gmmu_alloc - Allocate DMA memory
 *
 * @g    - The GPU.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * memory can be either placed in VIDMEM or SYSMEM, which ever is more
 * convenient for the driver.
 */
int gk20a_gmmu_alloc(struct gk20a *g, size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_flags - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * memory can be either placed in VIDMEM or SYSMEM, which ever is more
 * convenient for the driver.
 *
 * The following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int gk20a_gmmu_alloc_flags(struct gk20a *g, unsigned long flags, size_t size,
		struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_sys - Allocate DMA memory
 *
 * @g    - The GPU.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in SYSMEM.
 */
int gk20a_gmmu_alloc_sys(struct gk20a *g, size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_flags_sys - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in SYSMEM.
 *
 * The following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int gk20a_gmmu_alloc_flags_sys(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_vid - Allocate DMA memory
 *
 * @g    - The GPU.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 */
int gk20a_gmmu_alloc_vid(struct gk20a *g, size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_flags_vid - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 *
 * Only the following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *
 */
int gk20a_gmmu_alloc_flags_vid(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_flags_vid_at - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 * @at    - A specific location to attempt to allocate memory from or 0 if the
 *          caller does not care what the address is.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 *
 * Only the following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 */
int gk20a_gmmu_alloc_flags_vid_at(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem, dma_addr_t at);

/**
 * gk20a_gmmu_free - Free a DMA allocation
 *
 * @g   - The GPU.
 * @mem - An allocation to free.
 *
 * Free memory created with any of:
 *
 *   gk20a_gmmu_alloc()
 *   gk20a_gmmu_alloc_flags()
 *   gk20a_gmmu_alloc_sys()
 *   gk20a_gmmu_alloc_flags_sys()
 *   gk20a_gmmu_alloc_vid()
 *   gk20a_gmmu_alloc_flags_vid()
 *   gk20a_gmmu_alloc_flags_vid_at()
 */
void gk20a_gmmu_free(struct gk20a *g, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_map - Allocate DMA memory and map into GMMU.
 *
 * @vm   - VM context for GMMU mapping.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * Note this is different than mapping it into the CPU. This memory can be
 * either placed in VIDMEM or SYSMEM, which ever is more convenient for the
 * driver.
 */
int gk20a_gmmu_alloc_map(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_map_flags - Allocate DMA memory and map into GMMU.
 *
 * @vm    - VM context for GMMU mapping.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * Note this is different than mapping it into the CPU. This memory can be
 * either placed in VIDMEM or SYSMEM, which ever is more convenient for the
 * driver.
 *
 * This version passes @flags on to the underlying DMA allocation. The accepted
 * flags are:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int gk20a_gmmu_alloc_map_flags(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_map_sys - Allocate DMA memory and map into GMMU.
 *
 * @vm   - VM context for GMMU mapping.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in SYSMEM.
 */
int gk20a_gmmu_alloc_map_sys(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_map_flags_sys - Allocate DMA memory and map into GMMU.
 *
 * @vm    - VM context for GMMU mapping.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in SYSMEM.
 *
 * This version passes @flags on to the underlying DMA allocation. The accepted
 * flags are:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int gk20a_gmmu_alloc_map_flags_sys(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_map_vid - Allocate DMA memory and map into GMMU.
 *
 * @vm   - VM context for GMMU mapping.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in VIDMEM.
 */
int gk20a_gmmu_alloc_map_vid(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_alloc_map_flags_vid - Allocate DMA memory and map into GMMU.
 *
 * @vm    - VM context for GMMU mapping.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in VIDMEM.
 *
 * This version passes @flags on to the underlying DMA allocation. The accepted
 * flags are:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int gk20a_gmmu_alloc_map_flags_vid(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * gk20a_gmmu_unmap_free - Free a DMA allocation
 *
 * @g   - The GPU.
 * @mem - An allocation to free.
 *
 * Free memory created with any of:
 *
 *   gk20a_gmmu_alloc_map()
 *   gk20a_gmmu_alloc_map_flags()
 *   gk20a_gmmu_alloc_map_sys()
 *   gk20a_gmmu_alloc_map_flags_sys()
 *   gk20a_gmmu_alloc_map_vid()
 *   gk20a_gmmu_alloc_map_flags_vid()
 */
void gk20a_gmmu_unmap_free(struct vm_gk20a *vm, struct nvgpu_mem *mem);

#endif
