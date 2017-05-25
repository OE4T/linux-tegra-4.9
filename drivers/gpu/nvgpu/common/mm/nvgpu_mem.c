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

#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>

#include "gk20a/gk20a.h"

struct nvgpu_mem_sgl *nvgpu_mem_sgl_next(struct nvgpu_mem_sgl *sgl)
{
	return sgl->next;
}

u64 nvgpu_mem_sgl_phys(struct nvgpu_mem_sgl *sgl)
{
	return sgl->phys;
}

u64 nvgpu_mem_sgl_dma(struct nvgpu_mem_sgl *sgl)
{
	return sgl->dma;
}

u64 nvgpu_mem_sgl_length(struct nvgpu_mem_sgl *sgl)
{
	return sgl->length;
}

/*
 * This builds a GPU address for the %sgl based on whether an IOMMU is present
 * or not. It also handles turning the physical address into the true GPU
 * physical address that should be programmed into the page tables.
 */
u64 nvgpu_mem_sgl_gpu_addr(struct gk20a *g, struct nvgpu_mem_sgl *sgl,
			   struct nvgpu_gmmu_attrs *attrs)
{
	if (nvgpu_mem_sgl_dma(sgl) == 0)
		return g->ops.mm.gpu_phys_addr(g, attrs,
					       nvgpu_mem_sgl_phys(sgl));

	if (nvgpu_mem_sgl_dma(sgl) == DMA_ERROR_CODE)
		return 0;

	return gk20a_mm_smmu_vaddr_translate(g, nvgpu_mem_sgl_dma(sgl));
}

void nvgpu_mem_sgl_free(struct gk20a *g, struct nvgpu_mem_sgl *sgl)
{
	struct nvgpu_mem_sgl *next;

	/*
	 * Free each of the elements. We expect each element to have been
	 * nvgpu_k[mz]alloc()ed.
	 */
	while (sgl) {
		next = nvgpu_mem_sgl_next(sgl);
		nvgpu_kfree(g, sgl);
		sgl = next;
	}
}
