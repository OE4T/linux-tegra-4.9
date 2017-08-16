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

#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/dma.h>

#include "gk20a/gk20a.h"

void *nvgpu_sgt_get_next(struct nvgpu_sgt *sgt, void *sgl)
{
	return sgt->ops->sgl_next(sgl);
}

u64 nvgpu_sgt_get_phys(struct nvgpu_sgt *sgt, void *sgl)
{
	return sgt->ops->sgl_phys(sgl);
}

u64 nvgpu_sgt_get_dma(struct nvgpu_sgt *sgt, void *sgl)
{
	return sgt->ops->sgl_dma(sgl);
}

u64 nvgpu_sgt_get_length(struct nvgpu_sgt *sgt, void *sgl)
{
	return sgt->ops->sgl_length(sgl);
}

u64 nvgpu_sgt_get_gpu_addr(struct nvgpu_sgt *sgt, struct gk20a *g, void *sgl,
			   struct nvgpu_gmmu_attrs *attrs)
{
	return sgt->ops->sgl_gpu_addr(g, sgl, attrs);
}

void nvgpu_sgt_free(struct nvgpu_sgt *sgt, struct gk20a *g)
{
	if (sgt && sgt->ops->sgt_free)
		sgt->ops->sgt_free(g, sgt);
}

u64 nvgpu_mem_iommu_translate(struct gk20a *g, u64 phys)
{
	/* ensure it is not vidmem allocation */
	WARN_ON(is_vidmem_page_alloc(phys));

	if (nvgpu_iommuable(g) && g->ops.mm.get_iommu_bit)
		return phys | 1ULL << g->ops.mm.get_iommu_bit(g);

	return phys;
}
