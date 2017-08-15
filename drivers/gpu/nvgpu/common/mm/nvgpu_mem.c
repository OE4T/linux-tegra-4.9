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
