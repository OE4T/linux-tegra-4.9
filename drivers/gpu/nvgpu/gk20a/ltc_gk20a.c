/*
 * GK20A L2
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/dma.h>

#include "gk20a.h"
#include "gr_gk20a.h"

int gk20a_ltc_alloc_phys_cbc(struct gk20a *g, size_t compbit_backing_size)
{
	struct gr_gk20a *gr = &g->gr;

	return nvgpu_dma_alloc_flags_sys(g, NVGPU_DMA_FORCE_CONTIGUOUS,
				    compbit_backing_size,
				    &gr->compbit_store.mem);
}

int gk20a_ltc_alloc_virt_cbc(struct gk20a *g, size_t compbit_backing_size)
{
	struct gr_gk20a *gr = &g->gr;

	return nvgpu_dma_alloc_flags_sys(g, NVGPU_DMA_NO_KERNEL_MAPPING,
				    compbit_backing_size,
				    &gr->compbit_store.mem);
}
