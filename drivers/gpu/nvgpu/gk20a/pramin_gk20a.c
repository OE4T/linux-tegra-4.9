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

#include <nvgpu/page_allocator.h>
#include <nvgpu/bug.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/pramin_gk20a.h"

#include <nvgpu/hw/gk20a/hw_bus_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pram_gk20a.h>

/* WARNING: returns pramin_window_lock taken, complement with pramin_exit() */
u32 gk20a_pramin_enter(struct gk20a *g, struct nvgpu_mem *mem,
		       struct nvgpu_sgt *sgt, void *sgl, u32 w)
{
	u64 bufbase = nvgpu_sgt_get_phys(sgt, sgl);
	u64 addr = bufbase + w * sizeof(u32);
	u32 hi = (u32)((addr & ~(u64)0xfffff)
		>> bus_bar0_window_target_bar0_window_base_shift_v());
	u32 lo = (u32)(addr & 0xfffff);
	u32 win = nvgpu_aperture_mask(g, mem,
			bus_bar0_window_target_sys_mem_noncoherent_f(),
			bus_bar0_window_target_vid_mem_f()) |
		bus_bar0_window_base_f(hi);

	gk20a_dbg(gpu_dbg_mem,
			"0x%08x:%08x begin for %p,%p at [%llx,%llx] (sz %llx)",
			hi, lo, mem, sgl, bufbase,
			bufbase + nvgpu_sgt_get_phys(sgt, sgl),
			nvgpu_sgt_get_length(sgt, sgl));

	WARN_ON(!bufbase);

	nvgpu_spinlock_acquire(&g->mm.pramin_window_lock);

	if (g->mm.pramin_window != win) {
		gk20a_writel(g, bus_bar0_window_r(), win);
		gk20a_readl(g, bus_bar0_window_r());
		g->mm.pramin_window = win;
	}

	return lo;
}

void gk20a_pramin_exit(struct gk20a *g, struct nvgpu_mem *mem,
		       void *sgl)
{
	gk20a_dbg(gpu_dbg_mem, "end for %p,%p", mem, sgl);

	nvgpu_spinlock_release(&g->mm.pramin_window_lock);
}
