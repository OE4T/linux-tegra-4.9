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

#include <nvgpu/page_allocator.h>
#include <nvgpu/bug.h>

#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gk20a/pramin_gk20a.h"

#include <nvgpu/hw/gk20a/hw_bus_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pram_gk20a.h>

/* WARNING: returns pramin_window_lock taken, complement with pramin_exit() */
u32 gk20a_pramin_enter(struct gk20a *g, struct nvgpu_mem *mem,
			      struct page_alloc_chunk *chunk, u32 w)
{
	u64 bufbase = chunk->base;
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
			hi, lo, mem, chunk, bufbase,
			bufbase + chunk->length, chunk->length);

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
			      struct page_alloc_chunk *chunk)
{
	gk20a_dbg(gpu_dbg_mem, "end for %p,%p", mem, chunk);

	nvgpu_spinlock_release(&g->mm.pramin_window_lock);
}
