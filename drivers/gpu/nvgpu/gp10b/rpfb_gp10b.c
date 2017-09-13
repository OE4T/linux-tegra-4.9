/*
 * GP10B RPFB
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/dma.h>

#include "gk20a/gk20a.h"

#include "rpfb_gp10b.h"

#include <nvgpu/hw/gp10b/hw_fifo_gp10b.h>
#include <nvgpu/hw/gp10b/hw_fb_gp10b.h>
#include <nvgpu/hw/gp10b/hw_bus_gp10b.h>
#include <nvgpu/hw/gp10b/hw_gmmu_gp10b.h>

int gp10b_replayable_pagefault_buffer_init(struct gk20a *g)
{
	u32 addr_lo;
	u32 addr_hi;
	struct vm_gk20a *vm = g->mm.bar2.vm;
	int err;
	size_t rbfb_size = NV_UVM_FAULT_BUF_SIZE *
		fifo_replay_fault_buffer_size_hw_entries_v();

	gk20a_dbg_fn("");

	if (!g->mm.bar2_desc.gpu_va) {
		err = nvgpu_dma_alloc_map_sys(vm, rbfb_size,
						&g->mm.bar2_desc);
		if (err) {
			nvgpu_err(g, "Error in replayable fault buffer");
			return err;
		}
	}
	addr_lo = u64_lo32(g->mm.bar2_desc.gpu_va >> 12);
	addr_hi = u64_hi32(g->mm.bar2_desc.gpu_va);
	gk20a_writel(g, fifo_replay_fault_buffer_hi_r(),
			fifo_replay_fault_buffer_hi_base_f(addr_hi));

	gk20a_writel(g, fifo_replay_fault_buffer_lo_r(),
			fifo_replay_fault_buffer_lo_base_f(addr_lo) |
			fifo_replay_fault_buffer_lo_enable_true_v());
	gk20a_dbg_fn("done");
	return 0;
}

void gp10b_replayable_pagefault_buffer_deinit(struct gk20a *g)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;

	nvgpu_dma_unmap_free(vm, &g->mm.bar2_desc);
}

u32 gp10b_replayable_pagefault_buffer_get_index(struct gk20a *g)
{
	u32 get_idx = 0;

	gk20a_dbg_fn("");

	get_idx = gk20a_readl(g, fifo_replay_fault_buffer_get_r());

	if (get_idx >= fifo_replay_fault_buffer_size_hw_entries_v())
		nvgpu_err(g, "Error in replayable fault buffer");

	gk20a_dbg_fn("done");
	return get_idx;
}

u32 gp10b_replayable_pagefault_buffer_put_index(struct gk20a *g)
{
	u32 put_idx = 0;

	gk20a_dbg_fn("");
	put_idx = gk20a_readl(g, fifo_replay_fault_buffer_put_r());

	if (put_idx >= fifo_replay_fault_buffer_size_hw_entries_v())
		nvgpu_err(g, "Error in UVM");

	gk20a_dbg_fn("done");
	return put_idx;
}

bool gp10b_replayable_pagefault_buffer_is_empty(struct gk20a *g)
{
	u32 get_idx = gk20a_readl(g, fifo_replay_fault_buffer_get_r());
	u32 put_idx = gk20a_readl(g, fifo_replay_fault_buffer_put_r());

	return (get_idx == put_idx ? true : false);
}

bool gp10b_replayable_pagefault_buffer_is_full(struct gk20a *g)
{
	u32 get_idx = gk20a_readl(g, fifo_replay_fault_buffer_get_r());
	u32 put_idx = gk20a_readl(g, fifo_replay_fault_buffer_put_r());
	u32 hw_entries = gk20a_readl(g, fifo_replay_fault_buffer_size_r());

	return (get_idx == ((put_idx + 1) % hw_entries) ? true : false);
}

bool gp10b_replayable_pagefault_buffer_is_overflow(struct gk20a *g)
{
	u32 info = gk20a_readl(g, fifo_replay_fault_buffer_info_r());

	return fifo_replay_fault_buffer_info_overflow_f(info);
}

void gp10b_replayable_pagefault_buffer_clear_overflow(struct gk20a *g)
{
	u32 info = gk20a_readl(g, fifo_replay_fault_buffer_info_r());

	info |= fifo_replay_fault_buffer_info_overflow_clear_v();
	gk20a_writel(g, fifo_replay_fault_buffer_info_r(), info);

}

void gp10b_replayable_pagefault_buffer_info(struct gk20a *g)
{

	gk20a_dbg_fn("");
	pr_info("rpfb low: 0x%x\n",
		(gk20a_readl(g, fifo_replay_fault_buffer_lo_r()) >> 12));
	pr_info("rpfb hi: 0x%x\n",
		gk20a_readl(g, fifo_replay_fault_buffer_hi_r()));
	pr_info("rpfb enabled: 0x%x\n",
		(gk20a_readl(g, fifo_replay_fault_buffer_lo_r()) & 0x1));
	pr_info("rpfb size: %d\n",
		gk20a_readl(g, fifo_replay_fault_buffer_size_r()));
	pr_info("rpfb get index: %d\n",
		gp10b_replayable_pagefault_buffer_get_index(g));
	pr_info("rpfb put index: %d\n",
		gp10b_replayable_pagefault_buffer_put_index(g));
	pr_info("rpfb empty: %d\n",
		gp10b_replayable_pagefault_buffer_is_empty(g));
	pr_info("rpfb full  %d\n",
		gp10b_replayable_pagefault_buffer_is_full(g));
	pr_info("rpfb overflow  %d\n",
		gp10b_replayable_pagefault_buffer_is_overflow(g));

	gk20a_dbg_fn("done");
}
