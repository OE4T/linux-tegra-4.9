/*
 * GK20A memory interface
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <trace/events/gk20a.h>

#include "gk20a.h"
#include "kind_gk20a.h"
#include "fb_gk20a.h"

#include <nvgpu/timers.h>

#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_fb_gk20a.h>

void fb_gk20a_reset(struct gk20a *g)
{
	u32 val;

	gk20a_dbg_info("reset gk20a fb");

	g->ops.mc.reset(g, mc_enable_pfb_enabled_f() |
			mc_enable_l2_enabled_f() |
			mc_enable_xbar_enabled_f() |
			mc_enable_hub_enabled_f());

	val = gk20a_readl(g, mc_elpg_enable_r());
	val |= mc_elpg_enable_xbar_enabled_f()
		| mc_elpg_enable_pfb_enabled_f()
		| mc_elpg_enable_hub_enabled_f();
	gk20a_writel(g, mc_elpg_enable_r(), val);
}

void gk20a_fb_init_hw(struct gk20a *g)
{
	u32 addr = g->ops.mm.get_iova_addr(g,
			g->mm.sysmem_flush.priv.sgt->sgl, 0) >> 8;

	gk20a_writel(g, fb_niso_flush_sysmem_addr_r(), addr);
}

static void gk20a_fb_set_mmu_page_size(struct gk20a *g)
{
	/* set large page size in fb */
	u32 fb_mmu_ctrl = gk20a_readl(g, fb_mmu_ctrl_r());

	fb_mmu_ctrl = (fb_mmu_ctrl &
		       ~fb_mmu_ctrl_vm_pg_size_f(~0x0)) |
		fb_mmu_ctrl_vm_pg_size_128kb_f();

	gk20a_writel(g, fb_mmu_ctrl_r(), fb_mmu_ctrl);
}

static unsigned int gk20a_fb_compression_page_size(struct gk20a *g)
{
	return SZ_128K;
}

static unsigned int gk20a_fb_compressible_page_size(struct gk20a *g)
{
	return SZ_64K;
}

static bool gk20a_fb_debug_mode_enabled(struct gk20a *g)
{
	u32 debug_ctrl = gk20a_readl(g, fb_mmu_debug_ctrl_r());
	return fb_mmu_debug_ctrl_debug_v(debug_ctrl) ==
		fb_mmu_debug_ctrl_debug_enabled_v();
}

static void gk20a_fb_set_debug_mode(struct gk20a *g, bool enable)
{
	u32 reg_val, debug_ctrl;

	reg_val = gk20a_readl(g, fb_mmu_debug_ctrl_r());
	if (enable) {
		debug_ctrl = fb_mmu_debug_ctrl_debug_enabled_f();
		g->mmu_debug_ctrl = true;
	} else {
		debug_ctrl = fb_mmu_debug_ctrl_debug_disabled_f();
		g->mmu_debug_ctrl = false;
	}

	reg_val = set_field(reg_val,
				fb_mmu_debug_ctrl_debug_m(), debug_ctrl);
	gk20a_writel(g, fb_mmu_debug_ctrl_r(), reg_val);
}

void gk20a_fb_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb)
{
	struct nvgpu_timeout timeout;
	u32 addr_lo;
	u32 data;

	gk20a_dbg_fn("");

	/* pagetables are considered sw states which are preserved after
	   prepare_poweroff. When gk20a deinit releases those pagetables,
	   common code in vm unmap path calls tlb invalidate that touches
	   hw. Use the power_on flag to skip tlb invalidation when gpu
	   power is turned off */

	if (!g->power_on)
		return;

	addr_lo = u64_lo32(gk20a_mem_get_base_addr(g, pdb, 0) >> 12);

	nvgpu_mutex_acquire(&g->mm.tlb_lock);

	trace_gk20a_mm_tlb_invalidate(g->name);

	nvgpu_timeout_init(g, &timeout, 1000, NVGPU_TIMER_RETRY_TIMER);

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_space_v(data) != 0)
			break;
		nvgpu_udelay(2);
	} while (!nvgpu_timeout_expired_msg(&timeout,
					 "wait mmu fifo space"));

	if (nvgpu_timeout_peek_expired(&timeout))
		goto out;

	nvgpu_timeout_init(g, &timeout, 1000, NVGPU_TIMER_RETRY_TIMER);

	gk20a_writel(g, fb_mmu_invalidate_pdb_r(),
		fb_mmu_invalidate_pdb_addr_f(addr_lo) |
		nvgpu_aperture_mask(g, pdb,
		  fb_mmu_invalidate_pdb_aperture_sys_mem_f(),
		  fb_mmu_invalidate_pdb_aperture_vid_mem_f()));

	gk20a_writel(g, fb_mmu_invalidate_r(),
		fb_mmu_invalidate_all_va_true_f() |
		fb_mmu_invalidate_trigger_true_f());

	do {
		data = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_empty_v(data) !=
			fb_mmu_ctrl_pri_fifo_empty_false_f())
			break;
		nvgpu_udelay(2);
	} while (!nvgpu_timeout_expired_msg(&timeout,
					 "wait mmu invalidate"));

	trace_gk20a_mm_tlb_invalidate_done(g->name);

out:
	nvgpu_mutex_release(&g->mm.tlb_lock);
}

void gk20a_init_fb(struct gpu_ops *gops)
{
	gops->fb.init_hw = gk20a_fb_init_hw;
	gops->fb.reset = fb_gk20a_reset;
	gops->fb.set_mmu_page_size = gk20a_fb_set_mmu_page_size;
	gops->fb.compression_page_size = gk20a_fb_compression_page_size;
	gops->fb.compressible_page_size = gk20a_fb_compressible_page_size;
	gops->fb.is_debug_mode_enabled = gk20a_fb_debug_mode_enabled;
	gops->fb.set_debug_mode = gk20a_fb_set_debug_mode;
	gops->fb.tlb_invalidate = gk20a_fb_tlb_invalidate;
	gk20a_init_uncompressed_kind_map();
	gk20a_init_kind_attr();
}
