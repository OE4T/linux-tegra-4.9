/*
 * GM20B MMU
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/pm_runtime.h>
#include "gk20a/gk20a.h"
#include "mm_gm20b.h"
#include "hw_gmmu_gm20b.h"
#include "hw_fb_gm20b.h"
#include "hw_gr_gm20b.h"
#include "hw_ram_gm20b.h"

static int gm20b_mm_mmu_vpr_info_fetch_wait(struct gk20a *g,
		const unsigned int msec)
{
	unsigned long timeout;

	if (tegra_platform_is_silicon())
		timeout = jiffies + msecs_to_jiffies(msec);
	else
		timeout = msecs_to_jiffies(msec);

	while (1) {
		u32 val;
		val = gk20a_readl(g, fb_mmu_vpr_info_r());
		if (fb_mmu_vpr_info_fetch_v(val) ==
				fb_mmu_vpr_info_fetch_false_v())
			break;
		if (tegra_platform_is_silicon()) {
			if (WARN_ON(time_after(jiffies, timeout)))
				return -ETIME;
		} else if (--timeout == 0)
			return -ETIME;
	}
	return 0;
}

int gm20b_mm_mmu_vpr_info_fetch(struct gk20a *g)
{
	int ret = 0;

	gk20a_busy_noresume(g->dev);
#ifdef CONFIG_PM_RUNTIME
	if (!pm_runtime_active(&g->dev->dev))
		goto fail;
#endif

	if (gm20b_mm_mmu_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT)) {
		ret = -ETIME;
		goto fail;
	}

	gk20a_writel(g, fb_mmu_vpr_info_r(),
			fb_mmu_vpr_info_fetch_true_v());

	ret = gm20b_mm_mmu_vpr_info_fetch_wait(g, VPR_INFO_FETCH_WAIT);

fail:
	pm_runtime_put(&g->dev->dev);
	return ret;
}

static bool gm20b_mm_mmu_debug_mode_enabled(struct gk20a *g)
{
	u32 debug_ctrl = gk20a_readl(g, gr_gpcs_pri_mmu_debug_ctrl_r());
	return gr_gpcs_pri_mmu_debug_ctrl_debug_v(debug_ctrl) ==
		gr_gpcs_pri_mmu_debug_ctrl_debug_enabled_v();
}

static void gm20b_mm_set_big_page_size(struct gk20a *g,
				void *inst_ptr, int size)
{
	u32 val;

	gk20a_dbg_fn("");

	gk20a_dbg_info("big page size %d\n", size);
	val = gk20a_mem_rd32(inst_ptr, ram_in_big_page_size_w());
	val &= ~ram_in_big_page_size_m();

	if (size == SZ_64K)
		val |= ram_in_big_page_size_64kb_f();
	else
		val |= ram_in_big_page_size_128kb_f();

	gk20a_mem_wr32(inst_ptr, ram_in_big_page_size_w(), val);
	gk20a_dbg_fn("done");
}

static u32 gm20b_mm_get_big_page_sizes(void)
{
	return SZ_64K | SZ_128K;
}

static bool gm20b_mm_support_sparse(struct gk20a *g)
{
	return true;
}

void gm20b_init_mm(struct gpu_ops *gops)
{
	gops->mm.support_sparse = gm20b_mm_support_sparse;
	gops->mm.is_debug_mode_enabled = gm20b_mm_mmu_debug_mode_enabled;
	gops->mm.gmmu_map = gk20a_locked_gmmu_map;
	gops->mm.gmmu_unmap = gk20a_locked_gmmu_unmap;
	gops->mm.vm_remove = gk20a_vm_remove_support;
	gops->mm.vm_alloc_share = gk20a_vm_alloc_share;
	gops->mm.vm_bind_channel = gk20a_vm_bind_channel;
	gops->mm.fb_flush = gk20a_mm_fb_flush;
	gops->mm.l2_invalidate = gk20a_mm_l2_invalidate;
	gops->mm.l2_flush = gk20a_mm_l2_flush;
	gops->mm.tlb_invalidate = gk20a_mm_tlb_invalidate;
	gops->mm.set_big_page_size = gm20b_mm_set_big_page_size;
	gops->mm.get_big_page_sizes = gm20b_mm_get_big_page_sizes;
	gops->mm.get_iova_addr = gk20a_mm_iova_addr;
	gops->mm.get_physical_addr_bits = gk20a_mm_get_physical_addr_bits;
	gops->mm.get_mmu_levels = gk20a_mm_get_mmu_levels;
	gops->mm.init_pdb = gk20a_mm_init_pdb;
	gops->mm.init_mm_setup_hw = gk20a_init_mm_setup_hw;
}
