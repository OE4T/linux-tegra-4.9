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
#include <nvgpu/log.h>
#include <nvgpu/soc.h>
#include <nvgpu/bus.h>

#include "gk20a.h"
#include "bus_gk20a.h"

#include <nvgpu/hw/gk20a/hw_bus_gk20a.h>
#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_timer_gk20a.h>

void gk20a_bus_init_hw(struct gk20a *g)
{
	/* enable pri timeout only on silicon */
	if (nvgpu_platform_is_silicon(g)) {
		gk20a_writel(g,
			timer_pri_timeout_r(),
			timer_pri_timeout_period_f(
				g->default_pri_timeout ?
					g->default_pri_timeout : 0x186A0) |
			timer_pri_timeout_en_en_enabled_f());
	} else {
		gk20a_writel(g,
			timer_pri_timeout_r(),
			timer_pri_timeout_period_f(0x186A0) |
			timer_pri_timeout_en_en_disabled_f());
	}

	if (!nvgpu_platform_is_silicon(g))
		gk20a_writel(g, bus_intr_en_0_r(), 0x0);
	else
		gk20a_writel(g, bus_intr_en_0_r(),
			        bus_intr_en_0_pri_squash_m() |
			        bus_intr_en_0_pri_fecserr_m() |
			        bus_intr_en_0_pri_timeout_m());
}

void gk20a_bus_isr(struct gk20a *g)
{
	u32 val, err_code;
	val = gk20a_readl(g, bus_intr_0_r());
	if (val & (bus_intr_0_pri_squash_m() |
			bus_intr_0_pri_fecserr_m() |
			bus_intr_0_pri_timeout_m())) {
		gk20a_dbg(gpu_dbg_intr, "pmc_enable : 0x%x",
			gk20a_readl(g, mc_enable_r()));
		gk20a_dbg(gpu_dbg_intr, "NV_PBUS_INTR_0 : 0x%x", val);
		gk20a_dbg(gpu_dbg_intr,
			"NV_PTIMER_PRI_TIMEOUT_SAVE_0: 0x%x\n",
			gk20a_readl(g, timer_pri_timeout_save_0_r()));
		gk20a_dbg(gpu_dbg_intr,
			"NV_PTIMER_PRI_TIMEOUT_SAVE_1: 0x%x\n",
			gk20a_readl(g, timer_pri_timeout_save_1_r()));
		err_code = gk20a_readl(g, timer_pri_timeout_fecs_errcode_r());
		gk20a_dbg(gpu_dbg_intr,
			"NV_PTIMER_PRI_TIMEOUT_FECS_ERRCODE: 0x%x\n",
			err_code);
		if (err_code == 0xbadf13)
			gk20a_dbg(gpu_dbg_intr,
			"NV_PGRAPH_PRI_GPC0_GPCCS_FS_GPC: 0x%x\n",
			gk20a_readl(g, gr_gpc0_fs_gpc_r()));

		gk20a_writel(g, timer_pri_timeout_save_0_r(), 0);
		gk20a_writel(g, timer_pri_timeout_save_1_r(), 0);
	}

	if (val)
		gk20a_dbg(gpu_dbg_intr,
			"Unhandled pending pbus interrupt\n");

	gk20a_writel(g, bus_intr_0_r(), val);
}

int gk20a_read_ptimer(struct gk20a *g, u64 *value)
{
	const unsigned int max_iterations = 3;
	unsigned int i = 0;
	u32 gpu_timestamp_hi_prev = 0;

	if (!value)
		return -EINVAL;

	/* Note. The GPU nanosecond timer consists of two 32-bit
	 * registers (high & low). To detect a possible low register
	 * wrap-around between the reads, we need to read the high
	 * register before and after low. The wraparound happens
	 * approximately once per 4 secs. */

	/* get initial gpu_timestamp_hi value */
	gpu_timestamp_hi_prev = gk20a_readl(g, timer_time_1_r());

	for (i = 0; i < max_iterations; ++i) {
		u32 gpu_timestamp_hi = 0;
		u32 gpu_timestamp_lo = 0;

		gpu_timestamp_lo = gk20a_readl(g, timer_time_0_r());
		gpu_timestamp_hi = gk20a_readl(g, timer_time_1_r());

		if (gpu_timestamp_hi == gpu_timestamp_hi_prev) {
			*value = (((u64)gpu_timestamp_hi) << 32) |
				gpu_timestamp_lo;
			return 0;
		}

		/* wrap-around detected, retry */
		gpu_timestamp_hi_prev = gpu_timestamp_hi;
	}

	/* too many iterations, bail out */
	nvgpu_err(g, "failed to read ptimer");
	return -EBUSY;
}

int gk20a_bus_bar1_bind(struct gk20a *g, struct nvgpu_mem *bar1_inst)
{
	u64 iova = gk20a_mm_inst_block_addr(g, bar1_inst);
	u32 ptr_v = (u32)(iova >> bar1_instance_block_shift_gk20a());

	gk20a_dbg_info("bar1 inst block ptr: 0x%08x", ptr_v);

	gk20a_writel(g, bus_bar1_block_r(),
		     nvgpu_aperture_mask(g, bar1_inst,
		       bus_bar1_block_target_sys_mem_ncoh_f(),
		       bus_bar1_block_target_vid_mem_f()) |
		     bus_bar1_block_mode_virtual_f() |
		     bus_bar1_block_ptr_f(ptr_v));

	return 0;
}

void gk20a_init_bus(struct gpu_ops *gops)
{
	gops->bus.init_hw = gk20a_bus_init_hw;
	gops->bus.isr = gk20a_bus_isr;
	gops->bus.read_ptimer = gk20a_read_ptimer;
	gops->bus.get_timestamps_zipper = nvgpu_get_timestamps_zipper;
	gops->bus.bar1_bind = gk20a_bus_bar1_bind;
}
