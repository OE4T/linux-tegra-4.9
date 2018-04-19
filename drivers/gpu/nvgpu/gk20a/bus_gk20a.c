/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/enabled.h>
#include <nvgpu/log.h>
#include <nvgpu/soc.h>
#include <nvgpu/bus.h>
#include <nvgpu/mm.h>

#include "gk20a.h"
#include "bus_gk20a.h"

#include <nvgpu/hw/gk20a/hw_bus_gk20a.h>
#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_timer_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringstation_sys_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringstation_gpc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringstation_fbp_gk20a.h>

void gk20a_bus_init_hw(struct gk20a *g)
{
	u32 timeout_period, intr_en_mask = 0;

	if (nvgpu_platform_is_silicon(g))
		timeout_period = g->default_pri_timeout ?
					g->default_pri_timeout : 0x186A0;
	else
		timeout_period = 0x186A0;

	if (nvgpu_platform_is_silicon(g) || nvgpu_platform_is_fpga(g)) {
		intr_en_mask = bus_intr_en_0_pri_squash_m() |
				bus_intr_en_0_pri_fecserr_m() |
				bus_intr_en_0_pri_timeout_m();
		gk20a_writel(g,
			timer_pri_timeout_r(),
			timer_pri_timeout_period_f(timeout_period) |
			timer_pri_timeout_en_en_enabled_f());

	} else {
		gk20a_writel(g,
			timer_pri_timeout_r(),
			timer_pri_timeout_period_f(timeout_period) |
			timer_pri_timeout_en_en_disabled_f());
	}
	gk20a_writel(g, bus_intr_en_0_r(), intr_en_mask);
}

void gk20a_bus_isr(struct gk20a *g)
{
	u32 val, save0, save1, fecs_errcode = 0;

	val = gk20a_readl(g, bus_intr_0_r());

	if (val & (bus_intr_0_pri_squash_m() |
			bus_intr_0_pri_fecserr_m() |
			bus_intr_0_pri_timeout_m())) {

		nvgpu_log(g, gpu_dbg_intr, "pmc_enable : 0x%x",
			gk20a_readl(g, mc_enable_r()));

		save0 = gk20a_readl(g, timer_pri_timeout_save_0_r());
		if (timer_pri_timeout_save_0_fecs_tgt_v(save0)) {
			/*
			 * write & addr fields in timeout_save0
			 * might not be reliable
			 */
			fecs_errcode = gk20a_readl(g,
					timer_pri_timeout_fecs_errcode_r());
		}

		save1 = gk20a_readl(g, timer_pri_timeout_save_1_r());
		nvgpu_err(g, "NV_PBUS_INTR_0: 0x%08x ADR 0x%08x "
			"%s  DATA 0x%08x ",
			val,
			timer_pri_timeout_save_0_addr_v(save0) << 2,
			timer_pri_timeout_save_0_write_v(save0) ?
			"WRITE" : "READ", save1);

		gk20a_writel(g, timer_pri_timeout_save_0_r(), 0);
		gk20a_writel(g, timer_pri_timeout_save_1_r(), 0);

		if (fecs_errcode) {
			nvgpu_err(g, "FECS_ERRCODE 0x%08x", fecs_errcode);
			if (g->ops.priv_ring.decode_error_code)
				g->ops.priv_ring.decode_error_code(g,
							fecs_errcode);

			if ((fecs_errcode & 0xffffff00) == 0xbadf1300)
				nvgpu_err(g, "NV_PGRAPH_PRI_GPC0_GPCCS_FS_GPC: "
					"0x%08x",
					gk20a_readl(g, gr_gpc0_fs_gpc_r()));
		}

	} else {
		nvgpu_err(g, "Unhandled NV_PBUS_INTR_0: 0x%08x", val);
	}
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
	u64 iova = nvgpu_inst_block_addr(g, bar1_inst);
	u32 ptr_v = (u32)(iova >> bus_bar1_block_ptr_shift_v());

	nvgpu_log(g, gpu_dbg_info, "bar1 inst block ptr: 0x%08x", ptr_v);

	gk20a_writel(g, bus_bar1_block_r(),
		     nvgpu_aperture_mask(g, bar1_inst,
					 bus_bar1_block_target_sys_mem_ncoh_f(),
					 bus_bar1_block_target_sys_mem_coh_f(),
					 bus_bar1_block_target_vid_mem_f()) |
		     bus_bar1_block_mode_virtual_f() |
		     bus_bar1_block_ptr_f(ptr_v));

	return 0;
}

void gk20a_bus_set_ppriv_timeout_settings(struct gk20a *g)
{
	/*
	 * Bug 1340570: increase the clock timeout to avoid potential
	 * operation failure at high gpcclk rate. Default values are 0x400.
	 */
	nvgpu_writel(g, pri_ringstation_sys_master_config_r(0x15), 0x800);
	nvgpu_writel(g, pri_ringstation_gpc_master_config_r(0xa), 0x800);
	nvgpu_writel(g, pri_ringstation_fbp_master_config_r(0x8), 0x800);
}
