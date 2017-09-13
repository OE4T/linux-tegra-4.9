/*
 * GP10B priv ring
 *
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

#include "gk20a/gk20a.h"

#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gp10b/hw_mc_gp10b.h>
#include <nvgpu/hw/gp10b/hw_pri_ringmaster_gp10b.h>
#include <nvgpu/hw/gp10b/hw_pri_ringstation_sys_gp10b.h>
#include <nvgpu/hw/gp10b/hw_pri_ringstation_gpc_gp10b.h>

void gp10b_priv_ring_isr(struct gk20a *g)
{
	u32 status0, status1;
	u32 cmd;
	s32 retry = 100;
	u32 gpc;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		return;

	status0 = gk20a_readl(g, pri_ringmaster_intr_status0_r());
	status1 = gk20a_readl(g, pri_ringmaster_intr_status1_r());

	nvgpu_err(g, "ringmaster intr status0: 0x%08x,"
		"status1: 0x%08x", status0, status1);

	if (pri_ringmaster_intr_status0_gbl_write_error_sys_v(status0) != 0) {
		nvgpu_err(g, "SYS write error. ADR %08x WRDAT %08x INFO %08x, CODE %08x",
			gk20a_readl(g, pri_ringstation_sys_priv_error_adr_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_wrdat_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_info_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_code_r()));
	}

	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		if (status1 & BIT(gpc)) {
			nvgpu_err(g, "GPC%u write error. ADR %08x WRDAT %08x INFO %08x, CODE %08x", gpc,
				gk20a_readl(g, pri_ringstation_gpc_gpc0_priv_error_adr_r() + gpc * gpc_stride),
				gk20a_readl(g, pri_ringstation_gpc_gpc0_priv_error_wrdat_r() + gpc * gpc_stride),
				gk20a_readl(g, pri_ringstation_gpc_gpc0_priv_error_info_r() + gpc * gpc_stride),
				gk20a_readl(g, pri_ringstation_gpc_gpc0_priv_error_code_r() + gpc * gpc_stride));
		}
	}

	cmd = gk20a_readl(g, pri_ringmaster_command_r());
	cmd = set_field(cmd, pri_ringmaster_command_cmd_m(),
		pri_ringmaster_command_cmd_ack_interrupt_f());
	gk20a_writel(g, pri_ringmaster_command_r(), cmd);

	do {
		cmd = pri_ringmaster_command_cmd_v(
			gk20a_readl(g, pri_ringmaster_command_r()));
		nvgpu_usleep_range(20, 40);
	} while (cmd != pri_ringmaster_command_cmd_no_cmd_v() && --retry);

	if (retry <= 0)
		nvgpu_warn(g, "priv ringmaster cmd ack too many retries");
}
