/*
 * GK20A priv ring
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

#include "gk20a.h"

#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringmaster_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringstation_sys_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pri_ringstation_gpc_gk20a.h>

void gk20a_enable_priv_ring(struct gk20a *g)
{
	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		return;

	if (g->ops.clock_gating.slcg_priring_load_gating_prod)
		g->ops.clock_gating.slcg_priring_load_gating_prod(g,
				g->slcg_enabled);

	gk20a_writel(g,pri_ringmaster_command_r(),
			0x4);

	gk20a_writel(g, pri_ringstation_sys_decode_config_r(),
			0x2);

	gk20a_readl(g, pri_ringstation_sys_decode_config_r());

}

void gk20a_priv_ring_isr(struct gk20a *g)
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

	gk20a_dbg(gpu_dbg_intr, "ringmaster intr status0: 0x%08x,"
		"status1: 0x%08x", status0, status1);

	if (pri_ringmaster_intr_status0_gbl_write_error_sys_v(status0) != 0) {
		gk20a_dbg(gpu_dbg_intr, "SYS write error. ADR %08x WRDAT %08x INFO %08x, CODE %08x",
			gk20a_readl(g, pri_ringstation_sys_priv_error_adr_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_wrdat_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_info_r()),
			gk20a_readl(g, pri_ringstation_sys_priv_error_code_r()));
	}

	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		if (status1 & BIT(gpc)) {
			gk20a_dbg(gpu_dbg_intr, "GPC%u write error. ADR %08x WRDAT %08x INFO %08x, CODE %08x", gpc,
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
