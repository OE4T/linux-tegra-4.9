/*
 * Volta GPU series Copy Engine.
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program.
 */

#include "nvgpu/log.h"
#include "nvgpu/bitops.h"

#include "gk20a/gk20a.h"

#include "gp10b/ce_gp10b.h"

#include "ce_gv11b.h"

#include <nvgpu/hw/gv11b/hw_ce_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>

u32 gv11b_ce_get_num_pce(struct gk20a *g)
{
	/* register contains a bitmask indicating which physical copy
	 * engines are present (and not floorswept).
	 */
	u32 num_pce;
	u32 ce_pce_map = gk20a_readl(g, ce_pce_map_r());

	num_pce = get_count_order(ce_pce_map) + 1;
	nvgpu_log_info(g, "num PCE: %d", num_pce);
	return num_pce;
}

void gv11b_ce_isr(struct gk20a *g, u32 inst_id, u32 pri_base)
{
	u32 ce_intr = gk20a_readl(g, ce_intr_status_r(inst_id));
	u32 clear_intr = 0;

	nvgpu_log(g, gpu_dbg_intr, "ce isr 0x%08x 0x%08x", ce_intr, inst_id);

	/* An INVALID_CONFIG interrupt will be generated if a floorswept
	 * PCE is assigned to a valid LCE in the NV_CE_PCE2LCE_CONFIG
	 * registers. This is a fatal error and the LCE will have to be
	 * reset to get back to a working state.
	 */
	if (ce_intr & ce_intr_status_invalid_config_pending_f()) {
		nvgpu_log(g, gpu_dbg_intr,
			"ce: inst %d: invalid config", inst_id);
		clear_intr |= ce_intr_status_invalid_config_reset_f();
	}

	/* A MTHD_BUFFER_FAULT interrupt will be triggered if any access
	 * to a method buffer during context load or save encounters a fault.
	 * This is a fatal interrupt and will require at least the LCE to be
	 * reset before operations can start again, if not the entire GPU.
	 */
	if (ce_intr & ce_intr_status_mthd_buffer_fault_pending_f()) {
		nvgpu_log(g, gpu_dbg_intr,
			"ce: inst %d: mthd buffer fault", inst_id);
		clear_intr |= ce_intr_status_mthd_buffer_fault_reset_f();
	}

	gk20a_writel(g, ce_intr_status_r(inst_id), clear_intr);

	gp10b_ce_isr(g, inst_id, pri_base);
}

u32 gv11b_ce_get_num_lce(struct gk20a *g)
{
	u32 reg_val, num_lce;

	reg_val = gk20a_readl(g, top_num_ces_r());
	num_lce = top_num_ces_value_v(reg_val);
	nvgpu_log_info(g, "num LCE: %d", num_lce);

	return num_lce;
}

void gv11b_ce_mthd_buffer_fault_in_bar2_fault(struct gk20a *g)
{
	u32 reg_val, num_lce, lce, clear_intr;

	num_lce = gv11b_ce_get_num_lce(g);

	for (lce = 0; lce < num_lce; lce++) {
		reg_val = gk20a_readl(g, ce_intr_status_r(lce));
		if (reg_val & ce_intr_status_mthd_buffer_fault_pending_f()) {
			nvgpu_log(g, gpu_dbg_intr,
			"ce: lce %d: mthd buffer fault", lce);
			clear_intr = ce_intr_status_mthd_buffer_fault_reset_f();
			gk20a_writel(g, ce_intr_status_r(lce), clear_intr);
		}
	}
}
