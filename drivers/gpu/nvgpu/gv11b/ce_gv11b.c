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

#include "gk20a/gk20a.h"

#include "gp10b/ce_gp10b.h"

#include "ce_gv11b.h"

#include <nvgpu/hw/gv11b/hw_ce_gv11b.h>

static void gv11b_ce_isr(struct gk20a *g, u32 inst_id, u32 pri_base)
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

void gv11b_init_ce(struct gpu_ops *gops)
{
	gp10b_init_ce(gops);
	gops->ce2.isr_stall = gv11b_ce_isr;
}
