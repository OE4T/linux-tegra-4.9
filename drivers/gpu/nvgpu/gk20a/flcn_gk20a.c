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
 */
#include <nvgpu/falcon.h>

#include "gk20a/gk20a.h"

#include <nvgpu/hw/gk20a/hw_falcon_gk20a.h>

static int gk20a_flcn_reset(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 unit_status = 0;
	int status = 0;

	if (flcn->flcn_engine_dep_ops.reset_eng)
		/* falcon & engine reset */
		status = flcn->flcn_engine_dep_ops.reset_eng(g);
	else {
		/* do falcon CPU hard reset */
		unit_status = gk20a_readl(g, base_addr +
				falcon_falcon_cpuctl_r());
		gk20a_writel(g, base_addr + falcon_falcon_cpuctl_r(),
			(unit_status | falcon_falcon_cpuctl_hreset_f(1)));
	}

	return status;
}

static void gk20a_flcn_set_irq(struct nvgpu_falcon *flcn, bool enable)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;

	if (!flcn->is_interrupt_enabled) {
		nvgpu_warn(g, "Interrupt not supported on flcn 0x%x ",
			flcn->flcn_id);
		/* Keep interrupt disabled */
		enable = false;
	}

	if (enable) {
		gk20a_writel(g, base_addr + falcon_falcon_irqmset_r(),
			flcn->intr_mask);
		gk20a_writel(g, base_addr + falcon_falcon_irqdest_r(),
			flcn->intr_dest);
	} else
		gk20a_writel(g, base_addr + falcon_falcon_irqmclr_r(),
			0xffffffff);
}

static bool gk20a_is_falcon_cpu_halted(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;

	return (gk20a_readl(g, base_addr + falcon_falcon_cpuctl_r()) &
			falcon_falcon_cpuctl_halt_intr_m() ?
			true : false);
}

static bool gk20a_is_falcon_idle(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 unit_status = 0;
	bool status = false;

	unit_status = gk20a_readl(g,
		base_addr + falcon_falcon_idlestate_r());

	if (falcon_falcon_idlestate_falcon_busy_v(unit_status) == 0 &&
		falcon_falcon_idlestate_ext_busy_v(unit_status) == 0)
		status = true;
	else
		status = false;

	return status;
}

static bool gk20a_is_falcon_scrubbing_done(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 unit_status = 0;
	bool status = false;

	unit_status = gk20a_readl(g,
		base_addr + falcon_falcon_dmactl_r());

	if (unit_status & (falcon_falcon_dmactl_dmem_scrubbing_m() |
		 falcon_falcon_dmactl_imem_scrubbing_m()))
		status = false;
	else
		status = true;

	return status;
}

static void gk20a_falcon_engine_dependency_ops(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_engine_dependency_ops *flcn_eng_dep_ops =
			&flcn->flcn_engine_dep_ops;

	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn_eng_dep_ops->reset_eng = gk20a_pmu_reset;
		break;
	default:
		/* NULL assignment make sure
		 * CPU hard reset in gk20a_flcn_reset() gets execute
		 * if falcon doesn't need specific reset implementation
		 */
		flcn_eng_dep_ops->reset_eng = NULL;
		break;
	}
}

static void gk20a_falcon_ops(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	flcn_ops->reset = gk20a_flcn_reset;
	flcn_ops->set_irq = gk20a_flcn_set_irq;
	flcn_ops->is_falcon_cpu_halted =  gk20a_is_falcon_cpu_halted;
	flcn_ops->is_falcon_idle =  gk20a_is_falcon_idle;
	flcn_ops->is_falcon_scrubbing_done =  gk20a_is_falcon_scrubbing_done;

	gk20a_falcon_engine_dependency_ops(flcn);
}

static void gk20a_falcon_hal_sw_init(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;

	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn->flcn_base = FALCON_PWR_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = true;
		break;
	case FALCON_ID_FECS:
		flcn->flcn_base = FALCON_FECS_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
	break;
	case FALCON_ID_GPCCS:
		flcn->flcn_base = FALCON_GPCCS_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
	break;
	default:
		flcn->is_falcon_supported = false;
		nvgpu_err(g, "Invalid flcn request");
		break;
	}

	if (flcn->is_falcon_supported) {
		gk20a_falcon_ops(flcn);
	} else
		nvgpu_info(g, "flcn-Id 0x%x not supported on current chip",
			flcn->flcn_id);
}

void gk20a_falcon_init_hal(struct gpu_ops *gops)
{
	gops->falcon.falcon_hal_sw_init = gk20a_falcon_hal_sw_init;
}
