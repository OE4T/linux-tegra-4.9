/*
 * GP10B master
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

#include "gk20a/gk20a.h"
#include "gk20a/mc_gk20a.h"

#include "mc_gp10b.h"

#include <nvgpu/atomic.h>
#include <nvgpu/unit.h>

#include <nvgpu/hw/gp10b/hw_mc_gp10b.h>

void mc_gp10b_intr_enable(struct gk20a *g)
{
	u32 eng_intr_mask = gk20a_fifo_engine_interrupt_mask(g);

	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING),
				0xffffffff);
	g->ops.mc.intr_mask_restore[NVGPU_MC_INTR_STALLING] =
				mc_intr_pfifo_pending_f() |
				 mc_intr_priv_ring_pending_f() |
				 mc_intr_pbus_pending_f() |
				 mc_intr_ltc_pending_f() |
				 mc_intr_replayable_fault_pending_f() |
				 eng_intr_mask;
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_STALLING),
			g->ops.mc.intr_mask_restore[NVGPU_MC_INTR_STALLING]);

	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
				0xffffffff);
	g->ops.mc.intr_mask_restore[NVGPU_MC_INTR_NONSTALLING] =
				mc_intr_pfifo_pending_f() |
				 eng_intr_mask;
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_NONSTALLING),
			g->ops.mc.intr_mask_restore[NVGPU_MC_INTR_NONSTALLING]);
}

void mc_gp10b_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask)
{
	u32 intr_index = 0;
	u32 reg = 0;

	intr_index = (is_stalling ? NVGPU_MC_INTR_STALLING :
			NVGPU_MC_INTR_NONSTALLING);
	if (enable) {
		reg = mc_intr_en_set_r(intr_index);
		g->ops.mc.intr_mask_restore[intr_index] |= mask;

	} else {
		reg = mc_intr_en_clear_r(intr_index);
		g->ops.mc.intr_mask_restore[intr_index] &= ~mask;
	}

	gk20a_writel(g, reg, mask);
}

void mc_gp10b_isr_stall(struct gk20a *g)
{
	u32 mc_intr_0;
	int hw_irq_count;

	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	mc_intr_0 = gk20a_readl(g, mc_intr_r(0));
	hw_irq_count = atomic_read(&g->hw_irq_stall_count);

	gk20a_dbg(gpu_dbg_intr, "stall intr 0x%08x\n", mc_intr_0);

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines; engine_id_idx++) {
		active_engine_id = g->fifo.active_engines_list[engine_id_idx];

		if (mc_intr_0 & g->fifo.engine_info[active_engine_id].intr_mask) {
			engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
			/* GR Engine */
			if (engine_enum == ENGINE_GR_GK20A) {
				gr_gk20a_elpg_protected_call(g, gk20a_gr_isr(g));
			}

			/* CE Engine */
			if (((engine_enum == ENGINE_GRCE_GK20A) ||
				(engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
				g->ops.ce2.isr_stall){
					g->ops.ce2.isr_stall(g,
					g->fifo.engine_info[active_engine_id].inst_id,
					g->fifo.engine_info[active_engine_id].pri_base);
			}
		}
	}
	if (g->ops.mc.is_intr_hub_pending &&
		 g->ops.mc.is_intr_hub_pending(g, mc_intr_0))
		g->ops.fb.hub_isr(g);
	if (mc_intr_0 & mc_intr_pfifo_pending_f())
		gk20a_fifo_isr(g);
	if (mc_intr_0 & mc_intr_pmu_pending_f())
		gk20a_pmu_isr(g);
	if (mc_intr_0 & mc_intr_priv_ring_pending_f())
		gk20a_priv_ring_isr(g);
	if (mc_intr_0 & mc_intr_ltc_pending_f())
		g->ops.ltc.isr(g);
	if (mc_intr_0 & mc_intr_pbus_pending_f())
		g->ops.bus.isr(g);

	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&g->sw_irq_stall_last_handled, hw_irq_count);

	gk20a_dbg(gpu_dbg_intr, "stall intr done 0x%08x\n", mc_intr_0);

}

u32 mc_gp10b_intr_stall(struct gk20a *g)
{
	return gk20a_readl(g, mc_intr_r(NVGPU_MC_INTR_STALLING));
}

void mc_gp10b_intr_stall_pause(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING), 0xffffffff);
}

void mc_gp10b_intr_stall_resume(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_STALLING),
			g->ops.mc.intr_mask_restore[NVGPU_MC_INTR_STALLING]);
}

u32 mc_gp10b_intr_nonstall(struct gk20a *g)
{
	return gk20a_readl(g, mc_intr_r(NVGPU_MC_INTR_NONSTALLING));
}

void mc_gp10b_intr_nonstall_pause(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
		     0xffffffff);
}

void mc_gp10b_intr_nonstall_resume(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_NONSTALLING),
			g->ops.mc.intr_mask_restore[NVGPU_MC_INTR_NONSTALLING]);
}

bool mc_gp10b_is_intr1_pending(struct gk20a *g,
				      enum nvgpu_unit unit, u32 mc_intr_1)
{
	u32 mask = 0;
	bool is_pending;

	switch (unit) {
	case NVGPU_UNIT_FIFO:
		mask = mc_intr_pfifo_pending_f();
		break;
	default:
		break;
	}

	if (mask == 0) {
		nvgpu_err(g, "unknown unit %d", unit);
		is_pending = false;
	} else {
		is_pending = (mc_intr_1 & mask) ? true : false;
	}

	return is_pending;
}
