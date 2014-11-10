/*
 * GK20A memory interface
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>

#include "gk20a.h"
#include "mc_gk20a.h"
#include "hw_mc_gk20a.h"

irqreturn_t mc_gk20a_isr_stall(struct gk20a *g)
{
	u32 mc_intr_0;

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	mc_intr_0 = gk20a_readl(g, mc_intr_0_r());
	if (unlikely(!mc_intr_0))
		return IRQ_NONE;

	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_disabled_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_0_r());

	return IRQ_WAKE_THREAD;
}

irqreturn_t mc_gk20a_isr_nonstall(struct gk20a *g)
{
	u32 mc_intr_1;

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	mc_intr_1 = gk20a_readl(g, mc_intr_1_r());
	if (unlikely(!mc_intr_1))
		return IRQ_NONE;

	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_disabled_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_1_r());

	return IRQ_WAKE_THREAD;
}

irqreturn_t mc_gk20a_intr_thread_stall(struct gk20a *g)
{
	u32 mc_intr_0;

	gk20a_dbg(gpu_dbg_intr, "interrupt thread launched");

	mc_intr_0 = gk20a_readl(g, mc_intr_0_r());

	gk20a_dbg(gpu_dbg_intr, "stall intr %08x\n", mc_intr_0);

	if (mc_intr_0 & BIT(g->fifo.engine_info[ENGINE_GR_GK20A].intr_id))
		gr_gk20a_elpg_protected_call(g, gk20a_gr_isr(g));
	if (mc_intr_0 & mc_intr_0_pfifo_pending_f())
		gk20a_fifo_isr(g);
	if (mc_intr_0 & mc_intr_0_pmu_pending_f())
		gk20a_pmu_isr(g);
	if (mc_intr_0 & mc_intr_0_priv_ring_pending_f())
		gk20a_priv_ring_isr(g);
	if (mc_intr_0 & mc_intr_0_ltc_pending_f())
		g->ops.ltc.isr(g);
	if (mc_intr_0 & mc_intr_0_pbus_pending_f())
		gk20a_pbus_isr(g);

	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_0_r());

	return IRQ_HANDLED;
}

irqreturn_t mc_gk20a_intr_thread_nonstall(struct gk20a *g)
{
	u32 mc_intr_1;

	gk20a_dbg(gpu_dbg_intr, "interrupt thread launched");

	mc_intr_1 = gk20a_readl(g, mc_intr_1_r());

	gk20a_dbg(gpu_dbg_intr, "non-stall intr %08x\n", mc_intr_1);

	if (mc_intr_1 & mc_intr_0_pfifo_pending_f())
		gk20a_fifo_nonstall_isr(g);
	if (mc_intr_1 & BIT(g->fifo.engine_info[ENGINE_GR_GK20A].intr_id))
		gk20a_gr_nonstall_isr(g);

	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_hardware_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_1_r());

	return IRQ_HANDLED;
}

void mc_gk20a_intr_enable(struct gk20a *g)
{
	u32 eng_intr_mask = gk20a_fifo_engine_interrupt_mask(g);

	gk20a_writel(g, mc_intr_mask_1_r(),
		     mc_intr_0_pfifo_pending_f()
		     | eng_intr_mask);
	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_hardware_f());

	gk20a_writel(g, mc_intr_mask_0_r(),
		     mc_intr_0_pfifo_pending_f()
		     | mc_intr_0_priv_ring_pending_f()
		     | mc_intr_0_ltc_pending_f()
		     | mc_intr_0_pbus_pending_f()
		     | eng_intr_mask);
	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());
}

void gk20a_init_mc(struct gpu_ops *gops)
{
	gops->mc.intr_enable = mc_gk20a_intr_enable;
	gops->mc.isr_stall = mc_gk20a_isr_stall;
	gops->mc.isr_nonstall = mc_gk20a_isr_nonstall;
	gops->mc.isr_thread_stall = mc_gk20a_intr_thread_stall;
	gops->mc.isr_thread_nonstall = mc_gk20a_intr_thread_nonstall;
}
