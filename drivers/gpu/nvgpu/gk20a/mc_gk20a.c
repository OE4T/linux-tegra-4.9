/*
 * GK20A Master Control
 *
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <trace/events/gk20a.h>

#include "gk20a.h"
#include "mc_gk20a.h"

#include <nvgpu/atomic.h>

#include <nvgpu/hw/gk20a/hw_mc_gk20a.h>

void mc_gk20a_nonstall_cb(struct work_struct *work)
{
	struct gk20a *g = container_of(work, struct gk20a, nonstall_fn_work);
	u32 ops;
	bool semaphore_wakeup, post_events;

	do {
		ops = atomic_xchg(&g->nonstall_ops, 0);

		semaphore_wakeup = ops & gk20a_nonstall_ops_wakeup_semaphore;
		post_events = ops & gk20a_nonstall_ops_post_events;

		if (semaphore_wakeup)
			gk20a_channel_semaphore_wakeup(g, post_events);

	} while (atomic_read(&g->nonstall_ops) != 0);
}

irqreturn_t mc_gk20a_isr_stall(struct gk20a *g)
{
	u32 mc_intr_0;

	trace_mc_gk20a_intr_stall(g->name);

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

	atomic_inc(&g->hw_irq_stall_count);

	trace_mc_gk20a_intr_stall_done(g->name);

	return IRQ_WAKE_THREAD;
}

irqreturn_t mc_gk20a_isr_nonstall(struct gk20a *g)
{
	u32 mc_intr_1;
	u32 hw_irq_count;

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

	if (g->ops.mc.isr_thread_nonstall)
		g->ops.mc.isr_thread_nonstall(g, mc_intr_1);

	hw_irq_count = atomic_inc_return(&g->hw_irq_nonstall_count);

	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&g->sw_irq_nonstall_last_handled, hw_irq_count);

	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_hardware_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_1_r());

	wake_up_all(&g->sw_irq_nonstall_last_handled_wq);

	return IRQ_HANDLED;
}

irqreturn_t mc_gk20a_intr_thread_stall(struct gk20a *g)
{
	u32 mc_intr_0;
	int hw_irq_count;
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	gk20a_dbg(gpu_dbg_intr, "interrupt thread launched");

	trace_mc_gk20a_intr_thread_stall(g->name);

	mc_intr_0 = gk20a_readl(g, mc_intr_0_r());
	hw_irq_count = atomic_read(&g->hw_irq_stall_count);

	gk20a_dbg(gpu_dbg_intr, "stall intr %08x\n", mc_intr_0);

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
	if (mc_intr_0 & mc_intr_0_pfifo_pending_f())
		gk20a_fifo_isr(g);
	if (mc_intr_0 & mc_intr_0_pmu_pending_f())
		gk20a_pmu_isr(g);
	if (mc_intr_0 & mc_intr_0_priv_ring_pending_f())
		gk20a_priv_ring_isr(g);
	if (mc_intr_0 & mc_intr_0_ltc_pending_f())
		g->ops.ltc.isr(g);
	if (mc_intr_0 & mc_intr_0_pbus_pending_f())
		g->ops.bus.isr(g);

	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&g->sw_irq_stall_last_handled, hw_irq_count);

	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());

	/* flush previous write */
	gk20a_readl(g, mc_intr_en_0_r());

	wake_up_all(&g->sw_irq_stall_last_handled_wq);

	trace_mc_gk20a_intr_thread_stall_done(g->name);

	return IRQ_HANDLED;
}

void mc_gk20a_intr_thread_nonstall(struct gk20a *g, u32 mc_intr_1)
{
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	int ops_old, ops_new, ops = 0;

	if (mc_intr_1 & mc_intr_0_pfifo_pending_f())
		ops |= gk20a_fifo_nonstall_isr(g);

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines;
							engine_id_idx++) {
		active_engine_id = g->fifo.active_engines_list[engine_id_idx];

		if (mc_intr_1 &
			g->fifo.engine_info[active_engine_id].intr_mask) {
			engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
			/* GR Engine */
			if (engine_enum == ENGINE_GR_GK20A)
				ops |= gk20a_gr_nonstall_isr(g);

			/* CE Engine */
			if (((engine_enum == ENGINE_GRCE_GK20A) ||
				(engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
				g->ops.ce2.isr_nonstall)
					ops |= g->ops.ce2.isr_nonstall(g,
					g->fifo.engine_info[active_engine_id].
								inst_id,
					g->fifo.engine_info[active_engine_id].
								pri_base);
		}
	}
	if (ops) {
		do {
			ops_old = atomic_read(&g->nonstall_ops);
			ops_new  = ops_old | ops;
		} while (ops_old != atomic_cmpxchg(&g->nonstall_ops,
						ops_old, ops_new));

		queue_work(g->nonstall_work_queue, &g->nonstall_fn_work);
	}
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

void mc_gk20a_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask)
{
	u32 mask_reg = (is_stalling ? mc_intr_mask_0_r() :
					mc_intr_mask_1_r());

	if (enable) {
		gk20a_writel(g, mask_reg,
			gk20a_readl(g, mask_reg) |
			mask);
	} else {
		gk20a_writel(g, mask_reg,
			gk20a_readl(g, mask_reg) &
			~mask);
	}
}

void gk20a_mc_disable(struct gk20a *g, u32 units)
{
	u32 pmc;

	gk20a_dbg(gpu_dbg_info, "pmc disable: %08x\n", units);

	nvgpu_spinlock_acquire(&g->mc_enable_lock);
	pmc = gk20a_readl(g, mc_enable_r());
	pmc &= ~units;
	gk20a_writel(g, mc_enable_r(), pmc);
	nvgpu_spinlock_release(&g->mc_enable_lock);
}

void gk20a_mc_enable(struct gk20a *g, u32 units)
{
	u32 pmc;

	gk20a_dbg(gpu_dbg_info, "pmc enable: %08x\n", units);

	nvgpu_spinlock_acquire(&g->mc_enable_lock);
	pmc = gk20a_readl(g, mc_enable_r());
	pmc |= units;
	gk20a_writel(g, mc_enable_r(), pmc);
	gk20a_readl(g, mc_enable_r());
	nvgpu_spinlock_release(&g->mc_enable_lock);

	udelay(20);
}

void gk20a_mc_reset(struct gk20a *g, u32 units)
{
	g->ops.mc.disable(g, units);
	if (units & gk20a_fifo_get_all_ce_engine_reset_mask(g))
		udelay(500);
	else
		udelay(20);
	g->ops.mc.enable(g, units);
}

u32 gk20a_mc_boot_0(struct gk20a *g, u32 *arch, u32 *impl, u32 *rev)
{
	u32 val = gk20a_readl(g, mc_boot_0_r());

	if (arch)
		*arch = mc_boot_0_architecture_v(val) <<
			NVGPU_GPU_ARCHITECTURE_SHIFT;

	if (impl)
		*impl = mc_boot_0_implementation_v(val);

	if (rev)
		*rev = (mc_boot_0_major_revision_v(val) << 4) |
			mc_boot_0_minor_revision_v(val);

	return val;
}

void gk20a_init_mc(struct gpu_ops *gops)
{
	gops->mc.intr_enable = mc_gk20a_intr_enable;
	gops->mc.intr_unit_config = mc_gk20a_intr_unit_config;
	gops->mc.isr_stall = mc_gk20a_isr_stall;
	gops->mc.isr_nonstall = mc_gk20a_isr_nonstall;
	gops->mc.isr_thread_stall = mc_gk20a_intr_thread_stall;
	gops->mc.isr_thread_nonstall = mc_gk20a_intr_thread_nonstall;
	gops->mc.isr_nonstall_cb = mc_gk20a_nonstall_cb;
	gops->mc.enable = gk20a_mc_enable;
	gops->mc.disable = gk20a_mc_disable;
	gops->mc.reset = gk20a_mc_reset;
	gops->mc.boot_0 = gk20a_mc_boot_0;
}
