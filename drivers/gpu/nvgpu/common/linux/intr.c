/*
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

#include <trace/events/gk20a.h>

#include "gk20a/gk20a.h"

#include <nvgpu/atomic.h>

irqreturn_t nvgpu_intr_stall(struct gk20a *g)
{
	u32 mc_intr_0;

	trace_mc_gk20a_intr_stall(g->name);

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	mc_intr_0 = g->ops.mc.intr_stall(g);
	if (unlikely(!mc_intr_0))
		return IRQ_NONE;

	g->ops.mc.intr_stall_pause(g);

	atomic_inc(&g->hw_irq_stall_count);

	trace_mc_gk20a_intr_stall_done(g->name);

	return IRQ_WAKE_THREAD;
}

irqreturn_t nvgpu_intr_thread_stall(struct gk20a *g)
{
	gk20a_dbg(gpu_dbg_intr, "interrupt thread launched");

	trace_mc_gk20a_intr_thread_stall(g->name);

	g->ops.mc.isr_stall(g);
	g->ops.mc.intr_stall_resume(g);

	wake_up_all(&g->sw_irq_stall_last_handled_wq);

	trace_mc_gk20a_intr_thread_stall_done(g->name);

	return IRQ_HANDLED;
}

