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
#include <linux/irqreturn.h>

#include "gk20a/gk20a.h"

#include <nvgpu/atomic.h>
#include <nvgpu/unit.h>
#include "os_linux.h"

irqreturn_t nvgpu_intr_stall(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	u32 mc_intr_0;

	trace_mc_gk20a_intr_stall(g->name);

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	mc_intr_0 = g->ops.mc.intr_stall(g);
	if (unlikely(!mc_intr_0))
		return IRQ_NONE;

	g->ops.mc.intr_stall_pause(g);

	atomic_inc(&l->hw_irq_stall_count);

	trace_mc_gk20a_intr_stall_done(g->name);

	return IRQ_WAKE_THREAD;
}

irqreturn_t nvgpu_intr_thread_stall(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	int hw_irq_count;

	gk20a_dbg(gpu_dbg_intr, "interrupt thread launched");

	trace_mc_gk20a_intr_thread_stall(g->name);

	hw_irq_count = atomic_read(&l->hw_irq_stall_count);
	g->ops.mc.isr_stall(g);
	g->ops.mc.intr_stall_resume(g);
	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&l->sw_irq_stall_last_handled, hw_irq_count);

	wake_up_all(&l->sw_irq_stall_last_handled_wq);

	trace_mc_gk20a_intr_thread_stall_done(g->name);

	return IRQ_HANDLED;
}

irqreturn_t nvgpu_intr_nonstall(struct gk20a *g)
{
	u32 mc_intr_1;
	u32 hw_irq_count;
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	int ops_old, ops_new, ops = 0;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	mc_intr_1 = g->ops.mc.intr_nonstall(g);
	if (unlikely(!mc_intr_1))
		return IRQ_NONE;

	g->ops.mc.intr_nonstall_pause(g);

	if (g->ops.mc.is_intr1_pending(g, NVGPU_UNIT_FIFO, mc_intr_1))
		ops |= gk20a_fifo_nonstall_isr(g);

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines;
							engine_id_idx++) {
		struct fifo_engine_info_gk20a *engine_info;

		active_engine_id = g->fifo.active_engines_list[engine_id_idx];
		engine_info = &g->fifo.engine_info[active_engine_id];

		if (mc_intr_1 & engine_info->intr_mask) {
			engine_enum = engine_info->engine_enum;
			/* GR Engine */
			if (engine_enum == ENGINE_GR_GK20A)
				ops |= gk20a_gr_nonstall_isr(g);

			/* CE Engine */
			if (((engine_enum == ENGINE_GRCE_GK20A) ||
				(engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
				g->ops.ce2.isr_nonstall)
					ops |= g->ops.ce2.isr_nonstall(g,
						engine_info->inst_id,
						engine_info->pri_base);
		}
	}
	if (ops) {
		do {
			ops_old = atomic_read(&l->nonstall_ops);
			ops_new  = ops_old | ops;
		} while (ops_old != atomic_cmpxchg(&l->nonstall_ops,
						ops_old, ops_new));

		queue_work(l->nonstall_work_queue, &l->nonstall_fn_work);
	}

	hw_irq_count = atomic_inc_return(&l->hw_irq_nonstall_count);

	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&l->sw_irq_nonstall_last_handled, hw_irq_count);

	g->ops.mc.intr_nonstall_resume(g);

	wake_up_all(&l->sw_irq_nonstall_last_handled_wq);

	return IRQ_HANDLED;
}

void nvgpu_intr_nonstall_cb(struct work_struct *work)
{
	struct nvgpu_os_linux *l =
		container_of(work, struct nvgpu_os_linux, nonstall_fn_work);
	struct gk20a *g = &l->g;
	u32 ops;
	bool semaphore_wakeup, post_events;

	do {
		ops = atomic_xchg(&l->nonstall_ops, 0);

		semaphore_wakeup = ops & gk20a_nonstall_ops_wakeup_semaphore;
		post_events = ops & gk20a_nonstall_ops_post_events;

		if (semaphore_wakeup)
			gk20a_channel_semaphore_wakeup(g, post_events);

	} while (atomic_read(&l->nonstall_ops) != 0);
}
