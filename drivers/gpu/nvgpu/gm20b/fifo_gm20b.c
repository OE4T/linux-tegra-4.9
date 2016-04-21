/*
 * GM20B Fifo
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
#include "gk20a/gk20a.h"
#include "fifo_gm20b.h"
#include "hw_ccsr_gm20b.h"
#include "hw_ram_gm20b.h"
#include "hw_fifo_gm20b.h"

static void channel_gm20b_bind(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;

	u32 inst_ptr = gk20a_mm_inst_block_addr(g, &c->inst_block)
		>> ram_in_base_shift_v();

	gk20a_dbg_info("bind channel %d inst ptr 0x%08x",
		c->hw_chid, inst_ptr);

	c->bound = true;

	gk20a_writel(g, ccsr_channel_inst_r(c->hw_chid),
		ccsr_channel_inst_ptr_f(inst_ptr) |
		(g->mm.vidmem_is_vidmem ?
		 ccsr_channel_inst_target_sys_mem_ncoh_f() :
		 ccsr_channel_inst_target_vid_mem_f()) |
		ccsr_channel_inst_bind_true_f());

	gk20a_writel(g, ccsr_channel_r(c->hw_chid),
		(gk20a_readl(g, ccsr_channel_r(c->hw_chid)) &
		 ~ccsr_channel_enable_set_f(~0)) |
		 ccsr_channel_enable_set_true_f());
}

static inline u32 gm20b_engine_id_to_mmu_id(u32 engine_id)
{
	switch (engine_id) {
	case ENGINE_GR_GK20A:
		return 0;
	case ENGINE_CE2_GK20A:
		return 1;
	default:
		return ~0;
	}
}

static void gm20b_fifo_trigger_mmu_fault(struct gk20a *g,
		unsigned long engine_ids)
{
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(gk20a_get_gr_idle_timeout(g));
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	unsigned long engine_id;
	int ret = -EBUSY;

	/* trigger faults for all bad engines */
	for_each_set_bit(engine_id, &engine_ids, 32) {
		u32 engine_mmu_id;

		if (engine_id > g->fifo.max_engines) {
			gk20a_err(dev_from_gk20a(g),
				  "faulting unknown engine %ld", engine_id);
		} else {
			engine_mmu_id = gm20b_engine_id_to_mmu_id(engine_id);
			gk20a_writel(g, fifo_trigger_mmu_fault_r(engine_mmu_id),
				     fifo_trigger_mmu_fault_enable_f(1));
		}
	}

	/* Wait for MMU fault to trigger */
	do {
		if (gk20a_readl(g, fifo_intr_0_r()) &
				fifo_intr_0_mmu_fault_pending_f()) {
			ret = 0;
			break;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (time_before(jiffies, end_jiffies) ||
			!tegra_platform_is_silicon());

	if (ret)
		gk20a_err(dev_from_gk20a(g), "mmu fault timeout");

	/* release mmu fault trigger */
	for_each_set_bit(engine_id, &engine_ids, 32)
		gk20a_writel(g, fifo_trigger_mmu_fault_r(engine_id), 0);
}

static u32 gm20b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

void gm20b_init_fifo(struct gpu_ops *gops)
{
	gops->fifo.bind_channel = channel_gm20b_bind;
	gops->fifo.unbind_channel = channel_gk20a_unbind;
	gops->fifo.disable_channel = channel_gk20a_disable;
	gops->fifo.enable_channel = channel_gk20a_enable;
	gops->fifo.alloc_inst = channel_gk20a_alloc_inst;
	gops->fifo.free_inst = channel_gk20a_free_inst;
	gops->fifo.setup_ramfc = channel_gk20a_setup_ramfc;
	gops->fifo.channel_set_priority = gk20a_channel_set_priority;
	gops->fifo.channel_set_timeslice = gk20a_channel_set_timeslice;

	gops->fifo.preempt_channel = gk20a_fifo_preempt_channel;
	gops->fifo.update_runlist = gk20a_fifo_update_runlist;
	gops->fifo.trigger_mmu_fault = gm20b_fifo_trigger_mmu_fault;
	gops->fifo.wait_engine_idle = gk20a_fifo_wait_engine_idle;
	gops->fifo.get_num_fifos = gm20b_fifo_get_num_fifos;
	gops->fifo.get_pbdma_signature = gk20a_fifo_get_pbdma_signature;
	gops->fifo.set_runlist_interleave = gk20a_fifo_set_runlist_interleave;
	gops->fifo.force_reset_ch = gk20a_fifo_force_reset_ch;
}
