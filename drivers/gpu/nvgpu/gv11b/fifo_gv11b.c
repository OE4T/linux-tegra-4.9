/*
 * GV11B fifo
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/types.h>
#include "gk20a/gk20a.h"
#include "gk20a/fifo_gk20a.h"
#include "gp10b/fifo_gp10b.h"
#include "hw_pbdma_gv11b.h"
#include "fifo_gv11b.h"
#include "hw_fifo_gv11b.h"
#include "hw_ram_gv11b.h"
#include "hw_ccsr_gv11b.h"
#include "hw_usermode_gv11b.h"

static void gv11b_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist)
{

	u32 runlist_entry_0 = ram_rl_entry_type_tsg_v();

	if (tsg->timeslice_timeout)
		runlist_entry_0 |=
		ram_rl_entry_tsg_timeslice_scale_f(tsg->timeslice_scale) |
		ram_rl_entry_tsg_timeslice_timeout_f(tsg->timeslice_timeout);
	else
		runlist_entry_0 |=
			ram_rl_entry_tsg_timeslice_scale_f(
				ram_rl_entry_tsg_timeslice_scale_3_v()) |
			ram_rl_entry_tsg_timeslice_timeout_f(
				ram_rl_entry_tsg_timeslice_timeout_128_v());

	runlist[0] = runlist_entry_0;
	runlist[1] = ram_rl_entry_tsg_length_f(tsg->num_active_channels);
	runlist[2] = ram_rl_entry_tsg_tsgid_f(tsg->tsgid);
	runlist[3] = 0;

	gk20a_dbg_info("gv11b tsg runlist [0] %x [1]  %x [2] %x [3] %x\n",
		runlist[0], runlist[1], runlist[2], runlist[3]);

}

static void gv11b_get_ch_runlist_entry(struct channel_gk20a *c, u32 *runlist)
{
	struct gk20a *g = c->g;
	u32 addr_lo, addr_hi;
	u32 runlist_entry;

	/* Time being use 0 pbdma sequencer */
	runlist_entry = ram_rl_entry_type_channel_v() |
			ram_rl_entry_chan_runqueue_selector_f(0) |
			ram_rl_entry_chan_userd_target_f(
			ram_rl_entry_chan_userd_target_sys_mem_ncoh_v()) |
			ram_rl_entry_chan_inst_target_f(
			ram_rl_entry_chan_userd_target_sys_mem_ncoh_v());

	addr_lo = u64_lo32(c->userd_iova) >>
			ram_rl_entry_chan_userd_ptr_align_shift_v();
	addr_hi = u64_hi32(c->userd_iova);
	runlist[0] = runlist_entry | ram_rl_entry_chan_userd_ptr_lo_f(addr_lo);
	runlist[1] = ram_rl_entry_chan_userd_ptr_hi_f(addr_hi);

	addr_lo = u64_lo32(gk20a_mm_inst_block_addr(g, &c->inst_block)) >>
			ram_rl_entry_chan_inst_ptr_align_shift_v();
	addr_hi = u64_hi32(gk20a_mm_inst_block_addr(g, &c->inst_block));

	runlist[2] = ram_rl_entry_chan_inst_ptr_lo_f(addr_lo) |
				ram_rl_entry_chid_f(c->hw_chid);
	runlist[3] = ram_rl_entry_chan_inst_ptr_hi_f(addr_hi);

	gk20a_dbg_info("gv11b channel runlist [0] %x [1]  %x [2] %x [3] %x\n",
			runlist[0], runlist[1], runlist[2], runlist[3]);
}

static void gv11b_ring_channel_doorbell(struct channel_gk20a *c)
{
	gk20a_dbg_info("channel ring door bell %d\n", c->hw_chid);

	gk20a_writel(c->g, usermode_notify_channel_pending_r(),
		usermode_notify_channel_pending_id_f(c->hw_chid));
}

static u32 gv11b_userd_gp_get(struct gk20a *g, struct channel_gk20a *c)
{
	struct mem_desc *userd_mem = &g->fifo.userd;
	u32 offset = c->hw_chid * (g->fifo.userd_entry_size / sizeof(u32));

	return gk20a_mem_rd32(g, userd_mem,
			offset + ram_userd_gp_get_w());

}

static void gv11b_userd_gp_put(struct gk20a *g, struct channel_gk20a *c)
{
	struct mem_desc *userd_mem = &g->fifo.userd;
	u32 offset = c->hw_chid * (g->fifo.userd_entry_size / sizeof(u32));

	gk20a_mem_wr32(g, userd_mem, offset + ram_userd_gp_put_w(),
							c->gpfifo.put);
	/* commit everything to cpu */
	smp_mb();

	gv11b_ring_channel_doorbell(c);

}


static u32 gv11b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

void gv11b_init_fifo(struct gpu_ops *gops)
{
	gp10b_init_fifo(gops);
	/* for gv11b no need to do any thing special for fifo hw setup */
	gops->fifo.init_fifo_setup_hw = NULL;
	gops->fifo.runlist_entry_size = ram_rl_entry_size_v;
	gops->fifo.get_tsg_runlist_entry = gv11b_get_tsg_runlist_entry;
	gops->fifo.get_ch_runlist_entry = gv11b_get_ch_runlist_entry;
	gops->fifo.get_num_fifos = gv11b_fifo_get_num_fifos;
	gops->fifo.userd_gp_get = gv11b_userd_gp_get;
	gops->fifo.userd_gp_put = gv11b_userd_gp_put;
}
