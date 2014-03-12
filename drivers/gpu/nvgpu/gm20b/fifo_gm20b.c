/*
 * GM20B Fifo
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

#include "gk20a/gk20a.h"
#include "fifo_gm20b.h"
#include "hw_ccsr_gm20b.h"
#include "hw_ram_gm20b.h"

static void channel_gm20b_bind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = ch_gk20a->g;

	u32 inst_ptr = ch_gk20a->inst_block.cpu_pa
		>> ram_in_base_shift_v();

	gk20a_dbg_info("bind channel %d inst ptr 0x%08x",
		ch_gk20a->hw_chid, inst_ptr);

	ch_gk20a->bound = true;

	gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->hw_chid),
		ccsr_channel_inst_ptr_f(inst_ptr) |
		ccsr_channel_inst_target_vid_mem_f() |
		ccsr_channel_inst_bind_true_f());

	gk20a_writel(g, ccsr_channel_r(ch_gk20a->hw_chid),
		(gk20a_readl(g, ccsr_channel_r(ch_gk20a->hw_chid)) &
		 ~ccsr_channel_enable_set_f(~0)) |
		 ccsr_channel_enable_set_true_f());
}

void gm20b_init_fifo(struct gpu_ops *gops)
{
	gops->fifo.bind_channel = channel_gm20b_bind;
}
