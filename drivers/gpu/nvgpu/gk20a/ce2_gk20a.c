/*
 * GK20A Graphics Copy Engine  (gr host)
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*TODO: remove uncecessary */
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <trace/events/gk20a.h>
#include <linux/dma-mapping.h>
#include <linux/nvhost.h>

#include "gk20a.h"
#include "debug_gk20a.h"
#include "semaphore_gk20a.h"
#include "hw_ce2_gk20a.h"
#include "hw_pbdma_gk20a.h"
#include "hw_ccsr_gk20a.h"
#include "hw_ram_gk20a.h"
#include "hw_proj_gk20a.h"
#include "hw_top_gk20a.h"
#include "hw_mc_gk20a.h"
#include "hw_gr_gk20a.h"

static u32 ce2_nonblockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 non-blocking pipe interrupt\n");

	return ce2_intr_status_nonblockpipe_pending_f();
}

static u32 ce2_blockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 blocking pipe interrupt\n");

	return ce2_intr_status_blockpipe_pending_f();
}

static u32 ce2_launcherr_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 launch error interrupt\n");

	return ce2_intr_status_launcherr_pending_f();
}

void gk20a_ce2_isr(struct gk20a *g)
{
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r());
	u32 clear_intr = 0;

	gk20a_dbg(gpu_dbg_intr, "ce2 isr %08x\n", ce2_intr);

	/* clear blocking interrupts: they exibit broken behavior */
	if (ce2_intr & ce2_intr_status_blockpipe_pending_f())
		clear_intr |= ce2_blockpipe_isr(g, ce2_intr);

	if (ce2_intr & ce2_intr_status_launcherr_pending_f())
		clear_intr |= ce2_launcherr_isr(g, ce2_intr);

	gk20a_writel(g, ce2_intr_status_r(), clear_intr);
	return;
}

void gk20a_ce2_nonstall_isr(struct gk20a *g)
{
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r());

	gk20a_dbg(gpu_dbg_intr, "ce2 nonstall isr %08x\n", ce2_intr);

	if (ce2_intr & ce2_intr_status_nonblockpipe_pending_f()) {
		gk20a_writel(g, ce2_intr_status_r(),
			ce2_nonblockpipe_isr(g, ce2_intr));

		/* wake threads waiting in this channel */
		gk20a_channel_semaphore_wakeup(g);
	}

	return;
}
void gk20a_init_ce2(struct gpu_ops *gops)
{
	gops->ce2.isr_stall = gk20a_ce2_isr;
	gops->ce2.isr_nonstall = gk20a_ce2_nonstall_isr;
}
