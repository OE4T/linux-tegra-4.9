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

#include "gk20a/gk20a.h" /* FERMI and MAXWELL classes defined here */
#include "hw_ce2_gp10b.h"
#include "ce2_gp10b.h"

static u32 ce2_nonblockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	gk20a_dbg(gpu_dbg_intr, "ce2 non-blocking pipe interrupt\n");

	/* wake theads waiting in this channel */
	gk20a_channel_semaphore_wakeup(g, true);
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

static void gp10b_ce2_isr(struct gk20a *g)
{
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r(0));
	u32 clear_intr = 0;

	gk20a_dbg(gpu_dbg_intr, "ce2 isr %08x\n", ce2_intr);

	/* clear blocking interrupts: they exibit broken behavior */
	if (ce2_intr & ce2_intr_status_blockpipe_pending_f())
		clear_intr |= ce2_blockpipe_isr(g, ce2_intr);

	if (ce2_intr & ce2_intr_status_launcherr_pending_f())
		clear_intr |= ce2_launcherr_isr(g, ce2_intr);

	gk20a_writel(g, ce2_intr_status_r(0), clear_intr);
	return;
}

static void gp10b_ce2_nonstall_isr(struct gk20a *g)
{
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r(0));
	u32 clear_intr = 0;

	gk20a_dbg(gpu_dbg_intr, "ce2 nonstall isr %08x\n", ce2_intr);

	if (ce2_intr & ce2_intr_status_nonblockpipe_pending_f())
		clear_intr |= ce2_nonblockpipe_isr(g, ce2_intr);

	gk20a_writel(g, ce2_intr_status_r(0), clear_intr);

	return;
}
void gp10b_init_ce2(struct gpu_ops *gops)
{
	gops->ce2.isr_stall = gp10b_ce2_isr;
	gops->ce2.isr_nonstall = gp10b_ce2_nonstall_isr;
}
