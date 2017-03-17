/*
 * drivers/video/tegra/host/gk20a/sim_gk20a.h
 *
 * GK20A sim support
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __SIM_GK20A_H__
#define __SIM_GK20A_H__

struct gk20a;
struct platform_device;

struct sim_gk20a {
	struct gk20a *g;
	struct resource *reg_mem;
	void __iomem *regs;
	struct {
		struct page *page;
		void *kvaddr;
		u64 phys;
	} send_bfr, recv_bfr, msg_bfr;
	u32 send_ring_put;
	u32 recv_ring_get;
	u32 recv_ring_put;
	u32 sequence_base;
	void (*remove_support)(struct sim_gk20a *);
};

int gk20a_init_sim_support(struct platform_device *pdev);
int gk20a_sim_esc_readl(struct gk20a *g, char *path, u32 index, u32 *data);

#endif /*__SIM_GK20A_H__*/
