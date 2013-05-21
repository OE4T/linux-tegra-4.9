/*
 * drivers/video/tegra/host/gk20a/clk_gk20a.h
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011 - 2013, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _NVHOST_CLK_GK20A_H_
#define _NVHOST_CLK_GK20A_H_

enum {
	/* only one PLL for gk20a */
	GK20A_GPC_PLL = 0,
};

struct pll {
	u32 id;
	u32 clk_in;	/* MHz */
	u32 M;
	u32 N;
	u32 PL;
	u32 freq;	/* MHz */
};

struct pll_parms {
	u32 min_freq, max_freq;	/* MHz */
	u32 min_vco, max_vco;	/* MHz */
	u32 min_u,   max_u;	/* MHz */
	u32 min_M,   max_M;
	u32 min_N,   max_N;
	u32 min_PL,  max_PL;
};

struct clk_gk20a {
	struct gk20a *g;
	struct clk *tegra_clk;
	struct pll gpc_pll;
	u32 pll_delay; /* default PLL settle time */

	bool sw_ready;
};

int gk20a_init_clk_support(struct gk20a *g);

u32 gk20a_clk_get_rate(struct gk20a *g);
int gk20a_clk_set_rate(struct gk20a *g, u32 rate);

extern struct pll_parms gpc_pll_params;

#endif /* _NVHOST_CLK_GK20A_H_ */
