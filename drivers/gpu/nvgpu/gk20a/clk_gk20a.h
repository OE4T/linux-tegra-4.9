/*
 * drivers/video/tegra/host/gk20a/clk_gk20a.h
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011 - 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/mutex.h>

#define GPUFREQ_TABLE_END     ~(u32)1
enum {
	/* only one PLL for gk20a */
	GK20A_GPC_PLL = 0,
};

struct pll {
	u32 id;
	u32 clk_in;	/* KHz */
	u32 M;
	u32 N;
	u32 PL;
	u32 freq;	/* KHz */
	bool enabled;
};

struct pll_parms {
	u32 min_freq, max_freq;	/* KHz */
	u32 min_vco, max_vco;	/* KHz */
	u32 min_u,   max_u;	/* KHz */
	u32 min_M,   max_M;
	u32 min_N,   max_N;
	u32 min_PL,  max_PL;
};

struct clk_gk20a {
	struct gk20a *g;
	struct clk *tegra_clk;
	struct pll gpc_pll;
	u32 pll_delay; /* default PLL settle time */
	struct mutex clk_mutex;
	bool sw_ready;
	bool clk_hw_on;
};

int gk20a_init_clk_support(struct gk20a *g);

unsigned long gk20a_clk_get_rate(struct gk20a *g);
int gk20a_clk_set_rate(struct gk20a *g, unsigned long rate);
int gk20a_suspend_clk_support(struct gk20a *g);
struct clk *gk20a_clk_get(struct gk20a *g);
long gk20a_clk_round_rate(struct gk20a *g, unsigned long rate);

extern struct pll_parms gpc_pll_params;

#define KHZ 1000
#define MHZ 1000000

static inline unsigned long rate_gpc2clk_to_gpu(unsigned long rate)
{
	/* convert the kHz gpc2clk frequency to Hz gpcpll frequency */
	return (rate * KHZ) / 2;
}
static inline unsigned long rate_gpu_to_gpc2clk(unsigned long rate)
{
	/* convert the Hz gpcpll frequency to kHz gpc2clk frequency */
	return (rate * 2) / KHZ;
}

#endif /* _NVHOST_CLK_GK20A_H_ */
