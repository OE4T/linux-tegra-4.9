/*
 * GM20B Graphics
 *
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
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef _NVHOST_CLK_GM20B_H_
#define _NVHOST_CLK_GM20B_H_

#include <nvgpu/lock.h>

struct nvgpu_clk_pll_debug_data {
	u32 trim_sys_sel_vco_reg;
	u32 trim_sys_sel_vco_val;

	u32 trim_sys_gpc2clk_out_reg;
	u32 trim_sys_gpc2clk_out_val;

	u32 trim_sys_bypassctrl_reg;
	u32 trim_sys_bypassctrl_val;

	u32 trim_sys_gpcpll_cfg_reg;
	u32 trim_sys_gpcpll_dvfs2_reg;

	u32 trim_sys_gpcpll_cfg_val;
	bool trim_sys_gpcpll_cfg_enabled;
	bool trim_sys_gpcpll_cfg_locked;
	bool trim_sys_gpcpll_cfg_sync_on;

	u32 trim_sys_gpcpll_coeff_val;
	u32 trim_sys_gpcpll_coeff_mdiv;
	u32 trim_sys_gpcpll_coeff_ndiv;
	u32 trim_sys_gpcpll_coeff_pldiv;

	u32 trim_sys_gpcpll_dvfs0_val;
	u32 trim_sys_gpcpll_dvfs0_dfs_coeff;
	u32 trim_sys_gpcpll_dvfs0_dfs_det_max;
	u32 trim_sys_gpcpll_dvfs0_dfs_dc_offset;
};

int gm20b_init_clk_setup_sw(struct gk20a *g);

int gm20b_clk_prepare(struct clk_gk20a *clk);
void gm20b_clk_unprepare(struct clk_gk20a *clk);
int gm20b_clk_is_prepared(struct clk_gk20a *clk);
unsigned long gm20b_recalc_rate(struct clk_gk20a *clk, unsigned long parent_rate);
int gm20b_gpcclk_set_rate(struct clk_gk20a *clk, unsigned long rate,
		unsigned long parent_rate);
long gm20b_round_rate(struct clk_gk20a *clk, unsigned long rate,
		unsigned long *parent_rate);
struct pll_parms *gm20b_get_gpc_pll_parms(void);
#ifdef CONFIG_DEBUG_FS
int gm20b_clk_init_debugfs(struct gk20a *g);
#endif

int gm20b_clk_pll_reg_write(struct gk20a *g, u32 reg, u32 val);
int gm20b_init_clk_support(struct gk20a *g);
int gm20b_suspend_clk_support(struct gk20a *g);
int gm20b_clk_get_voltage(struct clk_gk20a *clk, u64 *val);
int gm20b_clk_get_gpcclk_clock_counter(struct clk_gk20a *clk, u64 *val);
int gm20b_clk_get_pll_debug_data(struct gk20a *g,
			struct nvgpu_clk_pll_debug_data *d);

/* 1:1 match between post divider settings and divisor value */
static inline u32 nvgpu_pl_to_div(u32 pl)
{
	return pl;
}

static inline u32 nvgpu_div_to_pl(u32 div)
{
	return div;
}

#endif /* _NVHOST_CLK_GM20B_H_ */
