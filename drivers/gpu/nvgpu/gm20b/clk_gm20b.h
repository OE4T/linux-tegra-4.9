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

void gm20b_init_clk_ops(struct gpu_ops *gops);

int gm20b_init_clk_setup_sw(struct gk20a *g);

int gm20b_clk_prepare(struct clk_gk20a *clk);
void gm20b_clk_unprepare(struct clk_gk20a *clk);
int gm20b_clk_is_prepared(struct clk_gk20a *clk);
unsigned long gm20b_recalc_rate(struct clk_gk20a *clk, unsigned long parent_rate);
int gm20b_gpcclk_set_rate(struct clk_gk20a *clk, unsigned long rate,
		unsigned long parent_rate);
long gm20b_round_rate(struct clk_gk20a *clk, unsigned long rate,
		unsigned long *parent_rate);

#endif /* _NVHOST_CLK_GM20B_H_ */
