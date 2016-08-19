/*
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA_DRIVERS_CLK_H
#define __TEGRA_DRIVERS_CLK_H

int tegra_fake_clks_init(struct device_node *np);

struct clk *tegra_clk_register_bpmp(const char *name, int parent,
		const char **parent_names, int *parent_ids, int num_parents,
		unsigned long flags, int clk_num, int bpmp_flags);

struct clk **tegra_bpmp_clk_init(struct device_node *np);

#endif
