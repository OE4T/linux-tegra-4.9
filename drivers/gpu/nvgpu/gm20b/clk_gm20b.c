/*
 * GM20B Clocks
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>	/* for mdelay */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>
#include <mach/thermal.h>

#include "gk20a/gk20a.h"
#include "gk20a/hw_trim_gk20a.h"
#include "gk20a/hw_timer_gk20a.h"

void gm20b_init_clk_ops(struct gpu_ops *gops)
{
	gops->clk.init_clk_support = gk20a_init_clk_support;
}

