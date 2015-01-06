/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include "bpmp.h"

struct bpmp_cpuidle_state plat_cpuidle_state[] = {
	{ 0, NULL }
};

int bpmp_platdbg_init(struct dentry *root, struct platform_device *pdev)
{
	return 0;
}

int bpmp_clk_init(struct platform_device *pdev)
{
	return 0;
}
