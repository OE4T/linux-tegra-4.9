/*
 * drivers/video/tegra/host/gk20a/ltc_gk20a.c
 *
 * GK20A Graphics
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include "hw_ltc_gk20a.h"
#include "hw_proj_gk20a.h"

#include "ltc_common.c"

void gk20a_init_ltc(struct gpu_ops *gops)
{
	gops->ltc.determine_L2_size_bytes = gk20a_determine_L2_size_bytes;
	gops->ltc.set_max_ways_evict_last = gk20a_ltc_set_max_ways_evict_last;
	gops->ltc.init_comptags = gk20a_ltc_init_comptags;
	gops->ltc.clear_comptags = gk20a_ltc_clear_comptags;
	gops->ltc.set_zbc_color_entry = gk20a_ltc_set_zbc_color_entry;
	gops->ltc.set_zbc_depth_entry = gk20a_ltc_set_zbc_depth_entry;
	gops->ltc.clear_zbc_color_entry = gk20a_ltc_clear_zbc_color_entry;
	gops->ltc.clear_zbc_depth_entry = gk20a_ltc_clear_zbc_depth_entry;
	gops->ltc.init_zbc = gk20a_ltc_init_zbc;
	gops->ltc.init_cbc = gk20a_ltc_init_cbc;
}
