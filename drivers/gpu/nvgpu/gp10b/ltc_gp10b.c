/*
 * GP10B L2
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
 */

#include <linux/types.h>

#include "gk20a/gk20a.h"
#include "gm20b/ltc_gm20b.h"
#include "hw_ltc_gp10b.h"

static int gp10b_determine_L2_size_bytes(struct gk20a *g)
{
	u32 tmp;
	int ret;

	gk20a_dbg_fn("");

	tmp = gk20a_readl(g, ltc_ltc0_lts0_tstg_info_1_r());

	ret = g->ltc_count *
		ltc_ltc0_lts0_tstg_info_1_slice_size_in_kb_v(tmp) *
		ltc_ltc0_lts0_tstg_info_1_slices_per_l2_v(tmp);

	gk20a_dbg(gpu_dbg_info, "L2 size: %d\n", ret);

	gk20a_dbg_fn("done");

	return ret;
}

void gp10b_init_ltc(struct gpu_ops *gops)
{
	gm20b_init_ltc(gops);

	gops->ltc.determine_L2_size_bytes = gp10b_determine_L2_size_bytes;
}
