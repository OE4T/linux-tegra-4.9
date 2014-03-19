/*
 * GM20B Graphics
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
#include "dev.h"
#include "gk20a/gk20a.h"
#include "ltc_gm20b.h"
#include "gr_gm20b.h"
#include "ltc_gm20b.h"
#include "fb_gm20b.h"

#include "gm20b_gating_reglist.h"
struct gpu_ops gm20b_ops = {
	.clock_gating = {
		.slcg_gr_load_gating_prod =
			gr_gm20b_slcg_gr_load_gating_prod,
		.slcg_perf_load_gating_prod =
			gr_gm20b_slcg_perf_load_gating_prod,
		.blcg_gr_load_gating_prod =
			gr_gm20b_blcg_gr_load_gating_prod,
		.pg_gr_load_gating_prod =
			gr_gm20b_pg_gr_load_gating_prod,
		.slcg_therm_load_gating_prod =
			gr_gm20b_slcg_therm_load_gating_prod,
	}
};

int gm20b_init_hal(struct gpu_ops *gops)
{
	*gops = gm20b_ops;
	gm20b_init_ltc(gops);
	gm20b_init_gr(gops);
	gm20b_init_ltc(gops);
	gm20b_init_fb(gops);
	gops->name = "gm20b";

	return 0;
}
