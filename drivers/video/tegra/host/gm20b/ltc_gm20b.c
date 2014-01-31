/*
 * GM20B L2
 *
 * Copyright (c) 2014 NVIDIA CORPORATION.  All rights reserved.
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
#include <dev.h>
#include "gk20a/gk20a.h"
#include "hw_ltc_gm20b.h"
#include "hw_top_gm20b.h"
#include "hw_pri_ringmaster_gm20b.h"

#include "gk20a/ltc_common.c"

#include "hw_top_gm20b.h"
#include "hw_pri_ringmaster_gm20b.h"

static void ltc_gm20b_init_fs_state(struct gk20a *g)
{
	nvhost_dbg_info("initialize gm20b l2");

	g->max_ltc_count = gk20a_readl(g, top_num_ltcs_r());
	g->ltc_count = gk20a_readl(g, pri_ringmaster_enum_ltc_r());
	nvhost_dbg_info("%d ltcs out of %d", g->ltc_count, g->max_ltc_count);

	gk20a_writel(g, ltc_ltcs_ltss_cbc_num_active_ltcs_r(),
	g->ltc_count);
	gk20a_writel(g, ltc_ltcs_misc_ltc_num_active_ltcs_r(),
	g->ltc_count);
}
void gm20b_init_ltc(struct gpu_ops *gops)
{
	gops->ltc.determine_L2_size_bytes = gk20a_determine_L2_size_bytes;
	gops->ltc.init_fs_state = ltc_gm20b_init_fs_state;
}
