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

#include "gk20a/gk20a.h"

#include "ltc_gm20b.h"
#include "gr_gm20b.h"
#include "ltc_gm20b.h"
#include "fb_gm20b.h"
#include "gm20b_gating_reglist.h"
#include "fifo_gm20b.h"
#include "gr_ctx_gm20b.h"
#include "mm_gm20b.h"
#include "pmu_gm20b.h"
#include "clk_gm20b.h"
#include <linux/tegra-fuse.h>
#include "regops_gm20b.h"

#define FUSE_OPT_PRIV_SEC_DIS_0 0x264
#define PRIV_SECURITY_DISABLE 0x01

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
#ifdef CONFIG_TEGRA_ACR
	if (tegra_platform_is_linsim()) {
		gops->privsecurity = 1;
	} else {
		if (tegra_fuse_readl(FUSE_OPT_PRIV_SEC_DIS_0) &
				PRIV_SECURITY_DISABLE) {
			gk20a_dbg_info("priv security is disabled in HW");
			gops->privsecurity = 0;
		} else {
			gops->privsecurity = 1;
		}
	}
#else
	if (tegra_platform_is_linsim()) {
		gk20a_dbg_info("running ASIM with PRIV security disabled");
		gops->privsecurity = 0;
	} else {
		if (tegra_fuse_readl(FUSE_OPT_PRIV_SEC_DIS_0) &
				PRIV_SECURITY_DISABLE) {
			gops->privsecurity = 0;
		} else {
			gk20a_dbg_info("priv security is not supported but enabled");
			gops->privsecurity = 1;
			return -EPERM;
		}
	}
#endif

	gm20b_init_ltc(gops);
	gm20b_init_gr(gops);
	gm20b_init_ltc(gops);
	gm20b_init_fb(gops);
	gm20b_init_fifo(gops);
	gm20b_init_gr_ctx(gops);
	gm20b_init_mm(gops);
	gm20b_init_pmu_ops(gops);
	gm20b_init_clk_ops(gops);
	gm20b_init_regops(gops);
	gops->name = "gm20b";

	return 0;
}
