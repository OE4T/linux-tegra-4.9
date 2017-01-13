/*
 * GV11B LTC
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include "gp10b/ltc_gp10b.h"
#include "gv11b/ltc_gv11b.h"
#include "hw_ltc_gv11b.h"

/*
 * Sets the ZBC stencil for the passed index.
 */
static void gv11b_ltc_set_zbc_stencil_entry(struct gk20a *g,
					  struct zbc_entry *stencil_val,
					  u32 index)
{
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	gk20a_writel(g, ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_r(),
		     stencil_val->depth);

	gk20a_readl(g, ltc_ltcs_ltss_dstg_zbc_index_r());
}

void gv11b_init_ltc(struct gpu_ops *gops)
{
	gp10b_init_ltc(gops);
	gops->ltc.set_zbc_s_entry = gv11b_ltc_set_zbc_stencil_entry;
}
