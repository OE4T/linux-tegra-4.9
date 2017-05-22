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

#include "gk20a/gk20a.h"

#include "gp10b/ltc_gp10b.h"

#include "ltc_gv11b.h"

#include <nvgpu/hw/gv11b/hw_ltc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pri_ringmaster_gv11b.h>

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

static void gv11b_ltc_init_fs_state(struct gk20a *g)
{
	u32 ltc_intr;
	u32 reg;

	gk20a_dbg_info("initialize gv11b l2");

	g->max_ltc_count = gk20a_readl(g, top_num_ltcs_r());
	g->ltc_count = gk20a_readl(g, pri_ringmaster_enum_ltc_r());
	gk20a_dbg_info("%u ltcs out of %u", g->ltc_count, g->max_ltc_count);

	reg = gk20a_readl(g, ltc_ltcs_ltss_cbc_num_active_ltcs_r());
	reg |= ltc_ltcs_ltss_cbc_num_active_ltcs_serialize_f(true);
	gk20a_writel(g, ltc_ltcs_ltss_cbc_num_active_ltcs_r(), reg);

	gk20a_writel(g, ltc_ltcs_ltss_dstg_cfg0_r(),
		gk20a_readl(g, ltc_ltc0_lts0_dstg_cfg0_r()) |
		ltc_ltcs_ltss_dstg_cfg0_vdc_4to2_disable_m());

	/* Disable LTC interrupts */
	reg = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	reg &= ~ltc_ltcs_ltss_intr_en_evicted_cb_m();
	reg &= ~ltc_ltcs_ltss_intr_en_illegal_compstat_access_m();
	gk20a_writel(g, ltc_ltcs_ltss_intr_r(), reg);

	/* Enable ECC interrupts */
	ltc_intr = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	ltc_intr |= ltc_ltcs_ltss_intr_en_ecc_sec_error_enabled_f() |
		ltc_ltcs_ltss_intr_en_ecc_ded_error_enabled_f();
	gk20a_writel(g, ltc_ltcs_ltss_intr_r(),
				ltc_intr);
}

static u32 gv11b_ltc_cbc_fix_config(struct gk20a *g, int base)
{
	u32 val = gk20a_readl(g, ltc_ltcs_ltss_cbc_num_active_ltcs_r());

	if (ltc_ltcs_ltss_cbc_num_active_ltcs__v(val) == 2)
		return base * 2;
	else if (ltc_ltcs_ltss_cbc_num_active_ltcs__v(val) != 1) {
		nvgpu_err(g, "Invalid number of active ltcs: %08x", val);
	}
	return base;
}


void gv11b_init_ltc(struct gpu_ops *gops)
{
	gp10b_init_ltc(gops);
	gops->ltc.set_zbc_s_entry = gv11b_ltc_set_zbc_stencil_entry;
	gops->ltc.init_fs_state = gv11b_ltc_init_fs_state;
	gops->ltc.cbc_fix_config = gv11b_ltc_cbc_fix_config;
}
