/*
 * GV11B FB
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
#include "gk20a/kind_gk20a.h"

#include "gp10b/fb_gp10b.h"

#include "gv11b/fb_gv11b.h"

#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fb_gv11b.h>

static void gv11b_init_uncompressed_kind_map(void)
{
	gk20a_uc_kind_map[gmmu_pte_kind_c32_ms2_4cbra_v()] =
	gk20a_uc_kind_map[gmmu_pte_kind_c64_ms2_4cbra_v()] =
		gmmu_pte_kind_generic_16bx2_v();
}

static bool gv11b_kind_supported(u8 k)
{
	return (k == gmmu_pte_kind_c32_ms2_4cbra_v()
		|| k == gmmu_pte_kind_c64_ms2_4cbra_v());
}

static bool gv11b_kind_z(u8 k)
{
	return (k == gmmu_pte_kind_c32_ms2_4cbra_v()
		|| k == gmmu_pte_kind_c64_ms2_4cbra_v());
}

static bool gv11b_kind_compressible(u8 k)
{

	return (k == gmmu_pte_kind_c32_ms2_4cbra_v()
		|| k == gmmu_pte_kind_c64_ms2_4cbra_v());
}

static bool gv11b_kind_zbc(u8 k)
{

	return (k == gmmu_pte_kind_c32_ms2_4cbra_v()
		|| k == gmmu_pte_kind_c64_ms2_4cbra_v());
}

static void gv11b_init_kind_attr(void)
{
	u16 k;

	for (k = 0; k < 256; k++) {
		if (gv11b_kind_supported((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_SUPPORTED;
		if (gv11b_kind_compressible((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_COMPRESSIBLE;
		if (gv11b_kind_z((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_Z;
		if (gv11b_kind_zbc((u8)k))
			gk20a_kind_attr[k] |= GK20A_KIND_ATTR_ZBC;
	}
}

static void gv11b_fb_hub_isr(struct gk20a *g)
{
	u32 niso_intr = gk20a_readl(g, fb_niso_intr_r());

	gk20a_dbg_info("enter hub isr, niso_intr = 0x%x", niso_intr);

	if (niso_intr &
		 (fb_niso_intr_hub_access_counter_notify_pending_f() |
		  fb_niso_intr_hub_access_counter_error_pending_f())) {

		gk20a_dbg_info("hub access counter notify/error");
	} else {
		gk20a_dbg_info("mmu fault : TODO");
	}
}

void gv11b_init_fb(struct gpu_ops *gops)
{
	gp10b_init_fb(gops);
	gops->fb.hub_isr = gv11b_fb_hub_isr;
	gv11b_init_uncompressed_kind_map();
	gv11b_init_kind_attr();

}
