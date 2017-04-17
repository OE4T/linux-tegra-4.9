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
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>

static void gv11b_init_nvlink_soc_credits(struct gk20a *g)
{
	void __iomem *soc1 = ioremap(0x02b10010, 4096);
	void __iomem *soc2 = ioremap(0x02b20010, 4096);
	void __iomem *soc3 = ioremap(0x02b30010, 4096);
	void __iomem *soc4 = ioremap(0x02b40010, 4096);

	gk20a_dbg_info("init nvlink soc credits");

	writel_relaxed(0x14050000, soc1);
	writel_relaxed(0x08020000, soc1 + 4);
	writel_relaxed(0x14050000, soc2);
	writel_relaxed(0x08020000, soc2 + 4);
	writel_relaxed(0x14050000, soc3);
	writel_relaxed(0x08020000, soc3 + 4);
	writel_relaxed(0x14050000, soc4);
	writel_relaxed(0x08020000, soc4 + 4);

}

static void gv11b_fb_reset(struct gk20a *g)
{
	u32 val;

	gk20a_dbg_info("reset gv11b fb");

	g->ops.mc.reset(g, mc_enable_pfb_enabled_f() |
				mc_enable_l2_enabled_f() |
				mc_enable_xbar_enabled_f() |
				mc_enable_hub_enabled_f());

	val = gk20a_readl(g, mc_elpg_enable_r());
	val |= mc_elpg_enable_xbar_enabled_f() |
		mc_elpg_enable_pfb_enabled_f() |
		mc_elpg_enable_l2_enabled_f() |
		mc_elpg_enable_hub_enabled_f();
	gk20a_writel(g, mc_elpg_enable_r(), val);

	/* fs hub should be out of reset by now */
	gv11b_init_nvlink_soc_credits(g);

	val = gk20a_readl(g, fifo_fb_iface_r());
	gk20a_dbg_info("fifo_fb_iface val = 0x%x", val);
	if (!(val & fifo_fb_iface_control_enable_f() &&
		val & fifo_fb_iface_status_enabled_f())) {
		gk20a_dbg_info("fifo_fb_iface set control enable");
		gk20a_writel(g, fifo_fb_iface_r(),
				fifo_fb_iface_control_enable_f());
		val = gk20a_readl(g, fifo_fb_iface_r());
		gk20a_dbg_info("fifo_fb_iface val = 0x%x", val);
	}
}

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
	gops->fb.reset = gv11b_fb_reset;

	gv11b_init_uncompressed_kind_map();
	gv11b_init_kind_attr();

}
