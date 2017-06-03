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

#include <nvgpu/log.h>
#include <nvgpu/enabled.h>

static void gv11b_init_nvlink_soc_credits(struct gk20a *g)
{
	void __iomem *soc1 = ioremap(0x01f20010, 4096); //MSS_NVLINK_1_BASE
	void __iomem *soc2 = ioremap(0x01f40010, 4096); //MSS_NVLINK_2_BASE
	void __iomem *soc3 = ioremap(0x01f60010, 4096); //MSS_NVLINK_3_BASE
	void __iomem *soc4 = ioremap(0x01f80010, 4096); //MSS_NVLINK_4_BASE
	u32 val;

	/* TODO : replace this code with proper nvlink API */
	nvgpu_info(g, "init nvlink soc credits");

	val = readl_relaxed(soc1);
	writel_relaxed(val, soc1);
	val = readl_relaxed(soc1 + 4);
	writel_relaxed(val, soc1 + 4);

	val = readl_relaxed(soc2);
	writel_relaxed(val, soc2);
	val = readl_relaxed(soc2 + 4);
	writel_relaxed(val, soc2 + 4);

	val = readl_relaxed(soc3);
	writel_relaxed(val, soc3);
	val = readl_relaxed(soc3 + 4);
	writel_relaxed(val, soc3 + 4);

	val = readl_relaxed(soc4);
	writel_relaxed(val, soc4);
	val = readl_relaxed(soc4 + 4);
	writel_relaxed(val, soc4 + 4);

}

static void gv11b_fb_init_fs_state(struct gk20a *g)
{
	nvgpu_log(g, gpu_dbg_fn, "initialize gv11b fb");

	nvgpu_log(g, gpu_dbg_info, "fbhub active ltcs %u",
			gk20a_readl(g, fb_fbhub_num_active_ltcs_r()));

	gk20a_writel(g, fb_mmu_num_active_ltcs_r(),
			fb_mmu_num_active_ltcs_count_f(g->ltc_count));

	nvgpu_log(g, gpu_dbg_info, "mmu active ltcs %u",
			fb_mmu_num_active_ltcs_count_v(
			gk20a_readl(g, fb_mmu_num_active_ltcs_r())));
}

static void gv11b_fb_init_cbc(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 max_size = gr->max_comptag_mem;
	/* one tag line covers 64KB */
	u32 max_comptag_lines = max_size << 4;
	u32 compbit_base_post_divide;
	u64 compbit_base_post_multiply64;
	u64 compbit_store_iova;
	u64 compbit_base_post_divide64;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		compbit_store_iova = gk20a_mem_phys(&gr->compbit_store.mem);
	else
		compbit_store_iova = g->ops.mm.get_iova_addr(g,
				gr->compbit_store.mem.priv.sgt->sgl, 0);

	compbit_base_post_divide64 = compbit_store_iova >>
		fb_mmu_cbc_base_address_alignment_shift_v();

	do_div(compbit_base_post_divide64, g->ltc_count);
	compbit_base_post_divide = u64_lo32(compbit_base_post_divide64);

	compbit_base_post_multiply64 = ((u64)compbit_base_post_divide *
		g->ltc_count) << fb_mmu_cbc_base_address_alignment_shift_v();

	if (compbit_base_post_multiply64 < compbit_store_iova)
		compbit_base_post_divide++;

	if (g->ops.ltc.cbc_fix_config)
		compbit_base_post_divide =
			g->ops.ltc.cbc_fix_config(g, compbit_base_post_divide);

	gk20a_writel(g, fb_mmu_cbc_base_r(),
		fb_mmu_cbc_base_address_f(compbit_base_post_divide));

	nvgpu_log(g, gpu_dbg_info | gpu_dbg_map_v | gpu_dbg_pte,
		"compbit base.pa: 0x%x,%08x cbc_base:0x%08x\n",
		(u32)(compbit_store_iova >> 32),
		(u32)(compbit_store_iova & 0xffffffff),
		compbit_base_post_divide);
	nvgpu_log(g, gpu_dbg_fn, "cbc base %x",
		gk20a_readl(g, fb_mmu_cbc_base_r()));

	gr->compbit_store.base_hw = compbit_base_post_divide;

	g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_invalidate,
			0, max_comptag_lines - 1);

}

static void gv11b_fb_reset(struct gk20a *g)
{
	u32 val;

	nvgpu_info(g, "reset gv11b fb");

	g->ops.mc.reset(g, mc_enable_pfb_enabled_f() |
				mc_enable_xbar_enabled_f() |
				mc_enable_hub_enabled_f());

	val = gk20a_readl(g, mc_elpg_enable_r());
	val |= mc_elpg_enable_xbar_enabled_f() |
		mc_elpg_enable_pfb_enabled_f() |
		mc_elpg_enable_hub_enabled_f();
	gk20a_writel(g, mc_elpg_enable_r(), val);

	/* fs hub should be out of reset by now */
	gv11b_init_nvlink_soc_credits(g);

	val = gk20a_readl(g, fifo_fb_iface_r());
	nvgpu_info(g, "fifo_fb_iface val = 0x%x", val);
	if (!(val & fifo_fb_iface_control_enable_f() &&
		val & fifo_fb_iface_status_enabled_f())) {
		nvgpu_info(g, "fifo_fb_iface set control enable");
		gk20a_writel(g, fifo_fb_iface_r(),
				fifo_fb_iface_control_enable_f());
		val = gk20a_readl(g, fifo_fb_iface_r());
		nvgpu_info(g, "fifo_fb_iface val = 0x%x", val);
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

static void gv11b_fb_intr_en_set(struct gk20a *g,
			 unsigned int index, u32 mask)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fb_niso_intr_en_set_r(index));
	reg_val |= mask;
	gk20a_writel(g, fb_niso_intr_en_set_r(index), reg_val);
}

static void gv11b_fb_intr_en_clr(struct gk20a *g,
			 unsigned int index, u32 mask)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fb_niso_intr_en_clr_r(index));
	reg_val |= mask;
	gk20a_writel(g, fb_niso_intr_en_clr_r(index), reg_val);
}

static u32 gv11b_fb_get_hub_intr_clr_mask(struct gk20a *g,
			 unsigned int intr_type)
{
	u32 mask = 0;

	if (intr_type == HUB_INTR_TYPE_ALL) {
		mask |=
		 fb_niso_intr_en_clr_mmu_ecc_uncorrected_error_notify_set_f();
		return mask;
	}

	if (intr_type & HUB_INTR_TYPE_ECC_UNCORRECTED) {
		mask |=
		 fb_niso_intr_en_clr_mmu_ecc_uncorrected_error_notify_set_f();
	}

	return mask;
}

static u32 gv11b_fb_get_hub_intr_en_mask(struct gk20a *g,
			 unsigned int intr_type)
{
	u32 mask = 0;

	if (intr_type == HUB_INTR_TYPE_ALL) {
		mask |=
		 fb_niso_intr_en_set_mmu_ecc_uncorrected_error_notify_set_f();
		return mask;
	}

	if (intr_type & HUB_INTR_TYPE_ECC_UNCORRECTED) {
		mask |=
		 fb_niso_intr_en_set_mmu_ecc_uncorrected_error_notify_set_f();
	}

	return mask;
}

void gv11b_fb_enable_hub_intr(struct gk20a *g,
			 unsigned int index, unsigned int intr_type)
{
	u32 mask = 0;

	mask = gv11b_fb_get_hub_intr_en_mask(g, intr_type);

	if (mask)
		gv11b_fb_intr_en_set(g, index, mask);
}

void gv11b_fb_disable_hub_intr(struct gk20a *g,
			 unsigned int index, unsigned int intr_type)
{
	u32 mask = 0;

	mask = gv11b_fb_get_hub_intr_clr_mask(g, intr_type);

	if (mask)
		gv11b_fb_intr_en_clr(g, index, mask);
}

static void gv11b_handle_l2tlb_ecc_isr(struct gk20a *g, u32 ecc_status)
{
	u32 ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;

	ecc_addr = gk20a_readl(g, fb_mmu_l2tlb_ecc_address_r());
	corrected_cnt = gk20a_readl(g,
		fb_mmu_l2tlb_ecc_corrected_err_count_r());
	uncorrected_cnt = gk20a_readl(g,
		fb_mmu_l2tlb_ecc_uncorrected_err_count_r());

	corrected_delta = fb_mmu_l2tlb_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = fb_mmu_l2tlb_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		fb_mmu_l2tlb_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		fb_mmu_l2tlb_ecc_status_uncorrected_err_total_counter_overflow_m();

	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow)
		gk20a_writel(g, fb_mmu_l2tlb_ecc_corrected_err_count_r(), 0);
	if ((uncorrected_delta > 0) || uncorrected_overflow)
		gk20a_writel(g, fb_mmu_l2tlb_ecc_uncorrected_err_count_r(), 0);

	gk20a_writel(g, fb_mmu_l2tlb_ecc_status_r(),
				fb_mmu_l2tlb_ecc_status_reset_clear_f());

	/* Handle overflow */
	if (corrected_overflow)
		corrected_delta += (0x1UL << fb_mmu_l2tlb_ecc_corrected_err_count_total_s());
	if (uncorrected_overflow)
		uncorrected_delta += (0x1UL << fb_mmu_l2tlb_ecc_uncorrected_err_count_total_s());


	g->ecc.eng.t19x.mmu_l2tlb_corrected_err_count.counters[0] +=
							corrected_delta;
	g->ecc.eng.t19x.mmu_l2tlb_uncorrected_err_count.counters[0] +=
							uncorrected_delta;

	if (ecc_status & fb_mmu_l2tlb_ecc_status_corrected_err_l2tlb_sa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc sa data error");
	if (ecc_status & fb_mmu_l2tlb_ecc_status_uncorrected_err_l2tlb_sa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc sa data error");
	if (corrected_overflow || uncorrected_overflow)
		nvgpu_info(g, "mmu l2tlb ecc counter overflow!");

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error address: 0x%x", ecc_addr);
	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.eng.t19x.mmu_l2tlb_corrected_err_count.counters[0],
		g->ecc.eng.t19x.mmu_l2tlb_uncorrected_err_count.counters[0]);
}

static void gv11b_handle_hubtlb_ecc_isr(struct gk20a *g, u32 ecc_status)
{
	u32 ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;

	ecc_addr = gk20a_readl(g, fb_mmu_hubtlb_ecc_address_r());
	corrected_cnt = gk20a_readl(g,
		fb_mmu_hubtlb_ecc_corrected_err_count_r());
	uncorrected_cnt = gk20a_readl(g,
		fb_mmu_hubtlb_ecc_uncorrected_err_count_r());

	corrected_delta = fb_mmu_hubtlb_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = fb_mmu_hubtlb_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		fb_mmu_hubtlb_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		fb_mmu_hubtlb_ecc_status_uncorrected_err_total_counter_overflow_m();

	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow)
		gk20a_writel(g, fb_mmu_hubtlb_ecc_corrected_err_count_r(), 0);
	if ((uncorrected_delta > 0) || uncorrected_overflow)
		gk20a_writel(g, fb_mmu_hubtlb_ecc_uncorrected_err_count_r(), 0);

	gk20a_writel(g, fb_mmu_hubtlb_ecc_status_r(),
				fb_mmu_hubtlb_ecc_status_reset_clear_f());

	/* Handle overflow */
	if (corrected_overflow)
		corrected_delta += (0x1UL << fb_mmu_hubtlb_ecc_corrected_err_count_total_s());
	if (uncorrected_overflow)
		uncorrected_delta += (0x1UL << fb_mmu_hubtlb_ecc_uncorrected_err_count_total_s());


	g->ecc.eng.t19x.mmu_hubtlb_corrected_err_count.counters[0] +=
							corrected_delta;
	g->ecc.eng.t19x.mmu_hubtlb_uncorrected_err_count.counters[0] +=
							uncorrected_delta;

	if (ecc_status & fb_mmu_hubtlb_ecc_status_corrected_err_sa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc sa data error");
	if (ecc_status & fb_mmu_hubtlb_ecc_status_uncorrected_err_sa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc sa data error");
	if (corrected_overflow || uncorrected_overflow)
		nvgpu_info(g, "mmu hubtlb ecc counter overflow!");

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error address: 0x%x", ecc_addr);
	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.eng.t19x.mmu_hubtlb_corrected_err_count.counters[0],
		g->ecc.eng.t19x.mmu_hubtlb_uncorrected_err_count.counters[0]);
}

static void gv11b_handle_fillunit_ecc_isr(struct gk20a *g, u32 ecc_status)
{
	u32 ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;

	ecc_addr = gk20a_readl(g, fb_mmu_fillunit_ecc_address_r());
	corrected_cnt = gk20a_readl(g,
		fb_mmu_fillunit_ecc_corrected_err_count_r());
	uncorrected_cnt = gk20a_readl(g,
		fb_mmu_fillunit_ecc_uncorrected_err_count_r());

	corrected_delta = fb_mmu_fillunit_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = fb_mmu_fillunit_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		fb_mmu_fillunit_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		fb_mmu_fillunit_ecc_status_uncorrected_err_total_counter_overflow_m();

	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow)
		gk20a_writel(g, fb_mmu_fillunit_ecc_corrected_err_count_r(), 0);
	if ((uncorrected_delta > 0) || uncorrected_overflow)
		gk20a_writel(g, fb_mmu_fillunit_ecc_uncorrected_err_count_r(), 0);

	gk20a_writel(g, fb_mmu_fillunit_ecc_status_r(),
				fb_mmu_fillunit_ecc_status_reset_clear_f());

	/* Handle overflow */
	if (corrected_overflow)
		corrected_delta += (0x1UL << fb_mmu_fillunit_ecc_corrected_err_count_total_s());
	if (uncorrected_overflow)
		uncorrected_delta += (0x1UL << fb_mmu_fillunit_ecc_uncorrected_err_count_total_s());


	g->ecc.eng.t19x.mmu_fillunit_corrected_err_count.counters[0] +=
							corrected_delta;
	g->ecc.eng.t19x.mmu_fillunit_uncorrected_err_count.counters[0] +=
							uncorrected_delta;

	if (ecc_status & fb_mmu_fillunit_ecc_status_corrected_err_pte_data_m())
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc pte data error");
	if (ecc_status & fb_mmu_fillunit_ecc_status_uncorrected_err_pte_data_m())
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc pte data error");
	if (ecc_status & fb_mmu_fillunit_ecc_status_corrected_err_pde0_data_m())
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc pde0 data error");
	if (ecc_status & fb_mmu_fillunit_ecc_status_uncorrected_err_pde0_data_m())
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc pde0 data error");

	if (corrected_overflow || uncorrected_overflow)
		nvgpu_info(g, "mmu fillunit ecc counter overflow!");

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error address: 0x%x", ecc_addr);
	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.eng.t19x.mmu_fillunit_corrected_err_count.counters[0],
		g->ecc.eng.t19x.mmu_fillunit_uncorrected_err_count.counters[0]);
}

static void gv11b_fb_hub_isr(struct gk20a *g)
{
	u32 status;
	u32 niso_intr = gk20a_readl(g, fb_niso_intr_r());

	nvgpu_info(g, "enter hub isr, niso_intr = 0x%x", niso_intr);

	if (niso_intr &
		 (fb_niso_intr_hub_access_counter_notify_pending_f() |
		  fb_niso_intr_hub_access_counter_error_pending_f())) {

		nvgpu_info(g, "hub access counter notify/error");
	} else if (niso_intr &
		fb_niso_intr_mmu_ecc_uncorrected_error_notify_pending_f()) {

		nvgpu_info(g, "ecc uncorrected error notify");

		/* disable interrupts during handling */
		gv11b_fb_disable_hub_intr(g, STALL_REG_INDEX,
						HUB_INTR_TYPE_ECC_UNCORRECTED);

		status = gk20a_readl(g, fb_mmu_l2tlb_ecc_status_r());
		if (status)
			gv11b_handle_l2tlb_ecc_isr(g, status);

		status = gk20a_readl(g, fb_mmu_hubtlb_ecc_status_r());
		if (status)
			gv11b_handle_hubtlb_ecc_isr(g, status);

		status = gk20a_readl(g, fb_mmu_fillunit_ecc_status_r());
		if (status)
			gv11b_handle_fillunit_ecc_isr(g, status);

		/* re-enable interrupts after handling */
		gv11b_fb_enable_hub_intr(g, STALL_REG_INDEX,
						HUB_INTR_TYPE_ECC_UNCORRECTED);

	} else {
		nvgpu_info(g, "mmu fault : TODO");
	}
}

void gv11b_init_fb(struct gpu_ops *gops)
{
	gp10b_init_fb(gops);
	gops->fb.hub_isr = gv11b_fb_hub_isr;
	gops->fb.reset = gv11b_fb_reset;
	gops->fb.init_fs_state = gv11b_fb_init_fs_state;
	gops->fb.init_cbc = gv11b_fb_init_cbc;

	gv11b_init_uncompressed_kind_map();
	gv11b_init_kind_attr();

}
