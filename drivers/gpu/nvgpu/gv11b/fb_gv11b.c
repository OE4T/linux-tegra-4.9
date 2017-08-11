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

#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/gmmu.h>

#include "gk20a/gk20a.h"
#include "gk20a/kind_gk20a.h"
#include "gk20a/mm_gk20a.h"

#include "gp10b/fb_gp10b.h"

#include "gv11b/fifo_gv11b.h"
#include "gv11b/fb_gv11b.h"
#include "gv11b/ce_gv11b.h"

#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fb_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>


static int gv11b_fb_fix_page_fault(struct gk20a *g,
		 struct mmu_fault_info *mmfault);

static int gv11b_fb_mmu_invalidate_replay(struct gk20a *g,
			 u32 invalidate_replay_val);

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

	nvgpu_log(g, gpu_dbg_info, "fbhub active ltcs %x",
			gk20a_readl(g, fb_fbhub_num_active_ltcs_r()));

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
		compbit_store_iova = nvgpu_mem_get_addr(g,
							&gr->compbit_store.mem);

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

static const char * const invalid_str = "invalid";

static const char *const fault_type_descs_gv11b[] = {
	"invalid pde",
	"invalid pde size",
	"invalid pte",
	"limit violation",
	"unbound inst block",
	"priv violation",
	"write",
	"read",
	"pitch mask violation",
	"work creation",
	"unsupported aperture",
	"compression failure",
	"unsupported kind",
	"region violation",
	"poison",
	"atomic"
};

static const char *const fault_client_type_descs_gv11b[] = {
	"gpc",
	"hub",
};

static const char *const fault_access_type_descs_gv11b[] = {
	"virt read",
	"virt write",
	"virt atomic strong",
	"virt prefetch",
	"virt atomic weak",
	"xxx",
	"xxx",
	"xxx",
	"phys read",
	"phys write",
	"phys atomic",
	"phys prefetch",
};

static const char *const hub_client_descs_gv11b[] = {
	"vip", "ce0", "ce1", "dniso", "fe", "fecs", "host", "host cpu",
	"host cpu nb", "iso", "mmu", "nvdec", "nvenc1", "nvenc2",
	"niso", "p2p", "pd", "perf", "pmu", "raster twod", "scc",
	"scc nb", "sec", "ssync", "gr copy", "xv", "mmu nb",
	"nvenc", "d falcon", "sked", "a falcon", "hsce0", "hsce1",
	"hsce2", "hsce3", "hsce4", "hsce5", "hsce6", "hsce7", "hsce8",
	"hsce9", "hshub", "ptp x0", "ptp x1", "ptp x2", "ptp x3",
	"ptp x4", "ptp x5", "ptp x6", "ptp x7", "vpr scrubber0",
	"vpr scrubber1", "dwbif", "fbfalcon", "ce shim", "gsp",
	"dont care"
};

static const char *const gpc_client_descs_gv11b[] = {
	"t1 0", "t1 1", "t1 2", "t1 3",
	"t1 4", "t1 5", "t1 6", "t1 7",
	"pe 0", "pe 1", "pe 2", "pe 3",
	"pe 4", "pe 5", "pe 6", "pe 7",
	"rast", "gcc", "gpccs",
	"prop 0", "prop 1", "prop 2", "prop 3",
	"gpm",
	"ltp utlb 0", "ltp utlb 1", "ltp utlb 2", "ltp utlb 3",
	"ltp utlb 4", "ltp utlb 5", "ltp utlb 6", "ltp utlb 7",
	"utlb",
	"t1 8", "t1 9", "t1 10", "t1 11",
	"t1 12", "t1 13", "t1 14", "t1 15",
	"tpccs 0", "tpccs 1", "tpccs 2", "tpccs 3",
	"tpccs 4", "tpccs 5", "tpccs 6", "tpccs 7",
	"pe 8", "pe 9", "tpccs 8", "tpccs 9",
	"t1 16", "t1 17", "t1 18", "t1 19",
	"pe 10", "pe 11", "tpccs 10", "tpccs 11",
	"t1 20", "t1 21", "t1 22", "t1 23",
	"pe 12", "pe 13", "tpccs 12", "tpccs 13",
	"t1 24", "t1 25", "t1 26", "t1 27",
	"pe 14", "pe 15", "tpccs 14", "tpccs 15",
	"t1 28", "t1 29", "t1 30", "t1 31",
	"pe 16", "pe 17", "tpccs 16", "tpccs 17",
	"t1 32", "t1 33", "t1 34", "t1 35",
	"pe 18", "pe 19", "tpccs 18", "tpccs 19",
	"t1 36", "t1 37", "t1 38", "t1 39",
};

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

u32 gv11b_fb_is_fault_buf_enabled(struct gk20a *g,
				 unsigned int index)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_size_r(index));
	return fb_mmu_fault_buffer_size_enable_v(reg_val);
}

static void gv11b_fb_fault_buffer_get_ptr_update(struct gk20a *g,
				 unsigned int index, u32 next)
{
	u32 reg_val;

	nvgpu_log(g, gpu_dbg_intr, "updating get index with = %d", next);

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_get_r(index));
	reg_val = set_field(reg_val, fb_mmu_fault_buffer_get_ptr_m(),
			 fb_mmu_fault_buffer_get_ptr_f(next));

	/* while the fault is being handled it is possible for overflow
	 * to happen,
	 */
	if (reg_val & fb_mmu_fault_buffer_get_overflow_m())
		reg_val |= fb_mmu_fault_buffer_get_overflow_clear_f();

	gk20a_writel(g, fb_mmu_fault_buffer_get_r(index), reg_val);

	/* make sure get ptr update is visible to everyone to avoid
	 * reading already read entry
	 */
	mb();
}

static u32 gv11b_fb_fault_buffer_get_index(struct gk20a *g,
			unsigned int index)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_get_r(index));
	return fb_mmu_fault_buffer_get_ptr_v(reg_val);
}

static u32 gv11b_fb_fault_buffer_put_index(struct gk20a *g,
				 unsigned int index)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_put_r(index));
	return fb_mmu_fault_buffer_put_ptr_v(reg_val);
}

static u32 gv11b_fb_fault_buffer_size_val(struct gk20a *g,
				 unsigned int index)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_size_r(index));
	return fb_mmu_fault_buffer_size_val_v(reg_val);
}

static bool gv11b_fb_is_fault_buffer_empty(struct gk20a *g,
		 unsigned int index, u32 *get_idx)
{
	u32 put_idx;

	*get_idx = gv11b_fb_fault_buffer_get_index(g, index);
	put_idx = gv11b_fb_fault_buffer_put_index(g, index);

	return *get_idx == put_idx;
}

static bool gv11b_fb_is_fault_buffer_full(struct gk20a *g,
				 unsigned int index)
{
	u32 get_idx, put_idx, entries;


	get_idx = gv11b_fb_fault_buffer_get_index(g, index);

	put_idx = gv11b_fb_fault_buffer_put_index(g, index);

	entries = gv11b_fb_fault_buffer_size_val(g, index);

	return get_idx == ((put_idx + 1) % entries);
}

void gv11b_fb_fault_buf_set_state_hw(struct gk20a *g,
		 unsigned int index, unsigned int state)
{
	u32 fault_status;
	u32 reg_val;

	nvgpu_log_fn(g, " ");

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_size_r(index));
	if (state) {
		if (gv11b_fb_is_fault_buf_enabled(g, index)) {
			nvgpu_log_info(g, "fault buffer is already enabled");
		} else {
			reg_val |= fb_mmu_fault_buffer_size_enable_true_f();
			gk20a_writel(g, fb_mmu_fault_buffer_size_r(index),
					 reg_val);
		}

	} else {
		struct nvgpu_timeout timeout;
		u32 delay = GR_IDLE_CHECK_DEFAULT;

		nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

		reg_val &= (~(fb_mmu_fault_buffer_size_enable_m()));
		gk20a_writel(g, fb_mmu_fault_buffer_size_r(index), reg_val);

		fault_status = gk20a_readl(g, fb_mmu_fault_status_r());

		do {
			if (!(fault_status & fb_mmu_fault_status_busy_true_f()))
				break;
			/*
			 * Make sure fault buffer is disabled.
			 * This is to avoid accessing fault buffer by hw
			 * during the window BAR2 is being unmapped by s/w
			 */
			nvgpu_log_info(g, "fault status busy set, check again");
			fault_status = gk20a_readl(g, fb_mmu_fault_status_r());

			nvgpu_usleep_range(delay, delay * 2);
			delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
		} while (!nvgpu_timeout_expired_msg(&timeout,
				"fault status busy set"));
	}
}

void gv11b_fb_fault_buf_configure_hw(struct gk20a *g, unsigned int index)
{
	u32 addr_lo;
	u32 addr_hi;

	nvgpu_log_fn(g, " ");

	gv11b_fb_fault_buf_set_state_hw(g, index,
					 FAULT_BUF_DISABLED);
	addr_lo = u64_lo32(g->mm.hw_fault_buf[index].gpu_va >>
					ram_in_base_shift_v());
	addr_hi = u64_hi32(g->mm.hw_fault_buf[index].gpu_va);

	gk20a_writel(g, fb_mmu_fault_buffer_lo_r(index),
			fb_mmu_fault_buffer_lo_addr_f(addr_lo));

	gk20a_writel(g, fb_mmu_fault_buffer_hi_r(index),
			fb_mmu_fault_buffer_hi_addr_f(addr_hi));

	gk20a_writel(g, fb_mmu_fault_buffer_size_r(index),
		fb_mmu_fault_buffer_size_val_f(g->ops.fifo.get_num_fifos(g)) |
		fb_mmu_fault_buffer_size_overflow_intr_enable_f());

	gv11b_fb_fault_buf_set_state_hw(g, index, FAULT_BUF_ENABLED);
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

	if (intr_type & HUB_INTR_TYPE_OTHER) {
		mask |=
		 fb_niso_intr_en_clr_mmu_other_fault_notify_m();
	}

	if (intr_type & HUB_INTR_TYPE_NONREPLAY) {
		mask |=
		fb_niso_intr_en_clr_mmu_nonreplayable_fault_notify_m() |
		fb_niso_intr_en_clr_mmu_nonreplayable_fault_overflow_m();
	}

	if (intr_type & HUB_INTR_TYPE_REPLAY) {
		mask |=
		 fb_niso_intr_en_clr_mmu_replayable_fault_notify_m() |
		 fb_niso_intr_en_clr_mmu_replayable_fault_overflow_m();
	}

	if (intr_type & HUB_INTR_TYPE_ECC_UNCORRECTED) {
		mask |=
		 fb_niso_intr_en_clr_mmu_ecc_uncorrected_error_notify_m();
	}

	if (intr_type & HUB_INTR_TYPE_ACCESS_COUNTER) {
		mask |=
		 fb_niso_intr_en_clr_hub_access_counter_notify_m() |
		 fb_niso_intr_en_clr_hub_access_counter_error_m();
	}

	return mask;
}

static u32 gv11b_fb_get_hub_intr_en_mask(struct gk20a *g,
			 unsigned int intr_type)
{
	u32 mask = 0;

	if (intr_type & HUB_INTR_TYPE_OTHER) {
		mask |=
		 fb_niso_intr_en_set_mmu_other_fault_notify_m();
	}

	if (intr_type & HUB_INTR_TYPE_NONREPLAY) {
		mask |=
		fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_m() |
		fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_m();
	}

	if (intr_type & HUB_INTR_TYPE_REPLAY) {
		mask |=
		fb_niso_intr_en_set_mmu_replayable_fault_notify_m() |
		fb_niso_intr_en_set_mmu_replayable_fault_overflow_m();
	}

	if (intr_type & HUB_INTR_TYPE_ECC_UNCORRECTED) {
		mask |=
		 fb_niso_intr_en_set_mmu_ecc_uncorrected_error_notify_m();
	}

	if (intr_type & HUB_INTR_TYPE_ACCESS_COUNTER) {
		mask |=
		 fb_niso_intr_en_set_hub_access_counter_notify_m() |
		 fb_niso_intr_en_set_hub_access_counter_error_m();
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

static void gv11b_fb_parse_mmfault(struct mmu_fault_info *mmfault)
{
	if (WARN_ON(mmfault->fault_type >=
				ARRAY_SIZE(fault_type_descs_gv11b)))
		mmfault->fault_type_desc =  invalid_str;
	else
		mmfault->fault_type_desc =
			 fault_type_descs_gv11b[mmfault->fault_type];

	if (WARN_ON(mmfault->client_type >=
			ARRAY_SIZE(fault_client_type_descs_gv11b)))
		mmfault->client_type_desc = invalid_str;
	else
		mmfault->client_type_desc =
			 fault_client_type_descs_gv11b[mmfault->client_type];

	mmfault->client_id_desc = invalid_str;
	if (mmfault->client_type ==
			gmmu_fault_client_type_hub_v()) {

		if (!(WARN_ON(mmfault->client_id >=
				 ARRAY_SIZE(hub_client_descs_gv11b))))
			mmfault->client_id_desc =
				 hub_client_descs_gv11b[mmfault->client_id];
	} else if (mmfault->client_type ==
			gmmu_fault_client_type_gpc_v()) {
		if (!(WARN_ON(mmfault->client_id >=
				 ARRAY_SIZE(gpc_client_descs_gv11b))))
			mmfault->client_id_desc =
				 gpc_client_descs_gv11b[mmfault->client_id];
	}

}

static void gv11b_fb_print_fault_info(struct gk20a *g,
			 struct mmu_fault_info *mmfault)
{
	if (mmfault && mmfault->valid) {
		nvgpu_err(g, "[MMU FAULT] "
			"mmu engine id:  %d, "
			"ch id:  %d, "
			"fault addr: 0x%llx, "
			"fault addr aperture: %d, "
			"fault type: %s, "
			"access type: %s, ",
			mmfault->mmu_engine_id,
			mmfault->chid,
			mmfault->fault_addr,
			mmfault->fault_addr_aperture,
			mmfault->fault_type_desc,
			fault_access_type_descs_gv11b[mmfault->access_type]);
		nvgpu_log(g, gpu_dbg_intr, "[MMU FAULT] "
			"mmu engine id:  %d, "
			"faulted act eng id if any: 0x%x, "
			"faulted veid if any: 0x%x, "
			"faulted pbdma id if any: 0x%x, "
			"fault addr: 0x%llx, ",
			mmfault->mmu_engine_id,
			mmfault->faulted_engine,
			mmfault->faulted_subid,
			mmfault->faulted_pbdma,
			mmfault->fault_addr);
		nvgpu_log(g, gpu_dbg_intr, "[MMU FAULT] "
			"fault addr aperture: %d, "
			"fault type: %s, "
			"access type: %s, "
			"inst ptr: 0x%llx, "
			"inst ptr aperture: %d, ",
			mmfault->fault_addr_aperture,
			mmfault->fault_type_desc,
			fault_access_type_descs_gv11b[mmfault->access_type],
			mmfault->inst_ptr,
			mmfault->inst_aperture);
		nvgpu_log(g, gpu_dbg_intr, "[MMU FAULT] "
			"ch id:  %d, "
			"timestamp hi:lo 0x%08x:0x%08x, "
			"client type: %s, "
			"client id:  %s, "
			"gpc id if client type is gpc: %d, ",
			mmfault->chid,
			mmfault->timestamp_hi, mmfault->timestamp_lo,
			mmfault->client_type_desc,
			mmfault->client_id_desc,
			mmfault->gpc_id);
		nvgpu_log(g, gpu_dbg_intr, "[MMU FAULT] "
			"protected mode: %d, "
			"replayable fault: %d, "
			"replayable fault en:  %d ",
			mmfault->protected_mode,
			mmfault->replayable_fault,
			mmfault->replay_fault_en);
	}
}

/*
 *Fault buffer format
 *
 * 31    28     24 23           16 15            8 7     4       0
 *.-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-.
 *|              inst_lo                  |0 0|apr|0 0 0 0 0 0 0 0|
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|                             inst_hi                           |
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|              addr_31_12               |                   |AP |
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|                            addr_63_32                         |
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|                          timestamp_lo                         |
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|                          timestamp_hi                         |
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|                           (reserved)        |    engine_id    |
 *`-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-'
 *|V|R|P|  gpc_id |0 0 0|t|0|acctp|0|   client    |RF0 0|faulttype|
 */

static void gv11b_fb_copy_from_hw_fault_buf(struct gk20a *g,
	 struct nvgpu_mem *mem, u32 offset, struct mmu_fault_info *mmfault)
{
	u32 rd32_val;
	u32 addr_lo, addr_hi;
	u64 inst_ptr;
	u32 chid = FIFO_INVAL_CHANNEL_ID;
	struct channel_gk20a *refch;

	memset(mmfault, 0, sizeof(*mmfault));

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			 gmmu_fault_buf_entry_inst_lo_w());
	addr_lo = gmmu_fault_buf_entry_inst_lo_v(rd32_val);
	addr_lo = addr_lo << ram_in_base_shift_v();

	addr_hi = nvgpu_mem_rd32(g, mem, offset +
				 gmmu_fault_buf_entry_inst_hi_w());
	addr_hi = gmmu_fault_buf_entry_inst_hi_v(addr_hi);

	inst_ptr = hi32_lo32_to_u64(addr_hi, addr_lo);

	/* refch will be put back after fault is handled */
	refch = gk20a_refch_from_inst_ptr(g, inst_ptr);
	if (refch)
		chid = refch->chid;

	/* it is ok to continue even if refch is NULL */
	mmfault->refch = refch;
	mmfault->chid = chid;
	mmfault->inst_ptr = inst_ptr;
	mmfault->inst_aperture = gmmu_fault_buf_entry_inst_aperture_v(rd32_val);

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			 gmmu_fault_buf_entry_addr_lo_w());

	mmfault->fault_addr_aperture =
		gmmu_fault_buf_entry_addr_phys_aperture_v(rd32_val);
	addr_lo = gmmu_fault_buf_entry_addr_lo_v(rd32_val);
	addr_lo = addr_lo << ram_in_base_shift_v();

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			 gmmu_fault_buf_entry_addr_hi_w());
	addr_hi = gmmu_fault_buf_entry_addr_hi_v(rd32_val);
	mmfault->fault_addr = hi32_lo32_to_u64(addr_hi, addr_lo);

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			 gmmu_fault_buf_entry_timestamp_lo_w());
	mmfault->timestamp_lo =
		 gmmu_fault_buf_entry_timestamp_lo_v(rd32_val);

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			 gmmu_fault_buf_entry_timestamp_hi_w());
	mmfault->timestamp_hi =
		 gmmu_fault_buf_entry_timestamp_hi_v(rd32_val);

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			 gmmu_fault_buf_entry_engine_id_w());

	mmfault->mmu_engine_id =
		 gmmu_fault_buf_entry_engine_id_v(rd32_val);
	gv11b_mmu_fault_id_to_eng_pbdma_id_and_veid(g, mmfault->mmu_engine_id,
		 &mmfault->faulted_engine, &mmfault->faulted_subid,
		 &mmfault->faulted_pbdma);

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			gmmu_fault_buf_entry_fault_type_w());
	mmfault->client_id =
		 gmmu_fault_buf_entry_client_v(rd32_val);
	mmfault->replayable_fault =
		gmmu_fault_buf_entry_replayable_fault_v(rd32_val);

	mmfault->fault_type =
		 gmmu_fault_buf_entry_fault_type_v(rd32_val);
	mmfault->access_type =
		 gmmu_fault_buf_entry_access_type_v(rd32_val);

	mmfault->client_type =
		gmmu_fault_buf_entry_mmu_client_type_v(rd32_val);

	mmfault->gpc_id =
		 gmmu_fault_buf_entry_gpc_id_v(rd32_val);
	mmfault->protected_mode =
		gmmu_fault_buf_entry_protected_mode_v(rd32_val);

	mmfault->replay_fault_en =
		gmmu_fault_buf_entry_replayable_fault_en_v(rd32_val);

	mmfault->valid = gmmu_fault_buf_entry_valid_v(rd32_val);

	rd32_val = nvgpu_mem_rd32(g, mem, offset +
			gmmu_fault_buf_entry_fault_type_w());
	rd32_val &= ~(gmmu_fault_buf_entry_valid_m());
	nvgpu_mem_wr32(g, mem, offset + gmmu_fault_buf_entry_valid_w(),
					 rd32_val);

	gv11b_fb_parse_mmfault(mmfault);
}

static void gv11b_fb_handle_mmu_fault_common(struct gk20a *g,
		 struct mmu_fault_info *mmfault, u32 *invalidate_replay_val)
{
	unsigned int id_type;
	u32 num_lce, act_eng_bitmask = 0;
	int err = 0;

	if (!mmfault->valid)
		return;

	gv11b_fb_print_fault_info(g, mmfault);

	num_lce = gv11b_ce_get_num_lce(g);
	if ((mmfault->mmu_engine_id >=
			gmmu_fault_mmu_eng_id_ce0_v()) &&
			(mmfault->mmu_engine_id <
			gmmu_fault_mmu_eng_id_ce0_v() + num_lce)) {
		/* CE page faults are not reported as replayable */
		nvgpu_log(g, gpu_dbg_intr, "CE Faulted");
		err = gv11b_fb_fix_page_fault(g, mmfault);
		gv11b_fifo_reset_pbdma_and_eng_faulted(g, mmfault->refch,
			mmfault->faulted_pbdma, mmfault->faulted_engine);
		if (!err) {
			nvgpu_log(g, gpu_dbg_intr, "CE Page Fault Fixed");
			*invalidate_replay_val = 0;
			/* refch in mmfault is assigned at the time of copying
			 * fault info from snap reg or bar2 fault buf
			 */
			gk20a_channel_put(mmfault->refch);
			return;
		}
		/* Do recovery. Channel recovery needs refch */
		nvgpu_log(g, gpu_dbg_intr, "CE Page Fault Not Fixed");
	}

	if (!mmfault->replayable_fault) {
		if (mmfault->fault_type ==
				gmmu_fault_type_unbound_inst_block_v()) {
		/*
		 * Bug 1847172: When an engine faults due to an unbound
		 * instance block, the fault cannot be isolated to a
		 * single context so we need to reset the entire runlist
		 */
		id_type = ID_TYPE_UNKNOWN;
		nvgpu_log(g, gpu_dbg_intr, "UNBOUND INST BLOCK MMU FAULT");

		} else if (mmfault->refch) {
			if (gk20a_is_channel_marked_as_tsg(mmfault->refch))
				id_type = ID_TYPE_TSG;
			else
				id_type = ID_TYPE_CHANNEL;
		} else {
			id_type = ID_TYPE_UNKNOWN;
		}
		if (mmfault->faulted_engine != FIFO_INVAL_ENGINE_ID)
			act_eng_bitmask = BIT(mmfault->faulted_engine);

		g->ops.fifo.teardown_ch_tsg(g, act_eng_bitmask,
			mmfault->chid, id_type, RC_TYPE_MMU_FAULT, mmfault);
	} else {
		err = gv11b_fb_fix_page_fault(g, mmfault);
		if (err) {
			*invalidate_replay_val |=
				fb_mmu_invalidate_replay_cancel_global_f();
		} else {
			*invalidate_replay_val |=
				fb_mmu_invalidate_replay_start_ack_all_f();
		}
		/* refch in mmfault is assigned at the time of copying
		 * fault info from snap reg or bar2 fault buf
		 */
		gk20a_channel_put(mmfault->refch);
	}
}

static void gv11b_fb_replay_or_cancel_faults(struct gk20a *g,
			 u32 invalidate_replay_val)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (invalidate_replay_val &
			 fb_mmu_invalidate_replay_cancel_global_f()) {
		/*
		 * cancel faults so that next time it faults as
		 * replayable faults and channel recovery can be done
		 */
		err = gv11b_fb_mmu_invalidate_replay(g,
			fb_mmu_invalidate_replay_cancel_global_f());
	} else if (invalidate_replay_val &
			 fb_mmu_invalidate_replay_start_ack_all_f()) {
		/* pte valid is fixed. replay faulting request */
		err = gv11b_fb_mmu_invalidate_replay(g,
			fb_mmu_invalidate_replay_start_ack_all_f());
	}
}

static void gv11b_fb_handle_mmu_nonreplay_replay_fault(struct gk20a *g,
		 u32 fault_status, unsigned int index)
{
	u32 get_indx, offset, rd32_val, entries;
	struct nvgpu_mem *mem;
	struct mmu_fault_info *mmfault;
	u32 invalidate_replay_val = 0;
	u64 prev_fault_addr =  0ULL;
	u64 next_fault_addr =  0ULL;

	if (gv11b_fb_is_fault_buffer_empty(g, index, &get_indx)) {
		nvgpu_log(g, gpu_dbg_intr,
			"SPURIOUS mmu fault: reg index:%d", index);
		return;
	}
	nvgpu_info(g, "%s MMU FAULT" ,
			index == REPLAY_REG_INDEX ? "REPLAY" : "NON-REPLAY");

	nvgpu_log(g, gpu_dbg_intr, "get ptr = %d", get_indx);

	mem = &g->mm.hw_fault_buf[index];
	mmfault = g->mm.fault_info[index];

	entries = gv11b_fb_fault_buffer_size_val(g, index);
	nvgpu_log(g, gpu_dbg_intr, "buffer num entries = %d", entries);

	offset = (get_indx * gmmu_fault_buf_size_v()) / sizeof(u32);
	nvgpu_log(g, gpu_dbg_intr, "starting word offset = 0x%x", offset);

	rd32_val = nvgpu_mem_rd32(g, mem,
		 offset + gmmu_fault_buf_entry_valid_w());
	nvgpu_log(g, gpu_dbg_intr, "entry valid offset val = 0x%x", rd32_val);

	while ((rd32_val & gmmu_fault_buf_entry_valid_m())) {

		nvgpu_log(g, gpu_dbg_intr, "entry valid = 0x%x", rd32_val);

		gv11b_fb_copy_from_hw_fault_buf(g, mem, offset, mmfault);

		get_indx = (get_indx + 1) % entries;
		nvgpu_log(g, gpu_dbg_intr, "new get index = %d", get_indx);

		gv11b_fb_fault_buffer_get_ptr_update(g, index, get_indx);

		offset = (get_indx * gmmu_fault_buf_size_v()) / sizeof(u32);
		nvgpu_log(g, gpu_dbg_intr, "next word offset = 0x%x", offset);

		rd32_val = nvgpu_mem_rd32(g, mem,
			 offset + gmmu_fault_buf_entry_valid_w());

		if (index == REPLAY_REG_INDEX && mmfault->fault_addr != 0ULL) {
			/* fault_addr "0" is not supposed to be fixed ever.
			 * For the first time when prev = 0, next = 0 and
			 * fault addr is also 0 then handle_mmu_fault_common will
			 * not be called. Fix by checking fault_addr not equal to 0
			 */
			prev_fault_addr = next_fault_addr;
			next_fault_addr = mmfault->fault_addr;
			if (prev_fault_addr == next_fault_addr) {
				nvgpu_log(g, gpu_dbg_intr, "pte is fixed");
				if (mmfault->refch)
					gk20a_channel_put(mmfault->refch);
				/* pte already fixed for this addr */
				continue;
			}
		}

		gv11b_fb_handle_mmu_fault_common(g, mmfault,
				 &invalidate_replay_val);

	}
	if (index == REPLAY_REG_INDEX && invalidate_replay_val)
		gv11b_fb_replay_or_cancel_faults(g, invalidate_replay_val);
}

static void gv11b_mm_copy_from_fault_snap_reg(struct gk20a *g,
		u32 fault_status, struct mmu_fault_info *mmfault)
{
	u32 reg_val;
	u32 addr_lo, addr_hi;
	u64 inst_ptr;
	int chid = FIFO_INVAL_CHANNEL_ID;
	struct channel_gk20a *refch;

	memset(mmfault, 0, sizeof(*mmfault));

	if (!(fault_status & fb_mmu_fault_status_valid_set_f())) {

		nvgpu_log(g, gpu_dbg_intr, "mmu fault status valid not set");
		return;
	}

	reg_val = gk20a_readl(g, fb_mmu_fault_inst_lo_r());
	addr_lo = fb_mmu_fault_inst_lo_addr_v(reg_val);
	addr_lo = addr_lo << ram_in_base_shift_v();

	addr_hi = gk20a_readl(g, fb_mmu_fault_inst_hi_r());
	addr_hi = fb_mmu_fault_inst_hi_addr_v(addr_hi);
	inst_ptr = hi32_lo32_to_u64(addr_hi, addr_lo);

	/* refch will be put back after fault is handled */
	refch = gk20a_refch_from_inst_ptr(g, inst_ptr);
	if (refch)
		chid = refch->chid;

	/* It is still ok to continue if refch is NULL */
	mmfault->refch = refch;
	mmfault->chid = chid;
	mmfault->inst_ptr = inst_ptr;
	mmfault->inst_aperture = fb_mmu_fault_inst_lo_aperture_v(reg_val);
	mmfault->mmu_engine_id = fb_mmu_fault_inst_lo_engine_id_v(reg_val);

	gv11b_mmu_fault_id_to_eng_pbdma_id_and_veid(g, mmfault->mmu_engine_id,
		 &mmfault->faulted_engine, &mmfault->faulted_subid,
		 &mmfault->faulted_pbdma);

	reg_val = gk20a_readl(g, fb_mmu_fault_addr_lo_r());
	addr_lo = fb_mmu_fault_addr_lo_addr_v(reg_val);
	addr_lo = addr_lo << ram_in_base_shift_v();

	mmfault->fault_addr_aperture =
			 fb_mmu_fault_addr_lo_phys_aperture_v(reg_val);

	addr_hi = gk20a_readl(g, fb_mmu_fault_addr_hi_r());
	addr_hi = fb_mmu_fault_addr_hi_addr_v(addr_hi);
	mmfault->fault_addr = hi32_lo32_to_u64(addr_hi, addr_lo);

	reg_val = gk20a_readl(g, fb_mmu_fault_info_r());
	mmfault->fault_type = fb_mmu_fault_info_fault_type_v(reg_val);
	mmfault->replayable_fault =
			 fb_mmu_fault_info_replayable_fault_v(reg_val);
	mmfault->client_id = fb_mmu_fault_info_client_v(reg_val);
	mmfault->access_type = fb_mmu_fault_info_access_type_v(reg_val);
	mmfault->client_type = fb_mmu_fault_info_client_type_v(reg_val);
	mmfault->gpc_id = fb_mmu_fault_info_gpc_id_v(reg_val);
	mmfault->protected_mode =
			 fb_mmu_fault_info_protected_mode_v(reg_val);
	mmfault->replay_fault_en =
			fb_mmu_fault_info_replayable_fault_en_v(reg_val);

	mmfault->valid = fb_mmu_fault_info_valid_v(reg_val);

	fault_status &= ~(fb_mmu_fault_status_valid_m());
	gk20a_writel(g, fb_mmu_fault_status_r(), fault_status);

	gv11b_fb_parse_mmfault(mmfault);

}

static void gv11b_fb_handle_replay_fault_overflow(struct gk20a *g,
			 u32 fault_status)
{
	u32 reg_val;
	unsigned int index = REPLAY_REG_INDEX;

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_get_r(index));

	if (fault_status &
		 fb_mmu_fault_status_replayable_getptr_corrupted_m()) {

		nvgpu_err(g, "replayable getptr corrupted set");

		gv11b_fb_fault_buf_configure_hw(g, index);

		reg_val = set_field(reg_val,
			fb_mmu_fault_buffer_get_getptr_corrupted_m(),
			fb_mmu_fault_buffer_get_getptr_corrupted_clear_f());
	}

	if (fault_status &
		 fb_mmu_fault_status_replayable_overflow_m()) {
		bool buffer_full = gv11b_fb_is_fault_buffer_full(g, index);

		nvgpu_err(g, "replayable overflow: buffer full:%s",
				buffer_full?"true":"false");

		reg_val = set_field(reg_val,
			fb_mmu_fault_buffer_get_overflow_m(),
			fb_mmu_fault_buffer_get_overflow_clear_f());
	}

	gk20a_writel(g, fb_mmu_fault_buffer_get_r(index), reg_val);
}

static void gv11b_fb_handle_nonreplay_fault_overflow(struct gk20a *g,
			 u32 fault_status)
{
	u32 reg_val;
	unsigned int index = NONREPLAY_REG_INDEX;

	reg_val = gk20a_readl(g, fb_mmu_fault_buffer_get_r(index));

	if (fault_status &
		 fb_mmu_fault_status_non_replayable_getptr_corrupted_m()) {

		nvgpu_err(g, "non replayable getptr corrupted set");

		gv11b_fb_fault_buf_configure_hw(g, index);

		reg_val = set_field(reg_val,
			fb_mmu_fault_buffer_get_getptr_corrupted_m(),
			fb_mmu_fault_buffer_get_getptr_corrupted_clear_f());
	}

	if (fault_status &
		 fb_mmu_fault_status_non_replayable_overflow_m()) {

		bool buffer_full = gv11b_fb_is_fault_buffer_full(g, index);

		nvgpu_err(g, "non replayable overflow: buffer full:%s",
				buffer_full?"true":"false");

		reg_val = set_field(reg_val,
			fb_mmu_fault_buffer_get_overflow_m(),
			fb_mmu_fault_buffer_get_overflow_clear_f());
	}

	gk20a_writel(g, fb_mmu_fault_buffer_get_r(index), reg_val);
}

static void gv11b_fb_handle_bar2_fault(struct gk20a *g,
			struct mmu_fault_info *mmfault, u32 fault_status)
{
	gv11b_fb_disable_hub_intr(g, STALL_REG_INDEX,
		HUB_INTR_TYPE_NONREPLAY | HUB_INTR_TYPE_REPLAY);


	if (fault_status & fb_mmu_fault_status_non_replayable_error_m()) {
		if (gv11b_fb_is_fault_buf_enabled(g, NONREPLAY_REG_INDEX))
			gv11b_fb_fault_buf_configure_hw(g, NONREPLAY_REG_INDEX);
	}

	if (fault_status & fb_mmu_fault_status_replayable_error_m()) {
		if (gv11b_fb_is_fault_buf_enabled(g, REPLAY_REG_INDEX))
			gv11b_fb_fault_buf_configure_hw(g, REPLAY_REG_INDEX);
	}
	gv11b_ce_mthd_buffer_fault_in_bar2_fault(g);

	g->ops.mm.init_bar2_mm_hw_setup(g);

	if (mmfault->refch) {
		gk20a_channel_put(mmfault->refch);
		mmfault->refch = NULL;
	}
	gv11b_fb_enable_hub_intr(g, STALL_REG_INDEX,
		HUB_INTR_TYPE_NONREPLAY | HUB_INTR_TYPE_REPLAY);
}

static void gv11b_fb_handle_other_fault_notify(struct gk20a *g,
			 u32 fault_status)
{
	struct mmu_fault_info *mmfault;
	u32 invalidate_replay_val = 0;

	mmfault = g->mm.fault_info[FAULT_TYPE_OTHER_AND_NONREPLAY];

	gv11b_mm_copy_from_fault_snap_reg(g, fault_status, mmfault);

	/* BAR2/Physical faults will not be snapped in hw fault buf */
	if (mmfault->mmu_engine_id == gmmu_fault_mmu_eng_id_bar2_v()) {
		nvgpu_err(g, "BAR2 MMU FAULT");
		gv11b_fb_handle_bar2_fault(g, mmfault, fault_status);

	} else if (mmfault->mmu_engine_id ==
			gmmu_fault_mmu_eng_id_physical_v()) {
		/* usually means VPR or out of bounds physical accesses */
		nvgpu_err(g, "PHYSICAL MMU FAULT");

	} else {
		gv11b_fb_handle_mmu_fault_common(g, mmfault,
				 &invalidate_replay_val);

		if (invalidate_replay_val)
			gv11b_fb_replay_or_cancel_faults(g,
					invalidate_replay_val);
	}
}

static void gv11b_fb_handle_dropped_mmu_fault(struct gk20a *g, u32 fault_status)
{
	u32 dropped_faults = 0;

	dropped_faults = fb_mmu_fault_status_dropped_bar1_phys_set_f() |
			fb_mmu_fault_status_dropped_bar1_virt_set_f() |
			fb_mmu_fault_status_dropped_bar2_phys_set_f() |
			fb_mmu_fault_status_dropped_bar2_virt_set_f() |
			fb_mmu_fault_status_dropped_ifb_phys_set_f() |
			fb_mmu_fault_status_dropped_ifb_virt_set_f() |
			fb_mmu_fault_status_dropped_other_phys_set_f()|
			fb_mmu_fault_status_dropped_other_virt_set_f();

	if (fault_status & dropped_faults) {
		nvgpu_err(g, "dropped mmu fault (0x%08x)",
				 fault_status & dropped_faults);
		gk20a_writel(g, fb_mmu_fault_status_r(), dropped_faults);
	}
}


static void gv11b_fb_handle_mmu_fault(struct gk20a *g, u32 niso_intr)
{
	u32 fault_status = gk20a_readl(g, fb_mmu_fault_status_r());

	nvgpu_log(g, gpu_dbg_intr, "mmu_fault_status = 0x%08x", fault_status);

	if (niso_intr &
		 fb_niso_intr_mmu_other_fault_notify_m()) {

		gv11b_fb_handle_dropped_mmu_fault(g, fault_status);

		gv11b_fb_handle_other_fault_notify(g, fault_status);
	}

	if (gv11b_fb_is_fault_buf_enabled(g, NONREPLAY_REG_INDEX)) {

		if (niso_intr &
		 fb_niso_intr_mmu_nonreplayable_fault_notify_m()) {

			gv11b_fb_handle_mmu_nonreplay_replay_fault(g,
					fault_status, NONREPLAY_REG_INDEX);

			/*
			 * When all the faults are processed,
			 * GET and PUT will have same value and mmu fault status
			 * bit will be reset by HW
			 */
		}
		if (niso_intr &
		 fb_niso_intr_mmu_nonreplayable_fault_overflow_m()) {

			gv11b_fb_handle_nonreplay_fault_overflow(g,
				 fault_status);
		}

	}

	if (gv11b_fb_is_fault_buf_enabled(g, REPLAY_REG_INDEX)) {

		if (niso_intr &
		 fb_niso_intr_mmu_replayable_fault_notify_m()) {

			gv11b_fb_handle_mmu_nonreplay_replay_fault(g,
					fault_status, REPLAY_REG_INDEX);
		}
		if (niso_intr &
		 fb_niso_intr_mmu_replayable_fault_overflow_m()) {

			gv11b_fb_handle_replay_fault_overflow(g,
				 fault_status);
		}

	}

	nvgpu_log(g, gpu_dbg_intr, "clear mmu fault status");
	gk20a_writel(g, fb_mmu_fault_status_r(),
				fb_mmu_fault_status_valid_clear_f());
}

static void gv11b_fb_hub_isr(struct gk20a *g)
{
	u32 status, niso_intr;

	nvgpu_mutex_acquire(&g->mm.hub_isr_mutex);

	niso_intr = gk20a_readl(g, fb_niso_intr_r());

	nvgpu_info(g, "enter hub isr, niso_intr = 0x%08x", niso_intr);

	if (niso_intr &
		 (fb_niso_intr_hub_access_counter_notify_m() |
		  fb_niso_intr_hub_access_counter_error_m())) {

		nvgpu_info(g, "hub access counter notify/error");
	}
	if (niso_intr &
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

	}
	if (niso_intr &
		(fb_niso_intr_mmu_other_fault_notify_m() |
		fb_niso_intr_mmu_replayable_fault_notify_m() |
		fb_niso_intr_mmu_replayable_fault_overflow_m() |
		fb_niso_intr_mmu_nonreplayable_fault_notify_m() |
		fb_niso_intr_mmu_nonreplayable_fault_overflow_m())) {

		nvgpu_info(g, "MMU Fault");
		gv11b_fb_handle_mmu_fault(g, niso_intr);
	}

	nvgpu_mutex_release(&g->mm.hub_isr_mutex);
}

bool gv11b_fb_mmu_fault_pending(struct gk20a *g)
{
	if (gk20a_readl(g, fb_niso_intr_r()) &
		(fb_niso_intr_mmu_other_fault_notify_m() |
		 fb_niso_intr_mmu_ecc_uncorrected_error_notify_m() |
		 fb_niso_intr_mmu_replayable_fault_notify_m() |
		 fb_niso_intr_mmu_replayable_fault_overflow_m() |
		 fb_niso_intr_mmu_nonreplayable_fault_notify_m() |
		 fb_niso_intr_mmu_nonreplayable_fault_overflow_m()))
		return true;

	return false;
}

static int gv11b_fb_mmu_invalidate_replay(struct gk20a *g,
			 u32 invalidate_replay_val)
{
	int err = -ETIMEDOUT;
	u32 reg_val;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");

	nvgpu_mutex_acquire(&g->mm.tlb_lock);

	reg_val = gk20a_readl(g, fb_mmu_invalidate_r());

	reg_val |= fb_mmu_invalidate_all_va_true_f() |
		fb_mmu_invalidate_all_pdb_true_f() |
		invalidate_replay_val |
		fb_mmu_invalidate_trigger_true_f();

	gk20a_writel(g, fb_mmu_invalidate_r(), reg_val);

	/* retry 200 times */
	nvgpu_timeout_init(g, &timeout, 200, NVGPU_TIMER_RETRY_TIMER);
	do {
		reg_val = gk20a_readl(g, fb_mmu_ctrl_r());
		if (fb_mmu_ctrl_pri_fifo_empty_v(reg_val) !=
			fb_mmu_ctrl_pri_fifo_empty_false_f()) {
			err = 0;
			break;
		}
		nvgpu_udelay(5);
	} while (!nvgpu_timeout_expired_msg(&timeout,
			    "invalidate replay failed on 0x%llx"));
	if (err)
		nvgpu_err(g, "invalidate replay timedout");

	nvgpu_mutex_release(&g->mm.tlb_lock);

	return err;
}

static int gv11b_fb_fix_page_fault(struct gk20a *g,
			 struct mmu_fault_info *mmfault)
{
	int err = 0;
	u32 pte[2];

	if (mmfault->refch == NULL) {
		nvgpu_log(g, gpu_dbg_intr, "refch from mmu_fault_info is NULL");
		return -EINVAL;
	}

	err = __nvgpu_get_pte(g,
			mmfault->refch->vm, mmfault->fault_addr, &pte[0]);
	if (err) {
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_pte, "pte not found");
		return err;
	}
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_pte,
			"pte: %#08x %#08x", pte[1], pte[0]);

	pte[0] |= gmmu_new_pte_valid_true_f();
	if (pte[0] & gmmu_new_pte_read_only_true_f())
		pte[0] &= ~(gmmu_new_pte_read_only_true_f());
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_pte,
			"new pte: %#08x %#08x", pte[1], pte[0]);

	err = __nvgpu_set_pte(g,
			mmfault->refch->vm, mmfault->fault_addr, &pte[0]);
	if (err) {
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_pte, "pte not fixed");
		return err;
	}
	/* invalidate tlb so that GMMU does not use old cached translation */
	g->ops.fb.tlb_invalidate(g, mmfault->refch->vm->pdb.mem);

	err = __nvgpu_get_pte(g,
			mmfault->refch->vm, mmfault->fault_addr, &pte[0]);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_pte,
			"pte after tlb invalidate: %#08x %#08x",
			pte[1], pte[0]);
	return err;
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
