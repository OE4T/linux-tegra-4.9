/*
 * GV11b GPU GR
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

#include <linux/delay.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/tegra_gpu_t19x.h>

#include <soc/tegra/fuse.h>

#include <nvgpu/timers.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/debug.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gk20a/gr_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "gk20a/gr_pri_gk20a.h"

#include "gm20b/gr_gm20b.h"

#include "gp10b/gr_gp10b.h"

#include "gv11b/gr_gv11b.h"
#include "gv11b/mm_gv11b.h"
#include "gv11b/subctx_gv11b.h"

#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_proj_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_therm_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fb_gv11b.h>

bool gr_gv11b_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case VOLTA_COMPUTE_A:
	case VOLTA_A:
	case VOLTA_DMA_COPY_A:
		valid = true;
		break;

	case MAXWELL_COMPUTE_B:
	case MAXWELL_B:
	case FERMI_TWOD_A:
	case KEPLER_DMA_COPY_A:
	case MAXWELL_DMA_COPY_A:
	case PASCAL_COMPUTE_A:
	case PASCAL_A:
	case PASCAL_DMA_COPY_A:
		valid = true;
		break;

	default:
		break;
	}
	gk20a_dbg_info("class=0x%x valid=%d", class_num, valid);
	return valid;
}

bool gr_gv11b_is_valid_gfx_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case VOLTA_A:
	case PASCAL_A:
	case MAXWELL_B:
		valid = true;
		break;

	default:
		break;
	}
	return valid;
}

bool gr_gv11b_is_valid_compute_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case VOLTA_COMPUTE_A:
	case PASCAL_COMPUTE_A:
	case MAXWELL_COMPUTE_B:
		valid = true;
		break;

	default:
		break;
	}
	return valid;
}

static u32 gv11b_gr_sm_offset(struct gk20a *g, u32 sm)
{

	u32 sm_pri_stride = nvgpu_get_litter_value(g, GPU_LIT_SM_PRI_STRIDE);
	u32 sm_offset = sm_pri_stride * sm;

	return sm_offset;
}

static int gr_gv11b_handle_l1_tag_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 l1_tag_ecc_status, l1_tag_ecc_corrected_err_status = 0;
	u32 l1_tag_ecc_uncorrected_err_status = 0;
	u32 l1_tag_corrected_err_count_delta = 0;
	u32 l1_tag_uncorrected_err_count_delta = 0;
	bool is_l1_tag_ecc_corrected_total_err_overflow = 0;
	bool is_l1_tag_ecc_uncorrected_total_err_overflow = 0;

	/* Check for L1 tag ECC errors. */
	l1_tag_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_r() + offset);
	l1_tag_ecc_corrected_err_status = l1_tag_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_el1_1_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_pixrpf_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_miss_fifo_m());
	l1_tag_ecc_uncorrected_err_status = l1_tag_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_el1_1_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_pixrpf_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_miss_fifo_m());

	if ((l1_tag_ecc_corrected_err_status == 0) && (l1_tag_ecc_uncorrected_err_status == 0))
		return 0;

	l1_tag_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_r() +
				offset));
	l1_tag_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_r() +
				offset));
	is_l1_tag_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_total_counter_overflow_v(l1_tag_ecc_status);
	is_l1_tag_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_total_counter_overflow_v(l1_tag_ecc_status);

	if ((l1_tag_corrected_err_count_delta > 0) || is_l1_tag_ecc_corrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM L1 tag! err_mask [%08x] is_overf [%d]",
			l1_tag_ecc_corrected_err_status, is_l1_tag_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		l1_tag_corrected_err_count_delta +=
			(is_l1_tag_ecc_corrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_total_s());
		g->ecc.gr.t19x.sm_l1_tag_corrected_err_count.counters[tpc] +=
							l1_tag_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((l1_tag_uncorrected_err_count_delta > 0) || is_l1_tag_ecc_uncorrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM L1 tag! err_mask [%08x] is_overf [%d]",
			l1_tag_ecc_uncorrected_err_status, is_l1_tag_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		l1_tag_uncorrected_err_count_delta +=
			(is_l1_tag_ecc_uncorrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_total_s());
		g->ecc.gr.t19x.sm_l1_tag_uncorrected_err_count.counters[tpc] +=
							l1_tag_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_reset_task_f());

	return 0;

}

static int gr_gv11b_handle_lrf_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 lrf_ecc_status, lrf_ecc_corrected_err_status = 0;
	u32 lrf_ecc_uncorrected_err_status = 0;
	u32 lrf_corrected_err_count_delta = 0;
	u32 lrf_uncorrected_err_count_delta = 0;
	bool is_lrf_ecc_corrected_total_err_overflow = 0;
	bool is_lrf_ecc_uncorrected_total_err_overflow = 0;

	/* Check for LRF ECC errors. */
	lrf_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_r() + offset);
	lrf_ecc_corrected_err_status = lrf_ecc_status &
		(gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp0_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp1_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp2_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp3_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp4_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp5_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp6_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp7_m());
	lrf_ecc_uncorrected_err_status = lrf_ecc_status &
		(gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp0_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp1_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp2_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp3_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp4_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp5_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp6_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp7_m());

	if ((lrf_ecc_corrected_err_status == 0) && (lrf_ecc_uncorrected_err_status == 0))
		return 0;

	lrf_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_r() +
				offset));
	lrf_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_r() +
				offset));
	is_lrf_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_total_counter_overflow_v(lrf_ecc_status);
	is_lrf_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_total_counter_overflow_v(lrf_ecc_status);

	if ((lrf_corrected_err_count_delta > 0) || is_lrf_ecc_corrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM LRF! err_mask [%08x] is_overf [%d]",
			lrf_ecc_corrected_err_status, is_lrf_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		lrf_corrected_err_count_delta +=
			(is_lrf_ecc_corrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_total_s());
		g->ecc.gr.t18x.sm_lrf_single_err_count.counters[tpc] +=
							lrf_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((lrf_uncorrected_err_count_delta > 0) || is_lrf_ecc_uncorrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM LRF! err_mask [%08x] is_overf [%d]",
			lrf_ecc_uncorrected_err_status, is_lrf_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		lrf_uncorrected_err_count_delta +=
			(is_lrf_ecc_uncorrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_total_s());
		g->ecc.gr.t18x.sm_lrf_double_err_count.counters[tpc] +=
							lrf_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_lrf_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_lrf_ecc_status_reset_task_f());

	return 0;

}

void gr_gv11b_enable_hww_exceptions(struct gk20a *g)
{
	/* enable exceptions */
	gk20a_writel(g, gr_fe_hww_esr_r(),
		     gr_fe_hww_esr_en_enable_f() |
		     gr_fe_hww_esr_reset_active_f());
	gk20a_writel(g, gr_memfmt_hww_esr_r(),
		     gr_memfmt_hww_esr_en_enable_f() |
		     gr_memfmt_hww_esr_reset_active_f());
}

void gr_gv11b_enable_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 reg_val;

	/*
	 * clear exceptions :
	 * other than SM : hww_esr are reset in *enable_hww_excetpions*
	 * SM            : cleared in *set_hww_esr_report_mask*
	 */

	/* enable exceptions */
	gk20a_writel(g, gr_exception2_en_r(), 0x0); /* BE not enabled */
	gk20a_writel(g, gr_exception1_en_r(), (1 << gr->gpc_count) - 1);

	reg_val = gr_exception_en_fe_enabled_f() |
			gr_exception_en_memfmt_enabled_f() |
			gr_exception_en_ds_enabled_f() |
			gr_exception_en_gpc_enabled_f();
	gk20a_writel(g, gr_exception_en_r(), reg_val);

}

static int gr_gv11b_handle_cbu_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 cbu_ecc_status, cbu_ecc_corrected_err_status = 0;
	u32 cbu_ecc_uncorrected_err_status = 0;
	u32 cbu_corrected_err_count_delta = 0;
	u32 cbu_uncorrected_err_count_delta = 0;
	bool is_cbu_ecc_corrected_total_err_overflow = 0;
	bool is_cbu_ecc_uncorrected_total_err_overflow = 0;

	/* Check for CBU ECC errors. */
	cbu_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_cbu_ecc_status_r() + offset);
	cbu_ecc_corrected_err_status = cbu_ecc_status &
		(gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_warp_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_warp_sm1_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_barrier_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_barrier_sm1_m());
	cbu_ecc_uncorrected_err_status = cbu_ecc_status &
		(gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_warp_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_warp_sm1_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_barrier_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_barrier_sm1_m());

	if ((cbu_ecc_corrected_err_status == 0) && (cbu_ecc_uncorrected_err_status == 0))
		return 0;

	cbu_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_r() +
				offset));
	cbu_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_r() +
				offset));
	is_cbu_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_total_counter_overflow_v(cbu_ecc_status);
	is_cbu_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_total_counter_overflow_v(cbu_ecc_status);

	if ((cbu_corrected_err_count_delta > 0) || is_cbu_ecc_corrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM CBU! err_mask [%08x] is_overf [%d]",
			cbu_ecc_corrected_err_status, is_cbu_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		cbu_corrected_err_count_delta +=
			(is_cbu_ecc_corrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_total_s());
		g->ecc.gr.t19x.sm_cbu_corrected_err_count.counters[tpc] +=
							cbu_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((cbu_uncorrected_err_count_delta > 0) || is_cbu_ecc_uncorrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM CBU! err_mask [%08x] is_overf [%d]",
			cbu_ecc_uncorrected_err_status, is_cbu_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		cbu_uncorrected_err_count_delta +=
			(is_cbu_ecc_uncorrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_total_s());
		g->ecc.gr.t19x.sm_cbu_uncorrected_err_count.counters[tpc] +=
							cbu_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_cbu_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_cbu_ecc_status_reset_task_f());

	return 0;

}

static int gr_gv11b_handle_l1_data_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 l1_data_ecc_status, l1_data_ecc_corrected_err_status = 0;
	u32 l1_data_ecc_uncorrected_err_status = 0;
	u32 l1_data_corrected_err_count_delta = 0;
	u32 l1_data_uncorrected_err_count_delta = 0;
	bool is_l1_data_ecc_corrected_total_err_overflow = 0;
	bool is_l1_data_ecc_uncorrected_total_err_overflow = 0;

	/* Check for L1 data ECC errors. */
	l1_data_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_r() + offset);
	l1_data_ecc_corrected_err_status = l1_data_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_corrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_corrected_err_el1_1_m());
	l1_data_ecc_uncorrected_err_status = l1_data_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_uncorrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_uncorrected_err_el1_1_m());

	if ((l1_data_ecc_corrected_err_status == 0) && (l1_data_ecc_uncorrected_err_status == 0))
		return 0;

	l1_data_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_r() +
				offset));
	l1_data_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_r() +
				offset));
	is_l1_data_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_corrected_err_total_counter_overflow_v(l1_data_ecc_status);
	is_l1_data_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_uncorrected_err_total_counter_overflow_v(l1_data_ecc_status);

	if ((l1_data_corrected_err_count_delta > 0) || is_l1_data_ecc_corrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM L1 data! err_mask [%08x] is_overf [%d]",
			l1_data_ecc_corrected_err_status, is_l1_data_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		l1_data_corrected_err_count_delta +=
			(is_l1_data_ecc_corrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_total_s());
		g->ecc.gr.t19x.sm_l1_data_corrected_err_count.counters[tpc] +=
							l1_data_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((l1_data_uncorrected_err_count_delta > 0) || is_l1_data_ecc_uncorrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM L1 data! err_mask [%08x] is_overf [%d]",
			l1_data_ecc_uncorrected_err_status, is_l1_data_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		l1_data_uncorrected_err_count_delta +=
			(is_l1_data_ecc_uncorrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_total_s());
		g->ecc.gr.t19x.sm_l1_data_uncorrected_err_count.counters[tpc] +=
							l1_data_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_reset_task_f());

	return 0;

}

static int gr_gv11b_handle_icache_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 icache_ecc_status, icache_ecc_corrected_err_status = 0;
	u32 icache_ecc_uncorrected_err_status = 0;
	u32 icache_corrected_err_count_delta = 0;
	u32 icache_uncorrected_err_count_delta = 0;
	bool is_icache_ecc_corrected_total_err_overflow = 0;
	bool is_icache_ecc_uncorrected_total_err_overflow = 0;

	/* Check for L0 && L1 icache ECC errors. */
	icache_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_icache_ecc_status_r() + offset);
	icache_ecc_corrected_err_status = icache_ecc_status &
		(gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l0_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l0_predecode_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l1_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l1_predecode_m());
	icache_ecc_uncorrected_err_status = icache_ecc_status &
		(gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l0_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l0_predecode_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l1_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l1_predecode_m());

	if ((icache_ecc_corrected_err_status == 0) && (icache_ecc_uncorrected_err_status == 0))
		return 0;

	icache_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_r() +
				offset));
	icache_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_r() +
				offset));
	is_icache_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_total_counter_overflow_v(icache_ecc_status);
	is_icache_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_total_counter_overflow_v(icache_ecc_status);

	if ((icache_corrected_err_count_delta > 0) || is_icache_ecc_corrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM L0 && L1 icache! err_mask [%08x] is_overf [%d]",
			icache_ecc_corrected_err_status, is_icache_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		icache_corrected_err_count_delta +=
			(is_icache_ecc_corrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_total_s());
		g->ecc.gr.t19x.sm_icache_corrected_err_count.counters[tpc] +=
							icache_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((icache_uncorrected_err_count_delta > 0) || is_icache_ecc_uncorrected_total_err_overflow) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM L0 && L1 icache! err_mask [%08x] is_overf [%d]",
			icache_ecc_uncorrected_err_status, is_icache_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		icache_uncorrected_err_count_delta +=
			(is_icache_ecc_uncorrected_total_err_overflow <<
			 gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_total_s());
		g->ecc.gr.t19x.sm_icache_uncorrected_err_count.counters[tpc] +=
							icache_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_icache_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_icache_ecc_status_reset_task_f());

	return 0;

}

int gr_gv11b_handle_tpc_sm_ecc_exception(struct gk20a *g,
		u32 gpc, u32 tpc,
		bool *post_event, struct channel_gk20a *fault_ch,
		u32 *hww_global_esr)
{
	int ret = 0;

	/* Check for L1 tag ECC errors. */
	gr_gv11b_handle_l1_tag_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for LRF ECC errors. */
	gr_gv11b_handle_lrf_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for CBU ECC errors. */
	gr_gv11b_handle_cbu_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for L1 data ECC errors. */
	gr_gv11b_handle_l1_data_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for L0 && L1 icache ECC errors. */
	gr_gv11b_handle_icache_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	return ret;
}

int gr_gv11b_handle_gcc_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 offset = gpc_stride * gpc;
	u32 gcc_l15_ecc_status, gcc_l15_ecc_corrected_err_status = 0;
	u32 gcc_l15_ecc_uncorrected_err_status = 0;
	u32 gcc_l15_corrected_err_count_delta = 0;
	u32 gcc_l15_uncorrected_err_count_delta = 0;
	bool is_gcc_l15_ecc_corrected_total_err_overflow = 0;
	bool is_gcc_l15_ecc_uncorrected_total_err_overflow = 0;

	/* Check for gcc l15 ECC errors. */
	gcc_l15_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_gcc_l15_ecc_status_r() + offset);
	gcc_l15_ecc_corrected_err_status = gcc_l15_ecc_status &
		(gr_pri_gpc0_gcc_l15_ecc_status_corrected_err_bank0_m() |
		 gr_pri_gpc0_gcc_l15_ecc_status_corrected_err_bank1_m());
	gcc_l15_ecc_uncorrected_err_status = gcc_l15_ecc_status &
		(gr_pri_gpc0_gcc_l15_ecc_status_uncorrected_err_bank0_m() |
		 gr_pri_gpc0_gcc_l15_ecc_status_uncorrected_err_bank1_m());

	if ((gcc_l15_ecc_corrected_err_status == 0) && (gcc_l15_ecc_uncorrected_err_status == 0))
		return 0;

	gcc_l15_corrected_err_count_delta =
		gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_r() +
				offset));
	gcc_l15_uncorrected_err_count_delta =
		gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_r() +
				offset));
	is_gcc_l15_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_gcc_l15_ecc_status_corrected_err_total_counter_overflow_v(gcc_l15_ecc_status);
	is_gcc_l15_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_gcc_l15_ecc_status_uncorrected_err_total_counter_overflow_v(gcc_l15_ecc_status);

	if ((gcc_l15_corrected_err_count_delta > 0) || is_gcc_l15_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in GCC L1.5! err_mask [%08x] is_overf [%d]",
			gcc_l15_ecc_corrected_err_status, is_gcc_l15_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		gcc_l15_corrected_err_count_delta +=
			(is_gcc_l15_ecc_corrected_total_err_overflow <<
			 gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_total_s());
		g->ecc.gr.t19x.gcc_l15_corrected_err_count.counters[gpc] +=
							gcc_l15_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((gcc_l15_uncorrected_err_count_delta > 0) || is_gcc_l15_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in GCC L1.5! err_mask [%08x] is_overf [%d]",
			gcc_l15_ecc_uncorrected_err_status, is_gcc_l15_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		gcc_l15_uncorrected_err_count_delta +=
			(is_gcc_l15_ecc_uncorrected_total_err_overflow <<
			 gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_total_s());
		g->ecc.gr.t19x.gcc_l15_uncorrected_err_count.counters[gpc] +=
							gcc_l15_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_gcc_l15_ecc_status_r() + offset,
			gr_pri_gpc0_gcc_l15_ecc_status_reset_task_f());

	return 0;
}

static int gr_gv11b_handle_gpcmmu_ecc_exception(struct gk20a *g, u32 gpc,
								u32 exception)
{
	int ret = 0;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 offset = gpc_stride * gpc;
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;
	int hww_esr;

	hww_esr = gk20a_readl(g, gr_gpc0_mmu_gpcmmu_global_esr_r() + offset);

	if (!(hww_esr & (gr_gpc0_mmu_gpcmmu_global_esr_ecc_corrected_m() |
		         gr_gpc0_mmu_gpcmmu_global_esr_ecc_uncorrected_m())))
		return ret;

	ecc_status = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_status_r() + offset);
	ecc_addr = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_address_r() + offset);
	corrected_cnt = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_r() + offset);
	uncorrected_cnt = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_r() + offset);

	corrected_delta = gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		gr_gpc0_mmu_l1tlb_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		gr_gpc0_mmu_l1tlb_ecc_status_uncorrected_err_total_counter_overflow_m();


	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow)
		gk20a_writel(g,
			gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_r() +
			offset, 0);
	if ((uncorrected_delta > 0) || uncorrected_overflow)
		gk20a_writel(g,
			gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_r() +
			offset, 0);

	gk20a_writel(g, gr_gpc0_mmu_l1tlb_ecc_status_r() + offset,
				gr_gpc0_mmu_l1tlb_ecc_status_reset_task_f());

	/* Handle overflow */
	if (corrected_overflow)
		corrected_delta += (0x1UL << gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_total_s());
	if (uncorrected_overflow)
		uncorrected_delta += (0x1UL << gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_total_s());


	g->ecc.gr.t19x.mmu_l1tlb_corrected_err_count.counters[gpc] +=
							corrected_delta;
	g->ecc.gr.t19x.mmu_l1tlb_uncorrected_err_count.counters[gpc] +=
							uncorrected_delta;
	nvgpu_log(g, gpu_dbg_intr,
			"mmu l1tlb gpc:%d ecc interrupt intr: 0x%x", gpc, hww_esr);

	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_corrected_err_l1tlb_sa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc sa data error");
	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_uncorrected_err_l1tlb_sa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc sa data error");
	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_corrected_err_l1tlb_fa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc fa data error");
	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_uncorrected_err_l1tlb_fa_data_m())
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc fa data error");
	if (corrected_overflow || uncorrected_overflow)
		nvgpu_info(g, "mmu l1tlb ecc counter overflow!");

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error address: 0x%x", ecc_addr);
	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.gr.t19x.mmu_l1tlb_corrected_err_count.counters[gpc],
		g->ecc.gr.t19x.mmu_l1tlb_uncorrected_err_count.counters[gpc]);

	return ret;
}

static int gr_gv11b_handle_gpccs_ecc_exception(struct gk20a *g, u32 gpc,
								u32 exception)
{
	int ret = 0;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 offset = gpc_stride * gpc;
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;
	int hww_esr;

	hww_esr = gk20a_readl(g, gr_gpc0_gpccs_hww_esr_r() + offset);

	if (!(hww_esr & (gr_gpc0_gpccs_hww_esr_ecc_uncorrected_m() |
		         gr_gpc0_gpccs_hww_esr_ecc_corrected_m())))
		return ret;

	ecc_status = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_status_r() + offset);
	ecc_addr = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_address_r() + offset);
	corrected_cnt = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_corrected_err_count_r() + offset);
	uncorrected_cnt = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_uncorrected_err_count_r() + offset);

	corrected_delta = gr_gpc0_gpccs_falcon_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = gr_gpc0_gpccs_falcon_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_uncorrected_err_total_counter_overflow_m();


	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow)
		gk20a_writel(g,
			gr_gpc0_gpccs_falcon_ecc_corrected_err_count_r() +
			offset, 0);
	if ((uncorrected_delta > 0) || uncorrected_overflow)
		gk20a_writel(g,
			gr_gpc0_gpccs_falcon_ecc_uncorrected_err_count_r() +
			offset, 0);

	gk20a_writel(g, gr_gpc0_gpccs_falcon_ecc_status_r() + offset,
				gr_gpc0_gpccs_falcon_ecc_status_reset_task_f());

	g->ecc.gr.t19x.gpccs_corrected_err_count.counters[gpc] +=
							corrected_delta;
	g->ecc.gr.t19x.gpccs_uncorrected_err_count.counters[gpc] +=
							uncorrected_delta;
	nvgpu_log(g, gpu_dbg_intr,
			"gppcs gpc:%d ecc interrupt intr: 0x%x", gpc, hww_esr);

	if (ecc_status & gr_gpc0_gpccs_falcon_ecc_status_corrected_err_imem_m())
		nvgpu_log(g, gpu_dbg_intr, "imem ecc error corrected");
	if (ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_uncorrected_err_imem_m())
		nvgpu_log(g, gpu_dbg_intr, "imem ecc error uncorrected");
	if (ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_corrected_err_dmem_m())
		nvgpu_log(g, gpu_dbg_intr, "dmem ecc error corrected");
	if (ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_uncorrected_err_dmem_m())
		nvgpu_log(g, gpu_dbg_intr, "dmem ecc error uncorrected");
	if (corrected_overflow || uncorrected_overflow)
		nvgpu_info(g, "gpccs ecc counter overflow!");

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error row address: 0x%x",
		gr_gpc0_gpccs_falcon_ecc_address_row_address_v(ecc_addr));

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.gr.t19x.gpccs_corrected_err_count.counters[gpc],
		g->ecc.gr.t19x.gpccs_uncorrected_err_count.counters[gpc]);

	return ret;
}

int gr_gv11b_handle_gpc_gpcmmu_exception(struct gk20a *g, u32 gpc,
							u32 gpc_exception)
{
	if (gpc_exception & gr_gpc0_gpccs_gpc_exception_gpcmmu_m())
		return gr_gv11b_handle_gpcmmu_ecc_exception(g, gpc,
								gpc_exception);
	return 0;
}

int gr_gv11b_handle_gpc_gpccs_exception(struct gk20a *g, u32 gpc,
							u32 gpc_exception)
{
	if (gpc_exception & gr_gpc0_gpccs_gpc_exception_gpccs_m())
		return gr_gv11b_handle_gpccs_ecc_exception(g, gpc,
								gpc_exception);

	return 0;
}

void gr_gv11b_enable_gpc_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_mask;

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(),
			gr_gpcs_tpcs_tpccs_tpc_exception_en_sm_enabled_f() |
			gr_gpcs_tpcs_tpccs_tpc_exception_en_mpc_enabled_f());

	tpc_mask =
		gr_gpcs_gpccs_gpc_exception_en_tpc_f((1 << gr->tpc_count) - 1);

	gk20a_writel(g, gr_gpcs_gpccs_gpc_exception_en_r(),
		(tpc_mask | gr_gpcs_gpccs_gpc_exception_en_gcc_f(1) |
			    gr_gpcs_gpccs_gpc_exception_en_gpccs_f(1) |
			    gr_gpcs_gpccs_gpc_exception_en_gpcmmu_f(1)));
}

int gr_gv11b_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event)
{
	return 0;
}

int gr_gv11b_zbc_s_query_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params)
{
	u32 index = query_params->index_size;

	if (index >= GK20A_ZBC_TABLE_SIZE) {
		nvgpu_err(g, "invalid zbc stencil table index");
		return -EINVAL;
	}
	query_params->depth = gr->zbc_s_tbl[index].stencil;
	query_params->format = gr->zbc_s_tbl[index].format;
	query_params->ref_cnt = gr->zbc_s_tbl[index].ref_cnt;

	return 0;
}

bool gr_gv11b_add_zbc_type_s(struct gk20a *g, struct gr_gk20a *gr,
		     struct zbc_entry *zbc_val, int *ret_val)
{
	struct zbc_s_table *s_tbl;
	u32 i;
	bool added = false;

	*ret_val = -ENOMEM;

	/* search existing tables */
	for (i = 0; i < gr->max_used_s_index; i++) {

		s_tbl = &gr->zbc_s_tbl[i];

		if (s_tbl->ref_cnt &&
		    s_tbl->stencil == zbc_val->depth &&
		    s_tbl->format == zbc_val->format) {
			added = true;
			s_tbl->ref_cnt++;
			*ret_val = 0;
			break;
		}
	}
	/* add new table */
	if (!added &&
	    gr->max_used_s_index < GK20A_ZBC_TABLE_SIZE) {

		s_tbl = &gr->zbc_s_tbl[gr->max_used_s_index];
		WARN_ON(s_tbl->ref_cnt != 0);

		*ret_val = g->ops.gr.add_zbc_s(g, gr,
				zbc_val, gr->max_used_s_index);

		if (!(*ret_val))
			gr->max_used_s_index++;
	}
	return added;
}

int gr_gv11b_add_zbc_stencil(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *stencil_val, u32 index)
{
	u32 zbc_s;

	/* update l2 table */
	g->ops.ltc.set_zbc_s_entry(g, stencil_val, index);

	/* update local copy */
	gr->zbc_s_tbl[index].stencil = stencil_val->depth;
	gr->zbc_s_tbl[index].format = stencil_val->format;
	gr->zbc_s_tbl[index].ref_cnt++;

	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_s_r(index), stencil_val->depth);
	zbc_s = gk20a_readl(g, gr_gpcs_swdx_dss_zbc_s_01_to_04_format_r() +
				 (index & ~3));
	zbc_s &= ~(0x7f << (index % 4) * 7);
	zbc_s |= stencil_val->format << (index % 4) * 7;
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_s_01_to_04_format_r() +
				 (index & ~3), zbc_s);

	return 0;
}

int gr_gv11b_load_stencil_default_tbl(struct gk20a *g,
		 struct gr_gk20a *gr)
{
	struct zbc_entry zbc_val;
	u32 err;

	/* load default stencil table */
	zbc_val.type = GV11B_ZBC_TYPE_STENCIL;

	zbc_val.depth = 0x0;
	zbc_val.format = ZBC_STENCIL_CLEAR_FMT_U8;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);

	zbc_val.depth = 0x1;
	zbc_val.format = ZBC_STENCIL_CLEAR_FMT_U8;
	err |= gr_gk20a_add_zbc(g, gr, &zbc_val);

	zbc_val.depth = 0xff;
	zbc_val.format = ZBC_STENCIL_CLEAR_FMT_U8;
	err |= gr_gk20a_add_zbc(g, gr, &zbc_val);

	if (!err) {
		gr->max_default_s_index = 3;
	} else {
		nvgpu_err(g, "fail to load default zbc stencil table");
		return err;
	}

	return 0;
}

int gr_gv11b_load_stencil_tbl(struct gk20a *g, struct gr_gk20a *gr)
{
	int ret;
	u32 i;

	for (i = 0; i < gr->max_used_s_index; i++) {
		struct zbc_s_table *s_tbl = &gr->zbc_s_tbl[i];
		struct zbc_entry zbc_val;

		zbc_val.type = GV11B_ZBC_TYPE_STENCIL;
		zbc_val.depth = s_tbl->stencil;
		zbc_val.format = s_tbl->format;

		ret = g->ops.gr.add_zbc_s(g, gr, &zbc_val, i);
		if (ret)
			return ret;
	}
	return 0;
}

u32 gr_gv11b_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

int gr_gv11b_calc_global_ctx_buffer_size(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int size;

	gr->attrib_cb_size = gr->attrib_cb_default_size;
	gr->alpha_cb_size = gr->alpha_cb_default_size;

	gr->attrib_cb_size = min(gr->attrib_cb_size,
		 gr_gpc0_ppc0_cbm_beta_cb_size_v_f(~0) / g->gr.tpc_count);
	gr->alpha_cb_size = min(gr->alpha_cb_size,
		 gr_gpc0_ppc0_cbm_alpha_cb_size_v_f(~0) / g->gr.tpc_count);

	size = gr->attrib_cb_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	size += gr->alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	size = ALIGN(size, 128);

	return size;
}

static void gr_gv11b_set_go_idle_timeout(struct gk20a *g, u32 data)
{
	gk20a_writel(g, gr_fe_go_idle_timeout_r(), data);
}

static void gr_gv11b_set_coalesce_buffer_size(struct gk20a *g, u32 data)
{
	u32 val;

	gk20a_dbg_fn("");

	val = gk20a_readl(g, gr_gpcs_tc_debug0_r());
	val = set_field(val, gr_gpcs_tc_debug0_limit_coalesce_buffer_size_m(),
			     gr_gpcs_tc_debug0_limit_coalesce_buffer_size_f(data));
	gk20a_writel(g, gr_gpcs_tc_debug0_r(), val);

	gk20a_dbg_fn("done");
}

static void gr_gv11b_set_tex_in_dbg(struct gk20a *g, u32 data)
{
	u32 val;
	bool flag;

	gk20a_dbg_fn("");

	val = gk20a_readl(g, gr_gpcs_tpcs_tex_in_dbg_r());
	flag = (data & NVC397_SET_TEX_IN_DBG_TSL1_RVCH_INVALIDATE) ? 1 : 0;
	val = set_field(val, gr_gpcs_tpcs_tex_in_dbg_tsl1_rvch_invalidate_m(),
			gr_gpcs_tpcs_tex_in_dbg_tsl1_rvch_invalidate_f(flag));
	gk20a_writel(g, gr_gpcs_tpcs_tex_in_dbg_r(), val);

	val = gk20a_readl(g, gr_gpcs_tpcs_sm_l1tag_ctrl_r());
	flag = (data &
		NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_LD) ? 1 : 0;
	val = set_field(val, gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_ld_m(),
			gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_ld_f(flag));
	flag = (data &
		NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_ST) ? 1 : 0;
	val = set_field(val, gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_st_m(),
			gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_st_f(flag));
	gk20a_writel(g, gr_gpcs_tpcs_sm_l1tag_ctrl_r(), val);
}

static void gr_gv11b_set_skedcheck(struct gk20a *g, u32 data)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, gr_sked_hww_esr_en_r());

	if ((data & NVC397_SET_SKEDCHECK_18_MASK) ==
			NVC397_SET_SKEDCHECK_18_DISABLE) {
		reg_val = set_field(reg_val,
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_m(),
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_disabled_f()
			);
	} else if ((data & NVC397_SET_SKEDCHECK_18_MASK) ==
			NVC397_SET_SKEDCHECK_18_ENABLE) {
		reg_val = set_field(reg_val,
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_m(),
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_enabled_f()
			);
	}
	nvgpu_log_info(g, "sked_hww_esr_en = 0x%x", reg_val);
	gk20a_writel(g, gr_sked_hww_esr_en_r(), reg_val);

}

static void gv11b_gr_set_shader_exceptions(struct gk20a *g, u32 data)
{
	gk20a_dbg_fn("");

	if (data == NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE) {
		gk20a_writel(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r(),
				 0);
		gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r(),
				 0);
	} else {
		g->ops.gr.set_hww_esr_report_mask(g);
	}
}

int gr_gv11b_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data)
{
	gk20a_dbg_fn("");

	if (class_num == VOLTA_COMPUTE_A) {
		switch (offset << 2) {
		case NVC0C0_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
			break;
		case NVC3C0_SET_SKEDCHECK:
			gr_gv11b_set_skedcheck(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == VOLTA_A) {
		switch (offset << 2) {
		case NVC397_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
			break;
		case NVC397_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVC397_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_alpha_circular_buffer_size(g, data);
			break;
		case NVC397_SET_GO_IDLE_TIMEOUT:
			gr_gv11b_set_go_idle_timeout(g, data);
			break;
		case NVC097_SET_COALESCE_BUFFER_SIZE:
			gr_gv11b_set_coalesce_buffer_size(g, data);
			break;
		case NVC397_SET_TEX_IN_DBG:
			gr_gv11b_set_tex_in_dbg(g, data);
			break;
		case NVC397_SET_SKEDCHECK:
			gr_gv11b_set_skedcheck(g, data);
			break;
		case NVC397_SET_BES_CROP_DEBUG3:
			g->ops.gr.set_bes_crop_debug3(g, data);
			break;
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

void gr_gv11b_bundle_cb_defaults(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->bundle_cb_default_size =
		gr_scc_bundle_cb_size_div_256b__prod_v();
	gr->min_gpm_fifo_depth =
		gr_pd_ab_dist_cfg2_state_limit_min_gpm_fifo_depths_v();
	gr->bundle_cb_token_limit =
		gr_pd_ab_dist_cfg2_token_limit_init_v();
}

void gr_gv11b_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (!gr->attrib_cb_default_size)
		gr->attrib_cb_default_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v();
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
}

void gr_gv11b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 pd_ab_max_output;
	u32 alpha_cb_size = data * 4;

	gk20a_dbg_fn("");

	if (alpha_cb_size > gr->alpha_cb_size)
		alpha_cb_size = gr->alpha_cb_size;

	gk20a_writel(g, gr_ds_tga_constraintlogic_alpha_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_alpha_r()) &
		 ~gr_ds_tga_constraintlogic_alpha_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_alpha_cbsize_f(alpha_cb_size));

	pd_ab_max_output = alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() /
		gr_pd_ab_dist_cfg1_max_output_granularity_v();

	gk20a_writel(g, gr_pd_ab_dist_cfg1_r(),
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output) |
		gr_pd_ab_dist_cfg1_max_batches_init_f());

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		stride = proj_gpc_stride_v() * gpc_index;

		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
			ppc_index++) {

			val = gk20a_readl(g, gr_gpc0_ppc0_cbm_alpha_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index);

			val = set_field(val, gr_gpc0_ppc0_cbm_alpha_cb_size_v_m(),
					gr_gpc0_ppc0_cbm_alpha_cb_size_v_f(alpha_cb_size *
						gr->pes_tpc_count[ppc_index][gpc_index]));

			gk20a_writel(g, gr_gpc0_ppc0_cbm_alpha_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index, val);
		}
	}
}

void gr_gv11b_set_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 cb_size_steady = data * 4, cb_size;

	gk20a_dbg_fn("");

	if (cb_size_steady > gr->attrib_cb_size)
		cb_size_steady = gr->attrib_cb_size;
	if (gk20a_readl(g, gr_gpc0_ppc0_cbm_beta_cb_size_r()) !=
		gk20a_readl(g,
			gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_r())) {
		cb_size = cb_size_steady +
			(gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v() -
			 gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v());
	} else {
		cb_size = cb_size_steady;
	}

	gk20a_writel(g, gr_ds_tga_constraintlogic_beta_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_beta_r()) &
		 ~gr_ds_tga_constraintlogic_beta_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_beta_cbsize_f(cb_size_steady));

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		stride = proj_gpc_stride_v() * gpc_index;

		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
			ppc_index++) {

			val = gk20a_readl(g, gr_gpc0_ppc0_cbm_beta_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index);

			val = set_field(val,
				gr_gpc0_ppc0_cbm_beta_cb_size_v_m(),
				gr_gpc0_ppc0_cbm_beta_cb_size_v_f(cb_size *
					gr->pes_tpc_count[ppc_index][gpc_index]));

			gk20a_writel(g, gr_gpc0_ppc0_cbm_beta_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index, val);

			gk20a_writel(g, proj_ppc_in_gpc_stride_v() * ppc_index +
				gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_r() +
				stride,
				gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_v_f(
					cb_size_steady));

			val = gk20a_readl(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index));

			val = set_field(val,
				gr_gpcs_swdx_tc_beta_cb_size_v_m(),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(
					cb_size_steady *
					gr->gpc_ppc_count[gpc_index]));

			gk20a_writel(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index), val);
		}
	}
}

int gr_gv11b_alloc_buffer(struct vm_gk20a *vm, size_t size,
			struct nvgpu_mem *mem)
{
	int err;

	gk20a_dbg_fn("");

	err = nvgpu_dma_alloc_sys(vm->mm->g, size, mem);
	if (err)
		return err;

	mem->gpu_va = nvgpu_gmmu_map(vm,
				mem,
				size,
				NVGPU_MAP_BUFFER_FLAGS_CACHEABLE_TRUE,
				gk20a_mem_flag_none,
				false,
				mem->aperture);

	if (!mem->gpu_va) {
		err = -ENOMEM;
		goto fail_free;
	}

	return 0;

fail_free:
	nvgpu_dma_free(vm->mm->g, mem);
	return err;
}

static void gr_gv11b_dump_gr_per_sm_regs(struct gk20a *g,
			struct gk20a_debug_output *o,
			u32 gpc, u32 tpc, u32 sm, u32 offset)
{

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_WARP_ESR: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_WARP_ESR_REPORT_MASK: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_GLOBAL_ESR: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_GLOBAL_ESR_REPORT_MASK: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_DBGR_CONTROL0: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_DBGR_STATUS0: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset));
}

static int gr_gv11b_dump_gr_sm_regs(struct gk20a *g,
			   struct gk20a_debug_output *o)
{
	u32 gpc, tpc, sm, sm_per_tpc;
	u32 gpc_offset, tpc_offset, offset;

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_HWW_GLOBAL_ESR_REPORT_MASK: 0x%x\n",
		gk20a_readl(g,
		gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_HWW_WARP_ESR_REPORT_MASK: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_HWW_GLOBAL_ESR: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_hww_global_esr_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_CONTROL0: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_control0_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_STATUS0: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_status0_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_BPT_PAUSE_MASK_0: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_bpt_pause_mask_0_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_BPT_PAUSE_MASK_1: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_bpt_pause_mask_1_r()));

	sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		gpc_offset = gk20a_gr_gpc_offset(g, gpc);

		for (tpc = 0; tpc < g->gr.tpc_count; tpc++) {
			tpc_offset = gk20a_gr_tpc_offset(g, tpc);

			for (sm = 0; sm < sm_per_tpc; sm++) {
				offset = gpc_offset + tpc_offset +
					gv11b_gr_sm_offset(g, sm);

				gr_gv11b_dump_gr_per_sm_regs(g, o,
					gpc, tpc, sm, offset);
			}
		}
	}

	return 0;
}

int gr_gv11b_dump_gr_status_regs(struct gk20a *g,
			   struct gk20a_debug_output *o)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gr_engine_id;

	gr_engine_id = gk20a_fifo_get_gr_engine_id(g);

	gk20a_debug_output(o, "NV_PGRAPH_STATUS: 0x%x\n",
		gk20a_readl(g, gr_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_STATUS1: 0x%x\n",
		gk20a_readl(g, gr_status_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_STATUS2: 0x%x\n",
		gk20a_readl(g, gr_status_2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ENGINE_STATUS: 0x%x\n",
		gk20a_readl(g, gr_engine_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_GRFIFO_STATUS : 0x%x\n",
		gk20a_readl(g, gr_gpfifo_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_GRFIFO_CONTROL : 0x%x\n",
		gk20a_readl(g, gr_gpfifo_ctl_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_HOST_INT_STATUS : 0x%x\n",
		gk20a_readl(g, gr_fecs_host_int_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_EXCEPTION  : 0x%x\n",
		gk20a_readl(g, gr_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_FECS_INTR  : 0x%x\n",
		gk20a_readl(g, gr_fecs_intr_r()));
	gk20a_debug_output(o, "NV_PFIFO_ENGINE_STATUS(GR) : 0x%x\n",
		gk20a_readl(g, fifo_engine_status_r(gr_engine_id)));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY1: 0x%x\n",
		gk20a_readl(g, gr_activity_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY2: 0x%x\n",
		gk20a_readl(g, gr_activity_2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY4: 0x%x\n",
		gk20a_readl(g, gr_activity_4_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_SKED_ACTIVITY: 0x%x\n",
		gk20a_readl(g, gr_pri_sked_activity_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY1: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY2: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY3: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity3_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tpccs_tpc_activity_0_r()));
	if (gr->gpc_tpc_count[0] == 2)
		gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC1_TPCCS_TPC_ACTIVITY0: 0x%x\n",
			gk20a_readl(g, gr_pri_gpc0_tpc1_tpccs_tpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPCS_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpcs_tpccs_tpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY1: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY2: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY3: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_3_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_TPC0_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_tpc0_tpccs_tpc_activity_0_r()));
	if (gr->gpc_tpc_count[0] == 2)
		gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_TPC1_TPCCS_TPC_ACTIVITY0: 0x%x\n",
			gk20a_readl(g, gr_pri_gpcs_tpc1_tpccs_tpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_TPCS_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_tpcs_tpccs_tpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_BECS_BE_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_be0_becs_be_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE1_BECS_BE_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_be1_becs_be_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_BECS_BE_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_bes_becs_be_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_DS_MPIPE_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_ds_mpipe_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_TIMEOUT : 0x%x\n",
		gk20a_readl(g, gr_fe_go_idle_timeout_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_INFO : 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_info_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TEX_M_TEX_SUBUNITS_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tex_m_tex_subunits_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_FS: 0x%x\n",
		gk20a_readl(g, gr_cwd_fs_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_TPC_FS(0): 0x%x\n",
		gk20a_readl(g, gr_fe_tpc_fs_r(0)));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_GPC_TPC_ID: 0x%x\n",
		gk20a_readl(g, gr_cwd_gpc_tpc_id_r(0)));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_SM_ID(0): 0x%x\n",
		gk20a_readl(g, gr_cwd_sm_id_r(0)));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CTXSW_STATUS_FE_0: 0x%x\n",
		gk20a_readl(g, gr_fecs_ctxsw_status_fe_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CTXSW_STATUS_1: 0x%x\n",
		gk20a_readl(g, gr_fecs_ctxsw_status_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_CTXSW_STATUS_GPC_0: 0x%x\n",
		gk20a_readl(g, gr_gpc0_gpccs_ctxsw_status_gpc_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_CTXSW_STATUS_1: 0x%x\n",
		gk20a_readl(g, gr_gpc0_gpccs_ctxsw_status_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CTXSW_IDLESTATE : 0x%x\n",
		gk20a_readl(g, gr_fecs_ctxsw_idlestate_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_CTXSW_IDLESTATE : 0x%x\n",
		gk20a_readl(g, gr_gpc0_gpccs_ctxsw_idlestate_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CURRENT_CTX : 0x%x\n",
		gk20a_readl(g, gr_fecs_current_ctx_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_NEW_CTX : 0x%x\n",
		gk20a_readl(g, gr_fecs_new_ctx_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_HOST_INT_ENABLE : 0x%x\n",
		gk20a_readl(g, gr_fecs_host_int_enable_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_HOST_INT_STATUS : 0x%x\n",
		gk20a_readl(g, gr_fecs_host_int_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_CROP_STATUS1 : 0x%x\n",
		gk20a_readl(g, gr_pri_be0_crop_status1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_CROP_STATUS1 : 0x%x\n",
		gk20a_readl(g, gr_pri_bes_crop_status1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_ZROP_STATUS : 0x%x\n",
		gk20a_readl(g, gr_pri_be0_zrop_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_ZROP_STATUS2 : 0x%x\n",
		gk20a_readl(g, gr_pri_be0_zrop_status2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_ZROP_STATUS : 0x%x\n",
		gk20a_readl(g, gr_pri_bes_zrop_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_ZROP_STATUS2 : 0x%x\n",
		gk20a_readl(g, gr_pri_bes_zrop_status2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_BECS_BE_EXCEPTION: 0x%x\n",
		gk20a_readl(g, gr_pri_be0_becs_be_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_BECS_BE_EXCEPTION_EN: 0x%x\n",
		gk20a_readl(g, gr_pri_be0_becs_be_exception_en_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_EXCEPTION: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_EXCEPTION_EN: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_exception_en_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TPCCS_TPC_EXCEPTION: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tpccs_tpc_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TPCCS_TPC_EXCEPTION_EN: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tpccs_tpc_exception_en_r()));

	gr_gv11b_dump_gr_sm_regs(g, o);

	return 0;
}

static bool gr_activity_empty_or_preempted(u32 val)
{
	while (val) {
		u32 v = val & 7;
		if (v != gr_activity_4_gpc0_empty_v() &&
		    v != gr_activity_4_gpc0_preempted_v())
			return false;
		val >>= 3;
	}

	return true;
}

int gr_gv11b_wait_empty(struct gk20a *g, unsigned long duration_ms,
		       u32 expect_delay)
{
	u32 delay = expect_delay;
	bool gr_enabled;
	bool ctxsw_active;
	bool gr_busy;
	u32 gr_status;
	u32 activity0, activity1, activity2, activity4;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");

	nvgpu_timeout_init(g, &timeout, duration_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		/* fmodel: host gets fifo_engine_status(gr) from gr
		   only when gr_status is read */
		gr_status = gk20a_readl(g, gr_status_r());

		gr_enabled = gk20a_readl(g, mc_enable_r()) &
			mc_enable_pgraph_enabled_f();

		ctxsw_active = gr_status & 1<<7;

		activity0 = gk20a_readl(g, gr_activity_0_r());
		activity1 = gk20a_readl(g, gr_activity_1_r());
		activity2 = gk20a_readl(g, gr_activity_2_r());
		activity4 = gk20a_readl(g, gr_activity_4_r());

		gr_busy = !(gr_activity_empty_or_preempted(activity0) &&
			    gr_activity_empty_or_preempted(activity1) &&
			    activity2 == 0 &&
			    gr_activity_empty_or_preempted(activity4));

		if (!gr_enabled || (!gr_busy && !ctxsw_active)) {
			gk20a_dbg_fn("done");
			return 0;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);

	} while (!nvgpu_timeout_expired(&timeout));

	nvgpu_err(g,
		"timeout, ctxsw busy : %d, gr busy : %d, %08x, %08x, %08x, %08x",
		ctxsw_active, gr_busy, activity0, activity1, activity2, activity4);

	return -EAGAIN;
}

void gr_gv11b_commit_global_attrib_cb(struct gk20a *g,
					     struct channel_ctx_gk20a *ch_ctx,
					     u64 addr, bool patch)
{
	struct gr_ctx_desc *gr_ctx = ch_ctx->gr_ctx;
	int attrBufferSize;

	if (gr_ctx->t18x.preempt_ctxsw_buffer.gpu_va)
		attrBufferSize = gr_ctx->t18x.betacb_ctxsw_buffer.size;
	else
		attrBufferSize = g->ops.gr.calc_global_ctx_buffer_size(g);

	attrBufferSize /= gr_gpcs_tpcs_tex_rm_cb_1_size_div_128b_granularity_f();

	gr_gm20b_commit_global_attrib_cb(g, ch_ctx, addr, patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_r(),
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_v_f(addr) |
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_tex_rm_cb_0_r(),
		gr_gpcs_tpcs_tex_rm_cb_0_base_addr_43_12_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_tex_rm_cb_1_r(),
		gr_gpcs_tpcs_tex_rm_cb_1_size_div_128b_f(attrBufferSize) |
		gr_gpcs_tpcs_tex_rm_cb_1_valid_true_f(), patch);
}


void gr_gv11b_init_cyclestats(struct gk20a *g)
{
#if defined(CONFIG_GK20A_CYCLE_STATS)
	g->gpu_characteristics.flags |=
		NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS;
	g->gpu_characteristics.flags |=
		NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS_SNAPSHOT;
#else
	(void)g;
#endif
}

void gr_gv11b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	tegra_fuse_writel(0x1, FUSE_FUSEBYPASS_0);
	tegra_fuse_writel(0x0, FUSE_WRITE_ACCESS_SW_0);
#else
	tegra_fuse_control_write(0x1, FUSE_FUSEBYPASS_0);
	tegra_fuse_control_write(0x0, FUSE_WRITE_ACCESS_SW_0);
#endif

	if (g->gr.gpc_tpc_mask[gpc_index] == 0x1)
		tegra_fuse_writel(0x2, FUSE_OPT_GPU_TPC0_DISABLE_0);
	else if (g->gr.gpc_tpc_mask[gpc_index] == 0x2)
		tegra_fuse_writel(0x1, FUSE_OPT_GPU_TPC0_DISABLE_0);
	else
		tegra_fuse_writel(0x0, FUSE_OPT_GPU_TPC0_DISABLE_0);
}

void gr_gv11b_get_access_map(struct gk20a *g,
				   u32 **whitelist, int *num_entries)
{
	static u32 wl_addr_gv11b[] = {
		/* this list must be sorted (low to high) */
		0x404468, /* gr_pri_mme_max_instructions       */
		0x418300, /* gr_pri_gpcs_rasterarb_line_class  */
		0x418800, /* gr_pri_gpcs_setup_debug           */
		0x418e00, /* gr_pri_gpcs_swdx_config           */
		0x418e40, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e44, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e48, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e4c, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e50, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e58, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e5c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e60, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e64, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e68, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e6c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e70, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e74, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e78, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e7c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e80, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e84, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e88, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e8c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e90, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e94, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x419864, /* gr_pri_gpcs_tpcs_pe_l2_evict_policy */
		0x419a04, /* gr_pri_gpcs_tpcs_tex_lod_dbg      */
		0x419a08, /* gr_pri_gpcs_tpcs_tex_samp_dbg     */
		0x419e84, /* gr_pri_gpcs_tpcs_sms_dbgr_control0 */
		0x419ba4, /* gr_pri_gpcs_tpcs_sm_disp_ctrl     */
	};

	*whitelist = wl_addr_gv11b;
	*num_entries = ARRAY_SIZE(wl_addr_gv11b);
}

/* @brief pre-process work on the SM exceptions to determine if we clear them or not.
 *
 * On Pascal, if we are in CILP preemtion mode, preempt the channel and handle errors with special processing
 */
int gr_gv11b_pre_process_sm_exception(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm, u32 global_esr, u32 warp_esr,
		bool sm_debugger_attached, struct channel_gk20a *fault_ch,
		bool *early_exit, bool *ignore_debugger)
{
	int ret;
	bool cilp_enabled = false;
	u32 global_mask = 0, dbgr_control0, global_esr_copy;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	*early_exit = false;
	*ignore_debugger = false;

	if (fault_ch)
		cilp_enabled = (fault_ch->ch_ctx.gr_ctx->compute_preempt_mode ==
			NVGPU_COMPUTE_PREEMPTION_MODE_CILP);

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"SM Exception received on gpc %d tpc %d sm %d = 0x%08x",
			gpc, tpc, sm, global_esr);

	if (cilp_enabled && sm_debugger_attached) {
		if (global_esr & gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f())
			gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
					gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f());

		if (global_esr & gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f())
			gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
					gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f());

		global_mask = gr_gpc0_tpc0_sm0_hww_global_esr_multiple_warp_errors_pending_f() |
			gr_gpc0_tpc0_sm0_hww_global_esr_bpt_pause_pending_f();

		if (warp_esr != 0 || (global_esr & global_mask) != 0) {
			*ignore_debugger = true;

			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"CILP: starting wait for LOCKED_DOWN on "
				"gpc %d tpc %d sm %d",
				gpc, tpc, sm);

			if (gk20a_dbg_gpu_broadcast_stop_trigger(fault_ch)) {
				gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: Broadcasting STOP_TRIGGER from "
					"gpc %d tpc %d sm %d",
					gpc, tpc, sm);
				g->ops.gr.suspend_all_sms(g,
						global_mask, false);

				gk20a_dbg_gpu_clear_broadcast_stop_trigger(fault_ch);
			} else {
				gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: STOP_TRIGGER from "
					"gpc %d tpc %d sm %d",
					gpc, tpc, sm);
				g->ops.gr.suspend_single_sm(g,
					gpc, tpc, sm, global_mask, true);
			}

			/* reset the HWW errors after locking down */
			global_esr_copy = g->ops.gr.get_sm_hww_global_esr(g,
							gpc, tpc, sm);
			g->ops.gr.clear_sm_hww(g,
					gpc, tpc, sm, global_esr_copy);
			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: HWWs cleared for "
					"gpc %d tpc %d sm %d",
					gpc, tpc, sm);

			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "CILP: Setting CILP preempt pending\n");
			ret = gr_gp10b_set_cilp_preempt_pending(g, fault_ch);
			if (ret) {
				nvgpu_err(g, "CILP: error while setting CILP preempt pending!");
				return ret;
			}

			dbgr_control0 = gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
			if (dbgr_control0 & gr_gpc0_tpc0_sm0_dbgr_control0_single_step_mode_enable_f()) {
				gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: clearing SINGLE_STEP_MODE "
					"before resume for gpc %d tpc %d sm %d",
						gpc, tpc, sm);
				dbgr_control0 = set_field(dbgr_control0,
						gr_gpc0_tpc0_sm0_dbgr_control0_single_step_mode_m(),
						gr_gpc0_tpc0_sm0_dbgr_control0_single_step_mode_disable_f());
				gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset, dbgr_control0);
			}

			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"CILP: resume for gpc %d tpc %d sm %d",
					gpc, tpc, sm);
			g->ops.gr.resume_single_sm(g, gpc, tpc, sm);

			*ignore_debugger = true;
			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"CILP: All done on gpc %d, tpc %d sm %d",
				gpc, tpc, sm);
		}

		*early_exit = true;
	}
	return 0;
}

static void gr_gv11b_handle_fecs_ecc_error(struct gk20a *g, u32 intr)
{
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;

	if (intr & (gr_fecs_host_int_status_ecc_uncorrected_m() |
		    gr_fecs_host_int_status_ecc_corrected_m())) {
		ecc_status = gk20a_readl(g, gr_fecs_falcon_ecc_status_r());
		ecc_addr = gk20a_readl(g,
			gr_fecs_falcon_ecc_address_r());
		corrected_cnt = gk20a_readl(g,
			gr_fecs_falcon_ecc_corrected_err_count_r());
		uncorrected_cnt = gk20a_readl(g,
			gr_fecs_falcon_ecc_uncorrected_err_count_r());

		corrected_delta =
			gr_fecs_falcon_ecc_corrected_err_count_total_v(
							corrected_cnt);
		uncorrected_delta =
			gr_fecs_falcon_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);

		corrected_overflow = ecc_status &
			gr_fecs_falcon_ecc_status_corrected_err_total_counter_overflow_m();
		uncorrected_overflow = ecc_status &
			gr_fecs_falcon_ecc_status_uncorrected_err_total_counter_overflow_m();

		/* clear the interrupt */
		if ((corrected_delta > 0) || corrected_overflow)
			gk20a_writel(g,
				gr_fecs_falcon_ecc_corrected_err_count_r(), 0);
		if ((uncorrected_delta > 0) || uncorrected_overflow)
			gk20a_writel(g,
				gr_fecs_falcon_ecc_uncorrected_err_count_r(),
				0);


		/* clear the interrupt */
		gk20a_writel(g, gr_fecs_falcon_ecc_uncorrected_err_count_r(),
				0);
		gk20a_writel(g, gr_fecs_falcon_ecc_corrected_err_count_r(), 0);

		/* clear the interrupt */
		gk20a_writel(g, gr_fecs_falcon_ecc_status_r(),
				gr_fecs_falcon_ecc_status_reset_task_f());

		g->ecc.gr.t19x.fecs_corrected_err_count.counters[0] +=
							corrected_delta;
		g->ecc.gr.t19x.fecs_uncorrected_err_count.counters[0] +=
							uncorrected_delta;

		nvgpu_log(g, gpu_dbg_intr,
			"fecs ecc interrupt intr: 0x%x", intr);

		if (ecc_status &
			gr_fecs_falcon_ecc_status_corrected_err_imem_m())
			nvgpu_log(g, gpu_dbg_intr, "imem ecc error corrected");
		if (ecc_status &
			gr_fecs_falcon_ecc_status_uncorrected_err_imem_m())
			nvgpu_log(g, gpu_dbg_intr,
						"imem ecc error uncorrected");
		if (ecc_status &
			gr_fecs_falcon_ecc_status_corrected_err_dmem_m())
			nvgpu_log(g, gpu_dbg_intr, "dmem ecc error corrected");
		if (ecc_status &
			gr_fecs_falcon_ecc_status_uncorrected_err_dmem_m())
			nvgpu_log(g, gpu_dbg_intr,
						"dmem ecc error uncorrected");
		if (corrected_overflow || uncorrected_overflow)
			nvgpu_info(g, "fecs ecc counter overflow!");

		nvgpu_log(g, gpu_dbg_intr,
			"ecc error row address: 0x%x",
			gr_fecs_falcon_ecc_address_row_address_v(ecc_addr));

		nvgpu_log(g, gpu_dbg_intr,
			"ecc error count corrected: %d, uncorrected %d",
			g->ecc.gr.t19x.fecs_corrected_err_count.counters[0],
			g->ecc.gr.t19x.fecs_uncorrected_err_count.counters[0]);
	}
}

int gr_gv11b_handle_fecs_error(struct gk20a *g,
				struct channel_gk20a *__ch,
				struct gr_gk20a_isr_data *isr_data)
{
	u32 gr_fecs_intr = gk20a_readl(g, gr_fecs_host_int_status_r());
	int ret;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	ret = gr_gp10b_handle_fecs_error(g, __ch, isr_data);

	/* Handle ECC errors */
	gr_gv11b_handle_fecs_ecc_error(g, gr_fecs_intr);

	return ret;
}

int gr_gv11b_setup_rop_mapping(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 map;
	u32 i, j, mapregs;
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g,
				GPU_LIT_NUM_TPC_PER_GPC);

	gk20a_dbg_fn("");

	if (!gr->map_tiles)
		return -1;

	gk20a_writel(g, gr_crstr_map_table_cfg_r(),
		gr_crstr_map_table_cfg_row_offset_f(gr->map_row_offset) |
		gr_crstr_map_table_cfg_num_entries_f(gr->tpc_count));

	/* 6 tpc can be stored in one map register */
	mapregs = (num_gpcs * num_tpc_per_gpc + 5) / 6;

	for (i = 0, j = 0; i < mapregs; i++, j = j + 6) {
		map =  gr_crstr_gpc_map_tile0_f(gr->map_tiles[j]) |
			gr_crstr_gpc_map_tile1_f(gr->map_tiles[j + 1]) |
			gr_crstr_gpc_map_tile2_f(gr->map_tiles[j + 2]) |
			gr_crstr_gpc_map_tile3_f(gr->map_tiles[j + 3]) |
			gr_crstr_gpc_map_tile4_f(gr->map_tiles[j + 4]) |
			gr_crstr_gpc_map_tile5_f(gr->map_tiles[j + 5]);

		gk20a_writel(g, gr_crstr_gpc_map_r(i), map);
		gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map_r(i), map);
		gk20a_writel(g, gr_rstr2d_gpc_map_r(i), map);
	}

	gk20a_writel(g, gr_ppcs_wwdx_map_table_cfg_r(),
		gr_ppcs_wwdx_map_table_cfg_row_offset_f(gr->map_row_offset) |
		gr_ppcs_wwdx_map_table_cfg_num_entries_f(gr->tpc_count));

	for (i = 0, j = 1; i < gr_ppcs_wwdx_map_table_cfg_coeff__size_1_v();
					i++, j = j + 4) {
		gk20a_writel(g, gr_ppcs_wwdx_map_table_cfg_coeff_r(i),
			gr_ppcs_wwdx_map_table_cfg_coeff_0_mod_value_f(
					((1 << j) % gr->tpc_count)) |
			gr_ppcs_wwdx_map_table_cfg_coeff_1_mod_value_f(
				((1 << (j + 1)) % gr->tpc_count)) |
			gr_ppcs_wwdx_map_table_cfg_coeff_2_mod_value_f(
				((1 << (j + 2)) % gr->tpc_count)) |
			gr_ppcs_wwdx_map_table_cfg_coeff_3_mod_value_f(
				((1 << (j + 3)) % gr->tpc_count)));
	}

	gk20a_writel(g, gr_rstr2d_map_table_cfg_r(),
		gr_rstr2d_map_table_cfg_row_offset_f(gr->map_row_offset) |
		gr_rstr2d_map_table_cfg_num_entries_f(gr->tpc_count));

	return 0;
}

static void gv11b_write_bundle_veid_state(struct gk20a *g, u32 index)
{
	struct av_list_gk20a *sw_veid_bundle_init =
			&g->gr.ctx_vars.sw_veid_bundle_init;
	u32 j;
	u32  num_subctx, err = 0;

	num_subctx = g->fifo.t19x.max_subctx_count;

	for (j = 0; j < num_subctx; j++) {

		gk20a_writel(g, gr_pipe_bundle_address_r(),
			sw_veid_bundle_init->l[index].addr |
			gr_pipe_bundle_address_veid_f(j));

		err = gr_gk20a_wait_fe_idle(g, gk20a_get_gr_idle_timeout(g),
					    GR_IDLE_CHECK_DEFAULT);
	}
}

int gr_gv11b_init_sw_veid_bundle(struct gk20a *g)
{
	struct av_list_gk20a *sw_veid_bundle_init =
			&g->gr.ctx_vars.sw_veid_bundle_init;
	u32 i;
	u32 last_bundle_data = 0;
	u32 err = 0;

	gk20a_dbg_fn("");
	for (i = 0; i < sw_veid_bundle_init->count; i++) {

		if (i == 0 || last_bundle_data !=
				sw_veid_bundle_init->l[i].value) {
			gk20a_writel(g, gr_pipe_bundle_data_r(),
				sw_veid_bundle_init->l[i].value);
			last_bundle_data = sw_veid_bundle_init->l[i].value;
		}

		if (gr_pipe_bundle_address_value_v(
			sw_veid_bundle_init->l[i].addr) == GR_GO_IDLE_BUNDLE) {
				gk20a_writel(g, gr_pipe_bundle_address_r(),
					sw_veid_bundle_init->l[i].addr);
				err |= gr_gk20a_wait_idle(g,
						gk20a_get_gr_idle_timeout(g),
						GR_IDLE_CHECK_DEFAULT);
		} else
			gv11b_write_bundle_veid_state(g, i);

		if (err)
			break;
	}
	gk20a_dbg_fn("done");
	return err;
}

void gr_gv11b_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
					u32 *zcull_map_tiles)
{
	u32 val, i, j;

	gk20a_dbg_fn("");

	for (i = 0, j = 0; i < (zcull_num_entries / 8); i++, j += 8) {
		val =
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_0_f(
						zcull_map_tiles[j+0]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_1_f(
						zcull_map_tiles[j+1]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_2_f(
						zcull_map_tiles[j+2]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_3_f(
						zcull_map_tiles[j+3]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_4_f(
						zcull_map_tiles[j+4]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_5_f(
						zcull_map_tiles[j+5]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_6_f(
						zcull_map_tiles[j+6]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_7_f(
						zcull_map_tiles[j+7]);

		gk20a_writel(g, gr_gpcs_zcull_sm_in_gpc_number_map_r(i), val);
	}
}

void gr_gv11b_detect_sm_arch(struct gk20a *g)
{
	u32 v = gk20a_readl(g, gr_gpc0_tpc0_sm_arch_r());

	g->gpu_characteristics.sm_arch_spa_version =
		gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	g->gpu_characteristics.sm_arch_sm_version =
		gr_gpc0_tpc0_sm_arch_sm_version_v(v);
	g->gpu_characteristics.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

void gr_gv11b_init_sm_id_table(struct gk20a *g)
{
	u32 gpc, tpc, sm;
	u32 sm_id = 0;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	/* TODO populate smids based on power efficiency */
	for (tpc = 0; tpc < g->gr.max_tpc_per_gpc_count; tpc++) {
		for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {

			if (tpc >= g->gr.gpc_tpc_count[gpc])
				continue;

			for (sm = 0; sm < sm_per_tpc; sm++) {
				g->gr.sm_to_cluster[sm_id].tpc_index = tpc;
				g->gr.sm_to_cluster[sm_id].gpc_index = gpc;
				g->gr.sm_to_cluster[sm_id].sm_index = sm_id % 2;
				g->gr.sm_to_cluster[sm_id].global_tpc_index =
									tpc;
				sm_id++;
			}
		}
	}
	g->gr.no_of_sm = sm_id;
	nvgpu_log_info(g, " total number of sm = %d", g->gr.no_of_sm);
}

void gr_gv11b_program_sm_id_numbering(struct gk20a *g,
					u32 gpc, u32 tpc, u32 smid)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 gpc_offset = gpc_stride * gpc;
	u32 tpc_offset = tpc_in_gpc_stride * tpc;
	u32 global_tpc_index = g->gr.sm_to_cluster[smid].global_tpc_index;

	gk20a_writel(g, gr_gpc0_tpc0_sm_cfg_r() + gpc_offset + tpc_offset,
		gr_gpc0_tpc0_sm_cfg_tpc_id_f(global_tpc_index));
	gk20a_writel(g, gr_gpc0_gpm_pd_sm_id_r(tpc) + gpc_offset,
			gr_gpc0_gpm_pd_sm_id_id_f(global_tpc_index));
	gk20a_writel(g, gr_gpc0_tpc0_pe_cfg_smid_r() + gpc_offset + tpc_offset,
			gr_gpc0_tpc0_pe_cfg_smid_value_f(global_tpc_index));
}

int gr_gv11b_load_smid_config(struct gk20a *g)
{
	u32 *tpc_sm_id;
	u32 i, j;
	u32 tpc_index, gpc_index, tpc_id;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	int num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);

	tpc_sm_id = kcalloc(gr_cwd_sm_id__size_1_v(), sizeof(u32), GFP_KERNEL);
	if (!tpc_sm_id)
		return -ENOMEM;

	/* Each NV_PGRAPH_PRI_CWD_GPC_TPC_ID can store 4 TPCs.*/
	for (i = 0; i <= ((g->gr.tpc_count-1) / 4); i++) {
		u32 reg = 0;
		u32 bit_stride = gr_cwd_gpc_tpc_id_gpc0_s() +
				 gr_cwd_gpc_tpc_id_tpc0_s();

		for (j = 0; j < 4; j++) {
			u32 sm_id;
			u32 bits;

			tpc_id = (i << 2) + j;
			sm_id = tpc_id * sm_per_tpc;

			if (sm_id >= g->gr.no_of_sm)
				break;

			gpc_index = g->gr.sm_to_cluster[sm_id].gpc_index;
			tpc_index = g->gr.sm_to_cluster[sm_id].tpc_index;

			bits = gr_cwd_gpc_tpc_id_gpc0_f(gpc_index) |
				gr_cwd_gpc_tpc_id_tpc0_f(tpc_index);
			reg |= bits << (j * bit_stride);

			tpc_sm_id[gpc_index + (num_gpcs * ((tpc_index & 4)
				 >> 2))] |= tpc_id << tpc_index * bit_stride;
		}
		gk20a_writel(g, gr_cwd_gpc_tpc_id_r(i), reg);
	}

	for (i = 0; i < gr_cwd_sm_id__size_1_v(); i++)
		gk20a_writel(g, gr_cwd_sm_id_r(i), tpc_sm_id[i]);
	kfree(tpc_sm_id);

	return 0;
}

int gr_gv11b_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	u32 addr_lo;
	u32 addr_hi;
	struct ctx_header_desc *ctx;
	int err;

	gk20a_dbg_fn("");

	err = gv11b_alloc_subctx_header(c);
	if (err)
		return err;

	err = gv11b_update_subctx_header(c, gpu_va);
	if (err)
		return err;

	ctx = &c->ch_ctx.ctx_header;
	addr_lo = u64_lo32(ctx->mem.gpu_va) >> ram_in_base_shift_v();
	addr_hi = u64_hi32(ctx->mem.gpu_va);

	/* point this address to engine_wfi_ptr */
	nvgpu_mem_wr32(c->g, &c->inst_block, ram_in_engine_wfi_target_w(),
		ram_in_engine_cs_wfi_v() |
		ram_in_engine_wfi_mode_f(ram_in_engine_wfi_mode_virtual_v()) |
		ram_in_engine_wfi_ptr_lo_f(addr_lo));

	nvgpu_mem_wr32(c->g, &c->inst_block, ram_in_engine_wfi_ptr_hi_w(),
		ram_in_engine_wfi_ptr_hi_f(addr_hi));

	return 0;
}



int gr_gv11b_commit_global_timeslice(struct gk20a *g,
					struct channel_gk20a *c, bool patch)
{
	struct channel_ctx_gk20a *ch_ctx = NULL;
	u32 pd_ab_dist_cfg0;
	u32 ds_debug;
	u32 mpc_vtg_debug;
	u32 pe_vaf;
	u32 pe_vsc_vpc;

	gk20a_dbg_fn("");

	pd_ab_dist_cfg0 = gk20a_readl(g, gr_pd_ab_dist_cfg0_r());
	ds_debug = gk20a_readl(g, gr_ds_debug_r());
	mpc_vtg_debug = gk20a_readl(g, gr_gpcs_tpcs_mpc_vtg_debug_r());

	if (patch) {
		int err;

		ch_ctx = &c->ch_ctx;
		err = gr_gk20a_ctx_patch_write_begin(g, ch_ctx);
		if (err)
			return err;
	}

	pe_vaf = gk20a_readl(g, gr_gpcs_tpcs_pe_vaf_r());
	pe_vsc_vpc = gk20a_readl(g, gr_gpcs_tpcs_pes_vsc_vpc_r());

	pe_vaf = gr_gpcs_tpcs_pe_vaf_fast_mode_switch_true_f() | pe_vaf;
	pe_vsc_vpc = gr_gpcs_tpcs_pes_vsc_vpc_fast_mode_switch_true_f() |
								pe_vsc_vpc;
	pd_ab_dist_cfg0 = gr_pd_ab_dist_cfg0_timeslice_enable_en_f() |
							pd_ab_dist_cfg0;
	ds_debug = gr_ds_debug_timeslice_mode_enable_f() | ds_debug;
	mpc_vtg_debug = gr_gpcs_tpcs_mpc_vtg_debug_timeslice_mode_enabled_f() |
							mpc_vtg_debug;

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_pe_vaf_r(), pe_vaf,
									patch);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_pes_vsc_vpc_r(),
							pe_vsc_vpc, patch);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg0_r(),
							pd_ab_dist_cfg0, patch);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_ds_debug_r(), ds_debug, patch);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_mpc_vtg_debug_r(),
							mpc_vtg_debug, patch);

	if (patch)
		gr_gk20a_ctx_patch_write_end(g, ch_ctx);

	return 0;
}

void gv11b_restore_context_header(struct gk20a *g,
					struct nvgpu_mem *ctxheader)
{
	u32 va_lo, va_hi;
	struct gr_gk20a *gr = &g->gr;

	va_hi = nvgpu_mem_rd(g, ctxheader,
			ctxsw_prog_main_image_context_buffer_ptr_hi_o());
	va_lo = nvgpu_mem_rd(g, ctxheader,
			ctxsw_prog_main_image_context_buffer_ptr_o());
	nvgpu_mem_wr_n(g, ctxheader, 0,
                       gr->ctx_vars.local_golden_image,
                       gr->ctx_vars.golden_image_size);
	nvgpu_mem_wr(g, ctxheader,
			ctxsw_prog_main_image_context_buffer_ptr_hi_o(), va_hi);
	nvgpu_mem_wr(g, ctxheader,
			ctxsw_prog_main_image_context_buffer_ptr_o(), va_lo);
}
void gr_gv11b_write_zcull_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 va_lo, va_hi;

	gpu_va = gpu_va >> 8;
	va_lo = u64_lo32(gpu_va);
	va_hi = u64_hi32(gpu_va);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_zcull_ptr_o(), va_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_zcull_ptr_hi_o(), va_hi);
}


void gr_gv11b_write_pm_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 va_lo, va_hi;

	gpu_va = gpu_va >> 8;
	va_lo = u64_lo32(gpu_va);
	va_hi = u64_hi32(gpu_va);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_pm_ptr_o(), va_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_pm_ptr_hi_o(), va_hi);
}

void gr_gv11b_init_elcg_mode(struct gk20a *g, u32 mode, u32 engine)
{
	u32 gate_ctrl;

	gate_ctrl = gk20a_readl(g, therm_gate_ctrl_r(engine));

	switch (mode) {
	case ELCG_RUN:
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_clk_m(),
				therm_gate_ctrl_eng_clk_run_f());
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_idle_holdoff_m(),
				therm_gate_ctrl_idle_holdoff_on_f());
		break;
	case ELCG_STOP:
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_clk_m(),
				therm_gate_ctrl_eng_clk_stop_f());
		break;
	case ELCG_AUTO:
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_clk_m(),
				therm_gate_ctrl_eng_clk_auto_f());
		break;
	default:
		nvgpu_err(g, "invalid elcg mode %d", mode);
	}

	gk20a_writel(g, therm_gate_ctrl_r(engine), gate_ctrl);
}

void gr_gv11b_load_tpc_mask(struct gk20a *g)
{
	u32 pes_tpc_mask = 0, fuse_tpc_mask;
	u32 gpc, pes, val;
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g,
				GPU_LIT_NUM_TPC_PER_GPC);

	/* gv11b has 1 GPC and 4 TPC/GPC, so mask will not overflow u32 */
	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		for (pes = 0; pes < g->gr.pe_count_per_gpc; pes++) {
			pes_tpc_mask |= g->gr.pes_tpc_mask[pes][gpc] <<
				num_tpc_per_gpc * gpc;
		}
	}

	gk20a_dbg_info("pes_tpc_mask %u\n", pes_tpc_mask);
	fuse_tpc_mask = g->ops.gr.get_gpc_tpc_mask(g, gpc);
	if (g->tpc_fs_mask_user &&
			g->tpc_fs_mask_user != fuse_tpc_mask &&
			fuse_tpc_mask == (0x1U << g->gr.max_tpc_count) - 1U) {
		val = g->tpc_fs_mask_user;
		val &= (0x1U << g->gr.max_tpc_count) - 1U;
		val = (0x1U << hweight32(val)) - 1U;
		gk20a_writel(g, gr_fe_tpc_fs_r(0), val);
	} else {
		gk20a_writel(g, gr_fe_tpc_fs_r(0), pes_tpc_mask);
	}

}

void gr_gv11b_set_preemption_buffer_va(struct gk20a *g,
			struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 addr_lo, addr_hi;

	addr_lo = u64_lo32(gpu_va);
	addr_hi = u64_hi32(gpu_va);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_o(), addr_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_hi_o(), addr_hi);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_veid0_o(), addr_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_veid0_hi_o(),
		addr_hi);

}

int gr_gv11b_init_fs_state(struct gk20a *g)
{
	u32 data;

	gk20a_dbg_fn("");

	data = gk20a_readl(g, gr_gpcs_tpcs_sm_texio_control_r());
	data = set_field(data, gr_gpcs_tpcs_sm_texio_control_oor_addr_check_mode_m(),
			gr_gpcs_tpcs_sm_texio_control_oor_addr_check_mode_arm_63_48_match_f());
	gk20a_writel(g, gr_gpcs_tpcs_sm_texio_control_r(), data);

	data = gk20a_readl(g, gr_gpcs_tpcs_sm_disp_ctrl_r());
	data = set_field(data, gr_gpcs_tpcs_sm_disp_ctrl_re_suppress_m(),
			 gr_gpcs_tpcs_sm_disp_ctrl_re_suppress_disable_f());
	gk20a_writel(g, gr_gpcs_tpcs_sm_disp_ctrl_r(), data);

	if (g->gr.t18x.fecs_feature_override_ecc_val != 0) {
		gk20a_writel(g,
			gr_fecs_feature_override_ecc_r(),
			g->gr.t18x.fecs_feature_override_ecc_val);
	}

	return gr_gm20b_init_fs_state(g);
}

void gv11b_gr_get_esr_sm_sel(struct gk20a *g, u32 gpc, u32 tpc,
				u32 *esr_sm_sel)
{
	u32 reg_val;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);

	reg_val = gk20a_readl(g, gr_gpc0_tpc0_sm_tpc_esr_sm_sel_r() + offset);
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"sm tpc esr sm sel reg val: 0x%x", reg_val);
	*esr_sm_sel = 0;
	if (gr_gpc0_tpc0_sm_tpc_esr_sm_sel_sm0_error_v(reg_val))
		*esr_sm_sel = 1;
	if (gr_gpc0_tpc0_sm_tpc_esr_sm_sel_sm1_error_v(reg_val))
		*esr_sm_sel |= 1 << 1;
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"esr_sm_sel bitmask: 0x%x", *esr_sm_sel);
}

int gv11b_gr_sm_trigger_suspend(struct gk20a *g)
{
	u32 dbgr_control0;

	/* assert stop trigger. uniformity assumption: all SMs will have
	 * the same state in dbg_control0.
	 */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();

	/* broadcast write */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"stop trigger enable: broadcast dbgr_control0: 0x%x ",
			dbgr_control0);

	return 0;
}

void gv11b_gr_bpt_reg_info(struct gk20a *g, struct warpstate *w_state)
{
	/* Check if we have at least one valid warp
	 * get paused state on maxwell
	 */
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm, sm_id;
	u32 offset;
	u64 warps_valid = 0, warps_paused = 0, warps_trapped = 0;

	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {
		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
		sm = g->gr.sm_to_cluster[sm_id].sm_index;

		offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

		/* 64 bit read */
		warps_valid = (u64)gk20a_readl(g,
				gr_gpc0_tpc0_sm0_warp_valid_mask_1_r() +
				offset) << 32;
		warps_valid |= gk20a_readl(g,
				gr_gpc0_tpc0_sm0_warp_valid_mask_0_r() +
				offset);

		/* 64 bit read */
		warps_paused = (u64)gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_1_r() +
				offset) << 32;
		warps_paused |= gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_0_r() +
				offset);

		/* 64 bit read */
		warps_trapped = (u64)gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_1_r() +
				offset) << 32;
		warps_trapped |= gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_0_r() +
				offset);

		w_state[sm_id].valid_warps[0] = warps_valid;
		w_state[sm_id].trapped_warps[0] = warps_trapped;
		w_state[sm_id].paused_warps[0] = warps_paused;
	}


	/* Only for debug purpose */
	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {
		gk20a_dbg_fn("w_state[%d].valid_warps[0]: %llx\n",
					sm_id, w_state[sm_id].valid_warps[0]);
		gk20a_dbg_fn("w_state[%d].valid_warps[1]: %llx\n",
					sm_id, w_state[sm_id].valid_warps[1]);

		gk20a_dbg_fn("w_state[%d].trapped_warps[0]: %llx\n",
					sm_id, w_state[sm_id].trapped_warps[0]);
		gk20a_dbg_fn("w_state[%d].trapped_warps[1]: %llx\n",
					sm_id, w_state[sm_id].trapped_warps[1]);

		gk20a_dbg_fn("w_state[%d].paused_warps[0]: %llx\n",
					sm_id, w_state[sm_id].paused_warps[0]);
		gk20a_dbg_fn("w_state[%d].paused_warps[1]: %llx\n",
					sm_id, w_state[sm_id].paused_warps[1]);
	}
}

int gv11b_gr_update_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id,
		struct nvgpu_dbg_gpu_sm_error_state_record *sm_error_state)
{
	u32 gpc, tpc, sm, offset;
	struct gr_gk20a *gr = &g->gr;
	struct channel_ctx_gk20a *ch_ctx = &ch->ch_ctx;
	int err = 0;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	gr->sm_error_states[sm_id].hww_global_esr =
			sm_error_state->hww_global_esr;
	gr->sm_error_states[sm_id].hww_warp_esr =
			sm_error_state->hww_warp_esr;
	gr->sm_error_states[sm_id].hww_warp_esr_pc =
			sm_error_state->hww_warp_esr_pc;
	gr->sm_error_states[sm_id].hww_global_esr_report_mask =
			sm_error_state->hww_global_esr_report_mask;
	gr->sm_error_states[sm_id].hww_warp_esr_report_mask =
			sm_error_state->hww_warp_esr_report_mask;

	err = gr_gk20a_disable_ctxsw(g);
	if (err) {
		nvgpu_err(g, "unable to stop gr ctxsw");
		goto fail;
	}

	gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
	tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
	sm = g->gr.sm_to_cluster[sm_id].sm_index;

	offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

	if (gk20a_is_channel_ctx_resident(ch)) {
		gk20a_writel(g,
			gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
			gr->sm_error_states[sm_id].hww_global_esr);
		gk20a_writel(g,
			gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset,
			gr->sm_error_states[sm_id].hww_warp_esr);
		gk20a_writel(g,
			gr_gpc0_tpc0_sm0_hww_warp_esr_pc_r() + offset,
			gr->sm_error_states[sm_id].hww_warp_esr_pc);
		gk20a_writel(g,
			gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_r() + offset,
			gr->sm_error_states[sm_id].hww_global_esr_report_mask);
		gk20a_writel(g,
			gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_r() + offset,
			gr->sm_error_states[sm_id].hww_warp_esr_report_mask);
	} else {
		err = gr_gk20a_ctx_patch_write_begin(g, ch_ctx);
		if (err)
			goto enable_ctxsw;

		gr_gk20a_ctx_patch_write(g, ch_ctx,
			gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r() +
			offset,
			gr->sm_error_states[sm_id].hww_global_esr_report_mask,
			true);
		gr_gk20a_ctx_patch_write(g, ch_ctx,
			gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r() +
			offset,
			gr->sm_error_states[sm_id].hww_warp_esr_report_mask,
			true);

		gr_gk20a_ctx_patch_write_end(g, ch_ctx);
	}

enable_ctxsw:
	err = gr_gk20a_enable_ctxsw(g);

fail:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return err;
}

int gv11b_gr_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable)
{
	struct nvgpu_dbg_gpu_reg_op *ops;
	unsigned int i = 0, sm_id;
	int err;

	ops = nvgpu_kcalloc(g, g->gr.no_of_sm, sizeof(*ops));
	if (!ops)
		return -ENOMEM;
	for (sm_id = 0; sm_id < g->gr.no_of_sm; sm_id++) {
		u32 gpc, tpc, sm;
		u32 reg_offset, reg_mask, reg_val;

		if (!(sms & (1 << sm_id)))
			continue;

		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
		sm = g->gr.sm_to_cluster[sm_id].sm_index;

		reg_offset = gk20a_gr_gpc_offset(g, gpc) +
				gk20a_gr_tpc_offset(g, tpc) +
				gv11b_gr_sm_offset(g, sm);

		ops[i].op = REGOP(WRITE_32);
		ops[i].type = REGOP(TYPE_GR_CTX);
		ops[i].offset = gr_gpc0_tpc0_sm0_dbgr_control0_r() + reg_offset;

		reg_mask = 0;
		reg_val = 0;
		if (enable) {
			nvgpu_log(g, gpu_dbg_gpu_dbg,
				"SM:%d debuggger mode ON", sm);
			reg_mask |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_m();
			reg_val |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_on_f();
		} else {
			nvgpu_log(g, gpu_dbg_gpu_dbg,
				"SM:%d debuggger mode Off", sm);
			reg_mask |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_m();
			reg_val |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_off_f();
		}

		ops[i].and_n_mask_lo = reg_mask;
		ops[i].value_lo = reg_val;
		i++;
	}

	err = gr_gk20a_exec_ctx_ops(ch, ops, i, i, 0);
	if (err)
		nvgpu_err(g, "Failed to access register\n");
	nvgpu_kfree(g, ops);
	return err;
}

int gv11b_gr_record_sm_error_state(struct gk20a *g, u32 gpc, u32 tpc)
{
	int sm_id;
	struct gr_gk20a *gr = &g->gr;
	u32 offset, sm, sm_per_tpc;
	u32 gpc_tpc_offset;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	gpc_tpc_offset = gk20a_gr_gpc_offset(g, gpc) +
				 gk20a_gr_tpc_offset(g, tpc);

	sm_id = gr_gpc0_tpc0_sm_cfg_tpc_id_v(gk20a_readl(g,
			gr_gpc0_tpc0_sm_cfg_r() + gpc_tpc_offset));

	sm = sm_id % sm_per_tpc;

	offset = gpc_tpc_offset + gv11b_gr_sm_offset(g, sm);

	gr->sm_error_states[sm_id].hww_global_esr = gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset);

	gr->sm_error_states[sm_id].hww_warp_esr = gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset);

	gr->sm_error_states[sm_id].hww_warp_esr_pc = gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_pc_r() + offset);

	gr->sm_error_states[sm_id].hww_global_esr_report_mask = gk20a_readl(g,
	       gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_r() + offset);

	gr->sm_error_states[sm_id].hww_warp_esr_report_mask = gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_r() + offset);

	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return 0;
}

void gv11b_gr_set_hww_esr_report_mask(struct gk20a *g)
{

	/* clear hww */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_r(), 0xffffffff);
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_r(), 0xffffffff);

	/* setup sm warp esr report masks */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r(),
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_stack_error_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_api_stack_error_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_pc_wrap_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_misaligned_pc_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_pc_overflow_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_misaligned_reg_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_illegal_instr_encoding_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_illegal_instr_param_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_oor_reg_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_oor_addr_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_misaligned_addr_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_invalid_addr_space_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_invalid_const_addr_ldc_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_stack_overflow_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_mmu_fault_report_f());

	/* setup sm global esr report mask. vat_alarm_report is not enabled */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r(),
		gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_multiple_warp_errors_report_f());
}

bool gv11b_gr_sm_debugger_attached(struct gk20a *g)
{
	u32 debugger_mode;
	u32 dbgr_control0 = gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());

	/* check if sm debugger is attached.
	 * assumption: all SMs will have debug mode enabled/disabled
	 * uniformly.
	 */
	debugger_mode =
		gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_v(dbgr_control0);
	gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"SM Debugger Mode: %d", debugger_mode);
	if (debugger_mode ==
			gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_on_v())
		return true;

	return false;
}

void gv11b_gr_suspend_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors)
{
	int err;
	u32 dbgr_control0;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	/* if an SM debugger isn't attached, skip suspend */
	if (!g->ops.gr.sm_debugger_attached(g)) {
		nvgpu_err(g,
			"SM debugger not attached, skipping suspend!");
		return;
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
		"suspending gpc:%d, tpc:%d, sm%d", gpc, tpc, sm);

	/* assert stop trigger. */
	dbgr_control0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset,
			dbgr_control0);

	err = g->ops.gr.wait_for_sm_lock_down(g, gpc, tpc, sm,
			global_esr_mask, check_errors);
	if (err) {
		nvgpu_err(g,
			"SuspendSm failed");
		return;
	}
}

void gv11b_gr_suspend_all_sms(struct gk20a *g,
		u32 global_esr_mask, bool check_errors)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm;
	int err;
	u32 dbgr_control0;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	/* if an SM debugger isn't attached, skip suspend */
	if (!g->ops.gr.sm_debugger_attached(g)) {
		nvgpu_err(g,
			"SM debugger not attached, skipping suspend!");
		return;
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "suspending all sms");

	/* assert stop trigger. uniformity assumption: all SMs will have
	 * the same state in dbg_control0.
	 */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();

	/* broadcast write */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		for (tpc = 0; tpc < gr_gk20a_get_tpc_count(gr, gpc); tpc++) {
			for (sm = 0; sm < sm_per_tpc; sm++) {
				err = g->ops.gr.wait_for_sm_lock_down(g,
					gpc, tpc, sm,
					global_esr_mask, check_errors);
				if (err) {
					nvgpu_err(g,
						"SuspendAllSms failed");
					return;
				}
			}
		}
	}
}

void gv11b_gr_resume_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm)
{
	u32 dbgr_control0, dbgr_status0;
	u32 offset;
	/*
	 * The following requires some clarification. Despite the fact that both
	 * RUN_TRIGGER and STOP_TRIGGER have the word "TRIGGER" in their
	 *  names, only one is actually a trigger, and that is the STOP_TRIGGER.
	 * Merely writing a 1(_TASK) to the RUN_TRIGGER is not sufficient to
	 * resume the gpu - the _STOP_TRIGGER must explicitly be set to 0
	 * (_DISABLE) as well.

	* Advice from the arch group:  Disable the stop trigger first, as a
	* separate operation, in order to ensure that the trigger has taken
	* effect, before enabling the run trigger.
	*/

	offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"resuming gpc:%d, tpc:%d, sm%d", gpc, tpc, sm);
	dbgr_control0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_status0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before stop trigger disable: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);

	/*De-assert stop trigger */
	dbgr_control0 = set_field(dbgr_control0,
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_m(),
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_disable_f());
	gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() +
				offset, dbgr_control0);

	dbgr_control0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_status0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);
	/* Run trigger */
	dbgr_control0 |=
		gr_gpc0_tpc0_sm0_dbgr_control0_run_trigger_task_f();
	gk20a_writel(g,
		gr_gpc0_tpc0_sm0_dbgr_control0_r() +
		offset, dbgr_control0);

	dbgr_control0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_status0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);
	/* run trigger is not sticky bit. SM clears it immediately */
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"after run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);

}

void gv11b_gr_resume_all_sms(struct gk20a *g)
{
	u32 dbgr_control0, dbgr_status0;
	/*
	 * The following requires some clarification. Despite the fact that both
	 * RUN_TRIGGER and STOP_TRIGGER have the word "TRIGGER" in their
	 *  names, only one is actually a trigger, and that is the STOP_TRIGGER.
	 * Merely writing a 1(_TASK) to the RUN_TRIGGER is not sufficient to
	 * resume the gpu - the _STOP_TRIGGER must explicitly be set to 0
	 * (_DISABLE) as well.

	* Advice from the arch group:  Disable the stop trigger first, as a
	* separate operation, in order to ensure that the trigger has taken
	* effect, before enabling the run trigger.
	*/

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "resuming all sms");

	/* Read from unicast registers */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_status0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_status0_r());

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before stop trigger disable: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);

	dbgr_control0 = set_field(dbgr_control0,
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_m(),
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_disable_f());
	/* Write to broadcast registers */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	/* Read from unicast registers */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_status0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_status0_r());

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);
	/* Run trigger */
	dbgr_control0 |=
		gr_gpc0_tpc0_sm0_dbgr_control0_run_trigger_task_f();
	/* Write to broadcast registers */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	/* Read from unicast registers */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_status0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_status0_r());
	/* run trigger is not sticky bit. SM clears it immediately */
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"after run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);
}

int gv11b_gr_resume_from_pause(struct gk20a *g)
{
	int err = 0;
	u32 reg_val;

	/* Clear the pause mask to tell the GPU we want to resume everyone */
	gk20a_writel(g, gr_gpcs_tpcs_sms_dbgr_bpt_pause_mask_0_r(), 0);

	/* explicitly re-enable forwarding of SM interrupts upon any resume */
	reg_val = gk20a_readl(g, gr_gpc0_tpc0_tpccs_tpc_exception_en_r());
	reg_val |= gr_gpc0_tpc0_tpccs_tpc_exception_en_sm_enabled_f();

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(), reg_val);

	g->ops.gr.resume_all_sms(g);

	return err;
}

u32 gv11b_gr_get_sm_hww_warp_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

	u32 hww_warp_esr = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset);
	return hww_warp_esr;
}

u32 gv11b_gr_get_sm_hww_global_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

	u32 hww_global_esr = gk20a_readl(g,
				 gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset);

	return hww_global_esr;
}

u32 gv11b_gr_get_sm_no_lock_down_hww_global_esr_mask(struct gk20a *g)
{
	/*
	 * These three interrupts don't require locking down the SM. They can
	 * be handled by usermode clients as they aren't fatal. Additionally,
	 * usermode clients may wish to allow some warps to execute while others
	 * are at breakpoints, as opposed to fatal errors where all warps should
	 * halt.
	 */
	u32 global_esr_mask =
		gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f()   |
		gr_gpc0_tpc0_sm0_hww_global_esr_bpt_pause_pending_f() |
		gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f();

	return global_esr_mask;
}

static void gv11b_gr_sm_dump_warp_bpt_pause_trap_mask_regs(struct gk20a *g,
					u32 offset, bool timeout)
{
	u64 warps_valid = 0, warps_paused = 0, warps_trapped = 0;
	u32 dbgr_control0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	u32 dbgr_status0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);
	/* 64 bit read */
	warps_valid =
		(u64)gk20a_readl(g, gr_gpc0_tpc0_sm0_warp_valid_mask_1_r() +
					offset) << 32;
	warps_valid |= gk20a_readl(g,
			gr_gpc0_tpc0_sm0_warp_valid_mask_0_r() + offset);

	/* 64 bit read */
	warps_paused =
		(u64)gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_1_r() +
					offset) << 32;
	warps_paused |= gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_0_r() + offset);

	/* 64 bit read */
	warps_trapped =
		(u64)gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_1_r() +
					offset) << 32;
	warps_trapped |= gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_0_r() + offset);
	if (timeout)
		nvgpu_err(g,
		"STATUS0=0x%x CONTROL0=0x%x VALID_MASK=0x%llx "
		"PAUSE_MASK=0x%llx TRAP_MASK=0x%llx\n",
		dbgr_status0, dbgr_control0, warps_valid,
		warps_paused, warps_trapped);
	else
		gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
		"STATUS0=0x%x CONTROL0=0x%x VALID_MASK=0x%llx "
		"PAUSE_MASK=0x%llx TRAP_MASK=0x%llx\n",
		dbgr_status0, dbgr_control0, warps_valid,
		warps_paused, warps_trapped);
}

int gv11b_gr_wait_for_sm_lock_down(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors)
{
	bool locked_down;
	bool no_error_pending;
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	bool mmu_debug_mode_enabled = g->ops.fb.is_debug_mode_enabled(g);
	u32 dbgr_status0 = 0;
	u32 warp_esr, global_esr;
	struct nvgpu_timeout timeout;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
		"GPC%d TPC%d: locking down SM%d", gpc, tpc, sm);

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	/* wait for the sm to lock down */
	do {
		global_esr = g->ops.gr.get_sm_hww_global_esr(g, gpc, tpc, sm);
		dbgr_status0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);

		warp_esr = g->ops.gr.get_sm_hww_warp_esr(g, gpc, tpc, sm);

		locked_down =
		    (gr_gpc0_tpc0_sm0_dbgr_status0_locked_down_v(dbgr_status0) ==
		     gr_gpc0_tpc0_sm0_dbgr_status0_locked_down_true_v());
		no_error_pending =
			check_errors &&
			(gr_gpc0_tpc0_sm0_hww_warp_esr_error_v(warp_esr) ==
			 gr_gpc0_tpc0_sm0_hww_warp_esr_error_none_v()) &&
			((global_esr & global_esr_mask) == 0);

		if (locked_down) {
		/*
		 * if SM reports locked down, it means that SM is idle and
		 * trapped and also that one of the these conditions are true
		 * 1) sm is nonempty and all valid warps are paused
		 * 2) sm is empty and held in trapped state due to stop trigger
		 * 3) sm is nonempty and some warps are not paused, but are
		 *    instead held at RTT due to an "active" stop trigger
		 * Check for Paused warp mask != Valid
		 * warp mask after SM reports it is locked down in order to
		 * distinguish case 1 from case 3.  When case 3 is detected,
		 * it implies a misprogrammed trap handler code, as all warps
		 * in the handler must promise to BPT.PAUSE instead of RTT
		 * whenever SR64 read in trap mode indicates stop trigger
		 * is asserted.
		 */
			gv11b_gr_sm_dump_warp_bpt_pause_trap_mask_regs(g,
						offset, false);
		}

		if (locked_down || no_error_pending) {
			gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"GPC%d TPC%d: locked down SM%d", gpc, tpc, sm);
			return 0;
		}

		/* if an mmu fault is pending and mmu debug mode is not
		 * enabled, the sm will never lock down.
		 */
		if (!mmu_debug_mode_enabled &&
		     (g->ops.mm.mmu_fault_pending(g))) {
			nvgpu_err(g,
				"GPC%d TPC%d: mmu fault pending,"
				" SM%d will never lock down!", gpc, tpc, sm);
			return -EFAULT;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	nvgpu_err(g, "GPC%d TPC%d: timed out while trying to "
			"lock down SM%d", gpc, tpc, sm);
	gv11b_gr_sm_dump_warp_bpt_pause_trap_mask_regs(g, offset, true);

	return -ETIMEDOUT;
}

int gv11b_gr_lock_down_sm(struct gk20a *g,
			 u32 gpc, u32 tpc, u32 sm, u32 global_esr_mask,
			 bool check_errors)
{
	u32 dbgr_control0;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"GPC%d TPC%d SM%d: assert stop trigger", gpc, tpc, sm);

	/* assert stop trigger */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g,
		gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset, dbgr_control0);

	return g->ops.gr.wait_for_sm_lock_down(g, gpc, tpc, sm, global_esr_mask,
			check_errors);
}

void gv11b_gr_clear_sm_hww(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
				u32 global_esr)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
			global_esr);
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"Cleared HWW global esr, current reg val: 0x%x",
			gk20a_readl(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() +
						offset));

	gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset, 0);
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"Cleared HWW warp esr, current reg val: 0x%x",
			gk20a_readl(g, gr_gpc0_tpc0_sm0_hww_warp_esr_r() +
						offset));
}

int gr_gv11b_handle_tpc_mpc_exception(struct gk20a *g,
		u32 gpc, u32 tpc, bool *post_event)
{
	u32 esr;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	u32 tpc_exception = gk20a_readl(g, gr_gpc0_tpc0_tpccs_tpc_exception_r()
			+ offset);

	if (!(tpc_exception & gr_gpc0_tpc0_tpccs_tpc_exception_mpc_m()))
		return 0;

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"GPC%d TPC%d MPC exception", gpc, tpc);

	esr = gk20a_readl(g, gr_gpc0_tpc0_mpc_hww_esr_r() + offset);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, "mpc hww esr 0x%08x", esr);

	esr = gk20a_readl(g, gr_gpc0_tpc0_mpc_hww_esr_info_r() + offset);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"mpc hww esr info: veid 0x%08x",
			gr_gpc0_tpc0_mpc_hww_esr_info_veid_v(esr));

	gk20a_writel(g, gr_gpc0_tpc0_mpc_hww_esr_r() + offset,
		     gr_gpc0_tpc0_mpc_hww_esr_reset_trigger_f());

	return 0;
}

static const u32 _num_ovr_perf_regs = 20;
static u32 _ovr_perf_regs[20] = { 0, };

void gv11b_gr_init_ovr_sm_dsm_perf(void)
{
	if (_ovr_perf_regs[0] != 0)
		return;

	_ovr_perf_regs[0]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control_sel0_r();
	_ovr_perf_regs[1]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control_sel1_r();
	_ovr_perf_regs[2]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control0_r();
	_ovr_perf_regs[3]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control1_r();
	_ovr_perf_regs[4]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control2_r();
	_ovr_perf_regs[5]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control3_r();
	_ovr_perf_regs[6]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control4_r();
	_ovr_perf_regs[7]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control5_r();
	_ovr_perf_regs[8]  = gr_egpc0_etpc0_sm_dsm_perf_counter0_control_r();
	_ovr_perf_regs[9]  = gr_egpc0_etpc0_sm_dsm_perf_counter1_control_r();
	_ovr_perf_regs[10] = gr_egpc0_etpc0_sm_dsm_perf_counter2_control_r();
	_ovr_perf_regs[11] = gr_egpc0_etpc0_sm_dsm_perf_counter3_control_r();
	_ovr_perf_regs[12] = gr_egpc0_etpc0_sm_dsm_perf_counter4_control_r();
	_ovr_perf_regs[13] = gr_egpc0_etpc0_sm_dsm_perf_counter5_control_r();
	_ovr_perf_regs[14] = gr_egpc0_etpc0_sm_dsm_perf_counter6_control_r();
	_ovr_perf_regs[15] = gr_egpc0_etpc0_sm_dsm_perf_counter7_control_r();

	_ovr_perf_regs[16] = gr_egpc0_etpc0_sm0_dsm_perf_counter4_r();
	_ovr_perf_regs[17] = gr_egpc0_etpc0_sm0_dsm_perf_counter5_r();
	_ovr_perf_regs[18] = gr_egpc0_etpc0_sm0_dsm_perf_counter6_r();
	_ovr_perf_regs[19] = gr_egpc0_etpc0_sm0_dsm_perf_counter7_r();
}

/* Following are the blocks of registers that the ucode
 * stores in the extended region.
 */
/* ==  ctxsw_extended_sm_dsm_perf_counter_register_stride_v() ? */
static const u32 _num_sm_dsm_perf_regs;
/* ==  ctxsw_extended_sm_dsm_perf_counter_control_register_stride_v() ?*/
static const u32 _num_sm_dsm_perf_ctrl_regs = 2;
static u32 *_sm_dsm_perf_regs;
static u32 _sm_dsm_perf_ctrl_regs[2];

void gv11b_gr_init_sm_dsm_reg_info(void)
{
	if (_sm_dsm_perf_ctrl_regs[0] != 0)
		return;

	_sm_dsm_perf_ctrl_regs[0] =
			      gr_egpc0_etpc0_sm_dsm_perf_counter_control0_r();
	_sm_dsm_perf_ctrl_regs[1] =
			      gr_egpc0_etpc0_sm_dsm_perf_counter_control5_r();
}

void gv11b_gr_get_sm_dsm_perf_regs(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride)
{
	*num_sm_dsm_perf_regs = _num_sm_dsm_perf_regs;
	*sm_dsm_perf_regs = _sm_dsm_perf_regs;
	*perf_register_stride =
		ctxsw_prog_extended_sm_dsm_perf_counter_register_stride_v();
}

void gv11b_gr_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_ctrl_regs,
					       u32 **sm_dsm_perf_ctrl_regs,
					       u32 *ctrl_register_stride)
{
	*num_sm_dsm_perf_ctrl_regs = _num_sm_dsm_perf_ctrl_regs;
	*sm_dsm_perf_ctrl_regs = _sm_dsm_perf_ctrl_regs;
	*ctrl_register_stride =
		ctxsw_prog_extended_sm_dsm_perf_counter_control_register_stride_v();
}

void gv11b_gr_get_ovr_perf_regs(struct gk20a *g, u32 *num_ovr_perf_regs,
					       u32 **ovr_perf_regs)
{
	*num_ovr_perf_regs = _num_ovr_perf_regs;
	*ovr_perf_regs = _ovr_perf_regs;
}

void gv11b_gr_access_smpc_reg(struct gk20a *g, u32 quad, u32 offset)
{
	u32 reg_val;
	u32 quad_ctrl;
	u32 half_ctrl;
	u32 tpc, gpc;
	u32 gpc_tpc_addr;
	u32 gpc_tpc_stride;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "offset=0x%x", offset);

	gpc = pri_get_gpc_num(g, offset);
	gpc_tpc_addr = pri_gpccs_addr_mask(offset);
	tpc = g->ops.gr.get_tpc_num(g, gpc_tpc_addr);

	quad_ctrl = quad & 0x1; /* first bit tells us quad */
	half_ctrl = (quad >> 1) & 0x1; /* second bit tells us half */

	gpc_tpc_stride = gpc * gpc_stride + tpc * tpc_in_gpc_stride;
	gpc_tpc_addr = gr_gpc0_tpc0_sm_halfctl_ctrl_r() + gpc_tpc_stride;

	/* read from unicast reg */
	reg_val = gk20a_readl(g, gpc_tpc_addr);
	reg_val = set_field(reg_val,
		gr_gpcs_tpcs_sm_halfctl_ctrl_sctl_read_quad_ctl_m(),
		gr_gpcs_tpcs_sm_halfctl_ctrl_sctl_read_quad_ctl_f(quad_ctrl));

	/* write to broadcast reg */
	gk20a_writel(g, gr_gpcs_tpcs_sm_halfctl_ctrl_r(), reg_val);

	gpc_tpc_addr = gr_gpc0_tpc0_sm_debug_sfe_control_r() + gpc_tpc_stride;
	reg_val = gk20a_readl(g, gpc_tpc_addr);
	reg_val = set_field(reg_val,
		gr_gpcs_tpcs_sm_debug_sfe_control_read_half_ctl_m(),
		gr_gpcs_tpcs_sm_debug_sfe_control_read_half_ctl_f(half_ctrl));

	/* write to broadcast reg */
	gk20a_writel(g, gr_gpcs_tpcs_sm_debug_sfe_control_r(), reg_val);
}

static bool pri_is_egpc_addr_shared(struct gk20a *g, u32 addr)
{
	u32 egpc_shared_base = EGPC_PRI_SHARED_BASE;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	return (addr >= egpc_shared_base) &&
		(addr < egpc_shared_base + gpc_stride);
}

bool gv11b_gr_pri_is_egpc_addr(struct gk20a *g, u32 addr)
{
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);

	return	((addr >= egpc_base) &&
		 (addr < egpc_base + num_gpcs * gpc_stride)) ||
		pri_is_egpc_addr_shared(g, addr);
}

bool gv11b_gr_pri_is_etpc_addr(struct gk20a *g, u32 addr)
{
	u32 egpc_addr = 0;

	if (g->ops.gr.is_egpc_addr(g, addr)) {
		egpc_addr = pri_gpccs_addr_mask(addr);
		if (g->ops.gr.is_tpc_addr(g, egpc_addr))
			return true;
	}

	return false;
}

static u32 pri_get_egpc_num(struct gk20a *g, u32 addr)
{
	u32 i, start;
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	for (i = 0; i < num_gpcs; i++) {
		start = egpc_base + (i * gpc_stride);
		if ((addr >= start) && (addr < (start + gpc_stride)))
			return i;
	}
	return 0;
}

static u32 pri_egpc_addr(struct gk20a *g, u32 addr, u32 gpc)
{
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	return egpc_base + (gpc * gpc_stride) + addr;
}

static u32 pri_etpc_addr(struct gk20a *g, u32 addr, u32 gpc, u32 tpc)
{
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);

	return egpc_base + (gpc * gpc_stride) +
		tpc_in_gpc_base + (tpc * tpc_in_gpc_stride) +
		addr;
}

void gv11b_gr_get_egpc_etpc_num(struct gk20a *g, u32 addr,
			u32 *egpc_num, u32 *etpc_num)
{
	u32 egpc_addr = 0;

	*egpc_num = pri_get_egpc_num(g, addr);
	egpc_addr = pri_gpccs_addr_mask(addr);
	*etpc_num = g->ops.gr.get_tpc_num(g, egpc_addr);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"egpc_num = %d etpc_num = %d", *egpc_num, *etpc_num);
}

int gv11b_gr_decode_egpc_addr(struct gk20a *g, u32 addr, int *addr_type,
	u32 *gpc_num, u32 *tpc_num, u32 *broadcast_flags)
{
	u32 gpc_addr;

	if (g->ops.gr.is_egpc_addr(g, addr)) {
		nvgpu_log_info(g, "addr=0x%x is egpc", addr);

		*addr_type = CTXSW_ADDR_TYPE_EGPC;
		gpc_addr = pri_gpccs_addr_mask(addr);
		if (pri_is_egpc_addr_shared(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_EGPC;
			*gpc_num = 0;
			nvgpu_log_info(g, "shared egpc");
		} else {
			*gpc_num = pri_get_egpc_num(g, addr);
			nvgpu_log_info(g, "gpc=0x%x", *gpc_num);
		}
		if (g->ops.gr.is_tpc_addr(g, gpc_addr)) {
			nvgpu_log_info(g, "addr=0x%x is etpc", addr);
			*addr_type = CTXSW_ADDR_TYPE_ETPC;
			if (pri_is_tpc_addr_shared(g, gpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_ETPC;
				*tpc_num = 0;
				nvgpu_log_info(g, "shared etpc");
			} else {
				*tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
				nvgpu_log_info(g, "tpc=0x%x", *tpc_num);
			}
		}

		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"addr_type = %d, broadcast_flags = %#08x",
			*addr_type, *broadcast_flags);
		return 0;
	}
	return -EINVAL;
}

void gv11b_gr_egpc_etpc_priv_addr_table(struct gk20a *g, u32 addr,
		 u32 gpc, u32 broadcast_flags, u32 *priv_addr_table, u32 *t)
{
	u32 gpc_num, tpc_num;

	nvgpu_log_info(g, "addr=0x%x", addr);

	/* The GPC/TPC unicast registers are included in the compressed PRI
	 * tables. Convert a GPC/TPC broadcast address to unicast addresses so
	 * that we can look up the offsets.
	 */
	if (broadcast_flags & PRI_BROADCAST_FLAGS_EGPC) {
		nvgpu_log_info(g, "broadcast flags egpc");
		for (gpc_num = 0; gpc_num < g->gr.gpc_count; gpc_num++) {

			if (broadcast_flags & PRI_BROADCAST_FLAGS_ETPC) {
				nvgpu_log_info(g, "broadcast flags etpc");
				for (tpc_num = 0;
				     tpc_num < g->gr.gpc_tpc_count[gpc_num];
				     tpc_num++) {
					priv_addr_table[*t] =
						pri_etpc_addr(g,
						pri_tpccs_addr_mask(addr),
						     gpc_num, tpc_num);
					nvgpu_log_info(g,
					"priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
					(*t)++;
				}
			} else {
				priv_addr_table[*t] =
					pri_egpc_addr(g,
					pri_gpccs_addr_mask(addr),
					     gpc_num);
				nvgpu_log_info(g, "priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
				(*t)++;
			}
		}
	} else if (!(broadcast_flags & PRI_BROADCAST_FLAGS_EGPC)) {
		if (broadcast_flags & PRI_BROADCAST_FLAGS_ETPC) {
			nvgpu_log_info(g, "broadcast flags etpc but not egpc");
			for (tpc_num = 0;
			     tpc_num < g->gr.gpc_tpc_count[gpc];
			     tpc_num++) {
				priv_addr_table[*t] =
					pri_etpc_addr(g,
					pri_tpccs_addr_mask(addr),
					     gpc, tpc_num);
					nvgpu_log_info(g,
					"priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
					(*t)++;
			}
		} else {
			priv_addr_table[*t] = addr;
			nvgpu_log_info(g, "priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
			(*t)++;
		}
	}
}

u32 gv11b_gr_get_egpc_base(struct gk20a *g)
{
	return EGPC_PRI_BASE;
}

void gr_gv11b_init_gpc_mmu(struct gk20a *g)
{
	u32 temp;

	nvgpu_log_info(g, "initialize gpc mmu");

	if (!nvgpu_is_enabled(g, NVGPU_SEC_PRIVSECURITY)) {
		/* Bypass MMU check for non-secure boot. For
		 * secure-boot,this register write has no-effect */
		gk20a_writel(g, fb_priv_mmu_phy_secure_r(), 0xffffffff);
	}
	temp = gk20a_readl(g, fb_mmu_ctrl_r());
	temp &= gr_gpcs_pri_mmu_ctrl_vm_pg_size_m() |
		gr_gpcs_pri_mmu_ctrl_use_pdb_big_page_size_m() |
		gr_gpcs_pri_mmu_ctrl_vol_fault_m() |
		gr_gpcs_pri_mmu_ctrl_comp_fault_m() |
		gr_gpcs_pri_mmu_ctrl_miss_gran_m() |
		gr_gpcs_pri_mmu_ctrl_cache_mode_m() |
		gr_gpcs_pri_mmu_ctrl_mmu_aperture_m() |
		gr_gpcs_pri_mmu_ctrl_mmu_vol_m() |
		gr_gpcs_pri_mmu_ctrl_mmu_disable_m();
	gk20a_writel(g, gr_gpcs_pri_mmu_ctrl_r(), temp);
	gk20a_writel(g, gr_gpcs_pri_mmu_pm_unit_mask_r(), 0);
	gk20a_writel(g, gr_gpcs_pri_mmu_pm_req_mask_r(), 0);

	gk20a_writel(g, gr_gpcs_pri_mmu_debug_ctrl_r(),
			gk20a_readl(g, fb_mmu_debug_ctrl_r()));
	gk20a_writel(g, gr_gpcs_pri_mmu_debug_wr_r(),
			gk20a_readl(g, fb_mmu_debug_wr_r()));
	gk20a_writel(g, gr_gpcs_pri_mmu_debug_rd_r(),
			gk20a_readl(g, fb_mmu_debug_rd_r()));
}
