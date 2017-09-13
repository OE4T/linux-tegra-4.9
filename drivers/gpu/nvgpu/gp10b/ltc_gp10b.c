/*
 * GP10B L2
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <dt-bindings/memory/tegra-swgroup.h>

#include "gk20a/gk20a.h"
#include "gm20b/ltc_gm20b.h"

#include <nvgpu/log.h>
#include <nvgpu/enabled.h>

#include <nvgpu/hw/gp10b/hw_mc_gp10b.h>
#include <nvgpu/hw/gp10b/hw_ltc_gp10b.h>

#include "gk20a/ltc_gk20a.h"
#include "ltc_gp10b.h"

int gp10b_determine_L2_size_bytes(struct gk20a *g)
{
	u32 tmp;
	int ret;

	gk20a_dbg_fn("");

	tmp = gk20a_readl(g, ltc_ltc0_lts0_tstg_info_1_r());

	ret = g->ltc_count *
		ltc_ltc0_lts0_tstg_info_1_slice_size_in_kb_v(tmp)*1024 *
		ltc_ltc0_lts0_tstg_info_1_slices_per_l2_v(tmp);

	gk20a_dbg(gpu_dbg_info, "L2 size: %d\n", ret);

	gk20a_dbg_fn("done");

	return ret;
}

int gp10b_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	/* max memory size (MB) to cover */
	u32 max_size = gr->max_comptag_mem;
	/* one tag line covers 64KB */
	u32 max_comptag_lines = max_size << 4;

	u32 hw_max_comptag_lines =
		ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_init_v();

	u32 cbc_param =
		gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r());
	u32 comptags_per_cacheline =
		ltc_ltcs_ltss_cbc_param_comptags_per_cache_line_v(cbc_param);
	u32 cacheline_size =
		512 << ltc_ltcs_ltss_cbc_param_cache_line_size_v(cbc_param);
	u32 slices_per_ltc =
		ltc_ltcs_ltss_cbc_param_slices_per_ltc_v(cbc_param);
	u32 cbc_param2 =
		gk20a_readl(g, ltc_ltcs_ltss_cbc_param2_r());
	u32 gobs_per_comptagline_per_slice =
		ltc_ltcs_ltss_cbc_param2_gobs_per_comptagline_per_slice_v(cbc_param2);

	u32 compbit_backing_size;

	int err;

	gk20a_dbg_fn("");

	if (max_comptag_lines == 0)
		return 0;

	if (max_comptag_lines > hw_max_comptag_lines)
		max_comptag_lines = hw_max_comptag_lines;

	compbit_backing_size =
		roundup(max_comptag_lines * gobs_per_comptagline_per_slice,
			cacheline_size);
	compbit_backing_size =
		roundup(compbit_backing_size * slices_per_ltc * g->ltc_count,
			g->ops.fb.compressible_page_size(g));

	/* aligned to 2KB * ltc_count */
	compbit_backing_size +=
		g->ltc_count << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	/* must be a multiple of 64KB */
	compbit_backing_size = roundup(compbit_backing_size, 64*1024);

	gk20a_dbg_info("compbit backing store size : %d",
		compbit_backing_size);
	gk20a_dbg_info("max comptag lines : %d",
		max_comptag_lines);
	gk20a_dbg_info("gobs_per_comptagline_per_slice: %d",
		gobs_per_comptagline_per_slice);

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL))
		err = gk20a_ltc_alloc_phys_cbc(g, compbit_backing_size);
	else
		err = gk20a_ltc_alloc_virt_cbc(g, compbit_backing_size);

	if (err)
		return err;

	err = gk20a_comptag_allocator_init(&gr->comp_tags, max_comptag_lines);
	if (err)
		return err;

	gr->comptags_per_cacheline = comptags_per_cacheline;
	gr->slices_per_ltc = slices_per_ltc;
	gr->cacheline_size = cacheline_size;
	gr->gobs_per_comptagline_per_slice = gobs_per_comptagline_per_slice;

	return 0;
}

void gp10b_ltc_isr(struct gk20a *g)
{
	u32 mc_intr, ltc_intr;
	unsigned int ltc, slice;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	mc_intr = gk20a_readl(g, mc_intr_ltc_r());
	nvgpu_err(g, "mc_ltc_intr: %08x", mc_intr);
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		if ((mc_intr & 1 << ltc) == 0)
			continue;
		for (slice = 0; slice < g->gr.slices_per_ltc; slice++) {
			u32 offset = ltc_stride * ltc + lts_stride * slice;
			ltc_intr = gk20a_readl(g, ltc_ltc0_lts0_intr_r() + offset);

			/* Detect and handle ECC errors */
			if (ltc_intr &
				ltc_ltcs_ltss_intr_ecc_sec_error_pending_f()) {
				u32 ecc_stats_reg_val;

				nvgpu_err(g,
					"Single bit error detected in GPU L2!");

				ecc_stats_reg_val =
					gk20a_readl(g,
						ltc_ltc0_lts0_dstg_ecc_report_r() + offset);
				g->ecc.gr.t18x.l2_sec_count.counters[ltc] +=
					ltc_ltc0_lts0_dstg_ecc_report_sec_count_v(ecc_stats_reg_val);
				ecc_stats_reg_val &=
					~(ltc_ltc0_lts0_dstg_ecc_report_sec_count_m());
				gk20a_writel(g,
					ltc_ltc0_lts0_dstg_ecc_report_r() + offset,
					ecc_stats_reg_val);

				g->ops.mm.l2_flush(g, true);
			}
			if (ltc_intr &
				ltc_ltcs_ltss_intr_ecc_ded_error_pending_f()) {
				u32 ecc_stats_reg_val;

				nvgpu_err(g,
					"Double bit error detected in GPU L2!");

				ecc_stats_reg_val =
					gk20a_readl(g,
						ltc_ltc0_lts0_dstg_ecc_report_r() + offset);
				g->ecc.gr.t18x.l2_ded_count.counters[ltc] +=
					ltc_ltc0_lts0_dstg_ecc_report_ded_count_v(ecc_stats_reg_val);
				ecc_stats_reg_val &=
					~(ltc_ltc0_lts0_dstg_ecc_report_ded_count_m());
				gk20a_writel(g,
					ltc_ltc0_lts0_dstg_ecc_report_r() + offset,
					ecc_stats_reg_val);
			}

			nvgpu_err(g, "ltc%d, slice %d: %08x",
				  ltc, slice, ltc_intr);
			gk20a_writel(g, ltc_ltc0_lts0_intr_r() +
					   ltc_stride * ltc + lts_stride * slice,
				     ltc_intr);
		}
	}
}

void gp10b_ltc_init_fs_state(struct gk20a *g)
{
	u32 ltc_intr;

	gm20b_ltc_init_fs_state(g);

	gk20a_writel(g, ltc_ltca_g_axi_pctrl_r(),
			ltc_ltca_g_axi_pctrl_user_sid_f(TEGRA_SID_GPUB));

	/* Enable ECC interrupts */
	ltc_intr = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	ltc_intr |= ltc_ltcs_ltss_intr_en_ecc_sec_error_enabled_f() |
			ltc_ltcs_ltss_intr_en_ecc_ded_error_enabled_f();
	gk20a_writel(g, ltc_ltcs_ltss_intr_r(),
			ltc_intr);
}

void gp10b_ltc_set_enabled(struct gk20a *g, bool enabled)
{
	u32 reg_f = ltc_ltcs_ltss_tstg_set_mgmt_2_l2_bypass_mode_enabled_f();
	u32 reg = gk20a_readl(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r());

	if (enabled)
		/* bypass disabled (normal caching ops)*/
		reg &= ~reg_f;
	else
		/* bypass enabled (no caching) */
		reg |= reg_f;

	gk20a_writel(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r(), reg);
}
