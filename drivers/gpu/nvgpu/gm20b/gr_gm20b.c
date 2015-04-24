/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/delay.h>	/* for mdelay */
#include <linux/io.h>
#include <linux/tegra-fuse.h>
#include <linux/vmalloc.h>

#include "gk20a/gk20a.h"
#include "gk20a/gr_gk20a.h"

#include "gr_gm20b.h"
#include "hw_gr_gm20b.h"
#include "hw_fifo_gm20b.h"
#include "hw_fb_gm20b.h"
#include "hw_top_gm20b.h"
#include "hw_proj_gm20b.h"
#include "hw_ctxsw_prog_gm20b.h"
#include "hw_fuse_gm20b.h"
#include "pmu_gm20b.h"
#include "acr_gm20b.h"

static void gr_gm20b_init_gpc_mmu(struct gk20a *g)
{
	u32 temp;

	gk20a_dbg_info("initialize gpc mmu");

	if (!g->ops.privsecurity) {
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

	gk20a_writel(g, gr_gpcs_mmu_num_active_ltcs_r(),
		gk20a_readl(g, fb_fbhub_num_active_ltcs_r()));
}

static void gr_gm20b_bundle_cb_defaults(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->bundle_cb_default_size =
		gr_scc_bundle_cb_size_div_256b__prod_v();
	gr->min_gpm_fifo_depth =
		gr_pd_ab_dist_cfg2_state_limit_min_gpm_fifo_depths_v();
	gr->bundle_cb_token_limit =
		gr_pd_ab_dist_cfg2_token_limit_init_v();
}

static void gr_gm20b_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->attrib_cb_default_size =
		gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v();
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
}

static int gr_gm20b_calc_global_ctx_buffer_size(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int size;

	gr->attrib_cb_size = gr->attrib_cb_default_size
		+ (gr->attrib_cb_default_size >> 1);
	gr->alpha_cb_size = gr->alpha_cb_default_size
		+ (gr->alpha_cb_default_size >> 1);

	size = gr->attrib_cb_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	size += gr->alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	return size;
}

static void gr_gk20a_commit_global_attrib_cb(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, bool patch)
{
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_setup_attrib_cb_base_r(),
		gr_gpcs_setup_attrib_cb_base_addr_39_12_f(addr) |
		gr_gpcs_setup_attrib_cb_base_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_pe_pin_cb_global_base_addr_r(),
		gr_gpcs_tpcs_pe_pin_cb_global_base_addr_v_f(addr) |
		gr_gpcs_tpcs_pe_pin_cb_global_base_addr_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_r(),
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_v_f(addr) |
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_valid_true_f(), patch);
}

static void gr_gm20b_commit_global_bundle_cb(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u64 size, bool patch)
{
	u32 data;

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_scc_bundle_cb_base_r(),
		gr_scc_bundle_cb_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_scc_bundle_cb_size_r(),
		gr_scc_bundle_cb_size_div_256b_f(size) |
		gr_scc_bundle_cb_size_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_swdx_bundle_cb_base_r(),
		gr_gpcs_swdx_bundle_cb_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_swdx_bundle_cb_size_r(),
		gr_gpcs_swdx_bundle_cb_size_div_256b_f(size) |
		gr_gpcs_swdx_bundle_cb_size_valid_true_f(), patch);

	/* data for state_limit */
	data = (g->gr.bundle_cb_default_size *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v()) /
		gr_pd_ab_dist_cfg2_state_limit_scc_bundle_granularity_v();

	data = min_t(u32, data, g->gr.min_gpm_fifo_depth);

	gk20a_dbg_info("bundle cb token limit : %d, state limit : %d",
		   g->gr.bundle_cb_token_limit, data);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg2_r(),
		gr_pd_ab_dist_cfg2_token_limit_f(g->gr.bundle_cb_token_limit) |
		gr_pd_ab_dist_cfg2_state_limit_f(data), patch);

}

static int gr_gm20b_commit_global_cb_manager(struct gk20a *g,
			struct channel_gk20a *c, bool patch)
{
	struct gr_gk20a *gr = &g->gr;
	struct channel_ctx_gk20a *ch_ctx = NULL;
	u32 attrib_offset_in_chunk = 0;
	u32 alpha_offset_in_chunk = 0;
	u32 pd_ab_max_output;
	u32 gpc_index, ppc_index;
	u32 temp;
	u32 cbm_cfg_size1, cbm_cfg_size2;

	gk20a_dbg_fn("");

	if (patch) {
		int err;
		ch_ctx = &c->ch_ctx;
		err = gr_gk20a_ctx_patch_write_begin(g, ch_ctx);
		if (err)
			return err;
	}

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_ds_tga_constraintlogic_r(),
		gr_ds_tga_constraintlogic_beta_cbsize_f(gr->attrib_cb_default_size) |
		gr_ds_tga_constraintlogic_alpha_cbsize_f(gr->alpha_cb_default_size),
		patch);

	pd_ab_max_output = (gr->alpha_cb_default_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v()) /
		gr_pd_ab_dist_cfg1_max_output_granularity_v();

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg1_r(),
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output) |
		gr_pd_ab_dist_cfg1_max_batches_init_f(), patch);

	alpha_offset_in_chunk = attrib_offset_in_chunk +
		gr->tpc_count * gr->attrib_cb_size;

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		temp = proj_gpc_stride_v() * gpc_index;
		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
		     ppc_index++) {
			cbm_cfg_size1 = gr->attrib_cb_default_size *
				gr->pes_tpc_count[ppc_index][gpc_index];
			cbm_cfg_size2 = gr->alpha_cb_default_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_cb_size_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				cbm_cfg_size1, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_cb_offset_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				attrib_offset_in_chunk, patch);

			attrib_offset_in_chunk += gr->attrib_cb_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_alpha_cb_size_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				cbm_cfg_size2, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_alpha_cb_offset_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				alpha_offset_in_chunk, patch);

			alpha_offset_in_chunk += gr->alpha_cb_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpcs_swdx_tc_beta_cb_size_r(ppc_index + gpc_index),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(cbm_cfg_size1) |
				gr_gpcs_swdx_tc_beta_cb_size_div3_f(cbm_cfg_size1/3),
				patch);
		}
	}

	if (patch)
		gr_gk20a_ctx_patch_write_end(g, ch_ctx);

	return 0;
}

static void gr_gm20b_commit_global_pagepool(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u32 size, bool patch)
{
	gr_gk20a_commit_global_pagepool(g, ch_ctx, addr, size, patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_swdx_rm_pagepool_r(),
		gr_gpcs_swdx_rm_pagepool_total_pages_f(size) |
		gr_gpcs_swdx_rm_pagepool_valid_true_f(), patch);

}

static int gr_gm20b_handle_sw_method(struct gk20a *g, u32 addr,
					  u32 class_num, u32 offset, u32 data)
{
	gk20a_dbg_fn("");

	if (class_num == MAXWELL_COMPUTE_B) {
		switch (offset << 2) {
		case NVB1C0_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == MAXWELL_B) {
		switch (offset << 2) {
		case NVB197_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		case NVB197_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVB197_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_alpha_circular_buffer_size(g, data);
			break;
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

static void gr_gm20b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 pd_ab_max_output;
	u32 alpha_cb_size = data * 4;

	gk20a_dbg_fn("");
	/* if (NO_ALPHA_BETA_TIMESLICE_SUPPORT_DEF)
		return; */

	if (alpha_cb_size > gr->alpha_cb_size)
		alpha_cb_size = gr->alpha_cb_size;

	gk20a_writel(g, gr_ds_tga_constraintlogic_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_r()) &
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

static void gr_gm20b_set_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 cb_size = data * 4;

	gk20a_dbg_fn("");

	if (cb_size > gr->attrib_cb_size)
		cb_size = gr->attrib_cb_size;

	gk20a_writel(g, gr_ds_tga_constraintlogic_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_r()) &
		 ~gr_ds_tga_constraintlogic_beta_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_beta_cbsize_f(cb_size));

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

			val = gk20a_readl(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index));

			val = set_field(val,
				gr_gpcs_swdx_tc_beta_cb_size_v_m(),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(cb_size *
					gr->gpc_ppc_count[gpc_index]));
			val = set_field(val,
				gr_gpcs_swdx_tc_beta_cb_size_div3_m(),
				gr_gpcs_swdx_tc_beta_cb_size_div3_f((cb_size *
					gr->gpc_ppc_count[gpc_index])/3));

			gk20a_writel(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index), val);
		}
	}
}

static void gr_gm20b_set_hww_esr_report_mask(struct gk20a *g)
{
	/* setup sm warp esr report masks */
	gk20a_writel(g, gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_r(),
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_stack_error_report_f()	|
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_api_stack_error_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_ret_empty_stack_error_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_pc_wrap_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_pc_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_pc_overflow_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_immc_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_reg_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_encoding_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_sph_instr_combo_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_param_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_const_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_oor_reg_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_oor_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_addr_space_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_param2_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_const_addr_ldc_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_mmu_fault_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_stack_overflow_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_geometry_sm_error_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_divergent_report_f());

	/* setup sm global esr report mask */
	gk20a_writel(g, gr_gpcs_tpcs_sm_hww_global_esr_report_mask_r(),
		gr_gpcs_tpcs_sm_hww_global_esr_report_mask_sm_to_sm_fault_report_f() |
		gr_gpcs_tpcs_sm_hww_global_esr_report_mask_multiple_warp_errors_report_f());
}

static bool gr_gm20b_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case MAXWELL_COMPUTE_B:
	case MAXWELL_B:
	case FERMI_TWOD_A:
	case KEPLER_DMA_COPY_A:
	case MAXWELL_DMA_COPY_A:
		valid = true;
		break;

	default:
		break;
	}

	return valid;
}

/* Following are the blocks of registers that the ucode
 stores in the extended region.*/
/* ==  ctxsw_extended_sm_dsm_perf_counter_register_stride_v() ? */
static const u32 _num_sm_dsm_perf_regs;
/* ==  ctxsw_extended_sm_dsm_perf_counter_control_register_stride_v() ?*/
static const u32 _num_sm_dsm_perf_ctrl_regs = 2;
static u32 *_sm_dsm_perf_regs;
static u32 _sm_dsm_perf_ctrl_regs[2];

static void gr_gm20b_init_sm_dsm_reg_info(void)
{
	if (_sm_dsm_perf_ctrl_regs[0] != 0)
		return;

	_sm_dsm_perf_ctrl_regs[0] =
			      gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control0_r();
	_sm_dsm_perf_ctrl_regs[1] =
			      gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control5_r();
}

static void gr_gm20b_get_sm_dsm_perf_regs(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride)
{
	*num_sm_dsm_perf_regs = _num_sm_dsm_perf_regs;
	*sm_dsm_perf_regs = _sm_dsm_perf_regs;
	*perf_register_stride = 0;
}

static void gr_gm20b_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_ctrl_regs,
					       u32 **sm_dsm_perf_ctrl_regs,
					       u32 *ctrl_register_stride)
{
	*num_sm_dsm_perf_ctrl_regs = _num_sm_dsm_perf_ctrl_regs;
	*sm_dsm_perf_ctrl_regs = _sm_dsm_perf_ctrl_regs;

	*ctrl_register_stride =
	    ctxsw_prog_extended_sm_dsm_perf_counter_control_register_stride_v();
}

static u32 gr_gm20b_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	u32 val;
	struct gr_gk20a *gr = &g->gr;

	/* Toggle the bits of NV_FUSE_STATUS_OPT_TPC_GPC */
	val = gk20a_readl(g, fuse_status_opt_tpc_gpc_r(gpc_index));

	return (~val) & ((0x1 << gr->max_tpc_per_gpc_count) - 1);
}

static void gr_gm20b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	tegra_clk_writel(CLK_RST_CONTROLLER_MISC_CLK_ENB_0_ALL_VISIBLE,
			CLK_RST_CONTROLLER_MISC_CLK_ENB_0);

	tegra_fuse_writel(0x1, FUSE_FUSEBYPASS_0);
	tegra_fuse_writel(0x0, FUSE_WRITE_ACCESS_SW_0);

	if (g->gr.gpc_tpc_mask[gpc_index] == 0x1) {
		tegra_fuse_writel(0x0, FUSE_OPT_GPU_TPC0_DISABLE_0);
		tegra_fuse_writel(0x1, FUSE_OPT_GPU_TPC1_DISABLE_0);
	} else if (g->gr.gpc_tpc_mask[gpc_index] == 0x2) {
		tegra_fuse_writel(0x1, FUSE_OPT_GPU_TPC0_DISABLE_0);
		tegra_fuse_writel(0x0, FUSE_OPT_GPU_TPC1_DISABLE_0);
	} else {
		tegra_fuse_writel(0x0, FUSE_OPT_GPU_TPC0_DISABLE_0);
		tegra_fuse_writel(0x0, FUSE_OPT_GPU_TPC1_DISABLE_0);
	}
}

static int gr_gm20b_ctx_state_floorsweep(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_index, gpc_index;
	u32 tpc_offset, gpc_offset;
	u32 sm_id = 0;
	u32 tpc_per_gpc = 0;
	u32 tpc_sm_id = 0, gpc_tpc_id = 0;
	u32 pes_tpc_mask = 0, pes_index;

	gk20a_dbg_fn("");

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		gpc_offset = proj_gpc_stride_v() * gpc_index;
		for (tpc_index = 0; tpc_index < gr->gpc_tpc_count[gpc_index];
								tpc_index++) {
			tpc_offset = proj_tpc_in_gpc_stride_v() * tpc_index;

			gk20a_writel(g, gr_gpc0_tpc0_sm_cfg_r()
					+ gpc_offset + tpc_offset,
				gr_gpc0_tpc0_sm_cfg_sm_id_f(sm_id));
			gk20a_writel(g, gr_gpc0_gpm_pd_sm_id_r(tpc_index)
					+ gpc_offset,
				gr_gpc0_gpm_pd_sm_id_id_f(sm_id));
			gk20a_writel(g, gr_gpc0_tpc0_pe_cfg_smid_r()
					+ gpc_offset + tpc_offset,
				gr_gpc0_tpc0_pe_cfg_smid_value_f(sm_id));

			g->gr.sm_to_cluster[sm_id].tpc_index = tpc_index;
			g->gr.sm_to_cluster[sm_id].gpc_index = gpc_index;

			sm_id++;
		}
	}

	gr->no_of_sm = sm_id;

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++)
		tpc_per_gpc |= gr->gpc_tpc_count[gpc_index]
			     << (gr_pd_num_tpc_per_gpc__size_1_v() * gpc_index);
	gk20a_writel(g, gr_pd_num_tpc_per_gpc_r(0), tpc_per_gpc);
	gk20a_writel(g, gr_ds_num_tpc_per_gpc_r(0), tpc_per_gpc);

	/* gr__setup_pd_mapping stubbed for gk20a */
	gr_gk20a_setup_rop_mapping(g, gr);

	for (gpc_index = 0;
	     gpc_index < gr_pd_dist_skip_table__size_1_v() * 4;
	     gpc_index += 4) {

		gk20a_writel(g, gr_pd_dist_skip_table_r(gpc_index/4),
			     gr_pd_dist_skip_table_gpc_4n0_mask_f(gr->gpc_skip_mask[gpc_index]) ||
			     gr_pd_dist_skip_table_gpc_4n1_mask_f(gr->gpc_skip_mask[gpc_index + 1]) ||
			     gr_pd_dist_skip_table_gpc_4n2_mask_f(gr->gpc_skip_mask[gpc_index + 2]) ||
			     gr_pd_dist_skip_table_gpc_4n3_mask_f(gr->gpc_skip_mask[gpc_index + 3]));
	}

	gk20a_writel(g, gr_cwd_fs_r(),
		     gr_cwd_fs_num_gpcs_f(gr->gpc_count) |
		     gr_cwd_fs_num_tpcs_f(gr->tpc_count));

	gk20a_writel(g, gr_bes_zrop_settings_r(),
		     gr_bes_zrop_settings_num_active_ltcs_f(gr->num_fbps));
	gk20a_writel(g, gr_bes_crop_settings_r(),
		     gr_bes_crop_settings_num_active_ltcs_f(gr->num_fbps));

	gk20a_writel(g, gr_bes_crop_debug3_r(),
		     gk20a_readl(g, gr_be0_crop_debug3_r()) |
		     gr_bes_crop_debug3_comp_vdc_4to2_disable_m());

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++)
		for (pes_index = 0; pes_index < gr->pe_count_per_gpc;
								pes_index++)
			pes_tpc_mask |= gr->pes_tpc_mask[pes_index][gpc_index];
	gk20a_writel(g, gr_fe_tpc_fs_r(), pes_tpc_mask);

	for (tpc_index = 0; tpc_index < gr->tpc_count; tpc_index++) {
		if (tpc_index == 0) {
			gpc_tpc_id |= gr_cwd_gpc_tpc_id_tpc0_f(tpc_index);
			tpc_sm_id |= gr_cwd_sm_id_tpc0_f(tpc_index);
		} else if (tpc_index == 1) {
			gpc_tpc_id |= gr_cwd_gpc_tpc_id_tpc1_f(tpc_index);
			tpc_sm_id |= gr_cwd_sm_id_tpc1_f(tpc_index);
		}
	}

	/* Each NV_PGRAPH_PRI_CWD_GPC_TPC_ID can store 4 TPCs.
	 * Since we know TPC number is less than 5. We select
	 * index 0 directly. */
	gk20a_writel(g, gr_cwd_gpc_tpc_id_r(0), gpc_tpc_id);

	gk20a_writel(g, gr_cwd_sm_id_r(0), tpc_sm_id);

	return 0;
}

static int gr_gm20b_load_ctxsw_ucode_segments(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset)
{
	gk20a_writel(g, reg_offset + gr_fecs_dmactl_r(),
			gr_fecs_dmactl_require_ctx_f(0));

	/* Copy falcon bootloader into dmem */
	gr_gk20a_load_ctxsw_ucode_header(g, addr_base, segments, reg_offset);
	gr_gk20a_load_ctxsw_ucode_boot(g, addr_base, segments, reg_offset);

	/* start the falcon immediately if PRIV security is disabled*/
	if (!g->ops.privsecurity) {
		gk20a_writel(g, reg_offset + gr_fecs_cpuctl_r(),
				gr_fecs_cpuctl_startcpu_f(0x01));
	}

	return 0;
}

static bool gr_gm20b_is_tpc_addr_shared(u32 addr)
{
	return (addr >= proj_tpc_in_gpc_shared_base_v()) &&
		(addr < (proj_tpc_in_gpc_shared_base_v() +
			 proj_tpc_in_gpc_stride_v()));
}

static bool gr_gm20b_is_tpc_addr(u32 addr)
{
	return ((addr >= proj_tpc_in_gpc_base_v()) &&
		(addr < proj_tpc_in_gpc_base_v() +
		 (proj_scal_litter_num_tpc_per_gpc_v() *
		  proj_tpc_in_gpc_stride_v())))
		|| gr_gm20b_is_tpc_addr_shared(addr);
}

static u32 gr_gm20b_get_tpc_num(u32 addr)
{
	u32 i, start;
	u32 num_tpcs = proj_scal_litter_num_tpc_per_gpc_v();

	for (i = 0; i < num_tpcs; i++) {
		start = proj_tpc_in_gpc_base_v() +
			(i * proj_tpc_in_gpc_stride_v());
		if ((addr >= start) &&
		    (addr < (start + proj_tpc_in_gpc_stride_v())))
			return i;
	}
	return 0;
}

#ifdef CONFIG_TEGRA_ACR
static void gr_gm20b_load_gpccs_with_bootloader(struct gk20a *g)
{
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	u64 addr_base = ucode_info->surface_desc.gpu_va;

	gr_gk20a_load_falcon_bind_instblk(g);

	g->ops.gr.falcon_load_ucode(g, addr_base,
		&g->ctxsw_ucode_info.gpccs,
		gr_gpcs_gpccs_falcon_hwcfg_r() -
		gr_fecs_falcon_hwcfg_r());
}

static int gr_gm20b_ctx_wait_lsf_ready(struct gk20a *g, u32 timeout, u32 val)
{
	unsigned long end_jiffies = jiffies + msecs_to_jiffies(timeout);
	unsigned long delay = GR_FECS_POLL_INTERVAL;
	u32 reg;

	gk20a_dbg_fn("");
	reg = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
	do {
		reg = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
		if (reg == val)
			return 0;
		udelay(delay);
	} while (time_before(jiffies, end_jiffies) ||
			!tegra_platform_is_silicon());

	return -ETIMEDOUT;
}

static int gr_gm20b_load_ctxsw_ucode(struct gk20a *g)
{
	u32 err;
	gk20a_dbg_fn("");

	if (tegra_platform_is_linsim()) {
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(7),
			gr_fecs_ctxsw_mailbox_value_f(0xc0de7777));
		gk20a_writel(g, gr_gpccs_ctxsw_mailbox_r(7),
			gr_gpccs_ctxsw_mailbox_value_f(0xc0de7777));
	}

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), ~0x0);
	gm20b_pmu_load_lsf(g, LSF_FALCON_ID_FECS);

	gr_gm20b_load_gpccs_with_bootloader(g);

	if (g->ops.pmu.fecsrecoveryinprogress) {
		unsigned long timeout = gk20a_get_gr_idle_timeout(g);
		err = gr_gm20b_ctx_wait_lsf_ready(g, timeout, 0x55AA55AA);
		if (err) {
			gk20a_err(dev_from_gk20a(g), "Unable to recover FECS");
			return err;
		} else {
			g->ops.pmu.fecsrecoveryinprogress = 0;
			gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), ~0x0);
			gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(1), 0x1);
			gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(6),
					0xffffffff);

			gk20a_writel(g, gr_gpccs_dmactl_r(),
					gr_gpccs_dmactl_require_ctx_f(0));
			gk20a_writel(g, gr_gpccs_cpuctl_r(),
					gr_gpccs_cpuctl_startcpu_f(1));

			gk20a_writel(g, gr_fecs_cpuctl_alias_r(),
					gr_fecs_cpuctl_startcpu_f(1));
		}
	}


	if (!g->ops.pmu.fecsbootstrapdone) {
		g->ops.pmu.fecsbootstrapdone = true;
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), ~0x0);
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(1), 0x1);
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(6), 0xffffffff);

		gk20a_writel(g, gr_gpccs_dmactl_r(),
				gr_gpccs_dmactl_require_ctx_f(0));
		gk20a_writel(g, gr_gpccs_cpuctl_r(),
				gr_gpccs_cpuctl_startcpu_f(1));

		gk20a_writel(g, gr_fecs_cpuctl_alias_r(),
				gr_fecs_cpuctl_startcpu_f(1));
	}

	gk20a_dbg_fn("done");

	return 0;
}
#else

static int gr_gm20b_load_ctxsw_ucode(struct gk20a *g)
{
	return -EPERM;
}

#endif

static void gr_gm20b_detect_sm_arch(struct gk20a *g)
{
	u32 v = gk20a_readl(g, gr_gpc0_tpc0_sm_arch_r());

	g->gpu_characteristics.sm_arch_spa_version =
		gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	g->gpu_characteristics.sm_arch_sm_version =
		gr_gpc0_tpc0_sm_arch_sm_version_v(v);
	g->gpu_characteristics.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

static u32 gr_gm20b_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

static int gr_gm20b_alloc_gr_ctx(struct gk20a *g,
			  struct gr_ctx_desc **gr_ctx, struct vm_gk20a *vm,
			  u32 class,
			  u32 flags)
{
	int err;

	gk20a_dbg_fn("");

	err = gr_gk20a_alloc_gr_ctx(g, gr_ctx, vm, class, flags);
	if (err)
		return err;

	if (class == MAXWELL_COMPUTE_B)
		(*gr_ctx)->preempt_mode = NVGPU_GR_PREEMPTION_MODE_CTA;

	gk20a_dbg_fn("done");

	return 0;
}

static void gr_gm20b_update_ctxsw_preemption_mode(struct gk20a *g,
		struct channel_ctx_gk20a *ch_ctx,
		void *ctx_ptr)
{
	struct gr_ctx_desc *gr_ctx = ch_ctx->gr_ctx;
	u32 cta_preempt_option =
		ctxsw_prog_main_image_preemption_options_control_cta_enabled_f();

	gk20a_dbg_fn("");

	if (gr_ctx->preempt_mode == NVGPU_GR_PREEMPTION_MODE_CTA) {
		gk20a_dbg_info("CTA: %x", cta_preempt_option);
		gk20a_mem_wr32(ctx_ptr + ctxsw_prog_main_image_preemption_options_o(), 0,
				cta_preempt_option);
	}

	gk20a_dbg_fn("done");
}

static int gr_gm20b_dump_gr_status_regs(struct gk20a *g,
			   struct gk20a_debug_output *o)
{
	struct gr_gk20a *gr = &g->gr;

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
		gk20a_readl(g, fifo_engine_status_r(ENGINE_GR_GK20A)));
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
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_ON_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_on_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_TIMEOUT : 0x%x\n",
		gk20a_readl(g, gr_fe_go_idle_timeout_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_CHECK : 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_check_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_INFO : 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_info_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TEX_M_TEX_SUBUNITS_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tex_m_tex_subunits_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_FS: 0x%x\n",
		gk20a_readl(g, gr_cwd_fs_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_TPC_FS: 0x%x\n",
		gk20a_readl(g, gr_fe_tpc_fs_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_GPC_TPC_ID(0): 0x%x\n",
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

	return 0;
}

static int gr_gm20b_update_pc_sampling(struct channel_gk20a *c,
				       bool enable)
{
	struct channel_ctx_gk20a *ch_ctx = &c->ch_ctx;
	void *ctx_ptr = NULL;
	u32 v;

	gk20a_dbg_fn("");

	if (!ch_ctx || !ch_ctx->gr_ctx || c->vpr)
		return -EINVAL;

	ctx_ptr = vmap(ch_ctx->gr_ctx->mem.pages,
			PAGE_ALIGN(ch_ctx->gr_ctx->mem.size) >> PAGE_SHIFT,
			0, pgprot_writecombine(PAGE_KERNEL));
	if (!ctx_ptr)
		return -ENOMEM;

	v = gk20a_mem_rd32(ctx_ptr + ctxsw_prog_main_image_pm_o(), 0);
	v &= ~ctxsw_prog_main_image_pm_pc_sampling_m();
	v |= ctxsw_prog_main_image_pm_pc_sampling_f(enable);
	gk20a_mem_wr32(ctx_ptr + ctxsw_prog_main_image_pm_o(), 0, v);

	vunmap(ctx_ptr);

	gk20a_dbg_fn("done");

	return 0;
}

static u32 gr_gm20b_get_fbp_en_mask(struct gk20a *g)
{
	u32 fbp_en_mask, opt_fbio;
	opt_fbio = gk20a_readl(g,  fuse_status_opt_fbio_r());
	fbp_en_mask = fuse_status_opt_fbio_data_v(opt_fbio);
	return fbp_en_mask;
}

static u32 gr_gm20b_get_max_ltc_per_fbp(struct gk20a *g)
{
	u32 ltc_per_fbp, reg;
	reg = gk20a_readl(g,  top_ltc_per_fbp_r());
	ltc_per_fbp = top_ltc_per_fbp_value_v(reg);
	return ltc_per_fbp;
}

static u32 gr_gm20b_get_max_lts_per_ltc(struct gk20a *g)
{
	u32 lts_per_ltc, reg;
	reg = gk20a_readl(g,  top_slices_per_ltc_r());
	lts_per_ltc = top_slices_per_ltc_value_v(reg);
	return lts_per_ltc;
}

static u32 *gr_gm20b_rop_l2_en_mask(struct gk20a *g)
{
	struct nvgpu_gpu_characteristics *gpu = &g->gpu_characteristics;
	u32 i, tmp, max_fbps_count;
	tmp = gk20a_readl(g, top_num_fbps_r());
	max_fbps_count = top_num_fbps_value_v(tmp);

	/* mask of Rop_L2 for each FBP */
	for (i = 0; i < max_fbps_count; i++)
		gpu->rop_l2_en_mask[i] = fuse_status_opt_rop_l2_fbp_r(i);

	return gpu->rop_l2_en_mask;
}

static u32 gr_gm20b_get_max_fbps_count(struct gk20a *g)
{
	u32 tmp, max_fbps_count;
	tmp = gk20a_readl(g, top_num_fbps_r());
	max_fbps_count = top_num_fbps_value_v(tmp);
	return max_fbps_count;
}

void gm20b_init_gr(struct gpu_ops *gops)
{
	gops->gr.init_gpc_mmu = gr_gm20b_init_gpc_mmu;
	gops->gr.bundle_cb_defaults = gr_gm20b_bundle_cb_defaults;
	gops->gr.cb_size_default = gr_gm20b_cb_size_default;
	gops->gr.calc_global_ctx_buffer_size =
		gr_gm20b_calc_global_ctx_buffer_size;
	gops->gr.commit_global_attrib_cb = gr_gk20a_commit_global_attrib_cb;
	gops->gr.commit_global_bundle_cb = gr_gm20b_commit_global_bundle_cb;
	gops->gr.commit_global_cb_manager = gr_gm20b_commit_global_cb_manager;
	gops->gr.commit_global_pagepool = gr_gm20b_commit_global_pagepool;
	gops->gr.handle_sw_method = gr_gm20b_handle_sw_method;
	gops->gr.set_alpha_circular_buffer_size = gr_gm20b_set_alpha_circular_buffer_size;
	gops->gr.set_circular_buffer_size = gr_gm20b_set_circular_buffer_size;
	gops->gr.enable_hww_exceptions = gr_gk20a_enable_hww_exceptions;
	gops->gr.is_valid_class = gr_gm20b_is_valid_class;
	gops->gr.get_sm_dsm_perf_regs = gr_gm20b_get_sm_dsm_perf_regs;
	gops->gr.get_sm_dsm_perf_ctrl_regs = gr_gm20b_get_sm_dsm_perf_ctrl_regs;
	gops->gr.init_fs_state = gr_gm20b_ctx_state_floorsweep;
	gops->gr.set_hww_esr_report_mask = gr_gm20b_set_hww_esr_report_mask;
	gops->gr.falcon_load_ucode = gr_gm20b_load_ctxsw_ucode_segments;
	if (gops->privsecurity)
		gops->gr.load_ctxsw_ucode = gr_gm20b_load_ctxsw_ucode;
	else
		gops->gr.load_ctxsw_ucode = gr_gk20a_load_ctxsw_ucode;
	gops->gr.set_gpc_tpc_mask = gr_gm20b_set_gpc_tpc_mask;
	gops->gr.get_gpc_tpc_mask = gr_gm20b_get_gpc_tpc_mask;
	gops->gr.free_channel_ctx = gk20a_free_channel_ctx;
	gops->gr.alloc_obj_ctx = gk20a_alloc_obj_ctx;
	gops->gr.free_obj_ctx = gk20a_free_obj_ctx;
	gops->gr.bind_ctxsw_zcull = gr_gk20a_bind_ctxsw_zcull;
	gops->gr.get_zcull_info = gr_gk20a_get_zcull_info;
	gops->gr.is_tpc_addr = gr_gm20b_is_tpc_addr;
	gops->gr.get_tpc_num = gr_gm20b_get_tpc_num;
	gops->gr.detect_sm_arch = gr_gm20b_detect_sm_arch;
	gops->gr.add_zbc_color = gr_gk20a_add_zbc_color;
	gops->gr.add_zbc_depth = gr_gk20a_add_zbc_depth;
	gops->gr.pagepool_default_size = gr_gm20b_pagepool_default_size;
	gops->gr.init_ctx_state = gr_gk20a_init_ctx_state;
	gops->gr.alloc_gr_ctx = gr_gm20b_alloc_gr_ctx;
	gops->gr.free_gr_ctx = gr_gk20a_free_gr_ctx;
	gops->gr.update_ctxsw_preemption_mode =
		gr_gm20b_update_ctxsw_preemption_mode;
	gops->gr.dump_gr_regs = gr_gm20b_dump_gr_status_regs;
	gops->gr.update_pc_sampling = gr_gm20b_update_pc_sampling;
	gops->gr.get_fbp_en_mask = gr_gm20b_get_fbp_en_mask;
	gops->gr.get_max_ltc_per_fbp = gr_gm20b_get_max_ltc_per_fbp;
	gops->gr.get_max_lts_per_ltc = gr_gm20b_get_max_lts_per_ltc;
	gops->gr.get_rop_l2_en_mask = gr_gm20b_rop_l2_en_mask;
	gops->gr.get_max_fbps_count = gr_gm20b_get_max_fbps_count;
	gops->gr.init_sm_dsm_reg_info = gr_gm20b_init_sm_dsm_reg_info;
}
