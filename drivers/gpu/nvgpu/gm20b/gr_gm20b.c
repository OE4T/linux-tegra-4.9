/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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
#include "gk20a/gr_gk20a.h"

#include "gr_gm20b.h"
#include "hw_gr_gm20b.h"
#include "hw_fb_gm20b.h"
#include "hw_proj_gm20b.h"
#include "hw_ctxsw_prog_gm20b.h"
#include "hw_fuse_gm20b.h"

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
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output));

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

void gr_gm20b_set_circular_buffer_size(struct gk20a *g, u32 data)
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

static void gr_gm20b_enable_hww_exceptions(struct gk20a *g)
{
	gr_gk20a_enable_hww_exceptions(g);

	gk20a_writel(g, gr_ds_hww_esr_2_r(),
			gr_ds_hww_esr_2_en_enabled_f() |
			gr_ds_hww_esr_2_reset_task_f());
	gk20a_writel(g, gr_ds_hww_report_mask_2_r(),
			gr_ds_hww_report_mask_2_sph24_err_report_f());
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

static void gr_gm20b_get_sm_dsm_perf_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_regs,
					       u32 **sm_dsm_perf_regs,
					       u32 *perf_register_stride)
{
	gr_gk20a_get_sm_dsm_perf_regs(g, num_sm_dsm_perf_regs,
				      sm_dsm_perf_regs,
				      perf_register_stride);
	*perf_register_stride = ctxsw_prog_extended_sm_dsm_perf_counter_register_stride_v();
}

static void gr_gm20b_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_regs,
					       u32 **sm_dsm_perf_regs,
					       u32 *ctrl_register_stride)
{
	gr_gk20a_get_sm_dsm_perf_ctrl_regs(g, num_sm_dsm_perf_regs,
					   sm_dsm_perf_regs,
					   ctrl_register_stride);
	*ctrl_register_stride = ctxsw_prog_extended_sm_dsm_perf_counter_control_register_stride_v();
}

static u32 gr_gm20b_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	u32 val;
	struct gr_gk20a *gr = &g->gr;

	/* Toggle the bits of NV_FUSE_STATUS_OPT_TPC_GPC */
	val = gk20a_readl(g, fuse_status_opt_tpc_gpc_r(gpc_index));

	return (~val) & ((0x1 << gr->max_tpc_per_gpc_count) - 1);
}

static int gr_gm20b_ctx_state_floorsweep(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_index, gpc_index;
	u32 tpc_offset, gpc_offset;
	u32 sm_id = 0, gpc_id = 0;
	u32 sm_id_to_gpc_id[proj_scal_max_gpcs_v() * proj_scal_max_tpc_per_gpc_v()];
	u32 tpc_per_gpc;
	u32 tpc_fs_mask = 0, tpc_sm_id, gpc_tpc_id;

	gk20a_dbg_fn("");

	for (tpc_index = 0; tpc_index < gr->max_tpc_per_gpc_count; tpc_index++) {
		for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
			gpc_offset = proj_gpc_stride_v() * gpc_index;
			if (tpc_index < gr->gpc_tpc_count[gpc_index]) {
				tpc_offset = proj_tpc_in_gpc_stride_v() * tpc_index;

				gk20a_writel(g, gr_gpc0_tpc0_sm_cfg_r() + gpc_offset + tpc_offset,
					     gr_gpc0_tpc0_sm_cfg_sm_id_f(sm_id));
				gk20a_writel(g, gr_gpc0_gpm_pd_sm_id_r(tpc_index) + gpc_offset,
					     gr_gpc0_gpm_pd_sm_id_id_f(sm_id));
				gk20a_writel(g, gr_gpc0_tpc0_pe_cfg_smid_r() + gpc_offset + tpc_offset,
					     gr_gpc0_tpc0_pe_cfg_smid_value_f(sm_id));

				sm_id_to_gpc_id[sm_id] = gpc_index;
				sm_id++;
			}
		}
	}

	for (tpc_index = 0, gpc_id = 0;
	     tpc_index < gr_pd_num_tpc_per_gpc__size_1_v();
	     tpc_index++, gpc_id += 8) {

		if (gpc_id >= gr->gpc_count)
			gpc_id = 0;

		tpc_per_gpc =
			gr_pd_num_tpc_per_gpc_count0_f(gr->gpc_tpc_count[gpc_id + 0]) |
			gr_pd_num_tpc_per_gpc_count1_f(gr->gpc_tpc_count[gpc_id + 1]) |
			gr_pd_num_tpc_per_gpc_count2_f(gr->gpc_tpc_count[gpc_id + 2]) |
			gr_pd_num_tpc_per_gpc_count3_f(gr->gpc_tpc_count[gpc_id + 3]) |
			gr_pd_num_tpc_per_gpc_count4_f(gr->gpc_tpc_count[gpc_id + 4]) |
			gr_pd_num_tpc_per_gpc_count5_f(gr->gpc_tpc_count[gpc_id + 5]) |
			gr_pd_num_tpc_per_gpc_count6_f(gr->gpc_tpc_count[gpc_id + 6]) |
			gr_pd_num_tpc_per_gpc_count7_f(gr->gpc_tpc_count[gpc_id + 7]);

		gk20a_writel(g, gr_pd_num_tpc_per_gpc_r(tpc_index), tpc_per_gpc);
		gk20a_writel(g, gr_ds_num_tpc_per_gpc_r(tpc_index), tpc_per_gpc);
	}

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

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		tpc_fs_mask |= gr->gpc_tpc_mask[gpc_index] <<
				(gr->max_tpc_per_gpc_count * gpc_index);
	}
	gk20a_writel(g, gr_fe_tpc_fs_r(), tpc_fs_mask);

	if (tpc_fs_mask & (0x1 << 0)) {
		tpc_sm_id |= gr_cwd_sm_id_tpc0_f(0);
		gpc_tpc_id |= gr_cwd_gpc_tpc_id_tpc0_f(0);
	}
	if (tpc_fs_mask & (0x1 << 1)) {
		gpc_tpc_id |= gr_cwd_gpc_tpc_id_tpc1_f(1);
		tpc_sm_id |= gr_cwd_sm_id_tpc1_f(1);
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
	u32 addr_code32;
	u32 addr_data32;
	u32 addr_load32;
	u32 dst = 0;
	u32 blocks;
	u32 b;

	addr_code32 = u64_lo32((addr_base + segments->code.offset) >> 8);
	addr_data32 = u64_lo32((addr_base + segments->data.offset) >> 8);
	addr_load32 = u64_lo32((addr_base + segments->boot.offset) >> 8);

	gk20a_writel(g, reg_offset + gr_fecs_dmactl_r(),
			gr_fecs_dmactl_require_ctx_f(0));

	/*
	 * Copy falcon bootloader header into dmem at offset 0.
	 * Configure dmem port 0 for auto-incrementing writes starting at dmem
	 * offset 0.
	 */
	gk20a_writel(g, reg_offset + gr_fecs_dmemc_r(0),
			gr_fecs_dmemc_offs_f(0) |
			gr_fecs_dmemc_blk_f(0) |
			gr_fecs_dmemc_aincw_f(1));

	/* Write out the actual data */
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 4);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), addr_code32);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), segments->code.size);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), 0);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), addr_data32);
	gk20a_writel(g, reg_offset + gr_fecs_dmemd_r(0), segments->data.size);

	blocks = ((segments->boot.size + 0xFF) & ~0xFF) >> 8;

	/*
	 * Set the base FB address for the DMA transfer. Subtract off the 256
	 * byte IMEM block offset such that the relative FB and IMEM offsets
	 * match, allowing the IMEM tags to be properly created.
	 */

	dst = segments->boot_imem_offset;
	gk20a_writel(g, reg_offset + gr_fecs_dmatrfbase_r(),
			(addr_load32 - (dst >> 8)));

	for (b = 0; b < blocks; b++) {
		/* Setup destination IMEM offset */
		gk20a_writel(g, reg_offset + gr_fecs_dmatrfmoffs_r(),
				dst + (b << 8));

		/* Setup source offset (relative to BASE) */
		gk20a_writel(g, reg_offset + gr_fecs_dmatrffboffs_r(),
				dst + (b << 8));

		gk20a_writel(g, reg_offset + gr_fecs_dmatrfcmd_r(),
				gr_fecs_dmatrfcmd_imem_f(0x01) |
				gr_fecs_dmatrfcmd_write_f(0x00) |
				gr_fecs_dmatrfcmd_size_f(0x06) |
				gr_fecs_dmatrfcmd_ctxdma_f(0));
	}

	/* Specify the falcon boot vector */
	gk20a_writel(g, reg_offset + gr_fecs_bootvec_r(),
			gr_fecs_bootvec_vec_f(segments->boot_entry));

	/* Write to CPUCTL to start the falcon */
	gk20a_writel(g, reg_offset + gr_fecs_cpuctl_r(),
			gr_fecs_cpuctl_startcpu_f(0x01));

	return 0;
}

#ifdef CONFIG_TEGRA_ACR
static void gr_gm20b_load_gpccs_with_bootloader(struct gk20a *g)
{
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	u64 addr_base = ucode_info->ucode_gpuva;

	gr_gk20a_load_falcon_bind_instblk(g);

	g->ops.gr.falcon_load_ucode(g, addr_base,
		&g->ctxsw_ucode_info.gpccs,
		gr_gpcs_gpccs_falcon_hwcfg_r() -
		gr_fecs_falcon_hwcfg_r());
}

static int gr_gm20b_load_ctxsw_ucode(struct gk20a *g)
{
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	u64 addr_base = ucode_info->ucode_gpuva;
	int i;

	gk20a_dbg_fn("");

	if (tegra_platform_is_linsim()) {
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(7),
			gr_fecs_ctxsw_mailbox_value_f(0xc0de7777));
		gk20a_writel(g, gr_gpccs_ctxsw_mailbox_r(7),
			gr_gpccs_ctxsw_mailbox_value_f(0xc0de7777));
	}

	gr_gk20a_load_falcon_bind_instblk(g);
	g->ops.gr.falcon_load_ucode(g, addr_base,
		&g->ctxsw_ucode_info.gpccs,
		gr_gpcs_gpccs_falcon_hwcfg_r() -
		gr_fecs_falcon_hwcfg_r());

	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), 0x0);
	gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(1), 0x1);
	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(6), 0xffffffff);

	gk20a_writel(g, gr_gpccs_dmactl_r(), gr_gpccs_dmactl_require_ctx_f(0));

	gk20a_writel(g, gr_gpccs_cpuctl_r(), gr_gpccs_cpuctl_startcpu_f(1));
	gk20a_writel(g, gr_fecs_cpuctl_alias_r(), gr_fecs_cpuctl_startcpu_f(1));

	gk20a_dbg_fn("done");

	return 0;
}
#else

static int gr_gm20b_load_ctxsw_ucode(struct gk20a *g)
{
	return -EPERM;
}

#endif

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
	gops->gr.enable_hww_exceptions = gr_gm20b_enable_hww_exceptions;
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
	gops->gr.get_gpc_tpc_mask = gr_gm20b_get_gpc_tpc_mask;
	gops->gr.free_channel_ctx = gk20a_free_channel_ctx;
	gops->gr.alloc_obj_ctx = gk20a_alloc_obj_ctx;
	gops->gr.free_obj_ctx = gk20a_free_obj_ctx;
	gops->gr.bind_ctxsw_zcull = gr_gk20a_bind_ctxsw_zcull;
	gops->gr.get_zcull_info = gr_gk20a_get_zcull_info;
}
