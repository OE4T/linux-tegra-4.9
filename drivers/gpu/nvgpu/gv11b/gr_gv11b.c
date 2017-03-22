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

#include "gk20a/gk20a.h"
#include "gk20a/gr_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"

#include "gm20b/gr_gm20b.h"

#include "gv11b/gr_gv11b.h"
#include "gv11b/mm_gv11b.h"
#include "gv11b/subctx_gv11b.h"

#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_proj_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_therm_gv11b.h>

static bool gr_gv11b_is_valid_class(struct gk20a *g, u32 class_num)
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

static int gr_gv11b_handle_sm_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	int ret = 0;
	u32 offset = proj_gpc_stride_v() * gpc +
			proj_tpc_in_gpc_stride_v() * tpc;
	u32 lrf_ecc_status;

	gr_gk20a_handle_sm_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for LRF ECC errors. */
	lrf_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_r() + offset);

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_lrf_ecc_status_r() + offset,
			lrf_ecc_status);
	return ret;
}

static void gr_gv11b_enable_gpc_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_mask;

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(),
			gr_gpcs_tpcs_tpccs_tpc_exception_en_sm_enabled_f());

	tpc_mask =
		gr_gpcs_gpccs_gpc_exception_en_tpc_f((1 << gr->tpc_count) - 1);

	gk20a_writel(g, gr_gpcs_gpccs_gpc_exception_en_r(), tpc_mask);
}

static int gr_gv11b_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event)
{
	return 0;
}

static int gr_gv11b_commit_global_cb_manager(struct gk20a *g,
			struct channel_gk20a *c, bool patch)
{
	struct gr_gk20a *gr = &g->gr;
	struct channel_ctx_gk20a *ch_ctx = &c->ch_ctx;
	struct gr_ctx_desc *gr_ctx = ch_ctx->gr_ctx;
	u32 attrib_offset_in_chunk = 0;
	u32 alpha_offset_in_chunk = 0;
	u32 pd_ab_max_output;
	u32 gpc_index, ppc_index;
	u32 temp, temp2;
	u32 cbm_cfg_size_beta, cbm_cfg_size_alpha, cbm_cfg_size_steadystate;
	u32 attrib_size_in_chunk, cb_attrib_cache_size_init;

	gk20a_dbg_fn("");

	if (gr_ctx->graphics_preempt_mode == NVGPU_GRAPHICS_PREEMPTION_MODE_GFXP) {
		attrib_size_in_chunk = gr->attrib_cb_default_size +
				  (gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v() -
				   gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v());
		cb_attrib_cache_size_init = gr->attrib_cb_default_size +
				  (gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v() -
				   gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v());
	} else {
		attrib_size_in_chunk = gr->attrib_cb_size;
		cb_attrib_cache_size_init = gr->attrib_cb_default_size;
	}

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_ds_tga_constraintlogic_beta_r(),
		gr->attrib_cb_default_size, patch);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_ds_tga_constraintlogic_alpha_r(),
		gr->alpha_cb_default_size, patch);

	pd_ab_max_output = (gr->alpha_cb_default_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v()) /
		gr_pd_ab_dist_cfg1_max_output_granularity_v();

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg1_r(),
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output) |
		gr_pd_ab_dist_cfg1_max_batches_init_f(), patch);

	attrib_offset_in_chunk = alpha_offset_in_chunk +
		gr->tpc_count * gr->alpha_cb_size;

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		temp = proj_gpc_stride_v() * gpc_index;
		temp2 = proj_scal_litter_num_pes_per_gpc_v() * gpc_index;
		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
		     ppc_index++) {
			cbm_cfg_size_beta = cb_attrib_cache_size_init *
				gr->pes_tpc_count[ppc_index][gpc_index];
			cbm_cfg_size_alpha = gr->alpha_cb_default_size *
				gr->pes_tpc_count[ppc_index][gpc_index];
			cbm_cfg_size_steadystate = gr->attrib_cb_default_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_cb_size_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				cbm_cfg_size_beta, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_cb_offset_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				attrib_offset_in_chunk, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				cbm_cfg_size_steadystate,
				patch);

			attrib_offset_in_chunk += attrib_size_in_chunk *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_alpha_cb_size_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				cbm_cfg_size_alpha, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_alpha_cb_offset_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				alpha_offset_in_chunk, patch);

			alpha_offset_in_chunk += gr->alpha_cb_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpcs_swdx_tc_beta_cb_size_r(ppc_index + temp2),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(cbm_cfg_size_steadystate),
				patch);
		}
	}

	return 0;
}

static void gr_gv11b_commit_global_pagepool(struct gk20a *g,
					    struct channel_ctx_gk20a *ch_ctx,
					    u64 addr, u32 size, bool patch)
{
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_scc_pagepool_base_r(),
		gr_scc_pagepool_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_scc_pagepool_r(),
		gr_scc_pagepool_total_pages_f(size) |
		gr_scc_pagepool_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_gcc_pagepool_base_r(),
		gr_gpcs_gcc_pagepool_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_gcc_pagepool_r(),
		gr_gpcs_gcc_pagepool_total_pages_f(size), patch);
}

static int gr_gv11b_zbc_s_query_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params)
{
	u32 index = query_params->index_size;

	if (index >= GK20A_ZBC_TABLE_SIZE) {
		gk20a_err(dev_from_gk20a(g),
			"invalid zbc stencil table index\n");
		return -EINVAL;
	}
	query_params->depth = gr->zbc_s_tbl[index].stencil;
	query_params->format = gr->zbc_s_tbl[index].format;
	query_params->ref_cnt = gr->zbc_s_tbl[index].ref_cnt;

	return 0;
}

static bool gr_gv11b_add_zbc_type_s(struct gk20a *g, struct gr_gk20a *gr,
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

static int gr_gv11b_add_zbc_stencil(struct gk20a *g, struct gr_gk20a *gr,
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

static int gr_gv11b_load_stencil_default_tbl(struct gk20a *g,
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
		gk20a_err(dev_from_gk20a(g),
			   "fail to load default zbc stencil table\n");
		return err;
	}

	return 0;
}

static int gr_gv11b_load_stencil_tbl(struct gk20a *g, struct gr_gk20a *gr)
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

static u32 gr_gv11b_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

static int gr_gv11b_calc_global_ctx_buffer_size(struct gk20a *g)
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


static void gv11b_gr_set_shader_exceptions(struct gk20a *g, u32 data)
{
	u32 val;

	gk20a_dbg_fn("");

	if (data == NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE)
		val = 0;
	else
		val = 0xffffffff;

	/* setup sm warp esr report masks */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r(), val);

	/* setup sm global esr report mask */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r(), val);
}

static int gr_gv11b_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data)
{
	gk20a_dbg_fn("");

	if (class_num == VOLTA_COMPUTE_A) {
		switch (offset << 2) {
		case NVC0C0_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
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
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

static void gr_gv11b_bundle_cb_defaults(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->bundle_cb_default_size =
		gr_scc_bundle_cb_size_div_256b__prod_v();
	gr->min_gpm_fifo_depth =
		gr_pd_ab_dist_cfg2_state_limit_min_gpm_fifo_depths_v();
	gr->bundle_cb_token_limit =
		gr_pd_ab_dist_cfg2_token_limit_init_v();
}

static void gr_gv11b_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (!gr->attrib_cb_default_size)
		gr->attrib_cb_default_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v();
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
}

static void gr_gv11b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data)
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

static void gr_gv11b_set_circular_buffer_size(struct gk20a *g, u32 data)
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

static int gr_gv11b_init_ctx_state(struct gk20a *g)
{
	struct fecs_method_op_gk20a op = {
		.mailbox = { .id = 0, .data = 0,
			     .clr = ~0, .ok = 0, .fail = 0},
		.method.data = 0,
		.cond.ok = GR_IS_UCODE_OP_NOT_EQUAL,
		.cond.fail = GR_IS_UCODE_OP_SKIP,
		};
	int err;

	gk20a_dbg_fn("");

	err = gr_gk20a_init_ctx_state(g);
	if (err)
		return err;

	if (!g->gr.t18x.ctx_vars.preempt_image_size) {
		op.method.addr =
			gr_fecs_method_push_adr_discover_preemption_image_size_v();
		op.mailbox.ret = &g->gr.t18x.ctx_vars.preempt_image_size;
		err = gr_gk20a_submit_fecs_method_op(g, op, false);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
					"query preempt image size failed");
			return err;
		}
	}

	gk20a_dbg_info("preempt image size: %u",
		g->gr.t18x.ctx_vars.preempt_image_size);

	gk20a_dbg_fn("done");

	return 0;
}

int gr_gv11b_alloc_buffer(struct vm_gk20a *vm, size_t size,
			struct nvgpu_mem *mem)
{
	int err;

	gk20a_dbg_fn("");

	err = gk20a_gmmu_alloc_sys(vm->mm->g, size, mem);
	if (err)
		return err;

	mem->gpu_va = gk20a_gmmu_map(vm,
				&mem->sgt,
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
	gk20a_gmmu_free(vm->mm->g, mem);
	return err;
}

static int gr_gv11b_alloc_gr_ctx(struct gk20a *g,
			  struct gr_ctx_desc **gr_ctx, struct vm_gk20a *vm,
			  u32 class,
			  u32 flags)
{
	int err;

	gk20a_dbg_fn("");

	err = gr_gk20a_alloc_gr_ctx(g, gr_ctx, vm, class, flags);
	if (err)
		return err;

	(*gr_ctx)->t18x.ctx_id_valid = false;

	if (class == PASCAL_A && g->gr.t18x.ctx_vars.force_preemption_gfxp)
		flags |= NVGPU_ALLOC_OBJ_FLAGS_GFXP;

	if (class == PASCAL_COMPUTE_A &&
			g->gr.t18x.ctx_vars.force_preemption_cilp)
		flags |= NVGPU_ALLOC_OBJ_FLAGS_CILP;

	if (flags & NVGPU_ALLOC_OBJ_FLAGS_GFXP) {
		u32 spill_size =
			gr_gpc0_swdx_rm_spill_buffer_size_256b_default_v() *
			gr_gpc0_swdx_rm_spill_buffer_size_256b_byte_granularity_v();
		u32 pagepool_size = g->ops.gr.pagepool_default_size(g) *
			gr_scc_pagepool_total_pages_byte_granularity_v();
		u32 betacb_size = g->gr.attrib_cb_default_size +
				  (gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v() -
				   gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v());
		u32 attrib_cb_size = (betacb_size + g->gr.alpha_cb_size) *
				  gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v() *
				  g->gr.max_tpc_count;
		attrib_cb_size = ALIGN(attrib_cb_size, 128);

		gk20a_dbg_info("gfxp context spill_size=%d", spill_size);
		gk20a_dbg_info("gfxp context pagepool_size=%d", pagepool_size);
		gk20a_dbg_info("gfxp context attrib_cb_size=%d",
				attrib_cb_size);
		err = gr_gv11b_alloc_buffer(vm,
					g->gr.t18x.ctx_vars.preempt_image_size,
					&(*gr_ctx)->t18x.preempt_ctxsw_buffer);
		if (err) {
			gk20a_err(dev_from_gk20a(vm->mm->g),
				  "cannot allocate preempt buffer");
			goto fail_free_gk20a_ctx;
		}

		err = gr_gv11b_alloc_buffer(vm,
					spill_size,
					&(*gr_ctx)->t18x.spill_ctxsw_buffer);
		if (err) {
			gk20a_err(dev_from_gk20a(vm->mm->g),
				  "cannot allocate spill buffer");
			goto fail_free_preempt;
		}

		err = gr_gv11b_alloc_buffer(vm,
					attrib_cb_size,
					&(*gr_ctx)->t18x.betacb_ctxsw_buffer);
		if (err) {
			gk20a_err(dev_from_gk20a(vm->mm->g),
				  "cannot allocate beta buffer");
			goto fail_free_spill;
		}

		err = gr_gv11b_alloc_buffer(vm,
					pagepool_size,
					&(*gr_ctx)->t18x.pagepool_ctxsw_buffer);
		if (err) {
			gk20a_err(dev_from_gk20a(vm->mm->g),
				  "cannot allocate page pool");
			goto fail_free_betacb;
		}

		(*gr_ctx)->graphics_preempt_mode = NVGPU_GRAPHICS_PREEMPTION_MODE_GFXP;
	}

	if (class == PASCAL_COMPUTE_A) {
		if (flags & NVGPU_ALLOC_OBJ_FLAGS_CILP)
			(*gr_ctx)->compute_preempt_mode = NVGPU_COMPUTE_PREEMPTION_MODE_CILP;
		else
			(*gr_ctx)->compute_preempt_mode = NVGPU_COMPUTE_PREEMPTION_MODE_CTA;
	}

	gk20a_dbg_fn("done");

	return err;

fail_free_betacb:
	gk20a_gmmu_unmap_free(vm, &(*gr_ctx)->t18x.betacb_ctxsw_buffer);
fail_free_spill:
	gk20a_gmmu_unmap_free(vm, &(*gr_ctx)->t18x.spill_ctxsw_buffer);
fail_free_preempt:
	gk20a_gmmu_unmap_free(vm, &(*gr_ctx)->t18x.preempt_ctxsw_buffer);
fail_free_gk20a_ctx:
	gr_gk20a_free_gr_ctx(g, vm, *gr_ctx);
	*gr_ctx = NULL;

	return err;
}

static void dump_ctx_switch_stats(struct gk20a *g, struct vm_gk20a *vm,
		  struct gr_ctx_desc *gr_ctx)
{
	struct nvgpu_mem *mem = &gr_ctx->mem;

	if (nvgpu_mem_begin(g, mem)) {
		WARN_ON("Cannot map context");
		return;
	}
	gk20a_err(dev_from_gk20a(g), "ctxsw_prog_main_image_magic_value_o : %x (expect %x)\n",
		nvgpu_mem_rd(g, mem,
				ctxsw_prog_main_image_magic_value_o()),
		ctxsw_prog_main_image_magic_value_v_value_v());


	gk20a_err(dev_from_gk20a(g), "NUM_SAVE_OPERATIONS : %d\n",
		nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_num_save_ops_o()));
	gk20a_err(dev_from_gk20a(g), "WFI_SAVE_OPERATIONS : %d\n",
		nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_num_wfi_save_ops_o()));
	gk20a_err(dev_from_gk20a(g), "CTA_SAVE_OPERATIONS : %d\n",
		nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_num_cta_save_ops_o()));
	gk20a_err(dev_from_gk20a(g), "GFXP_SAVE_OPERATIONS : %d\n",
		nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_num_gfxp_save_ops_o()));
	gk20a_err(dev_from_gk20a(g), "CILP_SAVE_OPERATIONS : %d\n",
		nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_num_cilp_save_ops_o()));
	gk20a_err(dev_from_gk20a(g),
		"image gfx preemption option (GFXP is 1) %x\n",
		nvgpu_mem_rd(g, mem,
			ctxsw_prog_main_image_graphics_preemption_options_o()));
	nvgpu_mem_end(g, mem);
}

static void gr_gv11b_free_gr_ctx(struct gk20a *g, struct vm_gk20a *vm,
			  struct gr_ctx_desc *gr_ctx)
{
	gk20a_dbg_fn("");

	if (!gr_ctx)
		return;

	if (g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close)
		dump_ctx_switch_stats(g, vm, gr_ctx);

	gk20a_gmmu_unmap_free(vm, &gr_ctx->t18x.pagepool_ctxsw_buffer);
	gk20a_gmmu_unmap_free(vm, &gr_ctx->t18x.betacb_ctxsw_buffer);
	gk20a_gmmu_unmap_free(vm, &gr_ctx->t18x.spill_ctxsw_buffer);
	gk20a_gmmu_unmap_free(vm, &gr_ctx->t18x.preempt_ctxsw_buffer);
	gr_gk20a_free_gr_ctx(g, vm, gr_ctx);
	gk20a_dbg_fn("done");
}


static void gr_gv11b_update_ctxsw_preemption_mode(struct gk20a *g,
		struct channel_ctx_gk20a *ch_ctx,
		struct nvgpu_mem *mem)
{
	struct gr_ctx_desc *gr_ctx = ch_ctx->gr_ctx;
	u32 gfxp_preempt_option =
		ctxsw_prog_main_image_graphics_preemption_options_control_gfxp_f();
	u32 cilp_preempt_option =
		ctxsw_prog_main_image_compute_preemption_options_control_cilp_f();
	int err;

	gk20a_dbg_fn("");

	if (gr_ctx->graphics_preempt_mode == NVGPU_GRAPHICS_PREEMPTION_MODE_GFXP) {
		gk20a_dbg_info("GfxP: %x", gfxp_preempt_option);
		nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_graphics_preemption_options_o(),
				gfxp_preempt_option);
	}

	if (gr_ctx->compute_preempt_mode == NVGPU_COMPUTE_PREEMPTION_MODE_CILP) {
		gk20a_dbg_info("CILP: %x", cilp_preempt_option);
		nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_compute_preemption_options_o(),
				cilp_preempt_option);
	}

	if (gr_ctx->t18x.preempt_ctxsw_buffer.gpu_va) {
		u32 addr;
		u32 size;
		u32 cbes_reserve;

		nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_full_preemption_ptr_o(),
				gr_ctx->t18x.preempt_ctxsw_buffer.gpu_va >> 8);

		err = gr_gk20a_ctx_patch_write_begin(g, ch_ctx);
		if (err) {
			gk20a_err(dev_from_gk20a(g),
					"can't map patch context");
			goto out;
		}

		addr = (u64_lo32(gr_ctx->t18x.betacb_ctxsw_buffer.gpu_va) >>
			gr_gpcs_setup_attrib_cb_base_addr_39_12_align_bits_v()) |
			(u64_hi32(gr_ctx->t18x.betacb_ctxsw_buffer.gpu_va) <<
			 (32 - gr_gpcs_setup_attrib_cb_base_addr_39_12_align_bits_v()));

		gk20a_dbg_info("attrib cb addr : 0x%016x", addr);
		g->ops.gr.commit_global_attrib_cb(g, ch_ctx, addr, true);

		addr = (u64_lo32(gr_ctx->t18x.pagepool_ctxsw_buffer.gpu_va) >>
			gr_scc_pagepool_base_addr_39_8_align_bits_v()) |
			(u64_hi32(gr_ctx->t18x.pagepool_ctxsw_buffer.gpu_va) <<
			 (32 - gr_scc_pagepool_base_addr_39_8_align_bits_v()));
		size = gr_ctx->t18x.pagepool_ctxsw_buffer.size;

		if (size == g->ops.gr.pagepool_default_size(g))
			size = gr_scc_pagepool_total_pages_hwmax_v();

		g->ops.gr.commit_global_pagepool(g, ch_ctx, addr, size, true);

		addr = (u64_lo32(gr_ctx->t18x.spill_ctxsw_buffer.gpu_va) >>
			gr_gpc0_swdx_rm_spill_buffer_addr_39_8_align_bits_v()) |
			(u64_hi32(gr_ctx->t18x.spill_ctxsw_buffer.gpu_va) <<
			 (32 - gr_gpc0_swdx_rm_spill_buffer_addr_39_8_align_bits_v()));
		size = gr_ctx->t18x.spill_ctxsw_buffer.size /
			gr_gpc0_swdx_rm_spill_buffer_size_256b_byte_granularity_v();

		gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_swdx_rm_spill_buffer_addr_r(),
				gr_gpc0_swdx_rm_spill_buffer_addr_39_8_f(addr),
				true);
		gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_swdx_rm_spill_buffer_size_r(),
				gr_gpc0_swdx_rm_spill_buffer_size_256b_f(size),
				true);

		cbes_reserve = gr_gpcs_swdx_beta_cb_ctrl_cbes_reserve_gfxp_v();
		gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpcs_swdx_beta_cb_ctrl_r(),
				gr_gpcs_swdx_beta_cb_ctrl_cbes_reserve_f(
					cbes_reserve),
				true);
		gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpcs_ppcs_cbm_beta_cb_ctrl_r(),
				gr_gpcs_ppcs_cbm_beta_cb_ctrl_cbes_reserve_f(
					cbes_reserve),
				true);

		gr_gk20a_ctx_patch_write_end(g, ch_ctx);
	}

out:
	gk20a_dbg_fn("done");
}

static int gr_gv11b_dump_gr_status_regs(struct gk20a *g,
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

static int gr_gv11b_wait_empty(struct gk20a *g, unsigned long duration_ms,
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

	gk20a_err(dev_from_gk20a(g),
		"timeout, ctxsw busy : %d, gr busy : %d, %08x, %08x, %08x, %08x",
		ctxsw_active, gr_busy, activity0, activity1, activity2, activity4);

	return -EAGAIN;
}

static void gr_gv11b_commit_global_attrib_cb(struct gk20a *g,
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

static void gr_gv11b_commit_global_bundle_cb(struct gk20a *g,
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

static int gr_gv11b_init_fs_state(struct gk20a *g)
{
	return gr_gp10b_init_fs_state(g);
}

static void gr_gv11b_init_cyclestats(struct gk20a *g)
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

static void gr_gv11b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
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

static void gr_gv11b_get_access_map(struct gk20a *g,
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
		0x419e10, /* gr_pri_gpcs_tpcs_sm_dbgr_control0 */
		0x419f78, /* gr_pri_gpcs_tpcs_sm_disp_ctrl     */
	};

	*whitelist = wl_addr_gv11b;
	*num_entries = ARRAY_SIZE(wl_addr_gv11b);
}

static int gr_gv11b_disable_channel_or_tsg(struct gk20a *g, struct channel_gk20a *fault_ch)
{
	int ret = 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	ret = gk20a_disable_channel_tsg(g, fault_ch);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
				"CILP: failed to disable channel/TSG!\n");
		return ret;
	}

	ret = g->ops.fifo.update_runlist(g, fault_ch->runlist_id, ~0, true, false);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
				"CILP: failed to restart runlist 0!");
		return ret;
	}

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "CILP: restarted runlist");

	if (gk20a_is_channel_marked_as_tsg(fault_ch))
		gk20a_fifo_issue_preempt(g, fault_ch->tsgid, true);
	else
		gk20a_fifo_issue_preempt(g, fault_ch->hw_chid, false);

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "CILP: preempted the channel/tsg");

	return ret;
}

static int gr_gv11b_set_cilp_preempt_pending(struct gk20a *g, struct channel_gk20a *fault_ch)
{
	int ret;
	struct gr_ctx_desc *gr_ctx = fault_ch->ch_ctx.gr_ctx;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	if (!gr_ctx)
		return -EINVAL;

	if (gr_ctx->t18x.cilp_preempt_pending) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
				"CILP is already pending for chid %d",
				fault_ch->hw_chid);
		return 0;
	}

	/* get ctx_id from the ucode image */
	if (!gr_ctx->t18x.ctx_id_valid) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
				"CILP: looking up ctx id");
		ret = gr_gk20a_get_ctx_id(g, fault_ch, &gr_ctx->t18x.ctx_id);
		if (ret) {
			gk20a_err(dev_from_gk20a(g), "CILP: error looking up ctx id!\n");
			return ret;
		}
		gr_ctx->t18x.ctx_id_valid = true;
	}

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
			"CILP: ctx id is 0x%x", gr_ctx->t18x.ctx_id);

	/* send ucode method to set ctxsw interrupt */
	ret = gr_gk20a_submit_fecs_sideband_method_op(g,
			(struct fecs_method_op_gk20a) {
			.method.data = gr_ctx->t18x.ctx_id,
			.method.addr =
			gr_fecs_method_push_adr_configure_interrupt_completion_option_v(),
			.mailbox = {
			.id = 1 /* sideband */, .data = 0,
			.clr = ~0, .ret = NULL,
			.ok = gr_fecs_ctxsw_mailbox_value_pass_v(),
			.fail = 0},
			.cond.ok = GR_IS_UCODE_OP_EQUAL,
			.cond.fail = GR_IS_UCODE_OP_SKIP});

	if (ret) {
		gk20a_err(dev_from_gk20a(g),
				"CILP: failed to enable ctxsw interrupt!");
		return ret;
	}

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
				"CILP: enabled ctxsw completion interrupt");

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
			"CILP: disabling channel %d",
			fault_ch->hw_chid);

	ret = gr_gv11b_disable_channel_or_tsg(g, fault_ch);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
				"CILP: failed to disable channel!!");
		return ret;
	}

	/* set cilp_preempt_pending = true and record the channel */
	gr_ctx->t18x.cilp_preempt_pending = true;
	g->gr.t18x.cilp_preempt_pending_chid = fault_ch->hw_chid;

	if (gk20a_is_channel_marked_as_tsg(fault_ch)) {
		struct tsg_gk20a *tsg = &g->fifo.tsg[fault_ch->tsgid];

		gk20a_tsg_event_id_post_event(tsg,
				NVGPU_IOCTL_CHANNEL_EVENT_ID_CILP_PREEMPTION_STARTED);
	} else {
		gk20a_channel_event_id_post_event(fault_ch,
				NVGPU_IOCTL_CHANNEL_EVENT_ID_CILP_PREEMPTION_STARTED);
	}

	return 0;
}

static int gr_gv11b_clear_cilp_preempt_pending(struct gk20a *g,
					       struct channel_gk20a *fault_ch)
{
	struct gr_ctx_desc *gr_ctx = fault_ch->ch_ctx.gr_ctx;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	if (!gr_ctx)
		return -EINVAL;

	/* The ucode is self-clearing, so all we need to do here is
	   to clear cilp_preempt_pending. */
	if (!gr_ctx->t18x.cilp_preempt_pending) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
				"CILP is already cleared for chid %d\n",
				fault_ch->hw_chid);
		return 0;
	}

	gr_ctx->t18x.cilp_preempt_pending = false;
	g->gr.t18x.cilp_preempt_pending_chid = -1;

	return 0;
}

/* @brief pre-process work on the SM exceptions to determine if we clear them or not.
 *
 * On Pascal, if we are in CILP preemtion mode, preempt the channel and handle errors with special processing
 */
static int gr_gv11b_pre_process_sm_exception(struct gk20a *g,
		u32 gpc, u32 tpc, u32 global_esr, u32 warp_esr,
		bool sm_debugger_attached, struct channel_gk20a *fault_ch,
		bool *early_exit, bool *ignore_debugger)
{
	int ret;
	bool cilp_enabled = false;
	u32 global_mask = 0, dbgr_control0, global_esr_copy;
	u32 offset = proj_gpc_stride_v() * gpc +
		     proj_tpc_in_gpc_stride_v() * tpc;

	*early_exit = false;
	*ignore_debugger = false;

	if (fault_ch)
		cilp_enabled = (fault_ch->ch_ctx.gr_ctx->compute_preempt_mode ==
			NVGPU_COMPUTE_PREEMPTION_MODE_CILP);

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "SM Exception received on gpc %d tpc %d = %u\n",
			gpc, tpc, global_esr);

	if (cilp_enabled && sm_debugger_attached) {
		if (global_esr & gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f())
			gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
					gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f());

		if (global_esr & gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f())
			gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
					gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f());

		global_mask = gr_gpcs_tpcs_sm0_hww_global_esr_multiple_warp_errors_pending_f() |
			gr_gpcs_tpcs_sm0_hww_global_esr_bpt_pause_pending_f();

		if (warp_esr != 0 || (global_esr & global_mask) != 0) {
			*ignore_debugger = true;

			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: starting wait for LOCKED_DOWN on gpc %d tpc %d\n",
					gpc, tpc);

			if (gk20a_dbg_gpu_broadcast_stop_trigger(fault_ch)) {
				gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
						"CILP: Broadcasting STOP_TRIGGER from gpc %d tpc %d\n",
						gpc, tpc);
				gk20a_suspend_all_sms(g, global_mask, false);

				gk20a_dbg_gpu_clear_broadcast_stop_trigger(fault_ch);
			} else {
				gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
						"CILP: STOP_TRIGGER from gpc %d tpc %d\n",
						gpc, tpc);
				gk20a_suspend_single_sm(g, gpc, tpc, global_mask, true);
			}

			/* reset the HWW errors after locking down */
			global_esr_copy = gk20a_readl(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset);
			gk20a_gr_clear_sm_hww(g, gpc, tpc, global_esr_copy);
			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: HWWs cleared for gpc %d tpc %d\n",
					gpc, tpc);

			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "CILP: Setting CILP preempt pending\n");
			ret = gr_gv11b_set_cilp_preempt_pending(g, fault_ch);
			if (ret) {
				gk20a_err(dev_from_gk20a(g), "CILP: error while setting CILP preempt pending!\n");
				return ret;
			}

			dbgr_control0 = gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
			if (dbgr_control0 & gr_gpcs_tpcs_sm0_dbgr_control0_single_step_mode_enable_f()) {
				gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
						"CILP: clearing SINGLE_STEP_MODE before resume for gpc %d tpc %d\n",
						gpc, tpc);
				dbgr_control0 = set_field(dbgr_control0,
						gr_gpcs_tpcs_sm0_dbgr_control0_single_step_mode_m(),
						gr_gpcs_tpcs_sm0_dbgr_control0_single_step_mode_disable_f());
				gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset, dbgr_control0);
			}

			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: resume for gpc %d tpc %d\n",
					gpc, tpc);
			gk20a_resume_single_sm(g, gpc, tpc);

			*ignore_debugger = true;
			gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "CILP: All done on gpc %d, tpc %d\n", gpc, tpc);
		}

		*early_exit = true;
	}
	return 0;
}

static int gr_gv11b_get_cilp_preempt_pending_chid(struct gk20a *g, int *__chid)
{
	struct gr_ctx_desc *gr_ctx;
	struct channel_gk20a *ch;
	int chid;
	int ret = -EINVAL;

	chid = g->gr.t18x.cilp_preempt_pending_chid;

	ch = gk20a_channel_get(gk20a_fifo_channel_from_hw_chid(g, chid));
	if (!ch)
		return ret;

	gr_ctx = ch->ch_ctx.gr_ctx;

	if (gr_ctx->t18x.cilp_preempt_pending) {
		*__chid = chid;
		ret = 0;
	}

	gk20a_channel_put(ch);

	return ret;
}

static int gr_gv11b_handle_fecs_error(struct gk20a *g,
				struct channel_gk20a *__ch,
				struct gr_gk20a_isr_data *isr_data)
{
	u32 gr_fecs_intr = gk20a_readl(g, gr_fecs_host_int_status_r());
	struct channel_gk20a *ch;
	int chid = -1;
	int ret = 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, "");

	/*
	 * INTR1 (bit 1 of the HOST_INT_STATUS_CTXSW_INTR)
	 * indicates that a CILP ctxsw save has finished
	 */
	if (gr_fecs_intr & gr_fecs_host_int_status_ctxsw_intr_f(2)) {
		gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr,
				"CILP: ctxsw save completed!\n");

		/* now clear the interrupt */
		gk20a_writel(g, gr_fecs_host_int_clear_r(),
				gr_fecs_host_int_clear_ctxsw_intr1_clear_f());

		ret = gr_gv11b_get_cilp_preempt_pending_chid(g, &chid);
		if (ret)
			goto clean_up;

		ch = gk20a_channel_get(
				gk20a_fifo_channel_from_hw_chid(g, chid));
		if (!ch)
			goto clean_up;


		/* set preempt_pending to false */
		ret = gr_gv11b_clear_cilp_preempt_pending(g, ch);
		if (ret) {
			gk20a_err(dev_from_gk20a(g), "CILP: error while unsetting CILP preempt pending!\n");
			gk20a_channel_put(ch);
			goto clean_up;
		}

		if (gk20a_gr_sm_debugger_attached(g)) {
			gk20a_dbg_gpu_post_events(ch);

			if (gk20a_is_channel_marked_as_tsg(ch)) {
				struct tsg_gk20a *tsg = &g->fifo.tsg[ch->tsgid];

				gk20a_tsg_event_id_post_event(tsg,
					NVGPU_IOCTL_CHANNEL_EVENT_ID_CILP_PREEMPTION_COMPLETE);
			} else {
				gk20a_channel_event_id_post_event(ch,
					NVGPU_IOCTL_CHANNEL_EVENT_ID_CILP_PREEMPTION_COMPLETE);
			}
		}

		gk20a_channel_put(ch);
	}

clean_up:
	/* handle any remaining interrupts */
	return gk20a_gr_handle_fecs_error(g, __ch, isr_data);
}

static u32 gv11b_mask_hww_warp_esr(u32 hww_warp_esr)
{
	if (!(hww_warp_esr & gr_gpc0_tpc0_sm0_hww_warp_esr_wrap_id_m()))
		hww_warp_esr = set_field(hww_warp_esr,
			gr_gpc0_tpc0_sm0_hww_warp_esr_addr_error_type_m(),
			gr_gpc0_tpc0_sm0_hww_warp_esr_addr_error_type_none_f());

	return hww_warp_esr;
}

static int gr_gv11b_setup_rop_mapping(struct gk20a *g, struct gr_gk20a *gr)
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
	u32 data = gk20a_readl(g, gr_pri_fe_chip_def_info_r());
	u32  num_subctx, err = 0;

	num_subctx = gr_pri_fe_chip_def_info_max_veid_count_v(data);

	for (j = 0; j < num_subctx; j++) {

		gk20a_writel(g, gr_pipe_bundle_address_r(),
			sw_veid_bundle_init->l[index].addr |
			gr_pipe_bundle_address_veid_f(j));

		err = gr_gk20a_wait_fe_idle(g, gk20a_get_gr_idle_timeout(g),
					    GR_IDLE_CHECK_DEFAULT);
	}
}

static int gr_gv11b_init_sw_veid_bundle(struct gk20a *g)
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

static void gr_gv11b_detect_sm_arch(struct gk20a *g)
{
	u32 v = gk20a_readl(g, gr_gpc0_tpc0_sm_arch_r());

	g->gpu_characteristics.sm_arch_spa_version =
		gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	g->gpu_characteristics.sm_arch_sm_version =
		gr_gpc0_tpc0_sm_arch_sm_version_v(v);
	g->gpu_characteristics.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

static void gr_gv11b_init_sm_id_table(struct gk20a *g)
{
	u32 gpc, tpc;
	u32 sm_id = 0;

	/* TODO populate smids based on power efficiency */
	for (tpc = 0; tpc < g->gr.max_tpc_per_gpc_count; tpc++) {
		for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {

			if (tpc < g->gr.gpc_tpc_count[gpc]) {
				g->gr.sm_to_cluster[sm_id].tpc_index = tpc;
				g->gr.sm_to_cluster[sm_id].gpc_index = gpc;
				g->gr.sm_to_cluster[sm_id].sm_index = sm_id % 2;
				g->gr.sm_to_cluster[sm_id].global_tpc_index =
									sm_id;
				sm_id++;
			}
		}
	}
	g->gr.no_of_sm = sm_id;
}

static void gr_gv11b_program_sm_id_numbering(struct gk20a *g,
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

static int gr_gv11b_load_smid_config(struct gk20a *g)
{
	u32 *tpc_sm_id;
	u32 i, j;
	u32 tpc_index, gpc_index, tpc_id;
	u32 sms_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
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
			sm_id = tpc_id * sms_per_tpc;

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

static int gr_gv11b_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	u32 addr_lo;
	u32 addr_hi;
	struct ctx_header_desc *ctx;

	gk20a_dbg_fn("");

	gv11b_alloc_subctx_header(c);

	gv11b_update_subctx_header(c, gpu_va);

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



static int gr_gv11b_commit_global_timeslice(struct gk20a *g,
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

static void gv11b_restore_context_header(struct gk20a *g,
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
        nvgpu_mem_wr(g, ctxheader,
			ctxsw_prog_main_image_num_restore_ops_o(), 0);
        nvgpu_mem_wr(g, ctxheader,
			ctxsw_prog_main_image_num_save_ops_o(), 0);
}
static void gr_gv11b_write_zcull_ptr(struct gk20a *g,
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


static void gr_gv11b_write_pm_ptr(struct gk20a *g,
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
		gk20a_err(dev_from_gk20a(g),
			"invalid elcg mode %d", mode);
	}

	gk20a_writel(g, therm_gate_ctrl_r(engine), gate_ctrl);
}

static void gr_gv11b_load_tpc_mask(struct gk20a *g)
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

void gv11b_init_gr(struct gpu_ops *gops)
{
	gp10b_init_gr(gops);
	gops->gr.init_preemption_state = NULL;
	gops->gr.init_fs_state = gr_gv11b_init_fs_state;
	gops->gr.detect_sm_arch = gr_gv11b_detect_sm_arch;
	gops->gr.is_valid_class = gr_gv11b_is_valid_class;
	gops->gr.commit_global_cb_manager = gr_gv11b_commit_global_cb_manager;
	gops->gr.commit_global_pagepool = gr_gv11b_commit_global_pagepool;
	gops->gr.add_zbc_s = gr_gv11b_add_zbc_stencil;
	gops->gr.load_zbc_s_default_tbl = gr_gv11b_load_stencil_default_tbl;
	gops->gr.load_zbc_s_tbl = gr_gv11b_load_stencil_tbl;
	gops->gr.zbc_s_query_table = gr_gv11b_zbc_s_query_table;
	gops->gr.add_zbc_type_s = gr_gv11b_add_zbc_type_s;
	gops->gr.pagepool_default_size = gr_gv11b_pagepool_default_size;
	gops->gr.calc_global_ctx_buffer_size =
		gr_gv11b_calc_global_ctx_buffer_size;
	gops->gr.commit_global_attrib_cb = gr_gv11b_commit_global_attrib_cb;
	gops->gr.commit_global_bundle_cb = gr_gv11b_commit_global_bundle_cb;
	gops->gr.handle_sw_method = gr_gv11b_handle_sw_method;
	gops->gr.bundle_cb_defaults = gr_gv11b_bundle_cb_defaults;
	gops->gr.cb_size_default = gr_gv11b_cb_size_default;
	gops->gr.set_alpha_circular_buffer_size =
		gr_gv11b_set_alpha_circular_buffer_size;
	gops->gr.set_circular_buffer_size =
		gr_gv11b_set_circular_buffer_size;
	gops->gr.init_ctx_state = gr_gv11b_init_ctx_state;
	gops->gr.alloc_gr_ctx = gr_gv11b_alloc_gr_ctx;
	gops->gr.free_gr_ctx = gr_gv11b_free_gr_ctx;
	gops->gr.update_ctxsw_preemption_mode =
		gr_gv11b_update_ctxsw_preemption_mode;
	gops->gr.dump_gr_regs = gr_gv11b_dump_gr_status_regs;
	gops->gr.wait_empty = gr_gv11b_wait_empty;
	gops->gr.init_cyclestats = gr_gv11b_init_cyclestats;
	gops->gr.set_gpc_tpc_mask = gr_gv11b_set_gpc_tpc_mask;
	gops->gr.get_access_map = gr_gv11b_get_access_map;
	gops->gr.handle_sm_exception = gr_gv11b_handle_sm_exception;
	gops->gr.handle_tex_exception = gr_gv11b_handle_tex_exception;
	gops->gr.enable_gpc_exceptions = gr_gv11b_enable_gpc_exceptions;
	gops->gr.mask_hww_warp_esr = gv11b_mask_hww_warp_esr;
	gops->gr.pre_process_sm_exception =
		gr_gv11b_pre_process_sm_exception;
	gops->gr.handle_fecs_error = gr_gv11b_handle_fecs_error;
	gops->gr.setup_rop_mapping = gr_gv11b_setup_rop_mapping;
	gops->gr.init_sw_veid_bundle = gr_gv11b_init_sw_veid_bundle;
	gops->gr.program_zcull_mapping = gr_gv11b_program_zcull_mapping;
	gops->gr.commit_global_timeslice = gr_gv11b_commit_global_timeslice;
	gops->gr.init_sm_id_table = gr_gv11b_init_sm_id_table;
	gops->gr.load_smid_config = gr_gv11b_load_smid_config;
	gops->gr.program_sm_id_numbering =
			gr_gv11b_program_sm_id_numbering;
	gops->gr.commit_inst = gr_gv11b_commit_inst;
	gops->gr.restore_context_header = gv11b_restore_context_header;
	gops->gr.write_zcull_ptr = gr_gv11b_write_zcull_ptr;
	gops->gr.write_pm_ptr = gr_gv11b_write_pm_ptr;
	gops->gr.init_elcg_mode = gr_gv11b_init_elcg_mode;
	gops->gr.load_tpc_mask = gr_gv11b_load_tpc_mask;

}
