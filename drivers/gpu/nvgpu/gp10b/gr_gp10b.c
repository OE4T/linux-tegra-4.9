/*
 * GP10B GPU GR
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h" /* FERMI and MAXWELL classes defined here */

#include "gk20a/gr_gk20a.h"

#include "gm20b/gr_gm20b.h" /* for MAXWELL classes */
#include "gp10b/gr_gp10b.h"
#include "hw_gr_gp10b.h"
#include "hw_proj_gp10b.h"

bool gr_gp10b_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case PASCAL_COMPUTE_A:
	case PASCAL_A:
	case PASCAL_DMA_COPY_A:
		valid = true;
		break;

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
	gk20a_dbg_info("class=0x%x valid=%d", class_num, valid);
	return valid;
}

int gr_gp10b_commit_global_cb_manager(struct gk20a *g,
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
				gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_r() + temp +
				proj_ppc_in_gpc_stride_v() * ppc_index,
				gr->alpha_cb_default_size * gr->pes_tpc_count[ppc_index][gpc_index],
				patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpcs_swdx_tc_beta_cb_size_r(ppc_index + gpc_index),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(cbm_cfg_size1),
				patch);
		}
	}

	if (patch)
		gr_gk20a_ctx_patch_write_end(g, ch_ctx);

	return 0;
}

void gr_gp10b_commit_global_pagepool(struct gk20a *g,
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

static int gr_gp10b_add_zbc_color(struct gk20a *g, struct gr_gk20a *gr,
				  struct zbc_entry *color_val, u32 index)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *gr_info = f->engine_info + ENGINE_GR_GK20A;
	u32 i;
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(gk20a_get_gr_idle_timeout(g));
	u32 ret;
	u32 zbc_c;

	ret = gk20a_fifo_disable_engine_activity(g, gr_info, true);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
			"failed to disable gr engine activity\n");
		return ret;
	}

	ret = gr_gk20a_wait_idle(g, end_jiffies, GR_IDLE_CHECK_DEFAULT);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
			"failed to idle graphics\n");
		goto clean_up;
	}

	/* update l2 table */
	g->ops.ltc.set_zbc_color_entry(g, color_val, index);

	/* update ds table */
	gk20a_writel(g, gr_ds_zbc_color_r_r(),
		gr_ds_zbc_color_r_val_f(color_val->color_ds[0]));
	gk20a_writel(g, gr_ds_zbc_color_g_r(),
		gr_ds_zbc_color_g_val_f(color_val->color_ds[1]));
	gk20a_writel(g, gr_ds_zbc_color_b_r(),
		gr_ds_zbc_color_b_val_f(color_val->color_ds[2]));
	gk20a_writel(g, gr_ds_zbc_color_a_r(),
		gr_ds_zbc_color_a_val_f(color_val->color_ds[3]));

	gk20a_writel(g, gr_ds_zbc_color_fmt_r(),
		gr_ds_zbc_color_fmt_val_f(color_val->format));

	gk20a_writel(g, gr_ds_zbc_tbl_index_r(),
		gr_ds_zbc_tbl_index_val_f(index + GK20A_STARTOF_ZBC_TABLE));

	/* trigger the write */
	gk20a_writel(g, gr_ds_zbc_tbl_ld_r(),
		gr_ds_zbc_tbl_ld_select_c_f() |
		gr_ds_zbc_tbl_ld_action_write_f() |
		gr_ds_zbc_tbl_ld_trigger_active_f());

	/* update local copy */
	for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
		gr->zbc_col_tbl[index].color_l2[i] = color_val->color_l2[i];
		gr->zbc_col_tbl[index].color_ds[i] = color_val->color_ds[i];
	}
	gr->zbc_col_tbl[index].format = color_val->format;
	gr->zbc_col_tbl[index].ref_cnt++;

	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_color_r_r(index), color_val->color_ds[0]);
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_color_g_r(index), color_val->color_ds[1]);
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_color_b_r(index), color_val->color_ds[2]);
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_color_a_r(index), color_val->color_ds[3]);
	zbc_c = gk20a_readl(g, gr_gpcs_swdx_dss_zbc_c_01_to_04_format_r() + ALIGN(index, 4));
	zbc_c |= color_val->format << (index % 4) * 6;
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_c_01_to_04_format_r() + ALIGN(index, 4), zbc_c);

clean_up:
	ret = gk20a_fifo_enable_engine_activity(g, gr_info);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
			"failed to enable gr engine activity\n");
	}

	return ret;
}

static int gr_gp10b_add_zbc_depth(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *depth_val, u32 index)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *gr_info = f->engine_info + ENGINE_GR_GK20A;
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(gk20a_get_gr_idle_timeout(g));
	u32 ret;
	u32 zbc_z;

	ret = gk20a_fifo_disable_engine_activity(g, gr_info, true);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
			"failed to disable gr engine activity\n");
		return ret;
	}

	ret = gr_gk20a_wait_idle(g, end_jiffies, GR_IDLE_CHECK_DEFAULT);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
			"failed to idle graphics\n");
		goto clean_up;
	}

	/* update l2 table */
	g->ops.ltc.set_zbc_depth_entry(g, depth_val, index);

	/* update ds table */
	gk20a_writel(g, gr_ds_zbc_z_r(),
		gr_ds_zbc_z_val_f(depth_val->depth));

	gk20a_writel(g, gr_ds_zbc_z_fmt_r(),
		gr_ds_zbc_z_fmt_val_f(depth_val->format));

	gk20a_writel(g, gr_ds_zbc_tbl_index_r(),
		gr_ds_zbc_tbl_index_val_f(index + GK20A_STARTOF_ZBC_TABLE));

	/* trigger the write */
	gk20a_writel(g, gr_ds_zbc_tbl_ld_r(),
		gr_ds_zbc_tbl_ld_select_z_f() |
		gr_ds_zbc_tbl_ld_action_write_f() |
		gr_ds_zbc_tbl_ld_trigger_active_f());

	/* update local copy */
	gr->zbc_dep_tbl[index].depth = depth_val->depth;
	gr->zbc_dep_tbl[index].format = depth_val->format;
	gr->zbc_dep_tbl[index].ref_cnt++;

	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_z_r(index), depth_val->depth);
	zbc_z = gk20a_readl(g, gr_gpcs_swdx_dss_zbc_z_01_to_04_format_r() + ALIGN(index, 4));
	zbc_z |= depth_val->format << (index % 4) * 6;
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_z_01_to_04_format_r() + ALIGN(index, 4), zbc_z);

clean_up:
	ret = gk20a_fifo_enable_engine_activity(g, gr_info);
	if (ret) {
		gk20a_err(dev_from_gk20a(g),
			"failed to enable gr engine activity\n");
	}

	return ret;
}

static void gr_gp10b_buffer_size_defaults(struct gk20a *g)
{
	g->gr.pagepool_default_size =
		gr_scc_pagepool_total_pages_hwmax_value_v();
	g->gr.pagepool_max_size =
		gr_scc_pagepool_total_pages_hwmax_value_v();
}

void gp10b_init_gr(struct gpu_ops *gops)
{
	gm20b_init_gr(gops);
	gops->gr.is_valid_class = gr_gp10b_is_valid_class;
	gops->gr.commit_global_cb_manager = gr_gp10b_commit_global_cb_manager;
	gops->gr.commit_global_pagepool = gr_gp10b_commit_global_pagepool;
	gops->gr.add_zbc_color = gr_gp10b_add_zbc_color;
	gops->gr.add_zbc_depth = gr_gp10b_add_zbc_depth;
	gops->gr.buffer_size_defaults = gr_gp10b_buffer_size_defaults;
}
