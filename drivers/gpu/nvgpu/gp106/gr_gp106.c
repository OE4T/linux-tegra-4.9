/*
 * GP106 GPU GR
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
#include "gr_gp106.h"
#include "hw_gr_gp106.h"

static bool gr_gp106_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case PASCAL_COMPUTE_A:
	case PASCAL_COMPUTE_B:
	case PASCAL_A:
	case PASCAL_B:
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

static u32 gr_gp106_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

static int gr_gp106_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data)
{
	gk20a_dbg_fn("");

	if (class_num == PASCAL_COMPUTE_B) {
		switch (offset << 2) {
		case NVC0C0_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == PASCAL_B) {
		switch (offset << 2) {
		case NVC097_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		case NVC097_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVC097_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
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

static void gr_gp106_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (!gr->attrib_cb_default_size)
		gr->attrib_cb_default_size = 0x800;
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
}

void gp106_init_gr(struct gpu_ops *gops)
{
	gp10b_init_gr(gops);
	gops->gr.is_valid_class = gr_gp106_is_valid_class;
	gops->gr.pagepool_default_size = gr_gp106_pagepool_default_size;
	gops->gr.handle_sw_method = gr_gp106_handle_sw_method;
	gops->gr.cb_size_default = gr_gp106_cb_size_default;
}
