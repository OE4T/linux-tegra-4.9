/*
 * Virtualized GPU L2
 *
 * Copyright (c) 2014-2016 NVIDIA CORPORATION.  All rights reserved.
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

#include "vgpu/vgpu.h"

static int vgpu_determine_L2_size_bytes(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	return priv->constants.l2_size;
}

static int vgpu_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	u32 max_comptag_lines = 0;
	int err;

	gk20a_dbg_fn("");

	gr->cacheline_size = priv->constants.cacheline_size;
	gr->comptags_per_cacheline = priv->constants.comptags_per_cacheline;
	gr->slices_per_ltc = priv->constants.slices_per_ltc;
	max_comptag_lines = priv->constants.comptag_lines;

	if (max_comptag_lines < 2)
		return -ENXIO;

	err = gk20a_comptag_allocator_init(&gr->comp_tags, max_comptag_lines);
	if (err)
		return err;

	return 0;
}

static void vgpu_ltc_init_fs_state(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	gk20a_dbg_fn("");

	g->ltc_count = priv->constants.ltc_count;
}

void vgpu_init_ltc_ops(struct gpu_ops *gops)
{
	gops->ltc.determine_L2_size_bytes = vgpu_determine_L2_size_bytes;
	gops->ltc.init_comptags = vgpu_ltc_init_comptags;
	gops->ltc.init_fs_state = vgpu_ltc_init_fs_state;
	gops->ltc.cbc_ctrl = NULL;
}
