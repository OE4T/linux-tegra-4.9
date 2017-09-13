/*
 * Virtualized GPU L2
 *
 * Copyright (c) 2014-2016 NVIDIA CORPORATION.  All rights reserved.
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
