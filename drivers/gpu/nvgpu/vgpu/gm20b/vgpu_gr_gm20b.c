/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "vgpu_gr_gm20b.h"

static int vgpu_gm20b_init_fs_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_index, gpc_index;
	u32 sm_id = 0;

	gk20a_dbg_fn("");

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		for (tpc_index = 0; tpc_index < gr->gpc_tpc_count[gpc_index];
								tpc_index++) {
			g->gr.sm_to_cluster[sm_id].tpc_index = tpc_index;
			g->gr.sm_to_cluster[sm_id].gpc_index = gpc_index;

			sm_id++;
		}
	}

	gr->no_of_sm = sm_id;
	return 0;
}

void vgpu_gm20b_init_gr_ops(struct gpu_ops *gops)
{
	gops->gr.init_fs_state = vgpu_gm20b_init_fs_state;
}
