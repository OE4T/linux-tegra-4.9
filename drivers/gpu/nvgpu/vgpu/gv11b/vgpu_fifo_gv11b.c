/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <gk20a/gk20a.h>
#include <vgpu/gp10b/vgpu_fifo_gp10b.h>

#include "vgpu/vgpu.h"

#include "vgpu_fifo_gv11b.h"
#include "vgpu_subctx_gv11b.h"

static int vgpu_gv11b_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	int err;

	err = vgpu_get_attribute(vgpu_get_handle(g),
			TEGRA_VGPU_ATTRIB_MAX_SUBCTX_COUNT,
			&f->t19x.max_subctx_count);
	if (err) {
		nvgpu_err(g, "get max_subctx_count failed %d", err);
		return err;
	}

	return 0;
}

void vgpu_gv11b_init_fifo_ops(struct gpu_ops *gops)
{
	vgpu_gp10b_init_fifo_ops(gops);

	gops->fifo.init_fifo_setup_hw = vgpu_gv11b_init_fifo_setup_hw;
	gops->fifo.free_channel_ctx_header = vgpu_gv11b_free_subctx_header;
	/* TODO: implement it for CE fault */
	gops->fifo.tsg_verify_status_faulted = NULL;
}
