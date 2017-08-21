/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
}
