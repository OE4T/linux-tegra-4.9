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
#include <vgpu/gr_vgpu.h>
#include <vgpu/gp10b/vgpu_gr_gp10b.h>

#include "vgpu_gr_gv11b.h"
#include "vgpu_subctx_gv11b.h"

static int vgpu_gr_gv11b_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	int err;

	err = vgpu_gv11b_alloc_subctx_header(c);
	if (err)
		return err;

	err = vgpu_gr_commit_inst(c, gpu_va);
	if (err)
		vgpu_gv11b_free_subctx_header(c);

	return err;
}

void vgpu_gv11b_init_gr_ops(struct gpu_ops *gops)
{
	vgpu_gp10b_init_gr_ops(gops);
	gops->gr.commit_inst = vgpu_gr_gv11b_commit_inst;
}
