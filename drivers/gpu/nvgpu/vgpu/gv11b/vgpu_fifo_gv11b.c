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

#include "vgpu_fifo_gv11b.h"
#include "vgpu_subctx_gv11b.h"

void vgpu_gv11b_init_fifo_ops(struct gpu_ops *gops)
{
	vgpu_gp10b_init_fifo_ops(gops);

	gops->fifo.init_fifo_setup_hw = NULL;
	gops->fifo.free_channel_ctx_header = vgpu_gv11b_free_subctx_header;
}
