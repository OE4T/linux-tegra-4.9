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
#include <gv11b/hal_gv11b.h>
#include <vgpu/vgpu.h>
#include <vgpu/vgpu_t19x.h>
#include <vgpu/gp10b/vgpu_mm_gp10b.h>

#include "vgpu_gr_gv11b.h"
#include "vgpu_fifo_gv11b.h"

int vgpu_gv11b_init_hal(struct gk20a *g)
{
	int err;

	gk20a_dbg_fn("");

	err = gv11b_init_hal(g);
	if (err)
		return err;

	vgpu_init_hal_common(g);
	vgpu_gp10b_init_mm_ops(&g->ops);

	vgpu_gv11b_init_gr_ops(&g->ops);
	vgpu_gv11b_init_fifo_ops(&g->ops);

	return 0;
}
