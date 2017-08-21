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

#include "gk20a/gk20a.h"

#include <nvgpu/enabled.h>
#include <nvgpu/enabled_t19x.h>

#include "vgpu/vgpu.h"
#include "vgpu_gv11b.h"

int vgpu_gv11b_init_gpu_characteristics(struct gk20a *g)
{
	int err;

	gk20a_dbg_fn("");


	nvgpu_err(g, "g->gpu_characteristics.flags=%llx",
		g->gpu_characteristics.flags);
	err = vgpu_init_gpu_characteristics(g);
	if (err) {
		nvgpu_err(g, "vgpu_init_gpu_characteristics failed, err %d\n", err);
		return err;
	}

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_TSG_SUBCONTEXTS, true);

	return 0;
}
