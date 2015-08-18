/*
 * GP10B CDE
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#include "cde_gp10b.h"

enum gp10b_programs {
	GP10B_PROG_HPASS              = 0,
	GP10B_PROG_HPASS_4K           = 1,
	GP10B_PROG_VPASS              = 2,
	GP10B_PROG_VPASS_4K           = 3,
	GP10B_PROG_HPASS_DEBUG        = 4,
	GP10B_PROG_HPASS_4K_DEBUG     = 5,
	GP10B_PROG_VPASS_DEBUG        = 6,
	GP10B_PROG_VPASS_4K_DEBUG     = 7,
	GP10B_PROG_PASSTHROUGH        = 8,
};

static void gp10b_cde_get_program_numbers(struct gk20a *g,
					  u32 block_height_log2,
					  int *hprog_out, int *vprog_out)
{
	int hprog, vprog;

	if (g->cde_app.shader_parameter == 1) {
		hprog = GP10B_PROG_PASSTHROUGH;
		vprog = GP10B_PROG_PASSTHROUGH;
	} else {
		hprog = GP10B_PROG_HPASS;
		vprog = GP10B_PROG_VPASS;
		if (g->cde_app.shader_parameter == 2) {
			hprog = GP10B_PROG_HPASS_DEBUG;
			vprog = GP10B_PROG_VPASS_DEBUG;
		}
		if (g->mm.bypass_smmu) {
			if (!g->mm.disable_bigpage) {
				gk20a_warn(&g->dev->dev,
					   "when bypass_smmu is 1, disable_bigpage must be 1 too");
			}
			hprog |= 1;
			vprog |= 1;
		}
	}

	*hprog_out = hprog;
	*vprog_out = vprog;
}

void gp10b_init_cde_ops(struct gpu_ops *gops)
{
	gops->cde.get_program_numbers = gp10b_cde_get_program_numbers;
}
