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
#include <nvgpu/lock.h>
#include <nvgpu/timers.h>
#include <nvgpu/falcon.h>

#include "gk20a/gk20a.h"

void nvgpu_flcn_sw_init(struct gk20a *g, u32 flcn_id)
{
	struct nvgpu_falcon *flcn = NULL;
	struct gpu_ops *gops = &g->ops;

	switch (flcn_id) {
	case FALCON_ID_PMU:
		flcn = &g->pmu_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_SEC2:
		flcn = &g->sec2_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_FECS:
		flcn = &g->fecs_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_GPCCS:
		flcn = &g->gpccs_flcn;
		flcn->flcn_id = flcn_id;
	break;
	default:
		nvgpu_err(g, "Invalid/Unsupported falcon ID %x", flcn->flcn_id);
		break;
	};

	/* call to HAL method to assign flcn base & ops to selected falcon */
	if (flcn) {
		flcn->g = g;
		gops->falcon.falcon_hal_sw_init(flcn);
	}
}
