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
#include "gk20a/gk20a.h"
#include "gk20a/flcn_gk20a.h"
#include "gp106/sec2_gp106.h"

#include <nvgpu/hw/gp106/hw_falcon_gp106.h>

static void gp106_falcon_engine_dependency_ops(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_engine_dependency_ops *flcn_eng_dep_ops =
			&flcn->flcn_engine_dep_ops;

	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn_eng_dep_ops->reset_eng = nvgpu_pmu_reset;
		break;
	case FALCON_ID_SEC2:
		flcn_eng_dep_ops->reset_eng = gp106_sec2_reset;
		break;
	default:
		flcn_eng_dep_ops->reset_eng = NULL;
		break;
	}
}

static void gp106_falcon_ops(struct nvgpu_falcon *flcn)
{
	gk20a_falcon_ops(flcn);
	gp106_falcon_engine_dependency_ops(flcn);
}

void gp106_falcon_hal_sw_init(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;

	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn->flcn_base = FALCON_PWR_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = true;
		break;
	case FALCON_ID_SEC2:
		flcn->flcn_base = FALCON_SEC_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
		break;
	case FALCON_ID_FECS:
		flcn->flcn_base = FALCON_FECS_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
	break;
	case FALCON_ID_GPCCS:
		flcn->flcn_base = FALCON_GPCCS_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
	break;
	default:
		flcn->is_falcon_supported = false;
		nvgpu_err(g, "Invalid flcn request");
		break;
	}

	if (flcn->is_falcon_supported) {
		nvgpu_mutex_init(&flcn->copy_lock);
		gp106_falcon_ops(flcn);
	} else
		nvgpu_info(g, "falcon 0x%x not supported on %s",
			flcn->flcn_id, g->name);
}
