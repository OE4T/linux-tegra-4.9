/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/pmu.h>
#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

#include "gk20a/gk20a.h"

void nvgpu_pmu_dump_elpg_stats(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = pmu->g;

	/* Print PG stats */
	nvgpu_err(g, "Print PG stats");
	nvgpu_flcn_print_dmem(pmu->flcn,
		pmu->stat_dmem_offset[PMU_PG_ELPG_ENGINE_ID_GRAPHICS],
		sizeof(struct pmu_pg_stats_v2));

	gk20a_pmu_dump_elpg_stats(pmu);
}

void nvgpu_pmu_dump_falcon_stats(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = pmu->g;

	nvgpu_flcn_dump_stats(pmu->flcn);
	gk20a_pmu_dump_falcon_stats(pmu);

	nvgpu_err(g, "pmu state: %d", pmu->pmu_state);
	nvgpu_err(g, "elpg state: %d", pmu->elpg_stat);

	/* PMU may crash due to FECS crash. Dump FECS status */
	gk20a_fecs_dump_falcon_stats(g);
}
