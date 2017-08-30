/*
 * GV11B PMU
 *
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

#ifndef __PMU_GV11B_H_
#define __PMU_GV11B_H_

struct gk20a;

bool gv11b_is_pmu_supported(struct gk20a *g);
int gv11b_pmu_bootstrap(struct nvgpu_pmu *pmu);
int gv11b_pg_gr_init(struct gk20a *g, u32 pg_engine_id);
int gv11b_pg_set_subfeature_mask(struct gk20a *g, u32 pg_engine_id);
bool gv11b_is_lazy_bootstrap(u32 falcon_id);
bool gv11b_is_priv_load(u32 falcon_id);

#endif /*__PMU_GV11B_H_*/
