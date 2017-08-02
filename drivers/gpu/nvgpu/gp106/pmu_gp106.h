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

#ifndef __PMU_GP106_H_
#define __PMU_GP106_H_

#define gp106_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)

struct gk20a;

bool gp106_is_pmu_supported(struct gk20a *g);
u32 gp106_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id);
u32 gp106_pmu_pg_engines_list(struct gk20a *g);
int gp106_pg_param_init(struct gk20a *g, u32 pg_engine_id);
bool gp106_pmu_is_lpwr_feature_supported(struct gk20a *g, u32 feature_id);
bool gp106_is_lazy_bootstrap(u32 falcon_id);
bool gp106_is_priv_load(u32 falcon_id);
int gp106_load_falcon_ucode(struct gk20a *g, u32 falconidmask);

void gp106_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
bool gp106_pmu_is_engine_in_reset(struct gk20a *g);
int gp106_pmu_engine_reset(struct gk20a *g, bool do_reset);

#endif /*__PMU_GP106_H_*/
