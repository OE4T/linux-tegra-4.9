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

void gp106_init_pmu_ops(struct gk20a *g);
void gp106_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
bool gp106_pmu_is_engine_in_reset(struct gk20a *g);
int gp106_pmu_engine_reset(struct gk20a *g, bool do_reset);

#endif /*__PMU_GP106_H_*/
