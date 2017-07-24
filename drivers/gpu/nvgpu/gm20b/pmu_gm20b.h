/*
 * GM20B PMU
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __PMU_GM20B_H_
#define __PMU_GM20B_H_

struct gk20a;

void gm20b_init_pmu_ops(struct gk20a *g);
void gm20b_pmu_load_lsf(struct gk20a *g, u32 falcon_id, u32 flags);
int gm20b_pmu_init_acr(struct gk20a *g);
void gm20b_write_dmatrfbase(struct gk20a *g, u32 addr);

#endif /*__PMU_GM20B_H_*/
