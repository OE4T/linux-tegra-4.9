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

#ifndef MC_GV11B_H
#define MC_GV11B_H
struct gk20a;

void mc_gv11b_intr_enable(struct gk20a *g);
bool gv11b_mc_is_intr_hub_pending(struct gk20a *g, u32 mc_intr_0);
#endif
