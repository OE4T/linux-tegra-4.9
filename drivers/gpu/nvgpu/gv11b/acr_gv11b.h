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

#ifndef __ACR_GV11B_H_
#define __ACR_GV11B_H_


int gv11b_bootstrap_hs_flcn(struct gk20a *g);
int gv11b_init_pmu_setup_hw1(struct gk20a *g,
		void *desc, u32 bl_sz);
#endif /*__PMU_GP106_H_*/
