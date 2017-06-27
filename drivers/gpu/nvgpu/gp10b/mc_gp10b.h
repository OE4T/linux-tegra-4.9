/*
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

#ifndef MC_GP20B_H
#define MC_GP20B_H
struct gk20a;

enum MC_INTERRUPT_REGLIST {
	NVGPU_MC_INTR_STALLING = 0,
	NVGPU_MC_INTR_NONSTALLING,
};

void mc_gp10b_intr_enable(struct gk20a *g);
void mc_gp10b_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask);
void mc_gp10b_isr_stall(struct gk20a *g);
bool mc_gp10b_is_intr1_pending(struct gk20a *g,
				      enum nvgpu_unit unit, u32 mc_intr_1);

u32 mc_gp10b_intr_stall(struct gk20a *g);
void mc_gp10b_intr_stall_pause(struct gk20a *g);
void mc_gp10b_intr_stall_resume(struct gk20a *g);
u32 mc_gp10b_intr_nonstall(struct gk20a *g);
void mc_gp10b_intr_nonstall_pause(struct gk20a *g);
void mc_gp10b_intr_nonstall_resume(struct gk20a *g);

#endif
