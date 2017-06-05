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

#ifndef MC_GK20A_H
#define MC_GK20A_H
struct gk20a;

void gk20a_init_mc(struct gpu_ops *gops);
void mc_gk20a_intr_enable(struct gk20a *g);
void mc_gk20a_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask);
void mc_gk20a_isr_stall(struct gk20a *g);
u32 mc_gk20a_intr_stall(struct gk20a *g);
void mc_gk20a_intr_stall_pause(struct gk20a *g);
void mc_gk20a_intr_stall_resume(struct gk20a *g);
u32 mc_gk20a_intr_nonstall(struct gk20a *g);
void mc_gk20a_intr_nonstall_pause(struct gk20a *g);
void mc_gk20a_intr_nonstall_resume(struct gk20a *g);
void gk20a_mc_enable(struct gk20a *g, u32 units);
void gk20a_mc_disable(struct gk20a *g, u32 units);
void gk20a_mc_reset(struct gk20a *g, u32 units);
u32 gk20a_mc_boot_0(struct gk20a *g, u32 *arch, u32 *impl, u32 *rev);
bool mc_gk20a_is_intr1_pending(struct gk20a *g,
		enum nvgpu_unit unit, u32 mc_intr_1);
#endif
