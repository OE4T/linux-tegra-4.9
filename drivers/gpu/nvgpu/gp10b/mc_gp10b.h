/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

void gp10b_init_mc(struct gpu_ops *gops);
void mc_gp10b_intr_enable(struct gk20a *g);
irqreturn_t mc_gp10b_isr_stall(struct gk20a *g);
irqreturn_t mc_gp10b_isr_nonstall(struct gk20a *g);
irqreturn_t mc_gp10b_intr_thread_stall(struct gk20a *g);
irqreturn_t mc_gp10b_intr_thread_nonstall(struct gk20a *g);
#endif
