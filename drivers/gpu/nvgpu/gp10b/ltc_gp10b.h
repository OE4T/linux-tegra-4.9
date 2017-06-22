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

#ifndef LTC_GP10B_H
#define LTC_GP10B_H
struct gpu_ops;

void gp10b_ltc_isr(struct gk20a *g);

int gp10b_determine_L2_size_bytes(struct gk20a *g);
int gp10b_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr);
void gp10b_ltc_init_fs_state(struct gk20a *g);
#ifdef CONFIG_DEBUG_FS
void gp10b_ltc_sync_debugfs(struct gk20a *g);
#endif
#endif
