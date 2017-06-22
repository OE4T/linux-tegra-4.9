/*
 * GK20A L2
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

#ifndef LTC_GK20A_H
#define LTC_GK20A_H
struct gpu_ops;
struct gr_gk20a;
struct zbc_entry;
enum gk20a_cbc_op;

int gk20a_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr);
int gk20a_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
			      u32 min, u32 max);
void gk20a_ltc_init_fs_state(struct gk20a *g);
void gk20a_ltc_isr(struct gk20a *g);
int gk20a_determine_L2_size_bytes(struct gk20a *g);
void gk20a_ltc_set_zbc_color_entry(struct gk20a *g,
					  struct zbc_entry *color_val,
					  u32 index);
void gk20a_ltc_set_zbc_depth_entry(struct gk20a *g,
					  struct zbc_entry *depth_val,
					  u32 index);
void gk20a_ltc_init_cbc(struct gk20a *g, struct gr_gk20a *gr);
#ifdef CONFIG_DEBUG_FS
void gk20a_ltc_sync_debugfs(struct gk20a *g);
#endif
int gk20a_ltc_alloc_phys_cbc(struct gk20a *g, size_t compbit_backing_size);
int gk20a_ltc_alloc_virt_cbc(struct gk20a *g, size_t compbit_backing_size);
#endif
