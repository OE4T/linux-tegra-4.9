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

#ifndef LTC_GV11B_H
#define LTC_GV11B_H
struct gk20a;

void gv11b_ltc_set_zbc_stencil_entry(struct gk20a *g,
					  struct zbc_entry *stencil_val,
					  u32 index);
void gv11b_ltc_init_fs_state(struct gk20a *g);
void gv11b_ltc_isr(struct gk20a *g);
u32 gv11b_ltc_cbc_fix_config(struct gk20a *g, int base);

#endif
