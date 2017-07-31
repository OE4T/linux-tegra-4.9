/*
 * GM20B FB
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

#ifndef _NVHOST_GM20B_FB
#define _NVHOST_GM20B_FB
struct gk20a;

void fb_gm20b_init_fs_state(struct gk20a *g);
void gm20b_fb_set_mmu_page_size(struct gk20a *g);
bool gm20b_fb_set_use_full_comp_tag_line(struct gk20a *g);
unsigned int gm20b_fb_compression_page_size(struct gk20a *g);
unsigned int gm20b_fb_compressible_page_size(struct gk20a *g);
void gm20b_fb_dump_vpr_wpr_info(struct gk20a *g);
int gm20b_fb_vpr_info_fetch(struct gk20a *g);
bool gm20b_fb_debug_mode_enabled(struct gk20a *g);
void gm20b_fb_set_debug_mode(struct gk20a *g, bool enable);

void gm20b_init_uncompressed_kind_map(void);
void gm20b_init_kind_attr(void);
#endif
