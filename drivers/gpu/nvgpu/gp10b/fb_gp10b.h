/*
 * GP10B FB
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

#ifndef _NVGPU_GP10B_FB
#define _NVGPU_GP10B_FB
struct gk20a;

noinline_for_stack void gp10b_init_uncompressed_kind_map(void);
void gp10b_init_kind_attr(void);
unsigned int gp10b_fb_compression_page_size(struct gk20a *g);
unsigned int gp10b_fb_compressible_page_size(struct gk20a *g);

#endif
