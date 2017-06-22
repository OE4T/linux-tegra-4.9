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

#ifndef FB_GK20A_H
#define FB_GK20A_H

struct gk20a;
struct nvgpu_mem;

void fb_gk20a_reset(struct gk20a *g);
void gk20a_fb_init_hw(struct gk20a *g);
void gk20a_fb_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb);

#endif
