/*
 *
 * Volta GPU series Subcontext
 *
 * Copyright (c) 2016 - 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.
 */
#ifndef __SUBCONTEXT_GV11B_H__
#define __SUBCONTEXT_GV11B_H__

int gv11b_alloc_subctx_header(struct channel_gk20a *c);

void gv11b_free_subctx_header(struct channel_gk20a *c);

int gv11b_update_subctx_header(struct channel_gk20a *c, u64 gpu_va);

#endif /* __SUBCONTEXT_GV11B_H__ */
