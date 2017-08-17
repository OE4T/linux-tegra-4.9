/*
 * GP106 GPU GR
 *
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

#ifndef _NVGPU_GR_GP106_H_
#define _NVGPU_GR_GP106_H_

struct gk20a;

enum {
	PASCAL_B                 = 0xC197,
	PASCAL_COMPUTE_B         = 0xC1C0,
};

bool gr_gp106_is_valid_class(struct gk20a *g, u32 class_num);
u32 gr_gp106_pagepool_default_size(struct gk20a *g);
int gr_gp106_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data);
void gr_gp106_cb_size_default(struct gk20a *g);
int gr_gp106_set_ctxsw_preemption_mode(struct gk20a *g,
				struct gr_ctx_desc *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode);

#endif
