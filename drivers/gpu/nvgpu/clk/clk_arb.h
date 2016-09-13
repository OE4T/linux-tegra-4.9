/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

#ifndef _CLK_ARB_H_
#define _CLK_ARB_H_

struct nvgpu_clk_arb;
struct nvgpu_clk_session;

int nvgpu_clk_arb_init_arbiter(struct gk20a *g);

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz);

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *actual_mhz);

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints);

u32 nvgpu_clk_arb_get_arbiter_req_nr(struct gk20a *g);

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g);

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g);

int nvgpu_clk_arb_install_session_fd(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session);

void nvgpu_clk_arb_cleanup_session(struct gk20a *g,
		struct nvgpu_clk_session *session);

void nvgpu_clk_arb_apply_session_constraints(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 target_mhz);

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *target_mhz);

u32 nvgpu_clk_arb_get_session_req_nr(struct gk20a *g,
		struct nvgpu_clk_session *session);



#endif /* _CLK_ARB_H_ */

