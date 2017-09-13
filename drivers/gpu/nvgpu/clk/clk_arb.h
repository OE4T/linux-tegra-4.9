/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _CLK_ARB_H_
#define _CLK_ARB_H_

struct gk20a;
struct nvgpu_clk_session;

int nvgpu_clk_arb_init_arbiter(struct gk20a *g);

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz);

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *actual_mhz);

int nvgpu_clk_arb_get_arbiter_effective_mhz(struct gk20a *g,
		u32 api_domain, u16 *effective_mhz);

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints);

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g);
bool nvgpu_clk_arb_is_valid_domain(struct gk20a *g, u32 api_domain);

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g);

int nvgpu_clk_arb_install_session_fd(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session);

void nvgpu_clk_arb_release_session(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_commit_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int request_fd);

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		int fd, u32 api_domain, u16 target_mhz);

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *target_mhz);

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd, u32 alarm_mask);

int nvgpu_clk_arb_install_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd);

void nvgpu_clk_arb_schedule_vf_table_update(struct gk20a *g);

int nvgpu_clk_arb_get_current_pstate(struct gk20a *g);

void nvgpu_clk_arb_pstate_change_lock(struct gk20a *g, bool lock);

void nvgpu_clk_arb_schedule_alarm(struct gk20a *g, u32 alarm);
#endif /* _CLK_ARB_H_ */

