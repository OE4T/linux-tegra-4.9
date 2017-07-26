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

#ifndef __FECS_TRACE_GK20A_H
#define __FECS_TRACE_GK20A_H

struct gk20a;
struct channel_gk20a;
struct nvgpu_ctxsw_trace_filter;

int gk20a_fecs_trace_poll(struct gk20a *g);
int gk20a_fecs_trace_init(struct gk20a *g);
int gk20a_fecs_trace_bind_channel(struct gk20a *g,
		struct channel_gk20a *ch);
int gk20a_fecs_trace_unbind_channel(struct gk20a *g, struct channel_gk20a *ch);
int gk20a_fecs_trace_reset(struct gk20a *g);
int gk20a_fecs_trace_deinit(struct gk20a *g);
int gk20a_gr_max_entries(struct gk20a *g,
		struct nvgpu_ctxsw_trace_filter *filter);
int gk20a_fecs_trace_enable(struct gk20a *g);
int gk20a_fecs_trace_disable(struct gk20a *g);
bool gk20a_fecs_trace_is_enabled(struct gk20a *g);

#endif /* __FECS_TRACE_GK20A_H */
