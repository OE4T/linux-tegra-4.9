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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __FECS_TRACE_VGPU_H
#define __FECS_TRACE_VGPU_H

#include <nvgpu/types.h>

struct gk20a;
struct vm_area_struct;
struct nvgpu_ctxsw_trace_filter;

void vgpu_fecs_trace_data_update(struct gk20a *g);
int vgpu_fecs_trace_init(struct gk20a *g);
int vgpu_fecs_trace_deinit(struct gk20a *g);
int vgpu_fecs_trace_enable(struct gk20a *g);
int vgpu_fecs_trace_disable(struct gk20a *g);
bool vgpu_fecs_trace_is_enabled(struct gk20a *g);
int vgpu_fecs_trace_poll(struct gk20a *g);
int vgpu_alloc_user_buffer(struct gk20a *g, void **buf, size_t *size);
int vgpu_free_user_buffer(struct gk20a *g);
int vgpu_mmap_user_buffer(struct gk20a *g, struct vm_area_struct *vma);
int vgpu_fecs_trace_max_entries(struct gk20a *g,
			struct nvgpu_ctxsw_trace_filter *filter);
int vgpu_fecs_trace_set_filter(struct gk20a *g,
			struct nvgpu_ctxsw_trace_filter *filter);

#endif /* __FECS_TRACE_VGPU_H */
