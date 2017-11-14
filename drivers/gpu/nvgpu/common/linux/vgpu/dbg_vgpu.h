/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _DBG_VGPU_H_
#define _DBG_VGPU_H_

struct dbg_session_gk20a;
struct nvgpu_dbg_gpu_reg_op;
struct dbg_profiler_object_data;
struct gk20a;

int vgpu_exec_regops(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_gpu_reg_op *ops,
		      u64 num_ops);
int vgpu_dbg_set_powergate(struct dbg_session_gk20a *dbg_s, bool disable_powergate);
bool vgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
bool vgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);

void vgpu_release_profiler_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
int vgpu_perfbuffer_enable(struct gk20a *g, u64 offset, u32 size);
int vgpu_perfbuffer_disable(struct gk20a *g);
#endif
