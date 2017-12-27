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

#ifndef _FIFO_VGPU_H_
#define _FIFO_VGPU_H_

#include <nvgpu/types.h>

struct gk20a;
struct channel_gk20a;
struct fifo_gk20a;
struct tsg_gk20a;

int vgpu_init_fifo_setup_hw(struct gk20a *g);
void vgpu_channel_bind(struct channel_gk20a *ch);
void vgpu_channel_unbind(struct channel_gk20a *ch);
int vgpu_channel_alloc_inst(struct gk20a *g, struct channel_gk20a *ch);
void vgpu_channel_free_inst(struct gk20a *g, struct channel_gk20a *ch);
void vgpu_channel_enable(struct channel_gk20a *ch);
void vgpu_channel_disable(struct channel_gk20a *ch);
int vgpu_channel_setup_ramfc(struct channel_gk20a *ch, u64 gpfifo_base,
				u32 gpfifo_entries,
				unsigned long acquire_timeout, u32 flags);
int vgpu_fifo_init_engine_info(struct fifo_gk20a *f);
int vgpu_fifo_preempt_channel(struct gk20a *g, u32 chid);
int vgpu_fifo_preempt_tsg(struct gk20a *g, u32 tsgid);
int vgpu_fifo_update_runlist(struct gk20a *g, u32 runlist_id,
				u32 chid, bool add, bool wait_for_finish);
int vgpu_fifo_wait_engine_idle(struct gk20a *g);
int vgpu_fifo_set_runlist_interleave(struct gk20a *g,
					u32 id,
					u32 runlist_id,
					u32 new_level);
int vgpu_channel_set_timeslice(struct channel_gk20a *ch, u32 timeslice);
int vgpu_fifo_force_reset_ch(struct channel_gk20a *ch,
					u32 err_code, bool verbose);
u32 vgpu_fifo_default_timeslice_us(struct gk20a *g);
int vgpu_tsg_open(struct tsg_gk20a *tsg);
void vgpu_tsg_release(struct tsg_gk20a *tsg);
int vgpu_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch);
int vgpu_tsg_unbind_channel(struct channel_gk20a *ch);
int vgpu_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice);
int vgpu_enable_tsg(struct tsg_gk20a *tsg);

#endif
