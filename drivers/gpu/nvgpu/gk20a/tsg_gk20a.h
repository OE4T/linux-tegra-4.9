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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __TSG_GK20A_H_
#define __TSG_GK20A_H_

#include <nvgpu/lock.h>
#include <nvgpu/kref.h>

#define NVGPU_INVALID_TSG_ID (-1)

struct channel_gk20a;

bool gk20a_is_channel_marked_as_tsg(struct channel_gk20a *ch);
struct tsg_gk20a *gk20a_tsg_open(struct gk20a *g);
void gk20a_tsg_release(struct nvgpu_ref *ref);

int gk20a_init_tsg_support(struct gk20a *g, u32 tsgid);
struct tsg_gk20a *tsg_gk20a_from_ch(struct channel_gk20a *ch);

struct tsg_gk20a {
	struct gk20a *g;

	bool in_use;
	int tsgid;

	struct nvgpu_ref refcount;

	struct nvgpu_list_node ch_list;
	int num_active_channels;
	struct rw_semaphore ch_list_lock;

	unsigned int timeslice_us;
	unsigned int timeslice_timeout;
	unsigned int timeslice_scale;

	struct gr_ctx_desc *tsg_gr_ctx;

	struct vm_gk20a *vm;

	u32 interleave_level;

	struct nvgpu_list_node event_id_list;
	struct nvgpu_mutex event_id_list_lock;

	u32 runlist_id;
	pid_t tgid;
	struct nvgpu_mem *eng_method_buffers;
};

int gk20a_enable_tsg(struct tsg_gk20a *tsg);
int gk20a_disable_tsg(struct tsg_gk20a *tsg);
int gk20a_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch);
int gk20a_tsg_unbind_channel(struct channel_gk20a *ch);

void gk20a_tsg_event_id_post_event(struct tsg_gk20a *tsg,
				       int event_id);
int gk20a_tsg_set_runlist_interleave(struct tsg_gk20a *tsg, u32 level);
int gk20a_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice);
u32 gk20a_tsg_get_timeslice(struct tsg_gk20a *tsg);
int gk20a_tsg_set_priority(struct gk20a *g, struct tsg_gk20a *tsg,
				u32 priority);


#endif /* __TSG_GK20A_H_ */
