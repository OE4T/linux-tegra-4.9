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
#ifndef __NVGPU_CHANNEL_H__
#define __NVGPU_CHANNEL_H__

#include <linux/workqueue.h>
#include <linux/dma-buf.h>

#include <nvgpu/types.h>

struct channel_gk20a;
struct nvgpu_gpfifo;
struct nvgpu_submit_gpfifo_args;
struct nvgpu_channel_fence;
struct gk20a_fence;
struct fifo_profile_gk20a;
struct nvgpu_os_linux;

struct nvgpu_channel_completion_cb {
	/*
	 * Signal channel owner via a callback, if set, in job cleanup with
	 * schedule_work. Means that something finished on the channel (perhaps
	 * more than one job).
	 */
	void (*fn)(struct channel_gk20a *, void *);
	void *user_data;
	/* Make access to the two above atomic */
	struct nvgpu_spinlock lock;
	/* Per-channel async work task, cannot reschedule itself */
	struct work_struct work;
};

struct nvgpu_error_notifier {
	struct dma_buf *dmabuf;
	void *vaddr;

	struct nvgpu_notification *notification;

	struct nvgpu_mutex mutex;
};

struct nvgpu_channel_linux {
	struct channel_gk20a *ch;

	struct nvgpu_channel_completion_cb completion_cb;
	struct nvgpu_error_notifier error_notifier;

	struct dma_buf *cyclestate_buffer_handler;
};

u32 nvgpu_submit_gpfifo_user_flags_to_common_flags(u32 user_flags);
int nvgpu_init_channel_support_linux(struct nvgpu_os_linux *l);
void nvgpu_remove_channel_support_linux(struct nvgpu_os_linux *l);

struct channel_gk20a *gk20a_open_new_channel_with_cb(struct gk20a *g,
		void (*update_fn)(struct channel_gk20a *, void *),
		void *update_fn_data,
		int runlist_id,
		bool is_privileged_channel);

int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvgpu_gpfifo_entry *gpfifo,
				struct nvgpu_submit_gpfifo_args *args,
				u32 num_entries,
				u32 flags,
				struct nvgpu_channel_fence *fence,
				struct gk20a_fence **fence_out,
				bool force_need_sync_fence,
				struct fifo_profile_gk20a *profile);

#endif /* __NVGPU_CHANNEL_H__ */
