/*
 * drivers/video/tegra/host/gk20a/channel_gk20a.h
 *
 * GK20A graphics channel
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef __CHANNEL_GK20A_H__
#define __CHANNEL_GK20A_H__

#include <linux/log2.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/nvhost_ioctl.h>
struct gk20a;
struct gr_gk20a;
struct dbg_session_gk20a;

#include "channel_sync_gk20a.h"

#include "mm_gk20a.h"
#include "gr_gk20a.h"

struct gpfifo {
	u32 entry0;
	u32 entry1;
};

struct notification {
	struct {
		u32 nanoseconds[2];
	} timestamp;
	u32 info32;
	u16 info16;
	u16 status;
};

struct fence {
	u32 hw_chid;
	u32 syncpt_val;
};

/* contexts associated with a channel */
struct channel_ctx_gk20a {
	struct gr_ctx_desc	*gr_ctx;
	struct pm_ctx_desc	pm_ctx;
	struct patch_desc	patch_ctx;
	struct zcull_ctx_desc	zcull_ctx;
	u64	global_ctx_buffer_va[NR_GLOBAL_CTX_BUF_VA];
	u64	global_ctx_buffer_size[NR_GLOBAL_CTX_BUF_VA];
	bool	global_ctx_buffer_mapped;
};

struct channel_gk20a_job {
	struct mapped_buffer_node **mapped_buffers;
	int num_mapped_buffers;
	struct gk20a_channel_fence pre_fence;
	struct gk20a_channel_fence post_fence;
	struct list_head list;
};

/* this is the priv element of struct nvhost_channel */
struct channel_gk20a {
	struct gk20a *g;
	bool in_use;
	int hw_chid;
	bool bound;
	bool first_init;
	bool vpr;
	pid_t pid;

	int tsgid;
	struct list_head ch_entry; /* channel's entry in TSG */

	struct list_head jobs;
	struct mutex jobs_lock;
	struct mutex submit_lock;

	struct vm_gk20a *vm;

	struct gpfifo_desc gpfifo;

	struct channel_ctx_gk20a ch_ctx;

	struct inst_desc inst_block;
	struct mem_desc_sub ramfc;

	void *userd_cpu_va;
	u64 userd_iova;
	u64 userd_gpu_va;

	s32 num_objects;
	u32 obj_class;	/* we support only one obj per channel */

	struct priv_cmd_queue priv_cmd_q;

	wait_queue_head_t notifier_wq;
	wait_queue_head_t semaphore_wq;
	wait_queue_head_t submit_wq;

	u32 timeout_accumulated_ms;
	u32 timeout_gpfifo_get;

	bool cmds_pending;
	struct {
		struct gk20a_channel_fence pre_fence;
		struct gk20a_channel_fence post_fence;
	} last_submit;

	void (*remove_support)(struct channel_gk20a *);
#if defined(CONFIG_GK20A_CYCLE_STATS)
	struct {
	void *cyclestate_buffer;
	u32 cyclestate_buffer_size;
	struct dma_buf *cyclestate_buffer_handler;
	struct mutex cyclestate_buffer_mutex;
	} cyclestate;
#endif
	struct mutex dbg_s_lock;
	struct list_head dbg_s_list;

	bool has_timedout;
	u32 timeout_ms_max;
	bool timeout_debug_dump;

	struct dma_buf *error_notifier_ref;
	struct nvhost_notification *error_notifier;
	void *error_notifier_va;

	struct gk20a_channel_sync *sync;
};

static inline bool gk20a_channel_as_bound(struct channel_gk20a *ch)
{
	return !!ch->vm;
}
int channel_gk20a_commit_va(struct channel_gk20a *c);
int gk20a_init_channel_support(struct gk20a *, u32 chid);
void gk20a_free_channel(struct channel_gk20a *ch, bool finish);
bool gk20a_channel_update_and_check_timeout(struct channel_gk20a *ch,
					    u32 timeout_delta_ms);
void gk20a_disable_channel(struct channel_gk20a *ch,
			   bool wait_for_finish,
			   unsigned long finish_timeout);
void gk20a_channel_abort(struct channel_gk20a *ch);
int gk20a_channel_finish(struct channel_gk20a *ch, unsigned long timeout);
void gk20a_set_error_notifier(struct channel_gk20a *ch, __u32 error);
void gk20a_channel_semaphore_wakeup(struct gk20a *g);
int gk20a_channel_alloc_priv_cmdbuf(struct channel_gk20a *c, u32 size,
			     struct priv_cmd_entry **entry);

int gk20a_channel_suspend(struct gk20a *g);
int gk20a_channel_resume(struct gk20a *g);

/* Channel file operations */
int gk20a_channel_open(struct inode *inode, struct file *filp);
long gk20a_channel_ioctl(struct file *filp,
			 unsigned int cmd,
			 unsigned long arg);
int gk20a_channel_release(struct inode *inode, struct file *filp);
struct channel_gk20a *gk20a_get_channel_from_file(int fd);
void gk20a_channel_update(struct channel_gk20a *c, int nr_completed);

void gk20a_init_channel(struct gpu_ops *gops);

int gk20a_wait_channel_idle(struct channel_gk20a *ch);
struct channel_gk20a *gk20a_open_new_channel(struct gk20a *g);
void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a);

int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvhost_gpfifo *gpfifo,
				u32 num_entries,
				struct nvhost_fence *fence,
				u32 flags);

int gk20a_alloc_channel_gpfifo(struct channel_gk20a *c,
			       struct nvhost_alloc_gpfifo_args *args);

#endif /*__CHANNEL_GK20A_H__*/
