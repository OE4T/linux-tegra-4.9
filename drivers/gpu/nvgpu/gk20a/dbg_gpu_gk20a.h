/*
 * Tegra GK20A GPU Debugger Driver
 *
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef DBG_GPU_GK20A_H
#define DBG_GPU_GK20A_H
#include <linux/poll.h>

/* module debug driver interface */
int gk20a_dbg_gpu_dev_release(struct inode *inode, struct file *filp);
int gk20a_dbg_gpu_dev_open(struct inode *inode, struct file *filp);
long gk20a_dbg_gpu_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
unsigned int gk20a_dbg_gpu_dev_poll(struct file *filep, poll_table *wait);

/* used by profiler driver interface */
int gk20a_prof_gpu_dev_open(struct inode *inode, struct file *filp);

/* used by the interrupt handler to post events */
void gk20a_dbg_gpu_post_events(struct channel_gk20a *fault_ch);

struct channel_gk20a *
nvgpu_dbg_gpu_get_session_channel(struct dbg_session_gk20a *dbg_s);

struct dbg_gpu_session_events {
	wait_queue_head_t wait_queue;
	bool events_enabled;
	int num_pending_events;
};

struct dbg_session_gk20a {
	/* dbg session id used for trace/prints */
	int id;

	/* profiler session, if any */
	bool is_profiler;

	/* has a valid profiler reservation */
	bool has_profiler_reservation;

	/* power enabled or disabled */
	bool is_pg_disabled;

	/* timeouts enabled or disabled */
	bool is_timeout_disabled;

	/*
	 * There can be different versions of the whitelists
	 * between both global and per-context sets; as well
	 * as between debugger and profiler interfaces.
	 */
	struct regops_whitelist *global;
	struct regops_whitelist *per_context;

	/* gpu module vagaries */
	struct device             *dev;
	struct gk20a              *g;

	/* list of bound channels, if any */
	struct nvgpu_list_node ch_list;
	struct nvgpu_mutex ch_list_lock;

	/* event support */
	struct dbg_gpu_session_events dbg_events;

	bool broadcast_stop_trigger;

	struct nvgpu_mutex ioctl_lock;
};

struct dbg_session_data {
	struct dbg_session_gk20a *dbg_s;
	struct nvgpu_list_node dbg_s_entry;
};

static inline struct dbg_session_data *
dbg_session_data_from_dbg_s_entry(struct nvgpu_list_node *node)
{
	return (struct dbg_session_data *)
	     ((uintptr_t)node - offsetof(struct dbg_session_data, dbg_s_entry));
};

struct dbg_session_channel_data {
	/*
	 * We have to keep a ref to the _file_, not the channel, because
	 * close(channel_fd) is synchronous and would deadlock if we had an
	 * open debug session fd holding a channel ref at that time. Holding a
	 * ref to the file makes close(channel_fd) just drop a kernel ref to
	 * the file; the channel will close when the last file ref is dropped.
	 */
	struct file *ch_f;
	int channel_fd;
	int chid;
	struct nvgpu_list_node ch_entry;
	struct dbg_session_data *session_data;
};

static inline struct dbg_session_channel_data *
dbg_session_channel_data_from_ch_entry(struct nvgpu_list_node *node)
{
	return (struct dbg_session_channel_data *)
	((uintptr_t)node - offsetof(struct dbg_session_channel_data, ch_entry));
};

struct dbg_profiler_object_data {
	int session_id;
	u32 prof_handle;
	struct channel_gk20a *ch;
	bool has_reservation;
	struct nvgpu_list_node prof_obj_entry;
};

static inline struct dbg_profiler_object_data *
dbg_profiler_object_data_from_prof_obj_entry(struct nvgpu_list_node *node)
{
	return (struct dbg_profiler_object_data *)
	((uintptr_t)node - offsetof(struct dbg_profiler_object_data, prof_obj_entry));
};

int dbg_unbind_single_channel_gk20a(struct dbg_session_gk20a *dbg_s,
			struct dbg_session_channel_data *ch_data);

bool gk20a_dbg_gpu_broadcast_stop_trigger(struct channel_gk20a *ch);
int gk20a_dbg_gpu_clear_broadcast_stop_trigger(struct channel_gk20a *ch);

int dbg_set_powergate(struct dbg_session_gk20a *dbg_s, u32  powermode);
bool nvgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
bool nvgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
void nvgpu_release_profiler_reservation(struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
int gk20a_perfbuf_enable_locked(struct gk20a *g, u64 offset, u32 size);
int gk20a_perfbuf_disable_locked(struct gk20a *g);

#endif /* DBG_GPU_GK20A_H */
