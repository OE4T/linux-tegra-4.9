/*
 * GK20A Graphics channel
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <trace/events/gk20a.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/dma-buf.h>

#include <nvgpu/semaphore.h>
#include <nvgpu/timers.h>
#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/list.h>
#include <nvgpu/debug.h>

#include "gk20a/gk20a.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/fence_gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "ioctl_channel.h"
#include "os_linux.h"

static void gk20a_channel_trace_sched_param(
	void (*trace)(int chid, int tsgid, pid_t pid, u32 timeslice,
		u32 timeout, const char *interleave,
		const char *graphics_preempt_mode,
		const char *compute_preempt_mode),
	struct channel_gk20a *ch)
{
	(trace)(ch->chid, ch->tsgid, ch->pid,
		gk20a_is_channel_marked_as_tsg(ch) ?
			tsg_gk20a_from_ch(ch)->timeslice_us : ch->timeslice_us,
		ch->timeout_ms_max,
		gk20a_fifo_interleave_level_name(ch->interleave_level),
		gr_gk20a_graphics_preempt_mode_name(ch->ch_ctx.gr_ctx ?
			ch->ch_ctx.gr_ctx->graphics_preempt_mode : 0),
		gr_gk20a_compute_preempt_mode_name(ch->ch_ctx.gr_ctx ?
			ch->ch_ctx.gr_ctx->compute_preempt_mode : 0));
}

/*
 * Although channels do have pointers back to the gk20a struct that they were
 * created under in cases where the driver is killed that pointer can be bad.
 * The channel memory can be freed before the release() function for a given
 * channel is called. This happens when the driver dies and userspace doesn't
 * get a chance to call release() until after the entire gk20a driver data is
 * unloaded and freed.
 */
struct channel_priv {
	struct gk20a *g;
	struct channel_gk20a *c;
};

#if defined(CONFIG_GK20A_CYCLE_STATS)

static int gk20a_channel_cycle_stats(struct channel_gk20a *ch,
		       struct nvgpu_cycle_stats_args *args)
{
	struct dma_buf *dmabuf;
	void *virtual_address;

	/* is it allowed to handle calls for current GPU? */
	if (0 == (ch->g->gpu_characteristics.flags &
			NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS))
		return -ENOSYS;

	if (args->dmabuf_fd && !ch->cyclestate.cyclestate_buffer_handler) {

		/* set up new cyclestats buffer */
		dmabuf = dma_buf_get(args->dmabuf_fd);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		virtual_address = dma_buf_vmap(dmabuf);
		if (!virtual_address)
			return -ENOMEM;

		ch->cyclestate.cyclestate_buffer_handler = dmabuf;
		ch->cyclestate.cyclestate_buffer = virtual_address;
		ch->cyclestate.cyclestate_buffer_size = dmabuf->size;
		return 0;

	} else if (!args->dmabuf_fd &&
			ch->cyclestate.cyclestate_buffer_handler) {
		gk20a_channel_free_cycle_stats_buffer(ch);
		return 0;

	} else if (!args->dmabuf_fd &&
			!ch->cyclestate.cyclestate_buffer_handler) {
		/* no requst from GL */
		return 0;

	} else {
		pr_err("channel already has cyclestats buffer\n");
		return -EINVAL;
	}
}

static int gk20a_flush_cycle_stats_snapshot(struct channel_gk20a *ch)
{
	int ret;

	nvgpu_mutex_acquire(&ch->cs_client_mutex);
	if (ch->cs_client)
		ret = gr_gk20a_css_flush(ch, ch->cs_client);
	else
		ret = -EBADF;
	nvgpu_mutex_release(&ch->cs_client_mutex);

	return ret;
}

static int gk20a_attach_cycle_stats_snapshot(struct channel_gk20a *ch,
				u32 dmabuf_fd,
				u32 perfmon_id_count,
				u32 *perfmon_id_start)
{
	int ret;

	nvgpu_mutex_acquire(&ch->cs_client_mutex);
	if (ch->cs_client) {
		ret = -EEXIST;
	} else {
		ret = gr_gk20a_css_attach(ch,
					dmabuf_fd,
					perfmon_id_count,
					perfmon_id_start,
					&ch->cs_client);
	}
	nvgpu_mutex_release(&ch->cs_client_mutex);

	return ret;
}

static int gk20a_channel_cycle_stats_snapshot(struct channel_gk20a *ch,
			struct nvgpu_cycle_stats_snapshot_args *args)
{
	int ret;

	/* is it allowed to handle calls for current GPU? */
	if (0 == (ch->g->gpu_characteristics.flags &
			NVGPU_GPU_FLAGS_SUPPORT_CYCLE_STATS_SNAPSHOT))
		return -ENOSYS;

	if (!args->dmabuf_fd)
		return -EINVAL;

	/* handle the command (most frequent cases first) */
	switch (args->cmd) {
	case NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT_CMD_FLUSH:
		ret = gk20a_flush_cycle_stats_snapshot(ch);
		args->extra = 0;
		break;

	case NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT_CMD_ATTACH:
		ret = gk20a_attach_cycle_stats_snapshot(ch,
						args->dmabuf_fd,
						args->extra,
						&args->extra);
		break;

	case NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT_CMD_DETACH:
		ret = gk20a_channel_free_cycle_stats_snapshot(ch);
		args->extra = 0;
		break;

	default:
		pr_err("cyclestats: unknown command %u\n", args->cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static int gk20a_channel_set_wdt_status(struct channel_gk20a *ch,
		struct nvgpu_channel_wdt_args *args)
{
	if (args->wdt_status == NVGPU_IOCTL_CHANNEL_DISABLE_WDT)
		ch->wdt_enabled = false;
	else if (args->wdt_status == NVGPU_IOCTL_CHANNEL_ENABLE_WDT)
		ch->wdt_enabled = true;

	return 0;
}

static void gk20a_channel_free_error_notifiers(struct channel_gk20a *ch)
{
	nvgpu_mutex_acquire(&ch->error_notifier_mutex);
	if (ch->error_notifier_ref) {
		dma_buf_vunmap(ch->error_notifier_ref, ch->error_notifier_va);
		dma_buf_put(ch->error_notifier_ref);
		ch->error_notifier_ref = NULL;
		ch->error_notifier = NULL;
		ch->error_notifier_va = NULL;
	}
	nvgpu_mutex_release(&ch->error_notifier_mutex);
}

static int gk20a_init_error_notifier(struct channel_gk20a *ch,
		struct nvgpu_set_error_notifier *args)
{
	struct dma_buf *dmabuf;
	void *va;
	u64 end = args->offset + sizeof(struct nvgpu_notification);

	if (!args->mem) {
		pr_err("gk20a_init_error_notifier: invalid memory handle\n");
		return -EINVAL;
	}

	dmabuf = dma_buf_get(args->mem);

	gk20a_channel_free_error_notifiers(ch);

	if (IS_ERR(dmabuf)) {
		pr_err("Invalid handle: %d\n", args->mem);
		return -EINVAL;
	}

	if (end > dmabuf->size || end < sizeof(struct nvgpu_notification)) {
		dma_buf_put(dmabuf);
		nvgpu_err(ch->g, "gk20a_init_error_notifier: invalid offset");
		return -EINVAL;
	}

	/* map handle */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		pr_err("Cannot map notifier handle\n");
		return -ENOMEM;
	}

	ch->error_notifier = va + args->offset;
	ch->error_notifier_va = va;
	memset(ch->error_notifier, 0, sizeof(struct nvgpu_notification));

	/* set channel notifiers pointer */
	nvgpu_mutex_acquire(&ch->error_notifier_mutex);
	ch->error_notifier_ref = dmabuf;
	nvgpu_mutex_release(&ch->error_notifier_mutex);

	return 0;
}

/*
 * This returns the channel with a reference. The caller must
 * gk20a_channel_put() the ref back after use.
 *
 * NULL is returned if the channel was not found.
 */
struct channel_gk20a *gk20a_get_channel_from_file(int fd)
{
	struct channel_gk20a *ch;
	struct channel_priv *priv;
	struct file *f = fget(fd);

	if (!f)
		return NULL;

	if (f->f_op != &gk20a_channel_ops) {
		fput(f);
		return NULL;
	}

	priv = (struct channel_priv *)f->private_data;
	ch = gk20a_channel_get(priv->c);
	fput(f);
	return ch;
}

int gk20a_channel_release(struct inode *inode, struct file *filp)
{
	struct channel_priv *priv = filp->private_data;
	struct channel_gk20a *ch;
	struct gk20a *g;

	int err;

	/* We could still end up here even if the channel_open failed, e.g.
	 * if we ran out of hw channel IDs.
	 */
	if (!priv)
		return 0;

	ch = priv->c;
	g = priv->g;

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to release a channel!");
		goto channel_release;
	}

	trace_gk20a_channel_release(dev_name(dev_from_gk20a(g)));

	gk20a_channel_close(ch);
	gk20a_channel_free_error_notifiers(ch);

	gk20a_idle(g);

channel_release:
	gk20a_put(g);
	nvgpu_kfree(g, filp->private_data);
	filp->private_data = NULL;
	return 0;
}

/* note: runlist_id -1 is synonym for the ENGINE_GR_GK20A runlist id */
static int __gk20a_channel_open(struct gk20a *g,
				struct file *filp, s32 runlist_id)
{
	int err;
	struct channel_gk20a *ch;
	struct channel_priv *priv;

	gk20a_dbg_fn("");

	g = gk20a_get(g);
	if (!g)
		return -ENODEV;

	trace_gk20a_channel_open(dev_name(dev_from_gk20a(g)));

	priv = nvgpu_kzalloc(g, sizeof(*priv));
	if (!priv) {
		err = -ENOMEM;
		goto free_ref;
	}

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on, %d", err);
		goto fail_busy;
	}
	/* All the user space channel should be non privilege */
	ch = gk20a_open_new_channel(g, runlist_id, false);
	gk20a_idle(g);
	if (!ch) {
		nvgpu_err(g,
			"failed to get f");
		err = -ENOMEM;
		goto fail_busy;
	}

	gk20a_channel_trace_sched_param(
		trace_gk20a_channel_sched_defaults, ch);

	priv->g = g;
	priv->c = ch;

	filp->private_data = priv;
	return 0;

fail_busy:
	nvgpu_kfree(g, priv);
free_ref:
	gk20a_put(g);
	return err;
}

int gk20a_channel_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l = container_of(inode->i_cdev,
			struct nvgpu_os_linux, channel.cdev);
	struct gk20a *g = &l->g;
	int ret;

	gk20a_dbg_fn("start");
	ret = __gk20a_channel_open(g, filp, -1);

	gk20a_dbg_fn("end");
	return ret;
}

int gk20a_channel_open_ioctl(struct gk20a *g,
		struct nvgpu_channel_open_args *args)
{
	int err;
	int fd;
	struct file *file;
	char name[64];
	s32 runlist_id = args->in.runlist_id;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	snprintf(name, sizeof(name), "nvhost-%s-fd%d",
		 dev_name(dev_from_gk20a(g)), fd);

	file = anon_inode_getfile(name, l->channel.cdev.ops, NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	err = __gk20a_channel_open(g, file, runlist_id);
	if (err)
		goto clean_up_file;

	fd_install(fd, file);
	args->out.channel_fd = fd;
	return 0;

clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(fd);
	return err;
}

int nvgpu_channel_ioctl_alloc_gpfifo(struct channel_gk20a *c,
		struct nvgpu_alloc_gpfifo_ex_args *args)
{
	return gk20a_channel_alloc_gpfifo(c, args->num_entries,
			args->num_inflight_jobs,
			args->flags);
}


static int gk20a_channel_wait_semaphore(struct channel_gk20a *ch,
					ulong id, u32 offset,
					u32 payload, u32 timeout)
{
	struct dma_buf *dmabuf;
	void *data;
	u32 *semaphore;
	int ret = 0;

	/* do not wait if channel has timed out */
	if (ch->has_timedout)
		return -ETIMEDOUT;

	dmabuf = dma_buf_get(id);
	if (IS_ERR(dmabuf)) {
		nvgpu_err(ch->g, "invalid notifier nvmap handle 0x%lx", id);
		return -EINVAL;
	}

	data = dma_buf_kmap(dmabuf, offset >> PAGE_SHIFT);
	if (!data) {
		nvgpu_err(ch->g, "failed to map notifier memory");
		ret = -EINVAL;
		goto cleanup_put;
	}

	semaphore = data + (offset & ~PAGE_MASK);

	ret = NVGPU_COND_WAIT_INTERRUPTIBLE(
			&ch->semaphore_wq,
			*semaphore == payload || ch->has_timedout,
			timeout);

	dma_buf_kunmap(dmabuf, offset >> PAGE_SHIFT, data);
cleanup_put:
	dma_buf_put(dmabuf);
	return ret;
}

static int gk20a_channel_wait(struct channel_gk20a *ch,
			      struct nvgpu_wait_args *args)
{
	struct dma_buf *dmabuf;
	struct gk20a *g = ch->g;
	struct notification *notif;
	struct timespec tv;
	u64 jiffies;
	ulong id;
	u32 offset;
	int remain, ret = 0;
	u64 end;

	gk20a_dbg_fn("");

	if (ch->has_timedout)
		return -ETIMEDOUT;

	switch (args->type) {
	case NVGPU_WAIT_TYPE_NOTIFIER:
		id = args->condition.notifier.dmabuf_fd;
		offset = args->condition.notifier.offset;
		end = offset + sizeof(struct notification);

		dmabuf = dma_buf_get(id);
		if (IS_ERR(dmabuf)) {
			nvgpu_err(g, "invalid notifier nvmap handle 0x%lx",
				   id);
			return -EINVAL;
		}

		if (end > dmabuf->size || end < sizeof(struct notification)) {
			dma_buf_put(dmabuf);
			nvgpu_err(g, "invalid notifier offset");
			return -EINVAL;
		}

		notif = dma_buf_vmap(dmabuf);
		if (!notif) {
			nvgpu_err(g, "failed to map notifier memory");
			return -ENOMEM;
		}

		notif = (struct notification *)((uintptr_t)notif + offset);

		/* user should set status pending before
		 * calling this ioctl */
		remain = NVGPU_COND_WAIT_INTERRUPTIBLE(
				&ch->notifier_wq,
				notif->status == 0 || ch->has_timedout,
				args->timeout);

		if (remain == 0 && notif->status != 0) {
			ret = -ETIMEDOUT;
			goto notif_clean_up;
		} else if (remain < 0) {
			ret = -EINTR;
			goto notif_clean_up;
		}

		/* TBD: fill in correct information */
		jiffies = get_jiffies_64();
		jiffies_to_timespec(jiffies, &tv);
		notif->timestamp.nanoseconds[0] = tv.tv_nsec;
		notif->timestamp.nanoseconds[1] = tv.tv_sec;
		notif->info32 = 0xDEADBEEF; /* should be object name */
		notif->info16 = ch->chid; /* should be method offset */

notif_clean_up:
		dma_buf_vunmap(dmabuf, notif);
		return ret;

	case NVGPU_WAIT_TYPE_SEMAPHORE:
		ret = gk20a_channel_wait_semaphore(ch,
				args->condition.semaphore.dmabuf_fd,
				args->condition.semaphore.offset,
				args->condition.semaphore.payload,
				args->timeout);

		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int gk20a_event_id_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct gk20a_event_id_data *event_id_data = filep->private_data;
	struct gk20a *g = event_id_data->g;
	u32 event_id = event_id_data->event_id;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_info, "");

	poll_wait(filep, &event_id_data->event_id_wq, wait);

	nvgpu_mutex_acquire(&event_id_data->lock);

	if (event_id_data->is_tsg) {
		struct tsg_gk20a *tsg = g->fifo.tsg + event_id_data->id;

		if (event_id_data->event_posted) {
			gk20a_dbg_info(
				"found pending event_id=%d on TSG=%d\n",
				event_id, tsg->tsgid);
			mask = (POLLPRI | POLLIN);
			event_id_data->event_posted = false;
		}
	} else {
		struct channel_gk20a *ch = g->fifo.channel
					   + event_id_data->id;

		if (event_id_data->event_posted) {
			gk20a_dbg_info(
				"found pending event_id=%d on chid=%d\n",
				event_id, ch->chid);
			mask = (POLLPRI | POLLIN);
			event_id_data->event_posted = false;
		}
	}

	nvgpu_mutex_release(&event_id_data->lock);

	return mask;
}

static int gk20a_event_id_release(struct inode *inode, struct file *filp)
{
	struct gk20a_event_id_data *event_id_data = filp->private_data;
	struct gk20a *g = event_id_data->g;

	if (event_id_data->is_tsg) {
		struct tsg_gk20a *tsg = g->fifo.tsg + event_id_data->id;

		nvgpu_mutex_acquire(&tsg->event_id_list_lock);
		nvgpu_list_del(&event_id_data->event_id_node);
		nvgpu_mutex_release(&tsg->event_id_list_lock);
	} else {
		struct channel_gk20a *ch = g->fifo.channel + event_id_data->id;

		nvgpu_mutex_acquire(&ch->event_id_list_lock);
		nvgpu_list_del(&event_id_data->event_id_node);
		nvgpu_mutex_release(&ch->event_id_list_lock);
	}

	nvgpu_mutex_destroy(&event_id_data->lock);
	gk20a_put(g);
	nvgpu_kfree(g, event_id_data);
	filp->private_data = NULL;

	return 0;
}

const struct file_operations gk20a_event_id_ops = {
	.owner = THIS_MODULE,
	.poll = gk20a_event_id_poll,
	.release = gk20a_event_id_release,
};

static int gk20a_channel_get_event_data_from_id(struct channel_gk20a *ch,
				u32 event_id,
				struct gk20a_event_id_data **event_id_data)
{
	struct gk20a_event_id_data *local_event_id_data;
	bool event_found = false;

	nvgpu_mutex_acquire(&ch->event_id_list_lock);
	list_for_each_entry(local_event_id_data, &ch->event_id_list,
						 event_id_node) {
		if (local_event_id_data->event_id == event_id) {
			event_found = true;
			break;
		}
	}
	nvgpu_mutex_release(&ch->event_id_list_lock);

	if (event_found) {
		*event_id_data = local_event_id_data;
		return 0;
	} else {
		return -1;
	}
}

void gk20a_channel_event_id_post_event(struct channel_gk20a *ch,
				       u32 event_id)
{
	struct gk20a_event_id_data *event_id_data;
	int err = 0;

	err = gk20a_channel_get_event_data_from_id(ch, event_id,
						&event_id_data);
	if (err)
		return;

	nvgpu_mutex_acquire(&event_id_data->lock);

	gk20a_dbg_info(
		"posting event for event_id=%d on ch=%d\n",
		event_id, ch->chid);
	event_id_data->event_posted = true;

	wake_up_interruptible_all(&event_id_data->event_id_wq);

	nvgpu_mutex_release(&event_id_data->lock);
}

static int gk20a_channel_event_id_enable(struct channel_gk20a *ch,
					 int event_id,
					 int *fd)
{
	struct gk20a *g;
	int err = 0;
	int local_fd;
	struct file *file;
	char name[64];
	struct gk20a_event_id_data *event_id_data;

	g = gk20a_get(ch->g);
	if (!g)
		return -ENODEV;

	err = gk20a_channel_get_event_data_from_id(ch,
				event_id, &event_id_data);
	if (err == 0) {
		/* We already have event enabled */
		err = -EINVAL;
		goto free_ref;
	}

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		goto free_ref;
	local_fd = err;

	snprintf(name, sizeof(name), "nvgpu-event%d-fd%d",
		 event_id, local_fd);
	file = anon_inode_getfile(name, &gk20a_event_id_ops,
				  NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	event_id_data = nvgpu_kzalloc(ch->g, sizeof(*event_id_data));
	if (!event_id_data) {
		err = -ENOMEM;
		goto clean_up_file;
	}
	event_id_data->g = g;
	event_id_data->id = ch->chid;
	event_id_data->is_tsg = false;
	event_id_data->event_id = event_id;

	init_waitqueue_head(&event_id_data->event_id_wq);
	err = nvgpu_mutex_init(&event_id_data->lock);
	if (err)
		goto clean_up_free;
	nvgpu_init_list_node(&event_id_data->event_id_node);

	nvgpu_mutex_acquire(&ch->event_id_list_lock);
	nvgpu_list_add_tail(&event_id_data->event_id_node, &ch->event_id_list);
	nvgpu_mutex_release(&ch->event_id_list_lock);

	fd_install(local_fd, file);
	file->private_data = event_id_data;

	*fd = local_fd;

	return 0;

clean_up_free:
	nvgpu_kfree(g, event_id_data);
clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(local_fd);
free_ref:
	gk20a_put(g);
	return err;
}

static int gk20a_channel_event_id_ctrl(struct channel_gk20a *ch,
		struct nvgpu_event_id_ctrl_args *args)
{
	int err = 0;
	int fd = -1;

	if (args->event_id >= NVGPU_IOCTL_CHANNEL_EVENT_ID_MAX)
		return -EINVAL;

	if (gk20a_is_channel_marked_as_tsg(ch))
		return -EINVAL;

	switch (args->cmd) {
	case NVGPU_IOCTL_CHANNEL_EVENT_ID_CMD_ENABLE:
		err = gk20a_channel_event_id_enable(ch, args->event_id, &fd);
		if (!err)
			args->event_fd = fd;
		break;

	default:
		nvgpu_err(ch->g,
			   "unrecognized channel event id cmd: 0x%x",
			   args->cmd);
		err = -EINVAL;
		break;
	}

	return err;
}

static int gk20a_channel_zcull_bind(struct channel_gk20a *ch,
			    struct nvgpu_zcull_bind_args *args)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;

	gk20a_dbg_fn("");

	return g->ops.gr.bind_ctxsw_zcull(g, gr, ch,
				args->gpu_va, args->mode);
}

static int gk20a_ioctl_channel_submit_gpfifo(
	struct channel_gk20a *ch,
	struct nvgpu_submit_gpfifo_args *args)
{
	struct gk20a_fence *fence_out;
	struct fifo_profile_gk20a *profile = NULL;

	int ret = 0;
	gk20a_dbg_fn("");

#ifdef CONFIG_DEBUG_FS
	profile = gk20a_fifo_profile_acquire(ch->g);

	if (profile)
		profile->timestamp[PROFILE_IOCTL_ENTRY] = sched_clock();
#endif
	if (ch->has_timedout)
		return -ETIMEDOUT;
	ret = gk20a_submit_channel_gpfifo(ch, NULL, args, args->num_entries,
					  args->flags, &args->fence,
					  &fence_out, false, profile);

	if (ret)
		goto clean_up;

	/* Convert fence_out to something we can pass back to user space. */
	if (args->flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET) {
		if (args->flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE) {
			int fd = gk20a_fence_install_fd(fence_out);
			if (fd < 0)
				ret = fd;
			else
				args->fence.id = fd;
		} else {
			args->fence.id = fence_out->syncpt_id;
			args->fence.value = fence_out->syncpt_value;
		}
	}
	gk20a_fence_put(fence_out);
#ifdef CONFIG_DEBUG_FS
	if (profile) {
		profile->timestamp[PROFILE_IOCTL_EXIT] = sched_clock();
		gk20a_fifo_profile_release(ch->g, profile);
	}
#endif
clean_up:
	return ret;
}

long gk20a_channel_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct channel_priv *priv = filp->private_data;
	struct channel_gk20a *ch = priv->c;
	struct device *dev = dev_from_gk20a(ch->g);
	u8 buf[NVGPU_IOCTL_CHANNEL_MAX_ARG_SIZE] = {0};
	int err = 0;

	gk20a_dbg_fn("start %d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_IOCTL_CHANNEL_LAST) ||
		(_IOC_SIZE(cmd) > NVGPU_IOCTL_CHANNEL_MAX_ARG_SIZE))
		return -EINVAL;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	/* take a ref or return timeout if channel refs can't be taken */
	ch = gk20a_channel_get(ch);
	if (!ch)
		return -ETIMEDOUT;

	/* protect our sanity for threaded userspace - most of the channel is
	 * not thread safe */
	nvgpu_mutex_acquire(&ch->ioctl_lock);

	/* this ioctl call keeps a ref to the file which keeps a ref to the
	 * channel */

	switch (cmd) {
	case NVGPU_IOCTL_CHANNEL_OPEN:
		err = gk20a_channel_open_ioctl(ch->g,
			(struct nvgpu_channel_open_args *)buf);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_NVMAP_FD:
		break;
	case NVGPU_IOCTL_CHANNEL_ALLOC_OBJ_CTX:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = ch->g->ops.gr.alloc_obj_ctx(ch,
				(struct nvgpu_alloc_obj_ctx_args *)buf);
		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_ALLOC_GPFIFO_EX:
	{
		struct nvgpu_alloc_gpfifo_ex_args *alloc_gpfifo_ex_args =
			(struct nvgpu_alloc_gpfifo_ex_args *)buf;

		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}

		if (!is_power_of_2(alloc_gpfifo_ex_args->num_entries)) {
			err = -EINVAL;
			gk20a_idle(ch->g);
			break;
		}
		err = gk20a_channel_alloc_gpfifo(ch,
				alloc_gpfifo_ex_args->num_entries,
				alloc_gpfifo_ex_args->num_inflight_jobs,
				alloc_gpfifo_ex_args->flags);
		gk20a_idle(ch->g);
		break;
	}
	case NVGPU_IOCTL_CHANNEL_ALLOC_GPFIFO:
	{
		struct nvgpu_alloc_gpfifo_args *alloc_gpfifo_args =
			(struct nvgpu_alloc_gpfifo_args *)buf;

		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}

		/*
		 * Kernel can insert one extra gpfifo entry before user
		 * submitted gpfifos and another one after, for internal usage.
		 * Triple the requested size.
		 */
		err = gk20a_channel_alloc_gpfifo(ch,
				alloc_gpfifo_args->num_entries * 3,
				0,
				alloc_gpfifo_args->flags);
		gk20a_idle(ch->g);
		break;
	}
	case NVGPU_IOCTL_CHANNEL_SUBMIT_GPFIFO:
		err = gk20a_ioctl_channel_submit_gpfifo(ch,
				(struct nvgpu_submit_gpfifo_args *)buf);
		break;
	case NVGPU_IOCTL_CHANNEL_WAIT:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}

		/* waiting is thread-safe, not dropping this mutex could
		 * deadlock in certain conditions */
		nvgpu_mutex_release(&ch->ioctl_lock);

		err = gk20a_channel_wait(ch,
				(struct nvgpu_wait_args *)buf);

		nvgpu_mutex_acquire(&ch->ioctl_lock);

		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_ZCULL_BIND:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_channel_zcull_bind(ch,
				(struct nvgpu_zcull_bind_args *)buf);
		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_ERROR_NOTIFIER:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_init_error_notifier(ch,
				(struct nvgpu_set_error_notifier *)buf);
		gk20a_idle(ch->g);
		break;
#ifdef CONFIG_GK20A_CYCLE_STATS
	case NVGPU_IOCTL_CHANNEL_CYCLE_STATS:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_channel_cycle_stats(ch,
				(struct nvgpu_cycle_stats_args *)buf);
		gk20a_idle(ch->g);
		break;
#endif
	case NVGPU_IOCTL_CHANNEL_SET_TIMEOUT:
	{
		u32 timeout =
			(u32)((struct nvgpu_set_timeout_args *)buf)->timeout;
		gk20a_dbg(gpu_dbg_gpu_dbg, "setting timeout (%d ms) for chid %d",
			   timeout, ch->chid);
		ch->timeout_ms_max = timeout;
		gk20a_channel_trace_sched_param(
			trace_gk20a_channel_set_timeout, ch);
		break;
	}
	case NVGPU_IOCTL_CHANNEL_SET_TIMEOUT_EX:
	{
		u32 timeout =
			(u32)((struct nvgpu_set_timeout_args *)buf)->timeout;
		bool timeout_debug_dump = !((u32)
			((struct nvgpu_set_timeout_ex_args *)buf)->flags &
			(1 << NVGPU_TIMEOUT_FLAG_DISABLE_DUMP));
		gk20a_dbg(gpu_dbg_gpu_dbg, "setting timeout (%d ms) for chid %d",
			   timeout, ch->chid);
		ch->timeout_ms_max = timeout;
		ch->timeout_debug_dump = timeout_debug_dump;
		gk20a_channel_trace_sched_param(
			trace_gk20a_channel_set_timeout, ch);
		break;
	}
	case NVGPU_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvgpu_get_param_args *)buf)->value =
			ch->has_timedout;
		break;
	case NVGPU_IOCTL_CHANNEL_SET_PRIORITY:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = ch->g->ops.fifo.channel_set_priority(ch,
			((struct nvgpu_set_priority_args *)buf)->priority);

		gk20a_idle(ch->g);
		gk20a_channel_trace_sched_param(
			trace_gk20a_channel_set_priority, ch);
		break;
	case NVGPU_IOCTL_CHANNEL_ENABLE:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		if (ch->g->ops.fifo.enable_channel)
			ch->g->ops.fifo.enable_channel(ch);
		else
			err = -ENOSYS;
		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_DISABLE:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		if (ch->g->ops.fifo.disable_channel)
			ch->g->ops.fifo.disable_channel(ch);
		else
			err = -ENOSYS;
		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_PREEMPT:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_fifo_preempt(ch->g, ch);
		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_FORCE_RESET:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = ch->g->ops.fifo.force_reset_ch(ch,
				NVGPU_CHANNEL_RESETCHANNEL_VERIF_ERROR, true);
		gk20a_idle(ch->g);
		break;
	case NVGPU_IOCTL_CHANNEL_EVENT_ID_CTRL:
		err = gk20a_channel_event_id_ctrl(ch,
			      (struct nvgpu_event_id_ctrl_args *)buf);
		break;
#ifdef CONFIG_GK20A_CYCLE_STATS
	case NVGPU_IOCTL_CHANNEL_CYCLE_STATS_SNAPSHOT:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_channel_cycle_stats_snapshot(ch,
				(struct nvgpu_cycle_stats_snapshot_args *)buf);
		gk20a_idle(ch->g);
		break;
#endif
	case NVGPU_IOCTL_CHANNEL_WDT:
		err = gk20a_channel_set_wdt_status(ch,
				(struct nvgpu_channel_wdt_args *)buf);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_RUNLIST_INTERLEAVE:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_channel_set_runlist_interleave(ch,
			((struct nvgpu_runlist_interleave_args *)buf)->level);

		gk20a_idle(ch->g);
		gk20a_channel_trace_sched_param(
			trace_gk20a_channel_set_runlist_interleave, ch);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_TIMESLICE:
		err = gk20a_busy(ch->g);
		if (err) {
			dev_err(dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = ch->g->ops.fifo.channel_set_timeslice(ch,
			((struct nvgpu_timeslice_args *)buf)->timeslice_us);

		gk20a_idle(ch->g);
		gk20a_channel_trace_sched_param(
			trace_gk20a_channel_set_timeslice, ch);
		break;
	case NVGPU_IOCTL_CHANNEL_GET_TIMESLICE:
		((struct nvgpu_timeslice_args *)buf)->timeslice_us =
			gk20a_channel_get_timeslice(ch);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_PREEMPTION_MODE:
		if (ch->g->ops.gr.set_preemption_mode) {
			err = gk20a_busy(ch->g);
			if (err) {
				dev_err(dev,
					"%s: failed to host gk20a for ioctl cmd: 0x%x",
					__func__, cmd);
				break;
			}
			err = ch->g->ops.gr.set_preemption_mode(ch,
			     ((struct nvgpu_preemption_mode_args *)buf)->graphics_preempt_mode,
			     ((struct nvgpu_preemption_mode_args *)buf)->compute_preempt_mode);
			gk20a_idle(ch->g);
		} else {
			err = -EINVAL;
		}
		break;
	case NVGPU_IOCTL_CHANNEL_SET_BOOSTED_CTX:
		if (ch->g->ops.gr.set_boosted_ctx) {
			bool boost =
				((struct nvgpu_boosted_ctx_args *)buf)->boost;

			err = gk20a_busy(ch->g);
			if (err) {
				dev_err(dev,
					"%s: failed to host gk20a for ioctl cmd: 0x%x",
					__func__, cmd);
				break;
			}
			err = ch->g->ops.gr.set_boosted_ctx(ch, boost);
			gk20a_idle(ch->g);
		} else {
			err = -EINVAL;
		}
		break;
	default:
		dev_dbg(dev, "unrecognized ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	nvgpu_mutex_release(&ch->ioctl_lock);

	gk20a_channel_put(ch);

	gk20a_dbg_fn("end");

	return err;
}
