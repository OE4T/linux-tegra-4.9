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

#if defined(CONFIG_DEBUG_FS) || defined(CONFIG_GK20A_CYCLE_STATS)
#include <linux/dma-buf.h>
#endif

#include <nvgpu/semaphore.h>
#include <nvgpu/timers.h>
#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/list.h>
#include <nvgpu/circ_buf.h>
#include <nvgpu/cond.h>
#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>
#include <nvgpu/ltc.h>

#include "gk20a.h"
#include "ctxsw_trace_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "fence_gk20a.h"

#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

/*
 * This is required for nvgpu_vm_find_buffer() which is used in the tracing
 * code. Once we can get and access userspace buffers without requiring
 * direct dma_buf usage this can be removed.
 */
#include "common/linux/vm_priv.h"

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

static void free_channel(struct fifo_gk20a *f, struct channel_gk20a *c);
static void gk20a_channel_dump_ref_actions(struct channel_gk20a *c);

static void free_priv_cmdbuf(struct channel_gk20a *c,
			     struct priv_cmd_entry *e);

static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c);
static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c);

static void channel_gk20a_free_prealloc_resources(struct channel_gk20a *c);

static void channel_gk20a_joblist_add(struct channel_gk20a *c,
		struct channel_gk20a_job *job);
static void channel_gk20a_joblist_delete(struct channel_gk20a *c,
		struct channel_gk20a_job *job);
static struct channel_gk20a_job *channel_gk20a_joblist_peek(
		struct channel_gk20a *c);

static int channel_gk20a_update_runlist(struct channel_gk20a *c,
					bool add);

static u32 gk20a_get_channel_watchdog_timeout(struct channel_gk20a *ch);

static void gk20a_channel_clean_up_jobs(struct channel_gk20a *c,
					bool clean_all);

/* allocate GPU channel */
static struct channel_gk20a *allocate_channel(struct fifo_gk20a *f)
{
	struct channel_gk20a *ch = NULL;
	struct gk20a *g = f->g;

	nvgpu_mutex_acquire(&f->free_chs_mutex);
	if (!nvgpu_list_empty(&f->free_chs)) {
		ch = nvgpu_list_first_entry(&f->free_chs, channel_gk20a,
							  free_chs);
		nvgpu_list_del(&ch->free_chs);
		WARN_ON(nvgpu_atomic_read(&ch->ref_count));
		WARN_ON(ch->referenceable);
		f->used_channels++;
	}
	nvgpu_mutex_release(&f->free_chs_mutex);

	if (g->aggressive_sync_destroy_thresh &&
			(f->used_channels >
			 g->aggressive_sync_destroy_thresh))
		g->aggressive_sync_destroy = true;

	return ch;
}

static void free_channel(struct fifo_gk20a *f,
		struct channel_gk20a *ch)
{
	struct gk20a *g = f->g;

	trace_gk20a_release_used_channel(ch->chid);
	/* refcount is zero here and channel is in a freed/dead state */
	nvgpu_mutex_acquire(&f->free_chs_mutex);
	/* add to head to increase visibility of timing-related bugs */
	nvgpu_list_add(&ch->free_chs, &f->free_chs);
	f->used_channels--;
	nvgpu_mutex_release(&f->free_chs_mutex);

	/*
	 * On teardown it is not possible to dereference platform, but ignoring
	 * this is fine then because no new channels would be created.
	 */
	if (!nvgpu_is_enabled(g, NVGPU_DRIVER_IS_DYING)) {
		if (g->aggressive_sync_destroy_thresh &&
			(f->used_channels <
			 g->aggressive_sync_destroy_thresh))
			g->aggressive_sync_destroy = false;
	}
}

int channel_gk20a_commit_va(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;

	gk20a_dbg_fn("");

	g->ops.mm.init_inst_block(&c->inst_block, c->vm,
			c->vm->gmmu_page_sizes[gmmu_page_size_big]);

	return 0;
}

u32 gk20a_channel_get_timeslice(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;

	if (!ch->timeslice_us)
		return g->ops.fifo.default_timeslice_us(g);

	return ch->timeslice_us;
}

int gk20a_channel_get_timescale_from_timeslice(struct gk20a *g,
		int timeslice_period,
		int *__timeslice_timeout, int *__timeslice_scale)
{
	int value = scale_ptimer(timeslice_period,
			ptimer_scalingfactor10x(g->ptimer_src_freq));
	int shift = 0;

	/* value field is 8 bits long */
	while (value >= 1 << 8) {
		value >>= 1;
		shift++;
	}

	/* time slice register is only 18bits long */
	if ((value << shift) >= 1<<19) {
		pr_err("Requested timeslice value is clamped to 18 bits\n");
		value = 255;
		shift = 10;
	}

	*__timeslice_timeout = value;
	*__timeslice_scale = shift;

	return 0;
}

static int channel_gk20a_update_runlist(struct channel_gk20a *c, bool add)
{
	return c->g->ops.fifo.update_runlist(c->g, c->runlist_id, c->chid, add, true);
}

int gk20a_enable_channel_tsg(struct gk20a *g, struct channel_gk20a *ch)
{
	struct tsg_gk20a *tsg;

	if (gk20a_is_channel_marked_as_tsg(ch)) {
		tsg = &g->fifo.tsg[ch->tsgid];
		gk20a_enable_tsg(tsg);
	} else {
		g->ops.fifo.enable_channel(ch);
	}

	return 0;
}

int gk20a_disable_channel_tsg(struct gk20a *g, struct channel_gk20a *ch)
{
	struct tsg_gk20a *tsg;

	if (gk20a_is_channel_marked_as_tsg(ch)) {
		tsg = &g->fifo.tsg[ch->tsgid];
		gk20a_disable_tsg(tsg);
	} else {
		g->ops.fifo.disable_channel(ch);
	}

	return 0;
}

void gk20a_channel_abort_clean_up(struct channel_gk20a *ch)
{
	struct channel_gk20a_job *job, *n;
	bool released_job_semaphore = false;
	bool pre_alloc_enabled = channel_gk20a_is_prealloc_enabled(ch);

	/* synchronize with actual job cleanup */
	nvgpu_mutex_acquire(&ch->joblist.cleanup_lock);

	/* ensure no fences are pending */
	nvgpu_mutex_acquire(&ch->sync_lock);
	if (ch->sync)
		ch->sync->set_min_eq_max(ch->sync);
	nvgpu_mutex_release(&ch->sync_lock);

	/* release all job semaphores (applies only to jobs that use
	   semaphore synchronization) */
	channel_gk20a_joblist_lock(ch);
	if (pre_alloc_enabled) {
		int tmp_get = ch->joblist.pre_alloc.get;
		int put = ch->joblist.pre_alloc.put;

		/*
		 * ensure put is read before any subsequent reads.
		 * see corresponding wmb in gk20a_channel_add_job()
		 */
		rmb();

		while (tmp_get != put) {
			job = &ch->joblist.pre_alloc.jobs[tmp_get];
			if (job->post_fence->semaphore) {
				__nvgpu_semaphore_release(
					job->post_fence->semaphore, true);
				released_job_semaphore = true;
			}
			tmp_get = (tmp_get + 1) % ch->joblist.pre_alloc.length;
		}
	} else {
		nvgpu_list_for_each_entry_safe(job, n,
				&ch->joblist.dynamic.jobs,
				channel_gk20a_job, list) {
			if (job->post_fence->semaphore) {
				__nvgpu_semaphore_release(
					job->post_fence->semaphore, true);
				released_job_semaphore = true;
			}
		}
	}
	channel_gk20a_joblist_unlock(ch);

	nvgpu_mutex_release(&ch->joblist.cleanup_lock);

	if (released_job_semaphore)
		nvgpu_cond_broadcast_interruptible(&ch->semaphore_wq);

	/*
	 * When closing the channel, this scheduled update holds one ref which
	 * is waited for before advancing with freeing.
	 */
	gk20a_channel_update(ch);
}

void gk20a_channel_abort(struct channel_gk20a *ch, bool channel_preempt)
{
	gk20a_dbg_fn("");

	if (gk20a_is_channel_marked_as_tsg(ch))
		return gk20a_fifo_abort_tsg(ch->g, ch->tsgid, channel_preempt);

	/* make sure new kickoffs are prevented */
	ch->has_timedout = true;

	ch->g->ops.fifo.disable_channel(ch);

	if (channel_preempt && ch->ch_ctx.gr_ctx)
		ch->g->ops.fifo.preempt_channel(ch->g, ch->chid);

	gk20a_channel_abort_clean_up(ch);
}

int gk20a_wait_channel_idle(struct channel_gk20a *ch)
{
	bool channel_idle = false;
	struct nvgpu_timeout timeout;

	nvgpu_timeout_init(ch->g, &timeout, gk20a_get_gr_idle_timeout(ch->g),
			   NVGPU_TIMER_CPU_TIMER);

	do {
		channel_gk20a_joblist_lock(ch);
		channel_idle = channel_gk20a_joblist_is_empty(ch);
		channel_gk20a_joblist_unlock(ch);
		if (channel_idle)
			break;

		nvgpu_usleep_range(1000, 3000);
	} while (!nvgpu_timeout_expired(&timeout));

	if (!channel_idle) {
		nvgpu_err(ch->g, "jobs not freed for channel %d",
				ch->chid);
		return -EBUSY;
	}

	return 0;
}

void gk20a_disable_channel(struct channel_gk20a *ch)
{
	gk20a_channel_abort(ch, true);
	channel_gk20a_update_runlist(ch, false);
}

int gk20a_channel_set_runlist_interleave(struct channel_gk20a *ch,
						u32 level)
{
	struct gk20a *g = ch->g;
	int ret;

	if (gk20a_is_channel_marked_as_tsg(ch)) {
		nvgpu_err(g, "invalid operation for TSG!");
		return -EINVAL;
	}

	switch (level) {
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW:
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_MEDIUM:
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_HIGH:
		ret = g->ops.fifo.set_runlist_interleave(g, ch->chid,
							false, 0, level);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	gk20a_dbg(gpu_dbg_sched, "chid=%u interleave=%u", ch->chid, level);

	return ret ? ret : g->ops.fifo.update_runlist(g, ch->runlist_id, ~0, true, true);
}

/**
 * gk20a_set_error_notifier_locked()
 * Should be called with ch->error_notifier_mutex held
 */
void gk20a_set_error_notifier_locked(struct channel_gk20a *ch, __u32 error)
{
	if (ch->error_notifier_ref) {
		struct timespec time_data;
		u64 nsec;
		getnstimeofday(&time_data);
		nsec = ((u64)time_data.tv_sec) * 1000000000u +
				(u64)time_data.tv_nsec;
		ch->error_notifier->time_stamp.nanoseconds[0] =
				(u32)nsec;
		ch->error_notifier->time_stamp.nanoseconds[1] =
				(u32)(nsec >> 32);
		ch->error_notifier->info32 = error;
		ch->error_notifier->status = 0xffff;

		nvgpu_err(ch->g,
		    "error notifier set to %d for ch %d", error, ch->chid);
	}
}

void gk20a_set_error_notifier(struct channel_gk20a *ch, __u32 error)
{
	nvgpu_mutex_acquire(&ch->error_notifier_mutex);
	gk20a_set_error_notifier_locked(ch, error);
	nvgpu_mutex_release(&ch->error_notifier_mutex);
}

static void gk20a_wait_until_counter_is_N(
	struct channel_gk20a *ch, nvgpu_atomic_t *counter, int wait_value,
	struct nvgpu_cond *c, const char *caller, const char *counter_name)
{
	while (true) {
		if (NVGPU_COND_WAIT(
			    c,
			    nvgpu_atomic_read(counter) == wait_value,
			    5000) == 0)
			break;

		nvgpu_warn(ch->g,
			   "%s: channel %d, still waiting, %s left: %d, waiting for: %d",
			   caller, ch->chid, counter_name,
			   nvgpu_atomic_read(counter), wait_value);

		gk20a_channel_dump_ref_actions(ch);
	}
}

#if defined(CONFIG_GK20A_CYCLE_STATS)
void gk20a_channel_free_cycle_stats_buffer(struct channel_gk20a *ch)
{
	/* disable existing cyclestats buffer */
	nvgpu_mutex_acquire(&ch->cyclestate.cyclestate_buffer_mutex);
	if (ch->cyclestate.cyclestate_buffer_handler) {
		dma_buf_vunmap(ch->cyclestate.cyclestate_buffer_handler,
				ch->cyclestate.cyclestate_buffer);
		dma_buf_put(ch->cyclestate.cyclestate_buffer_handler);
		ch->cyclestate.cyclestate_buffer_handler = NULL;
		ch->cyclestate.cyclestate_buffer = NULL;
		ch->cyclestate.cyclestate_buffer_size = 0;
	}
	nvgpu_mutex_release(&ch->cyclestate.cyclestate_buffer_mutex);
}

int gk20a_channel_free_cycle_stats_snapshot(struct channel_gk20a *ch)
{
	int ret;

	nvgpu_mutex_acquire(&ch->cs_client_mutex);
	if (ch->cs_client) {
		ret = gr_gk20a_css_detach(ch, ch->cs_client);
		ch->cs_client = NULL;
	} else {
		ret = 0;
	}
	nvgpu_mutex_release(&ch->cs_client_mutex);

	return ret;
}

#endif

/* call ONLY when no references to the channel exist: after the last put */
static void gk20a_free_channel(struct channel_gk20a *ch, bool force)
{
	struct gk20a *g = ch->g;
	struct fifo_gk20a *f = &g->fifo;
	struct gr_gk20a *gr = &g->gr;
	struct vm_gk20a *ch_vm = ch->vm;
	unsigned long timeout = gk20a_get_gr_idle_timeout(g);
	struct dbg_session_gk20a *dbg_s;
	struct dbg_session_data *session_data, *tmp_s;
	struct dbg_session_channel_data *ch_data, *tmp;

	gk20a_dbg_fn("");

	WARN_ON(ch->g == NULL);

	trace_gk20a_free_channel(ch->chid);

	/* abort channel and remove from runlist */
	gk20a_disable_channel(ch);

	/* wait until there's only our ref to the channel */
	if (!force)
		gk20a_wait_until_counter_is_N(
			ch, &ch->ref_count, 1, &ch->ref_count_dec_wq,
			__func__, "references");

	/* wait until all pending interrupts for recently completed
	 * jobs are handled */
	nvgpu_wait_for_deferred_interrupts(g);

	/* prevent new refs */
	nvgpu_spinlock_acquire(&ch->ref_obtain_lock);
	if (!ch->referenceable) {
		nvgpu_spinlock_release(&ch->ref_obtain_lock);
		nvgpu_err(ch->g,
			  "Extra %s() called to channel %u",
			  __func__, ch->chid);
		return;
	}
	ch->referenceable = false;
	nvgpu_spinlock_release(&ch->ref_obtain_lock);

	/* matches with the initial reference in gk20a_open_new_channel() */
	nvgpu_atomic_dec(&ch->ref_count);

	/* wait until no more refs to the channel */
	if (!force)
		gk20a_wait_until_counter_is_N(
			ch, &ch->ref_count, 0, &ch->ref_count_dec_wq,
			__func__, "references");

	/* if engine reset was deferred, perform it now */
	nvgpu_mutex_acquire(&f->deferred_reset_mutex);
	if (g->fifo.deferred_reset_pending) {
		gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg, "engine reset was"
			   " deferred, running now");
		/* if lock is already taken, a reset is taking place
		so no need to repeat */
		if (nvgpu_mutex_tryacquire(&g->fifo.gr_reset_mutex)) {
			gk20a_fifo_deferred_reset(g, ch);
			nvgpu_mutex_release(&g->fifo.gr_reset_mutex);
		}
	}
	nvgpu_mutex_release(&f->deferred_reset_mutex);

	if (!gk20a_channel_as_bound(ch))
		goto unbind;

	gk20a_dbg_info("freeing bound channel context, timeout=%ld",
			timeout);

	if (g->ops.fecs_trace.unbind_channel && !ch->vpr)
		g->ops.fecs_trace.unbind_channel(g, ch);

	/* release channel ctx */
	g->ops.gr.free_channel_ctx(ch);

	gk20a_gr_flush_channel_tlb(gr);

	nvgpu_dma_unmap_free(ch_vm, &ch->gpfifo.mem);
	nvgpu_big_free(g, ch->gpfifo.pipe);
	memset(&ch->gpfifo, 0, sizeof(struct gpfifo_desc));

#if defined(CONFIG_GK20A_CYCLE_STATS)
	gk20a_channel_free_cycle_stats_buffer(ch);
	gk20a_channel_free_cycle_stats_snapshot(ch);
#endif

	channel_gk20a_free_priv_cmdbuf(ch);

	/* sync must be destroyed before releasing channel vm */
	nvgpu_mutex_acquire(&ch->sync_lock);
	if (ch->sync) {
		gk20a_channel_sync_destroy(ch->sync);
		ch->sync = NULL;
	}
	nvgpu_mutex_release(&ch->sync_lock);

	/*
	 * free the channel used semaphore index.
	 * we need to do this before releasing the address space,
	 * as the semaphore pool might get freed after that point.
	 */
	if (ch->hw_sema)
		nvgpu_semaphore_free_hw_sema(ch);

	/*
	 * When releasing the channel we unbind the VM - so release the ref.
	 */
	nvgpu_vm_put(ch_vm);

	nvgpu_spinlock_acquire(&ch->update_fn_lock);
	ch->update_fn = NULL;
	ch->update_fn_data = NULL;
	nvgpu_spinlock_release(&ch->update_fn_lock);
	cancel_work_sync(&ch->update_fn_work);

	/* make sure we don't have deferred interrupts pending that
	 * could still touch the channel */
	nvgpu_wait_for_deferred_interrupts(g);

unbind:
	if (gk20a_is_channel_marked_as_tsg(ch))
		g->ops.fifo.tsg_unbind_channel(ch);

	g->ops.fifo.unbind_channel(ch);
	g->ops.fifo.free_inst(g, ch);

	/* put back the channel-wide submit ref from init */
	if (ch->deterministic) {
		down_read(&g->deterministic_busy);
		ch->deterministic = false;
		gk20a_idle(g);
		up_read(&g->deterministic_busy);
	}

	ch->vpr = false;
	ch->vm = NULL;

	WARN_ON(ch->sync);

	/* unlink all debug sessions */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	list_for_each_entry_safe(session_data, tmp_s,
				&ch->dbg_s_list, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
		list_for_each_entry_safe(ch_data, tmp,
					&dbg_s->ch_list, ch_entry) {
			if (ch_data->chid == ch->chid)
				dbg_unbind_single_channel_gk20a(dbg_s, ch_data);
		}
		nvgpu_mutex_release(&dbg_s->ch_list_lock);
	}

	nvgpu_mutex_release(&g->dbg_sessions_lock);

	/* free pre-allocated resources, if applicable */
	if (channel_gk20a_is_prealloc_enabled(ch))
		channel_gk20a_free_prealloc_resources(ch);

#if GK20A_CHANNEL_REFCOUNT_TRACKING
	memset(ch->ref_actions, 0, sizeof(ch->ref_actions));
	ch->ref_actions_put = 0;
#endif

	/* make sure we catch accesses of unopened channels in case
	 * there's non-refcounted channel pointers hanging around */
	ch->g = NULL;
	wmb();

	/* ALWAYS last */
	free_channel(f, ch);
}

static void gk20a_channel_dump_ref_actions(struct channel_gk20a *ch)
{
#if GK20A_CHANNEL_REFCOUNT_TRACKING
	size_t i, get;
	s64 now = nvgpu_current_time_ms();
	s64 prev = 0;
	struct device *dev = dev_from_gk20a(ch->g);

	nvgpu_spinlock_acquire(&ch->ref_actions_lock);

	dev_info(dev, "ch %d: refs %d. Actions, most recent last:\n",
			ch->chid, nvgpu_atomic_read(&ch->ref_count));

	/* start at the oldest possible entry. put is next insertion point */
	get = ch->ref_actions_put;

	/*
	 * If the buffer is not full, this will first loop to the oldest entry,
	 * skipping not-yet-initialized entries. There is no ref_actions_get.
	 */
	for (i = 0; i < GK20A_CHANNEL_REFCOUNT_TRACKING; i++) {
		struct channel_gk20a_ref_action *act = &ch->ref_actions[get];

		if (act->trace.nr_entries) {
			dev_info(dev, "%s ref %zu steps ago (age %d ms, diff %d ms)\n",
				act->type == channel_gk20a_ref_action_get
					? "GET" : "PUT",
				GK20A_CHANNEL_REFCOUNT_TRACKING - 1 - i,
				now - act->timestamp_ms,
				act->timestamp_ms - prev);

			print_stack_trace(&act->trace, 0);
			prev = act->timestamp_ms;
		}

		get = (get + 1) % GK20A_CHANNEL_REFCOUNT_TRACKING;
	}

	nvgpu_spinlock_release(&ch->ref_actions_lock);
#endif
}

static void gk20a_channel_save_ref_source(struct channel_gk20a *ch,
		enum channel_gk20a_ref_action_type type)
{
#if GK20A_CHANNEL_REFCOUNT_TRACKING
	struct channel_gk20a_ref_action *act;

	nvgpu_spinlock_acquire(&ch->ref_actions_lock);

	act = &ch->ref_actions[ch->ref_actions_put];
	act->type = type;
	act->trace.max_entries = GK20A_CHANNEL_REFCOUNT_TRACKING_STACKLEN;
	act->trace.nr_entries = 0;
	act->trace.skip = 3; /* onwards from the caller of this */
	act->trace.entries = act->trace_entries;
	save_stack_trace(&act->trace);
	act->timestamp_ms = nvgpu_current_time_ms();
	ch->ref_actions_put = (ch->ref_actions_put + 1) %
		GK20A_CHANNEL_REFCOUNT_TRACKING;

	nvgpu_spinlock_release(&ch->ref_actions_lock);
#endif
}

/* Try to get a reference to the channel. Return nonzero on success. If fails,
 * the channel is dead or being freed elsewhere and you must not touch it.
 *
 * Always when a channel_gk20a pointer is seen and about to be used, a
 * reference must be held to it - either by you or the caller, which should be
 * documented well or otherwise clearly seen. This usually boils down to the
 * file from ioctls directly, or an explicit get in exception handlers when the
 * channel is found by a chid.
 *
 * Most global functions in this file require a reference to be held by the
 * caller.
 */
struct channel_gk20a *_gk20a_channel_get(struct channel_gk20a *ch,
					 const char *caller) {
	struct channel_gk20a *ret;

	nvgpu_spinlock_acquire(&ch->ref_obtain_lock);

	if (likely(ch->referenceable)) {
		gk20a_channel_save_ref_source(ch, channel_gk20a_ref_action_get);
		nvgpu_atomic_inc(&ch->ref_count);
		ret = ch;
	} else
		ret = NULL;

	nvgpu_spinlock_release(&ch->ref_obtain_lock);

	if (ret)
		trace_gk20a_channel_get(ch->chid, caller);

	return ret;
}

void _gk20a_channel_put(struct channel_gk20a *ch, const char *caller)
{
	gk20a_channel_save_ref_source(ch, channel_gk20a_ref_action_put);
	trace_gk20a_channel_put(ch->chid, caller);
	nvgpu_atomic_dec(&ch->ref_count);
	nvgpu_cond_broadcast(&ch->ref_count_dec_wq);

	/* More puts than gets. Channel is probably going to get
	 * stuck. */
	WARN_ON(nvgpu_atomic_read(&ch->ref_count) < 0);

	/* Also, more puts than gets. ref_count can go to 0 only if
	 * the channel is closing. Channel is probably going to get
	 * stuck. */
	WARN_ON(nvgpu_atomic_read(&ch->ref_count) == 0 && ch->referenceable);
}

void gk20a_channel_close(struct channel_gk20a *ch)
{
	gk20a_free_channel(ch, false);
}

/*
 * Be careful with this - it is meant for terminating channels when we know the
 * driver is otherwise dying. Ref counts and the like are ignored by this
 * version of the cleanup.
 */
void __gk20a_channel_kill(struct channel_gk20a *ch)
{
	gk20a_free_channel(ch, true);
}

static void gk20a_channel_update_runcb_fn(struct work_struct *work)
{
	struct channel_gk20a *ch =
		container_of(work, struct channel_gk20a, update_fn_work);
	void (*update_fn)(struct channel_gk20a *, void *);
	void *update_fn_data;

	nvgpu_spinlock_acquire(&ch->update_fn_lock);
	update_fn = ch->update_fn;
	update_fn_data = ch->update_fn_data;
	nvgpu_spinlock_release(&ch->update_fn_lock);

	if (update_fn)
		update_fn(ch, update_fn_data);
}

struct channel_gk20a *gk20a_open_new_channel_with_cb(struct gk20a *g,
		void (*update_fn)(struct channel_gk20a *, void *),
		void *update_fn_data,
		int runlist_id,
		bool is_privileged_channel)
{
	struct channel_gk20a *ch = gk20a_open_new_channel(g, runlist_id, is_privileged_channel);

	if (ch) {
		nvgpu_spinlock_acquire(&ch->update_fn_lock);
		ch->update_fn = update_fn;
		ch->update_fn_data = update_fn_data;
		nvgpu_spinlock_release(&ch->update_fn_lock);
	}

	return ch;
}

struct channel_gk20a *gk20a_open_new_channel(struct gk20a *g,
		s32 runlist_id,
		bool is_privileged_channel)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch;
	struct gk20a_event_id_data *event_id_data, *event_id_data_temp;

	/* compatibility with existing code */
	if (!gk20a_fifo_is_valid_runlist_id(g, runlist_id)) {
		runlist_id = gk20a_fifo_get_gr_runlist_id(g);
	}

	gk20a_dbg_fn("");

	ch = allocate_channel(f);
	if (ch == NULL) {
		/* TBD: we want to make this virtualizable */
		nvgpu_err(g, "out of hw chids");
		return NULL;
	}

	trace_gk20a_open_new_channel(ch->chid);

	BUG_ON(ch->g);
	ch->g = g;

	/* Runlist for the channel */
	ch->runlist_id = runlist_id;

	/* Channel privilege level */
	ch->is_privileged_channel = is_privileged_channel;

	if (g->ops.fifo.alloc_inst(g, ch)) {
		ch->g = NULL;
		free_channel(f, ch);
		nvgpu_err(g,
			   "failed to open gk20a channel, out of inst mem");
		return NULL;
	}

	/* now the channel is in a limbo out of the free list but not marked as
	 * alive and used (i.e. get-able) yet */

	ch->pid = current->pid;
	ch->tgid = current->tgid;  /* process granularity for FECS traces */

	/* unhook all events created on this channel */
	nvgpu_mutex_acquire(&ch->event_id_list_lock);
	nvgpu_list_for_each_entry_safe(event_id_data, event_id_data_temp,
				&ch->event_id_list,
				gk20a_event_id_data,
				event_id_node) {
		nvgpu_list_del(&event_id_data->event_id_node);
	}
	nvgpu_mutex_release(&ch->event_id_list_lock);

	/* By default, channel is regular (non-TSG) channel */
	ch->tsgid = NVGPU_INVALID_TSG_ID;

	/* reset timeout counter and update timestamp */
	ch->timeout_accumulated_ms = 0;
	ch->timeout_gpfifo_get = 0;
	/* set gr host default timeout */
	ch->timeout_ms_max = gk20a_get_gr_idle_timeout(g);
	ch->timeout_debug_dump = true;
	ch->has_timedout = false;
	ch->wdt_enabled = true;
	ch->obj_class = 0;
	ch->interleave_level = NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW;
	ch->timeslice_us = g->timeslice_low_priority_us;
#ifdef CONFIG_TEGRA_19x_GPU
	memset(&ch->t19x, 0, sizeof(struct channel_t19x));
#endif


	/* The channel is *not* runnable at this point. It still needs to have
	 * an address space bound and allocate a gpfifo and grctx. */

	nvgpu_cond_init(&ch->notifier_wq);
	nvgpu_cond_init(&ch->semaphore_wq);

	ch->update_fn = NULL;
	ch->update_fn_data = NULL;
	nvgpu_spinlock_init(&ch->update_fn_lock);
	INIT_WORK(&ch->update_fn_work, gk20a_channel_update_runcb_fn);

	/* Mark the channel alive, get-able, with 1 initial use
	 * references. The initial reference will be decreased in
	 * gk20a_free_channel() */
	ch->referenceable = true;
	nvgpu_atomic_set(&ch->ref_count, 1);
	wmb();

	return ch;
}

/* allocate private cmd buffer.
   used for inserting commands before/after user submitted buffers. */
static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	u32 size;
	int err = 0;

	/*
	 * Compute the amount of priv_cmdbuf space we need. In general the worst
	 * case is the kernel inserts both a semaphore pre-fence and post-fence.
	 * Any sync-pt fences will take less memory so we can ignore them for
	 * now.
	 *
	 * A semaphore ACQ (fence-wait) is 8 dwords: semaphore_a, semaphore_b,
	 * semaphore_c, and semaphore_d. A semaphore INCR (fence-get) will be 10
	 * dwords: all the same as an ACQ plus a non-stalling intr which is
	 * another 2 dwords.
	 *
	 * Lastly the number of gpfifo entries per channel is fixed so at most
	 * we can use 2/3rds of the gpfifo entries (1 pre-fence entry, one
	 * userspace entry, and one post-fence entry). Thus the computation is:
	 *
	 *   (gpfifo entry number * (2 / 3) * (8 + 10) * 4 bytes.
	 */
	size = roundup_pow_of_two(c->gpfifo.entry_num *
				  2 * 18 * sizeof(u32) / 3);

	err = nvgpu_dma_alloc_map_sys(ch_vm, size, &q->mem);
	if (err) {
		nvgpu_err(g, "%s: memory allocation failed", __func__);
		goto clean_up;
	}

	q->size = q->mem.size / sizeof (u32);

	return 0;

clean_up:
	channel_gk20a_free_priv_cmdbuf(c);
	return err;
}

static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c)
{
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;

	if (q->size == 0)
		return;

	nvgpu_dma_unmap_free(ch_vm, &q->mem);

	memset(q, 0, sizeof(struct priv_cmd_queue));
}

/* allocate a cmd buffer with given size. size is number of u32 entries */
int gk20a_channel_alloc_priv_cmdbuf(struct channel_gk20a *c, u32 orig_size,
			     struct priv_cmd_entry *e)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	u32 free_count;
	u32 size = orig_size;

	gk20a_dbg_fn("size %d", orig_size);

	if (!e) {
		nvgpu_err(c->g,
			"ch %d: priv cmd entry is null",
			c->chid);
		return -EINVAL;
	}

	/* if free space in the end is less than requested, increase the size
	 * to make the real allocated space start from beginning. */
	if (q->put + size > q->size)
		size = orig_size + (q->size - q->put);

	gk20a_dbg_info("ch %d: priv cmd queue get:put %d:%d",
			c->chid, q->get, q->put);

	free_count = (q->size - (q->put - q->get) - 1) % q->size;

	if (size > free_count)
		return -EAGAIN;

	e->size = orig_size;
	e->mem = &q->mem;

	/* if we have increased size to skip free space in the end, set put
	   to beginning of cmd buffer (0) + size */
	if (size != orig_size) {
		e->off = 0;
		e->gva = q->mem.gpu_va;
		q->put = orig_size;
	} else {
		e->off = q->put;
		e->gva = q->mem.gpu_va + q->put * sizeof(u32);
		q->put = (q->put + orig_size) & (q->size - 1);
	}

	/* we already handled q->put + size > q->size so BUG_ON this */
	BUG_ON(q->put > q->size);

	/*
	 * commit the previous writes before making the entry valid.
	 * see the corresponding rmb() in gk20a_free_priv_cmdbuf().
	 */
	wmb();

	e->valid = true;
	gk20a_dbg_fn("done");

	return 0;
}

/* Don't call this to free an explict cmd entry.
 * It doesn't update priv_cmd_queue get/put */
static void free_priv_cmdbuf(struct channel_gk20a *c,
			     struct priv_cmd_entry *e)
{
	if (channel_gk20a_is_prealloc_enabled(c))
		memset(e, 0, sizeof(struct priv_cmd_entry));
	else
		nvgpu_kfree(c->g, e);
}

static int channel_gk20a_alloc_job(struct channel_gk20a *c,
		struct channel_gk20a_job **job_out)
{
	int err = 0;

	if (channel_gk20a_is_prealloc_enabled(c)) {
		int put = c->joblist.pre_alloc.put;
		int get = c->joblist.pre_alloc.get;

		/*
		 * ensure all subsequent reads happen after reading get.
		 * see corresponding wmb in gk20a_channel_clean_up_jobs()
		 */
		rmb();

		if (CIRC_SPACE(put, get, c->joblist.pre_alloc.length))
			*job_out = &c->joblist.pre_alloc.jobs[put];
		else {
			nvgpu_warn(c->g,
					"out of job ringbuffer space");
			err = -EAGAIN;
		}
	} else {
		*job_out = nvgpu_kzalloc(c->g,
					 sizeof(struct channel_gk20a_job));
		if (!*job_out)
			err = -ENOMEM;
	}

	return err;
}

static void channel_gk20a_free_job(struct channel_gk20a *c,
		struct channel_gk20a_job *job)
{
	/*
	 * In case of pre_allocated jobs, we need to clean out
	 * the job but maintain the pointers to the priv_cmd_entry,
	 * since they're inherently tied to the job node.
	 */
	if (channel_gk20a_is_prealloc_enabled(c)) {
		struct priv_cmd_entry *wait_cmd = job->wait_cmd;
		struct priv_cmd_entry *incr_cmd = job->incr_cmd;
		memset(job, 0, sizeof(*job));
		job->wait_cmd = wait_cmd;
		job->incr_cmd = incr_cmd;
	} else
		nvgpu_kfree(c->g, job);
}

void channel_gk20a_joblist_lock(struct channel_gk20a *c)
{
	if (channel_gk20a_is_prealloc_enabled(c))
		nvgpu_mutex_acquire(&c->joblist.pre_alloc.read_lock);
	else
		nvgpu_spinlock_acquire(&c->joblist.dynamic.lock);
}

void channel_gk20a_joblist_unlock(struct channel_gk20a *c)
{
	if (channel_gk20a_is_prealloc_enabled(c))
		nvgpu_mutex_release(&c->joblist.pre_alloc.read_lock);
	else
		nvgpu_spinlock_release(&c->joblist.dynamic.lock);
}

static struct channel_gk20a_job *channel_gk20a_joblist_peek(
		struct channel_gk20a *c)
{
	int get;
	struct channel_gk20a_job *job = NULL;

	if (channel_gk20a_is_prealloc_enabled(c)) {
		if (!channel_gk20a_joblist_is_empty(c)) {
			get = c->joblist.pre_alloc.get;
			job = &c->joblist.pre_alloc.jobs[get];
		}
	} else {
		if (!nvgpu_list_empty(&c->joblist.dynamic.jobs))
			job = nvgpu_list_first_entry(&c->joblist.dynamic.jobs,
				       channel_gk20a_job, list);
	}

	return job;
}

static void channel_gk20a_joblist_add(struct channel_gk20a *c,
		struct channel_gk20a_job *job)
{
	if (channel_gk20a_is_prealloc_enabled(c)) {
		c->joblist.pre_alloc.put = (c->joblist.pre_alloc.put + 1) %
				(c->joblist.pre_alloc.length);
	} else {
		nvgpu_list_add_tail(&job->list, &c->joblist.dynamic.jobs);
	}
}

static void channel_gk20a_joblist_delete(struct channel_gk20a *c,
		struct channel_gk20a_job *job)
{
	if (channel_gk20a_is_prealloc_enabled(c)) {
		c->joblist.pre_alloc.get = (c->joblist.pre_alloc.get + 1) %
				(c->joblist.pre_alloc.length);
	} else {
		nvgpu_list_del(&job->list);
	}
}

bool channel_gk20a_joblist_is_empty(struct channel_gk20a *c)
{
	if (channel_gk20a_is_prealloc_enabled(c)) {
		int get = c->joblist.pre_alloc.get;
		int put = c->joblist.pre_alloc.put;
		return !(CIRC_CNT(put, get, c->joblist.pre_alloc.length));
	}

	return nvgpu_list_empty(&c->joblist.dynamic.jobs);
}

bool channel_gk20a_is_prealloc_enabled(struct channel_gk20a *c)
{
	bool pre_alloc_enabled = c->joblist.pre_alloc.enabled;

	rmb();
	return pre_alloc_enabled;
}

static int channel_gk20a_prealloc_resources(struct channel_gk20a *c,
	       unsigned int num_jobs)
{
	unsigned int i;
	int err;
	size_t size;
	struct priv_cmd_entry *entries = NULL;

	if (channel_gk20a_is_prealloc_enabled(c) || !num_jobs)
		return -EINVAL;

	/*
	 * pre-allocate the job list.
	 * since vmalloc take in an unsigned long, we need
	 * to make sure we don't hit an overflow condition
	 */
	size = sizeof(struct channel_gk20a_job);
	if (num_jobs <= ULONG_MAX / size)
		c->joblist.pre_alloc.jobs = nvgpu_vzalloc(c->g,
							  num_jobs * size);
	if (!c->joblist.pre_alloc.jobs) {
		err = -ENOMEM;
		goto clean_up;
	}

	/*
	 * pre-allocate 2x priv_cmd_entry for each job up front.
	 * since vmalloc take in an unsigned long, we need
	 * to make sure we don't hit an overflow condition
	 */
	size = sizeof(struct priv_cmd_entry);
	if (num_jobs <= ULONG_MAX / (size << 1))
		entries = nvgpu_vzalloc(c->g, (num_jobs << 1) * size);
	if (!entries) {
		err = -ENOMEM;
		goto clean_up_joblist;
	}

	for (i = 0; i < num_jobs; i++) {
		c->joblist.pre_alloc.jobs[i].wait_cmd = &entries[i];
		c->joblist.pre_alloc.jobs[i].incr_cmd =
			&entries[i + num_jobs];
	}

	/* pre-allocate a fence pool */
	err = gk20a_alloc_fence_pool(c, num_jobs);
	if (err)
		goto clean_up_priv_cmd;

	c->joblist.pre_alloc.length = num_jobs;

	/*
	 * commit the previous writes before setting the flag.
	 * see corresponding rmb in channel_gk20a_is_prealloc_enabled()
	 */
	wmb();
	c->joblist.pre_alloc.enabled = true;

	return 0;

clean_up_priv_cmd:
	nvgpu_vfree(c->g, entries);
clean_up_joblist:
	nvgpu_vfree(c->g, c->joblist.pre_alloc.jobs);
clean_up:
	memset(&c->joblist.pre_alloc, 0, sizeof(c->joblist.pre_alloc));
	return err;
}

static void channel_gk20a_free_prealloc_resources(struct channel_gk20a *c)
{
	nvgpu_vfree(c->g, c->joblist.pre_alloc.jobs[0].wait_cmd);
	nvgpu_vfree(c->g, c->joblist.pre_alloc.jobs);
	gk20a_free_fence_pool(c);

	/*
	 * commit the previous writes before disabling the flag.
	 * see corresponding rmb in channel_gk20a_is_prealloc_enabled()
	 */
	wmb();
	c->joblist.pre_alloc.enabled = false;
}

int gk20a_channel_alloc_gpfifo(struct channel_gk20a *c,
		unsigned int num_entries,
		unsigned int num_inflight_jobs,
		u32 flags)
{
	struct gk20a *g = c->g;
	struct vm_gk20a *ch_vm;
	u32 gpfifo_size;
	int err = 0;
	unsigned long acquire_timeout;

	gpfifo_size = num_entries;

	if (flags & NVGPU_ALLOC_GPFIFO_EX_FLAGS_VPR_ENABLED)
		c->vpr = true;

	if (flags & NVGPU_ALLOC_GPFIFO_EX_FLAGS_DETERMINISTIC) {
		down_read(&g->deterministic_busy);
		/*
		 * Railgating isn't deterministic; instead of disallowing
		 * railgating globally, take a power refcount for this
		 * channel's lifetime. The gk20a_idle() pair for this happens
		 * when the channel gets freed.
		 *
		 * Deterministic flag and this busy must be atomic within the
		 * busy lock.
		 */
		err = gk20a_busy(g);
		if (err) {
			up_read(&g->deterministic_busy);
			return err;
		}

		c->deterministic = true;
		up_read(&g->deterministic_busy);
	}

	/* an address space needs to have been bound at this point. */
	if (!gk20a_channel_as_bound(c)) {
		nvgpu_err(g,
			    "not bound to an address space at time of gpfifo"
			    " allocation.");
		err = -EINVAL;
		goto clean_up_idle;
	}
	ch_vm = c->vm;

	if (c->gpfifo.mem.size) {
		nvgpu_err(g, "channel %d :"
			   "gpfifo already allocated", c->chid);
		err = -EEXIST;
		goto clean_up_idle;
	}

	err = nvgpu_dma_alloc_map_sys(ch_vm,
			gpfifo_size * sizeof(struct nvgpu_gpfifo),
			&c->gpfifo.mem);
	if (err) {
		nvgpu_err(g, "%s: memory allocation failed", __func__);
		goto clean_up;
	}

	if (c->gpfifo.mem.aperture == APERTURE_VIDMEM || g->mm.force_pramin) {
		c->gpfifo.pipe = nvgpu_big_malloc(g,
				gpfifo_size * sizeof(struct nvgpu_gpfifo));
		if (!c->gpfifo.pipe) {
			err = -ENOMEM;
			goto clean_up_unmap;
		}
	}

	c->gpfifo.entry_num = gpfifo_size;
	c->gpfifo.get = c->gpfifo.put = 0;

	gk20a_dbg_info("channel %d : gpfifo_base 0x%016llx, size %d",
		c->chid, c->gpfifo.mem.gpu_va, c->gpfifo.entry_num);

	g->ops.fifo.setup_userd(c);

	if (!g->aggressive_sync_destroy_thresh) {
		nvgpu_mutex_acquire(&c->sync_lock);
		c->sync = gk20a_channel_sync_create(c);
		if (!c->sync) {
			err = -ENOMEM;
			nvgpu_mutex_release(&c->sync_lock);
			goto clean_up_unmap;
		}
		nvgpu_mutex_release(&c->sync_lock);

		if (g->ops.fifo.resetup_ramfc) {
			err = g->ops.fifo.resetup_ramfc(c);
			if (err)
				goto clean_up_sync;
		}
	}

	if (!c->g->timeouts_enabled || !c->wdt_enabled)
		acquire_timeout = 0;
	else
		acquire_timeout = gk20a_get_channel_watchdog_timeout(c);

	err = g->ops.fifo.setup_ramfc(c, c->gpfifo.mem.gpu_va,
					c->gpfifo.entry_num,
					acquire_timeout, flags);
	if (err)
		goto clean_up_sync;

	/* TBD: setup engine contexts */

	if (num_inflight_jobs) {
		err = channel_gk20a_prealloc_resources(c,
				num_inflight_jobs);
		if (err)
			goto clean_up_sync;
	}

	err = channel_gk20a_alloc_priv_cmdbuf(c);
	if (err)
		goto clean_up_prealloc;

	err = channel_gk20a_update_runlist(c, true);
	if (err)
		goto clean_up_priv_cmd;

	g->ops.fifo.bind_channel(c);

	gk20a_dbg_fn("done");
	return 0;

clean_up_priv_cmd:
	channel_gk20a_free_priv_cmdbuf(c);
clean_up_prealloc:
	if (num_inflight_jobs)
		channel_gk20a_free_prealloc_resources(c);
clean_up_sync:
	if (c->sync) {
		gk20a_channel_sync_destroy(c->sync);
		c->sync = NULL;
	}
clean_up_unmap:
	nvgpu_big_free(g, c->gpfifo.pipe);
	nvgpu_dma_unmap_free(ch_vm, &c->gpfifo.mem);
clean_up:
	memset(&c->gpfifo, 0, sizeof(struct gpfifo_desc));
clean_up_idle:
	if (c->deterministic) {
		down_read(&g->deterministic_busy);
		gk20a_idle(g);
		c->deterministic = false;
		up_read(&g->deterministic_busy);
	}
	nvgpu_err(g, "fail");
	return err;
}

/* Update with this periodically to determine how the gpfifo is draining. */
static inline u32 update_gp_get(struct gk20a *g,
				struct channel_gk20a *c)
{
	u32 new_get = g->ops.fifo.userd_gp_get(g, c);

	if (new_get < c->gpfifo.get)
		c->gpfifo.wrap = !c->gpfifo.wrap;
	c->gpfifo.get = new_get;
	return new_get;
}

static inline u32 gp_free_count(struct channel_gk20a *c)
{
	return (c->gpfifo.entry_num - (c->gpfifo.put - c->gpfifo.get) - 1) %
		c->gpfifo.entry_num;
}

bool gk20a_channel_update_and_check_timeout(struct channel_gk20a *ch,
		u32 timeout_delta_ms, bool *progress)
{
	u32 gpfifo_get = update_gp_get(ch->g, ch);

	/* Count consequent timeout isr */
	if (gpfifo_get == ch->timeout_gpfifo_get) {
		/* we didn't advance since previous channel timeout check */
		ch->timeout_accumulated_ms += timeout_delta_ms;
		*progress = false;
	} else {
		/* first timeout isr encountered */
		ch->timeout_accumulated_ms = timeout_delta_ms;
		*progress = true;
	}

	ch->timeout_gpfifo_get = gpfifo_get;

	return ch->g->timeouts_enabled &&
		ch->timeout_accumulated_ms > ch->timeout_ms_max;
}

static u32 gk20a_get_channel_watchdog_timeout(struct channel_gk20a *ch)
{
	return ch->g->ch_wdt_timeout_ms;
}

static u32 get_gp_free_count(struct channel_gk20a *c)
{
	update_gp_get(c->g, c);
	return gp_free_count(c);
}

#ifdef CONFIG_DEBUG_FS
static void trace_write_pushbuffer(struct channel_gk20a *c,
				   struct nvgpu_gpfifo *g)
{
	void *mem = NULL;
	unsigned int words;
	u64 offset;
	struct dma_buf *dmabuf = NULL;

	if (gk20a_debug_trace_cmdbuf) {
		u64 gpu_va = (u64)g->entry0 |
			(u64)((u64)pbdma_gp_entry1_get_hi_v(g->entry1) << 32);
		int err;

		words = pbdma_gp_entry1_length_v(g->entry1);
		err = nvgpu_vm_find_buf(c->vm, gpu_va, &dmabuf, &offset);
		if (!err)
			mem = dma_buf_vmap(dmabuf);
	}

	if (mem) {
		u32 i;
		/*
		 * Write in batches of 128 as there seems to be a limit
		 * of how much you can output to ftrace at once.
		 */
		for (i = 0; i < words; i += 128U) {
			trace_gk20a_push_cmdbuf(
				c->g->name,
				0,
				min(words - i, 128U),
				offset + i * sizeof(u32),
				mem);
		}
		dma_buf_vunmap(dmabuf, mem);
	}
}
#endif

static void trace_write_pushbuffer_range(struct channel_gk20a *c,
					 struct nvgpu_gpfifo *g,
					 struct nvgpu_gpfifo __user *user_gpfifo,
					 int offset,
					 int count)
{
#ifdef CONFIG_DEBUG_FS
	u32 size;
	int i;
	struct nvgpu_gpfifo *gp;
	bool gpfifo_allocated = false;

	if (!gk20a_debug_trace_cmdbuf)
		return;

	if (!g && !user_gpfifo)
		return;

	if (!g) {
		size = count * sizeof(struct nvgpu_gpfifo);
		if (size) {
			g = nvgpu_big_malloc(c->g, size);
			if (!g)
				return;

			if (copy_from_user(g, user_gpfifo, size)) {
				nvgpu_big_free(c->g, g);
				return;
			}
		}
		gpfifo_allocated = true;
	}

	gp = g + offset;
	for (i = 0; i < count; i++, gp++)
		trace_write_pushbuffer(c, gp);

	if (gpfifo_allocated)
		nvgpu_big_free(c->g, g);
#endif
}

static void __gk20a_channel_timeout_start(struct channel_gk20a *ch)
{
	ch->timeout.gp_get = ch->g->ops.fifo.userd_gp_get(ch->g, ch);
	ch->timeout.pb_get = ch->g->ops.fifo.userd_pb_get(ch->g, ch);
	ch->timeout.running = true;
	nvgpu_timeout_init(ch->g, &ch->timeout.timer,
			gk20a_get_channel_watchdog_timeout(ch),
			NVGPU_TIMER_CPU_TIMER);
}

/**
 * Start a timeout counter (watchdog) on this channel.
 *
 * Trigger a watchdog to recover the channel after the per-platform timeout
 * duration (but strictly no earlier) if the channel hasn't advanced within
 * that time.
 *
 * If the timeout is already running, do nothing. This should be called when
 * new jobs are submitted. The timeout will stop when the last tracked job
 * finishes, making the channel idle.
 *
 * The channel's gpfifo read pointer will be used to determine if the job has
 * actually stuck at that time. After the timeout duration has expired, a
 * worker thread will consider the channel stuck and recover it if stuck.
 */
static void gk20a_channel_timeout_start(struct channel_gk20a *ch)
{
	if (!ch->g->timeouts_enabled || !gk20a_get_channel_watchdog_timeout(ch))
		return;

	if (!ch->wdt_enabled)
		return;

	nvgpu_raw_spinlock_acquire(&ch->timeout.lock);

	if (ch->timeout.running) {
		nvgpu_raw_spinlock_release(&ch->timeout.lock);
		return;
	}
	__gk20a_channel_timeout_start(ch);
	nvgpu_raw_spinlock_release(&ch->timeout.lock);
}

/**
 * Stop a running timeout counter (watchdog) on this channel.
 *
 * Make the watchdog consider the channel not running, so that it won't get
 * recovered even if no progress is detected. Progress is not tracked if the
 * watchdog is turned off.
 *
 * No guarantees are made about concurrent execution of the timeout handler.
 * (This should be called from an update handler running in the same thread
 * with the watchdog.)
 */
static bool gk20a_channel_timeout_stop(struct channel_gk20a *ch)
{
	bool was_running;

	nvgpu_raw_spinlock_acquire(&ch->timeout.lock);
	was_running = ch->timeout.running;
	ch->timeout.running = false;
	nvgpu_raw_spinlock_release(&ch->timeout.lock);
	return was_running;
}

/**
 * Continue a previously stopped timeout
 *
 * Enable the timeout again but don't reinitialize its timer.
 *
 * No guarantees are made about concurrent execution of the timeout handler.
 * (This should be called from an update handler running in the same thread
 * with the watchdog.)
 */
static void gk20a_channel_timeout_continue(struct channel_gk20a *ch)
{
	nvgpu_raw_spinlock_acquire(&ch->timeout.lock);
	ch->timeout.running = true;
	nvgpu_raw_spinlock_release(&ch->timeout.lock);
}

/**
 * Rewind the timeout on each non-dormant channel.
 *
 * Reschedule the timeout of each active channel for which timeouts are running
 * as if something was happened on each channel right now. This should be
 * called when a global hang is detected that could cause a false positive on
 * other innocent channels.
 */
void gk20a_channel_timeout_restart_all_channels(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];

		if (!gk20a_channel_get(ch))
			continue;

		nvgpu_raw_spinlock_acquire(&ch->timeout.lock);
		if (ch->timeout.running)
			__gk20a_channel_timeout_start(ch);
		nvgpu_raw_spinlock_release(&ch->timeout.lock);

		gk20a_channel_put(ch);
	}
}

/**
 * Check if a timed out channel has hung and recover it if it has.
 *
 * Test if this channel has really got stuck at this point (should be called
 * when the watchdog timer has expired) by checking if its gp_get has advanced
 * or not. If no gp_get action happened since when the watchdog was started,
 * force-reset the channel.
 *
 * The gpu is implicitly on at this point, because the watchdog can only run on
 * channels that have submitted jobs pending for cleanup.
 */
static void gk20a_channel_timeout_handler(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	u32 gp_get;
	u32 new_gp_get;
	u64 pb_get;
	u64 new_pb_get;

	gk20a_dbg_fn("");

	/* Get status and clear the timer */
	nvgpu_raw_spinlock_acquire(&ch->timeout.lock);
	gp_get = ch->timeout.gp_get;
	pb_get = ch->timeout.pb_get;
	ch->timeout.running = false;
	nvgpu_raw_spinlock_release(&ch->timeout.lock);

	new_gp_get = g->ops.fifo.userd_gp_get(ch->g, ch);
	new_pb_get = g->ops.fifo.userd_pb_get(ch->g, ch);

	if (new_gp_get != gp_get || new_pb_get != pb_get) {
		/* Channel has advanced, reschedule */
		gk20a_channel_timeout_start(ch);
		return;
	}

	nvgpu_err(g, "Job on channel %d timed out",
		  ch->chid);

	gk20a_debug_dump(g);
	gk20a_gr_debug_dump(g);

	g->ops.fifo.force_reset_ch(ch,
		NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT, true);
}

/**
 * Test if the per-channel timeout is expired and handle the timeout in that case.
 *
 * Each channel has an expiration time based watchdog. The timer is
 * (re)initialized in two situations: when a new job is submitted on an idle
 * channel and when the timeout is checked but progress is detected.
 *
 * Watchdog timeout does not yet necessarily mean a stuck channel so this may
 * or may not cause recovery.
 *
 * The timeout is stopped (disabled) after the last job in a row finishes
 * making the channel idle.
 */
static void gk20a_channel_timeout_check(struct channel_gk20a *ch)
{
	bool timed_out;

	nvgpu_raw_spinlock_acquire(&ch->timeout.lock);
	timed_out = ch->timeout.running &&
		nvgpu_timeout_peek_expired(&ch->timeout.timer);
	nvgpu_raw_spinlock_release(&ch->timeout.lock);

	if (timed_out)
		gk20a_channel_timeout_handler(ch);
}

/**
 * Loop every living channel, check timeouts and handle stuck channels.
 */
static void gk20a_channel_poll_timeouts(struct gk20a *g)
{
	unsigned int chid;


	for (chid = 0; chid < g->fifo.num_channels; chid++) {
		struct channel_gk20a *ch = &g->fifo.channel[chid];

		if (gk20a_channel_get(ch)) {
			gk20a_channel_timeout_check(ch);
			gk20a_channel_put(ch);
		}
	}
}

/*
 * Process one scheduled work item for this channel. Currently, the only thing
 * the worker does is job cleanup handling.
 */
static void gk20a_channel_worker_process_ch(struct channel_gk20a *ch)
{
	gk20a_dbg_fn("");

	gk20a_channel_clean_up_jobs(ch, true);

	/* ref taken when enqueued */
	gk20a_channel_put(ch);
}

/**
 * Tell the worker that one more work needs to be done.
 *
 * Increase the work counter to synchronize the worker with the new work. Wake
 * up the worker. If the worker was already running, it will handle this work
 * before going to sleep.
 */
static int __gk20a_channel_worker_wakeup(struct gk20a *g)
{
	int put;

	gk20a_dbg_fn("");

	/*
	 * Currently, the only work type is associated with a lock, which deals
	 * with any necessary barriers. If a work type with no locking were
	 * added, a a wmb() would be needed here. See ..worker_pending() for a
	 * pair.
	 */

	put = nvgpu_atomic_inc_return(&g->channel_worker.put);
	nvgpu_cond_signal(&g->channel_worker.wq);

	return put;
}

/**
 * Test if there is some work pending.
 *
 * This is a pair for __gk20a_channel_worker_wakeup to be called from the
 * worker. The worker has an internal work counter which is incremented once
 * per finished work item. This is compared with the number of queued jobs,
 * which may be channels on the items list or any other types of work.
 */
static bool __gk20a_channel_worker_pending(struct gk20a *g, int get)
{
	bool pending = nvgpu_atomic_read(&g->channel_worker.put) != get;

	/*
	 * This would be the place for a rmb() pairing a wmb() for a wakeup
	 * if we had any work with no implicit barriers caused by locking.
	 */

	return pending;
}

/**
 * Process the queued works for the worker thread serially.
 *
 * Flush all the work items in the queue one by one. This may block timeout
 * handling for a short while, as these are serialized.
 */
static void gk20a_channel_worker_process(struct gk20a *g, int *get)
{

	while (__gk20a_channel_worker_pending(g, *get)) {
		struct channel_gk20a *ch = NULL;

		/*
		 * If a channel is on the list, it's guaranteed to be handled
		 * eventually just once. However, the opposite is not true. A
		 * channel may be being processed if it's on the list or not.
		 *
		 * With this, processing channel works should be conservative
		 * as follows: it's always safe to look at a channel found in
		 * the list, and if someone enqueues the channel, it will be
		 * handled eventually, even if it's being handled at the same
		 * time. A channel is on the list only once; multiple calls to
		 * enqueue are harmless.
		 */
		nvgpu_spinlock_acquire(&g->channel_worker.items_lock);
		if (!nvgpu_list_empty(&g->channel_worker.items)) {
			ch = nvgpu_list_first_entry(&g->channel_worker.items,
				channel_gk20a,
				worker_item);
			nvgpu_list_del(&ch->worker_item);
		}
		nvgpu_spinlock_release(&g->channel_worker.items_lock);

		if (!ch) {
			/*
			 * Woke up for some other reason, but there are no
			 * other reasons than a channel added in the items list
			 * currently, so warn and ack the message.
			 */
			nvgpu_warn(g, "Spurious worker event!");
			++*get;
			break;
		}

		gk20a_channel_worker_process_ch(ch);
		++*get;
	}
}

/*
 * Look at channel states periodically, until canceled. Abort timed out
 * channels serially. Process all work items found in the queue.
 */
static int gk20a_channel_poll_worker(void *arg)
{
	struct gk20a *g = (struct gk20a *)arg;
	struct gk20a_channel_worker *worker = &g->channel_worker;
	unsigned long watchdog_interval = 100; /* milliseconds */
	struct nvgpu_timeout timeout;
	int get = 0;

	gk20a_dbg_fn("");

	nvgpu_timeout_init(g, &timeout, watchdog_interval,
			NVGPU_TIMER_CPU_TIMER);
	while (!nvgpu_thread_should_stop(&worker->poll_task)) {
		int ret;

		ret = NVGPU_COND_WAIT(
				&worker->wq,
				__gk20a_channel_worker_pending(g, get),
				watchdog_interval) > 0;

		if (ret == 0)
			gk20a_channel_worker_process(g, &get);

		if (nvgpu_timeout_peek_expired(&timeout)) {
			gk20a_channel_poll_timeouts(g);
			nvgpu_timeout_init(g, &timeout, watchdog_interval,
					NVGPU_TIMER_CPU_TIMER);
		}
	}
	return 0;
}

/**
 * Initialize the channel worker's metadata and start the background thread.
 */
int nvgpu_channel_worker_init(struct gk20a *g)
{
	int err;
	char thread_name[64];

	nvgpu_atomic_set(&g->channel_worker.put, 0);
	nvgpu_cond_init(&g->channel_worker.wq);
	nvgpu_init_list_node(&g->channel_worker.items);
	nvgpu_spinlock_init(&g->channel_worker.items_lock);
	snprintf(thread_name, sizeof(thread_name),
			"nvgpu_channel_poll_%s", g->name);

	err = nvgpu_thread_create(&g->channel_worker.poll_task, g,
			gk20a_channel_poll_worker, thread_name);
	if (err) {
		nvgpu_err(g, "failed to start channel poller thread");
		return err;
	}

	return 0;
}

void nvgpu_channel_worker_deinit(struct gk20a *g)
{
	nvgpu_thread_stop(&g->channel_worker.poll_task);
}

/**
 * Append a channel to the worker's list, if not there already.
 *
 * The worker thread processes work items (channels in its work list) and polls
 * for other things. This adds @ch to the end of the list and wakes the worker
 * up immediately. If the channel already existed in the list, it's not added,
 * because in that case it has been scheduled already but has not yet been
 * processed.
 */
static void gk20a_channel_worker_enqueue(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;

	gk20a_dbg_fn("");

	/*
	 * Ref released when this item gets processed. The caller should hold
	 * one ref already, so can't fail.
	 */
	if (WARN_ON(!gk20a_channel_get(ch))) {
		nvgpu_warn(g, "cannot get ch ref for worker!");
		return;
	}

	nvgpu_spinlock_acquire(&g->channel_worker.items_lock);
	if (!nvgpu_list_empty(&ch->worker_item)) {
		/*
		 * Already queued, so will get processed eventually.
		 * The worker is probably awake already.
		 */
		nvgpu_spinlock_release(&g->channel_worker.items_lock);
		gk20a_channel_put(ch);
		return;
	}
	nvgpu_list_add_tail(&ch->worker_item, &g->channel_worker.items);
	nvgpu_spinlock_release(&g->channel_worker.items_lock);

	__gk20a_channel_worker_wakeup(g);
}

int gk20a_free_priv_cmdbuf(struct channel_gk20a *c, struct priv_cmd_entry *e)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct gk20a *g = c->g;

	if (!e)
		return 0;

	if (e->valid) {
		/* read the entry's valid flag before reading its contents */
		rmb();
		if ((q->get != e->off) && e->off != 0)
			nvgpu_err(g, "requests out-of-order, ch=%d",
				  c->chid);
		q->get = e->off + e->size;
	}

	free_priv_cmdbuf(c, e);

	return 0;
}

static int gk20a_channel_add_job(struct channel_gk20a *c,
				 struct channel_gk20a_job *job,
				 bool skip_buffer_refcounting)
{
	struct vm_gk20a *vm = c->vm;
	struct nvgpu_mapped_buf **mapped_buffers = NULL;
	int err = 0, num_mapped_buffers = 0;
	bool pre_alloc_enabled = channel_gk20a_is_prealloc_enabled(c);

	if (!skip_buffer_refcounting) {
		err = nvgpu_vm_get_buffers(vm, &mapped_buffers,
					&num_mapped_buffers);
		if (err)
			return err;
	}

	/*
	 * Ref to hold the channel open during the job lifetime. This is
	 * released by job cleanup launched via syncpt or sema interrupt.
	 */
	c = gk20a_channel_get(c);

	if (c) {
		job->num_mapped_buffers = num_mapped_buffers;
		job->mapped_buffers = mapped_buffers;

		gk20a_channel_timeout_start(c);

		if (!pre_alloc_enabled)
			channel_gk20a_joblist_lock(c);

		/*
		 * ensure all pending write complete before adding to the list.
		 * see corresponding rmb in gk20a_channel_clean_up_jobs() &
		 * gk20a_channel_abort_clean_up()
		 */
		wmb();
		channel_gk20a_joblist_add(c, job);

		if (!pre_alloc_enabled)
			channel_gk20a_joblist_unlock(c);
	} else {
		err = -ETIMEDOUT;
		goto err_put_buffers;
	}

	return 0;

err_put_buffers:
	nvgpu_vm_put_buffers(vm, mapped_buffers, num_mapped_buffers);

	return err;
}

/**
 * Clean up job resources for further jobs to use.
 * @clean_all: If true, process as many jobs as possible, otherwise just one.
 *
 * Loop all jobs from the joblist until a pending job is found, or just one if
 * clean_all is not set. Pending jobs are detected from the job's post fence,
 * so this is only done for jobs that have job tracking resources. Free all
 * per-job memory for completed jobs; in case of preallocated resources, this
 * opens up slots for new jobs to be submitted.
 */
static void gk20a_channel_clean_up_jobs(struct channel_gk20a *c,
					bool clean_all)
{
	struct vm_gk20a *vm;
	struct channel_gk20a_job *job;
	struct gk20a *g;
	int job_finished = 0;
	bool watchdog_on = false;

	c = gk20a_channel_get(c);
	if (!c)
		return;

	if (!c->g->power_on) { /* shutdown case */
		gk20a_channel_put(c);
		return;
	}

	vm = c->vm;
	g = c->g;

	/*
	 * If !clean_all, we're in a condition where watchdog isn't supported
	 * anyway (this would be a no-op).
	 */
	if (clean_all)
		watchdog_on = gk20a_channel_timeout_stop(c);

	/* Synchronize with abort cleanup that needs the jobs. */
	nvgpu_mutex_acquire(&c->joblist.cleanup_lock);

	while (1) {
		bool completed;

		channel_gk20a_joblist_lock(c);
		if (channel_gk20a_joblist_is_empty(c)) {
			/*
			 * No jobs in flight, timeout will remain stopped until
			 * new jobs are submitted.
			 */
			channel_gk20a_joblist_unlock(c);
			break;
		}

		/*
		 * ensure that all subsequent reads occur after checking
		 * that we have a valid node. see corresponding wmb in
		 * gk20a_channel_add_job().
		 */
		rmb();
		job = channel_gk20a_joblist_peek(c);
		channel_gk20a_joblist_unlock(c);

		completed = gk20a_fence_is_expired(job->post_fence);
		if (!completed) {
			/*
			 * The watchdog eventually sees an updated gp_get if
			 * something happened in this loop. A new job can have
			 * been submitted between the above call to stop and
			 * this - in that case, this is a no-op and the new
			 * later timeout is still used.
			 */
			if (clean_all && watchdog_on)
				gk20a_channel_timeout_continue(c);
			break;
		}

		WARN_ON(!c->sync);

		if (c->sync) {
			c->sync->signal_timeline(c->sync);

			if (g->aggressive_sync_destroy_thresh) {
				nvgpu_mutex_acquire(&c->sync_lock);
				if (nvgpu_atomic_dec_and_test(
					&c->sync->refcount) &&
						g->aggressive_sync_destroy) {
					gk20a_channel_sync_destroy(c->sync);
					c->sync = NULL;
				}
				nvgpu_mutex_release(&c->sync_lock);
			}
		}

		if (job->num_mapped_buffers)
			nvgpu_vm_put_buffers(vm, job->mapped_buffers,
				job->num_mapped_buffers);

		/* Remove job from channel's job list before we close the
		 * fences, to prevent other callers (gk20a_channel_abort) from
		 * trying to dereference post_fence when it no longer exists.
		 */
		channel_gk20a_joblist_lock(c);
		channel_gk20a_joblist_delete(c, job);
		channel_gk20a_joblist_unlock(c);

		/* Close the fences (this will unref the semaphores and release
		 * them to the pool). */
		gk20a_fence_put(job->pre_fence);
		gk20a_fence_put(job->post_fence);

		/* Free the private command buffers (wait_cmd first and
		 * then incr_cmd i.e. order of allocation) */
		gk20a_free_priv_cmdbuf(c, job->wait_cmd);
		gk20a_free_priv_cmdbuf(c, job->incr_cmd);

		/* another bookkeeping taken in add_job. caller must hold a ref
		 * so this wouldn't get freed here. */
		gk20a_channel_put(c);

		/*
		 * ensure all pending writes complete before freeing up the job.
		 * see corresponding rmb in channel_gk20a_alloc_job().
		 */
		wmb();

		channel_gk20a_free_job(c, job);
		job_finished = 1;

		/*
		 * Deterministic channels have a channel-wide power reference;
		 * for others, there's one per submit.
		 */
		if (!c->deterministic)
			gk20a_idle(g);

		if (!clean_all) {
			/* Timeout isn't supported here so don't touch it. */
			break;
		}
	}

	nvgpu_mutex_release(&c->joblist.cleanup_lock);

	if (job_finished && c->update_fn)
		schedule_work(&c->update_fn_work);

	gk20a_channel_put(c);
}

/**
 * Schedule a job cleanup work on this channel to free resources and to signal
 * about completion.
 *
 * Call this when there has been an interrupt about finished jobs, or when job
 * cleanup needs to be performed, e.g., when closing a channel. This is always
 * safe to call even if there is nothing to clean up. Any visible actions on
 * jobs just before calling this are guaranteed to be processed.
 */
void gk20a_channel_update(struct channel_gk20a *c)
{
	if (!c->g->power_on) { /* shutdown case */
		return;
	}

	trace_gk20a_channel_update(c->chid);
	/* A queued channel is always checked for job cleanup. */
	gk20a_channel_worker_enqueue(c);
}

static void gk20a_submit_append_priv_cmdbuf(struct channel_gk20a *c,
		struct priv_cmd_entry *cmd)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *gpfifo_mem = &c->gpfifo.mem;
	struct nvgpu_gpfifo x = {
		.entry0 = u64_lo32(cmd->gva),
		.entry1 = u64_hi32(cmd->gva) |
			pbdma_gp_entry1_length_f(cmd->size)
	};

	nvgpu_mem_wr_n(g, gpfifo_mem, c->gpfifo.put * sizeof(x),
			&x, sizeof(x));

	if (cmd->mem->aperture == APERTURE_SYSMEM)
		trace_gk20a_push_cmdbuf(g->name, 0, cmd->size, 0,
				cmd->mem->cpu_va + cmd->off * sizeof(u32));

	c->gpfifo.put = (c->gpfifo.put + 1) & (c->gpfifo.entry_num - 1);
}

/*
 * Copy source gpfifo entries into the gpfifo ring buffer, potentially
 * splitting into two memcpys to handle wrap-around.
 */
static int gk20a_submit_append_gpfifo(struct channel_gk20a *c,
		struct nvgpu_gpfifo *kern_gpfifo,
		struct nvgpu_gpfifo __user *user_gpfifo,
		u32 num_entries)
{
	/* byte offsets */
	u32 gpfifo_size = c->gpfifo.entry_num * sizeof(struct nvgpu_gpfifo);
	u32 len = num_entries * sizeof(struct nvgpu_gpfifo);
	u32 start = c->gpfifo.put * sizeof(struct nvgpu_gpfifo);
	u32 end = start + len; /* exclusive */
	struct nvgpu_mem *gpfifo_mem = &c->gpfifo.mem;
	struct nvgpu_gpfifo *cpu_src;
	int err;

	if (user_gpfifo && !c->gpfifo.pipe) {
		/*
		 * This path (from userspace to sysmem) is special in order to
		 * avoid two copies unnecessarily (from user to pipe, then from
		 * pipe to gpu sysmem buffer).
		 *
		 * As a special case, the pipe buffer exists if PRAMIN writes
		 * are forced, although the buffers may not be in vidmem in
		 * that case.
		 */
		if (end > gpfifo_size) {
			/* wrap-around */
			int length0 = gpfifo_size - start;
			int length1 = len - length0;
			void __user *user2 = (u8 __user *)user_gpfifo + length0;

			err = copy_from_user(gpfifo_mem->cpu_va + start,
					user_gpfifo, length0);
			if (err)
				return err;

			err = copy_from_user(gpfifo_mem->cpu_va,
					user2, length1);
			if (err)
				return err;
		} else {
			err = copy_from_user(gpfifo_mem->cpu_va + start,
					user_gpfifo, len);
			if (err)
				return err;
		}

		trace_write_pushbuffer_range(c, NULL, user_gpfifo,
				0, num_entries);
		goto out;
	} else if (user_gpfifo) {
		/* from userspace to vidmem or sysmem when pramin forced, use
		 * the common copy path below */
		err = copy_from_user(c->gpfifo.pipe, user_gpfifo, len);
		if (err)
			return err;

		cpu_src = c->gpfifo.pipe;
	} else {
		/* from kernel to either sysmem or vidmem, don't need
		 * copy_from_user so use the common path below */
		cpu_src = kern_gpfifo;
	}

	if (end > gpfifo_size) {
		/* wrap-around */
		int length0 = gpfifo_size - start;
		int length1 = len - length0;
		void *src2 = (u8 *)cpu_src + length0;

		nvgpu_mem_wr_n(c->g, gpfifo_mem, start, cpu_src, length0);
		nvgpu_mem_wr_n(c->g, gpfifo_mem, 0, src2, length1);
	} else {
		nvgpu_mem_wr_n(c->g, gpfifo_mem, start, cpu_src, len);

	}

	trace_write_pushbuffer_range(c, cpu_src, NULL, 0, num_entries);

out:
	c->gpfifo.put = (c->gpfifo.put + num_entries) &
		(c->gpfifo.entry_num - 1);

	return 0;
}

/*
 * Handle the submit synchronization - pre-fences and post-fences.
 */
static int gk20a_submit_prepare_syncs(struct channel_gk20a *c,
				      struct nvgpu_fence *fence,
				      struct channel_gk20a_job *job,
				      struct priv_cmd_entry **wait_cmd,
				      struct priv_cmd_entry **incr_cmd,
				      struct gk20a_fence **pre_fence,
				      struct gk20a_fence **post_fence,
				      bool force_need_sync_fence,
				      bool register_irq,
				      u32 flags)
{
	struct gk20a *g = c->g;
	bool need_sync_fence = false;
	bool new_sync_created = false;
	int wait_fence_fd = -1;
	int err = 0;
	bool need_wfi = !(flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SUPPRESS_WFI);
	bool pre_alloc_enabled = channel_gk20a_is_prealloc_enabled(c);

	/*
	 * If user wants to always allocate sync_fence_fds then respect that;
	 * otherwise, allocate sync_fence_fd based on user flags.
	 */
	if (force_need_sync_fence)
		need_sync_fence = true;

	if (g->aggressive_sync_destroy_thresh) {
		nvgpu_mutex_acquire(&c->sync_lock);
		if (!c->sync) {
			c->sync = gk20a_channel_sync_create(c);
			if (!c->sync) {
				err = -ENOMEM;
				nvgpu_mutex_release(&c->sync_lock);
				goto fail;
			}
			new_sync_created = true;
		}
		nvgpu_atomic_inc(&c->sync->refcount);
		nvgpu_mutex_release(&c->sync_lock);
	}

	if (g->ops.fifo.resetup_ramfc && new_sync_created) {
		err = g->ops.fifo.resetup_ramfc(c);
		if (err)
			goto fail;
	}

	/*
	 * Optionally insert syncpt wait in the beginning of gpfifo submission
	 * when user requested and the wait hasn't expired. Validate that the id
	 * makes sense, elide if not. The only reason this isn't being
	 * unceremoniously killed is to keep running some tests which trigger
	 * this condition.
	 */
	if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT) {
		job->pre_fence = gk20a_alloc_fence(c);
		if (!job->pre_fence) {
			err = -ENOMEM;
			goto fail;
		}

		if (!pre_alloc_enabled)
			job->wait_cmd = nvgpu_kzalloc(g,
				sizeof(struct priv_cmd_entry));

		if (!job->wait_cmd) {
			err = -ENOMEM;
			goto clean_up_pre_fence;
		}

		if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE) {
			wait_fence_fd = fence->id;
			err = c->sync->wait_fd(c->sync, wait_fence_fd,
					       job->wait_cmd, job->pre_fence);
		} else {
			err = c->sync->wait_syncpt(c->sync, fence->id,
						   fence->value, job->wait_cmd,
						   job->pre_fence);
		}

		if (!err) {
			if (job->wait_cmd->valid)
				*wait_cmd = job->wait_cmd;
			*pre_fence = job->pre_fence;
		} else
			goto clean_up_wait_cmd;
	}

	if ((flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET) &&
	    (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE))
		need_sync_fence = true;

	/*
	 * Always generate an increment at the end of a GPFIFO submission. This
	 * is used to keep track of method completion for idle railgating. The
	 * sync_pt/semaphore PB is added to the GPFIFO later on in submit.
	 */
	job->post_fence = gk20a_alloc_fence(c);
	if (!job->post_fence) {
		err = -ENOMEM;
		goto clean_up_wait_cmd;
	}
	if (!pre_alloc_enabled)
		job->incr_cmd = nvgpu_kzalloc(g, sizeof(struct priv_cmd_entry));

	if (!job->incr_cmd) {
		err = -ENOMEM;
		goto clean_up_post_fence;
	}

	if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET)
		err = c->sync->incr_user(c->sync, wait_fence_fd, job->incr_cmd,
				 job->post_fence, need_wfi, need_sync_fence,
				 register_irq);
	else
		err = c->sync->incr(c->sync, job->incr_cmd,
				    job->post_fence, need_sync_fence,
				    register_irq);
	if (!err) {
		*incr_cmd = job->incr_cmd;
		*post_fence = job->post_fence;
	} else
		goto clean_up_incr_cmd;

	return 0;

clean_up_incr_cmd:
	free_priv_cmdbuf(c, job->incr_cmd);
	if (!pre_alloc_enabled)
		job->incr_cmd = NULL;
clean_up_post_fence:
	gk20a_fence_put(job->post_fence);
	job->post_fence = NULL;
clean_up_wait_cmd:
	free_priv_cmdbuf(c, job->wait_cmd);
	if (!pre_alloc_enabled)
		job->wait_cmd = NULL;
clean_up_pre_fence:
	gk20a_fence_put(job->pre_fence);
	job->pre_fence = NULL;
fail:
	*wait_cmd = NULL;
	*pre_fence = NULL;
	return err;
}

int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvgpu_gpfifo *gpfifo,
				struct nvgpu_submit_gpfifo_args *args,
				u32 num_entries,
				u32 flags,
				struct nvgpu_fence *fence,
				struct gk20a_fence **fence_out,
				bool force_need_sync_fence,
				struct fifo_profile_gk20a *profile)
{
	struct gk20a *g = c->g;
	struct priv_cmd_entry *wait_cmd = NULL;
	struct priv_cmd_entry *incr_cmd = NULL;
	struct gk20a_fence *pre_fence = NULL;
	struct gk20a_fence *post_fence = NULL;
	struct channel_gk20a_job *job = NULL;
	/* we might need two extra gpfifo entries - one for pre fence
	 * and one for post fence. */
	const int extra_entries = 2;
	bool skip_buffer_refcounting = (flags &
			NVGPU_SUBMIT_GPFIFO_FLAGS_SKIP_BUFFER_REFCOUNTING);
	int err = 0;
	bool need_job_tracking;
	bool need_deferred_cleanup = false;
	struct nvgpu_gpfifo __user *user_gpfifo = args ?
		(struct nvgpu_gpfifo __user *)(uintptr_t)args->gpfifo : NULL;

	if (nvgpu_is_enabled(g, NVGPU_DRIVER_IS_DYING))
		return -ENODEV;

	if (c->has_timedout)
		return -ETIMEDOUT;

	/* fifo not large enough for request. Return error immediately.
	 * Kernel can insert gpfifo entries before and after user gpfifos.
	 * So, add extra_entries in user request. Also, HW with fifo size N
	 * can accept only N-1 entreis and so the below condition */
	if (c->gpfifo.entry_num - 1 < num_entries + extra_entries) {
		nvgpu_err(g, "not enough gpfifo space allocated");
		return -ENOMEM;
	}

	if (!gpfifo && !args)
		return -EINVAL;

	if ((flags & (NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT |
		      NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET)) &&
	    !fence)
		return -EINVAL;

	/* an address space needs to have been bound at this point. */
	if (!gk20a_channel_as_bound(c)) {
		nvgpu_err(g,
			    "not bound to an address space at time of gpfifo"
			    " submission.");
		return -EINVAL;
	}

	if (profile)
		profile->timestamp[PROFILE_ENTRY] = sched_clock();

	/* update debug settings */
	nvgpu_ltc_sync_enabled(g);

	gk20a_dbg_info("channel %d", c->chid);

	/*
	 * Job tracking is necessary for any of the following conditions:
	 *  - pre- or post-fence functionality
	 *  - channel wdt
	 *  - GPU rail-gating with non-deterministic channels
	 *  - buffer refcounting
	 *
	 * If none of the conditions are met, then job tracking is not
	 * required and a fast submit can be done (ie. only need to write
	 * out userspace GPFIFO entries and update GP_PUT).
	 */
	need_job_tracking = (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT) ||
			(flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET) ||
			c->wdt_enabled ||
			(g->can_railgate && !c->deterministic) ||
			!skip_buffer_refcounting;

	if (need_job_tracking) {
		bool need_sync_framework = false;

		/*
		 * If the channel is to have deterministic latency and
		 * job tracking is required, the channel must have
		 * pre-allocated resources. Otherwise, we fail the submit here
		 */
		if (c->deterministic && !channel_gk20a_is_prealloc_enabled(c))
			return -EINVAL;

		need_sync_framework = force_need_sync_fence ||
			gk20a_channel_sync_needs_sync_framework(g) ||
			(flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE &&
			(flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT ||
			 flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET));

		/*
		 * Deferred clean-up is necessary for any of the following
		 * conditions:
		 * - channel's deterministic flag is not set
		 * - dependency on sync framework, which could make the
		 *   behavior of the clean-up operation non-deterministic
		 *   (should not be performed in the submit path)
		 * - channel wdt
		 * - GPU rail-gating with non-deterministic channels
		 * - buffer refcounting
		 *
		 * If none of the conditions are met, then deferred clean-up
		 * is not required, and we clean-up one job-tracking
		 * resource in the submit path.
		 */
		need_deferred_cleanup = !c->deterministic ||
					need_sync_framework ||
					c->wdt_enabled ||
					(g->can_railgate &&
					 !c->deterministic) ||
					!skip_buffer_refcounting;

		/*
		 * For deterministic channels, we don't allow deferred clean_up
		 * processing to occur. In cases we hit this, we fail the submit
		 */
		if (c->deterministic && need_deferred_cleanup)
			return -EINVAL;

		if (!c->deterministic) {
			/*
			 * Get a power ref unless this is a deterministic
			 * channel that holds them during the channel lifetime.
			 * This one is released by gk20a_channel_clean_up_jobs,
			 * via syncpt or sema interrupt, whichever is used.
			 */
			err = gk20a_busy(g);
			if (err) {
				nvgpu_err(g,
					"failed to host gk20a to submit gpfifo, process %s",
					current->comm);
				return err;
			}
		}

		if (!need_deferred_cleanup) {
			/* clean up a single job */
			gk20a_channel_clean_up_jobs(c, false);
		}
	}


	/* Grab access to HW to deal with do_idle */
	if (c->deterministic)
		down_read(&g->deterministic_busy);

	trace_gk20a_channel_submit_gpfifo(g->name,
					  c->chid,
					  num_entries,
					  flags,
					  fence ? fence->id : 0,
					  fence ? fence->value : 0);

	gk20a_dbg_info("pre-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	/*
	 * Make sure we have enough space for gpfifo entries. Check cached
	 * values first and then read from HW. If no space, return EAGAIN
	 * and let userpace decide to re-try request or not.
	 */
	if (gp_free_count(c) < num_entries + extra_entries) {
		if (get_gp_free_count(c) < num_entries + extra_entries) {
			err = -EAGAIN;
			goto clean_up;
		}
	}

	if (c->has_timedout) {
		err = -ETIMEDOUT;
		goto clean_up;
	}

	if (need_job_tracking) {
		err = channel_gk20a_alloc_job(c, &job);
		if (err)
			goto clean_up;

		err = gk20a_submit_prepare_syncs(c, fence, job,
						 &wait_cmd, &incr_cmd,
						 &pre_fence, &post_fence,
						 force_need_sync_fence,
						 need_deferred_cleanup,
						 flags);
		if (err)
			goto clean_up_job;
	}

	if (profile)
		profile->timestamp[PROFILE_JOB_TRACKING] = sched_clock();

	if (wait_cmd)
		gk20a_submit_append_priv_cmdbuf(c, wait_cmd);

	if (gpfifo || user_gpfifo)
		err = gk20a_submit_append_gpfifo(c, gpfifo, user_gpfifo,
				num_entries);
	if (err)
		goto clean_up_job;

	/*
	 * And here's where we add the incr_cmd we generated earlier. It should
	 * always run!
	 */
	if (incr_cmd)
		gk20a_submit_append_priv_cmdbuf(c, incr_cmd);

	if (fence_out)
		*fence_out = gk20a_fence_get(post_fence);

	if (need_job_tracking)
		/* TODO! Check for errors... */
		gk20a_channel_add_job(c, job, skip_buffer_refcounting);
	if (profile)
		profile->timestamp[PROFILE_APPEND] = sched_clock();

	g->ops.fifo.userd_gp_put(g, c);

	/* No hw access beyond this point */
	if (c->deterministic)
		up_read(&g->deterministic_busy);

	trace_gk20a_channel_submitted_gpfifo(g->name,
				c->chid,
				num_entries,
				flags,
				post_fence ? post_fence->syncpt_id : 0,
				post_fence ? post_fence->syncpt_value : 0);

	gk20a_dbg_info("post-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	if (profile)
		profile->timestamp[PROFILE_END] = sched_clock();
	gk20a_dbg_fn("done");
	return err;

clean_up_job:
	channel_gk20a_free_job(c, job);
clean_up:
	gk20a_dbg_fn("fail");
	gk20a_fence_put(pre_fence);
	gk20a_fence_put(post_fence);
	if (c->deterministic)
		up_read(&g->deterministic_busy);
	else if (need_deferred_cleanup)
		gk20a_idle(g);

	return err;
}

/*
 * Stop deterministic channel activity for do_idle() when power needs to go off
 * momentarily but deterministic channels keep power refs for potentially a
 * long time.
 *
 * Takes write access on g->deterministic_busy.
 *
 * Must be paired with gk20a_channel_deterministic_unidle().
 */
void gk20a_channel_deterministic_idle(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;

	/* Grab exclusive access to the hw to block new submits */
	down_write(&g->deterministic_busy);

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];

		if (!gk20a_channel_get(ch))
			continue;

		if (ch->deterministic) {
			/*
			 * Drop the power ref taken when setting deterministic
			 * flag. deterministic_unidle will put this and the
			 * channel ref back.
			 *
			 * Hold the channel ref: it must not get freed in
			 * between. A race could otherwise result in lost
			 * gk20a_busy() via unidle, and in unbalanced
			 * gk20a_idle() via closing the channel.
			 */
			gk20a_idle(g);
		} else {
			/* Not interesting, carry on. */
			gk20a_channel_put(ch);
		}
	}
}

/*
 * Allow deterministic channel activity again for do_unidle().
 *
 * This releases write access on g->deterministic_busy.
 */
void gk20a_channel_deterministic_unidle(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];

		if (!gk20a_channel_get(ch))
			continue;

		/*
		 * Deterministic state changes inside deterministic_busy lock,
		 * which we took in deterministic_idle.
		 */
		if (ch->deterministic) {
			if (gk20a_busy(g))
				nvgpu_err(g, "cannot busy() again!");
			/* Took this in idle() */
			gk20a_channel_put(ch);
		}

		gk20a_channel_put(ch);
	}

	/* Release submits, new deterministic channels and frees */
	up_write(&g->deterministic_busy);
}

int gk20a_init_channel_support(struct gk20a *g, u32 chid)
{
	struct channel_gk20a *c = g->fifo.channel+chid;
	int err;

	c->g = NULL;
	c->chid = chid;
	nvgpu_atomic_set(&c->bound, false);
	nvgpu_spinlock_init(&c->ref_obtain_lock);
	nvgpu_atomic_set(&c->ref_count, 0);
	c->referenceable = false;
	nvgpu_cond_init(&c->ref_count_dec_wq);

#if GK20A_CHANNEL_REFCOUNT_TRACKING
	nvgpu_spinlock_init(&c->ref_actions_lock);
#endif
	nvgpu_spinlock_init(&c->joblist.dynamic.lock);
	nvgpu_raw_spinlock_init(&c->timeout.lock);

	nvgpu_init_list_node(&c->joblist.dynamic.jobs);
	nvgpu_init_list_node(&c->dbg_s_list);
	nvgpu_init_list_node(&c->event_id_list);
	nvgpu_init_list_node(&c->worker_item);

	err = nvgpu_mutex_init(&c->ioctl_lock);
	if (err)
		return err;
	err = nvgpu_mutex_init(&c->error_notifier_mutex);
	if (err)
		goto fail_1;
	err = nvgpu_mutex_init(&c->joblist.cleanup_lock);
	if (err)
		goto fail_2;
	err = nvgpu_mutex_init(&c->joblist.pre_alloc.read_lock);
	if (err)
		goto fail_3;
	err = nvgpu_mutex_init(&c->sync_lock);
	if (err)
		goto fail_4;
#if defined(CONFIG_GK20A_CYCLE_STATS)
	err = nvgpu_mutex_init(&c->cyclestate.cyclestate_buffer_mutex);
	if (err)
		goto fail_5;
	err = nvgpu_mutex_init(&c->cs_client_mutex);
	if (err)
		goto fail_6;
#endif
	err = nvgpu_mutex_init(&c->event_id_list_lock);
	if (err)
		goto fail_7;
	err = nvgpu_mutex_init(&c->dbg_s_lock);
	if (err)
		goto fail_8;

	nvgpu_list_add(&c->free_chs, &g->fifo.free_chs);

	return 0;

fail_8:
	nvgpu_mutex_destroy(&c->event_id_list_lock);
fail_7:
#if defined(CONFIG_GK20A_CYCLE_STATS)
	nvgpu_mutex_destroy(&c->cs_client_mutex);
fail_6:
	nvgpu_mutex_destroy(&c->cyclestate.cyclestate_buffer_mutex);
fail_5:
#endif
	nvgpu_mutex_destroy(&c->sync_lock);
fail_4:
	nvgpu_mutex_destroy(&c->joblist.pre_alloc.read_lock);
fail_3:
	nvgpu_mutex_destroy(&c->joblist.cleanup_lock);
fail_2:
	nvgpu_mutex_destroy(&c->error_notifier_mutex);
fail_1:
	nvgpu_mutex_destroy(&c->ioctl_lock);

	return err;
}

/* in this context the "channel" is the host1x channel which
 * maps to *all* gk20a channels */
int gk20a_channel_suspend(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	bool channels_in_use = false;
	u32 active_runlist_ids = 0;

	gk20a_dbg_fn("");

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (gk20a_channel_get(ch)) {
			gk20a_dbg_info("suspend channel %d", chid);
			/* disable channel */
			gk20a_disable_channel_tsg(g, ch);
			/* preempt the channel */
			gk20a_fifo_preempt(g, ch);
			/* wait for channel update notifiers */
			if (ch->update_fn)
				cancel_work_sync(&ch->update_fn_work);

			channels_in_use = true;

			active_runlist_ids |= BIT(ch->runlist_id);

			gk20a_channel_put(ch);
		}
	}

	if (channels_in_use) {
		gk20a_fifo_update_runlist_ids(g, active_runlist_ids, ~0, false, true);

		for (chid = 0; chid < f->num_channels; chid++) {
			if (gk20a_channel_get(&f->channel[chid])) {
				g->ops.fifo.unbind_channel(&f->channel[chid]);
				gk20a_channel_put(&f->channel[chid]);
			}
		}
	}

	gk20a_dbg_fn("done");
	return 0;
}

int gk20a_channel_resume(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	bool channels_in_use = false;
	u32 active_runlist_ids = 0;

	gk20a_dbg_fn("");

	for (chid = 0; chid < f->num_channels; chid++) {
		if (gk20a_channel_get(&f->channel[chid])) {
			gk20a_dbg_info("resume channel %d", chid);
			g->ops.fifo.bind_channel(&f->channel[chid]);
			channels_in_use = true;
			active_runlist_ids |= BIT(f->channel[chid].runlist_id);
			gk20a_channel_put(&f->channel[chid]);
		}
	}

	if (channels_in_use)
		gk20a_fifo_update_runlist_ids(g, active_runlist_ids, ~0, true, true);

	gk20a_dbg_fn("done");
	return 0;
}

void gk20a_channel_semaphore_wakeup(struct gk20a *g, bool post_events)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;

	gk20a_dbg_fn("");

	/*
	 * Ensure that all pending writes are actually done  before trying to
	 * read semaphore values from DRAM.
	 */
	g->ops.mm.fb_flush(g);

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *c = g->fifo.channel+chid;
		if (gk20a_channel_get(c)) {
			if (nvgpu_atomic_read(&c->bound)) {
				nvgpu_cond_broadcast_interruptible(
						&c->semaphore_wq);
				if (post_events) {
					if (gk20a_is_channel_marked_as_tsg(c)) {
						struct tsg_gk20a *tsg =
							&g->fifo.tsg[c->tsgid];

						gk20a_tsg_event_id_post_event(tsg,
						    NVGPU_IOCTL_CHANNEL_EVENT_ID_BLOCKING_SYNC);
					} else {
						gk20a_channel_event_id_post_event(c,
						    NVGPU_IOCTL_CHANNEL_EVENT_ID_BLOCKING_SYNC);
					}
				}
				/*
				 * Only non-deterministic channels get the
				 * channel_update callback. We don't allow
				 * semaphore-backed syncs for these channels
				 * anyways, since they have a dependency on
				 * the sync framework.
				 * If deterministic channels are receiving a
				 * semaphore wakeup, it must be for a
				 * user-space managed
				 * semaphore.
				 */
				if (!c->deterministic)
					gk20a_channel_update(c);
			}
			gk20a_channel_put(c);
		}
	}
}
