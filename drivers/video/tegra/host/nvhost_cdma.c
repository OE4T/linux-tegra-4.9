/*
 * drivers/video/tegra/host/nvhost_cdma.c
 *
 * Tegra Graphics Host Command DMA
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvhost_cdma.h"
#include "dev.h"
#include <asm/cacheflush.h>

#include <linux/slab.h>
#include <linux/kfifo.h>
#include <trace/events/nvhost.h>
#include <linux/interrupt.h>

/*
 * TODO:
 *   stats
 *     - for figuring out what to optimize further
 *   resizable push buffer & sync queue
 *     - some channels hardly need any, some channels (3d) could use more
 */

/**
 * kfifo_save - save current out pointer
 * @fifo: address of the fifo to be used
 */
#define	kfifo_save(fifo) \
__kfifo_uint_must_check_helper( \
({ \
	typeof((fifo) + 1) __tmp = (fifo); \
	struct __kfifo *__kfifo = &__tmp->kfifo; \
	__kfifo->out; \
}) \
)

/**
 * kfifo_restore - restore previously saved pointer
 * @fifo: address of the fifo to be used
 * @out: output pointer
 */
#define	kfifo_restore(fifo, restore) \
(void)({ \
	typeof((fifo) + 1) __tmp = (fifo); \
	struct __kfifo *__kfifo = &__tmp->kfifo; \
	__kfifo->out = (restore); \
})

/**
 * Add an entry to the sync queue.
 */
static void add_to_sync_queue(struct nvhost_cdma *cdma,
			      struct nvhost_job *job,
			      u32 nr_slots,
			      u32 first_get)
{
	BUG_ON(job->syncpt_id == NVSYNCPT_INVALID);

	job->first_get = first_get;
	job->num_slots = nr_slots;
	nvhost_job_get(job);
	kfifo_in(&cdma->sync_queue, (void *)&job, 1);
}

/**
 * Return the status of the cdma's sync queue or push buffer for the given event
 *  - sq empty: returns 1 for empty, 0 for not empty (as in "1 empty queue" :-)
 *  - sq space: returns the number of handles that can be stored in the queue
 *  - pb space: returns the number of free slots in the channel's push buffer
 * Must be called with the cdma lock held.
 */
static unsigned int cdma_status_locked(struct nvhost_cdma *cdma,
		enum cdma_event event)
{
	switch (event) {
	case CDMA_EVENT_SYNC_QUEUE_EMPTY:
		return kfifo_len(&cdma->sync_queue) == 0 ? 1 : 0;
	case CDMA_EVENT_SYNC_QUEUE_SPACE:
		return kfifo_avail(&cdma->sync_queue);
	case CDMA_EVENT_PUSH_BUFFER_SPACE: {
		struct push_buffer *pb = &cdma->push_buffer;
		BUG_ON(!cdma_pb_op(cdma).space);
		return cdma_pb_op(cdma).space(pb);
	}
	default:
		return 0;
	}
}

/**
 * Sleep (if necessary) until the requested event happens
 *   - CDMA_EVENT_SYNC_QUEUE_EMPTY : sync queue is completely empty.
 *     - Returns 1
 *   - CDMA_EVENT_SYNC_QUEUE_SPACE : there is space in the sync queue.
 *   - CDMA_EVENT_PUSH_BUFFER_SPACE : there is space in the push buffer
 *     - Return the amount of space (> 0)
 * Must be called with the cdma lock held.
 */
unsigned int nvhost_cdma_wait_locked(struct nvhost_cdma *cdma,
		enum cdma_event event)
{
	for (;;) {
		unsigned int space = cdma_status_locked(cdma, event);
		if (space)
			return space;

		trace_nvhost_wait_cdma(cdma_to_channel(cdma)->desc->name,
				event);

		BUG_ON(cdma->event != CDMA_EVENT_NONE);
		cdma->event = event;

		mutex_unlock(&cdma->lock);
		down(&cdma->sem);
		mutex_lock(&cdma->lock);
	}
	return 0;
}

/**
 * Start timer for a buffer submition that has completed yet.
 * Must be called with the cdma lock held.
 */
static void cdma_start_timer_locked(struct nvhost_cdma *cdma,
		struct nvhost_job *job)
{
	BUG_ON(!job);
	if (cdma->timeout.clientid) {
		/* timer already started */
		return;
	}

	cdma->timeout.ctx = job->hwctx;
	cdma->timeout.clientid = job->clientid;
	cdma->timeout.syncpt_id = job->syncpt_id;
	cdma->timeout.syncpt_val = job->syncpt_end;
	cdma->timeout.start_ktime = ktime_get();

	schedule_delayed_work(&cdma->timeout.wq,
			msecs_to_jiffies(job->timeout));
}

/**
 * Stop timer when a buffer submition completes.
 * Must be called with the cdma lock held.
 */
static void stop_cdma_timer_locked(struct nvhost_cdma *cdma)
{
	cancel_delayed_work(&cdma->timeout.wq);
	cdma->timeout.ctx = NULL;
	cdma->timeout.clientid = 0;
}

/**
 * For all sync queue entries that have already finished according to the
 * current sync point registers:
 *  - unpin & unref their mems
 *  - pop their push buffer slots
 *  - remove them from the sync queue
 * This is normally called from the host code's worker thread, but can be
 * called manually if necessary.
 * Must be called with the cdma lock held.
 */
static void update_cdma_locked(struct nvhost_cdma *cdma)
{
	bool signal = false;
	struct nvhost_master *dev = cdma_to_dev(cdma);

	BUG_ON(!cdma->running);

	/*
	 * Walk the sync queue, reading the sync point registers as necessary,
	 * to consume as many sync queue entries as possible without blocking
	 */
	for (;;) {
		struct nvhost_syncpt *sp = &dev->syncpt;
		struct nvhost_job *job;
		int result;

		result = kfifo_peek(&cdma->sync_queue, &job);
		if (!result) {
			if (cdma->event == CDMA_EVENT_SYNC_QUEUE_EMPTY)
				signal = true;
			break;
		}

		BUG_ON(job->syncpt_id == NVSYNCPT_INVALID);

		/* Check whether this syncpt has completed, and bail if not */
		if (!nvhost_syncpt_min_cmp(sp,
				job->syncpt_id, job->syncpt_end)) {
			/* Start timer on next pending syncpt */
			if (job->timeout)
				cdma_start_timer_locked(cdma, job);
			break;
		}

		/* Cancel timeout, when a buffer completes */
		if (cdma->timeout.clientid)
			stop_cdma_timer_locked(cdma);

		/* Unpin the memory */
		nvhost_job_unpin(job);

		/* Pop push buffer slots */
		if (job->num_slots) {
			struct push_buffer *pb = &cdma->push_buffer;
			BUG_ON(!cdma_pb_op(cdma).pop_from);
			cdma_pb_op(cdma).pop_from(pb, job->num_slots);
			if (cdma->event == CDMA_EVENT_PUSH_BUFFER_SPACE)
				signal = true;
		}

		nvhost_job_put(job);
		kfifo_skip(&cdma->sync_queue);
		if (cdma->event == CDMA_EVENT_SYNC_QUEUE_SPACE)
			signal = true;
	}

	/* Wake up CdmaWait() if the requested event happened */
	if (signal) {
		cdma->event = CDMA_EVENT_NONE;
		up(&cdma->sem);
	}
}

void nvhost_cdma_update_sync_queue(struct nvhost_cdma *cdma,
		struct nvhost_syncpt *syncpt, struct device *dev)
{
	u32 get_restart;
	u32 syncpt_incrs;
	bool exec_ctxsave;
	unsigned int queue_restore;
	struct nvhost_job *job = NULL;
	int result;
	u32 syncpt_val;

	syncpt_val = nvhost_syncpt_update_min(syncpt, cdma->timeout.syncpt_id);
	queue_restore = kfifo_save(&cdma->sync_queue);

	dev_dbg(dev,
		"%s: starting cleanup (thresh %d, queue length %d)\n",
		__func__,
		syncpt_val, kfifo_len(&cdma->sync_queue));

	/*
	 * Move the sync_queue read pointer to the first entry that hasn't
	 * completed based on the current HW syncpt value. It's likely there
	 * won't be any (i.e. we're still at the head), but covers the case
	 * where a syncpt incr happens just prior/during the teardown.
	 */

	dev_dbg(dev,
		"%s: skip completed buffers still in sync_queue\n",
		__func__);

	result = kfifo_peek(&cdma->sync_queue, &job);
	while (result && syncpt_val >= job->syncpt_end) {
		nvhost_job_dump(dev, job);
		kfifo_skip(&cdma->sync_queue);
		result = kfifo_peek(&cdma->sync_queue, &job);
	}

	/*
	 * Walk the sync_queue, first incrementing with the CPU syncpts that
	 * are partially executed (the first buffer) or fully skipped while
	 * still in the current context (slots are also NOP-ed).
	 *
	 * At the point contexts are interleaved, syncpt increments must be
	 * done inline with the pushbuffer from a GATHER buffer to maintain
	 * the order (slots are modified to be a GATHER of syncpt incrs).
	 *
	 * Note: save in get_restart the location where the timed out buffer
	 * started in the PB, so we can start the refetch from there (with the
	 * modified NOP-ed PB slots). This lets things appear to have completed
	 * properly for this buffer and resources are freed.
	 */

	dev_dbg(dev,
		"%s: perform CPU incr on pending same ctx buffers\n",
		__func__);

	get_restart = cdma->last_put;
	if (kfifo_len(&cdma->sync_queue) > 0)
		get_restart = job->first_get;

	/* do CPU increments as long as this context continues */
	while (result && job->clientid == cdma->timeout.clientid) {
		/* different context, gets us out of this loop */
		if (job->clientid != cdma->timeout.clientid)
			break;

		/* won't need a timeout when replayed */
		job->timeout = 0;

		syncpt_incrs = job->syncpt_end - syncpt_val;
		dev_dbg(dev,
			"%s: CPU incr (%d)\n", __func__, syncpt_incrs);

		nvhost_job_dump(dev, job);

		/* safe to use CPU to incr syncpts */
		cdma_op(cdma).timeout_cpu_incr(cdma,
				job->first_get,
				syncpt_incrs,
				job->syncpt_end,
				job->num_slots);

		kfifo_skip(&cdma->sync_queue);
		result = kfifo_peek(&cdma->sync_queue, &job);
	}

	dev_dbg(dev,
		"%s: GPU incr blocked interleaved ctx buffers\n",
		__func__);

	exec_ctxsave = false;

	/* setup GPU increments */
	while (result) {
		/* same context, increment in the pushbuffer */
		if (job->clientid == cdma->timeout.clientid) {
			/* won't need a timeout when replayed */
			job->timeout = 0;

			/* update buffer's syncpts in the pushbuffer */
			cdma_op(cdma).timeout_pb_incr(cdma,
					job->first_get,
					job->syncpt_incrs,
					job->num_slots,
					exec_ctxsave);

			exec_ctxsave = false;
		} else {
			dev_dbg(dev,
				"%s: switch to a different userctx\n",
				__func__);
			/*
			 * If previous context was the timed out context
			 * then clear its CTXSAVE in this slot.
			 */
			exec_ctxsave = true;
		}

		nvhost_job_dump(dev, job);

		kfifo_skip(&cdma->sync_queue);
		result = kfifo_peek(&cdma->sync_queue, &job);
	}

	dev_dbg(dev,
		"%s: finished sync_queue modification\n", __func__);

	kfifo_restore(&cdma->sync_queue, queue_restore);

	/* roll back DMAGET and start up channel again */
	cdma_op(cdma).timeout_teardown_end(cdma, get_restart);

	if (cdma->timeout.ctx)
		cdma->timeout.ctx->has_timedout = true;
}

/**
 * Create a cdma
 */
int nvhost_cdma_init(struct nvhost_cdma *cdma)
{
	int err;
	struct push_buffer *pb = &cdma->push_buffer;
	BUG_ON(!cdma_pb_op(cdma).init);
	mutex_init(&cdma->lock);
	sema_init(&cdma->sem, 0);

	err = kfifo_alloc(&cdma->sync_queue,
			cdma_to_dev(cdma)->sync_queue_size
				* sizeof(struct nvhost_job *),
			GFP_KERNEL);
	if (err)
		return err;

	cdma->event = CDMA_EVENT_NONE;
	cdma->running = false;
	cdma->torndown = false;

	err = cdma_pb_op(cdma).init(pb);
	if (err)
		return err;
	return 0;
}

/**
 * Destroy a cdma
 */
void nvhost_cdma_deinit(struct nvhost_cdma *cdma)
{
	struct push_buffer *pb = &cdma->push_buffer;

	BUG_ON(!cdma_pb_op(cdma).destroy);
	BUG_ON(cdma->running);
	kfifo_free(&cdma->sync_queue);
	cdma_pb_op(cdma).destroy(pb);
	cdma_op(cdma).timeout_destroy(cdma);
}

/**
 * Begin a cdma submit
 */
int nvhost_cdma_begin(struct nvhost_cdma *cdma, struct nvhost_job *job)
{
	mutex_lock(&cdma->lock);

	if (job->timeout) {
		/* init state on first submit with timeout value */
		if (!cdma->timeout.initialized) {
			int err;
			BUG_ON(!cdma_op(cdma).timeout_init);
			err = cdma_op(cdma).timeout_init(cdma,
				job->syncpt_id);
			if (err) {
				mutex_unlock(&cdma->lock);
				return err;
			}
		}
	}
	if (!cdma->running) {
		BUG_ON(!cdma_op(cdma).start);
		cdma_op(cdma).start(cdma);
	}
	cdma->slots_free = 0;
	cdma->slots_used = 0;
	cdma->first_get = cdma_pb_op(cdma).putptr(&cdma->push_buffer);
	return 0;
}

/**
 * Push two words into a push buffer slot
 * Blocks as necessary if the push buffer is full.
 */
void nvhost_cdma_push(struct nvhost_cdma *cdma, u32 op1, u32 op2)
{
	nvhost_cdma_push_gather(cdma, NULL, NULL, op1, op2);
}

/**
 * Push two words into a push buffer slot
 * Blocks as necessary if the push buffer is full.
 */
void nvhost_cdma_push_gather(struct nvhost_cdma *cdma,
		struct nvmap_client *client,
		struct nvmap_handle *handle, u32 op1, u32 op2)
{
	u32 slots_free = cdma->slots_free;
	struct push_buffer *pb = &cdma->push_buffer;
	BUG_ON(!cdma_pb_op(cdma).push_to);
	BUG_ON(!cdma_op(cdma).kick);
	if (slots_free == 0) {
		cdma_op(cdma).kick(cdma);
		slots_free = nvhost_cdma_wait_locked(cdma,
				CDMA_EVENT_PUSH_BUFFER_SPACE);
	}
	cdma->slots_free = slots_free - 1;
	cdma->slots_used++;
	cdma_pb_op(cdma).push_to(pb, client, handle, op1, op2);
}

/**
 * End a cdma submit
 * Kick off DMA, add a contiguous block of memory handles to the sync queue,
 * and a number of slots to be freed from the pushbuffer.
 * Blocks as necessary if the sync queue is full.
 * The handles for a submit must all be pinned at the same time, but they
 * can be unpinned in smaller chunks.
 */
void nvhost_cdma_end(struct nvhost_cdma *cdma,
		struct nvhost_job *job)
{
	bool was_idle = kfifo_len(&cdma->sync_queue) == 0;

	BUG_ON(!cdma_op(cdma).kick);
	cdma_op(cdma).kick(cdma);

	BUG_ON(job->syncpt_id == NVSYNCPT_INVALID);

	nvhost_cdma_wait_locked(cdma, CDMA_EVENT_SYNC_QUEUE_SPACE);
	add_to_sync_queue(cdma,
			job,
			cdma->slots_used,
			cdma->first_get);

	/* start timer on idle -> active transitions */
	if (job->timeout && was_idle)
		cdma_start_timer_locked(cdma, job);

	mutex_unlock(&cdma->lock);
}

/**
 * Update cdma state according to current sync point values
 */
void nvhost_cdma_update(struct nvhost_cdma *cdma)
{
	mutex_lock(&cdma->lock);
	update_cdma_locked(cdma);
	mutex_unlock(&cdma->lock);
}

/**
 * Wait for push buffer to be empty.
 * @cdma pointer to channel cdma
 * @timeout timeout in ms
 * Returns -ETIME if timeout was reached, zero if push buffer is empty.
 */
int nvhost_cdma_flush(struct nvhost_cdma *cdma, int timeout)
{
	unsigned int space, err = 0;
	unsigned long end_jiffies = jiffies + msecs_to_jiffies(timeout);

	/*
	 * Wait for at most timeout ms. Recalculate timeout at each iteration
	 * to better keep within given timeout.
	 */
	while(!err && time_before(jiffies, end_jiffies)) {
		int timeout_jiffies = end_jiffies - jiffies;

		mutex_lock(&cdma->lock);
		space = cdma_status_locked(cdma,
				CDMA_EVENT_SYNC_QUEUE_EMPTY);
		if (space) {
			mutex_unlock(&cdma->lock);
			return 0;
		}

		/*
		 * Wait for sync queue to become empty. If there is already
		 * an event pending, we need to poll.
		 */
		if (cdma->event != CDMA_EVENT_NONE) {
			mutex_unlock(&cdma->lock);
			schedule();
		} else {
			cdma->event = CDMA_EVENT_SYNC_QUEUE_EMPTY;

			mutex_unlock(&cdma->lock);
			err = down_timeout(&cdma->sem,
					jiffies_to_msecs(timeout_jiffies));
		}
	}
	return err;
}
