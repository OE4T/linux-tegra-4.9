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
#include <trace/events/nvhost.h>
/*
 * TODO:
 *   stats
 *     - for figuring out what to optimize further
 *   resizable push buffer & sync queue
 *     - some channels hardly need any, some channels (3d) could use more
 */

/* Sync Queue
 *
 * The sync queue is a circular buffer of u32s interpreted as:
 *   0: SyncPointID
 *   1: SyncPointValue
 *   2: NumSlots (how many pushbuffer slots to free)
 *   3: NumHandles
 *   4: nvmap client which pinned the handles
 *   5..: NumHandles * nvmemhandle to unpin
 *
 * There's always one word unused, so (accounting for wrap):
 *   - Write == Read => queue empty
 *   - Write + 1 == Read => queue full
 * The queue must not be left with less than SYNC_QUEUE_MIN_ENTRY words
 * of space at the end of the array.
 *
 * We want to pass contiguous arrays of handles to nvmap_unpin_handles,
 * so arrays that would wrap at the end of the buffer will be split into
 * two (or more) entries.
 */

/* Number of words needed to store an entry containing one handle */
#define SYNC_QUEUE_MIN_ENTRY (4 + (2 * sizeof(void *) / sizeof(u32)))

/**
 * Reset to empty queue.
 */
static void reset_sync_queue(struct sync_queue *queue)
{
	queue->read = 0;
	queue->write = 0;
}

/**
 *  Find the number of handles that can be stashed in the sync queue without
 *  waiting.
 *  0 -> queue is full, must update to wait for some entries to be freed.
 */
static unsigned int sync_queue_space(struct sync_queue *queue)
{
	struct nvhost_cdma *cdma;
	struct nvhost_master *host;

	unsigned int read = queue->read;
	unsigned int write = queue->write;
	u32 size;

	cdma = container_of(queue, struct nvhost_cdma, sync_queue);
	host = cdma_to_dev(cdma);

	BUG_ON(read  > (host->sync_queue_size - SYNC_QUEUE_MIN_ENTRY));
	BUG_ON(write > (host->sync_queue_size - SYNC_QUEUE_MIN_ENTRY));

	/*
	 * We can use all of the space up to the end of the buffer, unless the
	 * read position is within that space (the read position may advance
	 * asynchronously, but that can't take space away once we've seen it).
	 */
	if (read > write) {
		size = (read - 1) - write;
	} else {
		size = host->sync_queue_size - write;

		/*
		 * If the read position is zero, it gets complicated. We can't
		 * use the last word in the buffer, because that would leave
		 * the queue empty.
		 * But also if we use too much we would not leave enough space
		 * for a single handle packet, and would have to wrap in
		 * add_to_sync_queue - also leaving write == read == 0,
		 * an empty queue.
		 */
		if (read == 0)
			size -= SYNC_QUEUE_MIN_ENTRY;
	}

	/*
	 * There must be room for an entry header and at least one handle,
	 * otherwise we report a full queue.
	 */
	if (size < SYNC_QUEUE_MIN_ENTRY)
		return 0;
	/* Minimum entry stores one handle */
	return (size - SYNC_QUEUE_MIN_ENTRY) + 1;
}

/**
 * Add an entry to the sync queue.
 */
#define entry_size(_cnt)	((1 + _cnt)*sizeof(void *)/sizeof(u32))

static void add_to_sync_queue(struct sync_queue *queue,
			      u32 sync_point_id, u32 sync_point_value,
			      u32 nr_slots, struct nvmap_client *user_nvmap,
			      struct nvmap_handle **handles, u32 nr_handles)
{
	struct nvhost_cdma *cdma;
	struct nvhost_master *host;
	u32 write = queue->write;
	u32 *p = queue->buffer + write;
	u32 size = 4 + (entry_size(nr_handles));

	cdma = container_of(queue, struct nvhost_cdma, sync_queue);
	host = cdma_to_dev(cdma);

	BUG_ON(sync_point_id == NVSYNCPT_INVALID);
	BUG_ON(sync_queue_space(queue) < nr_handles);

	write += size;
	BUG_ON(write > host->sync_queue_size);

	*p++ = sync_point_id;
	*p++ = sync_point_value;
	*p++ = nr_slots;
	*p++ = nr_handles;
	BUG_ON(!user_nvmap);
	*(struct nvmap_client **)p = nvmap_client_get(user_nvmap);

	p = (u32 *)((void *)p + sizeof(struct nvmap_client *));

	if (nr_handles)
		memcpy(p, handles, nr_handles * sizeof(struct nvmap_handle *));

	/* If there's not enough room for another entry, wrap to the start. */
	if ((write + SYNC_QUEUE_MIN_ENTRY) > host->sync_queue_size) {
		/*
		 * It's an error for the read position to be zero, as that
		 * would mean we emptied the queue while adding something.
		 */
		BUG_ON(queue->read == 0);
		write = 0;
	}

	queue->write = write;
}

/**
 * Get a pointer to the next entry in the queue, or NULL if the queue is empty.
 * Doesn't consume the entry.
 */
static u32 *sync_queue_head(struct sync_queue *queue)
{
	struct nvhost_cdma *cdma = container_of(queue,
						struct nvhost_cdma,
						sync_queue);
	struct nvhost_master *host = cdma_to_dev(cdma);
	u32 read = queue->read;
	u32 write = queue->write;

	BUG_ON(read  > (host->sync_queue_size - SYNC_QUEUE_MIN_ENTRY));
	BUG_ON(write > (host->sync_queue_size - SYNC_QUEUE_MIN_ENTRY));

	if (read == write)
		return NULL;
	return queue->buffer + read;
}

/**
 * Advances to the next queue entry, if you want to consume it.
 */
static void
dequeue_sync_queue_head(struct sync_queue *queue)
{
	struct nvhost_cdma *cdma = container_of(queue,
						struct nvhost_cdma,
						sync_queue);
	struct nvhost_master *host = cdma_to_dev(cdma);
	u32 read = queue->read;
	u32 size;

	BUG_ON(read == queue->write);

	size = 4 + entry_size(queue->buffer[read + 3]);

	read += size;
	BUG_ON(read > host->sync_queue_size);

	/* If there's not enough room for another entry, wrap to the start. */
	if ((read + SYNC_QUEUE_MIN_ENTRY) > host->sync_queue_size)
		read = 0;

	queue->read = read;
}



/**
 * Return the status of the cdma's sync queue or push buffer for the given event
 *  - sq empty: returns 1 for empty, 0 for not empty (as in "1 empty queue" :-)
 *  - sq space: returns the number of handles that can be stored in the queue
 *  - pb space: returns the number of free slots in the channel's push buffer
 * Must be called with the cdma lock held.
 */
static unsigned int cdma_status(struct nvhost_cdma *cdma, enum cdma_event event)
{
	switch (event) {
	case CDMA_EVENT_SYNC_QUEUE_EMPTY:
		return sync_queue_head(&cdma->sync_queue) ? 0 : 1;
	case CDMA_EVENT_SYNC_QUEUE_SPACE:
		return sync_queue_space(&cdma->sync_queue);
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
unsigned int nvhost_cdma_wait(struct nvhost_cdma *cdma, enum cdma_event event)
{
	for (;;) {
		unsigned int space = cdma_status(cdma, event);
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
static void update_cdma(struct nvhost_cdma *cdma)
{
	bool signal = false;
	struct nvhost_master *dev = cdma_to_dev(cdma);

	BUG_ON(!cdma->running);

	/*
	 * Walk the sync queue, reading the sync point registers as necessary,
	 * to consume as many sync queue entries as possible without blocking
	 */
	for (;;) {
		u32 syncpt_id, syncpt_val;
		unsigned int nr_slots, nr_handles;
		struct nvmap_handle **handles;
		struct nvmap_client *nvmap;
		u32 *sync;

		sync = sync_queue_head(&cdma->sync_queue);
		if (!sync) {
			if (cdma->event == CDMA_EVENT_SYNC_QUEUE_EMPTY)
				signal = true;
			break;
		}

		syncpt_id = *sync++;
		syncpt_val = *sync++;

		BUG_ON(syncpt_id == NVSYNCPT_INVALID);

		/* Check whether this syncpt has completed, and bail if not */
		if (!nvhost_syncpt_min_cmp(&dev->syncpt, syncpt_id, syncpt_val))
			break;

		nr_slots = *sync++;
		nr_handles = *sync++;
		nvmap = *(struct nvmap_client **)sync;
		sync = ((void *)sync + sizeof(struct nvmap_client *));
		handles = (struct nvmap_handle **)sync;

		BUG_ON(!nvmap);

		/* Unpin the memory */
		nvmap_unpin_handles(nvmap, handles, nr_handles);

		nvmap_client_put(nvmap);

		/* Pop push buffer slots */
		if (nr_slots) {
			struct push_buffer *pb = &cdma->push_buffer;
			BUG_ON(!cdma_pb_op(cdma).pop_from);
			cdma_pb_op(cdma).pop_from(pb, nr_slots);
			if (cdma->event == CDMA_EVENT_PUSH_BUFFER_SPACE)
				signal = true;
		}

		dequeue_sync_queue_head(&cdma->sync_queue);
		if (cdma->event == CDMA_EVENT_SYNC_QUEUE_SPACE)
			signal = true;
	}

	/* Wake up CdmaWait() if the requested event happened */
	if (signal) {
		cdma->event = CDMA_EVENT_NONE;
		up(&cdma->sem);
	}
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
	cdma->event = CDMA_EVENT_NONE;
	cdma->running = false;

	/* allocate sync queue memory */
	cdma->sync_queue.buffer = kzalloc(cdma_to_dev(cdma)->sync_queue_size
					  * sizeof(u32), GFP_KERNEL);
	if (!cdma->sync_queue.buffer)
		return -ENOMEM;

	err = cdma_pb_op(cdma).init(pb);
	if (err)
		return err;
	reset_sync_queue(&cdma->sync_queue);
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
	kfree(cdma->sync_queue.buffer);
	cdma->sync_queue.buffer = 0;
	cdma_pb_op(cdma).destroy(pb);
}


/**
 * Begin a cdma submit
 */
void nvhost_cdma_begin(struct nvhost_cdma *cdma)
{
	BUG_ON(!cdma_op(cdma).start);
	mutex_lock(&cdma->lock);
	if (!cdma->running)
		cdma_op(cdma).start(cdma);
	cdma->slots_free = 0;
	cdma->slots_used = 0;
}

/**
 * Push two words into a push buffer slot
 * Blocks as necessary if the push buffer is full.
 */
void nvhost_cdma_push(struct nvhost_cdma *cdma, u32 op1, u32 op2)
{
	nvhost_cdma_push_gather(cdma, NULL, op1, op2);
}

/**
 * Push two words into a push buffer slot
 * Blocks as necessary if the push buffer is full.
 */
void nvhost_cdma_push_gather(struct nvhost_cdma *cdma,
		struct nvmap_handle *handle, u32 op1, u32 op2)
{
	u32 slots_free = cdma->slots_free;
	struct push_buffer *pb = &cdma->push_buffer;
	BUG_ON(!cdma_pb_op(cdma).push_to);
	BUG_ON(!cdma_op(cdma).kick);
	if (slots_free == 0) {
		cdma_op(cdma).kick(cdma);
		slots_free = nvhost_cdma_wait(cdma,
				CDMA_EVENT_PUSH_BUFFER_SPACE);
	}
	cdma->slots_free = slots_free - 1;
	cdma->slots_used++;
	cdma_pb_op(cdma).push_to(pb, handle, op1, op2);
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
		struct nvmap_client *user_nvmap,
		u32 sync_point_id, u32 sync_point_value,
		struct nvmap_handle **handles, unsigned int nr_handles)
{
	BUG_ON(!cdma_op(cdma).kick);
	cdma_op(cdma).kick(cdma);

	while (nr_handles || cdma->slots_used) {
		unsigned int count;
		/*
		 * Wait until there's enough room in the
		 * sync queue to write something.
		 */
		count = nvhost_cdma_wait(cdma, CDMA_EVENT_SYNC_QUEUE_SPACE);

		/* Add reloc entries to sync queue (as many as will fit) */
		if (count > nr_handles)
			count = nr_handles;
		add_to_sync_queue(&cdma->sync_queue, sync_point_id,
				  sync_point_value, cdma->slots_used,
				  user_nvmap, handles, count);
		/* NumSlots only goes in the first packet */
		cdma->slots_used = 0;
		handles += count;
		nr_handles -= count;
	}

	mutex_unlock(&cdma->lock);
}

/**
 * Update cdma state according to current sync point values
 */
void nvhost_cdma_update(struct nvhost_cdma *cdma)
{
	mutex_lock(&cdma->lock);
	update_cdma(cdma);
	mutex_unlock(&cdma->lock);
}

/**
 * Manually spin until all CDMA has finished. Used if an async update
 * cannot be scheduled for any reason.
 */
void nvhost_cdma_flush(struct nvhost_cdma *cdma)
{
	mutex_lock(&cdma->lock);
	while (sync_queue_head(&cdma->sync_queue)) {
		update_cdma(cdma);
		mutex_unlock(&cdma->lock);
		schedule();
		mutex_lock(&cdma->lock);
	}
	mutex_unlock(&cdma->lock);
}

