/*
 * drivers/video/tegra/host/nvhost_cdma.h
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

#ifndef __NVHOST_CDMA_H
#define __NVHOST_CDMA_H

#include <linux/sched.h>
#include <linux/semaphore.h>

#include <linux/nvhost.h>
#include <mach/nvmap.h>

#include "nvhost_acm.h"

/*
 * cdma
 *
 * This is in charge of a host command DMA channel.
 * Sends ops to a push buffer, and takes responsibility for unpinning
 * (& possibly freeing) of memory after those ops have completed.
 * Producer:
 *	begin
 *		push - send ops to the push buffer
 *	end - start command DMA and enqueue handles to be unpinned
 * Consumer:
 *	update - call to update sync queue and push buffer, unpin memory
 */


struct push_buffer {
	struct nvmap_handle_ref *mem;	/* handle to pushbuffer memory */
	u32 *mapped;			/* mapped pushbuffer memory */
	u32 phys;			/* physical address of pushbuffer */
	u32 fence;			/* index we've written */
	u32 cur;			/* index to write to */
	struct nvmap_handle **handles;	/* nvmap handle for each opcode pair */
};

struct sync_queue {
	unsigned int read;		    /* read position within buffer */
	unsigned int write;		    /* write position within buffer */
	u32 *buffer;                        /* queue data */
};

enum cdma_event {
	CDMA_EVENT_NONE,		/* not waiting for any event */
	CDMA_EVENT_SYNC_QUEUE_EMPTY,	/* wait for empty sync queue */
	CDMA_EVENT_SYNC_QUEUE_SPACE,	/* wait for space in sync queue */
	CDMA_EVENT_PUSH_BUFFER_SPACE	/* wait for space in push buffer */
};

struct nvhost_cdma {
	struct mutex lock;		/* controls access to shared state */
	struct semaphore sem;		/* signalled when event occurs */
	enum cdma_event event;		/* event that sem is waiting for */
	unsigned int slots_used;	/* pb slots used in current submit */
	unsigned int slots_free;	/* pb slots free in current submit */
	unsigned int last_put;		/* last value written to DMAPUT */
	struct push_buffer push_buffer;	/* channel's push buffer */
	struct sync_queue sync_queue;	/* channel's sync queue */
	bool running;

};

#define cdma_to_channel(cdma) container_of(cdma, struct nvhost_channel, cdma)
#define cdma_to_dev(cdma) ((cdma_to_channel(cdma))->dev)
#define cdma_op(cdma) (cdma_to_dev(cdma)->op.cdma)
#define cdma_to_nvmap(cdma) ((cdma_to_dev(cdma))->nvmap)
#define pb_to_cdma(pb) container_of(pb, struct nvhost_cdma, push_buffer)
#define cdma_pb_op(cdma) (cdma_to_dev(cdma)->op.push_buffer)


int	nvhost_cdma_init(struct nvhost_cdma *cdma);
void	nvhost_cdma_deinit(struct nvhost_cdma *cdma);
void	nvhost_cdma_stop(struct nvhost_cdma *cdma);
void	nvhost_cdma_begin(struct nvhost_cdma *cdma);
void	nvhost_cdma_push(struct nvhost_cdma *cdma, u32 op1, u32 op2);
void	nvhost_cdma_push_gather(struct nvhost_cdma *cdma,
		struct nvmap_handle *handle, u32 op1, u32 op2);
void	nvhost_cdma_end(struct nvhost_cdma *cdma,
		struct nvmap_client *user_nvmap,
		u32 sync_point_id, u32 sync_point_value,
		struct nvmap_handle **handles, unsigned int nr_handles);
void	nvhost_cdma_update(struct nvhost_cdma *cdma);
void	nvhost_cdma_flush(struct nvhost_cdma *cdma);
void	nvhost_cdma_peek(struct nvhost_cdma *cdma,
		u32 dmaget, int slot, u32 *out);

unsigned int nvhost_cdma_wait(struct nvhost_cdma *cdma, enum cdma_event event);
#endif
