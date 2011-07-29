/*
 * drivers/video/tegra/host/t20/cdma_t20.c
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

#include <linux/slab.h>
#include "../nvhost_cdma.h"
#include "../dev.h"

#include "hardware_t20.h"

/*
 * push_buffer
 *
 * The push buffer is a circular array of words to be fetched by command DMA.
 * Note that it works slightly differently to the sync queue; fence == cur
 * means that the push buffer is full, not empty.
 */


/**
 * Reset to empty push buffer
 */
static void t20_push_buffer_reset(struct push_buffer *pb)
{
	pb->fence = PUSH_BUFFER_SIZE - 8;
	pb->cur = 0;
}

/**
 * Init push buffer resources
 */
static int t20_push_buffer_init(struct push_buffer *pb)
{
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvmap_client *nvmap = cdma_to_nvmap(cdma);
	pb->mem = NULL;
	pb->mapped = NULL;
	pb->phys = 0;
	pb->handles = NULL;

	BUG_ON(!cdma_pb_op(cdma).reset);
	cdma_pb_op(cdma).reset(pb);

	/* allocate and map pushbuffer memory */
	pb->mem = nvmap_alloc(nvmap, PUSH_BUFFER_SIZE + 4, 32,
			      NVMAP_HANDLE_WRITE_COMBINE);
	if (IS_ERR_OR_NULL(pb->mem)) {
		pb->mem = NULL;
		goto fail;
	}
	pb->mapped = nvmap_mmap(pb->mem);
	if (pb->mapped == NULL)
		goto fail;

	/* pin pushbuffer and get physical address */
	pb->phys = nvmap_pin(nvmap, pb->mem);
	if (pb->phys >= 0xfffff000) {
		pb->phys = 0;
		goto fail;
	}

	/* memory for storing nvmap handles for each opcode pair */
	pb->handles = kzalloc(PUSH_BUFFER_SIZE/2, GFP_KERNEL);
	if (!pb->handles)
		goto fail;

	/* put the restart at the end of pushbuffer memory */
	*(pb->mapped + (PUSH_BUFFER_SIZE >> 2)) = nvhost_opcode_restart(pb->phys);

	return 0;

fail:
	cdma_pb_op(cdma).destroy(pb);
	return -ENOMEM;
}

/**
 * Clean up push buffer resources
 */
static void t20_push_buffer_destroy(struct push_buffer *pb)
{
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvmap_client *nvmap = cdma_to_nvmap(cdma);
	if (pb->mapped)
		nvmap_munmap(pb->mem, pb->mapped);

	if (pb->phys != 0)
		nvmap_unpin(nvmap, pb->mem);

	if (pb->mem)
		nvmap_free(nvmap, pb->mem);

	kfree(pb->handles);

	pb->mem = NULL;
	pb->mapped = NULL;
	pb->phys = 0;
	pb->handles = 0;
}

/**
 * Push two words to the push buffer
 * Caller must ensure push buffer is not full
 */
static void t20_push_buffer_push_to(struct push_buffer *pb,
			struct nvmap_handle *handle, u32 op1, u32 op2)
{
	u32 cur = pb->cur;
	u32 *p = (u32*)((u32)pb->mapped + cur);
	BUG_ON(cur == pb->fence);
	*(p++) = op1;
	*(p++) = op2;
	pb->handles[cur/8] = handle;
	pb->cur = (cur + 8) & (PUSH_BUFFER_SIZE - 1);
	/* printk("push_to_push_buffer: op1=%08x; op2=%08x; cur=%x\n", op1, op2, pb->cur); */
}

/**
 * Pop a number of two word slots from the push buffer
 * Caller must ensure push buffer is not empty
 */
static void t20_push_buffer_pop_from(struct push_buffer *pb, unsigned int slots)
{
	pb->fence = (pb->fence + slots * 8) & (PUSH_BUFFER_SIZE - 1);
}

/**
 * Return the number of two word slots free in the push buffer
 */
static u32 t20_push_buffer_space(struct push_buffer *pb)
{
	return ((pb->fence - pb->cur) & (PUSH_BUFFER_SIZE - 1)) / 8;
}

static u32 t20_push_buffer_putptr(struct push_buffer *pb)
{
	return pb->phys + pb->cur;
}


/**
 * Start channel DMA
 */
static void t20_cdma_start(struct nvhost_cdma *cdma)
{
	void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;

	if (cdma->running)
		return;

	BUG_ON(!cdma_pb_op(cdma).putptr);

	cdma->last_put = cdma_pb_op(cdma).putptr(&cdma->push_buffer);

	writel(nvhost_channel_dmactrl(true, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	/* set base, put, end pointer (all of memory) */
	writel(0, chan_regs + HOST1X_CHANNEL_DMASTART);
	writel(cdma->last_put, chan_regs + HOST1X_CHANNEL_DMAPUT);
	writel(0xFFFFFFFF, chan_regs + HOST1X_CHANNEL_DMAEND);

	/* reset GET */
	writel(nvhost_channel_dmactrl(true, true, true),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	/* start the command DMA */
	writel(nvhost_channel_dmactrl(false, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	cdma->running = true;
}

/**
 * Kick channel DMA into action by writing its PUT offset (if it has changed)
 */
static void t20_cdma_kick(struct nvhost_cdma *cdma)
{
	u32 put;
	BUG_ON(!cdma_pb_op(cdma).putptr);

	put = cdma_pb_op(cdma).putptr(&cdma->push_buffer);

	if (put != cdma->last_put) {
		void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;
		wmb();
		writel(put, chan_regs + HOST1X_CHANNEL_DMAPUT);
		cdma->last_put = put;
	}
}

static void t20_cdma_stop(struct nvhost_cdma *cdma)
{
	void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;

	mutex_lock(&cdma->lock);
	if (cdma->running) {
		nvhost_cdma_wait(cdma, CDMA_EVENT_SYNC_QUEUE_EMPTY);
		writel(nvhost_channel_dmactrl(true, false, false),
			chan_regs + HOST1X_CHANNEL_DMACTRL);
		cdma->running = false;
	}
	mutex_unlock(&cdma->lock);
}

/**
 * Retrieve the op pair at a slot offset from a DMA address
 */
void t20_cdma_peek(struct nvhost_cdma *cdma,
			  u32 dmaget, int slot, u32 *out)
{
	u32 offset = dmaget - cdma->push_buffer.phys;
	u32 *p = cdma->push_buffer.mapped;

	offset = ((offset + slot * 8) & (PUSH_BUFFER_SIZE - 1)) >> 2;
	out[0] = p[offset];
	out[1] = p[offset + 1];
}

int nvhost_init_t20_cdma_support(struct nvhost_master *host)
{
	host->op.cdma.start = t20_cdma_start;
	host->op.cdma.stop = t20_cdma_stop;
	host->op.cdma.kick = t20_cdma_kick;

	host->sync_queue_size = NVHOST_SYNC_QUEUE_SIZE;

	host->op.push_buffer.reset = t20_push_buffer_reset;
	host->op.push_buffer.init = t20_push_buffer_init;
	host->op.push_buffer.destroy = t20_push_buffer_destroy;
	host->op.push_buffer.push_to = t20_push_buffer_push_to;
	host->op.push_buffer.pop_from = t20_push_buffer_pop_from;
	host->op.push_buffer.space = t20_push_buffer_space;
	host->op.push_buffer.putptr = t20_push_buffer_putptr;

	return 0;
}
