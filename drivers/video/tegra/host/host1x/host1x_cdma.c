/*
 * drivers/video/tegra/host/host1x/host1x_cdma.c
 *
 * Tegra Graphics Host Command DMA
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include <linux/slab.h>
#include "nvhost_acm.h"
#include "nvhost_cdma.h"
#include "nvhost_channel.h"
#include "dev.h"

#include "host1x_hardware.h"
#include "host1x_syncpt.h"
#include "host1x_cdma.h"
#include "host1x_hwctx.h"

static inline u32 host1x_channel_dmactrl(int stop, int get_rst, int init_get)
{
	return HOST1X_CREATE(CHANNEL_DMACTRL, DMASTOP, stop)
			| HOST1X_CREATE(CHANNEL_DMACTRL, DMAGETRST, get_rst)
			| HOST1X_CREATE(CHANNEL_DMACTRL, DMAINITGET, init_get);
}

static void cdma_timeout_handler(struct work_struct *work);

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
static void push_buffer_reset(struct push_buffer *pb)
{
	pb->fence = PUSH_BUFFER_SIZE - 8;
	pb->cur = 0;
}

/**
 * Init push buffer resources
 */
static int push_buffer_init(struct push_buffer *pb)
{
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvmap_client *nvmap = cdma_to_nvmap(cdma);
	pb->mem = NULL;
	pb->mapped = NULL;
	pb->phys = 0;
	pb->nvmap = NULL;

	BUG_ON(!cdma_pb_op().reset);
	cdma_pb_op().reset(pb);

	/* allocate and map pushbuffer memory */
	pb->mem = nvmap_alloc(nvmap, PUSH_BUFFER_SIZE + 4, 32,
			      NVMAP_HANDLE_WRITE_COMBINE, 0);
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

	/* memory for storing nvmap client and handles for each opcode pair */
	pb->nvmap = kzalloc(NVHOST_GATHER_QUEUE_SIZE *
				sizeof(struct nvmap_client_handle),
			GFP_KERNEL);
	if (!pb->nvmap)
		goto fail;

	/* put the restart at the end of pushbuffer memory */
	*(pb->mapped + (PUSH_BUFFER_SIZE >> 2)) =
		nvhost_opcode_restart(pb->phys);

	return 0;

fail:
	cdma_pb_op().destroy(pb);
	return -ENOMEM;
}

/**
 * Clean up push buffer resources
 */
static void push_buffer_destroy(struct push_buffer *pb)
{
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	struct nvmap_client *nvmap = cdma_to_nvmap(cdma);
	if (pb->mapped)
		nvmap_munmap(pb->mem, pb->mapped);

	if (pb->phys != 0)
		nvmap_unpin(nvmap, pb->mem);

	if (pb->mem)
		nvmap_free(nvmap, pb->mem);

	kfree(pb->nvmap);

	pb->mem = NULL;
	pb->mapped = NULL;
	pb->phys = 0;
	pb->nvmap = 0;
}

/**
 * Push two words to the push buffer
 * Caller must ensure push buffer is not full
 */
static void push_buffer_push_to(struct push_buffer *pb,
		struct nvmap_client *client,
		struct nvmap_handle_ref *handle, u32 op1, u32 op2)
{
	u32 cur = pb->cur;
	u32 *p = (u32 *)((u32)pb->mapped + cur);
	u32 cur_nvmap = (cur/8) & (NVHOST_GATHER_QUEUE_SIZE - 1);
	BUG_ON(cur == pb->fence);
	*(p++) = op1;
	*(p++) = op2;
	pb->nvmap[cur_nvmap].client = client;
	pb->nvmap[cur_nvmap].handle = handle;
	pb->cur = (cur + 8) & (PUSH_BUFFER_SIZE - 1);
}

/**
 * Pop a number of two word slots from the push buffer
 * Caller must ensure push buffer is not empty
 */
static void push_buffer_pop_from(struct push_buffer *pb,
		unsigned int slots)
{
	/* Clear the nvmap references for old items from pb */
	unsigned int i;
	u32 fence_nvmap = pb->fence/8;
	for (i = 0; i < slots; i++) {
		int cur_fence_nvmap = (fence_nvmap+i)
				& (NVHOST_GATHER_QUEUE_SIZE - 1);
		struct nvmap_client_handle *h =
				&pb->nvmap[cur_fence_nvmap];
		h->client = NULL;
		h->handle = NULL;
	}
	/* Advance the next write position */
	pb->fence = (pb->fence + slots * 8) & (PUSH_BUFFER_SIZE - 1);
}

/**
 * Return the number of two word slots free in the push buffer
 */
static u32 push_buffer_space(struct push_buffer *pb)
{
	return ((pb->fence - pb->cur) & (PUSH_BUFFER_SIZE - 1)) / 8;
}

static u32 push_buffer_putptr(struct push_buffer *pb)
{
	return pb->phys + pb->cur;
}

/*
 * The syncpt incr buffer is filled with methods to increment syncpts, which
 * is later GATHER-ed into the mainline PB. It's used when a timed out context
 * is interleaved with other work, so needs to inline the syncpt increments
 * to maintain the count (but otherwise does no work).
 */

/**
 * Init timeout resources
 */
static int cdma_timeout_init(struct nvhost_cdma *cdma,
				 u32 syncpt_id)
{
	if (syncpt_id == NVSYNCPT_INVALID)
		return -EINVAL;

	INIT_DELAYED_WORK(&cdma->timeout.wq, cdma_timeout_handler);
	cdma->timeout.initialized = true;

	return 0;
}

/**
 * Clean up timeout resources
 */
static void cdma_timeout_destroy(struct nvhost_cdma *cdma)
{
	if (cdma->timeout.initialized)
		cancel_delayed_work(&cdma->timeout.wq);
	cdma->timeout.initialized = false;
}

/**
 * Increment timedout buffer's syncpt via CPU.
 */
static void cdma_timeout_cpu_incr(struct nvhost_cdma *cdma, u32 getptr,
				u32 syncpt_incrs, u32 syncval, u32 nr_slots)
{
	struct nvhost_master *dev = cdma_to_dev(cdma);
	struct push_buffer *pb = &cdma->push_buffer;
	u32 i, getidx;

	for (i = 0; i < syncpt_incrs; i++)
		nvhost_syncpt_cpu_incr(&dev->syncpt, cdma->timeout.syncpt_id);

	/* after CPU incr, ensure shadow is up to date */
	nvhost_syncpt_update_min(&dev->syncpt, cdma->timeout.syncpt_id);

	/* update WAITBASE_3D by same number of incrs */
	if (cdma->timeout.syncpt_id == NVSYNCPT_3D) {
		void __iomem *p;
		p = dev->sync_aperture + HOST1X_SYNC_SYNCPT_BASE_0 +
				(NVWAITBASE_3D * sizeof(u32));
		writel(syncval, p);
	}

	/* NOP all the PB slots */
	getidx = getptr - pb->phys;
	while (nr_slots--) {
		u32 *p = (u32 *)((u32)pb->mapped + getidx);
		*(p++) = NVHOST_OPCODE_NOOP;
		*(p++) = NVHOST_OPCODE_NOOP;
		dev_dbg(&dev->dev->dev, "%s: NOP at 0x%x\n",
			__func__, pb->phys + getidx);
		getidx = (getidx + 8) & (PUSH_BUFFER_SIZE - 1);
	}
	wmb();
}

/**
 * Start channel DMA
 */
static void cdma_start(struct nvhost_cdma *cdma)
{
	void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;

	if (cdma->running)
		return;

	BUG_ON(!cdma_pb_op().putptr);
	cdma->last_put = cdma_pb_op().putptr(&cdma->push_buffer);

	writel(host1x_channel_dmactrl(true, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	/* set base, put, end pointer (all of memory) */
	writel(0, chan_regs + HOST1X_CHANNEL_DMASTART);
	writel(cdma->last_put, chan_regs + HOST1X_CHANNEL_DMAPUT);
	writel(0xFFFFFFFF, chan_regs + HOST1X_CHANNEL_DMAEND);

	/* reset GET */
	writel(host1x_channel_dmactrl(true, true, true),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	/* start the command DMA */
	writel(host1x_channel_dmactrl(false, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	cdma->running = true;
}

/**
 * Similar to cdma_start(), but rather than starting from an idle
 * state (where DMA GET is set to DMA PUT), on a timeout we restore
 * DMA GET from an explicit value (so DMA may again be pending).
 */
static void cdma_timeout_restart(struct nvhost_cdma *cdma, u32 getptr)
{
	struct nvhost_master *dev = cdma_to_dev(cdma);
	void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;

	if (cdma->running)
		return;

	BUG_ON(!cdma_pb_op().putptr);
	cdma->last_put = cdma_pb_op().putptr(&cdma->push_buffer);

	writel(host1x_channel_dmactrl(true, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	/* set base, end pointer (all of memory) */
	writel(0, chan_regs + HOST1X_CHANNEL_DMASTART);
	writel(0xFFFFFFFF, chan_regs + HOST1X_CHANNEL_DMAEND);

	/* set GET, by loading the value in PUT (then reset GET) */
	writel(getptr, chan_regs + HOST1X_CHANNEL_DMAPUT);
	writel(host1x_channel_dmactrl(true, true, true),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	dev_dbg(&dev->dev->dev,
		"%s: DMA GET 0x%x, PUT HW 0x%x / shadow 0x%x\n",
		__func__,
		readl(chan_regs + HOST1X_CHANNEL_DMAGET),
		readl(chan_regs + HOST1X_CHANNEL_DMAPUT),
		cdma->last_put);

	/* deassert GET reset and set PUT */
	writel(host1x_channel_dmactrl(true, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);
	writel(cdma->last_put, chan_regs + HOST1X_CHANNEL_DMAPUT);

	/* start the command DMA */
	writel(host1x_channel_dmactrl(false, false, false),
		chan_regs + HOST1X_CHANNEL_DMACTRL);

	cdma->running = true;
}

/**
 * Kick channel DMA into action by writing its PUT offset (if it has changed)
 */
static void cdma_kick(struct nvhost_cdma *cdma)
{
	u32 put;
	BUG_ON(!cdma_pb_op().putptr);

	put = cdma_pb_op().putptr(&cdma->push_buffer);

	if (put != cdma->last_put) {
		void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;
		wmb();
		writel(put, chan_regs + HOST1X_CHANNEL_DMAPUT);
		cdma->last_put = put;
	}
}

static void cdma_stop(struct nvhost_cdma *cdma)
{
	void __iomem *chan_regs = cdma_to_channel(cdma)->aperture;

	mutex_lock(&cdma->lock);
	if (cdma->running) {
		nvhost_cdma_wait_locked(cdma, CDMA_EVENT_SYNC_QUEUE_EMPTY);
		writel(host1x_channel_dmactrl(true, false, false),
			chan_regs + HOST1X_CHANNEL_DMACTRL);
		cdma->running = false;
	}
	mutex_unlock(&cdma->lock);
}

/**
 * Retrieve the op pair at a slot offset from a DMA address
 */
void cdma_peek(struct nvhost_cdma *cdma,
			  u32 dmaget, int slot, u32 *out)
{
	u32 offset = dmaget - cdma->push_buffer.phys;
	u32 *p = cdma->push_buffer.mapped;

	offset = ((offset + slot * 8) & (PUSH_BUFFER_SIZE - 1)) >> 2;
	out[0] = p[offset];
	out[1] = p[offset + 1];
}

/**
 * Stops both channel's command processor and CDMA immediately.
 * Also, tears down the channel and resets corresponding module.
 */
void cdma_timeout_teardown_begin(struct nvhost_cdma *cdma)
{
	struct nvhost_master *dev = cdma_to_dev(cdma);
	struct nvhost_channel *ch = cdma_to_channel(cdma);
	u32 cmdproc_stop;

	BUG_ON(cdma->torndown);

	dev_dbg(&dev->dev->dev,
		"begin channel teardown (channel id %d)\n", ch->chid);

	cmdproc_stop = readl(dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);
	cmdproc_stop |= BIT(ch->chid);
	writel(cmdproc_stop, dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);

	dev_dbg(&dev->dev->dev,
		"%s: DMA GET 0x%x, PUT HW 0x%x / shadow 0x%x\n",
		__func__,
		readl(ch->aperture + HOST1X_CHANNEL_DMAGET),
		readl(ch->aperture + HOST1X_CHANNEL_DMAPUT),
		cdma->last_put);

	writel(host1x_channel_dmactrl(true, false, false),
		ch->aperture + HOST1X_CHANNEL_DMACTRL);

	writel(BIT(ch->chid), dev->sync_aperture + HOST1X_SYNC_CH_TEARDOWN);
	nvhost_module_reset(ch->dev);

	cdma->running = false;
	cdma->torndown = true;
}

void cdma_timeout_teardown_end(struct nvhost_cdma *cdma, u32 getptr)
{
	struct nvhost_master *dev = cdma_to_dev(cdma);
	struct nvhost_channel *ch = cdma_to_channel(cdma);
	u32 cmdproc_stop;

	BUG_ON(!cdma->torndown || cdma->running);

	dev_dbg(&dev->dev->dev,
		"end channel teardown (id %d, DMAGET restart = 0x%x)\n",
		ch->chid, getptr);

	cmdproc_stop = readl(dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);
	cmdproc_stop &= ~(BIT(ch->chid));
	writel(cmdproc_stop, dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);

	cdma->torndown = false;
	cdma_timeout_restart(cdma, getptr);
}

/**
 * If this timeout fires, it indicates the current sync_queue entry has
 * exceeded its TTL and the userctx should be timed out and remaining
 * submits already issued cleaned up (future submits return an error).
 */
static void cdma_timeout_handler(struct work_struct *work)
{
	struct nvhost_cdma *cdma;
	struct nvhost_master *dev;
	struct nvhost_syncpt *sp;
	struct nvhost_channel *ch;

	u32 syncpt_val;

	u32 prev_cmdproc, cmdproc_stop;

	cdma = container_of(to_delayed_work(work), struct nvhost_cdma,
			    timeout.wq);
	dev = cdma_to_dev(cdma);
	sp = &dev->syncpt;
	ch = cdma_to_channel(cdma);

	mutex_lock(&cdma->lock);

	if (!cdma->timeout.clientid) {
		dev_dbg(&dev->dev->dev,
			 "cdma_timeout: expired, but has no clientid\n");
		mutex_unlock(&cdma->lock);
		return;
	}

	/* stop processing to get a clean snapshot */
	prev_cmdproc = readl(dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);
	cmdproc_stop = prev_cmdproc | BIT(ch->chid);
	writel(cmdproc_stop, dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);

	dev_dbg(&dev->dev->dev, "cdma_timeout: cmdproc was 0x%x is 0x%x\n",
		prev_cmdproc, cmdproc_stop);

	syncpt_val = nvhost_syncpt_update_min(&dev->syncpt,
			cdma->timeout.syncpt_id);

	/* has buffer actually completed? */
	if ((s32)(syncpt_val - cdma->timeout.syncpt_val) >= 0) {
		dev_dbg(&dev->dev->dev,
			 "cdma_timeout: expired, but buffer had completed\n");
		/* restore */
		cmdproc_stop = prev_cmdproc & ~(BIT(ch->chid));
		writel(cmdproc_stop,
			dev->sync_aperture + HOST1X_SYNC_CMDPROC_STOP);
		mutex_unlock(&cdma->lock);
		return;
	}

	dev_warn(&dev->dev->dev,
		"%s: timeout: %d (%s) ctx 0x%p, HW thresh %d, done %d\n",
		__func__,
		cdma->timeout.syncpt_id,
		syncpt_op().name(sp, cdma->timeout.syncpt_id),
		cdma->timeout.ctx,
		syncpt_val, cdma->timeout.syncpt_val);

	/* stop HW, resetting channel/module */
	cdma_op().timeout_teardown_begin(cdma);

	nvhost_cdma_update_sync_queue(cdma, sp, &dev->dev->dev);
	mutex_unlock(&cdma->lock);
}

int host1x_init_cdma_support(struct nvhost_chip_support *op)
{
	op->cdma.start = cdma_start;
	op->cdma.stop = cdma_stop;
	op->cdma.kick = cdma_kick;

	op->cdma.timeout_init = cdma_timeout_init;
	op->cdma.timeout_destroy = cdma_timeout_destroy;
	op->cdma.timeout_teardown_begin = cdma_timeout_teardown_begin;
	op->cdma.timeout_teardown_end = cdma_timeout_teardown_end;
	op->cdma.timeout_cpu_incr = cdma_timeout_cpu_incr;

	op->push_buffer.reset = push_buffer_reset;
	op->push_buffer.init = push_buffer_init;
	op->push_buffer.destroy = push_buffer_destroy;
	op->push_buffer.push_to = push_buffer_push_to;
	op->push_buffer.pop_from = push_buffer_pop_from;
	op->push_buffer.space = push_buffer_space;
	op->push_buffer.putptr = push_buffer_putptr;

	return 0;
}
