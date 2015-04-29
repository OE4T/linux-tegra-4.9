/*
 * drivers/video/tegra/host/host1x/host1x_cdma.c
 *
 * Tegra Graphics Host Command DMA
 *
 * Copyright (c) 2010-2015, NVIDIA Corporation. All rights reserved.
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
#include <linux/scatterlist.h>
#include "nvhost_acm.h"
#include "nvhost_cdma.h"
#include "nvhost_channel.h"
#include "debug.h"
#include "dev.h"
#include "class_ids.h"
#include "chip_support.h"
#include "nvhost_job.h"

#include "host1x_cdma.h"

static inline u32 host1x_channel_dmactrl(int stop, int get_rst, int init_get)
{
	return host1x_channel_dmactrl_dmastop_f(stop)
		| host1x_channel_dmactrl_dmagetrst_f(get_rst)
		| host1x_channel_dmactrl_dmainitget_f(init_get);
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
static void push_buffer_destroy(struct push_buffer *pb);
static int push_buffer_init(struct push_buffer *pb)
{
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	int err = 0;
	pb->mapped = NULL;
	pb->dma_addr = 0;

	cdma_pb_op().reset(pb);

	/* allocate the pushbuffer memory */
	pb->mapped = dma_alloc_writecombine(&cdma_to_dev(cdma)->dev->dev,
					PUSH_BUFFER_SIZE + 4,
					&pb->dma_addr,
					GFP_KERNEL);
	if (!pb->mapped) {
		err = -ENOMEM;
		pb->mapped = NULL;
		goto fail;
	}

	/* put the restart at the end of pushbuffer memory */
	*(pb->mapped + (PUSH_BUFFER_SIZE >> 2)) =
		nvhost_opcode_restart(pb->dma_addr);

	return 0;

fail:
	push_buffer_destroy(pb);
	return err;
}

/**
 * Clean up push buffer resources
 */
static void push_buffer_destroy(struct push_buffer *pb)
{
	struct nvhost_cdma *cdma = pb_to_cdma(pb);
	if (pb->mapped)
		dma_free_writecombine(&cdma_to_dev(cdma)->dev->dev,
					PUSH_BUFFER_SIZE + 4,
					pb->mapped,
					pb->dma_addr);

	pb->mapped = NULL;
	pb->dma_addr = 0;
}

/**
 * Push two words to the push buffer
 * Caller must ensure push buffer is not full
 */
static void push_buffer_push_to(struct push_buffer *pb,
				u32 op1, u32 op2)
{
	u32 cur = pb->cur;
	u32 *p = (u32 *)((uintptr_t)pb->mapped + cur);
	WARN_ON(cur == pb->fence);
	*(p++) = op1;
	*(p++) = op2;
	pb->cur = (cur + 8) & (PUSH_BUFFER_SIZE - 1);
}

/**
 * Pop a number of two word slots from the push buffer
 * Caller must ensure push buffer is not empty
 */
static void push_buffer_pop_from(struct push_buffer *pb,
		unsigned int slots)
{
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
	return pb->dma_addr + pb->cur;
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
static void cdma_timeout_pb_cleanup(struct nvhost_cdma *cdma, u32 getptr,
				u32 nr_slots)
{
	struct nvhost_master *dev = cdma_to_dev(cdma);
	struct push_buffer *pb = &cdma->push_buffer;
	u32 getidx;

	/* NOP all the PB slots */
	getidx = getptr - pb->dma_addr;
	while (nr_slots--) {
		u32 *p = (u32 *)((uintptr_t)pb->mapped + getidx);
		*(p++) = NVHOST_OPCODE_NOOP;
		*(p++) = NVHOST_OPCODE_NOOP;
		dev_dbg(&dev->dev->dev, "%s: NOP at 0x%llx\n",
			__func__, (u64)(pb->dma_addr + getidx));
		getidx = (getidx + 8) & (PUSH_BUFFER_SIZE - 1);
	}
	wmb();
}

/**
 * Start channel DMA
 */
static void cdma_start(struct nvhost_cdma *cdma)
{
	struct nvhost_channel *ch;

	if (cdma->running)
		return;

	ch = cdma_to_channel(cdma);
	if (!ch || !ch->dev) {
		pr_err("%s: channel already un-mapped\n", __func__);
		return;
	}

	cdma->last_put = cdma_pb_op().putptr(&cdma->push_buffer);

	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
		host1x_channel_dmactrl(true, false, false));

	/* set base, put, end pointer (all of memory) */
	host1x_channel_writel(ch, host1x_channel_dmastart_r(), 0);
	host1x_channel_writel(ch, host1x_channel_dmaput_r(), cdma->last_put);
	host1x_channel_writel(ch, host1x_channel_dmaend_r(), 0xFFFFFFFF);

	/* reset GET */
	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(true, true, true));

	/* prevent using setclass inside gathers */
	nvhost_channel_init_gather_filter(cdma_to_channel(cdma));

	/* start the command DMA */
	wmb();
	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(false, false, false));

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
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	if (cdma->running)
		return;

	cdma->last_put = cdma_pb_op().putptr(&cdma->push_buffer);

	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(true, false, false));

	/* set base, end pointer (all of memory) */
	host1x_channel_writel(ch, host1x_channel_dmastart_r(), 0);
	host1x_channel_writel(ch, host1x_channel_dmaend_r(), 0xFFFFFFFF);

	/* set GET, by loading the value in PUT (then reset GET) */
	host1x_channel_writel(ch, host1x_channel_dmaput_r(), getptr);
	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(true, true, true));

	dev_dbg(&dev->dev->dev,
		"%s: DMA GET 0x%x, PUT HW 0x%x / shadow 0x%x\n",
		__func__,
		host1x_channel_readl(ch, host1x_channel_dmaget_r()),
		host1x_channel_readl(ch, host1x_channel_dmaput_r()),
		cdma->last_put);

	/* deassert GET reset and set PUT */
	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(true, false, false));
	host1x_channel_writel(ch, host1x_channel_dmaput_r(), cdma->last_put);

	/* reinitialise gather filter for the channel */
	nvhost_channel_init_gather_filter(cdma_to_channel(cdma));

	/* start the command DMA */
	wmb();
	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(false, false, false));

	cdma->running = true;
}

/**
 * Kick channel DMA into action by writing its PUT offset (if it has changed)
 */
static void cdma_kick(struct nvhost_cdma *cdma)
{
	u32 put;
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	put = cdma_pb_op().putptr(&cdma->push_buffer);

	if (put != cdma->last_put) {
		wmb();
		host1x_channel_writel(ch, host1x_channel_dmaput_r(), put);
		cdma->last_put = put;
	}
}

static void cdma_stop(struct nvhost_cdma *cdma)
{
	void __iomem *chan_regs;
	struct nvhost_channel *ch = cdma_to_channel(cdma);

	if (!ch || !ch->dev) {
		pr_warn("%s: un-mapped channel\n", __func__);
		return;
	}
	chan_regs = ch->aperture;

	mutex_lock(&cdma->lock);
	if (cdma->running) {
		nvhost_cdma_wait_locked(cdma, CDMA_EVENT_SYNC_QUEUE_EMPTY);
		host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
				host1x_channel_dmactrl(true, false, false));
		cdma->running = false;
	}
	mutex_unlock(&cdma->lock);
}

static void cdma_timeout_release_mlocks(struct nvhost_cdma *cdma)
{
	struct nvhost_master *dev = cdma_to_dev(cdma);
	struct nvhost_syncpt *syncpt = &dev->syncpt;
	unsigned int chid = cdma_to_channel(cdma)->chid;
	int i;

	for (i = 0; i < nvhost_syncpt_nb_mlocks(syncpt); i++) {
		unsigned int owner;
		bool ch_own, cpu_own;
		syncpt_op().mutex_owner(syncpt, i, &cpu_own, &ch_own, &owner);

		if (!(ch_own && owner == chid))
			continue;

		syncpt_op().mutex_unlock(&dev->syncpt, i);
		dev_dbg(&dev->dev->dev, "released mlock %d\n", i);
	}

}

/**
 * Stops both channel's command processor and CDMA immediately.
 * Also, tears down the channel and resets corresponding module.
 */
static void cdma_timeout_teardown_begin(struct nvhost_cdma *cdma)
{
	struct nvhost_master *dev;
	struct nvhost_channel *ch = cdma_to_channel(cdma);
	u32 cmdproc_stop;

	dev = cdma_to_dev(cdma);
	if (cdma->torndown && !cdma->running) {
		dev_warn(&dev->dev->dev, "Already torn down\n");
		return;
	}

	dev_dbg(&dev->dev->dev,
		"begin channel teardown (channel id %d)\n", ch->chid);

	cmdproc_stop = host1x_sync_readl(dev->dev,
			host1x_sync_cmdproc_stop_r());
	cmdproc_stop |= BIT(ch->chid);
	host1x_sync_writel(dev->dev, host1x_sync_cmdproc_stop_r(),
			cmdproc_stop);

	dev_dbg(&dev->dev->dev,
		"%s: DMA GET 0x%x, PUT HW 0x%x / shadow 0x%x\n",
		__func__,
		host1x_channel_readl(ch, host1x_channel_dmaget_r()),
		host1x_channel_readl(ch, host1x_channel_dmaput_r()),
		cdma->last_put);

	host1x_channel_writel(ch, host1x_channel_dmactrl_r(),
			host1x_channel_dmactrl(true, false, false));

	host1x_sync_writel(dev->dev,
			host1x_sync_ch_teardown_r(), BIT(ch->chid));
	nvhost_module_reset(ch->dev, true);

	cdma_timeout_release_mlocks(cdma);

	cdma->running = false;
	cdma->torndown = true;
}

static void cdma_timeout_teardown_end(struct nvhost_cdma *cdma, u32 getptr)
{
	struct nvhost_master *dev;
	struct nvhost_channel *ch = cdma_to_channel(cdma);
	u32 cmdproc_stop;

	dev = cdma_to_dev(cdma);
	dev_dbg(&dev->dev->dev,
		"end channel teardown (id %d, DMAGET restart = 0x%x)\n",
		ch->chid, getptr);

	cmdproc_stop = host1x_sync_readl(dev->dev,
			host1x_sync_cmdproc_stop_r());
	cmdproc_stop &= ~(BIT(ch->chid));
	host1x_sync_writel(dev->dev, host1x_sync_cmdproc_stop_r(),
			cmdproc_stop);

	cdma->torndown = false;
	cdma_timeout_restart(cdma, getptr);
}

static bool cdma_check_dependencies(struct nvhost_cdma *cdma)
{
	struct nvhost_channel *ch = cdma_to_channel(cdma);
	struct nvhost_master *dev = cdma_to_dev(cdma);
	u32 cbstat = host1x_sync_readl(dev->dev,
		host1x_sync_cbstat_0_r() + 4 * ch->chid);
	u32 cbread = host1x_sync_readl(dev->dev,
		host1x_sync_cbread0_r() + 4 * ch->chid);
	u32 waiting = cbstat == 0x00010008;
	u32 syncpt_id = cbread >> 24;
	int i;

	if (!waiting)
		return false;

	for (i = 0; i < cdma->timeout.num_syncpts; ++i)
		if (cdma->timeout.sp[i].id == syncpt_id)
			return false;

	return true;
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
	int ret;
	bool completed;
	int i;

	u32 syncpt_val;

	u32 prev_cmdproc, cmdproc_stop;

	cdma = container_of(to_delayed_work(work), struct nvhost_cdma,
			    timeout.wq);
	ch = cdma_to_channel(cdma);
	if (!ch || !ch->dev) {
		pr_warn("%s: Channel un-mapped\n", __func__);
		return;
	}

	dev = cdma_to_dev(cdma);
	sp = &dev->syncpt;

	mutex_lock(&dev->timeout_mutex);

	ret = mutex_trylock(&cdma->lock);
	if (!ret) {
		schedule_delayed_work(&cdma->timeout.wq, msecs_to_jiffies(10));
		mutex_unlock(&dev->timeout_mutex);
		return;
	}

	if (nvhost_debug_force_timeout_dump ||
		cdma->timeout.timeout_debug_dump)
		nvhost_debug_dump_locked(cdma_to_dev(cdma), ch->chid);

	/* is this submit dependent with submits on other channels? */
	if (cdma->timeout.allow_dependency && cdma_check_dependencies(cdma)) {
		dev_dbg(&dev->dev->dev,
			"cdma_timeout: timeout handler rescheduled\n");
		cdma->timeout.allow_dependency = false;
		schedule_delayed_work(&cdma->timeout.wq,
				      msecs_to_jiffies(cdma->timeout.timeout));
		mutex_unlock(&cdma->lock);
		mutex_unlock(&dev->timeout_mutex);
		return;
	}

	if (!cdma->timeout.clientid) {
		dev_dbg(&dev->dev->dev,
			 "cdma_timeout: expired, but has no clientid\n");
		mutex_unlock(&cdma->lock);
		mutex_unlock(&dev->timeout_mutex);
		return;
	}

	/* stop processing to get a clean snapshot */
	prev_cmdproc = host1x_sync_readl(dev->dev,
			host1x_sync_cmdproc_stop_r());
	cmdproc_stop = prev_cmdproc | BIT(ch->chid);
	host1x_sync_writel(dev->dev,
			host1x_sync_cmdproc_stop_r(), cmdproc_stop);

	dev_dbg(&dev->dev->dev, "cdma_timeout: cmdproc was 0x%x is 0x%x\n",
		prev_cmdproc, cmdproc_stop);

	completed = true;
	for (i = 0; i < cdma->timeout.num_syncpts; ++i) {
		syncpt_val = nvhost_syncpt_update_min(&dev->syncpt,
				cdma->timeout.sp[i].id);

		if (!nvhost_syncpt_is_expired(&dev->syncpt,
			cdma->timeout.sp[i].id, cdma->timeout.sp[i].fence))
			completed = false;
	}

	/* has buffer actually completed? */
	if (completed) {
		dev_dbg(&dev->dev->dev,
			 "cdma_timeout: expired, but buffer had completed\n");
		/* restore */
		cmdproc_stop = prev_cmdproc & ~(BIT(ch->chid));
		host1x_sync_writel(dev->dev,
			host1x_sync_cmdproc_stop_r(), cmdproc_stop);
		mutex_unlock(&cdma->lock);
		mutex_unlock(&dev->timeout_mutex);
		return;
	}

	for (i = 0; i < cdma->timeout.num_syncpts; ++i) {
		syncpt_val = nvhost_syncpt_read_min(&dev->syncpt,
				cdma->timeout.sp[i].id);
		dev_warn(&dev->dev->dev,
			"%s: timeout: %d (%s) client %d, HW thresh %d, done %d\n",
			__func__, cdma->timeout.sp[i].id,
			syncpt_op().name(sp, cdma->timeout.sp[i].id),
			cdma->timeout.clientid, syncpt_val,
			cdma->timeout.sp[i].fence);
	}

	/* stop HW, resetting channel/module */
	cdma_op().timeout_teardown_begin(cdma);

	nvhost_cdma_update_sync_queue(cdma, sp, ch->dev);
	mutex_unlock(&cdma->lock);
	mutex_unlock(&dev->timeout_mutex);
}

static const struct nvhost_cdma_ops host1x_cdma_ops = {
	.start = cdma_start,
	.stop = cdma_stop,
	.kick = cdma_kick,

	.timeout_init = cdma_timeout_init,
	.timeout_destroy = cdma_timeout_destroy,
	.timeout_teardown_begin = cdma_timeout_teardown_begin,
	.timeout_teardown_end = cdma_timeout_teardown_end,
	.timeout_pb_cleanup = cdma_timeout_pb_cleanup,
};

static const struct nvhost_pushbuffer_ops host1x_pushbuffer_ops = {
	.reset = push_buffer_reset,
	.init = push_buffer_init,
	.destroy = push_buffer_destroy,
	.push_to = push_buffer_push_to,
	.pop_from = push_buffer_pop_from,
	.space = push_buffer_space,
	.putptr = push_buffer_putptr,
};

