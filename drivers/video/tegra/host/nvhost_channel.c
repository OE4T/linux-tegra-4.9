/*
 * drivers/video/tegra/host/nvhost_channel.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include "nvhost_channel.h"
#include "dev.h"
#include "nvhost_hwctx.h"
#include "nvhost_job.h"
#include <trace/events/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <linux/slab.h>

#include <linux/platform_device.h>

#define NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT 50

int nvhost_channel_init(struct nvhost_channel *ch,
		struct nvhost_master *dev, int index)
{
	int err;
	struct nvhost_device *ndev;
	struct resource *r = NULL;
	void __iomem *regs = NULL;
	struct resource *reg_mem = NULL;

	/* Link nvhost_device to nvhost_channel */
	err = host_channel_op(dev).init(ch, dev, index);
	if (err < 0) {
		dev_err(&dev->dev->dev, "failed to init channel %d\n",
				index);
		return err;
	}
	ndev = ch->dev;
	ndev->channel = ch;

	/* Map IO memory related to nvhost_device */
	if (ndev->moduleid != NVHOST_MODULE_NONE) {
		/* First one is host1x - skip that */
		r = platform_get_resource(dev->pdev,
				IORESOURCE_MEM, ndev->moduleid + 1);
		if (!r)
			goto fail;

		reg_mem = request_mem_region(r->start,
				resource_size(r), ndev->name);
		if (!reg_mem)
			goto fail;

		regs = ioremap(r->start, resource_size(r));
		if (!regs)
			goto fail;

		ndev->reg_mem = reg_mem;
		ndev->aperture = regs;
	}
	return 0;

fail:
	if (reg_mem)
		release_mem_region(r->start, resource_size(r));
	if (regs)
		iounmap(regs);
	dev_err(&ndev->dev, "failed to get register memory\n");
	return -ENXIO;

}

int nvhost_channel_submit(struct nvhost_job *job)
{
	/* Low priority submits wait until sync queue is empty. Ignores result
	 * from nvhost_cdma_flush, as we submit either when push buffer is
	 * empty or when we reach the timeout. */
	if (job->priority < NVHOST_PRIORITY_MEDIUM)
		(void)nvhost_cdma_flush(&job->ch->cdma,
				NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT);

	return channel_op(job->ch).submit(job);
}

struct nvhost_channel *nvhost_getchannel(struct nvhost_channel *ch)
{
	int err = 0;
	mutex_lock(&ch->reflock);
	if (ch->refcount == 0) {
		if (ch->dev->init)
			ch->dev->init(ch->dev);
		err = nvhost_cdma_init(&ch->cdma);
	} else if (ch->dev->exclusive) {
		err = -EBUSY;
	}
	if (!err)
		ch->refcount++;

	mutex_unlock(&ch->reflock);

	/* Keep alive modules that needs to be when a channel is open */
	if (!err && ch->dev->keepalive)
		nvhost_module_busy(ch->dev);

	return err ? NULL : ch;
}

void nvhost_putchannel(struct nvhost_channel *ch, struct nvhost_hwctx *ctx)
{
	BUG_ON(!channel_cdma_op(ch).stop);

	if (ctx) {
		mutex_lock(&ch->submitlock);
		if (ch->cur_ctx == ctx)
			ch->cur_ctx = NULL;
		mutex_unlock(&ch->submitlock);
	}

	/* Allow keep-alive'd module to be turned off */
	if (ch->dev->keepalive)
		nvhost_module_idle(ch->dev);

	mutex_lock(&ch->reflock);
	if (ch->refcount == 1) {
		channel_cdma_op(ch).stop(&ch->cdma);
		nvhost_cdma_deinit(&ch->cdma);
		nvhost_module_suspend(ch->dev, false);
	}
	ch->refcount--;
	mutex_unlock(&ch->reflock);
}

int nvhost_channel_suspend(struct nvhost_channel *ch)
{
	int ret = 0;

	mutex_lock(&ch->reflock);
	BUG_ON(!channel_cdma_op(ch).stop);

	if (ch->refcount) {
		ret = nvhost_module_suspend(ch->dev, false);
		if (!ret)
			channel_cdma_op(ch).stop(&ch->cdma);
	}
	mutex_unlock(&ch->reflock);

	return ret;
}
