/*
 * drivers/video/tegra/host/nvhost_channel.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2014, NVIDIA Corporation.  All rights reserved.
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

#include "nvhost_channel.h"
#include "dev.h"
#include "nvhost_acm.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include "chip_support.h"

#include <trace/events/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <linux/slab.h>

#define NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT 50

/* Constructor for the host1x device list */
int nvhost_channel_list_init(struct nvhost_master *host)
{
	INIT_LIST_HEAD(&host->chlist.list);
	mutex_init(&host->chlist_mutex);

	if (host->info.nb_channels > BITS_PER_LONG) {
		WARN(1, "host1x hardware has more channels than supported\n");
		return -ENOSYS;
	}

	return 0;
}

int nvhost_alloc_channels(struct nvhost_master *host)
{
	int max_channels = host->info.nb_channels;
	int i;
	struct nvhost_channel *ch;

	nvhost_channel_list_init(host);
	mutex_lock(&host->chlist_mutex);

	for (i = 0; i < max_channels; i++) {
		ch = nvhost_alloc_channel_internal(i, max_channels,
					&host->cnt_alloc_channels);
		if (!ch) {
			mutex_unlock(&host->chlist_mutex);
			return -ENOMEM;
		}
		ch->dev = NULL;
		ch->chid = NVHOST_INVALID_CHANNEL;

		list_add_tail(&ch->list, &host->chlist.list);
	}
	mutex_unlock(&host->chlist_mutex);

	return 0;
}

int nvhost_channel_unmap(struct nvhost_channel *ch)
{
	struct nvhost_device_data *pdata;
	struct nvhost_master *host;

	if (!ch->dev) {
		pr_warn("%s channel already unmapped\n", __func__);
		return 0;
	}

	pdata = platform_get_drvdata(ch->dev);
	host = nvhost_get_host(ch->dev);

	host->allocated_channels &= ~BIT(ch->chid);
	ch->chid = NVHOST_INVALID_CHANNEL;
	ch->dev = NULL;
	ch->ctxhandler = NULL;
	ch->cur_ctx = NULL;
	ch->aperture = NULL;
	pdata->channel = NULL;

	return 0;
}

struct nvhost_channel *nvhost_channel_map(struct nvhost_device_data *pdata)
{
	struct nvhost_master *host = NULL;
	struct nvhost_channel *ch = NULL;
	int max_channels = 0;
	int index = 0;
	int err;

	if (!pdata)
		return NULL;

	host = nvhost_get_host(pdata->pdev);
	max_channels = host->info.nb_channels;

	/* Check if already channel assigned for device */
	if (pdata->channel) {
		ch = pdata->channel;
		return ch;
	}
	mutex_lock(&host->chlist_mutex);
	list_for_each_entry(ch, &host->chlist.list, list) {
		index++;
		if (ch->chid == NVHOST_INVALID_CHANNEL) {
			ch->dev = pdata->pdev;
			ch->chid = index;
			pdata->channel = ch;
			nvhost_set_chanops(ch);
			break;
		}
	}
	mutex_unlock(&host->chlist_mutex);

	if (index > max_channels) {
		pr_warn("%s: All channels are in use!\n", __func__);
		return NULL;
	}

	err = nvhost_channel_init(ch, host);
	if (err) {
		nvhost_channel_unmap(ch);
		return NULL;
	}
	host->allocated_channels |= BIT(index);

	return ch;
}

int nvhost_channel_list_free(struct nvhost_master *host)
{
	return 0;
}
int nvhost_channel_init(struct nvhost_channel *ch,
		struct nvhost_master *dev)
{
	int err;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);

	/* Link platform_device to nvhost_channel */
	err = channel_op(ch).init(ch, dev);
	if (err < 0) {
		dev_err(&dev->dev->dev, "failed to init channel %d\n",
				ch->chid);
		return err;
	}
	pdata->channel = ch;

	return nvhost_cdma_init(&ch->cdma);
}

void nvhost_channel_init_gather_filter(struct nvhost_channel *ch)
{
	if (channel_op(ch).init_gather_filter)
		channel_op(ch).init_gather_filter(ch);
}

int nvhost_channel_submit(struct nvhost_job *job)
{
	/*
	 * Check if queue has higher priority jobs running. If so, wait until
	 * queue is empty. Ignores result from nvhost_cdma_flush, as we submit
	 * either when push buffer is empty or when we reach the timeout.
	 */
	int higher_count = 0;

	switch (job->priority) {
	case NVHOST_PRIORITY_HIGH:
		higher_count = 0;
		break;
	case NVHOST_PRIORITY_MEDIUM:
		higher_count = job->ch->cdma.high_prio_count;
		break;
	case NVHOST_PRIORITY_LOW:
		higher_count = job->ch->cdma.high_prio_count
			+ job->ch->cdma.med_prio_count;
		break;
	}
	if (higher_count > 0)
		(void)nvhost_cdma_flush(&job->ch->cdma,
				NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT);

	return channel_op(job->ch).submit(job);
}

void nvhost_add_refcnt(struct nvhost_channel *ch, int cnt)
{
	mutex_lock(&ch->reflock);
	ch->refcount += cnt;
	mutex_unlock(&ch->reflock);
}

void nvhost_sub_refcnt(struct nvhost_channel *ch, int cnt)
{
	mutex_lock(&ch->reflock);
	ch->refcount -= cnt;
	mutex_unlock(&ch->reflock);
}

struct nvhost_channel *nvhost_getchannel(struct nvhost_channel *ch,
		bool force, bool init)
{
	int err = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);

	mutex_lock(&ch->reflock);
	if (ch->refcount == 0) {
		if (!init)
			err = -EBUSY;
		else if (pdata->init)
			err = pdata->init(ch->dev);
	} else if (pdata->exclusive && !force)
		err = -EBUSY;

	if (!err)
		ch->refcount++;

	mutex_unlock(&ch->reflock);

	/* Keep alive modules that needs to be when a channel is open */
	if (!err && pdata->keepalive)
		nvhost_module_disable_poweroff(ch->dev);

	return err ? NULL : ch;
}

void nvhost_putchannel(struct nvhost_channel *ch, bool deinit)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);

	/* Allow keep-alive'd module to be turned off */
	if (pdata->keepalive)
		nvhost_module_enable_poweroff(ch->dev);

	mutex_lock(&ch->reflock);
	if (ch->refcount <= 1 && deinit && pdata->deinit) {
		pdata->deinit(ch->dev);
		nvhost_channel_unmap(ch);
	}

	ch->refcount--;
	mutex_unlock(&ch->reflock);
}

int nvhost_channel_suspend(struct nvhost_channel *ch)
{
	int ret = 0;

	if (channel_cdma_op().stop && ch->dev)
		channel_cdma_op().stop(&ch->cdma);

	return ret;
}

struct nvhost_channel *nvhost_alloc_channel_internal(int chindex,
	int max_channels, int *current_channel_count)
{
	struct nvhost_channel *ch = NULL;

	if ( (chindex > max_channels) ||
	     ( (*current_channel_count + 1) > max_channels) )
		return NULL;
	else {
		ch = kzalloc(sizeof(*ch), GFP_KERNEL);
		if (ch == NULL)
			return NULL;
		else {
			ch->chid = *current_channel_count;
			(*current_channel_count)++;
			return ch;
		}
	}
}

void nvhost_free_channel_internal(struct nvhost_channel *ch,
	int *current_channel_count)
{
	kfree(ch);
	(*current_channel_count)--;
}

int nvhost_channel_save_context(struct nvhost_channel *ch)
{
	int err = 0;

	if (ch && ch->cur_ctx)
		err = channel_op(ch).save_context(ch);

	return err;

}

int nvhost_channel_read_reg(struct nvhost_channel *ch,
	struct nvhost_hwctx *hwctx,
	u32 offset, u32 *value)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	if (!pdata->read_reg)
		return -EINVAL;

	return pdata->read_reg(ch->dev, ch, hwctx, offset, value);
}

static struct nvhost_hwctx *alloc_hwctx(struct nvhost_hwctx_handler *h,
		struct nvhost_channel *ch)
{
	struct nvhost_hwctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	kref_init(&ctx->ref);
	ctx->h = h;
	ctx->channel = ch;
	ctx->valid = true;

	return ctx;
}

static void free_hwctx(struct kref *ref)
{
	struct nvhost_hwctx *ctx = container_of(ref, struct nvhost_hwctx, ref);

	kfree(ctx);
}

static void get_hwctx(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

static void put_hwctx(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, free_hwctx);
}

struct nvhost_hwctx_handler *nvhost_alloc_hwctx_handler(u32 syncpt,
	u32 waitbase, struct nvhost_channel *ch)
{
	struct nvhost_hwctx_handler *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	p->syncpt = NVSYNCPT_INVALID;
	p->waitbase = waitbase;

	p->alloc = alloc_hwctx;
	p->get   = get_hwctx;
	p->put   = put_hwctx;

	return p;
}
