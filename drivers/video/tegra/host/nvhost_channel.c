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
#include <linux/nvhost.h>
#include <linux/slab.h>

#define NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT 50

/* Constructor for the host1x device list */
int nvhost_channel_list_init(struct nvhost_master *host)
{
	if (host->info.nb_channels > BITS_PER_LONG) {
		WARN(1, "host1x hardware has more channels than supported\n");
		return -ENOSYS;
	}

	host->chlist = kzalloc(host->info.nb_channels *
				sizeof(struct nvhost_channel *), GFP_KERNEL);
	if (host->chlist == NULL)
			return -ENOMEM;

	mutex_init(&host->chlist_mutex);

	return 0;
}

/* Memory allocation for all supported channels */
int nvhost_alloc_channels(struct nvhost_master *host)
{
	int max_channels = host->info.nb_channels;
	int i, err = 0;
	struct nvhost_channel *ch;

	err = nvhost_channel_list_init(host);
	if (err) {
		dev_err(&host->dev->dev, "failed to init channel list\n");
		return err;
	}

	mutex_lock(&host->chlist_mutex);
	for (i = 0; i < max_channels; i++) {
		ch = nvhost_alloc_channel_internal(i, max_channels);
		if (!ch) {
			dev_err(&host->dev->dev, "failed to alloc channels\n");
			mutex_unlock(&host->chlist_mutex);
			return -ENOMEM;
		}
		host->chlist[i] = ch;
		ch->dev = NULL;
		ch->chid = NVHOST_INVALID_CHANNEL;
	}
	mutex_unlock(&host->chlist_mutex);

	return 0;
}

/* return any one of assigned channel from device
 * This API can be used to check if any channel assigned to device
 */
struct nvhost_channel *nvhost_check_channel(struct nvhost_device_data *pdata)
{
	int i;
	struct nvhost_channel *ch;

	for (i = 0; i < pdata->num_channels; i++) {
		ch = pdata->channels[i];
		if (ch && ch->chid != NVHOST_INVALID_CHANNEL)
			return ch;
	}

	return NULL;
}

/* Check if more than channel needed for device and assign */
int nvhost_channel_assign(struct nvhost_device_data *pdata,
			  struct nvhost_channel *ch)
{
	int i;

	for (i = 0; i < pdata->num_channels; i++) {
		if (!pdata->channels[i]) {
			pdata->channels[i] = ch;
			pdata->num_mapped_chs++;
			ch->dev_chid = i;
			return 0;
		}
	}
	dev_err(&pdata->pdev->dev, "%s: All channels assigned\n", __func__);

	return -EINVAL;
}

/* Releases all channels assigned with device */
int nvhost_channel_release(struct nvhost_device_data *pdata)
{
	struct nvhost_channel *ch;
	int i;

	for (i = 0; i < pdata->num_channels; i++) {
		ch = pdata->channels[i];
		if (ch && ch->dev)
			nvhost_putchannel(ch, 1);
	}
	return 0;
}
/* Unmap channel from device and free all resources, deinit device */
static int nvhost_channel_unmap_locked(struct nvhost_channel *ch)
{
	struct nvhost_device_data *pdata;
	struct nvhost_master *host;
	int max_channels;
	int i = 0;

	if (!ch->dev) {
		pr_err("%s: freeing unmapped channel\n", __func__);
		return 0;
	}

	pdata = platform_get_drvdata(ch->dev);
	host = nvhost_get_host(pdata->pdev);

	max_channels = host->info.nb_channels;

	if (ch->chid == NVHOST_INVALID_CHANNEL) {
		dev_err(&host->dev->dev, "Freeing un-mapped channel\n");
		return 0;
	}
	if (ch->error_notifier_ref)
		nvhost_free_error_notifiers(ch);

	dev_dbg(&ch->dev->dev, "channel %d un-mapped\n", ch->chid);

	pdata->num_mapped_chs--;

	trace_nvhost_channel_unmap_locked(pdata->pdev->name, ch->chid,
		pdata->num_mapped_chs);

	/* Allow keep-alive'd module to be turned off
	 * make sure that all channels are unmapped before calling
	 * nvhost_module_enable_poweroff
	 */
	if (!pdata->num_mapped_chs) {
		channel_cdma_op().stop(&ch->cdma);
		nvhost_cdma_deinit(&ch->cdma);

		if (pdata->keepalive)
			nvhost_module_enable_poweroff(pdata->pdev);
	}

	/* Release channel syncpoinits */
	for (i = 0; i < NVHOST_MODULE_MAX_SYNCPTS; ++i) {
		if (ch->syncpts[i]) {
			nvhost_free_syncpt(ch->syncpts[i]);
			ch->syncpts[i] = 0;
		}
	}
	clear_bit(ch->chid, &host->allocated_channels);

	ch->chid = NVHOST_INVALID_CHANNEL;
	ch->dev = NULL;
	kfree(ch->ctxhandler);
	ch->ctxhandler = NULL;
	ch->cur_ctx = NULL;
	ch->aperture = NULL;
	ch->refcount = 0;
	pdata->channels[ch->dev_chid] = NULL;

	return 0;
}

/* Maps free channel with device */
int nvhost_channel_map(struct nvhost_device_data *pdata,
			struct nvhost_channel **channel)
{
	struct nvhost_master *host = NULL;
	struct nvhost_channel *ch = NULL;
	int max_channels = 0;
	int index = 0;
	int err = 0;

	if (!pdata) {
		pr_err("%s: NULL device data\n", __func__);
		return -EINVAL;
	}

	host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	max_channels = host->info.nb_channels;

	/* Check if already channel(s) assigned for device */
	if (pdata->num_channels == pdata->num_mapped_chs) {
		if (pdata->exclusive) {
			mutex_unlock(&host->chlist_mutex);
			return -EBUSY;
		}
		ch = nvhost_check_channel(pdata);
		if (ch)
			ch->refcount++;
		mutex_unlock(&host->chlist_mutex);
		*channel = ch;
		return 0;
	}

	index = find_next_zero_bit(&host->allocated_channels,
					max_channels, host->next_free_ch);

	if (index >= max_channels) {
		/* Reset next pointer and try */
		host->next_free_ch = 0;
		index = find_next_zero_bit(&host->allocated_channels,
					max_channels, host->next_free_ch);
		if (index >= max_channels) {
			pr_err("All host1x channels are mapped, BITMAP: %lu\n",
					host->allocated_channels);
			mutex_unlock(&host->chlist_mutex);
			return -ENOMEM;
		}
	}

	/* Get channel from list and map to device */
	ch = host->chlist[index];
	if (!ch || (ch->chid != NVHOST_INVALID_CHANNEL)) {
		dev_err(&host->dev->dev, "%s: wrong channel map\n", __func__);
		mutex_unlock(&host->chlist_mutex);
		return -EINVAL;
	}

	ch->dev = pdata->pdev;
	ch->chid = index;
	nvhost_channel_assign(pdata, ch);
	nvhost_set_chanops(ch);
	set_bit(ch->chid, &host->allocated_channels);
	ch->refcount = 1;

	/* Initialize channel */
	err = nvhost_channel_init(ch, host);
	if (err) {
		dev_err(&ch->dev->dev, "%s: channel init failed\n", __func__);
		nvhost_channel_unmap_locked(ch);
		mutex_unlock(&host->chlist_mutex);
		return err;
	}

	/* set next free channel */
	if (index >= (max_channels - 1))
		host->next_free_ch = 0;
	else
		host->next_free_ch = index + 1;

	if (pdata->init && pdata->num_mapped_chs == 1) {
		err = pdata->init(ch->dev);
		if (err) {
			dev_err(&ch->dev->dev, "device init failed\n");
			nvhost_channel_unmap_locked(ch);
			mutex_unlock(&host->chlist_mutex);
			return err;
		}
	}

	/* Keep alive modules that needs to be when a channel is open */
	if (pdata->keepalive && pdata->num_mapped_chs)
		nvhost_module_disable_poweroff(pdata->pdev);

	 trace_nvhost_channel_map(pdata->pdev->name, ch->chid,
		pdata->num_mapped_chs);

	dev_dbg(&ch->dev->dev, "channel %d mapped\n", ch->chid);
	mutex_unlock(&host->chlist_mutex);

	*channel = ch;
	return 0;
}

/* Free channel memory and list */
int nvhost_channel_list_free(struct nvhost_master *host)
{
	int i;

	for (i = 0; i < host->info.nb_channels; i++)
		kfree(host->chlist[i]);

	dev_info(&host->dev->dev, "channel list free'd\n");

	return 0;
}

int nvhost_channel_init(struct nvhost_channel *ch,
		struct nvhost_master *dev)
{
	int err;

	/* Link platform_device to nvhost_channel */
	err = channel_op(ch).init(ch, dev);
	if (err < 0) {
		dev_err(&dev->dev->dev, "failed to init channel %d\n",
				ch->chid);
		return err;
	}

	return nvhost_cdma_init(&ch->cdma);
}

void nvhost_channel_init_gather_filter(struct nvhost_channel *ch)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	if (channel_op(ch).init_gather_filter && pdata->gather_filter_enabled)
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


void nvhost_getchannel(struct nvhost_channel *ch)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	struct nvhost_master *host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	ch->refcount++;
	trace_nvhost_getchannel(pdata->pdev->name, ch->refcount, ch->chid);
	mutex_unlock(&host->chlist_mutex);
}

void nvhost_putchannel(struct nvhost_channel *ch, int cnt)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	struct nvhost_master *host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	ch->refcount -= cnt;
	trace_nvhost_putchannel(pdata->pdev->name, ch->refcount, ch->chid);
	/* WARN on negative reference, with zero reference unmap channel*/
	if (!ch->refcount)
		nvhost_channel_unmap_locked(ch);
	else if (ch->refcount < 0)
		WARN_ON(1);
	mutex_unlock(&host->chlist_mutex);
}

int nvhost_channel_suspend(struct nvhost_channel *ch)
{
	int ret = 0;

	if (channel_cdma_op().stop && ch->dev)
		channel_cdma_op().stop(&ch->cdma);

	return ret;
}

struct nvhost_channel *nvhost_alloc_channel_internal(int chindex,
			int max_channels)
{
	struct nvhost_channel *ch = NULL;

	ch = kzalloc(sizeof(*ch), GFP_KERNEL);
	if (ch)
		ch->chid = chindex;

	return ch;
}

int nvhost_channel_save_context(struct nvhost_channel *ch)
{
	int err = 0;

	if (ch && ch->cur_ctx)
		err = channel_op(ch).save_context(ch);

	return err;

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
	struct nvhost_channel *ch)
{
	struct nvhost_hwctx_handler *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	p->syncpt = NVSYNCPT_INVALID;

	p->alloc = alloc_hwctx;
	p->get   = get_hwctx;
	p->put   = put_hwctx;

	return p;
}
