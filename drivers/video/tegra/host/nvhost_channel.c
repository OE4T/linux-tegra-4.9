/*
 * drivers/video/tegra/host/nvhost_channel.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010-2015, NVIDIA Corporation.  All rights reserved.
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
#include "chip_support.h"
#include "vhost/vhost.h"

#include <trace/events/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <linux/delay.h>
#include <linux/nvhost.h>
#include <linux/slab.h>

#define NVHOST_CHANNEL_LOW_PRIO_MAX_WAIT 50

/* Memory allocation for all supported channels */
int nvhost_alloc_channels(struct nvhost_master *host)
{
	struct nvhost_channel *ch;
	int index, err;

	host->chlist = kzalloc(nvhost_channel_nb_channels(host) *
			       sizeof(struct nvhost_channel *), GFP_KERNEL);
	if (host->chlist == NULL)
		return -ENOMEM;

	mutex_init(&host->chlist_mutex);

	for (index = 0;	index < nvhost_channel_nb_channels(host); index++) {
		ch = kzalloc(sizeof(*ch), GFP_KERNEL);
		if (!ch) {
			dev_err(&host->dev->dev, "failed to alloc channels\n");
			return -ENOMEM;
		}

		/* initialize data structures */
		nvhost_set_chanops(ch);
		mutex_init(&ch->submitlock);
		mutex_init(&ch->syncpts_lock);
		ch->chid = nvhost_channel_get_id_from_index(host, index);

		/* initialize channel cdma */
		err = nvhost_cdma_init(host->dev, &ch->cdma);
		if (err) {
			dev_err(&host->dev->dev, "failed to initialize cdma\n");
			return err;
		}

		/* initialize hw specifics */
		err = channel_op(ch).init(ch, host);
		if (err < 0) {
			dev_err(&host->dev->dev, "failed to init channel %d\n",
				ch->chid);
			return err;
		}

		/* store the channel */
		host->chlist[index] = ch;
	}

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
		if (ch)
			return ch;
	}

	return NULL;
}

/* Check if more than channel needed for device and assign */
static int nvhost_channel_assign(struct nvhost_device_data *pdata,
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
	int i = 0;
	int index;

	if (!ch->dev) {
		pr_err("%s: freeing unmapped channel\n", __func__);
		return 0;
	}

	pdata = platform_get_drvdata(ch->dev);
	host = nvhost_get_host(pdata->pdev);

	/* turn off channel cdma */
	channel_cdma_op().stop(&ch->cdma);

	if (channel_op(ch).set_low_ch_prio)
		channel_op(ch).set_low_ch_prio(ch);

	/* this is used only if we map channel on open */
	if (nvhost_get_channel_policy() == MAP_CHANNEL_ON_OPEN)
		pdata->num_mapped_chs--;

	/* log this event */
	dev_dbg(&ch->dev->dev, "channel %d un-mapped\n", ch->chid);
	trace_nvhost_channel_unmap_locked(pdata->pdev->name, ch->chid,
		pdata->num_mapped_chs);

	/* Release channel syncpoints */
	for (i = 0; i < NVHOST_MODULE_MAX_SYNCPTS; ++i) {
		/* skip over unused syncpoints */
		if (!ch->syncpts[i])
			continue;

		/* first, mark syncpoint as unused by hardware */
		nvhost_syncpt_mark_unused(&host->syncpt, ch->syncpts[i]);

		/* release syncpoint if we allocate syncpoints per channels */
		if (nvhost_get_syncpt_policy() == SYNCPT_PER_CHANNEL)
			nvhost_free_syncpt(ch->syncpts[i]);

		/* finally, clear information from channel bookkeeping */
		ch->syncpts[i] = 0;
	}

	if (ch->client_managed_syncpt) {
		/* mark syncpoint as unused */
		nvhost_syncpt_mark_unused(&host->syncpt,
					  ch->client_managed_syncpt);

		/* release it */
		if (nvhost_get_syncpt_policy() == SYNCPT_PER_CHANNEL)
			nvhost_free_syncpt(ch->client_managed_syncpt);

		/* ..and handle bookkeeping */
		ch->client_managed_syncpt = 0;
	}

	index = nvhost_channel_get_index_from_id(host, ch->chid);
	clear_bit(index, &host->allocated_channels);

	ch->dev = NULL;
	ch->refcount = 0;
	ch->identifier = NULL;

	if (nvhost_get_channel_policy() == MAP_CHANNEL_ON_OPEN)
		pdata->channels[ch->dev_chid] = NULL;

	return 0;
}

/* Maps free channel with device */
int nvhost_channel_map(struct nvhost_device_data *pdata,
			struct nvhost_channel **channel,
			void *identifier)
{
	struct nvhost_master *host = NULL;
	struct nvhost_channel *ch = NULL;
	int max_channels = 0;
	int index = 0;

	if (!pdata) {
		pr_err("%s: NULL device data\n", __func__);
		return -EINVAL;
	}

	host = nvhost_get_host(pdata->pdev);

	mutex_lock(&host->chlist_mutex);
	max_channels = nvhost_channel_nb_channels(host);

	if (nvhost_get_channel_policy() == MAP_CHANNEL_ON_SUBMIT) {
		/* check if the channel is still in use */
		ch = *channel;
		if (ch && ch->refcount && ch->identifier == identifier) {
			/* yes, client can continue using it */
			ch->refcount++;
			mutex_unlock(&host->chlist_mutex);
			return 0;
		}
	} else if (pdata->num_channels == pdata->num_mapped_chs) {
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

	do {
		index = find_first_zero_bit(&host->allocated_channels,
					    max_channels);
		if (index >= max_channels) {
			mutex_unlock(&host->chlist_mutex);
			if (nvhost_get_channel_policy() !=
				MAP_CHANNEL_ON_SUBMIT)
				return -ENOMEM;
			mdelay(1);
			mutex_lock(&host->chlist_mutex);
		}
	} while (index >= max_channels);

	/* Reserve the channel */
	set_bit(index, &host->allocated_channels);
	ch = host->chlist[index];

	/* If virtual, allocate a client id on the server side. This is needed
	 * for channel recovery, to distinguish which clients own which gathers.
	 */
	if (nvhost_dev_is_virtual(pdata->pdev)) {
		int virt_moduleid = vhost_virt_moduleid(pdata->moduleid);
		struct nvhost_virt_ctx *virt_ctx =
					nvhost_get_virt_data(pdata->pdev);

		if (virt_moduleid < 0) {
			clear_bit(index, &host->allocated_channels);
			mutex_unlock(&host->chlist_mutex);
			return -EINVAL;
		}

		ch->virt_clientid =
			vhost_channel_alloc_clientid(virt_ctx->handle,
							virt_moduleid);
		if (ch->virt_clientid == 0) {
			dev_err(&pdata->pdev->dev,
				"vhost_channel_alloc_clientid failed\n");
			clear_bit(index, &host->allocated_channels);
			mutex_unlock(&host->chlist_mutex);
			return -ENOMEM;
		}
	}

	/* Bind the reserved channel to the device */
	ch->dev = pdata->pdev;
	ch->identifier = identifier;
	if (nvhost_get_channel_policy() == MAP_CHANNEL_ON_OPEN)
		nvhost_channel_assign(pdata, ch);
	ch->refcount = 1;

	/* Handle logging */
	trace_nvhost_channel_map(pdata->pdev->name, ch->chid,
				 pdata->num_mapped_chs);
	dev_dbg(&ch->dev->dev, "channel %d mapped\n", ch->chid);

	mutex_unlock(&host->chlist_mutex);

	*channel = ch;
	return 0;
}
EXPORT_SYMBOL(nvhost_channel_map);

/* Free channel memory and list */
int nvhost_channel_list_free(struct nvhost_master *host)
{
	int i;

	for (i = 0; i < nvhost_channel_nb_channels(host); i++)
		kfree(host->chlist[i]);

	dev_info(&host->dev->dev, "channel list free'd\n");

	return 0;
}

void nvhost_channel_init_gather_filter(struct nvhost_channel *ch)
{
	if (channel_op(ch).init_gather_filter)
		channel_op(ch).init_gather_filter(ch);
}

int nvhost_channel_submit(struct nvhost_job *job)
{
	return channel_op(job->ch).submit(job);
}
EXPORT_SYMBOL(nvhost_channel_submit);

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
EXPORT_SYMBOL(nvhost_putchannel);

int nvhost_channel_suspend(struct nvhost_master *host)
{
	int i;

	for (i = 0; i < nvhost_channel_nb_channels(host); i++) {
		struct nvhost_channel *ch = host->chlist[i];
		if (channel_cdma_op().stop && ch->dev)
			channel_cdma_op().stop(&ch->cdma);
	}

	return 0;
}

int nvhost_channel_nb_channels(struct nvhost_master *host)
{
	return host->info.nb_channels;
}

int nvhost_channel_ch_base(struct nvhost_master *host)
{
	return host->info.ch_base;
}

int nvhost_channel_ch_limit(struct nvhost_master *host)
{
	return host->info.ch_limit;
}

int nvhost_channel_get_id_from_index(struct nvhost_master *host, int index)
{
	return nvhost_channel_ch_base(host) + index;
}

int nvhost_channel_get_index_from_id(struct nvhost_master *host, int chid)
{
	return chid - nvhost_channel_ch_base(host);
}
