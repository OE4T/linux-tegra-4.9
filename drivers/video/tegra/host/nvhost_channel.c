/*
 * drivers/video/tegra/host/nvhost_channel.c
 *
 * Tegra Graphics Host Channel
 *
 * Copyright (c) 2010, NVIDIA Corporation.
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
#include <trace/events/nvhost.h>

#include <linux/platform_device.h>

struct nvhost_channel *nvhost_getchannel(struct nvhost_channel *ch)
{
	int err = 0;
	mutex_lock(&ch->reflock);
	if (ch->refcount == 0) {
		err = nvhost_module_init(&ch->mod, ch->desc->name,
					ch->desc->power, &ch->dev->mod,
					&ch->dev->pdev->dev);
		if (!err) {
			err = nvhost_cdma_init(&ch->cdma);
			if (err)
				nvhost_module_deinit(&ch->mod);
		}
	} else if (ch->desc->exclusive) {
		err = -EBUSY;
	}
	if (!err) {
		ch->refcount++;
	}
	mutex_unlock(&ch->reflock);

	/* Keep alive modules that needs to be when a channel is open */
	if (!err && ch->desc->keepalive)
		nvhost_module_busy(&ch->mod);

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
	if (ch->desc->keepalive)
		nvhost_module_idle(&ch->mod);

	mutex_lock(&ch->reflock);
	if (ch->refcount == 1) {
		nvhost_module_deinit(&ch->mod);
		/* cdma may already be stopped, that's ok */

		channel_cdma_op(ch).stop(&ch->cdma);
		nvhost_cdma_deinit(&ch->cdma);
	}
	ch->refcount--;
	mutex_unlock(&ch->reflock);
}

void nvhost_channel_suspend(struct nvhost_channel *ch)
{
	mutex_lock(&ch->reflock);
	BUG_ON(nvhost_module_powered(&ch->mod));
	BUG_ON(!channel_cdma_op(ch).stop);

	if (ch->refcount)
		channel_cdma_op(ch).stop(&ch->cdma);
	mutex_unlock(&ch->reflock);
}
