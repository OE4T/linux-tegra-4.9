/*
 * drivers/video/tegra/host/t30/channel_t30.c
 *
 * Tegra Graphics Host Channel
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

#include "3dctx_t30.h"
#include "../dev.h"
#include "../t20/channel_t20.h"
#include "../t20/hardware_t20.h"
#include "../t20/t20.h"

#define NVHOST_CHANNEL_BASE 0

static inline int t30_nvhost_hwctx_handler_init(
	struct nvhost_hwctx_handler *h,
	const char *module)
{
	if (strcmp(module, "gr3d") == 0)
		return t30_nvhost_3dctx_handler_init(h);

	return 0;
}

static inline void __iomem *t30_channel_aperture(void __iomem *p, int ndx)
{
	ndx += NVHOST_CHANNEL_BASE;
	p += NV_HOST1X_CHANNEL0_BASE;
	p += ndx * NV_HOST1X_CHANNEL_MAP_SIZE_BYTES;
	return p;
}


static int t30_channel_init(struct nvhost_channel *ch,
			    struct nvhost_master *dev, int index)
{
	ch->dev = dev;
	ch->chid = index;
	ch->desc = nvhost_t20_channelmap + index;
	mutex_init(&ch->reflock);
	mutex_init(&ch->submitlock);

	ch->aperture = t30_channel_aperture(dev->aperture, index);

	return t30_nvhost_hwctx_handler_init(&ch->ctxhandler, ch->desc->name);
}

int nvhost_init_t30_channel_support(struct nvhost_master *host)
{
	int result = nvhost_init_t20_channel_support(host);
	host->op.channel.init = t30_channel_init;

	return result;
}
