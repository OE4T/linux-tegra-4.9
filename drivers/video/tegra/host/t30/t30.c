/*
 * drivers/video/tegra/host/t30/t30.c
 *
 * Tegra Graphics Init for T30 Architecture Chips
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#include <linux/mutex.h>
#include <mach/powergate.h>
#include "dev.h"
#include "t30.h"
#include "gr3d/gr3d.h"
#include "mpe/mpe.h"
#include "gr3d/gr3d_t30.h"
#include "gr3d/scale3d.h"
#include "host1x/host1x_hardware.h"
#include "host1x/host1x_cdma.h"
#include "host1x/host1x_syncpt.h"
#include "gr3d/scale3d.h"
#include "../chip_support.h"

static struct nvhost_device devices[] = {
	{.name   = "gr3d", .id = -1 },
	{.name = "gr2d", .id = -1 },
	{.name = "isp", .id = -1 },
	{.name = "vi", .id = -1 },
	{.name = "mpe", .id = -1 },
	{.name = "dsi", .id = -1 },
};

#define NVMODMUTEX_2D_FULL   (1)
#define NVMODMUTEX_2D_SIMPLE (2)
#define NVMODMUTEX_2D_SB_A   (3)
#define NVMODMUTEX_2D_SB_B   (4)
#define NVMODMUTEX_3D        (5)
#define NVMODMUTEX_DISPLAYA  (6)
#define NVMODMUTEX_DISPLAYB  (7)
#define NVMODMUTEX_VI        (8)
#define NVMODMUTEX_DSI       (9)

#ifndef TEGRA_POWERGATE_3D1
#define TEGRA_POWERGATE_3D1 -1
#endif

const struct nvhost_channeldesc nvhost_t30_channelmap[] = {
{
	/* channel 0 */
	.name	       = "display",
	.syncpts       = BIT(NVSYNCPT_DISP0_A) | BIT(NVSYNCPT_DISP1_A) |
			 BIT(NVSYNCPT_DISP0_B) | BIT(NVSYNCPT_DISP1_B) |
			 BIT(NVSYNCPT_DISP0_C) | BIT(NVSYNCPT_DISP1_C) |
			 BIT(NVSYNCPT_VBLANK0) | BIT(NVSYNCPT_VBLANK1),
	.modulemutexes = BIT(NVMODMUTEX_DISPLAYA) | BIT(NVMODMUTEX_DISPLAYB),
	.module        = {
			NVHOST_MODULE_NO_POWERGATE_IDS,
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
			},
},
{
	/* channel 1 */
	.name	       = "gr3d",
	.syncpts       = BIT(NVSYNCPT_3D),
	.waitbases     = BIT(NVWAITBASE_3D),
	.modulemutexes = BIT(NVMODMUTEX_3D),
	.class	       = NV_GRAPHICS_3D_CLASS_ID,
	.module        = {
			.prepare_poweroff = nvhost_gr3d_prepare_power_off,
			.busy = nvhost_scale3d_notify_busy,
			.idle = nvhost_scale3d_notify_idle,
			.init = nvhost_scale3d_init,
			.deinit = nvhost_scale3d_deinit,
			.suspend = nvhost_scale3d_suspend,
			.clocks = {{"gr3d", UINT_MAX},
					{"gr3d2", UINT_MAX},
					{"emc", UINT_MAX} },
			.powergate_ids = {TEGRA_POWERGATE_3D,
					TEGRA_POWERGATE_3D1},
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
			.can_powergate = true,
			.powergate_delay = 100,
	},
},
{
	/* channel 2 */
	.name	       = "gr2d",
	.syncpts       = BIT(NVSYNCPT_2D_0) | BIT(NVSYNCPT_2D_1),
	.waitbases     = BIT(NVWAITBASE_2D_0) | BIT(NVWAITBASE_2D_1),
	.modulemutexes = BIT(NVMODMUTEX_2D_FULL) | BIT(NVMODMUTEX_2D_SIMPLE) |
			 BIT(NVMODMUTEX_2D_SB_A) | BIT(NVMODMUTEX_2D_SB_B),
	.module        = {
			.clocks = {{"gr2d", 0},
					{"epp", 0},
					{"emc", 300000000} },
			NVHOST_MODULE_NO_POWERGATE_IDS,
			.clockgate_delay = 0,
			},
},
{
	/* channel 3 */
	.name	 = "isp",
	.syncpts = 0,
	.module         = {
			NVHOST_MODULE_NO_POWERGATE_IDS,
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
			},
},
{
	/* channel 4 */
	.name	       = "vi",
	.syncpts       = BIT(NVSYNCPT_CSI_VI_0) | BIT(NVSYNCPT_CSI_VI_1) |
			 BIT(NVSYNCPT_VI_ISP_0) | BIT(NVSYNCPT_VI_ISP_1) |
			 BIT(NVSYNCPT_VI_ISP_2) | BIT(NVSYNCPT_VI_ISP_3) |
			 BIT(NVSYNCPT_VI_ISP_4),
	.modulemutexes = BIT(NVMODMUTEX_VI),
	.exclusive     = true,
	.module        = {
			NVHOST_MODULE_NO_POWERGATE_IDS,
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
			},
},
{
	/* channel 5 */
	.name	       = "mpe",
	.syncpts       = BIT(NVSYNCPT_MPE) | BIT(NVSYNCPT_MPE_EBM_EOF) |
			 BIT(NVSYNCPT_MPE_WR_SAFE),
	.waitbases     = BIT(NVWAITBASE_MPE),
	.class	       = NV_VIDEO_ENCODE_MPEG_CLASS_ID,
	.waitbasesync  = true,
	.keepalive     = true,
	.module        = {
			.prepare_poweroff = nvhost_mpe_prepare_power_off,
			.clocks = {{"mpe", UINT_MAX}, {"emc", UINT_MAX}, {} },
			.powergate_ids  = {TEGRA_POWERGATE_MPE, -1},
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
			.can_powergate  = true,
			.powergate_delay = 100,
			},
},
{
	/* channel 6 */
	.name	       = "dsi",
	.syncpts       = BIT(NVSYNCPT_DSI),
	.modulemutexes = BIT(NVMODMUTEX_DSI),
	.module        = {
			NVHOST_MODULE_NO_POWERGATE_IDS,
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
	},
} };

#define NVHOST_CHANNEL_BASE 0

static inline int t30_nvhost_hwctx_handler_init(
	struct nvhost_hwctx_handler *h,
	const char *module)
{
	if (strcmp(module, "gr3d") == 0)
		return nvhost_gr3d_t30_ctxhandler_init(h);
	else if (strcmp(module, "mpe") == 0)
		return nvhost_mpe_ctxhandler_init(h);

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
	ch->desc = nvhost_t30_channelmap + index;
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
int nvhost_init_t30_debug_support(struct nvhost_master *host)
{
	nvhost_init_t20_debug_support(host);
	host->op.debug.debug_init = nvhost_scale3d_debug_init;

	return 0;
}

int nvhost_init_t30_support(struct nvhost_master *host)
{
	int err;
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); i++)
		nvhost_device_register(&devices[i]);

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t30_channel_support(host);
	if (err)
		return err;
	err = host1x_init_cdma_support(host);
	if (err)
		return err;
	err = nvhost_init_t30_debug_support(host);
	if (err)
		return err;
	err = host1x_init_syncpt_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_intr_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_cpuaccess_support(host);
	if (err)
		return err;
	return 0;
}
