/*
 * drivers/video/tegra/host/t20/t20.c
 *
 * Tegra Graphics Init for T20 Architecture Chips
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

#include <linux/slab.h>
#include <mach/powergate.h>
#include "dev.h"
#include "t20.h"
#include "host1x/host1x_channel.h"
#include "host1x/host1x_syncpt.h"
#include "host1x/host1x_hardware.h"
#include "host1x/host1x_cdma.h"
#include "gr3d/gr3d.h"
#include "gr3d/gr3d_t20.h"
#include "mpe/mpe.h"

#define NVMODMUTEX_2D_FULL   (1)
#define NVMODMUTEX_2D_SIMPLE (2)
#define NVMODMUTEX_2D_SB_A   (3)
#define NVMODMUTEX_2D_SB_B   (4)
#define NVMODMUTEX_3D        (5)
#define NVMODMUTEX_DISPLAYA  (6)
#define NVMODMUTEX_DISPLAYB  (7)
#define NVMODMUTEX_VI        (8)
#define NVMODMUTEX_DSI       (9)

#define NVHOST_NUMCHANNELS (NV_HOST1X_CHANNELS - 1)

static struct nvhost_device devices[] = {
	{.name   = "gr3d", .id = -1 },
	{.name = "gr2d", .id = -1 },
	{.name = "isp", .id = -1 },
	{.name = "vi", .id = -1 },
	{.name = "mpe", .id = -1 },
	{.name = "dsi", .id = -1 }
};

const struct nvhost_channeldesc nvhost_t20_channelmap[] = {
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
			.clocks = {{"gr3d", UINT_MAX}, {"emc", UINT_MAX}, {} },
			.powergate_ids = {TEGRA_POWERGATE_3D, -1},
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
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
			.clocks = {{"gr2d", UINT_MAX} ,
					{"epp", UINT_MAX} ,
					{"emc", UINT_MAX} },
			NVHOST_MODULE_NO_POWERGATE_IDS,
			.clockgate_delay = 0,
			}
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
			}
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
			.powergate_ids = {TEGRA_POWERGATE_MPE, -1},
			NVHOST_DEFAULT_CLOCKGATE_DELAY,
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

static inline void __iomem *t20_channel_aperture(void __iomem *p, int ndx)
{
	p += NV_HOST1X_CHANNEL0_BASE;
	p += ndx * NV_HOST1X_CHANNEL_MAP_SIZE_BYTES;
	return p;
}

static inline int t20_nvhost_hwctx_handler_init(
	struct nvhost_hwctx_handler *h,
	const char *module)
{
	if (strcmp(module, "gr3d") == 0)
		return nvhost_gr3d_t20_ctxhandler_init(h);
	else if (strcmp(module, "mpe") == 0)
		return nvhost_mpe_ctxhandler_init(h);
	return 0;
}

static int t20_channel_init(struct nvhost_channel *ch,
			    struct nvhost_master *dev, int index)
{
	ch->dev = dev;
	ch->chid = index;
	ch->desc = nvhost_t20_channelmap + index;
	mutex_init(&ch->reflock);
	mutex_init(&ch->submitlock);

	ch->aperture = t20_channel_aperture(dev->aperture, index);

	return t20_nvhost_hwctx_handler_init(&ch->ctxhandler, ch->desc->name);
}

int nvhost_init_t20_channel_support(struct nvhost_master *host)
{
	host->nb_mlocks =  NV_HOST1X_SYNC_MLOCK_NUM;
	host->nb_channels =  NVHOST_NUMCHANNELS;

	host->op.channel.init = t20_channel_init;
	host->op.channel.submit = host1x_channel_submit;
	host->op.channel.read3dreg = host1x_channel_read_3d_reg;

	return 0;
}

int nvhost_init_t20_support(struct nvhost_master *host)
{
	int err;
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); i++)
		nvhost_device_register(&devices[i]);

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t20_channel_support(host);
	if (err)
		return err;
	err = host1x_init_cdma_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_debug_support(host);
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
