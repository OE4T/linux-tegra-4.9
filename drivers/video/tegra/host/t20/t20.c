/*
 * drivers/video/tegra/host/t20/t20.c
 *
 * Tegra Graphics Init for T20 Architecture Chips
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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
#include "nvhost_hwctx.h"

#define NVMODMUTEX_2D_FULL	(1)
#define NVMODMUTEX_2D_SIMPLE	(2)
#define NVMODMUTEX_2D_SB_A	(3)
#define NVMODMUTEX_2D_SB_B	(4)
#define NVMODMUTEX_3D		(5)
#define NVMODMUTEX_DISPLAYA	(6)
#define NVMODMUTEX_DISPLAYB	(7)
#define NVMODMUTEX_VI		(8)
#define NVMODMUTEX_DSI		(9)

#define NVHOST_NUMCHANNELS	(NV_HOST1X_CHANNELS - 1)

struct nvhost_device t20_devices[] = {
{
	/* channel 0 */
	.name		= "display",
	.id		= -1,
	.index		= 0,
	.syncpts 	= BIT(NVSYNCPT_DISP0_A) | BIT(NVSYNCPT_DISP1_A) |
			  BIT(NVSYNCPT_DISP0_B) | BIT(NVSYNCPT_DISP1_B) |
			  BIT(NVSYNCPT_DISP0_C) | BIT(NVSYNCPT_DISP1_C) |
			  BIT(NVSYNCPT_VBLANK0) | BIT(NVSYNCPT_VBLANK1),
	.modulemutexes	= BIT(NVMODMUTEX_DISPLAYA) | BIT(NVMODMUTEX_DISPLAYB),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_NONE,
},
{
	/* channel 1 */
	.name		= "gr3d",
	.id		= -1,
	.index		= 1,
	.syncpts	= BIT(NVSYNCPT_3D),
	.waitbases	= BIT(NVWAITBASE_3D),
	.modulemutexes	= BIT(NVMODMUTEX_3D),
	.class		= NV_GRAPHICS_3D_CLASS_ID,
	.prepare_poweroff = nvhost_gr3d_prepare_power_off,
	.alloc_hwctx_handler = nvhost_gr3d_t20_ctxhandler_init,
	.clocks 	= {{"gr3d", UINT_MAX}, {"emc", UINT_MAX}, {} },
	.powergate_ids	= {TEGRA_POWERGATE_3D, -1},
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_NONE,
},
{
	/* channel 2 */
	.name		= "gr2d",
	.id		= -1,
	.index		= 2,
	.syncpts	= BIT(NVSYNCPT_2D_0) | BIT(NVSYNCPT_2D_1),
	.waitbases	= BIT(NVWAITBASE_2D_0) | BIT(NVWAITBASE_2D_1),
	.modulemutexes	= BIT(NVMODMUTEX_2D_FULL) | BIT(NVMODMUTEX_2D_SIMPLE) |
			  BIT(NVMODMUTEX_2D_SB_A) | BIT(NVMODMUTEX_2D_SB_B),
	.clocks 	= { {"gr2d", UINT_MAX},
			    {"epp", UINT_MAX},
			    {"emc", UINT_MAX} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.clockgate_delay = 0,
	.moduleid	= NVHOST_MODULE_NONE,
},
{
	/* channel 3 */
	.name		= "isp",
	.id		= -1,
	.index		= 3,
	.syncpts	= 0,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_ISP,
},
{
	/* channel 4 */
	.name		= "vi",
	.id		= -1,
	.index		= 4,
	.syncpts	= BIT(NVSYNCPT_CSI_VI_0) | BIT(NVSYNCPT_CSI_VI_1) |
			  BIT(NVSYNCPT_VI_ISP_0) | BIT(NVSYNCPT_VI_ISP_1) |
			  BIT(NVSYNCPT_VI_ISP_2) | BIT(NVSYNCPT_VI_ISP_3) |
			  BIT(NVSYNCPT_VI_ISP_4),
	.modulemutexes	= BIT(NVMODMUTEX_VI),
	.exclusive	= true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_VI,
},
{
	/* channel 5 */
	.name		= "mpe",
	.id		= -1,
	.index		= 5,
	.syncpts	= BIT(NVSYNCPT_MPE) | BIT(NVSYNCPT_MPE_EBM_EOF) |
			  BIT(NVSYNCPT_MPE_WR_SAFE),
	.waitbases	= BIT(NVWAITBASE_MPE),
	.class		= NV_VIDEO_ENCODE_MPEG_CLASS_ID,
	.waitbasesync	= true,
	.keepalive	= true,
	.prepare_poweroff = nvhost_mpe_prepare_power_off,
	.alloc_hwctx_handler = nvhost_mpe_ctxhandler_init,
	.clocks		= { {"mpe", UINT_MAX},
			    {"emc", UINT_MAX} },
	.powergate_ids	= {TEGRA_POWERGATE_MPE, -1},
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_MPE,
},
{
	/* channel 6 */
	.name		= "dsi",
	.id		= -1,
	.index		= 6,
	.syncpts	= BIT(NVSYNCPT_DSI),
	.modulemutexes	= BIT(NVMODMUTEX_DSI),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_NONE,
} };


static inline void __iomem *t20_channel_aperture(void __iomem *p, int ndx)
{
	p += NV_HOST1X_CHANNEL0_BASE;
	p += ndx * NV_HOST1X_CHANNEL_MAP_SIZE_BYTES;
	return p;
}

static inline int t20_nvhost_hwctx_handler_init(struct nvhost_channel *ch)
{
	int err = 0;
	unsigned long syncpts = ch->dev->syncpts;
	unsigned long waitbases = ch->dev->waitbases;
	u32 syncpt = find_first_bit(&syncpts, BITS_PER_LONG);
	u32 waitbase = find_first_bit(&waitbases, BITS_PER_LONG);

	if (ch->dev->alloc_hwctx_handler) {
		ch->ctxhandler = ch->dev->alloc_hwctx_handler(syncpt,
				waitbase, ch);
		if (!ch->ctxhandler)
			err = -ENOMEM;
	}

	return err;
}

static int t20_channel_init(struct nvhost_channel *ch,
	struct nvhost_master *dev, int index)
{
	ch->chid = index;
	mutex_init(&ch->reflock);
	mutex_init(&ch->submitlock);

	ch->aperture = t20_channel_aperture(dev->aperture, index);

	return t20_nvhost_hwctx_handler_init(ch);
}

int nvhost_init_t20_channel_support(struct nvhost_master *host)
{
	host->nb_channels =  NVHOST_NUMCHANNELS;

	host->op.channel.init = t20_channel_init;
	host->op.channel.submit = host1x_channel_submit;
	host->op.channel.read3dreg = host1x_channel_read_3d_reg;

	return 0;
}

struct nvhost_device *t20_get_nvhost_device(struct nvhost_master *host,
	char *name)
{
	int i;

	for (i = 0; i < host->nb_channels; i++) {
		if (strcmp(t20_devices[i].name, name) == 0)
			return &t20_devices[i];
	}

	return NULL;
}

int nvhost_init_t20_support(struct nvhost_master *host)
{
	int err;

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
	host->op.nvhost_dev.get_nvhost_device = t20_get_nvhost_device;
	return 0;
}
