/*
 * drivers/video/tegra/host/t148/t148.c
 *
 * Tegra Graphics Init for T148 Architecture Chips
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include <linux/mutex.h>
#include <mach/powergate.h>
#include "dev.h"
#include "host1x/host1x_cdma.h"
#include "t20/t20.h"
#include "t30/t30.h"
#include "t114/t114.h"
#include "host1x/host1x_hardware.h"
#include "host1x/host1x_syncpt.h"
#include "gr3d/gr3d.h"
#include "gr3d/gr3d_t114.h"
#include "gr3d/scale3d.h"
#include "msenc/msenc.h"
#include "tsec/tsec.h"

#define NVMODMUTEX_2D_FULL   (1)
#define NVMODMUTEX_2D_SIMPLE (2)
#define NVMODMUTEX_2D_SB_A   (3)
#define NVMODMUTEX_2D_SB_B   (4)
#define NVMODMUTEX_3D        (5)
#define NVMODMUTEX_DISPLAYA  (6)
#define NVMODMUTEX_DISPLAYB  (7)
#define NVMODMUTEX_VI_0      (8)
#define NVMODMUTEX_DSI       (9)
#define NVMODMUTEX_VI_1      (10)



#define HOST_EMC_FLOOR 300000000

static struct nvhost_device t148_devices[] = {
{
	/* channel 0 */
	.name	       = "display",
	.id            = -1,
	.index         = 0,
	.syncpts       = BIT(NVSYNCPT_DISP0_A) | BIT(NVSYNCPT_DISP1_A) |
			 BIT(NVSYNCPT_DISP0_B) | BIT(NVSYNCPT_DISP1_B) |
			 BIT(NVSYNCPT_DISP0_C) | BIT(NVSYNCPT_DISP1_C) |
			 BIT(NVSYNCPT_VBLANK0) | BIT(NVSYNCPT_VBLANK1),
	.modulemutexes = BIT(NVMODMUTEX_DISPLAYA) | BIT(NVMODMUTEX_DISPLAYB),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
},
{
	/* channel 1 */
	.name	       = "gr3d",
	.id            = -1,
	.index         = 1,
	.syncpts       = BIT(NVSYNCPT_3D),
	.waitbases     = BIT(NVWAITBASE_3D),
	.modulemutexes = BIT(NVMODMUTEX_3D),
	.class	       = NV_GRAPHICS_3D_CLASS_ID,
	.prepare_poweroff = nvhost_gr3d_prepare_power_off,
	.busy = nvhost_scale3d_notify_busy,
	.idle = nvhost_scale3d_notify_idle,
	.init = nvhost_scale3d_init,
	.deinit = nvhost_scale3d_deinit,
	.alloc_hwctx_handler = t114_nvhost_3dctx_handler_init,
	.clocks = {{"gr3d", UINT_MAX},
			{"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
},
{
	/* channel 2 */
	.name	       = "gr2d",
	.id            = -1,
	.index         = 2,
	.syncpts       = BIT(NVSYNCPT_2D_0) | BIT(NVSYNCPT_2D_1),
	.waitbases     = BIT(NVWAITBASE_2D_0) | BIT(NVWAITBASE_2D_1),
	.modulemutexes = BIT(NVMODMUTEX_2D_FULL) | BIT(NVMODMUTEX_2D_SIMPLE) |
			 BIT(NVMODMUTEX_2D_SB_A) | BIT(NVMODMUTEX_2D_SB_B),
	.clocks = {{"gr2d", 0},
			{"epp", UINT_MAX},
			{"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
},
{
	/* channel 3 */
	.name	 = "isp",
	.id      = -1,
	.index   = 3,
	.syncpts = 0,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_ISP,
},
{
	/* channel 4 */
	.name	       = "vi",
	.id            = -1,
	.index         = 4,
	.syncpts       = BIT(NVSYNCPT_CSI_VI_0) | BIT(NVSYNCPT_CSI_VI_1) |
			 BIT(NVSYNCPT_VI_ISP_0) | BIT(NVSYNCPT_VI_ISP_1) |
			 BIT(NVSYNCPT_VI_ISP_2) | BIT(NVSYNCPT_VI_ISP_3) |
			 BIT(NVSYNCPT_VI_ISP_4),
	.modulemutexes = BIT(NVMODMUTEX_VI_0),
	.exclusive     = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VI,
},
{
	/* channel 5 */
	.name	       = "msenc",
	.id            = -1,
	.index         = 5,
	.syncpts       = BIT(NVSYNCPT_MSENC),
	.waitbases     = BIT(NVWAITBASE_MSENC),
	.class	       = NV_VIDEO_ENCODE_MSENC_CLASS_ID,
	.exclusive     = false,
	.keepalive     = true,
	.init          = nvhost_msenc_init,
	.deinit        = nvhost_msenc_deinit,
	.clocks = {{"msenc", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_MSENC,
},
{
	/* channel 6 */
	.name	       = "dsi",
	.id            = -1,
	.index         = 6,
	.syncpts       = BIT(NVSYNCPT_DSI),
	.modulemutexes = BIT(NVMODMUTEX_DSI),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
},
{
	/* channel 7 */
	.name          = "tsec",
	.id            = -1,
	.index         = 7,
	.syncpts       = BIT(NVSYNCPT_TSEC),
	.waitbases     = BIT(NVWAITBASE_TSEC),
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = false,
	.init          = nvhost_tsec_init,
	.deinit        = nvhost_tsec_deinit,
	.clocks = {{"tsec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_TSEC,
},
};


static struct nvhost_device *t148_get_nvhost_device(struct nvhost_master *host,
		char *name)
{
	int i;

	for (i = 0; i < host->nb_channels; i++) {
		if (strcmp(t148_devices[i].name, name) == 0)
			return &t148_devices[i];
	}

	return NULL;
}

int nvhost_init_t148_support(struct nvhost_master *host)
{
	int err;

	err = nvhost_init_t114_support(host);

	if (err)
		return err;
	host->op.nvhost_dev.get_nvhost_device = t148_get_nvhost_device;
	return 0;
}
