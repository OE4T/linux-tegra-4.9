/*
 * drivers/video/tegra/host/t148/t148.c
 *
 * Tegra Graphics Init for T148 Architecture Chips
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation.
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
#include <mach/iomap.h>
#include "dev.h"
#include "host1x/host1x_cdma.h"
#include "t20/t20.h"
#include "t30/t30.h"
#include "t148/t148.h"
#include "t114/t114.h"
#include "host1x/host1x03_hardware.h"
#include "host1x/host1x_syncpt.h"
#include "gr3d/gr3d.h"
#include "gr3d/gr3d_t114.h"
#include "gr3d/scale3d.h"
#include "msenc/msenc.h"
#include "tsec/tsec.h"
#include "linux/nvhost_ioctl.h"
#include "nvhost_channel.h"
#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "class_ids.h"

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

static int t148_num_alloc_channels = 0;

static struct resource tegra_host1x03_resources[] = {
	{
		.start = TEGRA_HOST1X_BASE,
		.end = TEGRA_HOST1X_BASE + TEGRA_HOST1X_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_HOST1X_MPCORE_SYNCPT,
		.end = INT_HOST1X_MPCORE_SYNCPT,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_HOST1X_MPCORE_GENERAL,
		.end = INT_HOST1X_MPCORE_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
};

static const char *s_syncpt_names[32] = {
	"gfx_host",
	"", "", "", "", "", "", "",
	"disp0_a", "disp1_a", "avp_0",
	"csi_vi_0", "csi_vi_1",
	"vi_isp_0", "vi_isp_1", "vi_isp_2", "vi_isp_3", "vi_isp_4",
	"2d_0", "2d_1",
	"disp0_b", "disp1_b",
	"3d",
	"msenc",
	"disp0_c", "disp1_c",
	"vblank0", "vblank1",
	"tsec", "msenc_unused",
	"2d_tinyblt",
	"dsi"
};

static struct host1x_device_info host1x03_info = {
	.nb_channels	= 12,
	.nb_pts		= 48,
	.nb_mlocks	= 16,
	.nb_bases	= 12,
	.syncpt_names	= s_syncpt_names,
	.client_managed	= NVSYNCPTS_CLIENT_MANAGED,
};

static struct nvhost_device_data tegra_host1x03_info = {
	.clocks		= { {"host1x", 81600000} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
};

static struct platform_device tegra_host1x03_device = {
	.name		= "host1x",
	.id		= -1,
	.resource	= tegra_host1x03_resources,
	.num_resources	= ARRAY_SIZE(tegra_host1x03_resources),
	.dev		= {
		.platform_data = &tegra_host1x03_info,
	},
};

static struct nvhost_device_data tegra_gr3d03_info = {
	.version	= 3,
	.index		= 1,
	.syncpts	= {NVSYNCPT_3D},
	.waitbases	= {NVWAITBASE_3D},
	.modulemutexes	= {NVMODMUTEX_3D},
	.class		= NV_GRAPHICS_3D_CLASS_ID,
	.clocks		= { {"gr3d", UINT_MAX, 8, true},
			    {"emc", UINT_MAX, 75} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.powerup_reset	= true,
	.moduleid	= NVHOST_MODULE_NONE,
};

static struct platform_device tegra_gr3d03_device = {
	.name		= "gr3d03",
	.id		= -1,
	.dev		= {
		.platform_data = &tegra_gr3d03_info,
	},
};

static struct nvhost_device_data tegra_gr2d03_info = {
	.index		= 2,
	.syncpts	= {NVSYNCPT_2D_0, NVSYNCPT_2D_1},
	.waitbases	= {NVWAITBASE_2D_0, NVWAITBASE_2D_1},
	.modulemutexes	= {NVMODMUTEX_2D_FULL, NVMODMUTEX_2D_SIMPLE,
			  NVMODMUTEX_2D_SB_A, NVMODMUTEX_2D_SB_B},
	.clocks		= { {"gr2d", 0, 7, true}, {"epp", 0, 10, true},
			    {"emc", 300000000, 75 } },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_NONE,
	.serialize	= true,
};

static struct platform_device tegra_gr2d03_device = {
	.name		= "gr2d02",
	.id		= -1,
	.dev		= {
		.platform_data = &tegra_gr2d03_info,
	},
};

static struct resource isp_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_ISP_BASE,
		.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	}
};

static struct nvhost_device_data tegra_isp01_info = {
	.index		= 3,
	.syncpts	= {NVSYNCPT_VI_ISP_2, NVSYNCPT_VI_ISP_3,
			  NVSYNCPT_VI_ISP_4},
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_ISP,
};

static struct platform_device tegra_isp01_device = {
	.name		= "isp",
	.id		= -1,
	.resource	= isp_resources,
	.num_resources	= ARRAY_SIZE(isp_resources),
	.dev		= {
		.platform_data = &tegra_isp01_info,
	},
};

static struct resource vi_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_VI_BASE,
		.end = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct nvhost_device_data tegra_vi01_info = {
	.index		= 4,
	.syncpts	= {NVSYNCPT_CSI_VI_0, NVSYNCPT_CSI_VI_1,
			  NVSYNCPT_VI_ISP_0, NVSYNCPT_VI_ISP_1,
			  NVSYNCPT_VI_ISP_2, NVSYNCPT_VI_ISP_3,
			  NVSYNCPT_VI_ISP_4},
	.modulemutexes	= {NVMODMUTEX_VI_0},
	.exclusive	= true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_VI,
};

static struct platform_device tegra_vi01_device = {
	.name		= "vi",
	.id		= -1,
	.resource	= vi_resources,
	.num_resources	= ARRAY_SIZE(vi_resources),
	.dev		= {
		.platform_data = &tegra_vi01_info,
	},
};

static struct resource msenc_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_MSENC_BASE,
		.end = TEGRA_MSENC_BASE + TEGRA_MSENC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct nvhost_device_data tegra_msenc03_info = {
	.version	= NVHOST_ENCODE_MSENC_VER(3, 0),
	.index		= 5,
	.syncpts	= {NVSYNCPT_MSENC},
	.waitbases	= {NVWAITBASE_MSENC},
	.class		= NV_VIDEO_ENCODE_MSENC_CLASS_ID,
	.clocks		= { {"msenc", UINT_MAX, 107, true},
			    {"emc", 300000000, 75} },
	.powergate_ids = { TEGRA_POWERGATE_MPE, -1 },
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.powergate_delay = 100,
	.can_powergate = false,
	.moduleid	= NVHOST_MODULE_MSENC,
};

static struct platform_device tegra_msenc03_device = {
	.name		= "msenc",
	.id		= -1,
	.resource	= msenc_resources,
	.num_resources	= ARRAY_SIZE(msenc_resources),
	.dev		= {
		.platform_data = &tegra_msenc03_info,
	},
};

static struct resource tsec_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_TSEC_BASE,
		.end = TEGRA_TSEC_BASE + TEGRA_TSEC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct nvhost_device_data tegra_tsec01_info = {
	.version	= NVHOST_ENCODE_TSEC_VER(1,0),
	.index		= 7,
	.syncpts	= {NVSYNCPT_TSEC},
	.waitbases	= {NVWAITBASE_TSEC},
	.class		= NV_TSEC_CLASS_ID,
	.exclusive	= false,
	.clocks		= { {"tsec", UINT_MAX, 108, true},
			    {"emc", 300000000, 75} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_TSEC,
};

static struct platform_device tegra_tsec01_device = {
	.name		= "tsec",
	.id		= -1,
	.resource	= tsec_resources,
	.num_resources	= ARRAY_SIZE(tsec_resources),
	.dev		= {
		.platform_data = &tegra_tsec01_info,
	},
};

static struct platform_device *t14_devices[] = {
	&tegra_gr3d03_device,
	&tegra_gr2d03_device,
	&tegra_isp01_device,
	&tegra_vi01_device,
	&tegra_msenc03_device,
	&tegra_tsec01_device,
};

struct platform_device *tegra14_register_host1x_devices(void)
{
	int index = 0;
	struct platform_device *pdev;

	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)tegra_host1x03_device.dev.platform_data;
	pdata->private_data = &host1x03_info;

	/* register host1x device first */
	platform_device_register(&tegra_host1x03_device);
	tegra_host1x03_device.dev.parent = NULL;

	/* register clients with host1x device as parent */
	for (index = 0; index < ARRAY_SIZE(t14_devices); index++) {
		pdev = t14_devices[index];
		pdev->dev.parent = &tegra_host1x03_device.dev;
		if (pdev == &tegra_gr3d03_device) {
			const char *real_name = pdev->name;
			pdev->name = "gr3d";
			platform_device_register(pdev);
			pdev->name = real_name;
		} else
			platform_device_register(pdev);
	}

	return &tegra_host1x03_device;
}

static void t148_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_free_channel_internal(ch, &t148_num_alloc_channels);
}

static struct nvhost_channel *t148_alloc_nvhost_channel(
		struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	return nvhost_alloc_channel_internal(pdata->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t148_num_alloc_channels);
}

#include "host1x/host1x_channel.c"
#include "host1x/host1x_cdma.c"
#include "host1x/host1x_debug.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr.c"
#include "host1x/host1x_actmon.c"
#include "host1x/host1x_tickctrl.c"

int nvhost_init_t148_support(struct nvhost_master *host,
	struct nvhost_chip_support *op)
{
	int err;

	op->channel = host1x_channel_ops;
	op->cdma = host1x_cdma_ops;
	op->push_buffer = host1x_pushbuffer_ops;
	op->debug = host1x_debug_ops;
	host->sync_aperture = host->aperture + HOST1X_CHANNEL_SYNC_REG_BASE;
	op->syncpt = host1x_syncpt_ops;
	op->intr = host1x_intr_ops;
	err = nvhost_memmgr_init(op);
	if (err)
		return err;
	op->nvhost_dev.alloc_nvhost_channel = t148_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t148_free_nvhost_channel;
	op->actmon = host1x_actmon_ops;
	op->tickctrl = host1x_tickctrl_ops;

	return 0;
}
