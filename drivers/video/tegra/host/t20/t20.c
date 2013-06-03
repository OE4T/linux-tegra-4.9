/*
 * drivers/video/tegra/host/t20/t20.c
 *
 * Tegra Graphics Init for T20 Architecture Chips
 *
 * Copyright (c) 2011-2013, NVIDIA Corporation.  All rights reserved.
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

#include <linux/export.h>
#include <linux/init.h>
#include <linux/nvhost_ioctl.h>
#include <linux/tegra-powergate.h>

#include <mach/mc.h>

#include "class_ids.h"
#include "t20.h"
#include "gr2d/gr2d_t30.h"
#include "gr3d/gr3d.h"
#include "gr3d/gr3d_t20.h"
#include "mpe/mpe.h"
#include "host1x/host1x.h"
#include "nvhost_channel.h"
#include "nvhost_memmgr.h"
#include "host1x/host1x01_hardware.h"
#include "chip_support.h"
#include "class_ids.h"

/* HACK! This needs to come from DT */
#include "../../../../../arch/arm/mach-tegra/iomap.h"

static int t20_num_alloc_channels = 0;

static struct resource tegra_host1x01_resources[] = {
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
	"mpe",
	"disp0_c", "disp1_c",
	"vblank0", "vblank1",
	"mpe_ebm_eof", "mpe_wr_safe",
	"2d_tinyblt",
	"dsi"
};

static struct host1x_device_info host1x01_info = {
	.nb_channels	= 8,
	.nb_pts		= 32,
	.nb_mlocks	= 16,
	.nb_bases	= 8,
	.syncpt_names	= s_syncpt_names,
	.client_managed	= NVSYNCPTS_CLIENT_MANAGED,
};

struct nvhost_device_data t20_host1x_info = {
	.clocks		= { {"host1x", UINT_MAX} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.private_data	= &host1x01_info,
};

struct platform_device tegra_host1x01_device = {
	.name		= "host1x",
	.id		= -1,
	.resource	= tegra_host1x01_resources,
	.num_resources	= ARRAY_SIZE(tegra_host1x01_resources),
	.dev		= {
		.platform_data = &t20_host1x_info,
	},
};

struct nvhost_device_data t20_gr3d_info = {
	.version	= 1,
	.index		= 1,
	.syncpts	= {NVSYNCPT_3D},
	.waitbases	= {NVWAITBASE_3D},
	.modulemutexes	= {NVMODMUTEX_3D},
	.class		= NV_GRAPHICS_3D_CLASS_ID,
	.clocks		= {{"gr3d", UINT_MAX, 8, TEGRA_MC_CLIENT_NV},
			   {"emc", UINT_MAX, 75}, {} },
	.powergate_ids	= {TEGRA_POWERGATE_3D, -1},
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_NONE,
	.finalize_poweron = NULL,
	.busy		= NULL,
	.idle		= NULL,
	.suspend_ndev	= NULL,
	.init		= NULL,
	.deinit		= NULL,
	.prepare_poweroff = nvhost_gr3d_prepare_power_off,
	.alloc_hwctx_handler = nvhost_gr3d_t20_ctxhandler_init,
	.read_reg	= nvhost_gr3d_t20_read_reg,
};

static struct platform_device tegra_gr3d01_device = {
	.name		= "gr3d",
	.id		= -1,
	.dev		= {
		.platform_data = &t20_gr3d_info,
	},
};

struct nvhost_device_data t20_gr2d_info = {
	.version	= 1,
	.index		= 2,
	.syncpts	= {NVSYNCPT_2D_0, NVSYNCPT_2D_1},
	.waitbases	= {NVWAITBASE_2D_0, NVWAITBASE_2D_1},
	.modulemutexes	= {NVMODMUTEX_2D_FULL, NVMODMUTEX_2D_SIMPLE,
			  NVMODMUTEX_2D_SB_A, NVMODMUTEX_2D_SB_B},
	.clocks		= { {"gr2d", UINT_MAX, 7, TEGRA_MC_CLIENT_G2},
			    {"epp", UINT_MAX, 10, TEGRA_MC_CLIENT_EPP},
			    {"emc", UINT_MAX, 75} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.clockgate_delay = 0,
	.moduleid	= NVHOST_MODULE_NONE,
	.serialize	= true,
	.finalize_poweron = nvhost_gr2d_t30_finalize_poweron,
};

static struct platform_device tegra_gr2d01_device = {
	.name		= "gr2d",
	.id		= -1,
	.dev		= {
		.platform_data = &t20_gr2d_info,
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

struct nvhost_device_data t20_isp_info = {
	.index		= 3,
	.keepalive	= true,
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
		.platform_data = &t20_isp_info,
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

struct nvhost_device_data t20_vi_info = {
	.index		= 4,
	.syncpts	= {NVSYNCPT_CSI_VI_0, NVSYNCPT_CSI_VI_1,
			  NVSYNCPT_VI_ISP_0, NVSYNCPT_VI_ISP_1,
			  NVSYNCPT_VI_ISP_2, NVSYNCPT_VI_ISP_3,
			  NVSYNCPT_VI_ISP_4},
	.modulemutexes	= {NVMODMUTEX_VI},
	.exclusive	= true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_VI,
};
EXPORT_SYMBOL(t20_vi_info);

static struct platform_device tegra_vi01_device = {
	.name		= "vi",
	.resource	= vi_resources,
	.num_resources	= ARRAY_SIZE(vi_resources),
	.id		= -1,
	.dev		= {
		.platform_data = &t20_vi_info,
	},
};

struct nvhost_device_data t20_mpe_info = {
	.version	= 1,
	.index		= 5,
	.syncpts	= {NVSYNCPT_MPE, NVSYNCPT_MPE_EBM_EOF,
			  NVSYNCPT_MPE_WR_SAFE},
	.waitbases	= {NVWAITBASE_MPE},
	.class		= NV_VIDEO_ENCODE_MPEG_CLASS_ID,
	.waitbasesync	= true,
	.keepalive	= true,
	.clocks		= { {"mpe", UINT_MAX, 29, TEGRA_MC_CLIENT_MPE},
			    {"emc", UINT_MAX, 75} },
	.powergate_ids	= {TEGRA_POWERGATE_MPE, -1},
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_MPE,
	.prepare_poweroff = nvhost_mpe_prepare_power_off,
	.alloc_hwctx_handler = nvhost_mpe_ctxhandler_init,
	.read_reg	= nvhost_mpe_read_reg,
};

static struct resource tegra_mpe01_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_MPE_BASE,
		.end = TEGRA_MPE_BASE + TEGRA_MPE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device tegra_mpe01_device = {
	.name		= "mpe",
	.id		= -1,
	.resource	= tegra_mpe01_resources,
	.num_resources	= ARRAY_SIZE(tegra_mpe01_resources),
	.dev		= {
		.platform_data = &t20_mpe_info,
	},
};

static struct platform_device *t20_devices[] = {
	&tegra_gr3d01_device,
	&tegra_gr2d01_device,
	&tegra_isp01_device,
	&tegra_vi01_device,
	&tegra_mpe01_device,
};

struct platform_device *tegra2_register_host1x_devices(void)
{
	int index = 0;
	struct platform_device *pdev;

	/* register host1x device first */
	platform_device_register(&tegra_host1x01_device);
	tegra_host1x01_device.dev.parent = NULL;

	/* register clients with host1x device as parent */
	for (index = 0; index < ARRAY_SIZE(t20_devices); index++) {
		pdev = t20_devices[index];
		pdev->dev.parent = &tegra_host1x01_device.dev;
		platform_device_register(pdev);
	}

	return &tegra_host1x01_device;
}

#include "host1x/host1x_channel.c"
#include "host1x/host1x_cdma.c"
#include "host1x/host1x_debug.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr.c"

static void t20_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_free_channel_internal(ch, &t20_num_alloc_channels);
}

static struct nvhost_channel *t20_alloc_nvhost_channel(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct nvhost_channel *ch = nvhost_alloc_channel_internal(pdata->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t20_num_alloc_channels);
	if (ch)
		ch->ops = host1x_channel_ops;
	return ch;
}

int nvhost_init_t20_support(struct nvhost_master *host,
	struct nvhost_chip_support *op)
{
	int err;

	op->cdma = host1x_cdma_ops;
	op->push_buffer = host1x_pushbuffer_ops;
	op->debug = host1x_debug_ops;
	host->sync_aperture = host->aperture + HOST1X_CHANNEL_SYNC_REG_BASE;
	op->syncpt = host1x_syncpt_ops;
	op->intr = host1x_intr_ops;
	err = nvhost_memmgr_init(op);
	if (err)
		return err;

	op->nvhost_dev.alloc_nvhost_channel = t20_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t20_free_nvhost_channel;

	return 0;
}
