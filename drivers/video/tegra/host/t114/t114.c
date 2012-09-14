/*
 * drivers/video/tegra/host/t114/t114.c
 *
 * Tegra Graphics Init for Tegra11 Architecture Chips
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

#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/nvhost_ioctl.h>
#include <mach/powergate.h>
#include <mach/iomap.h>
#include "t20/t20.h"
#include "t30/t30.h"
#include "t114.h"
#include "gr3d/gr3d_t114.h"
#include "gr3d/scale3d.h"
#include "host1x/host1x02_hardware.h"
#include "msenc/msenc.h"
#include "tsec/tsec.h"
#include "host1x/host1x.h"
#include "chip_support.h"
#include "nvhost_channel.h"
#include "nvhost_memmgr.h"
#include "host1x/host1x_syncpt.h"
#include "chip_support.h"
#include "gr3d/pod_scaling.h"

#define NVMODMUTEX_2D_FULL   (1)
#define NVMODMUTEX_2D_SIMPLE (2)
#define NVMODMUTEX_2D_SB_A   (3)
#define NVMODMUTEX_2D_SB_B   (4)
#define NVMODMUTEX_3D        (5)
#define NVMODMUTEX_DISPLAYA  (6)
#define NVMODMUTEX_DISPLAYB  (7)
#define NVMODMUTEX_VI        (8)
#define NVMODMUTEX_DSI       (9)

static int t114_num_alloc_channels = 0;

static struct resource tegra_host1x02_resources[] = {
	{
		.start = TEGRA_HOST1X_BASE,
		.end = TEGRA_HOST1X_BASE + TEGRA_HOST1X_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_SYNCPT_THRESH_BASE,
		.end = INT_SYNCPT_THRESH_BASE + INT_SYNCPT_THRESH_NR - 1,
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

static struct host1x_device_info host1x02_info = {
	.nb_channels	= 9,
	.nb_pts		= 32,
	.nb_mlocks	= 16,
	.nb_bases	= 8,
	.syncpt_names	= s_syncpt_names,
	.client_managed	= NVSYNCPTS_CLIENT_MANAGED,
};

static struct nvhost_device tegra_host1x02_device = {
	.dev		= {.platform_data = &host1x02_info},
	.name		= "host1x",
	.id		= -1,
	.resource	= tegra_host1x02_resources,
	.num_resources	= ARRAY_SIZE(tegra_host1x02_resources),
	.clocks		= {{"host1x", 102000000}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
};

static struct nvhost_device tegra_display01_device = {
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
};

static struct nvhost_device tegra_gr3d03_device = {
	.name	       = "gr3d",
	.version       = 3,
	.id            = -1,
	.index         = 1,
	.syncpts       = BIT(NVSYNCPT_3D),
	.waitbases     = BIT(NVWAITBASE_3D),
	.modulemutexes = BIT(NVMODMUTEX_3D),
	.class	       = NV_GRAPHICS_3D_CLASS_ID,
	.clocks = {{"gr3d", UINT_MAX},
			{"emc", UINT_MAX} },
	.powergate_ids = { TEGRA_POWERGATE_3D, -1 },
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.can_powergate = true,
	.powergate_delay = 250,
	.moduleid      = NVHOST_MODULE_NONE,
};

static struct nvhost_device tegra_gr2d03_device = {
	.name	       = "gr2d",
	.id            = -1,
	.index         = 2,
	.syncpts       = BIT(NVSYNCPT_2D_0) | BIT(NVSYNCPT_2D_1),
	.waitbases     = BIT(NVWAITBASE_2D_0) | BIT(NVWAITBASE_2D_1),
	.modulemutexes = BIT(NVMODMUTEX_2D_FULL) | BIT(NVMODMUTEX_2D_SIMPLE) |
			 BIT(NVMODMUTEX_2D_SB_A) | BIT(NVMODMUTEX_2D_SB_B),
	.clocks = {{"gr2d", 0},
			{"epp", 0},
			{"emc", 300000000} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.clockgate_delay = 0,
	.moduleid	= NVHOST_MODULE_NONE,
	.serialize	= true,
};

static struct resource isp_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_ISP_BASE,
		.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	}
};

static struct nvhost_device tegra_isp01_device = {
	.name		= "isp",
	.id		= -1,
	.resource	= isp_resources,
	.num_resources	= ARRAY_SIZE(isp_resources),
	.index		= 3,
	.syncpts	= BIT(NVSYNCPT_VI_ISP_2) | BIT(NVSYNCPT_VI_ISP_3) |
			  BIT(NVSYNCPT_VI_ISP_4),
	.clocks		= { {"epp", 0} },
	.keepalive	= true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_ISP,
};

static struct resource vi_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_VI_BASE,
		.end = TEGRA_VI_BASE + TEGRA_VI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct nvhost_device tegra_vi01_device = {
	.name	       = "vi",
	.resource      = vi_resources,
	.num_resources = ARRAY_SIZE(vi_resources),
	.id            = -1,
	.index         = 4,
	.syncpts       = BIT(NVSYNCPT_CSI_VI_0) | BIT(NVSYNCPT_CSI_VI_1) |
			 BIT(NVSYNCPT_VI_ISP_0) | BIT(NVSYNCPT_VI_ISP_1) |
			 BIT(NVSYNCPT_VI_ISP_2) | BIT(NVSYNCPT_VI_ISP_3) |
			 BIT(NVSYNCPT_VI_ISP_4),
	.modulemutexes = BIT(NVMODMUTEX_VI),
	.exclusive     = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VI,
};

static struct resource msenc_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_MSENC_BASE,
		.end = TEGRA_MSENC_BASE + TEGRA_MSENC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct nvhost_device tegra_msenc02_device = {
	.name	       = "msenc",
	.version       = NVHOST_ENCODE_MSENC_VER(2, 0),
	.id            = -1,
	.resource      = msenc_resources,
	.num_resources = ARRAY_SIZE(msenc_resources),
	.index         = 5,
	.syncpts       = BIT(NVSYNCPT_MSENC),
	.waitbases     = BIT(NVWAITBASE_MSENC),
	.class	       = NV_VIDEO_ENCODE_MSENC_CLASS_ID,
	.exclusive     = false,
	.keepalive     = true,
	.clocks = {{"msenc", UINT_MAX}, {"emc", 300000000} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_MSENC,
};

static struct nvhost_device tegra_dsi01_device = {
	.name	       = "dsi",
	.id            = -1,
	.index         = 6,
	.syncpts       = BIT(NVSYNCPT_DSI),
	.modulemutexes = BIT(NVMODMUTEX_DSI),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
};

static struct resource tsec_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_TSEC_BASE,
		.end = TEGRA_TSEC_BASE + TEGRA_TSEC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct nvhost_device tegra_tsec01_device = {
	/* channel 7 */
	.name          = "tsec",
	.version       = NVHOST_ENCODE_TSEC_VER(1,0),
	.id            = -1,
	.resource      = tsec_resources,
	.num_resources = ARRAY_SIZE(tsec_resources),
	.index         = 7,
	.syncpts       = BIT(NVSYNCPT_TSEC),
	.waitbases     = BIT(NVWAITBASE_TSEC),
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = false,
	.clocks = {{"tsec", UINT_MAX}, {"emc", 300000000} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_TSEC,
};

static struct nvhost_device *t11_devices[] = {
	&tegra_host1x02_device,
	&tegra_display01_device,
	&tegra_gr3d03_device,
	&tegra_gr2d03_device,
	&tegra_isp01_device,
	&tegra_vi01_device,
	&tegra_msenc02_device,
	&tegra_dsi01_device,
	&tegra_tsec01_device,
};

int tegra11_register_host1x_devices(void)
{
	return nvhost_add_devices(t11_devices, ARRAY_SIZE(t11_devices));
}

static void t114_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_free_channel_internal(ch, &t114_num_alloc_channels);
}

static struct nvhost_channel *t114_alloc_nvhost_channel(
		struct nvhost_device *dev)
{
	return nvhost_alloc_channel_internal(dev->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t114_num_alloc_channels);
}

#include "host1x/host1x_channel.c"
#include "host1x/host1x_cdma.c"
#include "host1x/host1x_debug.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr.c"
#include "host1x/host1x_actmon.c"
#include "host1x/host1x_tickctrl.c"

int nvhost_init_t114_support(struct nvhost_master *host,
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
	op->nvhost_dev.alloc_nvhost_channel = t114_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t114_free_nvhost_channel;
	op->actmon = host1x_actmon_ops;
	op->tickctrl = host1x_tickctrl_ops;

	return 0;
}
