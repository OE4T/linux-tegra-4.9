/*
 * drivers/video/tegra/host/t124/t124.c
 *
 * Tegra Graphics Init for T124 Architecture Chips
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.>
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
#include <linux/tegra-powergate.h>

#include <mach/mc.h>

#include "dev.h"
#include "nvhost_job.h"
#include "class_ids.h"

#include "t124.h"
#include "host1x/host1x.h"

#include "hardware_t124.h"
#include "syncpt_t124.h"

#include "gk20a/gk20a.h"
#include "gk20a/gk20a_scale.h"
#include "vic03/vic03.h"
#include "msenc/msenc.h"
#include "tsec/tsec.h"
#include "vi/vi.h"
#include "isp/isp.h"
#include "gr3d/pod_scaling.h"
#include "gr3d/scale3d.h"

#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "nvhost_scale.h"

static int t124_num_alloc_channels = 0;

#define HOST_EMC_FLOOR 300000000
#define VI_CLOCKGATE_DELAY 60
#define VI_POWERGATE_DELAY 500
#define ISP_CLOCKGATE_DELAY 60
#define ISP_POWERGATE_DELAY 500
#define TSEC_POWERGATE_DELAY 500
#define HOST1X_POWERGATE_DELAY 50

#define GK20A_DEV_NAME_SIZE 5

#define BIT64(nr) (1ULL << (nr))
#define NVSYNCPTS_CLIENT_MANAGED_T124 ( \
	BIT64(NVSYNCPT_DISP0_A) | BIT64(NVSYNCPT_DISP1_A) | \
	BIT64(NVSYNCPT_DISP0_B) | BIT64(NVSYNCPT_DISP1_B) | \
	BIT64(NVSYNCPT_DISP0_C) | BIT64(NVSYNCPT_DISP1_C) | \
	BIT(NVSYNCPT_DISP0_D) | \
	BIT(NVSYNCPT_DISP0_H) | BIT(NVSYNCPT_DISP1_H) | \
	BIT64(NVSYNCPT_DSI) | \
	BIT64(NVSYNCPT_VBLANK0) | BIT64(NVSYNCPT_VBLANK1) | \
	BIT64(NVSYNCPT_AVP_0))

/* Host1x driver matches module IDs while setting a
 * particular clock, This ID is used for EMC module.
 */
#define TEGRA_HOST1X_EMC_MODULE_ID 75

static struct resource tegra_host1x04_resources[] = {
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

static const char *s_syncpt_names[NV_HOST1X_SYNCPT_NB_PTS] = {
	[NVSYNCPT_ISP_0_0]	= "ispa_memory",
	[NVSYNCPT_ISP_0_1]	= "ispa_stats",
	[NVSYNCPT_ISP_0_2]	= "ispa_stream",
	[NVSYNCPT_ISP_0_3]	= "ispa_loadv",
	[NVSYNCPT_ISP_1_0]	= "ispb_memory",
	[NVSYNCPT_ISP_1_1]	= "ispb_stats",
	[NVSYNCPT_ISP_1_2]	= "ispb_stream",
	[NVSYNCPT_ISP_1_3]	= "ispb_loadv",
	[NVSYNCPT_VI_0_0]	= "vi0_ispa",
	[NVSYNCPT_VI_0_1]	= "vi0_ispb",
	[NVSYNCPT_VI_0_2]	= "vi0_stream",
	[NVSYNCPT_VI_0_3]	= "vi0_memory",
	[NVSYNCPT_VI_0_4]	= "vi0_flash",
	[NVSYNCPT_VI_1_0]	= "vi1_ispa",
	[NVSYNCPT_VI_1_1]	= "vi1_ispb",
	[NVSYNCPT_VI_1_2]	= "vi1_stream",
	[NVSYNCPT_VI_1_3]	= "vi1_memory",
	[NVSYNCPT_VI_1_4]	= "vi1_flash",
	[NVSYNCPT_3D]		= "3d",
	[NVSYNCPT_MPE]		= "mpe",
	[NVSYNCPT_MPE_EBM_EOF]	= "mpe_ebm_eof",
	[NVSYNCPT_MPE_WR_SAFE]	= "mpe_wr_safe",
	[NVSYNCPT_VIC]		= "vic",
	[NVSYNCPT_TSEC]		= "tsec",
	[NVSYNCPT_DISP0_A]	= "disp0",
	[NVSYNCPT_DISP1_A]	= "disp1",
	[NVSYNCPT_AVP_0]	= "avp",
	[NVSYNCPT_DISP0_B]	= "disp0b",
	[NVSYNCPT_DISP1_B]	= "disp1b",
	[NVSYNCPT_DISP0_C]	= "disp0c",
	[NVSYNCPT_DISP1_C]	= "disp1c",
	[NVSYNCPT_DISP0_D]	= "disp0d",
	[NVSYNCPT_DISP0_H]	= "disp0h",
	[NVSYNCPT_DISP1_H]	= "disp1h",
	[NVSYNCPT_VBLANK0]	= "vblank0",
	[NVSYNCPT_VBLANK1]	= "vblank1",
	[NVSYNCPT_DSI]		= "dsi",
};

static struct host1x_device_info host1x04_info = {
	.nb_channels	= T124_NVHOST_NUMCHANNELS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.nb_bases	= NV_HOST1X_SYNCPT_NB_BASES,
	.syncpt_names	= s_syncpt_names,
	.client_managed	= NVSYNCPTS_CLIENT_MANAGED_T124,
};

struct nvhost_device_data t124_host1x_info = {
	.clocks		= {{"host1x", 81600000}, {"actmon", UINT_MAX} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.can_powergate   = true,
	.powergate_delay = HOST1X_POWERGATE_DELAY,
	.private_data	= &host1x04_info,
	.finalize_poweron = nvhost_host1x_finalize_poweron,
	.prepare_poweroff = nvhost_host1x_prepare_poweroff,
};


static struct platform_device tegra_host1x04_device = {
	.name		= "host1x",
	.id		= -1,
	.resource	= tegra_host1x04_resources,
	.num_resources	= ARRAY_SIZE(tegra_host1x04_resources),
	.dev            = {
		.platform_data = &t124_host1x_info,
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

static struct platform_device tegra_isp01b_device;
struct nvhost_device_data t124_isp_info = {
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts         = NV_ISP_0_SYNCPTS,
	.moduleid        = NVHOST_MODULE_ISP,
	.modulemutexes   = {NVMODMUTEX_ISP_0},
	.exclusive       = true,
	.keepalive       = true,
	.powergate_ids   = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate   = true,
	.clockgate_delay = ISP_CLOCKGATE_DELAY,
	.powergate_delay = ISP_POWERGATE_DELAY,
	.clocks          = {
		{"isp", UINT_MAX, 0, TEGRA_MC_CLIENT_ISP},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID},
		{"sclk", 80000000} },
	.finalize_poweron = nvhost_isp_t124_finalize_poweron,
	.ctrl_ops         = &tegra_isp_ctrl_ops,
	.alloc_hwctx_handler = nvhost_alloc_hwctx_handler,
};
static struct platform_device tegra_isp01_device = {
	.name          = "isp",
	.resource      = isp_resources,
	.num_resources = ARRAY_SIZE(isp_resources),
	.dev           = {
		.platform_data = &t124_isp_info,
	},
};

static struct resource ispb_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_ISPB_BASE,
		.end = TEGRA_ISPB_BASE + TEGRA_ISPB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	}
};


struct nvhost_device_data t124_ispb_info = {
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts         = NV_ISP_1_SYNCPTS,
	.moduleid        = (1 << 16) | NVHOST_MODULE_ISP,
	.modulemutexes   = {NVMODMUTEX_ISP_1},
	.exclusive       = true,
	.keepalive       = true,
	.powergate_ids   = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate   = true,
	.clockgate_delay = ISP_CLOCKGATE_DELAY,
	.powergate_delay = ISP_POWERGATE_DELAY,
	.clocks          = {
		{"isp", UINT_MAX, 0, TEGRA_MC_CLIENT_ISPB},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID},
		{"sclk", 80000000} },
	.finalize_poweron = nvhost_isp_t124_finalize_poweron,
	.ctrl_ops         = &tegra_isp_ctrl_ops,
	.alloc_hwctx_handler = nvhost_alloc_hwctx_handler,
};

static struct platform_device tegra_isp01b_device = {
	.name          = "isp",
	.id            = 1, /* .1 on the dev node */
	.resource      = ispb_resources,
	.num_resources = ARRAY_SIZE(ispb_resources),
	.dev  = {
		.platform_data = &t124_ispb_info,
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

static struct platform_device tegra_vi01b_device;
struct nvhost_device_data t124_vi_info = {
	/* FIXME: resolve powergating dependency with DIS */
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts          = NV_VI_0_SYNCPTS,
	.moduleid         = NVHOST_MODULE_VI,
	.modulemutexes    = {NVMODMUTEX_VI_0},
	.exclusive        = true,
	.keepalive       = true,
	.powergate_ids    = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate    = true,
	.clockgate_delay  = VI_CLOCKGATE_DELAY,
	.powergate_delay  = VI_POWERGATE_DELAY,
	.clocks           = {
		{"vi", UINT_MAX, 0},
		{"csi", 0},
		{"cilab", 102000000},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID},
		{"sclk", 80000000} },
	.init             = nvhost_vi_init,
	.deinit           = nvhost_vi_deinit,
	.prepare_poweroff = nvhost_vi_prepare_poweroff,
	.finalize_poweron = nvhost_vi_finalize_poweron,
	.ctrl_ops         = &tegra_vi_ctrl_ops,
	.reset            = nvhost_vi_reset,
	.slave         = &tegra_vi01b_device,
	.alloc_hwctx_handler = nvhost_alloc_hwctx_handler,
};
EXPORT_SYMBOL(t124_vi_info);

static struct platform_device tegra_vi01_device = {
	.name		= "vi",
	.resource	= vi_resources,
	.num_resources	= ARRAY_SIZE(vi_resources),
	.dev		= {
		.platform_data = &t124_vi_info,
	},
};

struct nvhost_device_data t124_vib_info = {
	/* FIXME: resolve powergating dependency with DIS */
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts          = NV_VI_1_SYNCPTS,
	.moduleid         = (1 << 16 | NVHOST_MODULE_VI),
	.modulemutexes    = {NVMODMUTEX_VI_1},
	.exclusive        = true,
	.keepalive       = true,
	.powergate_ids    = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate    = true,
	.clockgate_delay  = VI_CLOCKGATE_DELAY,
	.powergate_delay  = VI_POWERGATE_DELAY,
	.clocks           = {
		{"vi", UINT_MAX},
		{"csi", 0},
		{"cilcd", 102000000},
		{"cile", 102000000},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID},
		{"sclk", 80000000} },
	.init             = nvhost_vi_init,
	.deinit           = nvhost_vi_deinit,
	.prepare_poweroff = nvhost_vi_prepare_poweroff,
	.finalize_poweron = nvhost_vi_finalize_poweron,
	.ctrl_ops         = &tegra_vi_ctrl_ops,
	.master           = &tegra_vi01_device,
	.reset            = nvhost_vi_reset,
	.alloc_hwctx_handler = nvhost_alloc_hwctx_handler,
};

static struct platform_device tegra_vi01b_device = {
	.name		= "vi",
	.id		= 1, /* .1 on the dev node */
	.dev		= {
		.platform_data = &t124_vib_info,
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

struct nvhost_device_data t124_msenc_info = {
	.version	= NVHOST_ENCODE_MSENC_VER(3, 1),
	.syncpts	= {NVSYNCPT_MSENC, NVSYNCPT_MSENC_SLICE},
	.waitbases	= {NVWAITBASE_MSENC},
	.class		= NV_VIDEO_ENCODE_MSENC_CLASS_ID,
	.clocks		= {{"msenc", UINT_MAX, 0, TEGRA_MC_CLIENT_MSENC},
			  {"emc", HOST_EMC_FLOOR} },
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_MSENC,
	.powergate_ids	= { TEGRA_POWERGATE_MPE, -1 },
	.powergate_delay = 100,
	.can_powergate	= true,
	.init           = nvhost_msenc_init,
	.deinit         = nvhost_msenc_deinit,
	.finalize_poweron = nvhost_msenc_finalize_poweron,
	.scaling_init	= nvhost_scale_init,
	.scaling_deinit	= nvhost_scale_deinit,
	.actmon_regs	= HOST1X_CHANNEL_ACTMON1_REG_BASE,
	.actmon_enabled	= true,
};

struct platform_device tegra_msenc03_device = {
	.name	       = "msenc",
	.id	       = -1,
	.resource      = msenc_resources,
	.num_resources = ARRAY_SIZE(msenc_resources),
	.dev           = {
		.platform_data = &t124_msenc_info,
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

struct nvhost_device_data t124_tsec_info = {
	.version       = NVHOST_ENCODE_TSEC_VER(1, 0),
	.syncpts       = {NVSYNCPT_TSEC},
	.waitbases     = {NVWAITBASE_TSEC},
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = true,
	.clocks	       = {{"tsec", UINT_MAX, 0, TEGRA_MC_CLIENT_TSEC},
			 {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.can_powergate    = true,
	.powergate_delay = TSEC_POWERGATE_DELAY,
	.keepalive       = true,
	.moduleid      = NVHOST_MODULE_TSEC,
	.init          = nvhost_tsec_init,
	.deinit        = nvhost_tsec_deinit,
	.finalize_poweron = nvhost_tsec_finalize_poweron,
	.prepare_poweroff = nvhost_tsec_prepare_poweroff,
};

static struct platform_device tegra_tsec01_device = {
	.name		= "tsec",
	.id		= -1,
	.resource	= tsec_resources,
	.num_resources	= ARRAY_SIZE(tsec_resources),
	.dev		= {
		.platform_data = &t124_tsec_info,
	},
};

#ifdef CONFIG_ARCH_TEGRA_VIC
static struct resource vic03_resources[] = {
	{
	.name = "base",
	.start = TEGRA_VIC_BASE,
	.end = TEGRA_VIC_BASE + TEGRA_VIC_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
};

struct nvhost_device_data t124_vic_info = {
	.syncpts		= {NVSYNCPT_VIC},
	.modulemutexes		= {NVMODMUTEX_VIC},
	.clocks			= {{"vic03", UINT_MAX, 0, TEGRA_MC_CLIENT_VIC},
				  {"emc", UINT_MAX} },
	.version = NVHOST_ENCODE_VIC_VER(3, 0),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VIC,
	.alloc_hwctx_handler = nvhost_vic03_alloc_hwctx_handler,
	.can_powergate		= true,
	.powergate_delay	= 500,
	.powergate_ids		= { TEGRA_POWERGATE_VIC, -1 },
	.init			= nvhost_vic03_init,
	.deinit			= nvhost_vic03_deinit,
	.alloc_hwctx_handler	= nvhost_vic03_alloc_hwctx_handler,
	.finalize_poweron	= nvhost_vic03_finalize_poweron,
	.prepare_poweroff	= nvhost_vic03_prepare_poweroff,
	.scaling_init		= nvhost_scale3d_init,
	.scaling_deinit		= nvhost_scale3d_deinit,
	.busy			= nvhost_scale_notify_busy,
	.idle			= nvhost_scale_notify_idle,
	.suspend_ndev		= nvhost_scale3d_suspend,
	.scaling_post_cb	= &nvhost_scale3d_callback,
	.devfreq_governor	= "nvhost_podgov",
	.actmon_regs		= HOST1X_CHANNEL_ACTMON2_REG_BASE,
	.actmon_enabled		= true,
	.linear_emc		= true,
};

struct platform_device tegra_vic03_device = {
	.name	       = "vic03",
	.num_resources = 1,
	.resource      = vic03_resources,
	.dev           = {
		.platform_data = &t124_vic_info,
	},
};
#endif

/*
 * T132 overrides for platform data.
 */

struct nvhost_device_data t132_isp_info = {
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts         = NV_ISP_0_SYNCPTS,
	.moduleid        = NVHOST_MODULE_ISP,
	.modulemutexes   = {NVMODMUTEX_ISP_0},
	.exclusive       = true,
	.keepalive       = true,
	.powergate_ids   = {TEGRA_POWERGATE_VENC, -1},
	.clockgate_delay = ISP_CLOCKGATE_DELAY,
	.powergate_delay = ISP_POWERGATE_DELAY,
	.clocks          = {
		{"isp", UINT_MAX, 0, TEGRA_MC_CLIENT_ISP},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID} },
	.finalize_poweron = nvhost_isp_t124_finalize_poweron,
	.ctrl_ops         = &tegra_isp_ctrl_ops,
};

struct nvhost_device_data t132_ispb_info = {
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts         = NV_ISP_1_SYNCPTS,
	.moduleid        = (1 << 16) | NVHOST_MODULE_ISP,
	.modulemutexes   = {NVMODMUTEX_ISP_1},
	.exclusive       = true,
	.keepalive       = true,
	.powergate_ids   = {TEGRA_POWERGATE_VENC, -1},
	.clockgate_delay = ISP_CLOCKGATE_DELAY,
	.powergate_delay = ISP_POWERGATE_DELAY,
	.clocks          = {
		{"isp", UINT_MAX, 0, TEGRA_MC_CLIENT_ISPB},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID} },
	.finalize_poweron = nvhost_isp_t124_finalize_poweron,
	.ctrl_ops         = &tegra_isp_ctrl_ops,
};

struct nvhost_device_data t132_vi_info = {
	/* FIXME: resolve powergating dependency with DIS */
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts          = NV_VI_0_SYNCPTS,
	.moduleid         = NVHOST_MODULE_VI,
	.modulemutexes    = {NVMODMUTEX_VI_0},
	.exclusive        = true,
	.keepalive       = true,
	.powergate_ids    = {TEGRA_POWERGATE_VENC, -1},
	.clockgate_delay  = VI_CLOCKGATE_DELAY,
	.powergate_delay  = VI_POWERGATE_DELAY,
	.clocks           = {
		{"vi", UINT_MAX, 0},
		{"csi", 0},
		{"cilab", 102000000},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID} },
	.init             = nvhost_vi_init,
	.deinit           = nvhost_vi_deinit,
	.prepare_poweroff = nvhost_vi_prepare_poweroff,
	.finalize_poweron = nvhost_vi_finalize_poweron,
	.ctrl_ops         = &tegra_vi_ctrl_ops,
	.reset            = nvhost_vi_reset,
	.slave         = &tegra_vi01b_device,
};

struct nvhost_device_data t132_vib_info = {
	/* FIXME: resolve powergating dependency with DIS */
	/* FIXME: control clocks from user space instead of hard-coding here */
	.syncpts          = NV_VI_1_SYNCPTS,
	.moduleid         = (1 << 16 | NVHOST_MODULE_VI),
	.modulemutexes    = {NVMODMUTEX_VI_1},
	.exclusive        = true,
	.keepalive       = true,
	.powergate_ids    = {TEGRA_POWERGATE_VENC, -1},
	.clockgate_delay  = VI_CLOCKGATE_DELAY,
	.powergate_delay  = VI_POWERGATE_DELAY,
	.clocks           = {
		{"vi", UINT_MAX},
		{"csi", 0},
		{"cilcd", 102000000},
		{"cile", 102000000},
		{"emc", 0, TEGRA_HOST1X_EMC_MODULE_ID} },
	.init             = nvhost_vi_init,
	.deinit           = nvhost_vi_deinit,
	.prepare_poweroff = nvhost_vi_prepare_poweroff,
	.finalize_poweron = nvhost_vi_finalize_poweron,
	.ctrl_ops         = &tegra_vi_ctrl_ops,
	.master           = &tegra_vi01_device,
	.reset            = nvhost_vi_reset,
};

struct nvhost_device_data t132_msenc_info = {
	.version	= NVHOST_ENCODE_MSENC_VER(3, 1),
	.syncpts	= {NVSYNCPT_MSENC, NVSYNCPT_MSENC_SLICE},
	.waitbases	= {NVWAITBASE_MSENC},
	.class		= NV_VIDEO_ENCODE_MSENC_CLASS_ID,
	.clocks		= {{"msenc", UINT_MAX, 0, TEGRA_MC_CLIENT_MSENC},
			  {"emc", HOST_EMC_FLOOR} },
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_MSENC,
	.powergate_ids	= { TEGRA_POWERGATE_MPE, -1 },
	.powergate_delay = 100,
	.init           = nvhost_msenc_init,
	.deinit         = nvhost_msenc_deinit,
	.finalize_poweron = nvhost_msenc_finalize_poweron,
};

struct nvhost_device_data t132_tsec_info = {
	.version       = NVHOST_ENCODE_TSEC_VER(1, 0),
	.syncpts       = {NVSYNCPT_TSEC},
	.waitbases     = {NVWAITBASE_TSEC},
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = true,
	.clocks	       = {{"tsec", UINT_MAX, 0, TEGRA_MC_CLIENT_TSEC},
			 {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_TSEC,
	.init          = nvhost_tsec_init,
	.deinit        = nvhost_tsec_deinit,
	.finalize_poweron = nvhost_tsec_finalize_poweron,
};

#ifdef CONFIG_ARCH_TEGRA_VIC
struct nvhost_device_data t132_vic_info = {
	.syncpts		= {NVSYNCPT_VIC},
	.modulemutexes		= {NVMODMUTEX_VIC},
	.clocks			= {{"vic03", UINT_MAX, 0, TEGRA_MC_CLIENT_VIC},
				  {"emc", UINT_MAX} },
	.version = NVHOST_ENCODE_VIC_VER(3, 0),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VIC,
	.alloc_hwctx_handler = nvhost_vic03_alloc_hwctx_handler,
	.powergate_delay	= 500,
	.powergate_ids		= { TEGRA_POWERGATE_VIC, -1 },
	.init			= nvhost_vic03_init,
	.deinit			= nvhost_vic03_deinit,
	.alloc_hwctx_handler	= nvhost_vic03_alloc_hwctx_handler,
	.finalize_poweron	= nvhost_vic03_finalize_poweron,
	.prepare_poweroff	= nvhost_vic03_prepare_poweroff,
};
#endif

#if defined(CONFIG_TEGRA_GK20A)
struct nvhost_device_data t132_gk20a_info = {
	.syncpts		= {NVSYNCPT_GK20A_BASE},
	.syncpt_base		= NVSYNCPT_GK20A_BASE,
	.class			= NV_GRAPHICS_GPU_CLASS_ID,
	.clocks			= {{"PLLG_ref", UINT_MAX},
				   {"pwr", 204000000},
				   {"emc", UINT_MAX},
				   {} },
	.powergate_ids		= { TEGRA_POWERGATE_GPU, -1 },
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.powergate_delay	= 500,
	.as_ops			= &tegra_gk20a_as_ops,
	.moduleid		= NVHOST_MODULE_GPU,
	.prepare_poweroff	= nvhost_gk20a_prepare_poweroff,
	.finalize_poweron	= nvhost_gk20a_finalize_poweron,
#ifdef CONFIG_GK20A_DEVFREQ
	.busy			= gk20a_scale_notify_busy,
	.idle			= gk20a_scale_notify_idle,
	.scaling_init		= nvhost_gk20a_scale_init,
	.scaling_deinit		= nvhost_gk20a_scale_deinit,
	.suspend_ndev		= nvhost_scale3d_suspend,
	.devfreq_governor	= "nvhost_podgov",
	.scaling_post_cb	= nvhost_gk20a_scale_callback,
	.gpu_edp_device		= true,
	.qos_id			= PM_QOS_GPU_FREQ_MIN,
#endif
};
#endif

static struct {
	struct nvhost_device_data *from;
	struct nvhost_device_data *to;
} t132_override[] = {
	{&t124_isp_info, &t132_isp_info},
	{&t124_ispb_info, &t132_ispb_info},
	{&t124_vi_info, &t132_vi_info},
	{&t124_msenc_info, &t132_msenc_info},
	{&t124_tsec_info, &t132_tsec_info},
#if defined(CONFIG_TEGRA_GK20A)
	{(void *)&gk20a_tegra_platform, &t132_gk20a_info},
#endif
#if defined(CONFIG_ARCH_TEGRA_VIC)
	{&t124_vic_info, &t132_vic_info},
#endif
};

static struct platform_device *t124_devices[] = {
	&tegra_isp01_device,
	&tegra_isp01b_device,
	&tegra_vi01_device,
	&tegra_msenc03_device,
	&tegra_tsec01_device,
#if defined(CONFIG_TEGRA_GK20A)
	&tegra_gk20a_device,
#endif
#if defined(CONFIG_ARCH_TEGRA_VIC)
	&tegra_vic03_device,
#endif
};


struct platform_device *tegra12_register_host1x_devices(void)
{
	int i = 0;
	struct platform_device *pdev;

	nvhost_dbg_fn("");

	/* register host1x device first */
	platform_device_register(&tegra_host1x04_device);
	tegra_host1x04_device.dev.parent = NULL;

	/* register clients with host1x device as parent */
	for (i = 0; i < ARRAY_SIZE(t124_devices); i++) {
		pdev = t124_devices[i];
		pdev->dev.parent = &tegra_host1x04_device.dev;
		platform_device_register(pdev);
	}

	return &tegra_host1x04_device;
}

#include "host1x/host1x_channel.c"

static void t124_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_dbg_fn("");
	nvhost_free_channel_internal(ch, &t124_num_alloc_channels);
}

static struct nvhost_channel *t124_alloc_nvhost_channel(
		struct platform_device *dev)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	struct nvhost_channel *ch;
	nvhost_dbg_fn("");
	ch = nvhost_alloc_channel_internal(pdata->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t124_num_alloc_channels);
	if (ch) {
#if defined(CONFIG_TEGRA_GK20A)
		if (strncmp(dev->name, "gk20a", GK20A_DEV_NAME_SIZE) == 0) {
			ch->ops.init          = host1x_channel_ops.init;
		} else
#endif
			ch->ops = host1x_channel_ops;

	}
	return ch;
}

int nvhost_init_t124_channel_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	op->nvhost_dev.alloc_nvhost_channel = t124_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t124_free_nvhost_channel;

	return 0;
}

static void t124_remove_support(struct nvhost_chip_support *op)
{
	kfree(op->priv);
	op->priv = 0;
}

#include "host1x/host1x_cdma.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr.c"
#include "host1x/host1x_actmon_t124.c"

int nvhost_init_t124_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	int i = 0;
	int err;
	struct t124 *t124 = 0;

	for (i = NVSYNCPT_GK20A_BASE; i <= NVSYNCPT_GK20A_LAST; i++)
		s_syncpt_names[i] = "gk20a";

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t124_channel_support(host, op);
	if (err)
		return err;

	op->cdma = host1x_cdma_ops;
	op->push_buffer = host1x_pushbuffer_ops;

	err = nvhost_init_t124_debug_support(op);
	if (err)
		return err;

	host->sync_aperture = host->aperture + HOST1X_CHANNEL_SYNC_REG_BASE;
	op->syncpt = host1x_syncpt_ops;
	op->intr = host1x_intr_ops;
	op->actmon = host1x_actmon_ops;

	err = nvhost_memmgr_init(op);
	if (err)
		return err;

	t124 = kzalloc(sizeof(struct t124), GFP_KERNEL);
	if (!t124) {
		err = -ENOMEM;
		goto err;
	}

	t124->host = host;
	op->priv = t124;
	op->remove_support = t124_remove_support;

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA13) {
		for (i = 0; i < ARRAY_SIZE(t132_override); i++) {
			struct nvhost_device_data *from = t132_override[i].from;
			struct nvhost_device_data *to = t132_override[i].to;

			/* replace the platform data by t132 data */
			*from = *to;
		}
	}

	return 0;

err:
	kfree(t124);

	op->priv = 0;
	op->remove_support = 0;
	return err;
}
