/*
 * Tegra Graphics Init for T210 Architecture Chips
 *
 * Copyright (c) 2011-2014, NVIDIA Corporation.  All rights reserved.
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
#include <linux/io.h>

#include <mach/mc.h>

#include "dev.h"
#include "nvhost_job.h"
#include "class_ids.h"

#include "t210.h"
#include "host1x/host1x.h"
#include "hardware_t124.h"
#include "syncpt_t124.h"
#include "msenc/msenc.h"
#include "nvdec/nvdec.h"
#include "nvjpg/nvjpg.h"
#include "tsec/tsec.h"
#include "vic03/vic03.h"
#include "vi/vi.h"

#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "nvhost_scale.h"

#define HOST_EMC_FLOOR 300000000

#define BIT64(nr) (1ULL << (nr))
#define NVSYNCPTS_CLIENT_MANAGED_T210 ( \
	BIT64(NVSYNCPT_DISP0_A) | BIT64(NVSYNCPT_DISP1_A) | \
	BIT64(NVSYNCPT_DISP0_B) | BIT64(NVSYNCPT_DISP1_B) | \
	BIT64(NVSYNCPT_DISP0_C) | BIT64(NVSYNCPT_DISP1_C) | \
	BIT64(NVSYNCPT_DISP0_D) | \
	BIT64(NVSYNCPT_DISP0_H) | BIT64(NVSYNCPT_DISP1_H) | \
	BIT64(NVSYNCPT_DSI) | \
	BIT64(NVSYNCPT_VBLANK0) | BIT64(NVSYNCPT_VBLANK1) | \
	BIT64(NVSYNCPT_AVP_0))

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
	[NVSYNCPT_DISP1_C]	= "disp0c",
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
	.client_managed	= NVSYNCPTS_CLIENT_MANAGED_T210,
};

struct nvhost_device_data t21_host1x_info = {
	.clocks		= {{"host1x", UINT_MAX}, {"actmon", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.private_data	= &host1x04_info,
};

struct nvhost_device_data t21_isp_info = {
	.syncpts = NV_ISP_0_SYNCPTS,
	.modulemutexes = {NVMODMUTEX_ISP_0},
	.exclusive     = true,
	/* HACK: Mark as keepalive until 1188795 is fixed */
	.keepalive = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks        = {{ "isp", UINT_MAX, 0, TEGRA_MC_CLIENT_ISP }},
	.moduleid      = NVHOST_MODULE_ISP,
};

struct nvhost_device_data t21_vi_info = {
	.syncpts       = NV_VI_0_SYNCPTS,
	.modulemutexes = {NVMODMUTEX_VI_0},
	.exclusive     = true,
	/* HACK: Mark as keepalive until 1188795 is fixed */
	.keepalive = true,
	.clocks		= {{"vi", UINT_MAX}, {"csi", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VI,
	.clocks = {
		{"vi", UINT_MAX},
		{"csi", 0},
		{"cilab", 102000000} },
	.ctrl_ops         = &tegra_vi_ctrl_ops,
};

struct nvhost_device_data t21_msenc_info = {
	.version       = NVHOST_ENCODE_MSENC_VER(5, 0),
	.syncpts       = {NVSYNCPT_MSENC, NVSYNCPT_MSENC_SLICE},
	.waitbases     = {NVWAITBASE_MSENC},
	.class	       = NV_VIDEO_ENCODE_NVENC_CLASS_ID,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.exclusive     = true,
	.keepalive     = true,
	.clocks		= {{"msenc", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_msenc_init,
	.deinit		= nvhost_msenc_deinit,
	.moduleid	= NVHOST_MODULE_MSENC,
};

struct nvhost_device_data t21_nvdec_info = {
	.version       = NVHOST_ENCODE_NVDEC_VER(2, 0),
	.syncpts       = {NVSYNCPT_NVDEC},
	.class	       = NV_NVDEC_CLASS_ID,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.exclusive     = true,
	.keepalive     = true,
	.clocks		= {{"nvdec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_nvdec_init,
	.deinit		= nvhost_nvdec_deinit,
	.moduleid	= NVHOST_MODULE_NVDEC,
};

struct nvhost_device_data t21_nvjpg_info = {
	.version       = NVHOST_ENCODE_NVJPG_VER(1, 0),
	.syncpts       = {NVSYNCPT_NVJPG},
	.class	       = NV_NVJPG_CLASS_ID,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.exclusive     = true,
	.keepalive     = true,
	.clocks		= { {"nvjpg", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_nvjpg_init,
	.deinit		= nvhost_nvjpg_deinit,
	.moduleid	= NVHOST_MODULE_NVJPG,
};

struct nvhost_device_data t21_tsec_info = {
	.version       = NVHOST_ENCODE_TSEC_VER(1, 0),
	.syncpts       = {NVSYNCPT_TSEC},
	.waitbases     = {NVWAITBASE_TSEC},
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks		= {{"tsec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.moduleid      = NVHOST_MODULE_TSEC,
};
#ifdef CONFIG_ARCH_TEGRA_VIC
struct nvhost_device_data t21_vic_info = {
	.syncpts	= {NVSYNCPT_VIC},
	.modulemutexes	= {NVMODMUTEX_VIC},
	.version = NVHOST_ENCODE_VIC_VER(4, 0),
	.clocks = {{"vic03", UINT_MAX}, {"emc", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VIC,
	.alloc_hwctx_handler = nvhost_vic03_alloc_hwctx_handler,
	.prepare_poweroff	= nvhost_vic03_prepare_poweroff,

	.init			= nvhost_vic03_init,
	.deinit			= nvhost_vic03_deinit,
	.alloc_hwctx_handler	= nvhost_vic03_alloc_hwctx_handler,
	.finalize_poweron	= nvhost_vic03_finalize_poweron,
};
#endif
