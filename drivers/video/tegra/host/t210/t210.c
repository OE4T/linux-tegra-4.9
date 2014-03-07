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
#include <linux/tegra-powergate.h>

#include "dev.h"
#include "nvhost_job.h"
#include "class_ids.h"

#include "t210.h"
#include "t124/t124.h"
#include "host1x/host1x.h"
#include "hardware_t124.h"
#include "syncpt_t124.h"
#include "msenc/msenc.h"
#include "nvdec/nvdec.h"
#include "nvjpg/nvjpg.h"
#include "tsec/tsec.h"
#include "vic03/vic03.h"
#include "vi/vi.h"
#include "isp/isp.h"
#include "gk20a/gk20a.h"

#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "nvhost_scale.h"

#define HOST_EMC_FLOOR 300000000

static int t210_num_alloc_channels = 0;

static struct host1x_device_info host1x04_info = {
	.nb_channels	= T124_NVHOST_NUMCHANNELS,
	.nb_pts		= NV_HOST1X_SYNCPT_NB_PTS,
	.nb_mlocks	= NV_HOST1X_NB_MLOCKS,
	.nb_bases	= NV_HOST1X_SYNCPT_NB_BASES,
};

struct nvhost_device_data t21_host1x_info = {
	.clocks		= {{"host1x", UINT_MAX}, {"actmon", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.private_data	= &host1x04_info,
};

struct nvhost_device_data t21_isp_info = {
	.modulemutexes = {NVMODMUTEX_ISP_0},
	.exclusive     = true,
	/* HACK: Mark as keepalive until 1188795 is fixed */
	.keepalive = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks        = {{ "isp", UINT_MAX, 0, TEGRA_MC_CLIENT_ISP }},
	.moduleid      = NVHOST_MODULE_ISP,
	.ctrl_ops      = &tegra_isp_ctrl_ops,
};

struct nvhost_device_data t21_vib_info = {
	.modulemutexes = {NVMODMUTEX_VI_1},
	.exclusive     = true,
	/* HACK: Mark as keepalive until 1188795 is fixed */
	.keepalive = true,
	.clocks		= {{"vi", UINT_MAX}, {"csi", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VI,
	.ctrl_ops         = &tegra_vi_ctrl_ops,
};

static struct platform_device tegra_vi01b_device = {
	.name		= "vi",
	.id		= 1, /* .1 on the dev node */
	.dev		= {
		.platform_data = &t21_vib_info,
	},
};

struct nvhost_device_data t21_vi_info = {
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
	.slave         = &tegra_vi01b_device,
};

struct nvhost_device_data t21_msenc_info = {
	.version       = NVHOST_ENCODE_MSENC_VER(5, 0),
	.waitbases     = {NVWAITBASE_MSENC},
	.class	       = NV_VIDEO_ENCODE_NVENC_CLASS_ID,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.keepalive     = true,
	.clocks		= {{"msenc", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_msenc_init,
	.deinit		= nvhost_msenc_deinit,
	.moduleid	= NVHOST_MODULE_MSENC,
};

struct nvhost_device_data t21_nvdec_info = {
	.version       = NVHOST_ENCODE_NVDEC_VER(2, 0),
	.class	       = NV_NVDEC_CLASS_ID,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.keepalive     = true,
	.clocks		= {{"nvdec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_nvdec_init,
	.deinit		= nvhost_nvdec_deinit,
	.moduleid	= NVHOST_MODULE_NVDEC,
};

struct nvhost_device_data t21_nvjpg_info = {
	.version       = NVHOST_ENCODE_NVJPG_VER(1, 0),
	.class	       = NV_NVJPG_CLASS_ID,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.keepalive     = true,
	.clocks		= { {"nvjpg", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_nvjpg_init,
	.deinit		= nvhost_nvjpg_deinit,
	.moduleid	= NVHOST_MODULE_NVJPG,
};

struct nvhost_device_data t21_tsec_info = {
	.version       = NVHOST_ENCODE_TSEC_VER(1, 0),
	.waitbases     = {NVWAITBASE_TSEC},
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.clocks		= {{"tsec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	.init		= nvhost_tsec_init,
	.deinit		= nvhost_tsec_deinit,
	.moduleid      = NVHOST_MODULE_TSEC,
};
#ifdef CONFIG_ARCH_TEGRA_VIC
struct nvhost_device_data t21_vic_info = {
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

#include "host1x/host1x_channel.c"

static void t210_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_dbg_fn("");
	nvhost_free_channel_internal(ch, &t210_num_alloc_channels);
}

static struct nvhost_channel *t210_alloc_nvhost_channel(
		struct platform_device *dev)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	struct nvhost_channel *ch;
	nvhost_dbg_fn("");
	ch = nvhost_alloc_channel_internal(pdata->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t210_num_alloc_channels);
	if (ch) {
#if defined(CONFIG_TEGRA_GK20A)
		if (!strcmp(dev->name, "57000000.gm20b")) {
			ch->ops.init          = host1x_channel_ops.init;
		} else
#endif
			ch->ops = host1x_channel_ops;

	}
	return ch;
}

int nvhost_init_t210_channel_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	op->nvhost_dev.alloc_nvhost_channel = t210_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t210_free_nvhost_channel;

	return 0;
}

static void t210_remove_support(struct nvhost_chip_support *op)
{
	kfree(op->priv);
	op->priv = 0;
}

#include "host1x/host1x_cdma.c"
#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr.c"
#include "host1x/host1x_actmon_t124.c"

int nvhost_init_t210_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	int err;
	struct t124 *t210 = 0;

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t210_channel_support(host, op);
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

	t210 = kzalloc(sizeof(struct t124), GFP_KERNEL);
	if (!t210) {
		err = -ENOMEM;
		goto err;
	}

	t210->host = host;
	op->priv = t210;
	op->remove_support = t210_remove_support;

	return 0;

err:
	kfree(t210);

	op->priv = 0;
	op->remove_support = 0;
	return err;
}
