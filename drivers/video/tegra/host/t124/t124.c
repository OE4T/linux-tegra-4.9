/*
 * drivers/video/tegra/host/t124/t124.c
 *
 * Tegra Graphics Init for T124 Architecture Chips
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/tegra-powergate.h>

#include "dev.h"
#include "nvhost_job.h"
#include "class_ids.h"

#include "t124.h"
#include "host1x/host1x.h"

#include "hardware_t124.h"
#include "syncpt_t124.h"

#include "gk20a/gk20a.h"
#include "t20/t20.h"
#include "vic03/vic03.h"
#include "msenc/msenc.h"
#include "tsec/tsec.h"
#include "vi/vi.h"
#include "isp/isp.h"

#include "nvhost_memmgr.h"
#include "chip_support.h"
#include "nvhost_scale.h"

static int t124_num_alloc_channels = 0;

#define HOST_EMC_FLOOR 300000000
/* FIXME: Tune Powergate and clockgate delays */
#define VI_CLOCKGATE_DELAY 60
#define VI_POWERGATE_DELAY 500
#define ISP_CLOCKGATE_DELAY 60
#define ISP_POWERGATE_DELAY 500

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
	.clocks		= {{"host1x", 81600000}, {"actmon", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	.private_data	= &host1x04_info,
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
	/* FIXME: Add control for sclk/emc */
	.syncpts         = NV_ISP_0_SYNCPTS,
	.moduleid        = NVHOST_MODULE_ISP,
	.modulemutexes   = {NVMODMUTEX_ISP_0},
	.exclusive       = true,
	.powergate_ids   = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate   = false,
	.clockgate_delay = ISP_CLOCKGATE_DELAY,
	.powergate_delay = ISP_POWERGATE_DELAY,
	.clocks          = { {"isp", UINT_MAX} },
	.slave           = &tegra_isp01b_device,
	.finalize_poweron = nvhost_isp_t124_finalize_poweron,
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
	/* FIXME: Add control for sclk/emc */
	.syncpts         = NV_ISP_1_SYNCPTS,
	.moduleid        = (1 << 16) | NVHOST_MODULE_ISP,
	.modulemutexes   = {NVMODMUTEX_ISP_1},
	.exclusive       = true,
	.powergate_ids   = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate   = false,
	.clockgate_delay = ISP_CLOCKGATE_DELAY,
	.powergate_delay = ISP_POWERGATE_DELAY,
	.clocks          = { {"isp", UINT_MAX} },
	.finalize_poweron = nvhost_isp_t124_finalize_poweron,
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
	/* FIXME: Add control for sclk/emc */
	.syncpts          = NV_VI_0_SYNCPTS,
	.moduleid         = NVHOST_MODULE_VI,
	.modulemutexes    = {NVMODMUTEX_VI_0},
	.exclusive        = true,
	.powergate_ids    = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate    = false,
	.clockgate_delay  = VI_CLOCKGATE_DELAY,
	.powergate_delay  = VI_POWERGATE_DELAY,
	.clocks           = {
		{"vi", UINT_MAX},
		{"csi", 0},
		{"cilab", 102000000} },
	.init             = nvhost_vi_init,
	.deinit           = nvhost_vi_deinit,
	.prepare_poweroff = nvhost_vi_prepare_poweroff,
	.finalize_poweron = nvhost_vi_finalize_poweron,
	.ctrl_ops         = &tegra_vi_ctrl_ops,
	.slave         = &tegra_vi01b_device,
};


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
	/* FIXME: Add control for sclk/emc */
	.syncpts          = NV_VI_1_SYNCPTS,
	.moduleid         = (1 << 16 | NVHOST_MODULE_VI),
	.modulemutexes    = {NVMODMUTEX_VI_1},
	.exclusive        = true,
	.powergate_ids    = {TEGRA_POWERGATE_VENC, -1},
	.can_powergate    = false,
	.clockgate_delay  = VI_CLOCKGATE_DELAY,
	.powergate_delay  = VI_POWERGATE_DELAY,
	.clocks           = {
		{"vi", UINT_MAX},
		{"csi", 0},
		{"cilcd", 102000000},
		{"cile", 102000000} },
	.init             = nvhost_vi_init,
	.deinit           = nvhost_vi_deinit,
	.prepare_poweroff = nvhost_vi_prepare_poweroff,
	.finalize_poweron = nvhost_vi_finalize_poweron,
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
	.clocks		= {{"msenc", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid	= NVHOST_MODULE_MSENC,
	.powergate_ids	= { TEGRA_POWERGATE_MPE, -1 },
	.powergate_delay = 100,
	.can_powergate	= true,
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
	.clocks = {{"tsec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_TSEC,
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



static struct platform_device *t124_devices[] = {
	&tegra_isp01_device,
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

	for (i = NVSYNCPT_GK20A_BASE; i <= NVSYNCPT_GK20A_LAST; i++) {
		s_syncpt_names[i] = "gk20a";
	}

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

static inline void __iomem *t124_channel_aperture(void __iomem *p, int ndx)
{
	return p + (ndx * NV_HOST1X_CHANNEL_MAP_SIZE_BYTES);
}

static int t124_channel_init(struct nvhost_channel *ch,
			    struct nvhost_master *dev, int index)
{
	ch->chid = index;
	mutex_init(&ch->reflock);
	mutex_init(&ch->submitlock);

	ch->aperture = t124_channel_aperture(dev->aperture, index);

	nvhost_dbg_fn("dev=%s chid=%d ap=%p",
		      dev_name(&ch->dev->dev),
		      ch->chid,
		      ch->aperture);

	return t124_nvhost_hwctx_handler_init(ch);
}

#include "host1x/host1x_channel.c"
static int t124_channel_submit(struct nvhost_job *job)
{
	nvhost_dbg_fn("");

#if defined(CONFIG_TEGRA_GK20A)
	if (is_gk20a_module(job->ch->dev))
		return -ENODEV; /* makes no sense/shouldn't happen */
	else
#endif
		return host1x_channel_submit(job);
}

#if defined(CONFIG_TEGRA_GK20A)
static int t124_channel_alloc_obj(struct nvhost_hwctx *hwctx,
				 struct nvhost_alloc_obj_ctx_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_alloc_obj_ctx(hwctx->priv, args);
}

static int t124_channel_free_obj(struct nvhost_hwctx *hwctx,
				struct nvhost_free_obj_ctx_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_free_obj_ctx(hwctx->priv, args);
}

static int t124_channel_alloc_gpfifo(struct nvhost_hwctx *hwctx,
				    struct nvhost_alloc_gpfifo_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_alloc_channel_gpfifo(hwctx->priv, args);
}

static int t124_channel_submit_gpfifo(struct nvhost_hwctx *hwctx,
				     struct nvhost_gpfifo *gpfifo, u32 num_entries,
				     struct nvhost_fence *fence, u32 flags)
{
	struct channel_gk20a *ch = hwctx->priv;
	struct nvhost_channel *nvhost_ch = hwctx->channel;
	void *completed_waiter = NULL;
	int err, ret;

	nvhost_dbg_fn("");

	if (hwctx->has_timedout || !ch)
		return -ETIMEDOUT;

	completed_waiter = nvhost_intr_alloc_waiter();
	if (!completed_waiter)
		return -ENOMEM;

	nvhost_module_busy(nvhost_ch->dev);

	ret = gk20a_submit_channel_gpfifo(hwctx->priv, gpfifo, num_entries,
					fence, flags);
	if (!ret) {
		err = nvhost_intr_add_action(
			&nvhost_get_host(nvhost_ch->dev)->intr,
			fence->syncpt_id, fence->value,
			NVHOST_INTR_ACTION_GPFIFO_SUBMIT_COMPLETE,
			ch,
			completed_waiter,
			NULL);
		WARN(err, "Failed to set submit complete interrupt");
	} else {
		pr_err("submit error %d\n", ret);
		kfree(completed_waiter);
	}
	return ret;
}

static int t124_channel_wait(struct nvhost_hwctx *hwctx,
			    struct nvhost_wait_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_wait(hwctx->priv, args);
}

#if defined(CONFIG_TEGRA_GPU_CYCLE_STATS)
static int t124_channel_cycle_stats(struct nvhost_hwctx *hwctx,
				struct nvhost_cycle_stats_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_cycle_stats(hwctx->priv, args);
}
#endif

static int t124_channel_zcull_bind(struct nvhost_hwctx *hwctx,
			    struct nvhost_zcull_bind_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_zcull_bind(hwctx->priv, args);
}
#endif /* CONFIG_TEGRA_GK20A */

static void t124_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_dbg_fn("");
	nvhost_free_channel_internal(ch, &t124_num_alloc_channels);
}

static struct nvhost_channel *t124_alloc_nvhost_channel(
		struct platform_device *dev)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
	nvhost_dbg_fn("");
	return nvhost_alloc_channel_internal(pdata->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t124_num_alloc_channels);
}

int nvhost_init_t124_channel_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	int i, num_channels;

	/* Set indices dynamically as we can have
	 * missing/non-static devices above (e.g.: vic, gk20a).
	 */

	for (num_channels = i = 0; i < ARRAY_SIZE(t124_devices); i++) {
		struct platform_device *dev = t124_devices[i];
		struct nvhost_device_data *pdata =
			(struct nvhost_device_data *)dev->dev.platform_data;
		pdata->index = num_channels++;
		nvhost_dbg_fn("assigned channel %d to %s",
			      pdata->index, dev_name(&dev->dev));
#if defined(CONFIG_ARCH_TEGRA_VIC)
		if (dev == &tegra_vic03_device) {
			pdata->modulemutexes[0] = NVMODMUTEX_VIC;
			pdata->syncpts[0] = NVSYNCPT_VIC;
		}
#endif
#if defined(CONFIG_TEGRA_GK20A)
		if (dev == &tegra_gk20a_device) {
			pdata->syncpts[0]       = NVSYNCPT_GK20A_BASE;
			pdata->syncpt_base      = NVSYNCPT_GK20A_BASE;
			pdata->waitbases[0]     = NVWAITBASE_3D;
			pdata->modulemutexes[0] = NVMODMUTEX_3D;
		}
#endif
		if (pdata->slave) {
			struct nvhost_device_data *slave_pdata =
				(struct nvhost_device_data *)pdata->slave->dev.platform_data;
			slave_pdata->index = num_channels++;
			nvhost_dbg_fn("assigned channel %d to %s",
				      slave_pdata->index,
				      dev_name(&pdata->slave->dev));
		}
	}
	nvhost_dbg_fn("max channels=%d num channels=%zd",
		      NV_HOST1X_CHANNELS, num_channels);
	if (num_channels > T124_NVHOST_NUMCHANNELS) {
		WARN(-ENODEV, "too many channel devices");
		return -ENODEV;
	}

	op->channel.init          = t124_channel_init;
	op->channel.submit        = t124_channel_submit;

#if defined(CONFIG_TEGRA_GK20A)
	op->channel.alloc_obj     = t124_channel_alloc_obj;
	op->channel.free_obj      = t124_channel_free_obj;
	op->channel.alloc_gpfifo  = t124_channel_alloc_gpfifo;
	op->channel.submit_gpfifo = t124_channel_submit_gpfifo;
	op->channel.wait          = t124_channel_wait;

#if defined(CONFIG_TEGRA_GPU_CYCLE_STATS)
	op->channel.cycle_stats   = t124_channel_cycle_stats;
#endif
	op->channel.zcull.bind     = t124_channel_zcull_bind;
#endif
	op->nvhost_dev.alloc_nvhost_channel = t124_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t124_free_nvhost_channel;

	return 0;
}

int t124_nvhost_hwctx_handler_init(struct nvhost_channel *ch)
{
	int err = 0;
	struct nvhost_device_data *pdata = nvhost_get_devdata(ch->dev);
	u32 syncpt = pdata->syncpts[0];
	u32 waitbase = pdata->waitbases[0];

	nvhost_dbg_fn("");

	if (pdata->alloc_hwctx_handler) {
		ch->ctxhandler = pdata->alloc_hwctx_handler(syncpt,
				waitbase, ch);
		if (!ch->ctxhandler)
			err = -ENOMEM;
	}

	return err;
}

static void t124_remove_support(struct nvhost_chip_support *op)
{
	kfree(op->priv);
	op->priv = 0;
}

#include "host1x/host1x_syncpt.c"
#include "host1x/host1x_intr.c"
#include "host1x/host1x_actmon_t124.c"

int nvhost_init_t124_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	int err;
	struct t124 *t124 = 0;

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t124_channel_support(host, op);
	if (err)
		return err;

	err = nvhost_init_t124_cdma_support(op);
	if (err)
		return err;

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

	err = nvhost_init_t124_as_support(op);
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
	op->nvhost_dev.alloc_nvhost_channel = t124_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t124_free_nvhost_channel;

	return 0;

err:
	kfree(t124);

	op->priv = 0;
	op->remove_support = 0;
	return err;
}
