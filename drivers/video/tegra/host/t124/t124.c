/*
 * drivers/video/tegra/host/t124/t124.c
 *
 * Tegra Graphics Init for T124 Architecture Chips
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

#include "nvhost_memmgr.h"
#include "chip_support.h"

static int t124_num_alloc_channels = 0;

static struct resource tegra_host1x04_resources[] = {
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

static struct host1x_device_info host1x04_info = {
	.nb_channels	= T124_NVHOST_NUMCHANNELS,
};

static struct nvhost_device tegra_host1x04_device = {
	.dev		= {.platform_data = &host1x04_info},
	.name		= "host1x",
	.id		= -1,
	.resource	= tegra_host1x04_resources,
	.num_resources	= ARRAY_SIZE(tegra_host1x04_resources),
	.clocks		= {{"host1x", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
};

static struct resource msenc_resources[] = {
	{
		.name = "regs",
		.start = TEGRA_MSENC_BASE,
		.end = TEGRA_MSENC_BASE + TEGRA_MSENC_SIZE - 1,
		.flags = IORESOURCE_MEM,
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

static struct nvhost_device *channel_devices[] = {
	&tegra_host1x04_device,
(struct nvhost_device []){{
	.name	       = "display",
	.syncpts       = BIT(NVSYNCPT_DISP0) | BIT(NVSYNCPT_DISP1) |
			 BIT(NVSYNCPT_VBLANK0) | BIT(NVSYNCPT_VBLANK1),
	.modulemutexes = BIT(NVMODMUTEX_DISPLAYA) | BIT(NVMODMUTEX_DISPLAYB),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
},},
(struct nvhost_device []){{
	.name	 = "isp",
	.syncpts = NV_ISP_0_SYNCPTS,
	.modulemutexes = BIT(NVMODMUTEX_ISP_0),
	.clocks = {{"isp", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_ISP,
},},
(struct nvhost_device []){{
	.name	 = "isp",
	.id      = 1, /* .1 on the dev node */
	.syncpts       = NV_ISP_1_SYNCPTS,
	.modulemutexes = BIT(NVMODMUTEX_ISP_1),
	.clocks = {{"isp.1", UINT_MAX}, {} },
	.exclusive     = true,
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = (1 << 16) | NVHOST_MODULE_ISP,
},},
(struct nvhost_device [])
{{	.name	       = "vi",
	.syncpts       = NV_VI_0_SYNCPTS,
	.modulemutexes = BIT(NVMODMUTEX_VI_0),
	.exclusive     = true,
	.clocks = {{"vi", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_VI,
},},
(struct nvhost_device [])
{{	.name	       = "vi",
	.id            = 1, /* .1 on the dev node */
	.syncpts       = NV_VI_1_SYNCPTS,
	.modulemutexes = BIT(NVMODMUTEX_VI_1),
	.exclusive     = true,
	.clocks = {{"vi.1", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = (1 << 16 | NVHOST_MODULE_VI),
},},
(struct nvhost_device []){{
	.name	       = "msenc",
	.resource      = msenc_resources,
	.num_resources = ARRAY_SIZE(msenc_resources),
	.syncpts       = BIT(NVSYNCPT_MSENC),
	.waitbases     = BIT(NVWAITBASE_MSENC),
	.class	       = NV_VIDEO_ENCODE_MSENC_CLASS_ID,
	.exclusive     = true,
	.keepalive     = true,
	.init          = nvhost_msenc_init,
	.deinit        = nvhost_msenc_deinit,
#define HOST_EMC_FLOOR 300000000
	.clocks = {{"msenc", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_MSENC,
},},
(struct nvhost_device []){{
	.name	       = "dsi",
	.syncpts       = BIT(NVSYNCPT_DSI),
	.modulemutexes = BIT(NVMODMUTEX_DSI),
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_NONE,
},},
(struct nvhost_device []){{
	.name          = "tsec",
	.resource      = tsec_resources,
	.num_resources = ARRAY_SIZE(tsec_resources),
	.waitbases     = BIT(NVWAITBASE_TSEC),
	.class         = NV_TSEC_CLASS_ID,
	.exclusive     = true,
	.init          = nvhost_tsec_init,
	.deinit        = nvhost_tsec_deinit,
	.clocks = {{"tsec", UINT_MAX}, {"emc", HOST_EMC_FLOOR} },
	NVHOST_MODULE_NO_POWERGATE_IDS,
	NVHOST_DEFAULT_CLOCKGATE_DELAY,
	.moduleid      = NVHOST_MODULE_TSEC,
},},
     &gk20a_device,
#ifdef CONFIG_ARCH_TEGRA_VIC
    &vic03_device,
#endif
};

int tegra12_register_host1x_devices(void)
{
	nvhost_dbg_fn("");
	return nvhost_add_devices(channel_devices, ARRAY_SIZE(channel_devices));
}

static inline void __iomem *t124_channel_aperture(void __iomem *p, int ndx)
{
	return p;
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
	if (job->ch->dev == &gk20a_device)
		return gk20a_channel_submit(job);
	else
		return host1x_channel_submit(job);
}

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
	nvhost_dbg_fn("");
	return gk20a_submit_channel_gpfifo(hwctx->priv, gpfifo, num_entries,
					fence, flags);
}

static int t124_channel_map_buffer(struct nvhost_hwctx *hwctx,
				    struct nvhost_map_buffer_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_map_buffer(hwctx->priv, args);
}

static int t124_channel_unmap_buffer(struct nvhost_hwctx *hwctx,
				    struct nvhost_unmap_buffer_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_unmap_buffer(hwctx->priv, args);
}

static int t124_channel_wait(struct nvhost_hwctx *hwctx,
			    struct nvhost_wait_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_wait(hwctx->priv, args);
}

static int t124_channel_read_3d_reg(struct nvhost_channel *channel,
			struct nvhost_hwctx *hwctx,
			u32 offset,
			u32 *value)
{
	return -EPERM;
}

static int t124_channel_zcull_get_size(struct nvhost_hwctx *hwctx,
			    struct nvhost_zcull_get_size_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_zcull_get_size(hwctx->priv, args);
}

static int t124_channel_zcull_bind(struct nvhost_hwctx *hwctx,
			    struct nvhost_zcull_bind_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_zcull_bind(hwctx->priv, args);
}

static int t124_channel_zcull_get_info(struct nvhost_hwctx *hwctx,
			    struct nvhost_zcull_get_info_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_zcull_get_info(hwctx->priv, args);
}

static int t124_channel_zbc_set_table(struct nvhost_hwctx *hwctx,
				struct nvhost_zbc_set_table_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_zbc_set_table(hwctx->priv, args);
}

static int t124_channel_zbc_query_table(struct nvhost_hwctx *hwctx,
				struct nvhost_zbc_query_table_args *args)
{
	nvhost_dbg_fn("");
	return gk20a_channel_zbc_query_table(hwctx->priv, args);
}

static void t124_free_nvhost_channel(struct nvhost_channel *ch)
{
	nvhost_dbg_fn("");
	nvhost_free_channel_internal(ch, &t124_num_alloc_channels);
}

static struct nvhost_channel *t124_alloc_nvhost_channel(
		struct nvhost_device *dev)
{
	nvhost_dbg_fn("");
	return nvhost_alloc_channel_internal(dev->index,
		nvhost_get_host(dev)->info.nb_channels,
		&t124_num_alloc_channels);
}

int nvhost_init_t124_channel_support(struct nvhost_master *host,
       struct nvhost_chip_support *op)
{
	int i, nb_channels;
	nvhost_dbg_fn("max channels=%d devices=%d",
		      NV_HOST1X_CHANNELS,
		      ARRAY_SIZE(channel_devices));
	BUILD_BUG_ON(T124_NVHOST_NUMCHANNELS < ARRAY_SIZE(channel_devices));

	nb_channels =  ARRAY_SIZE(channel_devices);

	/* Set indices dynamically as we can have
	 * missing/non-static devices above (e.g.: vic, gk20a).
	 */

	for (i = 0; i < nb_channels; i++ ) {
		struct nvhost_device *dev = channel_devices[i];

		dev->index = i;

		if (dev == &vic03_device) {
			dev->modulemutexes = BIT(NVMODMUTEX_VIC);
			dev->syncpts = BIT(NVSYNCPT_VIC);
		}
		if (dev == &gk20a_device) {
			dev->syncpts       = BIT(NVSYNCPT_3D);
			dev->waitbases     = BIT(NVWAITBASE_3D);
			dev->modulemutexes = BIT(NVMODMUTEX_3D);
		}
	}

	op->channel.init          = t124_channel_init;
	op->channel.submit        = t124_channel_submit;
	op->channel.alloc_obj     = t124_channel_alloc_obj;
	op->channel.free_obj      = t124_channel_free_obj;
	op->channel.alloc_gpfifo  = t124_channel_alloc_gpfifo;
	op->channel.submit_gpfifo = t124_channel_submit_gpfifo;
	op->channel.map_buffer    = t124_channel_map_buffer;
	op->channel.unmap_buffer  = t124_channel_unmap_buffer;
	op->channel.wait          = t124_channel_wait;
	op->channel.read3dreg     = t124_channel_read_3d_reg;

	op->channel.zcull.get_size = t124_channel_zcull_get_size;
	op->channel.zcull.bind     = t124_channel_zcull_bind;
	op->channel.zcull.get_info = t124_channel_zcull_get_info;

	op->channel.zbc.set_table   = t124_channel_zbc_set_table;
	op->channel.zbc.query_table = t124_channel_zbc_query_table;

	op->nvhost_dev.alloc_nvhost_channel = t124_alloc_nvhost_channel;
	op->nvhost_dev.free_nvhost_channel = t124_free_nvhost_channel;

	return 0;
}

int t124_nvhost_hwctx_handler_init(struct nvhost_channel *ch)
{
	int err = 0;
	unsigned long syncpts = ch->dev->syncpts;
	unsigned long waitbases = ch->dev->waitbases;
	u32 syncpt = find_first_bit(&syncpts, BITS_PER_LONG);
	u32 waitbase = find_first_bit(&waitbases, BITS_PER_LONG);

	nvhost_dbg_fn("");

	if (ch->dev->alloc_hwctx_handler) {
		ch->ctxhandler = ch->dev->alloc_hwctx_handler(syncpt,
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

	err = nvhost_init_t124_syncpt_support(host, op);
	if (err)
		return err;

	err = nvhost_init_t124_intr_support(op);
	if (err)
		return err;
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
