/*
 * drivers/video/tegra/host/gk20a/platform_gk20a_tegra.c
 *
 * GK20A Tegra Platform Interface
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include "platform_gk20a.h"
#include "gr3d/scale3d.h"
#include "gk20a_scale.h"
#include "nvhost_acm.h"
#include "bus_client.h"
#include "class_ids.h"
#include "t124/syncpt_t124.h"
#include "../../../../../arch/arm/mach-tegra/iomap.h"
#include <linux/tegra-powergate.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/nvhost_ioctl.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/pm_domains.h>

#include "gk20a.h"
#include "hal_gk20a.h"

#define TEGRA_GK20A_INTR		INT_GPU
#define TEGRA_GK20A_INTR_NONSTALL	INT_GPU_NONSTALL

#define TEGRA_GK20A_SIM_BASE 0x538F0000 /*tbd: get from iomap.h */
#define TEGRA_GK20A_SIM_SIZE 0x1000     /*tbd: this is a high-side guess */

struct gk20a_platform t132_gk20a_tegra_platform;

static int gk20a_tegra_channel_busy(struct platform_device *dev)
{
	int ret = 0;

	/* Explicitly turn on the host1x clocks
	 * - This is needed as host1x driver sets ignore_children = true
	 * to cater the use case of display clock ON but host1x clock OFF
	 * in OS-Idle-Display-ON case
	 * - This was easily done in ACM as it only checked the ref count
	 * of host1x (or any device for that matter) to be zero before
	 * turning off its clock
	 * - However, runtime PM checks to see if *ANY* child of device is
	 * in ACTIVE state and if yes, it doesn't suspend the parent. As a
	 * result of this, display && host1x clocks remains ON during
	 * OS-Idle-Display-ON case
	 * - The code below fixes this use-case
	 */
	if (nvhost_get_parent(dev))
		ret = nvhost_module_busy(nvhost_get_parent(dev));

	return ret;
}

static void gk20a_tegra_channel_idle(struct platform_device *dev)
{
	/* Explicitly turn off the host1x clocks */
	if (nvhost_get_parent(dev))
		nvhost_module_idle(nvhost_get_parent(dev));
}

#ifdef CONFIG_TEGRA_NVMAP
static void gk20a_tegra_secure_destroy(struct platform_device *pdev,
				       struct gr_ctx_buffer_desc *desc)
{
	struct dma_buf *dmabuf = desc->priv;

	gk20a_mm_unpin(&pdev->dev, dmabuf, desc->sgt);
	dma_buf_put(dmabuf);
}

static int gk20a_tegra_secure_alloc(struct platform_device *pdev,
				    struct gr_ctx_buffer_desc *desc,
				    size_t size)
{
	struct dma_buf *dmabuf;

	dmabuf = nvmap_alloc_dmabuf(size,
				    DEFAULT_ALLOC_ALIGNMENT,
				    NVMAP_HANDLE_UNCACHEABLE,
				    NVMAP_HEAP_CARVEOUT_VPR);
	if (!dmabuf)
		return -ENOMEM;
	if (gk20a_dmabuf_alloc_drvdata(dmabuf, &pdev->dev))
		return -ENOMEM;
	desc->sgt = gk20a_mm_pin(&pdev->dev, dmabuf);
	desc->size = size;
	desc->destroy = gk20a_tegra_secure_destroy;
	desc->priv = dmabuf;

	return 0;
}
#else
static int gk20a_tegra_secure_alloc(struct platform_device *pdev,
				    struct gr_ctx_buffer_desc *desc,
				    size_t size)
{
	return -ENOSYS;
}
#endif

/*
 * gk20a_tegra_railgate()
 *
 * Gate (disable) gk20a power rail
 */

static int gk20a_tegra_railgate(struct platform_device *pdev)
{
	if (tegra_powergate_is_powered(TEGRA_POWERGATE_GPU))
		tegra_powergate_partition(TEGRA_POWERGATE_GPU);
	return 0;
}

/*
 * gk20a_tegra_unrailgate()
 *
 * Ungate (enable) gk20a power rail
 */

static int gk20a_tegra_unrailgate(struct platform_device *pdev)
{
	tegra_unpowergate_partition(TEGRA_POWERGATE_GPU);
	return 0;
}

struct {
	char *name;
	unsigned long default_rate;
} tegra_gk20a_clocks[] = {
	{"PLLG_ref", UINT_MAX},
	{"pwr", 204000000},
	{"emc", UINT_MAX} };

/*
 * gk20a_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gk20a platform data.
 */

static int gk20a_tegra_get_clocks(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	char devname[16];
	int i;
	int ret = 0;

	snprintf(devname, sizeof(devname),
		 (pdev->id <= 0) ? "tegra_%s" : "tegra_%s.%d\n",
		 pdev->name, pdev->id);

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gk20a_clocks); i++) {
		long rate = tegra_gk20a_clocks[i].default_rate;
		struct clk *c;

		c = clk_get_sys(devname, tegra_gk20a_clocks[i].name);
		if (IS_ERR(c)) {
			ret = PTR_ERR(c);
			goto err_get_clock;
		}
		rate = clk_round_rate(c, rate);
		clk_set_rate(c, rate);
		platform->clk[platform->num_clks++] = c;
		platform->nvhost.clk[platform->nvhost.num_clks++] = c;
	}
	platform->num_clks = i;
	platform->nvhost.num_clks = i;

	return 0;

err_get_clock:

	while (i--)
		clk_put(platform->clk[i]);
	return ret;
}

static int gk20a_tegra_probe(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA13) {
		t132_gk20a_tegra_platform.g = platform->g;
		*platform = t132_gk20a_tegra_platform;
	}

	gk20a_tegra_get_clocks(dev);

	return 0;
}

static int gk20a_tegra_late_probe(struct platform_device *dev)
{
	int err;
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct nvhost_device_data *pdata = &platform->nvhost;

	/* Make gk20a power domain a subdomain of mc */
	tegra_pd_add_sd(&platform->g->pd);

	pdata->pdev = dev;
	mutex_init(&pdata->lock);

	err = nvhost_client_device_init(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client device for %s",
			      dev->name);
		goto fail;
	}

	platform->debugfs = pdata->debugfs;

	return 0;

fail:
	nvhost_module_deinit(dev);
	pdata->pdev = NULL;
	return err;
}

static int gk20a_tegra_suspend(struct device *dev)
{
	nvhost_scale3d_suspend(dev);
	tegra_edp_notify_gpu_load(0);

	return 0;
}

static struct resource gk20a_tegra_resources[] = {
	{
	.start = TEGRA_GK20A_BAR0_BASE,
	.end   = TEGRA_GK20A_BAR0_BASE + TEGRA_GK20A_BAR0_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{
	.start = TEGRA_GK20A_BAR1_BASE,
	.end   = TEGRA_GK20A_BAR1_BASE + TEGRA_GK20A_BAR1_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{ /* Used on ASIM only */
	.start = TEGRA_GK20A_SIM_BASE,
	.end   = TEGRA_GK20A_SIM_BASE + TEGRA_GK20A_SIM_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{
	.start = TEGRA_GK20A_INTR,
	.end   = TEGRA_GK20A_INTR,
	.flags = IORESOURCE_IRQ,
	},
	{
	.start = TEGRA_GK20A_INTR_NONSTALL,
	.end   = TEGRA_GK20A_INTR_NONSTALL,
	.flags = IORESOURCE_IRQ,
	},
};

struct gk20a_platform t132_gk20a_tegra_platform = {
	.nvhost = {
		.syncpts		= {NVSYNCPT_GK20A_BASE},
		.syncpt_base		= NVSYNCPT_GK20A_BASE,
		.class			= NV_GRAPHICS_GPU_CLASS_ID,
		.moduleid		= NVHOST_MODULE_GPU,
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
	},

	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,

	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,
	.railgate = gk20a_tegra_railgate,
	.unrailgate = gk20a_tegra_unrailgate,

	.channel_busy = gk20a_tegra_channel_busy,
	.channel_idle = gk20a_tegra_channel_idle,
	.secure_alloc = gk20a_tegra_secure_alloc,
};

struct gk20a_platform gk20a_tegra_platform = {
	.nvhost = {
		.syncpts		= {NVSYNCPT_GK20A_BASE},
		.syncpt_base		= NVSYNCPT_GK20A_BASE,
		.class			= NV_GRAPHICS_GPU_CLASS_ID,
		.moduleid		= NVHOST_MODULE_GPU,
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
	},

	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,
	.can_railgate		= true,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,

	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,
	.railgate = gk20a_tegra_railgate,
	.unrailgate = gk20a_tegra_unrailgate,

	.channel_busy = gk20a_tegra_channel_busy,
	.channel_idle = gk20a_tegra_channel_idle,
	.secure_alloc = gk20a_tegra_secure_alloc,
};

struct platform_device tegra_gk20a_device = {
	.name		= "gk20a",
	.resource	= gk20a_tegra_resources,
	.num_resources	= ARRAY_SIZE(gk20a_tegra_resources),
	.dev		= {
		.platform_data = &gk20a_tegra_platform,
	},
};
