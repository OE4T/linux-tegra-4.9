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
#include "gk20a_scale.h"
#include "nvhost_acm.h"
#include "bus_client.h"
#include "class_ids.h"
#include "t124/syncpt_t124.h"
#include "gr3d/pod_scaling.h"
#include "../../../../../arch/arm/mach-tegra/iomap.h"
#include <linux/tegra-powergate.h>
#include <linux/nvhost_ioctl.h>

#include "gk20a.h"

#define TEGRA_GK20A_SIM_BASE 0x538F0000 /*tbd: get from iomap.h */
#define TEGRA_GK20A_SIM_SIZE 0x1000     /*tbd: this is a high-side guess */

static int gk20a_tegra_getchannel(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	return nvhost_getchannel(platform->nvhost.channel, false) == NULL
		? -ENOMEM : 0;
}
static void gk20a_tegra_putchannel(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	nvhost_putchannel(platform->nvhost.channel);
}

static int gk20a_tegra_probe(struct platform_device *dev)
{
	int err;
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct nvhost_device_data *pdata = &platform->nvhost;

	pdata->pdev = dev;
	mutex_init(&pdata->lock);

	/* Initialize clocks and power. */
	err = nvhost_module_init(dev);
	if (err)
		return err;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "gk20a";

	err = nvhost_module_add_domain(&pdata->pd, dev);
	if (err)
		goto fail;
#endif

	err = nvhost_client_device_init(dev);
	if (err) {
		nvhost_dbg_fn("failed to init client device for %s",
			      dev->name);
		goto fail;
	}

	platform->can_powergate = pdata->can_powergate;
	platform->debugfs = pdata->debugfs;

	return 0;

fail:
	nvhost_module_deinit(dev);
	pdata->pdev = NULL;
	return err;
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
};

struct gk20a_platform gk20a_tegra_platform = {
	.nvhost = {
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
		.can_powergate		= true,
		.alloc_hwctx_handler	= nvhost_gk20a_alloc_hwctx_handler,
		.as_ops			= &tegra_gk20a_as_ops,
		.moduleid		= NVHOST_MODULE_GPU,
		.init			= nvhost_gk20a_init,
		.deinit			= nvhost_gk20a_deinit,
		.alloc_hwctx_handler	= nvhost_gk20a_alloc_hwctx_handler,
		.prepare_poweroff	= nvhost_gk20a_prepare_poweroff,
		.finalize_poweron	= nvhost_gk20a_finalize_poweron,
#ifdef CONFIG_GK20A_DEVFREQ
		.busy			= nvhost_gk20a_scale_notify_busy,
		.idle			= nvhost_gk20a_scale_notify_idle,
		.scaling_init		= nvhost_gk20a_scale_init,
		.scaling_deinit		= nvhost_gk20a_scale_deinit,
		.suspend_ndev		= nvhost_scale3d_suspend,
		.devfreq_governor	= "nvhost_podgov",
		.scaling_post_cb	= nvhost_gk20a_scale_callback,
		.gpu_edp_device		= true,
#endif
	},
	.probe = gk20a_tegra_probe,
	.getchannel = gk20a_tegra_getchannel,
	.putchannel = gk20a_tegra_putchannel,
};

struct platform_device tegra_gk20a_device = {
	.name		= "gk20a",
	.resource	= gk20a_tegra_resources,
	.num_resources	= ARRAY_SIZE(gk20a_tegra_resources),
	.dev		= {
		.platform_data = &gk20a_tegra_platform,
	},
};
