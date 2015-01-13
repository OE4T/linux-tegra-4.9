/*
 * drivers/video/tegra/host/gk20a/platform_gk20a_tegra.c
 *
 * GK20A Tegra Platform Interface
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/of_platform.h>
#include <linux/nvhost.h>
#include <linux/debugfs.h>
#include <linux/tegra-powergate.h>
#include <linux/platform_data/tegra_edp.h>
#include <uapi/linux/nvgpu.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <linux/tegra_pm_domains.h>
#include "gk20a/platform_gk20a.h"
#include "gk20a/gk20a.h"

static int gp10b_tegra_probe(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *host1x_node;
	struct platform_device *host1x_pdev;
	const __be32 *host1x_ptr;

	host1x_ptr = of_get_property(np, "nvidia,host1x", NULL);
	if (!host1x_ptr) {
		gk20a_err(&pdev->dev, "host1x device not available");
		return -ENOSYS;
	}

	host1x_node = of_find_node_by_phandle(be32_to_cpup(host1x_ptr));
	host1x_pdev = of_find_device_by_node(host1x_node);
	if (!host1x_pdev) {
		gk20a_err(&pdev->dev, "host1x device not available");
		return -ENOSYS;
	}

	platform->g->host1x_dev = host1x_pdev;

	return 0;
}

static int gp10b_tegra_late_probe(struct platform_device *pdev)
{
	return 0;
}

static bool gp10b_tegra_is_railgated(struct platform_device *pdev)
{
	return false;
}

static int gp10b_tegra_railgate(struct platform_device *pdev)
{
	return 0;
}

static int gp10b_tegra_unrailgate(struct platform_device *pdev)
{
	return 0;
}

static int gp10b_tegra_suspend(struct device *dev)
{
	return 0;
}

struct gk20a_platform t18x_gpu_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.enable_elpg            = false,

	.probe = gp10b_tegra_probe,
	.late_probe = gp10b_tegra_late_probe,

	/* power management callbacks */
	.suspend = gp10b_tegra_suspend,
	.railgate = gp10b_tegra_railgate,
	.unrailgate = gp10b_tegra_unrailgate,
	.is_railgated = gp10b_tegra_is_railgated,

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

	.default_big_page_size	= SZ_64K,

	.has_cde = true,
};
