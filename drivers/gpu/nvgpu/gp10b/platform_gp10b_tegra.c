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
#include <linux/reset.h>
#include <soc/tegra/tegra_bpmp.h>
#include "gk20a/platform_gk20a.h"
#include "gk20a/gk20a.h"
#include "platform_tegra.h"

static struct {
	char *name;
	unsigned long default_rate;
} tegra_gp10b_clocks[] = {
	{"gpu", 1900000000},
	{"gpu_sys", 204000000} };

/*
 * gp10b_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gp10b platform data.
 */

static int gp10b_tegra_get_clocks(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a *g = get_gk20a(pdev);
	struct device *dev = dev_from_gk20a(g);
	int i;

	if (tegra_platform_is_linsim())
		return 0;

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gp10b_clocks); i++) {
		long rate = tegra_gp10b_clocks[i].default_rate;
		struct clk *c;

		c = clk_get(dev, tegra_gp10b_clocks[i].name);
		if (IS_ERR(c)) {
			gk20a_err(&pdev->dev, "cannot get clock %s",
					tegra_gp10b_clocks[i].name);
		} else {
			clk_set_rate(c, rate);
			platform->clk[i] = c;
		}
	}
	platform->num_clks = i;

	return 0;
}

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
	platform->bypass_smmu = 1;
	platform->disable_bigpage = platform->bypass_smmu;

	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;
	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;

	platform->g->gr.t18x.ctx_vars.force_preemption_gfxp = false;
	platform->g->gr.t18x.ctx_vars.force_preemption_cilp = false;

	platform->g->gr.t18x.ctx_vars.debugfs_force_preemption_gfxp =
		debugfs_create_bool("force_preemption_gfxp", S_IRUGO|S_IWUSR,
			platform->debugfs,
			&platform->g->gr.t18x.ctx_vars.force_preemption_gfxp);

	platform->g->gr.t18x.ctx_vars.debugfs_force_preemption_cilp =
		debugfs_create_bool("force_preemption_cilp", S_IRUGO|S_IWUSR,
			platform->debugfs,
			&platform->g->gr.t18x.ctx_vars.force_preemption_cilp);

	platform->g->gr.t18x.ctx_vars.debugfs_dump_ctxsw_stats =
		debugfs_create_bool("dump_ctxsw_stats_on_channel_close",
			S_IRUGO|S_IWUSR,
			platform->debugfs,
			&platform->g->gr.t18x.
				ctx_vars.dump_ctxsw_stats_on_channel_close);

	gp10b_tegra_get_clocks(pdev);

	return 0;
}

static int gp10b_tegra_late_probe(struct platform_device *pdev)
{
	return 0;
}

static bool gp10b_tegra_is_railgated(struct platform_device *pdev)
{
	bool ret = false;

	if (tegra_bpmp_running())
		ret = !tegra_powergate_is_powered(TEGRA_POWERGATE_GPU);

	return ret;
}

static int gp10b_tegra_railgate(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);

	if (tegra_bpmp_running() &&
	    tegra_powergate_is_powered(TEGRA_POWERGATE_GPU)) {
		int i;
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_disable_unprepare(platform->clk[i]);
		}
		tegra_powergate_partition(TEGRA_POWERGATE_GPU);
	}
	return 0;
}

static int gp10b_tegra_unrailgate(struct platform_device *pdev)
{
	int ret = 0;
	struct gk20a_platform *platform = gk20a_get_platform(pdev);

	if (tegra_bpmp_running()) {
		int i;
		ret = tegra_unpowergate_partition(TEGRA_POWERGATE_GPU);
		for (i = 0; i < platform->num_clks; i++) {
			if (platform->clk[i])
				clk_prepare_enable(platform->clk[i]);
		}
	}
	return ret;
}

static int gp10b_tegra_suspend(struct device *dev)
{
	return 0;
}

static int gp10b_tegra_reset_assert(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int ret = 0;

	if (!platform->reset_control)
		return -EINVAL;

	ret = reset_control_assert(platform->reset_control);

	return ret;
}

static int gp10b_tegra_reset_deassert(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int ret = 0;

	if (!platform->reset_control)
		return -EINVAL;

	ret = reset_control_deassert(platform->reset_control);

	return ret;
}

struct gk20a_platform t18x_gpu_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,

	/* power management configuration */
	.can_railgate           = false,
	.enable_elpg            = false,

	/* ptimer scaling constant */
	.ptimerscaling10x	= 10,

	.ch_wdt_timeout_ms = 5000,

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

	.secure_alloc = gk20a_tegra_secure_alloc,
	.secure_page_alloc = gk20a_tegra_secure_page_alloc,

	.reset_assert = gp10b_tegra_reset_assert,
	.reset_deassert = gp10b_tegra_reset_deassert,

	.force_reset_in_do_idle = true,
};
