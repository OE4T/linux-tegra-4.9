/*
 * GV11B Tegra Platform Interface
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/hashtable.h>
#include "gk20a/platform_gk20a.h"
#include "gk20a/gk20a.h"
#include "platform_tegra.h"
#include "gr_gv11b.h"
#include "hw_gr_gv11b.h"

/*
 * gv11b_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gv11b platform data.
 */

static int gv11b_tegra_get_clocks(struct device *dev)
{
	/* TODO */
	return 0;
}

static int gv11b_tegra_probe(struct device *dev)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	struct device_node *host1x_node;
	struct platform_device *host1x_pdev;
	const __be32 *host1x_ptr;

	host1x_ptr = of_get_property(np, "nvidia,host1x", NULL);
	if (!host1x_ptr) {
		gk20a_err(dev, "host1x device not available");
		return -ENOSYS;
	}

	host1x_node = of_find_node_by_phandle(be32_to_cpup(host1x_ptr));
	host1x_pdev = of_find_device_by_node(host1x_node);
	if (!host1x_pdev) {
		gk20a_err(dev, "host1x device not available");
		return -ENOSYS;
	}

	platform->g->host1x_dev = host1x_pdev;
	platform->bypass_smmu = !device_is_iommuable(dev);
	platform->disable_bigpage = platform->bypass_smmu;

	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;
	platform->g->gr.t18x.ctx_vars.dump_ctxsw_stats_on_channel_close
		= false;

	platform->g->gr.t18x.ctx_vars.force_preemption_gfxp = false;
	platform->g->gr.t18x.ctx_vars.force_preemption_cilp = false;


	gv11b_tegra_get_clocks(dev);

	return 0;
}

static int gv11b_tegra_late_probe(struct device *dev)
{
	/* Make gk20a power domain a subdomain of host1x */
	nvhost_register_client_domain(dev_to_genpd(dev));
	return 0;
}

static int gv11b_tegra_remove(struct device *dev)
{
	/* remove gk20a power subdomain from host1x */
	nvhost_unregister_client_domain(dev_to_genpd(dev));

	return 0;

}

static bool gv11b_tegra_is_railgated(struct device *dev)
{
	bool ret = false;

	return ret;
}

static int gv11b_tegra_railgate(struct device *dev)
{
	return 0;
}

static int gv11b_tegra_unrailgate(struct device *dev)
{
	int ret = 0;
	return ret;
}

static int gv11b_tegra_suspend(struct device *dev)
{
	return 0;
}

struct gk20a_platform t19x_gpu_tegra_platform = {
	.has_syncpoints = false,

	/* power management configuration */

	/* ptimer src frequency in hz*/
	.ptimer_src_freq	= 31250000,

	.probe = gv11b_tegra_probe,
	.late_probe = gv11b_tegra_late_probe,
	.remove = gv11b_tegra_remove,

	/* power management callbacks */
	.suspend = gv11b_tegra_suspend,
	.railgate = gv11b_tegra_railgate,
	.unrailgate = gv11b_tegra_unrailgate,
	.is_railgated = gv11b_tegra_is_railgated,

	.busy = gk20a_tegra_busy,
	.idle = gk20a_tegra_idle,

	.dump_platform_dependencies = gk20a_tegra_debug_dump,

	.default_big_page_size	= SZ_64K,

};
