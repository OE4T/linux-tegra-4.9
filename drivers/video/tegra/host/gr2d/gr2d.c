/*
 * drivers/video/tegra/host/gr2d/gr2d.c
 *
 * Tegra Graphics 2D
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation.
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

#include <linux/export.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <mach/pm_domains.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "gr2d_t30.h"
#include "gr2d_t114.h"
#include "t20/t20.h"
#include "t30/t30.h"
#include "t114/t114.h"

static struct of_device_id tegra_gr2d_of_match[] = {
	{ .compatible = "nvidia,tegra20-gr2d",
		.data = (struct nvhost_device_data *)&t20_gr2d_info },
	{ .compatible = "nvidia,tegra30-gr2d",
		.data = (struct nvhost_device_data *)&t30_gr2d_info },
	{ .compatible = "nvidia,tegra114-gr2d",
		.data = (struct nvhost_device_data *)&t11_gr2d_info },
	{ },
};

struct gr2d_pm_domain {
	struct platform_device *dev;
	struct generic_pm_domain pd;
};

static int gr2d_unpowergate(struct generic_pm_domain *domain)
{
	struct gr2d_pm_domain *gr2d_pd;

	gr2d_pd = container_of(domain, struct gr2d_pm_domain, pd);
	return nvhost_module_power_on(gr2d_pd->dev);
}

static int gr2d_powergate(struct generic_pm_domain *domain)
{
	struct gr2d_pm_domain *gr2d_pd;

	gr2d_pd = container_of(domain, struct gr2d_pm_domain, pd);
	return nvhost_module_power_off(gr2d_pd->dev);
}

static int gr2d_enable_clock(struct device *dev)
{
	return nvhost_module_enable_clk(to_platform_device(dev));
}

static int gr2d_disable_clock(struct device *dev)
{
	return nvhost_module_disable_clk(to_platform_device(dev));
}

static int gr2d_restore_context(struct device *dev)
{
	struct platform_device *pdev;
	struct nvhost_device_data *pdata;

	pdev = to_platform_device(dev);
	if (!pdev)
		return -EINVAL;

	pdata = platform_get_drvdata(pdev);
	if (!pdata)
		return -EINVAL;

	if (pdata->finalize_poweron)
		pdata->finalize_poweron(pdev);

	return 0;
}

static int gr2d_suspend(struct device *dev)
{
	return nvhost_client_device_suspend(to_platform_device(dev));
}

static int gr2d_resume(struct device *dev)
{
	dev_info(dev, "resuming\n");
	return 0;
}

static struct gr2d_pm_domain gr2d_pd = {
	.pd = {
		.name = "gr2d",
		.power_off = gr2d_powergate,
		.power_on = gr2d_unpowergate,
		.dev_ops = {
			.start = gr2d_enable_clock,
			.stop = gr2d_disable_clock,
		},
	},
};

static int gr2d_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_gr2d_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);
	nvhost_module_init(dev);

	gr2d_pd.dev = dev;
	err = nvhost_module_add_domain(&gr2d_pd.pd, dev);

	/* overwrite save/restore fptrs set by pm_genpd_init */
	gr2d_pd.pd.domain.ops.suspend = gr2d_suspend;
	gr2d_pd.pd.domain.ops.resume = gr2d_resume;
	gr2d_pd.pd.dev_ops.restore_state = gr2d_restore_context;

	pm_runtime_set_autosuspend_delay(&dev->dev, pdata->clockgate_delay);
	pm_runtime_use_autosuspend(&dev->dev);
	pm_runtime_enable(&dev->dev);

	pm_runtime_get_sync(&dev->dev);
	err = nvhost_client_device_init(dev);
	pm_runtime_put(&dev->dev);
	if (err)
		return err;

	return 0;
}

static int __exit gr2d_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

static struct platform_driver gr2d_driver = {
	.probe = gr2d_probe,
	.remove = __exit_p(gr2d_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "gr2d",
#ifdef CONFIG_OF
		.of_match_table = tegra_gr2d_of_match,
#endif
	},
};

static int __init gr2d_init(void)
{
	return platform_driver_register(&gr2d_driver);
}

static void __exit gr2d_exit(void)
{
	platform_driver_unregister(&gr2d_driver);
}

module_init(gr2d_init);
module_exit(gr2d_exit);
