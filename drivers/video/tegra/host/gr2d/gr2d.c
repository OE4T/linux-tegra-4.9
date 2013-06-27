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
#include "t148/t148.h"

static struct of_device_id tegra_gr2d_of_match[] = {
#ifdef TEGRA_2X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra20-gr2d",
		.data = (struct nvhost_device_data *)&t20_gr2d_info },
#endif
#ifdef TEGRA_3X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra30-gr2d",
		.data = (struct nvhost_device_data *)&t30_gr2d_info },
#endif
#ifdef TEGRA_11X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra114-gr2d",
		.data = (struct nvhost_device_data *)&t11_gr2d_info },
#endif
#ifdef TEGRA_14X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra148-gr2d",
		.data = (struct nvhost_device_data *)&t14_gr2d_info },
#endif
	{ },
};

#ifdef CONFIG_PM_GENERIC_DOMAINS
static int gr2d_unpowergate(struct generic_pm_domain *domain)
{
	struct nvhost_device_data *pdata;

	pdata = container_of(domain, struct nvhost_device_data, pd);
	return nvhost_module_power_on(pdata->pdev);
}

static int gr2d_powergate(struct generic_pm_domain *domain)
{
	struct nvhost_device_data *pdata;

	pdata = container_of(domain, struct nvhost_device_data, pd);
	return nvhost_module_power_off(pdata->pdev);
}
#endif

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

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "gr2d";
	pdata->pd.power_off = gr2d_powergate;
	pdata->pd.power_on = gr2d_unpowergate;
	pdata->pd.dev_ops.start = nvhost_module_enable_clk;
	pdata->pd.dev_ops.stop = nvhost_module_disable_clk;

	err = nvhost_module_add_domain(&pdata->pd, dev);

	/* overwrite save/restore fptrs set by pm_genpd_init */
	pdata->pd.domain.ops.suspend = nvhost_client_device_suspend;
	pdata->pd.domain.ops.resume = nvhost_client_device_resume;
	pdata->pd.dev_ops.restore_state = nvhost_module_finalize_poweron;
#endif

#ifdef CONFIG_PM_RUNTIME
	if (pdata->clockgate_delay) {
		pm_runtime_set_autosuspend_delay(&dev->dev,
			pdata->clockgate_delay);
		pm_runtime_use_autosuspend(&dev->dev);
	}
	pm_runtime_enable(&dev->dev);
	pm_runtime_get_sync(&dev->dev);
#else
	nvhost_module_enable_clk(&dev->dev);
#endif

	err = nvhost_client_device_init(dev);

#ifdef CONFIG_PM_RUNTIME
	if (pdata->clockgate_delay)
		pm_runtime_put_sync_autosuspend(&dev->dev);
	else
		pm_runtime_put(&dev->dev);
	if (err)
		return err;
#endif

	return 0;
}

static int __exit gr2d_remove(struct platform_device *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put(&dev->dev);
	pm_runtime_disable(&dev->dev);
#else
	nvhost_module_disable_clk(&dev->dev);
#endif
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
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
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
