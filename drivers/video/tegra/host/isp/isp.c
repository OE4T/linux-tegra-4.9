/*
 * drivers/video/tegra/host/isp/isp.c
 *
 * Tegra Graphics ISP
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
#include <linux/resource.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <mach/pm_domains.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t20/t20.h"
#include "t30/t30.h"
#include "t114/t114.h"
#include "t148/t148.h"

static struct of_device_id tegra_isp_of_match[] = {
	{ .compatible = "nvidia,tegra20-isp",
		.data = (struct nvhost_device_data *)&t20_isp_info },
	{ .compatible = "nvidia,tegra30-isp",
		.data = (struct nvhost_device_data *)&t30_isp_info },
	{ .compatible = "nvidia,tegra114-isp",
		.data = (struct nvhost_device_data *)&t11_isp_info },
	{ .compatible = "nvidia,tegra148-isp",
		.data = (struct nvhost_device_data *)&t14_isp_info },
	{ },
};

static int isp_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_isp_of_match, &dev->dev);
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

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	err = nvhost_client_device_init(dev);
	if (err)
		return err;

	tegra_pd_add_device(&dev->dev);
	if (pdata->clockgate_delay) {
		pm_runtime_set_autosuspend_delay(&dev->dev,
			pdata->clockgate_delay);
		pm_runtime_use_autosuspend(&dev->dev);
	}
	pm_runtime_enable(&dev->dev);

	return 0;
}

static int __exit isp_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int isp_suspend(struct device *dev)
{
	return nvhost_client_device_suspend(to_platform_device(dev));
}

static int isp_resume(struct device *dev)
{
	dev_info(dev, "resuming\n");
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int isp_runtime_suspend(struct device *dev)
{
	return nvhost_module_disable_clk(to_platform_device(dev));
}

static int isp_runtime_resume(struct device *dev)
{
	return nvhost_module_enable_clk(to_platform_device(dev));
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops isp_pm_ops = {
	.suspend = isp_suspend,
	.resume = isp_resume,
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = isp_runtime_suspend,
	.runtime_resume = isp_runtime_resume,
#endif /* CONFIG_PM_RUNTIME */
};

#define ISP_PM_OPS	(&isp_pm_ops)

#else

#define ISP_PM_OPS	NULL

#endif

static struct platform_driver isp_driver = {
	.probe = isp_probe,
	.remove = __exit_p(isp_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "isp",
		.pm = ISP_PM_OPS,
#ifdef CONFIG_OF
		.of_match_table = tegra_isp_of_match,
#endif
	}
};

static int __init isp_init(void)
{
	return platform_driver_register(&isp_driver);
}

static void __exit isp_exit(void)
{
	platform_driver_unregister(&isp_driver);
}

module_init(isp_init);
module_exit(isp_exit);
