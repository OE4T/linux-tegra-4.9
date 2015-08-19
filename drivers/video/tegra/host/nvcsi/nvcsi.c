/*
 * NVCSI driver for T186
 *
 * Copyright (c) 2014-2015, NVIDIA Corporation.  All rights reserved.
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

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t186/t186.h"

static struct of_device_id tegra_nvcsi_of_match[] = {
	{ .compatible = "nvidia,tegra186-nvcsi",
		.data = (struct nvhost_device_data *)&t18_nvcsi_info },
	{ },
};

static int nvcsi_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvcsi_of_match, &dev->dev);
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

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_client_device_init(dev);

	return 0;
}

static int __exit nvcsi_remove(struct platform_device *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put(&dev->dev);
	pm_runtime_disable(&dev->dev);
#else
	nvhost_module_disable_clk(&dev->dev);
#endif
	return 0;
}

static struct platform_driver nvcsi_driver = {
	.probe = nvcsi_probe,
	.remove = __exit_p(nvcsi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvcsi",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvcsi_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

static struct of_device_id tegra_nvcsi_domain_match[] = {
	{.compatible = "nvidia,tegra186-ve-pd",
	.data = (struct nvhost_device_data *)&t18_nvcsi_info},
	{},
};

static int __init nvcsi_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_nvcsi_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&nvcsi_driver);
}

static void __exit nvcsi_exit(void)
{
	platform_driver_unregister(&nvcsi_driver);
}

module_init(nvcsi_init);
module_exit(nvcsi_exit);
