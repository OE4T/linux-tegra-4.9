/*
* Tegra Host1x Virtualization client common driver
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
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/tegra-soc.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"

#include "vhost.h"
#include "t124/t124.h"

static int nvhost_vhost_client_finalize_poweron(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id tegra_client_of_match[] = {
#ifdef CONFIG_ARCH_TEGRA_VIC
	{ .compatible = "nvidia,tegra124-vhost-vic",
		.data = (struct nvhost_device_data *)&t124_vic_info },
#endif
#if defined(CONFIG_TEGRA_GRHOST_VI) || defined(CONFIG_TEGRA_GRHOST_VI_MODULE)
	{ .compatible = "nvidia,tegra124-vhost-vi",
		.data = (struct nvhost_device_data *)&t124_vi_info },
#endif
#ifdef CONFIG_TEGRA_GRHOST_ISP
	{ .compatible = "nvidia,tegra124-vhost-isp",
		.data = (struct nvhost_device_data *)&t124_isp_info },
#endif
	{ .compatible = "nvidia,tegra124-vhost-msenc",
		.data = (struct nvhost_device_data *)&t124_msenc_info },
	{ },
};

static int vhost_client_probe(struct platform_device *dev)
{
	int err;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_client_of_match, &dev->dev);

		if (!match)
			return -ENODEV;

		pdata = (struct nvhost_device_data *)match->data;

#ifdef CONFIG_TEGRA_GRHOST_ISP
		/* If ISP, need to differentiate ISP.0 from ISP.1 */
		if (!IS_ENABLED(CONFIG_ARCH_TEGRA_18x_SOC)) {
			int dev_id = 0;
			if (sscanf(dev->name, "%x.isp", &dev_id) == 1) {
				switch (tegra_get_chipid()) {
				case TEGRA_CHIPID_TEGRA12:
					if (dev_id == 0x54600000)
						pdata = &t124_isp_info;
					if (dev_id == 0x54680000)
						pdata = &t124_ispb_info;
					break;
				default:
					/* Only T124 is virtualized, for now */
					return -EINVAL;
				}
			}
		}
#endif
	}

	if (!pdata) {
		dev_err(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	pdata->virtual_dev = true;

	nvhost_dbg_fn("dev:%p pdata:%p", dev, pdata);

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	/* Disable power management when virtual */
	pdata->can_powergate = false;
	pdata->busy = NULL;
	pdata->idle = NULL;
	pdata->scaling_init = NULL;
	pdata->finalize_poweron = nvhost_vhost_client_finalize_poweron;
	pdata->poweron_reset = false;
	pdata->engine_cg_regs = NULL;
	pdata->keepalive = false;

	pdata->hw_init = NULL;

	dev->dev.platform_data = NULL;

	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = kstrdup(dev->name, GFP_KERNEL);
	if (!pdata->pd.name)
		return -ENOMEM;

	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_virt_init(dev, pdata->moduleid);
	if (err) {
		dev_err(&dev->dev, "nvhost_virt_init failed for %s",
			      dev->name);
		pm_runtime_put(&dev->dev);
		return err;
	}

	err = nvhost_client_device_init(dev);
	if (err) {
		dev_err(&dev->dev, "failed to init client device for %s",
			      dev->name);
		pm_runtime_put(&dev->dev);
		return err;
	}

	return 0;
}

static int __exit vhost_client_remove(struct platform_device *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put(&dev->dev);
	pm_runtime_disable(&dev->dev);
#endif
	return 0;
}

static struct platform_device_id client_id_table[] = {
	{ .name = "vic03" },
	{ .name = "vi" },
	{ .name = "isp" },
	{ .name = "msenc" },
	{},
};
static struct platform_driver client_driver = {
	.probe = vhost_client_probe,
	.remove = __exit_p(vhost_client_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "vhost-client",
#ifdef CONFIG_OF
		.of_match_table = tegra_client_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
	.id_table = client_id_table,
};

static int __init vhost_client_init(void)
{
	return platform_driver_register(&client_driver);
}

static void __exit vhost_client_exit(void)
{
	platform_driver_unregister(&client_driver);
}

module_init(vhost_client_init);
module_exit(vhost_client_exit);
