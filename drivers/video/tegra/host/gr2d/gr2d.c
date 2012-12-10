/*
 * drivers/video/tegra/host/gr2d/gr2d.c
 *
 * Tegra Graphics 2D
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include "dev.h"
#include "bus_client.h"
#include "gr2d_t30.h"
#include "gr2d_t114.h"

enum gr2d_ip_ver {
	gr2d_01 = 1,
	gr2d_02,
};

struct gr2d_desc {
	void (*finalize_poweron)(struct platform_device *dev);
};

static const struct gr2d_desc gr2d[] = {
	[gr2d_01] = {
		.finalize_poweron = nvhost_gr2d_t30_finalize_poweron,
	},
	[gr2d_02] = {
		.finalize_poweron = nvhost_gr2d_t114_finalize_poweron,
	},
};

static struct platform_device_id gr2d_id[] = {
	{ "gr2d01", gr2d_01 },
	{ "gr2d02", gr2d_02 },
	{ },
};

MODULE_DEVICE_TABLE(nvhost, gr2d_id);

static int gr2d_probe(struct platform_device *dev)
{
	int index = 0;
	int err = 0;
	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)dev->dev.platform_data;

	/* HACK: reset device name */
	dev_set_name(&dev->dev, "%s", "gr2d");

	index = (int)(platform_get_device_id(dev)->driver_data);
	WARN_ON(index > gr2d_02);

	pdata->pdev = dev;
	pdata->finalize_poweron = gr2d[index].finalize_poweron;

	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_init(dev);
	if (err)
		return err;

	pm_runtime_use_autosuspend(&dev->dev);
	pm_runtime_set_autosuspend_delay(&dev->dev, 100);
	pm_runtime_enable(&dev->dev);

	return 0;
}

static int __exit gr2d_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int gr2d_suspend(struct device *dev)
{
	return nvhost_client_device_suspend(to_platform_device(dev));
}

static int gr2d_resume(struct device *dev)
{
	dev_info(dev, "resuming\n");
	return 0;
}

static const struct dev_pm_ops gr2d_pm_ops = {
	.suspend = gr2d_suspend,
	.resume = gr2d_resume,
};

#define GR2D_PM_OPS	(&gr2d_pm_ops)

#else

#define GR2D_PM_OPS	NULL

#endif /* CONFIG_PM */

static struct platform_driver gr2d_driver = {
	.probe = gr2d_probe,
	.remove = __exit_p(gr2d_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "gr2d",
		.pm = GR2D_PM_OPS,
	},
	.id_table = gr2d_id,
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
