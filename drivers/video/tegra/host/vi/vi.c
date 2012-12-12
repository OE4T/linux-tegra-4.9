/*
 * drivers/video/tegra/host/vi/vi.c
 *
 * Tegra Graphics Host VI
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
#include <linux/resource.h>

#include <mach/iomap.h>

#include "dev.h"
#include "bus_client.h"

static int vi_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata =
		(struct nvhost_device_data *)dev->dev.platform_data;

	pdata->pdev = dev;
	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	return nvhost_client_device_init(dev);
}

static int __exit vi_remove(struct platform_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int vi_suspend(struct device *dev)
{
	return nvhost_client_device_suspend(to_platform_device(dev));
}

static int vi_resume(struct device *dev)
{
	dev_info(dev, "resuming\n");
	return 0;
}

static const struct dev_pm_ops vi_pm_ops = {
	.suspend = vi_suspend,
	.resume = vi_resume,
};

#define VI_PM_OPS	(&vi_pm_ops)

#else

#define VI_PM_OPS	NULL

#endif

static struct platform_driver vi_driver = {
	.probe = vi_probe,
	.remove = __exit_p(vi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "vi",
		.pm = VI_PM_OPS,
	}
};

static int __init vi_init(void)
{
	return platform_driver_register(&vi_driver);
}

static void __exit vi_exit(void)
{
	platform_driver_unregister(&vi_driver);
}

module_init(vi_init);
module_exit(vi_exit);
