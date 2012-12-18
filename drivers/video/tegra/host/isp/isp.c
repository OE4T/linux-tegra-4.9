/*
 * drivers/video/tegra/host/isp/isp.c
 *
 * Tegra Graphics ISP
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

static int isp_probe(struct nvhost_device *dev,
	struct nvhost_device_id *id_table)
{
	int err = 0;

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	return nvhost_client_device_init(dev);
}

static int __exit isp_remove(struct nvhost_device *dev)
{
	/* Add clean-up */
	return 0;
}

static int isp_suspend(struct nvhost_device *dev, pm_message_t state)
{
	return nvhost_client_device_suspend(dev);
}

static int isp_resume(struct nvhost_device *dev)
{
	dev_info(&dev->dev, "resuming\n");
	return 0;
}

static struct resource isp_resources = {
	.name = "regs",
	.start = TEGRA_ISP_BASE,
	.end = TEGRA_ISP_BASE + TEGRA_ISP_SIZE - 1,
	.flags = IORESOURCE_MEM,
};

struct nvhost_device *isp_device;

static struct nvhost_driver isp_driver = {
	.probe = isp_probe,
	.remove = __exit_p(isp_remove),
#ifdef CONFIG_PM
	.suspend = isp_suspend,
	.resume = isp_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "isp",
	}
};

static int __init isp_init(void)
{
	int err;

	isp_device = nvhost_get_device("isp");
	if (!isp_device)
		return -ENXIO;

	isp_device->resource = &isp_resources;
	isp_device->num_resources = 1;
	err = nvhost_device_register(isp_device);
	if (err)
		return err;

	return nvhost_driver_register(&isp_driver);
}

static void __exit isp_exit(void)
{
	nvhost_driver_unregister(&isp_driver);
}

module_init(isp_init);
module_exit(isp_exit);
