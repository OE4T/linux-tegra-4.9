/*
 * drivers/video/tegra/host/dsi/dsi.c
 *
 * Tegra Graphics DSI
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

#include <linux/module.h>
#include "dev.h"
#include "bus_client.h"

static int dsi_probe(struct nvhost_device *dev)
{
	return nvhost_client_device_init(dev);
}

static int __exit dsi_remove(struct nvhost_device *dev)
{
	/* Add clean-up */
	return 0;
}

static int dsi_suspend(struct nvhost_device *dev, pm_message_t state)
{
	return nvhost_client_device_suspend(dev);
}

static int dsi_resume(struct nvhost_device *dev)
{
	dev_info(&dev->dev, "resuming\n");
	return 0;
}

struct nvhost_device *dsi_device;

static struct nvhost_driver dsi_driver = {
	.probe = dsi_probe,
	.remove = __exit_p(dsi_remove),
#ifdef CONFIG_PM
	.suspend = dsi_suspend,
	.resume = dsi_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "dsi",
	}
};

static int __init dsi_init(void)
{
	int err;

	dsi_device = nvhost_get_device("dsi");
	if (!dsi_device)
		return -ENXIO;

	err = nvhost_device_register(dsi_device);
	if (err)
		return err;

	return nvhost_driver_register(&dsi_driver);
}

static void __exit dsi_exit(void)
{
	nvhost_driver_unregister(&dsi_driver);
}

module_init(dsi_init);
module_exit(dsi_exit);
