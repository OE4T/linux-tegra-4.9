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

#include "dev.h"
#include "bus_client.h"
#include "gr2d_t114.h"

enum gr2d_ip_ver {
	gr2d_01 = 1,
	gr2d_02,
};

struct gr2d_desc {
	void (*finalize_poweron)(struct nvhost_device *dev);
};

static const struct gr2d_desc gr2d[] = {
	[gr2d_01] = {
		.finalize_poweron = NULL,
	},
	[gr2d_02] = {
		.finalize_poweron = nvhost_gr2d_t114_finalize_poweron,
	}
};

static struct nvhost_device_id gr2d_id[] = {
	{ "gr2d", gr2d_01 },
	{ "gr2d", gr2d_02 },
	{ },
};

MODULE_DEVICE_TABLE(nvhost, gr2d_id);

static int gr2d_probe(struct nvhost_device *dev,
	struct nvhost_device_id *id_table)
{
	int index = 0;
	struct nvhost_driver *drv = to_nvhost_driver(dev->dev.driver);

	index = id_table->version;

	drv->finalize_poweron = gr2d[index].finalize_poweron;

	return nvhost_client_device_init(dev);
}

static int __exit gr2d_remove(struct nvhost_device *dev)
{
	/* Add clean-up */
	return 0;
}

#ifdef CONFIG_PM
static int gr2d_suspend(struct nvhost_device *dev, pm_message_t state)
{
	return nvhost_client_device_suspend(dev);
}

static int gr2d_resume(struct nvhost_device *dev)
{
	dev_info(&dev->dev, "resuming\n");
	return 0;
}
#endif

static struct nvhost_driver gr2d_driver = {
	.probe = gr2d_probe,
	.remove = __exit_p(gr2d_remove),
#ifdef CONFIG_PM
	.suspend = gr2d_suspend,
	.resume = gr2d_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "gr2d",
	},
	.id_table = gr2d_id,
};

static int __init gr2d_init(void)
{
	return nvhost_driver_register(&gr2d_driver);
}

static void __exit gr2d_exit(void)
{
	nvhost_driver_unregister(&gr2d_driver);
}

module_init(gr2d_init);
module_exit(gr2d_exit);
