/*
 * PVA driver for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"
#include "pva.h"

/* Map PVA-A and PVA-B to respective configuration items in nvhost */
static struct of_device_id tegra_pva_of_match[] = {
	{
		.name = "pvaa",
		.compatible = "nvidia,tegra194-pva",
		.data = (struct nvhost_device_data *)&t19_pvaa_info },
	{
		.name = "pvab",
		.compatible = "nvidia,tegra194-pva",
		.data = (struct nvhost_device_data *)&t19_pvab_info },
	{ },
};

/**
 * struct pva - Driver private data, shared with all applications
 *
 * @mutex:	Mutex to ensure exclusive access to the mutable entries
 * @pdev:	Pointer to the PVA device
 */
struct pva {
	struct mutex mutex;
	struct platform_device *pdev;
};

int pva_finalize_poweron(struct platform_device *pdev)
{
	return 0;
}

int pva_prepare_poweroff(struct platform_device *pdev)
{
	/* TBD: Disable IRQ, etc. */
	return 0;
}

static int pva_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *pdata;
	const struct of_device_id *match;
	struct pva *pva;
	int err = 0;

	nvhost_dbg_fn("%s", __func__);

	match = of_match_device(tegra_pva_of_match, dev);
	pdata = (struct nvhost_device_data *)match->data;

	pva = devm_kzalloc(dev, sizeof(*pva), GFP_KERNEL);
	if (!pva) {
		err = -ENOMEM;
		goto err_alloc_pva;
	}

	/* Initialize PVA private data */
	mutex_init(&pva->mutex);
	pva->pdev = pdev;

	/* Initialize nvhost specific data */
	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	pdata->private_data = pva;
	platform_set_drvdata(pdev, pdata);

	/* Map MMIO range to kernel space */
	err = nvhost_client_device_get_resources(pdev);
	if (err < 0)
		goto err_get_resources;

	/* Get clocks */
	err = nvhost_module_init(pdev);
	if (err < 0)
		goto err_module_init;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	/* Initialize Linux power domain */
	err = nvhost_module_add_domain(&pdata->pd, pdev);
	if (err < 0)
		goto err_add_domain;
#endif

	/*
	 * Add this to nvhost device list, initialize scaling,
	 * setup memory management for the device, create dev nodes
	 */
	err = nvhost_client_device_init(pdev);
	if (err < 0)
		goto err_client_device_init;

	return 0;

err_client_device_init:
err_add_domain:
	nvhost_module_deinit(pdev);
err_module_init:
err_get_resources:
err_alloc_pva:

	return err;
}

static int __exit pva_remove(struct platform_device *pdev)
{
	nvhost_client_device_release(pdev);

	return 0;
}

static struct platform_driver pva_driver = {
	.probe = pva_probe,
	.remove = __exit_p(pva_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "pva",
#ifdef CONFIG_OF
		.of_match_table = tegra_pva_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

static struct of_device_id tegra_pva_domain_match[] = {
	{
		.compatible = "nvidia,tegra194-cv-pd",
		.data = (struct nvhost_device_data *)&t19_pvaa_info
	},
};

/* Register PVA power domains and driver */
static int __init pva_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_pva_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&pva_driver);
}

static void __exit pva_exit(void)
{
	platform_driver_unregister(&pva_driver);
}

module_init(pva_init);
module_exit(pva_exit);
