/*
 * NVDLA driver for T194
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_buffer.h"
#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"

#include "t194/t194.h"
#include "nvhost_queue.h"

#include "nvdla/nvdla.h"
#include "nvdla/nvdla_debug.h"
#include <linux/nvhost_nvdla_ioctl.h>
#include "dla_os_interface.h"

/**
 * Maximum buffer size for debug dump
 */
#define DEBUG_BUFFER_SIZE 0x100

/**
 * default falcon idle timeout
 */
#define FLCN_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */

#define ALIGNED_DMA(x) ((x >> 8) & 0xffffffff)

static DEFINE_DMA_ATTRS(attrs);

int nvhost_nvdla_flcn_isr(struct platform_device *pdev)
{
	struct flcn *m = get_flcn(pdev);
	uint32_t mailbox0;

	/* dump falcon data if debug enabled */
	mailbox0 = host1x_readl(pdev, flcn_mailbox0_r());
	if (mailbox0 == DLA_DEBUG_PRINT)
		dev_info(&pdev->dev, "falcon: %s",
			 (char *)m->debug_dump_va);

	return 0;
}

/* Helper API's */
void nvdla_send_cmd(struct platform_device *pdev,
		   uint32_t method_id, uint32_t method_data)
{
	host1x_writel(pdev, NV_DLA_THI_METHOD_ID, method_id);
	host1x_writel(pdev, NV_DLA_THI_METHOD_DATA, method_data);
}

static int nvdla_alloc_dump_region(struct platform_device *pdev)
{
	int err = 0;
	struct flcn *m;
	dma_addr_t region_pa;
	struct dla_region_printf *region;
	u32 timeout = FLCN_IDLE_TIMEOUT_DEFAULT * 5;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!pdata->flcn_isr)
		return 0;

	m = get_flcn(pdev);
	/* allocate dump region */
	m->debug_dump_va = dma_alloc_attrs(&pdev->dev,
				   DEBUG_BUFFER_SIZE, &m->debug_dump_pa,
				   GFP_KERNEL, &attrs);
	if (!m->debug_dump_va) {
		dev_err(&pdev->dev, "dma memory allocation failed");
		return -ENOMEM;
	}

	/* allocate memory for command */
	region = (struct dla_region_printf *)dma_alloc_attrs(&pdev->dev,
					sizeof(struct dla_region_printf),
					&region_pa, GFP_KERNEL, &attrs);
	if (!region) {
		dev_err(&pdev->dev, "dma memory allocation failed");
		err = -ENOMEM;
		goto set_region_failed;
	}

	region->region = DLA_REGION_PRINTF;
	region->address = ALIGNED_DMA(m->debug_dump_pa);
	region->size = DEBUG_BUFFER_SIZE;

	/* pass dump region to falcon */
	nvdla_send_cmd(pdev, DLA_CMD_SET_REGIONS,
			       ALIGNED_DMA(region_pa));

	/* wait for falcon to idle */
	err = flcn_wait_idle(pdev, &timeout);
	if (err != 0)
		dev_err(&pdev->dev, "failed for wait for idle in timeout");

	/* free memory allocated for command */
	dma_free_attrs(&pdev->dev, sizeof(struct dla_region_printf),
		       region, region_pa,
		       &attrs);

	return 0;

set_region_failed:
	dma_free_attrs(&pdev->dev, DEBUG_BUFFER_SIZE,
		       m->debug_dump_va, m->debug_dump_pa,
		       &attrs);

	return err;
}

static void nvdla_free_dump_region(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct flcn *m;

	if (!pdata->flcn_isr)
		return;

	m = get_flcn(pdev);
	if (m->debug_dump_pa) {
		dma_free_attrs(&pdev->dev, DEBUG_BUFFER_SIZE,
			       m->debug_dump_va, m->debug_dump_pa,
			       &attrs);
		m->debug_dump_va = NULL;
		m->debug_dump_pa = 0;
	}
}

/* power management API */
int nvhost_nvdla_finalize_poweron(struct platform_device *pdev)
{
	int ret = 0;

	ret = nvhost_flcn_finalize_poweron(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to poweron\n", __func__);
		return ret;
	}

	ret = nvdla_alloc_dump_region(pdev);
	if (ret)
		nvhost_nvdla_prepare_poweroff(pdev);

	return ret;
}

int nvhost_nvdla_prepare_poweroff(struct platform_device *pdev)
{
	int ret;

	/* free dump region */
	nvdla_free_dump_region(pdev);

	ret = nvhost_flcn_prepare_poweroff(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to poweroff\n", __func__);
		return ret;
	}

	return 0;
}

/* driver probe and init */
static struct of_device_id tegra_nvdla_of_match[] = {
	{
		.name = "nvdla0",
		.compatible = "nvidia,tegra194-nvdla",
		.data = (struct nvhost_device_data *)&t19_nvdla0_info },
	{
		.name = "nvdla1",
		.compatible = "nvidia,tegra194-nvdla",
		.data = (struct nvhost_device_data *)&t19_nvdla1_info },
	{ },
};

#ifdef CONFIG_PM_GENERIC_DOMAINS
static struct of_device_id tegra_nvdla_domain_match[] = {
	{.compatible = "nvidia,tegra194-dla-pd",
	.data = (struct nvhost_device_data *)&t19_nvdla0_info},
	{},
};
#endif

static int nvdla_probe(struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;
	struct nvdla_device *nvdla_dev = NULL;
	struct device *dev = &pdev->dev;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvdla_of_match, dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else {
		pdata = (struct nvhost_device_data *)pdev->dev.platform_data;
	}

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(dev, "no platform data\n");
		err = -ENODATA;
		goto err_get_pdata;
	}

	nvdla_dev = devm_kzalloc(dev, sizeof(*nvdla_dev), GFP_KERNEL);
	if (!nvdla_dev) {
		err = -ENOMEM;
		goto err_alloc_nvdla;
	}

	nvdla_dev->pdev = pdev;
	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	pdata->private_data = nvdla_dev;
	platform_set_drvdata(pdev, pdata);

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto err_get_resources;

	err = nvhost_module_init(pdev);
	if (err)
		goto err_module_init;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	err = nvhost_module_add_domain(&pdata->pd, pdev);
	if (err)
		goto err_add_domain;
#endif

	err = nvhost_client_device_init(pdev);
	if (err)
		goto err_client_device_init;

	/* create debugfs entries */
	nvdla_debug_init(pdev);

	if (pdata->flcn_isr)
		flcn_intr_init(pdev);

	nvdla_dev->pool = nvhost_queue_init(pdev, &nvdla_queue_ops,
				MAX_NVDLA_QUEUE_COUNT);
	if (IS_ERR(nvdla_dev->pool)) {
		err = PTR_ERR(nvdla_dev->pool);
		goto err_queue_init;
	}

	nvdla_dbg_info(pdev, "%s: pdata:%p\n", __func__, pdata);

	return 0;

err_queue_init:
	nvhost_client_device_release(pdev);
err_client_device_init:
#ifdef CONFIG_PM_GENERIC_DOMAINS
err_add_domain:
#endif
	nvhost_module_deinit(pdev);
err_module_init:
err_get_resources:
	devm_kfree(dev, nvdla_dev);
err_alloc_nvdla:
err_get_pdata:

	return err;
}

static int __exit nvdla_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	nvhost_queue_deinit(nvdla_dev->pool);
	nvhost_client_device_release(pdev);

	return 0;
}

static struct platform_driver nvdla_driver = {
	.probe = nvdla_probe,
	.remove = __exit_p(nvdla_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvdla",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvdla_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

static int __init nvdla_init(void)
{
	int ret;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	ret = nvhost_domain_init(tegra_nvdla_domain_match);
	if (ret)
		return ret;
#endif

	return platform_driver_register(&nvdla_driver);
}

static void __exit nvdla_exit(void)
{
	platform_driver_unregister(&nvdla_driver);
}

module_init(nvdla_init);
module_exit(nvdla_exit);
MODULE_AUTHOR("Shridhar Rasal <srasal@nvidia.com>");
