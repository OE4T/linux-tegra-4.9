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
#include "nvhost_syncpt_unit_interface.h"

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

#define CMD_TIMEOUT	500 * USEC_PER_SEC

static DEFINE_DMA_ATTRS(attrs);

int nvhost_nvdla_flcn_isr(struct platform_device *pdev)
{
	uint32_t message;
	uint32_t mailbox0;
	struct flcn *m = get_flcn(pdev);
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	/* dump falcon data if debug enabled */
	mailbox0 = host1x_readl(pdev, flcn_mailbox0_r());
	nvdla_dbg_reg(pdev, "mailbox0=[0x%x]", mailbox0);

	message = mailbox0 & DLA_RESPONSE_MSG_MASK;

	if (message == DLA_DEBUG_PRINT)
		nvdla_dbg_info(pdev, "falcon: %s", (char *)m->debug_dump_va);

	if ((message == DLA_CMD_COMPLETE ||
				message == DLA_CMD_ERROR) &&
				nvdla_dev->waiting) {
		nvdla_dev->cmd_status =
				(mailbox0 >> DLA_RESPONSE_ERROR_SHIFT) &
						DLA_RESPONSE_ERROR_MASK;
		nvdla_dev->waiting = 0;
		complete(&nvdla_dev->cmd_completion);
	}

	return 0;
}

/* Helper API's */
int nvdla_send_cmd(struct platform_device *pdev,
		   uint32_t method_id, uint32_t method_data, bool wait)
{
	unsigned long timeout;
	int ret = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	mutex_lock(&nvdla_dev->cmd_lock);

	/*
	 * enable notification for command completion or error if
	 * wait if required
	 */
	if (wait)
		method_id |= (1 << DLA_INT_ON_COMPLETE_SHIFT) |
					(1 << DLA_INT_ON_ERROR_SHIFT);

	nvdla_dev->waiting = 1;

	nvdla_dbg_reg(pdev, "method_id=[0x%x]", method_id);
	host1x_writel(pdev, NV_DLA_THI_METHOD_ID, method_id);

	nvdla_dbg_reg(pdev, "method_data=[0x%x]", method_data);
	host1x_writel(pdev, NV_DLA_THI_METHOD_DATA, method_data);

	if (!wait) {
		nvdla_dev->waiting = 0;
		mutex_unlock(&nvdla_dev->cmd_lock);
		return 0;
	}

	timeout = usecs_to_jiffies(CMD_TIMEOUT);
	if (!wait_for_completion_timeout(&nvdla_dev->cmd_completion, timeout)) {
		nvdla_dev->waiting = 0;
		mutex_unlock(&nvdla_dev->cmd_lock);
		return -ETIMEDOUT;
	}

	if (nvdla_dev->cmd_status != DLA_ERR_NONE) {
		nvdla_dbg_err(pdev, "Command %u failed\n", method_id);
		ret = -EINVAL;
	}

	/* Reset command status after use for next command */
	nvdla_dev->cmd_status = DLA_ERR_NONE;
	nvdla_dev->waiting = 0;

	mutex_unlock(&nvdla_dev->cmd_lock);

	return ret;
}

static int nvdla_alloc_trace_region(struct platform_device *pdev)
{
	int err = 0;
	struct flcn *m;
	dma_addr_t tregion_pa;
	struct dla_region_printf *trace_region = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!pdata->flcn_isr)
		return 0;

	nvdla_dbg_fn(pdev, "");

	m = get_flcn(pdev);
	if (!m)
		return -ENXIO;

	/* Trace buffer allocation must be done at once only. */
	if (!m->trace_dump_va) {
		/* allocate trace region */
		m->trace_dump_va = dma_alloc_attrs(&pdev->dev,
				   TRACE_BUFFER_SIZE, &m->trace_dump_pa,
				   GFP_KERNEL, &attrs);

		if (!m->trace_dump_va) {
			nvdla_dbg_err(pdev,
				"dma trace memory allocation failed");
			return -ENOMEM;
		}
	}

	/* allocate memory for trace command */
	trace_region = (struct dla_region_printf *)
			dma_alloc_attrs(&pdev->dev,
			sizeof(struct dla_region_printf),
			&tregion_pa, GFP_KERNEL, &attrs);
	if (!trace_region) {
		nvdla_dbg_err(pdev,
			"dma allocation failed for trace command.");
		err = -ENOMEM;
		goto alloc_trace_cmd_failed;
	}

	trace_region->region = DLA_REGION_TRACE;
	trace_region->address = m->trace_dump_pa;
	trace_region->size = TRACE_BUFFER_SIZE;

	err = nvdla_send_cmd(pdev, DLA_CMD_SET_REGIONS,
	       ALIGNED_DMA(tregion_pa), true);

	/* free memory allocated for trace command */
	dma_free_attrs(&pdev->dev, sizeof(struct dla_region_printf),
		trace_region, tregion_pa, &attrs);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send trace command");
		goto trace_send_cmd_failed;
	}

	return err;

trace_send_cmd_failed:
alloc_trace_cmd_failed:
	dma_free_attrs(&pdev->dev, TRACE_BUFFER_SIZE,
	       m->trace_dump_va, m->trace_dump_pa, &attrs);
	m->trace_dump_va = NULL;
	m->trace_dump_pa = 0;

	return err;
}

static int nvdla_alloc_dump_region(struct platform_device *pdev)
{
	int err = 0;
	struct flcn *m;
	dma_addr_t region_pa;
	struct dla_region_printf *region;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!pdata->flcn_isr)
		return 0;

	nvdla_dbg_fn(pdev, "");

	m = get_flcn(pdev);
	if (!m)
		return -ENXIO;

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
	err = nvdla_send_cmd(pdev, DLA_CMD_SET_REGIONS,
			       ALIGNED_DMA(region_pa), true);


	/* free memory allocated for debug print command */
	dma_free_attrs(&pdev->dev, sizeof(struct dla_region_printf),
	       region, region_pa, &attrs);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send printf command");
		goto region_send_cmd_failed;
	}

	return 0;

region_send_cmd_failed:
set_region_failed:
	dma_free_attrs(&pdev->dev, DEBUG_BUFFER_SIZE,
	       m->debug_dump_va, m->debug_dump_pa, &attrs);
	m->debug_dump_va = NULL;
	m->debug_dump_pa = 0;

	return err;
}

static void nvdla_free_dump_region(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct flcn *m;

	nvdla_dbg_fn(pdev, "");

	if (!pdata->flcn_isr)
		return;

	m = get_flcn(pdev);
	if (m && m->debug_dump_pa) {
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
	uint32_t fw_ver_read_bin;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;

	nvdla_dbg_fn(pdev, "");

	ret = nvhost_flcn_finalize_poweron(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to poweron\n", __func__);
		return ret;
	}

	fw_ver_read_bin = host1x_readl(pdev, NV_DLA_OS_VERSION);
	if (FIRMWARE_VERSION != fw_ver_read_bin) {
		nvdla_dbg_err(pdev,
		"Fw version of kernel [%u.%u.%u] doesn't match with actual version[%u.%u.%u]",
		(FIRMWARE_VERSION >> 16) & 0xff, (FIRMWARE_VERSION >> 8) & 0xff, FIRMWARE_VERSION & 0xff,
		(fw_ver_read_bin >> 16 ) & 0xff, (fw_ver_read_bin >> 8) & 0xff, fw_ver_read_bin & 0xff);

		return -EINVAL;
	}

	nvdla_dbg_info(pdev, "Fw version : [%u.%u.%u]\n",
		(fw_ver_read_bin >> 16) & 0xff,
		(fw_ver_read_bin >> 8) & 0xff,
		fw_ver_read_bin & 0xff);

	nvdla_dev->fw_version = fw_ver_read_bin;

	ret = nvdla_alloc_dump_region(pdev);
	if (ret)
		nvhost_nvdla_prepare_poweroff(pdev);

	ret = nvdla_alloc_trace_region(pdev);
	if (ret)
		nvhost_nvdla_prepare_poweroff(pdev);

	return ret;
}

int nvhost_nvdla_prepare_poweroff(struct platform_device *pdev)
{
	int ret;

	nvdla_dbg_fn(pdev, "");

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
	mutex_init(&nvdla_dev->cmd_lock);
	init_completion(&nvdla_dev->cmd_completion);
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
	err = nvhost_syncpt_unit_interface_init(pdev);
	if (err)
		goto err_mss_init;

	nvdla_dbg_info(pdev, "%s: pdata:%p\n", __func__, pdata);

	return 0;

err_mss_init:
	nvhost_queue_deinit(nvdla_dev->pool);
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
	struct flcn *m;

	nvhost_queue_deinit(nvdla_dev->pool);
	nvhost_client_device_release(pdev);

	m = get_flcn(pdev);
	if (!m)
		return -ENXIO;

	if (m->trace_dump_pa) {
		dma_free_attrs(&pdev->dev, TRACE_BUFFER_SIZE,
			       m->trace_dump_va, m->trace_dump_pa,
			       &attrs);
		m->trace_dump_va = NULL;
		m->trace_dump_pa = 0;
	}

	nvdla_dbg_fn(pdev, "");

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
