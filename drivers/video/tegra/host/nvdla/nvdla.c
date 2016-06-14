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

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "flcn/flcn.h"
#include "t194/t194.h"
#include "nvhost_nvdla_ioctl.h"
#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"
#include "nvdla/nvdla.h"
#include "nvdla_ucode_interface.h"

#define DEBUG_BUFFER_SIZE 0x100
DEFINE_DMA_ATTRS(attrs);

/* data structure to keep device data */
struct nvdla {
	struct platform_device *pdev;
};

int nvhost_nvdla_flcn_isr(struct platform_device *pdev)
{
	struct flcn *m = get_flcn(pdev);
	uint32_t mailbox0;

	/* dump falcon data if debug enabled */
	mailbox0 = host1x_readl(pdev, flcn_mailbox0_r());
	if (mailbox0 == DLA_DEBUG_PRINT)
		dev_info(&pdev->dev, "falcon: %s\n",
			 (char *)m->debug_dump_va);

	return 0;
}

/* Helper API's */
static void nvdla_send_cmd(struct platform_device *pdev,
		   uint32_t method_id, uint32_t method_data)
{
	host1x_writel(pdev, NV_DLA_THI_METHOD_ID, method_id);
	host1x_writel(pdev, NV_DLA_THI_METHOD_DATA, method_data);
}

static int nvdla_alloc_dump_region(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct flcn *m;

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

	/* pass dump region to falcon */
	nvdla_send_cmd(pdev, DLA_CMD_SET_REGIONS,
		       (m->debug_dump_pa >> 8) & 0xffffffff);

	return 0;
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

		/* reset dump region */
		nvdla_send_cmd(pdev, DLA_CMD_SET_REGIONS, m->debug_dump_pa);
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

/* IOCTL API's */
struct nvdla_private {
	struct platform_device *pdev;
};

static long nvdla_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct nvdla_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;

	nvhost_dbg_fn("pdev:%p priv:%p", pdev, priv);

	return -ENOIOCTLCMD;
}

static int nvdla_open(struct inode *inode, struct file *file)
{
	struct nvdla_private *priv;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL))
		return -ENOMEM;

	file->private_data = priv;

	return nonseekable_open(inode, file);
}

static int nvdla_release(struct inode *inode, struct file *file)
{
	struct nvdla_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;

	nvhost_dbg_fn("pdev:%p priv:%p", pdev, priv);

	kfree(priv);
	return 0;
}

const struct file_operations tegra_nvdla_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvdla_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvdla_ioctl,
#endif
	.open = nvdla_open,
	.release = nvdla_release,
};

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

static int nvdla_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;
	struct nvdla *nvdla = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvdla_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else {
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;
	}

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		err = -ENODATA;
		goto err_get_pdata;
	}

	nvhost_dbg_fn("dev:%p pdata:%p", dev, pdata);

	nvdla = devm_kzalloc(&dev->dev, sizeof(*nvdla), GFP_KERNEL);
	if (!nvdla) {
		err = -ENOMEM;
		goto err_alloc_nvdla;
	}

	nvdla->pdev = dev;
	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto err_get_resources;

	err = nvhost_module_init(dev);
	if (err)
		goto err_module_init;

#ifdef CONFIG_PM_GENERIC_DOMAINS
	err = nvhost_module_add_domain(&pdata->pd, dev);
	if (err)
		goto err_add_domain;
#endif

	err = nvhost_client_device_init(dev);
	if (err)
		goto err_client_device_init;

	if (pdata->flcn_isr)
		flcn_intr_init(dev);

	return 0;

err_client_device_init:
#ifdef CONFIG_PM_GENERIC_DOMAINS
err_add_domain:
#endif
	nvhost_module_deinit(dev);
err_module_init:
err_get_resources:
err_alloc_nvdla:
err_get_pdata:

	return err;
}

static int __exit nvdla_remove(struct platform_device *dev)
{
	nvhost_client_device_release(dev);

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
