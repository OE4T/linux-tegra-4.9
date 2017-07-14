/*
 * ISP5 driver for T194
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#include <asm/ioctls.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "isp5.h"
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"

struct host_isp5 {
	struct platform_device *pdev;
	struct platform_device *isp_thi;

	/* Debugfs */
	struct isp5_debug {
		struct debugfs_regset32 fwinfo;
	} debug;
};

static int isp5_init_debugfs(struct host_isp5 *isp5);
static void isp5_remove_debugfs(struct host_isp5 *isp5);

int isp5_finalize_poweron(struct platform_device *pdev)
{
	struct host_isp5 *isp5 = nvhost_get_private_data(pdev);
	int ret;

	if (isp5->isp_thi) {
		ret = nvhost_module_busy(isp5->isp_thi);
		if (ret != 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(isp5_finalize_poweron);

int isp5_prepare_poweroff(struct platform_device *pdev)
{
	struct host_isp5 *isp5 = nvhost_get_private_data(pdev);

	if (isp5->isp_thi) {
		nvhost_module_idle(isp5->isp_thi);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(isp5_prepare_poweroff);

static int isp5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	struct device_node *thi_np;
	struct platform_device *thi = NULL;
	struct host_isp5 *isp5;
	int err = 0;

	dev_info(dev, "probing\n");

	info = (void *)of_device_get_match_data(dev);
	if (unlikely(info == NULL)) {
		dev_WARN(dev, "no platform data\n");
		return -ENODATA;
	}

	thi_np = of_parse_phandle(dev->of_node, "nvidia,isp-falcon-device", 0);
	if (thi_np == NULL) {
		dev_WARN(dev, "missing %s handle\n",
			"nvidia,isp-falcon-device");
		return -ENODEV;
	}

	thi = of_find_device_by_node(thi_np);
	of_node_put(thi_np);

	if (thi == NULL)
		return -ENODEV;

	if (thi->dev.driver == NULL) {
		platform_device_put(thi);
		return -EPROBE_DEFER;
	}

	isp5 = devm_kzalloc(dev, sizeof(*isp5), GFP_KERNEL);
	if (!isp5)
		return -ENOMEM;

	isp5->isp_thi = thi;
	isp5->pdev = pdev;
	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);
	info->private_data = isp5;

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto put_thi;

	err = nvhost_module_init(pdev);
	if (err)
		goto put_thi;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto deinit;

	{
		struct nvhost_device_data *thi_info = platform_get_drvdata(thi);
		/* Steal isp-thi HW aperture */
		info->aperture[1] = thi_info->aperture[0];
	}

	isp5_init_debugfs(isp5);

	dev_info(dev, "probed\n");

	return 0;

deinit:
	nvhost_module_deinit(pdev);
put_thi:
	platform_device_put(thi);
	dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static int __exit isp5_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host_isp5 *isp5 = (struct host_isp5 *)pdata->private_data;

	isp5_remove_debugfs(isp5);
	platform_device_put(isp5->isp_thi);

	return 0;
}

static const struct of_device_id tegra_isp5_of_match[] = {
	{
		.compatible = "nvidia,tegra194-isp",
		.data = &t19_isp5_info,
	},
	{ },
};

static struct platform_driver isp5_driver = {
	.probe = isp5_probe,
	.remove = __exit_p(isp5_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra194-isp5",
#ifdef CONFIG_OF
		.of_match_table = tegra_isp5_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

module_platform_driver(isp5_driver);

/* === Debugfs ========================================================== */

static const struct debugfs_reg32 isp5_fwinfo_regs[] = {
	{ .name = "protocol_version", 0x100 },
	{ .name = "perforce_changelist", 0x104 },
	{ .name = "build_timestamp", 0x108 },
	{ .name = "channel_count", 0x200 },
	{ .name = "mem_write_format_support", 0x204 },
	{ .name = "mem_read_format_support", 0x208 },
};

static int isp5_init_debugfs(struct host_isp5 *isp5)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(isp5->pdev);
	struct dentry *dir = pdata->debugfs;
	struct isp5_debug *debug = &isp5->debug;

	debug->fwinfo.base = pdata->aperture[0];
	debug->fwinfo.regs = isp5_fwinfo_regs;
	debug->fwinfo.nregs = ARRAY_SIZE(isp5_fwinfo_regs);
	debugfs_create_regset32("fwinfo", S_IRUGO, dir, &debug->fwinfo);

	return 0;
}

static void isp5_remove_debugfs(struct host_isp5 *isp5)
{
}
