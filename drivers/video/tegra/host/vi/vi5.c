/*
 * VI5 driver for T194
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
#include <media/capture_vi_channel.h>

#include "vi5.h"
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"

#include <media/vi.h>
#include <media/mc_common.h>
#include "camera/vi/vi5_fops.h"

struct host_vi5 {
	struct platform_device *pdev;
	struct platform_device *vi_thi;
	struct vi vi_common;

	/* Debugfs */
	struct vi5_debug {
		struct debugfs_regset32 ch0;
	} debug;
};

static int vi5_init_debugfs(struct host_vi5 *vi5);
static void vi5_remove_debugfs(struct host_vi5 *vi5);

int vi5_finalize_poweron(struct platform_device *pdev)
{
	struct host_vi5 *vi5 = nvhost_get_private_data(pdev);
	int ret;

	if (vi5->vi_thi) {
		ret = nvhost_module_busy(vi5->vi_thi);
		if (ret != 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(vi5_finalize_poweron);

int vi5_prepare_poweroff(struct platform_device *pdev)
{
	struct host_vi5 *vi5 = nvhost_get_private_data(pdev);

	if (vi5->vi_thi) {
		nvhost_module_idle(vi5->vi_thi);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(vi5_prepare_poweroff);

static int vi5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	struct device_node *thi_np;
	struct platform_device *thi = NULL;
	struct host_vi5 *vi5;
	int err = 0;

	dev_info(dev, "probing\n");

	info = (void *)of_device_get_match_data(dev);
	if (unlikely(info == NULL)) {
		dev_WARN(dev, "no platform data\n");
		return -ENODATA;
	}

	thi_np = of_parse_phandle(dev->of_node, "nvidia,vi-falcon-device", 0);
	if (thi_np == NULL) {
		dev_WARN(dev, "missing %s handle\n", "nvidia,vi-falcon-device");
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

	vi5 = devm_kzalloc(dev, sizeof(*vi5), GFP_KERNEL);
	if (!vi5)
		return -ENOMEM;

	vi5->vi_thi = thi;
	vi5->pdev = pdev;
	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);
	info->private_data = vi5;

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto put_vi;

	err = nvhost_module_init(pdev);
	if (err)
		goto put_vi;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto deinit;

	{
		struct nvhost_device_data *thi_info;

		thi_info = platform_get_drvdata(thi);
		/* Steal vi-thi HW aperture */
		info->aperture[1] = thi_info->aperture[0];
	}

	vi5_init_debugfs(vi5);

	vi5->vi_common.mc_vi.vi = &vi5->vi_common;
	vi5->vi_common.mc_vi.fops = &vi5_fops;
	err = tegra_vi_media_controller_init(&vi5->vi_common.mc_vi, pdev);
	if (err) {
		dev_err(dev, "media controller init failed\n");
		err = 0;
	}

	err = vi_channel_drv_register(pdev);
	if (err)
		goto device_release;

	dev_info(dev, "probed\n");

	return 0;

device_release:
	nvhost_client_device_release(pdev);
deinit:
	nvhost_module_deinit(pdev);
put_vi:
	platform_device_put(thi);
	dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static int __exit vi5_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host_vi5 *vi5 = (struct host_vi5 *)pdata->private_data;

	vi_channel_drv_unregister(&pdev->dev);
	tegra_vi_media_controller_cleanup(&vi5->vi_common.mc_vi);
	vi5_remove_debugfs(vi5);
	platform_device_put(vi5->vi_thi);

	return 0;
}

static const struct of_device_id tegra_vi5_of_match[] = {
	{
		.compatible = "nvidia,tegra194-vi",
		.data = &t19_vi5_info,
	},
	{ },
};

static struct platform_driver vi5_driver = {
	.probe = vi5_probe,
	.remove = __exit_p(vi5_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra194-vi5",
#ifdef CONFIG_OF
		.of_match_table = tegra_vi5_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

module_platform_driver(vi5_driver);

/* === Debugfs ========================================================== */

static const struct debugfs_reg32 vi5_ch_regs[] = {
	{ .name = "protocol_version", 0x00 },
	{ .name = "perforce_changelist", 0x4 },
	{ .name = "build_timestamp", 0x8 },
	{ .name = "channel_count", 0x80 },
};

static int vi5_init_debugfs(struct host_vi5 *vi5)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(vi5->pdev);
	struct dentry *dir = pdata->debugfs;
	struct vi5_debug *debug = &vi5->debug;

	debug->ch0.base = pdata->aperture[0];
	debug->ch0.regs = vi5_ch_regs;
	debug->ch0.nregs = ARRAY_SIZE(vi5_ch_regs);
	debugfs_create_regset32("ch0", S_IRUGO, dir, &debug->ch0);

	return 0;
}

static void vi5_remove_debugfs(struct host_vi5 *vi5)
{
}
