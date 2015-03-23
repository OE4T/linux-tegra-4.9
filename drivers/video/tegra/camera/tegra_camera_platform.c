/*
 * drivers/video/tegra/camera/tegra_camera_platform.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/platform/tegra/mc.h>
#include "tegra_camera_platform.h"

#define CAMDEV_NAME "tegra_camera_ctrl"
static const struct of_device_id tegra_camera_of_ids[] = {
	{ .compatible = "nvidia, tegra-camera-platform" },
	{ },
};

static struct miscdevice tegra_camera_misc;

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	struct tegra_camera_info *info;
	struct miscdevice *mdev;
	int i, ret;

	mdev = file->private_data;
	info = dev_get_drvdata(mdev->parent);

	file->private_data = info;
	for (i = 0; i < NUM_CLKS; i++) {
		info->clks[i] = clk_get_sys(info->devname, clk_names[i]);
		if (IS_ERR(info->clks[i])) {
			dev_err(info->dev, "clk_get_sys failed for %s:%s\n",
				info->devname, clk_names[i]);
			goto err_get_clk;
		}

		ret = clk_prepare_enable(info->clks[i]);
		if (ret) {
			dev_err(info->dev, "Cannot enable %s\n", clk_names[i]);
			goto err_get_clk;
		}
	}
	return 0;

err_get_clk:

	for (; i >= 0; i--) {
		if (!IS_ERR_OR_NULL(info->clks[i])) {
			clk_disable_unprepare(info->clks[i]);
			clk_put(info->clks[i]);
		}
		info->clks[i] = NULL;
	}
	return -ENOENT;

}
static int tegra_camera_release(struct inode *inode, struct file *file)
{

	struct tegra_camera_info *info;
	int i;
	info = file->private_data;
	for (i = 0; i < NUM_CLKS; i++) {
		if (!IS_ERR_OR_NULL(info->clks[i])) {
			clk_disable_unprepare(info->clks[i]);
			clk_put(info->clks[i]);
		}
		info->clks[i] = NULL;
	}

	return 0;
}

static long tegra_camera_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret;
	struct bw_info kcopy;
	struct tegra_camera_info *info;
	info = file->private_data;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(TEGRA_CAMERA_IOCTL_SET_BW):
	{
		if (copy_from_user(&kcopy, (const void __user *)arg,
			sizeof(struct bw_info))) {
			dev_err(info->dev, "%s:Failed to get data from user\n",
				__func__);
			return -EFAULT;
		}
		if (kcopy.is_iso) {
			dev_err(info->dev, "%s: ISO bw not implemented\n",
				__func__);
		} else {

			dev_dbg(info->dev, "%s:set bw %llu\n",
				__func__, kcopy.bw);
			ret = clk_set_rate(info->clks[EMC],
				tegra_emc_bw_to_freq_req(kcopy.bw));
			if (ret)
				dev_err(info->dev, "%s:Failed to set bw\n",
					__func__);
		}
		break;
	}
	default:
		break;
	}
	return 0;
}

static const struct file_operations tegra_camera_ops = {
	.owner = THIS_MODULE,
	.open = tegra_camera_open,
	.unlocked_ioctl = tegra_camera_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tegra_camera_ioctl,
#endif
	.release = tegra_camera_release,
};

static int tegra_camera_probe(struct platform_device *pdev)
{
	int ret;
	struct tegra_camera_info *info;

	dev_info(&pdev->dev, "%s:camera_platform_driver probe\n", __func__);

	tegra_camera_misc.minor = MISC_DYNAMIC_MINOR;
	tegra_camera_misc.name = CAMDEV_NAME;
	tegra_camera_misc.fops = &tegra_camera_ops;
	tegra_camera_misc.parent = &pdev->dev;

	ret = misc_register(&tegra_camera_misc);
	if (ret) {
		dev_err(tegra_camera_misc.this_device, "register failed for %s\n",
			tegra_camera_misc.name);
		return ret;
	}
	info = devm_kzalloc(tegra_camera_misc.this_device,
		sizeof(struct tegra_camera_info), GFP_KERNEL);
	if (!info) {
		dev_err(tegra_camera_misc.this_device,
			"Can't allocate memory for %s\n",
			tegra_camera_misc.name);
		return -ENOMEM;
	}

	strcpy(info->devname, tegra_camera_misc.name);
	info->dev = tegra_camera_misc.this_device;
	platform_set_drvdata(pdev, info);
	return 0;

}
static int tegra_camera_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s:camera_platform_driver remove\n", __func__);
	return misc_deregister(&tegra_camera_misc);
}

static struct platform_driver tegra_camera_driver = {
	.probe = tegra_camera_probe,
	.remove = tegra_camera_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra_camera_platform",
		.of_match_table = tegra_camera_of_ids
	}
};
static int __init tegra_camera_init(void)
{
	return platform_driver_register(&tegra_camera_driver);
}
static void __exit tegra_camera_exit(void)
{
	platform_driver_unregister(&tegra_camera_driver);
}

module_init(tegra_camera_init);
module_exit(tegra_camera_exit);

