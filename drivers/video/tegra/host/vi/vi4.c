/*
 * VI driver for T186
 *
 * Copyright (c) 2015-2016 NVIDIA Corporation.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-soc.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "nvhost_acm.h"
#include "vi/vi_notify.h"
#include "vi/vi4.h"
#include "t186/t186.h"
#include <linux/nvhost_vi_ioctl.h>

/* NV host device */
void nvhost_vi4_reset(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	if (!IS_ERR(vi->vi_reset))
		reset_control_reset(vi->vi_reset);
	if (!IS_ERR(vi->vi_tsc_reset))
		reset_control_reset(vi->vi_tsc_reset);
}

static long nvhost_vi4_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct platform_device *pdev = file->private_data;

	switch (cmd) {
	case NVHOST_VI_IOCTL_SET_VI_CLK: {
		long rate;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (get_user(rate, (long __user *)arg))
			return -EFAULT;

		return nvhost_module_set_rate(pdev, file, rate, 0,
						NVHOST_CLOCK);
	}
	}

	return -ENOIOCTLCMD;
}

static int nvhost_vi4_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	int err;

	if ((file->f_flags & O_ACCMODE) != O_WRONLY)
		return -EACCES;

	err = nvhost_module_add_client(pdev, file);
	if (err)
		return err;

	file->private_data = pdev;
	return nonseekable_open(inode, file);
}

static int nvhost_vi4_release(struct inode *inode, struct file *file)
{
	struct platform_device *pdev = file->private_data;

	nvhost_module_remove_client(pdev, file);
	return 0;
}

const struct file_operations nvhost_vi4_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvhost_vi4_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvhost_vi4_ioctl,
#endif
	.open = nvhost_vi4_open,
	.release = nvhost_vi4_release,
};

/* Platform device */
static struct of_device_id tegra_vi4_of_match[] = {
	{ .compatible = "nvidia,tegra186-vi",
		.data = (struct nvhost_device_data *)&t18_vi_info },
	{ },
};

static int tegra_vi4_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvhost_device_data *pdata;
	struct nvhost_vi_dev *vi;
	int err;

	match = of_match_device(tegra_vi4_of_match, &pdev->dev);
	BUG_ON(match == NULL);

	pdata = (struct nvhost_device_data *)match->data;
	BUG_ON(pdata == NULL);

	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(pdev, pdata);

	vi = devm_kzalloc(&pdev->dev, sizeof(*vi), GFP_KERNEL);
	if (unlikely(vi == NULL))
		return -ENOMEM;

	vi->vi_reset = devm_reset_control_get(&pdev->dev, "vi");
	vi->vi_tsc_reset = devm_reset_control_get(&pdev->dev, "tsctnvi");
	vi->debug_dir = debugfs_create_dir("tegra_vi", NULL);

	nvhost_set_private_data(pdev, vi);
	err = nvhost_client_device_get_resources(pdev);
	if (err)
		return err;

	nvhost_module_init(pdev);
#ifdef CONFIG_PM_GENERIC_DOMAINS
	nvhost_module_add_domain(&pdata->pd, pdev);
#endif
	err = nvhost_client_device_init(pdev);
	if (err) {
		nvhost_module_deinit(pdev);
		return err;
	}

#ifdef CONFIG_TEGRA_VI_NOTIFY
	if (tegra_platform_is_unit_fpga())
		return 0;

	err = vi_notify_register(&nvhost_vi_notify_driver, &pdev->dev, 12);
	if (err) {
		nvhost_client_device_release(pdev);
		return err;
	}
#endif
	err = tegra_vi_media_controller_init(&vi->mc_vi, pdev);
	if (err) {
		nvhost_client_device_release(pdev);
		return err;
	}

	return 0;
}

static int __exit tegra_vi4_remove(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	debugfs_remove_recursive(vi->debug_dir);

#ifdef CONFIG_TEGRA_VI_NOTIFY
	if (!tegra_platform_is_unit_fpga())
		vi_notify_unregister(&nvhost_vi_notify_driver, &pdev->dev);
#endif

	tegra_vi_media_controller_cleanup(&vi->mc_vi);

	nvhost_client_device_release(pdev);
	/* ^ includes call to nvhost_module_deinit() */

#ifdef CONFIG_PM_GENERIC_DOMAINS
	tegra_pd_remove_device(&pdev->dev);
#endif
	return 0;
}

static struct platform_driver tegra_vi4_driver = {
	.probe = tegra_vi4_probe,
	.remove = __exit_p(tegra_vi4_remove),
	.driver = {
		.name = "tegra-vi4",
		.owner = THIS_MODULE,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef CONFIG_OF
		.of_match_table = tegra_vi4_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

static struct of_device_id tegra_vi4_domain_match[] = {
	{ .compatible = "nvidia,tegra186-ve-pd",
		.data = (struct nvhost_device_data *)&t18_vi_info },
	{ },
};

static int __init tegra_vi4_init(void)
{
	int err;

	err = nvhost_domain_init(tegra_vi4_domain_match);
	if (err)
		return err;

	return platform_driver_register(&tegra_vi4_driver);
}

static void __exit tegra_vi4_exit(void)
{
	platform_driver_unregister(&tegra_vi4_driver);
}

module_init(tegra_vi4_init);
module_exit(tegra_vi4_exit);
