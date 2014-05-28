/*
 * drivers/video/tegra/host/vi/vi.c
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/resource.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk/tegra.h>
#include <linux/tegra_pm_domains.h>

#include <media/tegra_v4l2_camera.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t124/t124.h"
#include "vi.h"
#include "vi_irq.h"

#define MAX_DEVID_LENGTH	16
#define TEGRA_VI_NAME		"tegra_vi"

static struct of_device_id tegra_vi_of_match[] = {
#ifdef TEGRA_12X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra124-vi",
		.data = (struct nvhost_device_data *)&t124_vi_info },
#endif
	{ },
};

static struct i2c_camera_ctrl *i2c_ctrl;

#if defined(CONFIG_TEGRA_ISOMGR)
static int vi_isomgr_unregister(struct vi *tegra_vi)
{
	tegra_isomgr_unregister(tegra_vi->isomgr_handle);
	tegra_vi->isomgr_handle = NULL;

	return 0;
}
#endif

static int vi_out_show(struct seq_file *s, void *unused)
{
	struct vi *vi = s->private;

	seq_printf(s, "vi[%d] overflow: %u\n", vi->ndev->id,
		atomic_read(&(vi->vi_out.overflow)));

	return 0;
}

static int vi_out_open(struct inode *inode, struct file *file)
{
	return single_open(file, vi_out_show, inode->i_private);
}

static const struct file_operations vi_out_fops = {
	.open		= vi_out_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void vi_remove_debugfs(struct vi *vi)
{
	debugfs_remove_recursive(vi->debugdir);
	vi->debugdir = NULL;
}

static void vi_create_debugfs(struct vi *vi)
{
	struct dentry *ret;
	char tegra_vi_name[20];
	char debugfs_file_name[20];


	snprintf(tegra_vi_name, sizeof(tegra_vi_name),
			"%s_%u", TEGRA_VI_NAME, vi->ndev->id);

	vi->debugdir = debugfs_create_dir(tegra_vi_name, NULL);
	if (!vi->debugdir) {
		dev_err(&vi->ndev->dev,
			"%s: failed to create %s directory",
			__func__, tegra_vi_name);
		goto create_debugfs_fail;
	}

	snprintf(debugfs_file_name, sizeof(debugfs_file_name),
			"%s_%u", "vi_out", vi->ndev->id);

	ret = debugfs_create_file(debugfs_file_name, S_IRUGO,
			vi->debugdir, vi, &vi_out_fops);
	if (!ret) {
		dev_err(&vi->ndev->dev,
		"%s: failed to create %s", __func__, debugfs_file_name);
		goto create_debugfs_fail;
	}

	return;

create_debugfs_fail:
	dev_err(&vi->ndev->dev, "%s: could not create debugfs", __func__);
	vi_remove_debugfs(vi);
}

static int vi_probe(struct platform_device *dev)
{
	int err = 0;
	struct vi *tegra_vi;
	struct nvhost_device_data *pdata = NULL;
	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_vi_of_match, &dev->dev);
		if (match) {
			pdata = (struct nvhost_device_data *)match->data;
			dev->dev.platform_data = pdata;
		}

		/* DT initializes it to -1, use below WAR to set correct value.
		 * TODO: Once proper fix for dev-id goes in, remove it.
		 */
		dev->id = dev->dev.id;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	dev_info(&dev->dev, "%s: ++\n", __func__);

	tegra_vi = devm_kzalloc(&dev->dev, sizeof(struct vi), GFP_KERNEL);
	if (!tegra_vi) {
		dev_err(&dev->dev, "can't allocate memory for vi\n");
		return -ENOMEM;
	}

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto vi_probe_fail;

	tegra_vi->ndev = dev;

	/* call vi_intr_init and stats_work */
	INIT_WORK(&tegra_vi->stats_work, vi_stats_worker);

	err = vi_intr_init(tegra_vi);
	if (err)
		goto vi_probe_fail;

	vi_create_debugfs(tegra_vi);

	i2c_ctrl = pdata->private_data;
	pdata->private_data = tegra_vi;

	/* Create I2C Devices according to settings from board file */
	if (i2c_ctrl && i2c_ctrl->new_devices)
		i2c_ctrl->new_devices(dev);

	tegra_vi->reg = regulator_get(&tegra_vi->ndev->dev, "avdd_dsi_csi");
	if (IS_ERR(tegra_vi->reg)) {
		err = PTR_ERR(tegra_vi->reg);
		if (err == -ENODEV)
			dev_info(&tegra_vi->ndev->dev,
				"%s: no regulator device\n", __func__);
		else
			dev_err(&tegra_vi->ndev->dev,
				"%s: couldn't get regulator\n", __func__);
		tegra_vi->reg = NULL;
		goto camera_i2c_unregister;
	}

#ifdef CONFIG_TEGRA_CAMERA
	tegra_vi->camera = tegra_camera_register(dev);
	if (!tegra_vi->camera) {
		dev_err(&dev->dev, "%s: can't register tegra_camera\n",
				__func__);
		goto vi_regulator_put;
	}
#endif

	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "ve";

	/* add module power domain and also add its domain
	 * as sub-domain of MC domain */
	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_client_device_init(dev);
	if (err)
		goto camera_unregister;

	return 0;

camera_unregister:
#ifdef CONFIG_TEGRA_CAMERA
	tegra_camera_unregister(tegra_vi->camera);
vi_regulator_put:
#endif
	regulator_put(tegra_vi->reg);
	tegra_vi->reg = NULL;

camera_i2c_unregister:
	if (i2c_ctrl && i2c_ctrl->remove_devices)
		i2c_ctrl->remove_devices(dev);
	pdata->private_data = i2c_ctrl;
vi_probe_fail:
	dev_err(&dev->dev, "%s: failed\n", __func__);
	return err;
}

static int __exit vi_remove(struct platform_device *dev)
{
#ifdef CONFIG_TEGRA_CAMERA
	int err = 0;
#endif
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct vi *tegra_vi = (struct vi *)pdata->private_data;

	dev_info(&dev->dev, "%s: ++\n", __func__);

#if defined(CONFIG_TEGRA_ISOMGR)
	if (tegra_vi->isomgr_handle)
		vi_isomgr_unregister(tegra_vi);
#endif

	vi_remove_debugfs(tegra_vi);

	vi_intr_free(tegra_vi);

	nvhost_client_device_release(dev);
	pdata->aperture[0] = NULL;

#ifdef CONFIG_TEGRA_CAMERA
	err = tegra_camera_unregister(tegra_vi->camera);
	if (err)
		return err;
#endif

#ifdef CONFIG_PM_GENERIC_DOMAINS
	tegra_pd_remove_device(&dev->dev);
#endif

	regulator_put(tegra_vi->reg);
	tegra_vi->reg = NULL;

	/* Remove I2C Devices according to settings from board file */
	if (i2c_ctrl && i2c_ctrl->remove_devices)
		i2c_ctrl->remove_devices(dev);

	pdata->private_data = i2c_ctrl;

	return 0;
}

static struct platform_driver vi_driver = {
	.probe = vi_probe,
	.remove = __exit_p(vi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "vi",
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = tegra_vi_of_match,
#endif
	}
};

static int __init vi_init(void)
{
	return platform_driver_register(&vi_driver);
}

static void __exit vi_exit(void)
{
	platform_driver_unregister(&vi_driver);
}

late_initcall(vi_init);
module_exit(vi_exit);
MODULE_LICENSE("GPL v2");
