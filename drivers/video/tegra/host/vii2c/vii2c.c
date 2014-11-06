/*
 * Tegra Graphics Host VII2C
 *
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/slab.h>
#include <linux/vii2c.h>
#include <linux/regulator/consumer.h>
#include <linux/tegra-powergate.h>

#include <media/tegra_v4l2_camera.h>
#include <media/camera.h>
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_job.h"
#include "nvhost_channel.h"
#include "nvhost_cdma.h"
#include "host1x/host1x01_hardware.h"
#include "class_ids.h"
#include "nvhost_cdma.h"
#include "t210/t210.h"

#define POWERGATE_DELAY		10

struct vii2c {
	bool registered;
	struct regulator *reg;
	u32 syncpt_id;
	u32 sync_fence;
};

static struct platform_device *vii2c_pdev;

static struct of_device_id vii2c_of_match[] = {
#if defined(CONFIG_TEGRA_GRHOST_VII2C)
	{
		.compatible = "nvidia,tegra210-vi-i2c",
		.data = (struct nvhost_device_data *)&t21_vii2c_info,
	},
#endif
	{ },
};

int nvhost_vii2c_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;
	int err = 0;

	if (vii2c->reg) {
		err = regulator_enable(vii2c->reg);
		if (err)
			nvhost_err(&pdev->dev,
				"%s: enable csi regulator failed.\n", __func__);
	}


	return err;
}

int nvhost_vii2c_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;
	int err = 0;

	if (vii2c->reg) {
		err = regulator_disable(vii2c->reg);
		if (err)
			nvhost_err(&pdev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
	}

	return err;
}

void nvhost_vii2c_module_reset(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (!pdata->clk[0])
		return;

	/* assert module reset */
	tegra_periph_reset_assert(pdata->clk[0]);

	udelay(POWERGATE_DELAY);
	/* deassert reset */
	tegra_periph_reset_deassert(pdata->clk[0]);
}

int nvhost_vii2c_start(struct platform_device *pdev)
{
	struct nvhost_syncpt *sp = &nvhost_get_host(pdev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;
	int err;

	nvhost_dbg_fn("");
	if (!vii2c->registered)
		return -EINVAL;

	err = nvhost_mutex_try_lock(sp, NVMODMUTEX_VI);
	if (err) {
		nvhost_err(&pdev->dev, "%s busy\n", __func__);
		return err;
	}
	nvhost_module_reset(pdev, true);
	err = nvhost_module_busy(pdev);
	if (err) {
		nvhost_err(&pdev->dev, "%s err %d\n", __func__, err);
		nvhost_mutex_unlock(sp, NVMODMUTEX_VI);
		return err;
	}

	vii2c->sync_fence = 0;

	return 0;
}
EXPORT_SYMBOL(nvhost_vii2c_start);

int nvhost_vii2c_end(struct platform_device *pdev)
{
	struct nvhost_syncpt *sp = &nvhost_get_host(pdev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;

	nvhost_dbg_fn("");

	if (!vii2c->registered)
		return -EINVAL;

	nvhost_module_idle(pdev);
	nvhost_mutex_unlock(sp, NVMODMUTEX_VI);

	return 0;
}
EXPORT_SYMBOL(nvhost_vii2c_end);

int nvhost_vii2c_reset(struct platform_device *pdev)
{
	struct nvhost_syncpt *sp = &nvhost_get_host(pdev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;

	if (!vii2c->registered)
		return -EINVAL;

	/* reset the vi-i2c state to ensure that the state is valid */
	nvhost_module_reset(pdev, false);

	/* ..then perform CPU increments */
	nvhost_syncpt_set_min_eq_max(sp, vii2c->syncpt_id);

	nvhost_syncpt_read_check(sp, vii2c->syncpt_id, &vii2c->sync_fence);
	nvhost_dbg_fn("%x\n", vii2c->sync_fence);
	return 0;
}
EXPORT_SYMBOL(nvhost_vii2c_reset);

int nvhost_vii2c_flush(struct platform_device *pdev)
{
	struct nvhost_syncpt *sp = &nvhost_get_host(pdev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;

	nvhost_dbg_fn("%s %d\n", __func__, vii2c->sync_fence);

	if (!vii2c->registered)
		return -EINVAL;

	vii2c->sync_fence += 1;
	return nvhost_syncpt_wait_timeout(sp, vii2c->syncpt_id,
		vii2c->sync_fence, 100, NULL, NULL, NULL);
}
EXPORT_SYMBOL(nvhost_vii2c_flush);

int nvhost_vii2c_hw_sync_inc(struct platform_device *pdev, int n)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct vii2c *vii2c = pdata->private_data;

	return nvhost_class_host_incr_syncpt(n, vii2c->syncpt_id);
}
EXPORT_SYMBOL(nvhost_vii2c_hw_sync_inc);

struct platform_device *nvhost_vii2c_open(void)
{
	struct nvhost_device_data *pdata;
	struct vii2c *vii2c;

	if (!vii2c_pdev)
		return NULL;

	pdata = platform_get_drvdata(vii2c_pdev);
	vii2c = pdata->private_data;

	if (vii2c->registered)
		return NULL;

	vii2c->registered = true;

	return vii2c_pdev;
}
EXPORT_SYMBOL(nvhost_vii2c_open);

void nvhost_vii2c_close(void)
{
	struct nvhost_device_data *pdata;
	struct vii2c *vii2c;

	if (!vii2c_pdev)
		return;

	pdata = platform_get_drvdata(vii2c_pdev);
	if (!pdata)
		return;

	vii2c = pdata->private_data;
	if (!vii2c)
		return;

	vii2c->registered = false;
}
EXPORT_SYMBOL(nvhost_vii2c_close);

static int vii2c_probe(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = NULL;
	struct vii2c *vii2c;
	int err = 0;

	if (dev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_device(vii2c_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	if (!pdata) {
		nvhost_err(&dev->dev, "no platform data");
		return -ENODATA;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	/* allocate vi-i2c private data */
	vii2c = devm_kzalloc(&dev->dev, sizeof(*vii2c), GFP_KERNEL);
	if (!vii2c)
		return -ENOMEM;
	pdata->private_data = vii2c;

	/* get regulator */
	vii2c->reg = devm_regulator_get(&dev->dev, "avdd_dsi_csi");
	if (IS_ERR(vii2c->reg)) {
		err = PTR_ERR(vii2c->reg);
		nvhost_err(&dev->dev, "could not get regulator: %d", err);
		return err;
	}

	/* allocate a syncpoint for vi-i2c */
	vii2c->syncpt_id = nvhost_get_syncpt_client_managed("vii2c");
	if (!vii2c->syncpt_id) {
		nvhost_err(&dev->dev, "could not allocate a syncpoint");
		return -ENOMEM;
	}

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto vii2c_probe_fail;

	nvhost_module_init(dev);

	/* add module power domain and also add its domain
	 * as sub-domain of MC domain */

	pdata->pd.name = "vii2c";
	err = nvhost_module_add_domain(&pdata->pd, dev);
	if (err)
		goto vii2c_probe_fail;

	err = nvhost_client_device_init(dev);
	if (err)
		goto vii2c_probe_fail;

	vii2c_pdev = dev;

	nvhost_dbg_info("initialized");
	return 0;

vii2c_probe_fail:
	nvhost_err(&dev->dev, "failed");
	return err;
}

static int __exit vii2c_remove(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct vii2c *vii2c = pdata->private_data;

	vii2c_pdev = NULL;

	nvhost_client_device_release(dev);
	nvhost_free_syncpt(vii2c->syncpt_id);

	return 0;
}

static struct platform_driver vii2c_driver = {
	.probe = vii2c_probe,
	.remove = __exit_p(vii2c_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "vi-i2c",
#ifdef CONFIG_OF
	.of_match_table = vii2c_of_match,
#endif
	}
};

static int __init vii2c_init(void)
{
	return platform_driver_register(&vii2c_driver);
}

static void __exit vii2c_exit(void)
{
	platform_driver_unregister(&vii2c_driver);
}

device_initcall(vii2c_init);
module_exit(vii2c_exit);
MODULE_LICENSE("GPL v2");
