/*
 * drivers/video/tegra/host/vii2c/vi_i2c.c
 *
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
#include "../../../../../arch/arm/mach-tegra/iomap.h"
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_job.h"
#include "nvhost_channel.h"
#include "nvhost_cdma.h"
#include "host1x/host1x01_hardware.h"
#include "class_ids.h"
#include "host1x/host1x_hwctx.h"
#include "class_ids.h"
#include "nvhost_cdma.h"
#include "t210/t210.h"

#define MAX_PROGRAM_SIZE	0x100

struct vii2c_struct {
	atomic_t in_use;
	struct platform_device *pdev;
	struct regulator *reg;
	void __iomem *base;
	struct nvhost_syncpt *sp;
};

static struct of_device_id vii2c_of_match[] = {
#if defined(CONFIG_TEGRA_GRHOST_VII2C)
	{
		.compatible = "nvidia,tegra210-vi-i2c",
		.data = (struct nvhost_device_data *)&t21_vii2c_info,
	},
#endif
	{ },
};

static struct vii2c_struct vii2c = {
	.in_use = ATOMIC_INIT(0),
};

/**
 * vii2c_close(): Initialise vii2c context
 *
 * The return value will be an error
 */
static int vii2c_close(struct nvhost_vii2c_struct *p_vii2c)
{
	struct host1x_hwctx *ctx = p_vii2c->ctx;
	int err;

	dev_dbg(&p_vii2c->pdev->dev, "%s: ++\n", __func__);

	tegra_powergate_partition(TEGRA_POWERGATE_VE);
	/* Release host1x_hwctx for VI_I2C */
	if (p_vii2c->job)
		nvhost_job_put(p_vii2c->job);
	p_vii2c->job = NULL;

	/* Release waiter */
	kfree(p_vii2c->completed_waiter);
	p_vii2c->completed_waiter = NULL;

	if (ctx && ctx->cpuva)
		dma_free_writecombine(&vii2c.pdev->dev,
			ctx->restore_size * 4, ctx->cpuva, ctx->iova);
	kfree(ctx);
	p_vii2c->ctx = NULL;

	if (vii2c.reg) {
		err = regulator_enable(vii2c.reg);
		if (err) {
			dev_err(&p_vii2c->pdev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
			return err;
		}
	}
	return 0;
}

/**
 * vii2c_open(): Initialise vii2c context
 *
 * The return value will be an error
 */
static int vii2c_open(struct nvhost_vii2c_struct *p_vii2c)
{
	struct device *dev = &p_vii2c->pdev->dev;
	struct host1x_hwctx *ctx;
	u32 syncpt_incrs = 1;
	int err = 0;

	dev_dbg(dev, "%s: ++\n", __func__);
	if (p_vii2c->ctx) {
		dev_err(dev, "%s already opened.\n", __func__);
		return 0;
	}

	if (vii2c.reg) {
		err = regulator_enable(vii2c.reg);
		if (err) {
			dev_err(dev, "%s: enable csi regulator failed.\n",
				__func__);
			return err;
		}
	}

	/* Allocate host1x_hwctx for VI_I2C */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(dev, "%s memory allocation failed @ %d\n",
			__func__, __LINE__);
		err = -ENOMEM;
		return err;
	}
	p_vii2c->ctx = ctx;

	/* Maximum size to be programmed */
	ctx->restore_size = MAX_PROGRAM_SIZE;

	ctx->cpuva = dma_alloc_writecombine(&vii2c.pdev->dev,
		ctx->restore_size * 4, &ctx->iova, GFP_KERNEL);
	if (!ctx->cpuva) {
		dev_err(dev, "%s memory allocation failed @ %d\n",
			__func__, __LINE__);
		err = -ENOMEM;
		goto fail;
	}

	kref_init(&ctx->hwctx.ref);

	/* Allocate waiter */
	p_vii2c->completed_waiter = nvhost_intr_alloc_waiter();

	/* Allocate Job */
	p_vii2c->job = nvhost_job_alloc(NULL, NULL, 0, 0, 0, syncpt_incrs);
	if (!p_vii2c->job) {
		dev_err(dev, "%s memory allocation failed @ %d\n",
			__func__, __LINE__);
		err = -ENOMEM;
		return err;
	}

	tegra_unpowergate_partition(TEGRA_POWERGATE_VE);

	dev_dbg(dev, "%s: done.\n", __func__);
	return 0;

fail:
	vii2c_close(p_vii2c);
	dev_err(dev, "%s: fail.\n", __func__);
	return err;
}

static int nvhost_vii2c_start(struct nvhost_vii2c_struct *p_vii2c)
{
	int err;

	dev_dbg(&p_vii2c->pdev->dev, "%s: ++ %p\n", __func__, p_vii2c->pdev);
	err = nvhost_mutex_try_lock(vii2c.sp, NVMODMUTEX_VI);
	if (err) {
		dev_err(&vii2c.pdev->dev, "%s busy\n", __func__);
		return -EBUSY;
	}

	nvhost_module_reset(vii2c.pdev, true);
	nvhost_module_enable_clk(&vii2c.pdev->dev);

	p_vii2c->syncval = nvhost_syncpt_incr_max(vii2c.sp, NVSYNCPT_VII2C, 1);

	mdelay(2);
	dev_dbg(&p_vii2c->pdev->dev, "%s: syncval = %d\n",
		__func__, p_vii2c->syncval);

	return 0;
}

static int nvhost_vii2c_end(struct nvhost_vii2c_struct *p_vii2c)
{
	dev_dbg(&p_vii2c->pdev->dev, "%s\n", __func__);
	nvhost_module_disable_clk(&vii2c.pdev->dev);
	nvhost_mutex_unlock(vii2c.sp, NVMODMUTEX_VI);
	return 0;
}

static int nvhost_vii2c_sync_wait(struct nvhost_vii2c_struct *p_vii2c)
{
	dev_dbg(&p_vii2c->pdev->dev, "%s: -- %d\n", __func__, p_vii2c->syncval);
	return nvhost_syncpt_wait_timeout(
		vii2c.sp,
		NVSYNCPT_VII2C,
		p_vii2c->syncval,
		100,
		NULL,
		NULL,
		NULL);
}

static int nvhost_vii2c_sync_inc(struct nvhost_vii2c_struct *p_vii2c, int n)
{
	return nvhost_class_host_incr_syncpt(n, NVSYNCPT_VII2C);
}

static void nvhost_vii2c_writel(u32 reg, u32 val)
{
	host1x_writel(vii2c.pdev, reg, val);
}

static u32 nvhost_vii2c_readl(u32 reg)
{
	return host1x_readl(vii2c.pdev, reg);
}

struct nvhost_vii2c_struct *nvhost_vii2c_open(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata;
	struct nvhost_vii2c_struct *pi2c;

	if ((atomic_read(&vii2c.in_use) == 0) || (vii2c.pdev == NULL)) {
		dev_err(&pdev->dev, "%s: not ready yet.\n", __func__);
		return NULL;
	}

	pi2c = devm_kzalloc(&pdev->dev, sizeof(*pi2c), GFP_KERNEL);
	if (!pi2c) {
		dev_err(&pdev->dev, "%s Could not alloc mem\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	pdata = platform_get_drvdata(vii2c.pdev);
	pi2c->base = pdata->aperture[0];
	pi2c->pdev = pdev;
	pi2c->open = vii2c_open;
	pi2c->close = vii2c_close;
	pi2c->module_start = nvhost_vii2c_start;
	pi2c->module_idle = nvhost_vii2c_end;
	pi2c->syncpt_wait = nvhost_vii2c_sync_wait;
	pi2c->syncpt_inc = nvhost_vii2c_sync_inc;
	pi2c->write_reg = nvhost_vii2c_writel;
	pi2c->read_reg = nvhost_vii2c_readl;

	return pi2c;
}
EXPORT_SYMBOL(nvhost_vii2c_open);

void nvhost_vii2c_close(struct nvhost_vii2c_struct *p_vii2c)
{
}
EXPORT_SYMBOL(nvhost_vii2c_close);

static int vii2c_probe(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = NULL;
	int err = 0;

	dev_info(&dev->dev, "%s: ++ %s\n", __func__, dev->name);
	if (atomic_xchg(&vii2c.in_use, 1)) {
		dev_err(&dev->dev, "%s OCCUPIED!\n", __func__);
		return -EBUSY;
	}

	if (dev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_device(vii2c_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	if (!pdata) {
		dev_err(&dev->dev, "%s: no platform data\n", __func__);
		return -ENODATA;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto vii2c_probe_fail;
	{
		int i;
		for (i = 0; i < dev->num_resources; i++)
			dev_dbg(&dev->dev,
				"%s: res[%d], %lx - %lx, %s %lx, map to %p\n",
				__func__, i,
				(unsigned long)dev->resource[i].start,
				(unsigned long)dev->resource[i].end,
				dev->resource[i].name, dev->resource[i].flags,
				pdata->aperture[i]);
	}
	vii2c.base = pdata->aperture[0];

	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "vii2c";
	/* add module power domain and also add its domain
	 * as sub-domain of MC domain */
	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_client_device_init(dev);
	if (err)
		goto vii2c_probe_fail;

	vii2c.reg = regulator_get(&dev->dev, "avdd_dsi_csi");
	if (IS_ERR(vii2c.reg)) {
		err = PTR_ERR(vii2c.reg);
		if (err == -ENODEV)
			dev_err(&dev->dev,
				"%s: no regulator device\n", __func__);
		else
			dev_err(&dev->dev, "%s: couldn't get regulator %d\n",
				__func__, err);
		vii2c.reg = NULL;
	}

	vii2c.sp = &nvhost_get_host(dev)->syncpt;
	vii2c.pdev = dev;
	dev_info(&dev->dev, "%s: ++ %s Initialized.\n", __func__, dev->name);
	return 0;

vii2c_probe_fail:
	dev_err(&dev->dev, "%s: failed\n", __func__);
	return err;
}

static int __exit vii2c_remove(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	dev_info(&dev->dev, "%s: ++\n", __func__);
	nvhost_client_device_release(dev);
	pdata->aperture[0] = NULL;
	regulator_put(vii2c.reg);
	vii2c.reg = NULL;

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
	pr_info("%s: ++\n", __func__);
	return platform_driver_register(&vii2c_driver);
}

static void __exit vii2c_exit(void)
{
	pr_info("%s: ++\n", __func__);
	platform_driver_unregister(&vii2c_driver);
}

device_initcall(vii2c_init);
module_exit(vii2c_exit);
MODULE_LICENSE("GPL v2");
