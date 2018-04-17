/*
 * VI5 driver for T194
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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
#include <soc/tegra/camrtc-capture.h>
#include <soc/tegra/chip-id.h>

#include "vi5.h"
#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "capture/capture-support.h"

#include "t194/t194.h"

#include <media/vi.h>
#include <media/mc_common.h>
#include <media/tegra_camera_platform.h>
#include "camera/vi/vi5_fops.h"
#include <linux/nvhost_vi_ioctl.h>
#include <linux/platform/tegra/latency_allowance.h>

/* HW capability, pixels per clock */
#define NUM_PPC		8
/* 15% bus protocol overhead */
#define VI_OVERHEAD	15

struct host_vi5 {
	struct platform_device *pdev;
	struct platform_device *vi_thi;
	struct vi vi_common;

	/* RCE RM area */
	struct sg_table rm_sgt;

	/* Debugfs */
	struct vi5_debug {
		struct debugfs_regset32 ch0;
	} debug;
};

static int vi5_init_debugfs(struct host_vi5 *vi5);
static void vi5_remove_debugfs(struct host_vi5 *vi5);

static int vi5_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id)
{
	struct host_vi5 *vi5 = nvhost_get_private_data(pdev);

	return t194_capture_alloc_syncpt(vi5->vi_thi, name, syncpt_id);
}

int nvhost_vi5_aggregate_constraints(struct platform_device *dev,
				int clk_index,
				unsigned long floor_rate,
				unsigned long pixelrate,
				unsigned long bw_constraint)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	if (!pdata) {
		dev_err(&dev->dev,
			"No platform data, fall back to default policy\n");
		return 0;
	}
	if (!pixelrate || clk_index != 0)
		return 0;
	/* SCF send request using NVHOST_CLK, which is calculated
	 * in floor_rate, so we need to aggregate its request
	 * with V4L2 pixelrate request
	 */
	return floor_rate + (pixelrate / pdata->num_ppc);
}

static void vi5_release_syncpt(struct platform_device *pdev, uint32_t id)
{
	struct host_vi5 *vi5 = nvhost_get_private_data(pdev);

	t194_capture_release_syncpt(vi5->vi_thi, id);
}

static void vi5_get_gos_table(struct platform_device *pdev, int *count,
			const dma_addr_t **table)
{
	struct host_vi5 *vi5 = nvhost_get_private_data(pdev);

	t194_capture_get_gos_table(vi5->vi_thi, count, table);
}

static int vi5_get_syncpt_gos_backing(struct platform_device *pdev,
			uint32_t id,
			dma_addr_t *syncpt_addr,
			uint32_t *gos_index,
			uint32_t *gos_offset)
{
	struct host_vi5 *vi5 = nvhost_get_private_data(pdev);

	return t194_capture_get_syncpt_gos_backing(vi5->vi_thi, id,
				syncpt_addr, gos_index, gos_offset);
}

static struct vi_channel_drv_ops vi5_channel_drv_ops = {
	.alloc_syncpt = vi5_alloc_syncpt,
	.release_syncpt = vi5_release_syncpt,
	.get_gos_table = vi5_get_gos_table,
	.get_syncpt_gos_backing = vi5_get_syncpt_gos_backing,
};

static int vi5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	struct device_node *thi_np;
	struct platform_device *thi = NULL;
	struct host_vi5 *vi5;
	struct tegra_camera_dev_info vi_info;
	int err = 0;

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
		err = -EPROBE_DEFER;
		goto put_vi;
	}

	vi5 = (struct host_vi5*) devm_kzalloc(dev, sizeof(*vi5), GFP_KERNEL);
	if (!vi5) {
		err = -ENOMEM;
		goto put_vi;
	}

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

	memset(&vi_info, 0, sizeof(vi_info));
	vi_info.pdev = pdev;
	vi_info.hw_type = HWTYPE_VI;
	vi_info.ppc = NUM_PPC;
	vi_info.overhead = VI_OVERHEAD;
	err = tegra_camera_device_register(&vi_info, vi5);
	if (err)
		goto device_release;

	err = vi_channel_drv_register(pdev, &vi5_channel_drv_ops);
	if (err)
		goto device_release;

	vi5_init_debugfs(vi5);

	vi5->vi_common.mc_vi.vi = &vi5->vi_common;
	vi5->vi_common.mc_vi.fops = &vi5_fops;
	err = tegra_vi_media_controller_init(&vi5->vi_common.mc_vi, pdev);
	if (err) {
		dev_warn(dev, "media controller init failed\n");
		err = 0;
	}

	return 0;

device_release:
	nvhost_client_device_release(pdev);
deinit:
	nvhost_module_deinit(pdev);
put_vi:
	platform_device_put(thi);
	if (err != -EPROBE_DEFER)
		dev_err(dev, "probe failed: %d\n", err);
	info->private_data = NULL;
	return err;
}

struct t194_vi5_file_private {
	struct platform_device *pdev;
	struct tegra_mc_vi mc_vi;
	struct mutex update_la_lock;
	unsigned int vi_bypass_bw;
};

static long nvhost_vi5_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	switch (cmd) {
	case NVHOST_VI_IOCTL_SET_VI_CLK: {
		return 0;
	}
	case _IOC_NR(NVHOST_VI_IOCTL_GET_VI_CLK): {
		return 0;
	}
	case NVHOST_VI_IOCTL_SET_VI_LA_BW: {
		/* TODO add LA setting later. */
		return 0;
	}
	}
	return -ENOIOCTLCMD;
}

static int nvhost_vi5_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct t194_vi5_file_private *filepriv;

	filepriv = kzalloc(sizeof(*filepriv), GFP_KERNEL);
	if (unlikely(filepriv == NULL))
		return -ENOMEM;

	filepriv->pdev = pdev;

	file->private_data = filepriv;

	return nonseekable_open(inode, file);
}

static int nvhost_vi5_release(struct inode *inode, struct file *file)
{
	struct t194_vi5_file_private *filepriv = file->private_data;

	kfree(filepriv);

	return 0;
}

const struct file_operations tegra194_vi5_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvhost_vi5_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvhost_vi5_ioctl,
#endif
	.open = nvhost_vi5_open,
	.release = nvhost_vi5_release,
};

static int vi5_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct host_vi5 *vi5 = pdata->private_data;

	tegra_camera_device_unregister(vi5);
	vi_channel_drv_unregister(&pdev->dev);
	tegra_vi_media_controller_cleanup(&vi5->vi_common.mc_vi);

	if (vi5->rm_sgt.sgl) {
		dma_unmap_sg(&pdev->dev, vi5->rm_sgt.sgl,
			vi5->rm_sgt.orig_nents, DMA_FROM_DEVICE);
		sg_free_table(&vi5->rm_sgt);
	}

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
	.remove = vi5_remove,
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

static int vi5_init_debugfs(struct host_vi5 *vi5)
{
	static const struct debugfs_reg32 vi5_ch_regs[] = {
		{ .name = "protocol_version", 0x00 },
		{ .name = "perforce_changelist", 0x4 },
		{ .name = "build_timestamp", 0x8 },
		{ .name = "channel_count", 0x80 },
	};
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
