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

/* Peak BPP for any of the YUV/Bayer formats */
#define CAMERA_PEAK_BPP 2

#define LANE_SPEED_1_GBPS 1000000000
#define LANE_SPEED_1_5_GBPS 1500000000

static const struct of_device_id tegra_camera_of_ids[] = {
	{ .compatible = "nvidia, tegra-camera-platform" },
	{ },
};

static struct miscdevice tegra_camera_misc;

static int tegra_camera_isomgr_register(struct tegra_camera_info *info)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	u32 num_csi_lanes;
	u32 max_num_streams;
	u64 max_lane_speed;
	u32 min_bits_per_pixel;

	dev_dbg(info->dev, "%s++\n", __func__);

	/* TODO: Extract these values from DT */
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	num_csi_lanes = 2;
	max_num_streams = 6;
	max_lane_speed = LANE_SPEED_1_5_GBPS;
	min_bits_per_pixel = 10;
#elif defined(CONFIG_ARCH_TEGRA_12x_SOC) || defined(CONFIG_ARCH_TEGRA_13x_SOC)
	num_csi_lanes = 2;
	max_num_streams = 2;
	max_lane_speed = LANE_SPEED_1_GBPS;
	min_bits_per_pixel = 10;
#else
	dev_err(info->dev, "%s Invalid chip-id\n", __func__);
	return -EINVAL;
#endif
	/*
	 * Let's go with simple registering max dedicated BW
	 * approach for now.
	 *
	 * The formula is:
	 * Camera's max total ISO BW =
	 * ((max_num_streams *
	 * num_csi_lanes * max_lane_speed) /
	 * min_bits_per_pixel) * max_peak_BPP
	 *
	 * Above considered cap is CSI link cap, but we need to
	 * consider real sensor-on-board cap also, DT based approach
	 * should handle it.
	 *
	 * Only VI out is considered, because in case of
	 * max # of cameras running, only VI is in ISO mode.
	 *
	 * TODO: Try renegotiate approach later.
	 */
	info->max_bw = (((num_csi_lanes * max_lane_speed * max_num_streams) /
				min_bits_per_pixel) * CAMERA_PEAK_BPP) / 1000;
	if (!info->max_bw) {
		dev_err(info->dev, "%s: BW must be non-zero\n", __func__);
		return -EINVAL;
	}

	dev_info(info->dev, "%s camera's max_iso_bw %llu\n",
				__func__, info->max_bw);

	/* Register with max possible BW for CAMERA usecases.*/
	info->isomgr_handle = tegra_isomgr_register(
					TEGRA_ISO_CLIENT_TEGRA_CAMERA,
					info->max_bw,
					NULL,	/* tegra_isomgr_renegotiate */
					NULL);	/* *priv */

	if (IS_ERR(info->isomgr_handle)) {
		dev_err(info->dev,
			"%s: unable to register to isomgr\n",
				__func__);
		return -ENOMEM;
	}
#endif

	return 0;
}

static int tegra_camera_isomgr_unregister(struct tegra_camera_info *info)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	tegra_isomgr_unregister(info->isomgr_handle);
	info->isomgr_handle = NULL;
#endif

	return 0;
}

static int tegra_camera_isomgr_request(
		struct tegra_camera_info *info, uint iso_bw, uint lt)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	int ret = 0;

	dev_dbg(info->dev,
		"%s++ bw=%u, lt=%u\n", __func__, iso_bw, lt);

	if (!info->isomgr_handle) {
		dev_err(info->dev,
		"%s: isomgr_handle is NULL\n",
		__func__);
		return -EINVAL;
	}

	/* return value of tegra_isomgr_reserve is dvfs latency in usec */
	ret = tegra_isomgr_reserve(info->isomgr_handle,
				iso_bw,	/* KB/sec */
				lt);	/* usec */
	if (!ret) {
		dev_err(info->dev,
		"%s: failed to reserve %u KBps\n", __func__, iso_bw);
		return -ENOMEM;
	}

	/* return value of tegra_isomgr_realize is dvfs latency in usec */
	ret = tegra_isomgr_realize(info->isomgr_handle);
	if (ret)
		dev_dbg(info->dev,
		"%s: tegra_camera isomgr latency is %d usec",
		__func__, ret);
	else {
		dev_err(info->dev,
		"%s: failed to realize %u KBps\n", __func__, iso_bw);
			return -ENOMEM;
	}
#endif

	return 0;
}

static int tegra_camera_isomgr_release(struct tegra_camera_info *info)
{
#if defined(CONFIG_TEGRA_ISOMGR)
	int ret = 0;
	dev_dbg(info->dev, "%s++\n", __func__);

	/* deallocate isomgr bw */
	ret = tegra_camera_isomgr_request(info, 0, 4);
	if (ret) {
		dev_err(info->dev,
		"%s: failed to deallocate memory in isomgr\n",
		__func__);
		return -ENOMEM;
	}
#endif

	return 0;
}

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	struct tegra_camera_info *info;
	struct miscdevice *mdev;
	int i, ret, index_put, index_disable;

	mdev = file->private_data;
	info = dev_get_drvdata(mdev->parent);

	file->private_data = info;
	for (i = 0; i < NUM_CLKS; i++) {
		info->clks[i] = clk_get_sys(info->devname, clk_names[i]);
		if (IS_ERR(info->clks[i])) {
			dev_err(info->dev, "clk_get_sys failed for %s:%s\n",
				info->devname, clk_names[i]);
			index_put = i-1;
			goto err_get_clk;
		}

		ret = clk_set_rate(info->clks[i], 0);
		if (ret) {
			dev_err(info->dev, "Cannot init %s\n", clk_names[i]);
			index_put = i;
			goto err_set_clk;
		}

		ret = clk_prepare_enable(info->clks[i]);
		if (ret) {
			dev_err(info->dev, "Cannot enable %s\n", clk_names[i]);
			index_put = i;
			index_disable = i-1;
			goto err_prep_clk;
		}
	}
	return 0;

err_prep_clk:
	for (i = index_disable; i >= 0; i--) {
		if (!IS_ERR_OR_NULL(info->clks[i]))
			clk_disable_unprepare(info->clks[i]);
	}

err_set_clk:
err_get_clk:
	for (i = index_put; i >= 0; i--) {
		if (!IS_ERR_OR_NULL(info->clks[i]))
			clk_put(info->clks[i]);
		info->clks[i] = NULL;
	}
	return -ENOENT;
}

static int tegra_camera_release(struct inode *inode, struct file *file)
{

	struct tegra_camera_info *info;
	int i;
	int ret;

	info = file->private_data;
	for (i = 0; i < NUM_CLKS; i++) {
		if (!IS_ERR_OR_NULL(info->clks[i])) {
			clk_disable_unprepare(info->clks[i]);
			clk_put(info->clks[i]);
		}
		info->clks[i] = NULL;
	}

	/* nullify isomgr request */
	if (info->isomgr_handle) {
		ret = tegra_camera_isomgr_release(info);
		if (ret) {
			dev_err(info->dev,
			"%s: failed to deallocate memory in isomgr\n",
			__func__);
			return -ENOMEM;
		}
	}

	return 0;
}

static long tegra_camera_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct tegra_camera_info *info;

	info = file->private_data;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(TEGRA_CAMERA_IOCTL_SET_BW):
	{
		struct bw_info kcopy;
		unsigned long mc_khz = 0;

		memset(&kcopy, 0, sizeof(kcopy));

		if (copy_from_user(&kcopy, (const void __user *)arg,
			sizeof(struct bw_info))) {
			dev_err(info->dev, "%s:Failed to get data from user\n",
				__func__);
			return -EFAULT;
		}

		/* Use Khz to prevent overflow */
		mc_khz = tegra_emc_bw_to_freq_req(kcopy.bw);
		mc_khz = min(ULONG_MAX / 1000, mc_khz);

		if (kcopy.is_iso) {
			dev_dbg(info->dev, "%s:Set iso bw %llu at %lu KHz\n",
				__func__, kcopy.bw, mc_khz);
			ret = clk_set_rate(info->clks[ISO_EMC], mc_khz * 1000);
			if (ret)
				dev_err(info->dev, "%s:Failed to set iso bw\n",
					__func__);

			/*
			 * Request to ISOMGR.
			 * 3 usec is minimum time to switch PLL source.
			 * Let's put 4 usec as latency for now.
			 */
			ret = tegra_camera_isomgr_request(info, kcopy.bw, 4);
			if (ret) {
				dev_err(info->dev,
				"%s: failed to reserve %llu KBps with isomgr\n",
				__func__, kcopy.bw);
				return -ENOMEM;
			}
		} else {
			dev_dbg(info->dev, "%s:Set bw %llu at %lu KHz\n",
				__func__, kcopy.bw, mc_khz);
			ret = clk_set_rate(info->clks[EMC], mc_khz * 1000);
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
		dev_err(tegra_camera_misc.this_device,
			"register failed for %s\n",
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

	memset(info, 0, sizeof(*info));

	strcpy(info->devname, tegra_camera_misc.name);
	info->dev = tegra_camera_misc.this_device;

	/* Register Camera as isomgr client. */
	ret = tegra_camera_isomgr_register(info);
	if (ret) {
		dev_err(info->dev,
		"%s: failed to register CAMERA as isomgr client\n",
		__func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, info);

	return 0;

}
static int tegra_camera_remove(struct platform_device *pdev)
{
	struct tegra_camera_info *info = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "%s:camera_platform_driver remove\n", __func__);

	if (info->isomgr_handle)
		tegra_camera_isomgr_unregister(info);

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

