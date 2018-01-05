/*
 * t19x-nvlink-endpt-debugfs.c:
 * This file adds various debugfs nodes for the Tegra NVLINK controller.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"
#include <linux/uaccess.h>

static int nvlink_refclk_rate_file_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t nvlink_refclk_rate_file_read(struct file *file,
				char __user *ubuf,
				size_t count, loff_t *offp)
{
	struct nvlink_device *nvlink = file->private_data;
	char *buf;
	int str_len;

	switch (nvlink->refclk) {
	case NVLINK_REFCLK_150:
		buf = "150";
		str_len = 3;
		break;
	case NVLINK_REFCLK_156:
		buf = "156";
		str_len = 3;
		break;
	default:
		buf = "150";
		str_len = 3;
		break;
	}

	return simple_read_from_buffer(ubuf, count, offp, buf, str_len);
}

static ssize_t nvlink_refclk_rate_file_write(struct file *file,
				const char __user *ubuf,
				size_t count, loff_t *offp)
{
	struct nvlink_device *nvlink = file->private_data;
	char tmp[3];
	int ret;
	enum device_state state = NVLINK_DEVICE_OFF;

	ret = nvlink_get_dev_state(nvlink, &state);
	if (ret < 0) {
		nvlink_err("Error retriving the device state!");
		return ret;
	}
	if (NVLINK_DEVICE_OFF != state)
		return -EINVAL;

	ret = copy_from_user(tmp, ubuf, count);

	if (!strncmp(tmp, "150", 3))
		nvlink->refclk = NVLINK_REFCLK_150;
	else if (!strncmp(tmp, "156", 3))
                nvlink->refclk = NVLINK_REFCLK_156;
	else
		return -EINVAL;
	return count;
}

static const struct file_operations  nvlink_refclk_rate_fops = {
	.open	= nvlink_refclk_rate_file_open,
	.read	= nvlink_refclk_rate_file_read,
	.write	= nvlink_refclk_rate_file_write,
	.owner	= THIS_MODULE,
};

static int nvlink_speedcontrol_file_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t nvlink_speedcontrol_file_read(struct file *file,
					     char __user *ubuf,
					     size_t count,
					     loff_t *offp)
{
	struct nvlink_device *nvlink = file->private_data;
	char *buf;
	int str_len;

	switch (nvlink->speed) {
	case NVLINK_SPEED_20:
		buf = "20";
		str_len = 2;
		break;
	case NVLINK_SPEED_25:
		buf = "25";
		str_len = 2;
		break;
	default:
		buf = "25";
		str_len = 2;
		break;
	}

	return simple_read_from_buffer(ubuf, count, offp, buf, str_len);
}

static ssize_t nvlink_speedcontrol_file_write(struct file *file,
						const char __user *ubuf,
						size_t count, loff_t *offp)
{
	struct nvlink_device *nvlink = file->private_data;
	char tmp[2];
	int ret;
	enum device_state state = NVLINK_DEVICE_OFF;

	ret = nvlink_get_dev_state(nvlink, &state);
	if (ret < 0) {
		nvlink_err("Error retriving the device state!");
		return ret;
	}
	if (NVLINK_DEVICE_OFF != state)
		return -EINVAL;

	ret = copy_from_user(tmp, ubuf, count);

	if (!strncmp(tmp, "20", 2))
		nvlink->speed = NVLINK_SPEED_20;
	else if (!strncmp(tmp, "25", 2))
		nvlink->speed = NVLINK_SPEED_25;
	else
		return -EINVAL;
	return count;
}

static const struct file_operations  nvlink_speedcontrol_fops = {
	.open	= nvlink_speedcontrol_file_open,
	.read	= nvlink_speedcontrol_file_read,
	.write	= nvlink_speedcontrol_file_write,
	.owner	= THIS_MODULE,
};

/* TODO: Add debugfs nodes */
void t19x_nvlink_endpt_debugfs_init(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tdev =
				(struct tegra_nvlink_device *)ndev->priv;

	if (!nvlink_debugfs) {
		nvlink_err("Root NVLINK debugfs directory doesn't exist");
		goto fail;
	}

	tdev->tegra_debugfs = debugfs_create_dir(NVLINK_DRV_NAME,
						nvlink_debugfs);
	if (!tdev->tegra_debugfs) {
		nvlink_err("Failed to create Tegra NVLINK endpoint driver's"
			" debugfs directory");
		goto fail;
	}

	/* nvlink_rate_config: to switch and set different NVLINK RefCLK rate */
	tdev->tegra_debugfs_file = debugfs_create_file("refclk_rate",
				(S_IWUSR | S_IRUGO), tdev->tegra_debugfs,
				ndev, &nvlink_refclk_rate_fops);
	if (IS_ERR_OR_NULL(tdev->tegra_debugfs_file)) {
		tdev->tegra_debugfs_file = NULL;
		nvlink_dbg("debugfs_create_file() for nvlink_refclk_rate failed");
		goto fail;
	}
	/* nvlink_rate_config: to switch and set different NVLINK Speed Control */
	tdev->tegra_debugfs_file = debugfs_create_file("speed_control",
				(S_IWUSR | S_IRUGO), tdev->tegra_debugfs,
				ndev, &nvlink_speedcontrol_fops);
	if (IS_ERR_OR_NULL(tdev->tegra_debugfs_file)) {
		tdev->tegra_debugfs_file = NULL;
		nvlink_dbg("debugfs_create_file() for nvlink_rate_config failed");
		goto fail;
	}

	return;

fail:
	nvlink_err("Failed to create debugfs nodes");
	debugfs_remove_recursive(tdev->tegra_debugfs);
	tdev->tegra_debugfs = NULL;
}

void t19x_nvlink_endpt_debugfs_deinit(struct nvlink_device *ndev)
{
	struct tegra_nvlink_device *tdev =
				(struct tegra_nvlink_device *)ndev->priv;

	debugfs_remove_recursive(tdev->tegra_debugfs);
	tdev->tegra_debugfs = NULL;
}
