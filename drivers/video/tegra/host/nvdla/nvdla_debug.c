/*
 * NVDLA debug utils
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/nvhost.h>

#include "nvdla/nvdla.h"

void nvdla_debug_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct dentry *de = pdata->debugfs;

	if (!de)
		return;

	debugfs_create_u32("debug_mask", S_IRUGO|S_IWUSR, de,
			&nvdla_dev->dbg_mask);
	debugfs_create_u32("en_trace", S_IRUGO|S_IWUSR, de,
			&nvdla_dev->en_trace);
}
