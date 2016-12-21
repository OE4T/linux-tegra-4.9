/*
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
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gk20a.h>

#include "vgpu/vgpu.h"

static ssize_t vgpu_load_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_gpu_load_params *p = &msg.params.gpu_load;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_GET_GPU_LOAD;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err)
		return err;

	return snprintf(buf, PAGE_SIZE, "%u\n", p->load);
}
static DEVICE_ATTR(load, S_IRUGO, vgpu_load_show, NULL);

void vgpu_create_sysfs(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_load))
		dev_err(dev, "Failed to create vgpu sysfs attributes!\n");
}

void vgpu_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_load);
}
