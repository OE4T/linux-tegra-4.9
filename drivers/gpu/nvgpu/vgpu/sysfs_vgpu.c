/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/device.h>

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
