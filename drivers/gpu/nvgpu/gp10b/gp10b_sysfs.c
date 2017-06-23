/*
 * GP10B specific sysfs files
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/platform_device.h>

#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "gp10b_sysfs.h"

#include <nvgpu/hw/gp10b/hw_gr_gp10b.h>

#define ROOTRW (S_IRWXU|S_IRGRP|S_IROTH)

static ssize_t czf_bypass_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= 4)
		return -EINVAL;

	g->gr.czf_bypass = val;

	return count;
}

static ssize_t czf_bypass_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "%d\n", g->gr.czf_bypass);
}

static DEVICE_ATTR(czf_bypass, ROOTRW, czf_bypass_read, czf_bypass_store);

void gp10b_create_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int error = 0;

	g->gr.czf_bypass = gr_gpc0_prop_debug1_czf_bypass_init_v();

	error |= device_create_file(dev, &dev_attr_czf_bypass);
	if (error)
		nvgpu_err(g, "Failed to create sysfs attributes!");
}

void gp10b_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_czf_bypass);
}
