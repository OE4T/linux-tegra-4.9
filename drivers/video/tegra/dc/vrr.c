/*
 * drivers/video/tegra/dc/vrr.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/kernel.h>
#include <mach/dc.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/backlight.h>
#include <linux/stat.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "vrr.h"

/* Elements for sysfs access */
#define VRR_ATTR(__name) static struct kobj_attribute vrr_attr_##__name = \
	__ATTR(__name, S_IRUGO|S_IWUSR, vrr_settings_show, vrr_settings_store)
#define VRR_ATTRS_ENTRY(__name) (&vrr_attr_##__name.attr)
#define IS_VRR_ATTR(__name) (attr == &vrr_attr_##__name)

static ssize_t vrr_settings_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);

static ssize_t vrr_settings_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count);

VRR_ATTR(capability);
VRR_ATTR(max_fps);
VRR_ATTR(min_fps);
VRR_ATTR(max_adj_pct);
VRR_ATTR(max_flip_pct);
VRR_ATTR(max_dc_balance);
VRR_ATTR(max_inc_pct);

static struct attribute *vrr_attrs[] = {
	VRR_ATTRS_ENTRY(capability),
	VRR_ATTRS_ENTRY(max_fps),
	VRR_ATTRS_ENTRY(min_fps),
	VRR_ATTRS_ENTRY(max_adj_pct),
	VRR_ATTRS_ENTRY(max_flip_pct),
	VRR_ATTRS_ENTRY(max_dc_balance),
	VRR_ATTRS_ENTRY(max_inc_pct),
	NULL,
};

static struct attribute_group vrr_attr_group = {
	.attrs = vrr_attrs,
};

static struct kobject *vrr_kobj;

/* Sysfs accessors */
static ssize_t vrr_settings_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_vrr *vrr  = dc->out->vrr;
	ssize_t res = 0;

	if (!vrr)
		return -EINVAL;

	if (IS_VRR_ATTR(capability))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->capability);
	else if (IS_VRR_ATTR(max_fps))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->vrr_max_fps);
	else if (IS_VRR_ATTR(min_fps))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->vrr_min_fps);
	else if (IS_VRR_ATTR(max_adj_pct))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->max_adj_pct);
	else if (IS_VRR_ATTR(max_flip_pct))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->max_flip_pct);
	else if (IS_VRR_ATTR(max_dc_balance))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->max_dc_balance);
	else if (IS_VRR_ATTR(max_inc_pct))
		res = snprintf(buf, PAGE_SIZE, "%d\n", vrr->max_inc_pct);
	else
		res = -EINVAL;

	return res;
}

#define vrr_check_and_update(_min, _max, _varname) { \
	int val; \
	if (kstrtol(buf, 10, (long *)&val) != -EINVAL) { \
		if (val >= _min && val <= _max) { \
			vrr->_varname = val; \
	} } }

static ssize_t vrr_settings_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct device *dev = container_of((kobj->parent), struct device, kobj);
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_vrr *vrr  = dc->out->vrr;
	ssize_t res = count;

	if (!vrr)
		return -EINVAL;

	if (IS_VRR_ATTR(capability))
		vrr_check_and_update(0, 1, capability)
	else if (IS_VRR_ATTR(max_fps))
		vrr_check_and_update(60, 120, vrr_max_fps)
	else if (IS_VRR_ATTR(min_fps))
		vrr_check_and_update(20, 60, vrr_min_fps)
	else if (IS_VRR_ATTR(max_adj_pct))
		vrr_check_and_update(0, 100, max_adj_pct)
	else if (IS_VRR_ATTR(max_flip_pct))
		vrr_check_and_update(0, 100, max_flip_pct)
	else if (IS_VRR_ATTR(max_dc_balance))
		vrr_check_and_update(0, 50000, max_dc_balance)
	else if (IS_VRR_ATTR(max_inc_pct))
		vrr_check_and_update(0, 100, max_inc_pct)
	else
		res = -EINVAL;

	return res;
}

/* Sysfs initializer */
int vrr_create_sysfs(struct device *dev)
{
	int retval = 0;

	vrr_kobj = kobject_create_and_add("vrr", &dev->kobj);

	if (!vrr_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(vrr_kobj, &vrr_attr_group);

	if (retval) {
		kobject_put(vrr_kobj);
		dev_err(dev, "%s: failed to create attributes\n", __func__);
	}

	return retval;
}
EXPORT_SYMBOL(vrr_create_sysfs);

/* Sysfs destructor */
void vrr_remove_sysfs(struct device *dev)
{
	if (vrr_kobj) {
		sysfs_remove_group(vrr_kobj, &vrr_attr_group);
		kobject_put(vrr_kobj);
	}
}
EXPORT_SYMBOL(vrr_remove_sysfs);

