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
 *
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include "stepper.h"

/*
 * File implements file system nodes to control the stepper motor.
*/

static ssize_t
max_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);

	return sprintf(buf, "Maximum allowed value = %d\n",
		stepper->ops->get_param(dev, STEPPER_MAX_RATE, 0));
}
static DEVICE_ATTR_RO(max_rate);

static ssize_t
status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);

	return sprintf(buf, "Input port status = 0x%x\n",
		stepper->ops->get_param(dev, STEPPER_STATUS, 0) & 0xff);
}
static DEVICE_ATTR_RO(status);


static ssize_t
start_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	ssize_t ret;
	struct stepper_device *stepper = to_stepper_device(dev);
	int dir, speed;

	data = stepper->ops->get_param(dev, STEPPER_START, 0);
	if (data) {
		dir = stepper->ops->get_param(dev, STEPPER_DIRECTION, 0);
		if (dir)
			speed = stepper->ops->get_param(dev,
				STEPPER_CW_RATE, 0);
		else
			speed = stepper->ops->get_param(dev,
				STEPPER_CCW_RATE, 0);

		ret = sprintf(buf, "Motor started %s with speed %d\n",
			dir?"clockwise":"counter-clockwise", speed);
	} else
		ret = sprintf(buf, "Motor stopped\n");
	return ret;
}

static ssize_t
start_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val & (~0x01))
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_START, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(start);

static ssize_t
stop_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val & (~0x01))
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_EMERGENCY, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_WO(stop);

static ssize_t
speed_cw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);

	return sprintf(buf, "Motor clockwise speed = %d\n",
		stepper->ops->get_param(dev, STEPPER_CW_RATE, 0));
}

static ssize_t
speed_cw_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0)
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_CW_RATE, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(speed_cw);

static ssize_t
speed_ccw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);

	return sprintf(buf, "Motor counter clockwise speed = %d\n",
		stepper->ops->get_param(dev, STEPPER_CCW_RATE, 0));
}

static ssize_t
speed_ccw_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0)
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_CCW_RATE, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(speed_ccw);

static ssize_t
ramp_up_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);

	return sprintf(buf, "Motor ramp up rate = %d\nMax value = 13\n",
		stepper->ops->get_param(dev, STEPPER_RAMPUP_VAL, 0));
}

static ssize_t
ramp_up_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if ((val > 0xf) || (val < 0))
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_RAMPUP_VAL, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(ramp_up);

static ssize_t
ramp_down_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);

	return sprintf(buf, "Motor ramp down rate = %d\nMax value = 13\n",
		stepper->ops->get_param(dev, STEPPER_RAMPDOWN_VAL, 0));
}

static ssize_t
ramp_down_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if ((val > 0xf) || (val < 0))
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_RAMPDOWN_VAL, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(ramp_down);

static ssize_t
direction_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int dir = stepper->ops->get_param(dev, STEPPER_DIRECTION, 0);
	ssize_t ret;

	if (dir)
		ret = sprintf(buf, "Clockwise rotation active\n");
	else
		ret = sprintf(buf, "Counter clockwise rotation active\n");
	return ret;
}

static ssize_t
direction_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val & (~0x01))
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_DIRECTION, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(direction);

static ssize_t
steps_cw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	ssize_t ret;

	ret = sprintf(buf, "Finite steps to rotate clockwise = %d\n",
		stepper->ops->get_param(dev, STEPPER_CW_STEPS, 0));
	return ret;
}

static ssize_t
steps_cw_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0)
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_CW_STEPS, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(steps_cw);

static ssize_t
steps_ccw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	ssize_t ret;

	ret = sprintf(buf, "Finite steps to rotate counter clockwise = %d\n",
		stepper->ops->get_param(dev, STEPPER_CCW_STEPS, 0));
	return ret;
}

static ssize_t
steps_ccw_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0)
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_CCW_STEPS, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(steps_ccw);

static ssize_t
mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	ssize_t ret;

	ret = sprintf(buf, "Mode \tDescription\r\n"
				"0\tContinuous\r\n"
				"1\tRotate till P0 sensor\r\n"
				"2\tCW then CCW\r\n"
				"3\tCCW then CW\r\n"
				"4\tFinite number of steps\r\n\r\nValue\t%d\n",
		stepper->ops->get_param(dev, STEPPER_MODE, 0));
	return ret;
}

static ssize_t
mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if ((val > STEPPER_FINITE_STEPS) || (val < 0))
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_MODE, 0, val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(mode);

static ssize_t
offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	ssize_t ret;
	struct stepper_device *stepper = to_stepper_device(dev);

	data = stepper->offset;
	ret = sprintf(buf, "Pointing at index = %d\n", data);
	return ret;
}

static ssize_t
offset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0)
		return -EINVAL;

	stepper->offset =  val;

	return n;
}
static DEVICE_ATTR_RW(offset);

static ssize_t
reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int data;
	ssize_t ret;
	struct stepper_device *stepper = to_stepper_device(dev);

	data = stepper->ops->get_param(dev, STEPPER_REG_OP, stepper->offset);
	ret = sprintf(buf, "Reading from register: %d, value = %d\n",
		stepper->offset, data);
	return ret;
}

static ssize_t
reg_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t n)
{
	struct stepper_device *stepper = to_stepper_device(dev);
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0)
		return -EINVAL;

	ret = stepper->ops->set_param(dev, STEPPER_REG_OP, stepper->offset,
					val);
	if (ret != 0)
		return -EINVAL;

	return n;
}
static DEVICE_ATTR_RW(reg);


static umode_t stepper_attr_is_visible(struct kobject *kobj,
				   struct attribute *attr, int n)
{
	umode_t mode = attr->mode;

	return mode;
}


static struct attribute *stepper_attrs[] = {
	&dev_attr_start.attr,
	&dev_attr_max_rate.attr,
	&dev_attr_speed_cw.attr,
	&dev_attr_speed_ccw.attr,
	&dev_attr_ramp_up.attr,
	&dev_attr_ramp_down.attr,
	&dev_attr_direction.attr,
	&dev_attr_steps_cw.attr,
	&dev_attr_steps_ccw.attr,
	&dev_attr_mode.attr,
	&dev_attr_stop.attr,
	&dev_attr_status.attr,
	&dev_attr_offset.attr,
	&dev_attr_reg.attr,
	NULL,
};

static struct attribute_group stepper_attr_group = {
	.is_visible	= stepper_attr_is_visible,
	.attrs		= stepper_attrs,
};

static const struct attribute_group *stepper_attr_groups[] = {
	&stepper_attr_group,
	NULL
};

const struct attribute_group **stepper_get_dev_attribute_groups(void)
{
	return stepper_attr_groups;
}
EXPORT_SYMBOL_GPL(stepper_get_dev_attribute_groups);

MODULE_AUTHOR("Vishruth Jain <vishruthj@nvidia.com");
MODULE_DESCRIPTION("Stepper motor class filesystem");
MODULE_LICENSE("GPL");

