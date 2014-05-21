/*
 * HID driver for NVIDIA Shield Wireless Joystick
 *
 * Copyright (c) 2013-2014, NVIDIA Corporation. All Rights Reserved.
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

#include <linux/device.h>
#include <linux/input.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include "hid-ids.h"

#define JOYSTICK_FUZZ 64
#define TRIGGER_FUZZ 64
#define JOYSTICK_FLAT 64
#define TRIGGER_FLAT 0

#define MAX_CHAR 255

#define TOUCHPAD_DEFAULT_X 128
#define TOUCHPAD_DEFAULT_Y 128
#define TOUCH_HID_REPORT_SIZE 4
#define TOUCH_ACTION_MASK 0x8
#define TOUCH_ACTION_SHFT 3
#define TOUCH_ACTION_UP 0
#define TOUCH_ACTION_DOWN 1
#define TOUCH_REPORT_ID 2

#define MAX_SPEED 10
#define DEFAULT_SPEED 7

#define SCALE_LEN 12

struct nvidia_tp_loc {
	u8 x;
	u8 y;
	u8 action;
	u8 speed;/* Not used for now but keep as an parameter */
};

/*
 * Scale mapping is a table based on quadratic function
 */
static s8 blake_touch_scale_table[SCALE_LEN] = {
	0, 1, 10, 20, 35, 45, 61, 70, 75, 79, 100, 125};

static u8 scale_rel(u8 rel, u8 coeff)
{
	s8 s_rel = (s8)rel;
	s8 sign = (s_rel >= 0) ? 1 : -1;
	s8 abs_rel = s_rel * sign;

	if (abs_rel >= SCALE_LEN)
		abs_rel = SCALE_LEN - 1;

	return (u8)(sign * blake_touch_scale_table[abs_rel]);
}

static int nvidia_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size) {

	if (!report)
		return -EINVAL;

	unsigned id = report->id;
	struct nvidia_tp_loc *loc =
		(struct nvidia_tp_loc *)hid_get_drvdata(hdev);
	if (!loc)
		return -EINVAL;

	u8 action;
	u8 x, y;
	int press = 0;
	int release = 0;
	u8 relx, rely;

	/* If not valid touch events, let generic driver to handle this */
	if (id != TOUCH_REPORT_ID)
		return 0;

	if (!data)
		return -EINVAL;
	action = (data[1] & TOUCH_ACTION_MASK) >> TOUCH_ACTION_SHFT;
	x = data[2];
	y = data[4];

	if (!loc->action && action)
		press = 1;
	else if (loc->action && !action)
		release = 1;
	else if (!loc->action && !action)
		return 1;/* Double release, don't do anything */

	relx = scale_rel(x - loc->x, loc->speed);
	rely = scale_rel(y - loc->y, loc->speed);

	loc->action = action;

	dbg_hid("%u %u %u rel %d %d\n", action, x, y, (s8)relx, (s8)rely);

	if (release) {/* If a release event, don't do anything */
		return 1;
	} else {
		/* Record coordinates */
		loc->x = x;
		loc->y = y;
		if (!press) {
			/*
			 * Neither press or release event, we
			 * need to report it to input subsystem
			*/
			data[2] = relx;
			data[3] = rely;
			return 0;
		} else {
			/*
			 * if it's a press event,
			 * don't report.
			 */
			return 1;
		}
	}
}

static ssize_t blake_show_speed(struct device *dev,
		struct device_attribute *attr,
		char *buf) {

	struct hid_device *hdev =
		container_of(dev, struct hid_device, dev);

	struct nvidia_tp_loc *loc =
		(struct nvidia_tp_loc *)hid_get_drvdata(hdev);

	return snprintf(buf, MAX_CHAR, "%d\n", loc->speed);
}

static ssize_t blake_store_speed(struct device *dev,
	struct device_attribute *attr, const char *buf,
	size_t count) {

	struct hid_device *hdev = container_of(dev, struct hid_device, dev);
	struct nvidia_tp_loc *loc =
		(struct nvidia_tp_loc *)hid_get_drvdata(hdev);
	unsigned long speed_val;

	if (!loc)
		return count;

	if (!buf)
		return count;

	if (!kstrtoul(buf, 10, &speed_val)) {
		if (speed_val > MAX_SPEED)
			return count;
		loc->speed = (u8)speed_val;
	}

	return count;
}

static DEVICE_ATTR(speed, S_IRUGO | S_IWUSR,
	blake_show_speed, blake_store_speed);

static int nvidia_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct nvidia_tp_loc *loc;

	loc = (struct nvidia_tp_loc *)
		kmalloc(sizeof(struct nvidia_tp_loc *), GFP_KERNEL);

	if (!loc) {
		hid_err(hdev, "cannot alloc device touchpad state\n");
		return -ENOMEM;
	}

	loc->x = TOUCHPAD_DEFAULT_X;
	loc->y = TOUCHPAD_DEFAULT_Y;
	loc->action = 0;
	loc->speed = DEFAULT_SPEED;
	hid_set_drvdata(hdev, loc);

	/* Parse the HID report now */
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err_parse;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		goto err_parse;

	ret = device_create_file(&hdev->dev, &dev_attr_speed);
	if (ret)
		hid_warn(hdev, "cannot create sysfs for speed\n");
	ret = device_create_file(&hdev->dev, &dev_attr_mode);

	if (ret)
		hid_warn(hdev, "cannot create sysfs for mode\n");
	return 0;

	nvidia_init_ff(hdev, loc);
	nvidia_find_tp_len(hdev, loc);

err_parse:
	kfree(loc);
	return ret;
}

static void nvidia_remove(struct hid_device *hdev)
{
	struct nvidia_tp_loc *loc = hid_get_drvdata(hdev);

	if (!loc)
		return;

	device_remove_file(&hdev->dev, &dev_attr_speed);
	device_remove_file(&hdev->dev, &dev_attr_mode);

	hid_hw_stop(hdev);
}

static int nvidia_input_mapped(struct hid_device *hdev, struct hid_input *hi,
			      struct hid_field *field, struct hid_usage *usage,
			      unsigned long **bit, int *max)
{
	int a = field->logical_minimum;
	int b = field->logical_maximum;
	int fuzz;
	int flat;

	if ((usage->type == EV_ABS) && (field->application == HID_GD_GAMEPAD
			|| field->application == HID_GD_JOYSTICK)) {
		switch (usage->hid) {
		case HID_GD_X:
		case HID_GD_Y:
		case HID_GD_RX:
		case HID_GD_RY:
			fuzz = JOYSTICK_FUZZ;
			flat = JOYSTICK_FLAT;
			break;
		case HID_GD_Z:
		case HID_GD_RZ:
			fuzz = TRIGGER_FUZZ;
			flat = TRIGGER_FLAT;
			break;
		default: return 0;/*Use generic mapping for HatX, HatY*/
		}
		set_bit(usage->type, hi->input->evbit);
		set_bit(usage->code, *bit);
		input_set_abs_params(hi->input, usage->code, a, b, fuzz, flat);
		input_abs_set_res(hi->input, usage->code,
			hidinput_calc_abs_res(field, usage->code));
		return -1;
	}
	return 0;
}

static const struct hid_device_id nvidia_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_NVIDIA, USB_DEVICE_ID_NVIDIA_BLAKE) },
	{ }
};
MODULE_DEVICE_TABLE(hid, nvidia_devices);

static struct hid_driver nvidia_driver = {
	.name = "hid-nvidia-blake",
	.id_table = nvidia_devices,
	.input_mapped = nvidia_input_mapped,
	.raw_event = nvidia_raw_event,
	.probe = nvidia_probe,
	.remove = nvidia_remove,
};
module_hid_driver(nvidia_driver);

MODULE_AUTHOR("Jun Yan <juyan@nvidia.com>");
MODULE_LICENSE("GPL");
