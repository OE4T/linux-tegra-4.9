/*
 * CY8C4014 LED chip driver
 *
 * Copyright (C) 2014 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/leds.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>

/* register definitions */
#define P1961_REG_CMD			0x00
#define P1961_REG_CMD_DAT		0x01
#define P1961_REG_CMD_STATUS		0x02
#define P1961_REG_APP_MINOR_REV		0x03
#define P1961_REG_APP_MAJOR_REV		0x04
#define P1961_REG_LED_STATE		0x05
#define P1961_REG_LED_ON_TIME		0x06
#define P1961_REG_LED_OFF_TIME		0x07
#define P1961_REG_MAX_BRIGHT		0x08
#define P1961_REG_NOM_BRIGHT		0x09
#define P1961_REG_LED_RAMP_UP		0x0A
#define P1961_REG_LED_RAMP_DOWN		0x0B
#define P1961_REG_HAPTIC_EN		0x0C
#define P1961_REG_HAPTIC_DRIVE_TIME	0x0D
#define P1961_REG_HAPTIC_BRAKE_DLY	0x0E
#define P1961_REG_HAPTIC_BRAKE_T	0x0F
#define P1961_REG_SOUND_PULSE_LEN	0x10
#define P1961_REG_MAX			0x11
#define P1961_CMD_ENTER_BL		0x01
#define P1961_CMD_WRITE_EEPROM		0x05

#define P1961_LED_STATE_BLINK       0x01
#define P1961_LED_STATE_BREATH		0x02
#define P1961_LED_STATE_SOLID       0x03



enum modes {
	MODE_BLINK = 0,
	MODE_BREATH,
	MODE_NORMAL,
};

struct mode_control {
	enum modes mode;
	char *mode_name;
	unsigned char reg_mode;
	unsigned char reg_mode_val;
	unsigned char reg_on_reg;
	unsigned char reg_off_reg;
};

static struct mode_control mode_controls[] = {
	{MODE_BLINK, "blink",
		P1961_REG_LED_STATE, P1961_LED_STATE_BLINK,
		P1961_REG_LED_ON_TIME, P1961_REG_LED_OFF_TIME},
	{MODE_BREATH, "breath",
		P1961_REG_LED_STATE, P1961_LED_STATE_BREATH,
		P1961_REG_LED_RAMP_UP, P1961_REG_LED_RAMP_DOWN},
	{MODE_NORMAL, "normal",
		P1961_REG_LED_STATE, P1961_LED_STATE_SOLID,
		P1961_REG_NOM_BRIGHT, P1961_REG_NOM_BRIGHT},
};
static int mode_controls_size =
	sizeof(mode_controls) / sizeof(mode_controls[0]);

struct cy8c_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct led_classdev led;
	int mode_index; /* mode index to mode_controls */
	struct mutex lock;
};

static int cy8c_get_mode_index(
	struct cy8c_data *data)
{
	int mode_index = 0;
	mutex_lock(&data->lock);
	mode_index = data->mode_index;
	mutex_unlock(&data->lock);
	return mode_index;
}

static void cy8c_set_mode_index(
	struct cy8c_data *data,
	int mode_index)
{
	mutex_lock(&data->lock);
	data->mode_index = mode_index;
	mutex_unlock(&data->lock);
}

static void set_led_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	struct cy8c_data *data = container_of(led_cdev, struct cy8c_data, led);
	int ret;

	ret = regmap_write(data->regmap,
		P1961_REG_NOM_BRIGHT, value & 0xff);
	ret |= regmap_write(data->regmap,
		P1961_REG_CMD_DAT, P1961_REG_NOM_BRIGHT);
	ret |= regmap_write(data->regmap,
		P1961_REG_CMD, P1961_CMD_WRITE_EEPROM);
	if (ret)
		dev_err(&data->client->dev, "cannot write %d\n", value);
}

static int of_led_parse_pdata(struct i2c_client *client, struct cy8c_data *data)
{
	struct device_node *np = client->dev.of_node;
	data->led.name = of_get_property(np, "label", NULL) ? : np->name;
	return 0;
}

static int cy8c_debug_set(void *data, u64 val)
{
	struct cy8c_data *cy_data = data;

	val = val & 0xff;
	if (val > P1961_CMD_ENTER_BL)
		return -EINVAL;

	pr_info("%s: send reset cmd %lld\n", __func__, val);
	regmap_write(cy_data->regmap, P1961_REG_CMD, val & 0xff);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cy8c_debug_fops, NULL, cy8c_debug_set, "%lld\n");

static ssize_t cy8c_effects_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	ssize_t ret = -EINVAL;
	struct regmap *regmap = NULL;
	int i = 0;
	int len = 0;

	data = container_of(led_cdev, struct cy8c_data, led);
	regmap = data->regmap;

	if (buf == NULL || buf[0] == 0) {
		dev_err(&data->client->dev, "input buf invalid.\n");
		return -EINVAL;
	}

	for (i = 0; i < mode_controls_size; i++) {
		/* use strncmp instead of strcmp
		   strcmp always returns incorrect value
		*/
		len = min(strlen(buf),
			strlen(mode_controls[i].mode_name));
		if (!strncmp(buf, mode_controls[i].mode_name, len))
			break;
	}

	if (i == mode_controls_size) {
		dev_err(&data->client->dev, "cannot find %s\n", buf);
		return -EINVAL;
	}

	ret = regmap_write(regmap, mode_controls[i].reg_mode,
		mode_controls[i].reg_mode_val);

	if (ret == 0)
		cy8c_set_mode_index(data, i);

	return ret == 0 ? size : -ENODEV;
}

static ssize_t cy8c_effects_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	int mode_index = 0;
	data = container_of(led_cdev, struct cy8c_data, led);
	mode_index = cy8c_get_mode_index(data);
	if (mode_index >= mode_controls_size) {
		dev_err(&data->client->dev, "mode error\n");
		return -EINVAL;
	} else
		return sprintf(buf, "%s\n",
			mode_controls[mode_index].mode_name);
}

static ssize_t cy8c_params_set(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	ssize_t ret = -EINVAL;
	struct regmap *regmap = NULL;
	int on_time, off_time;
	enum modes mode = 0;

	on_time = 0;
	off_time = 0;
	data = container_of(led_cdev, struct cy8c_data, led);
	regmap = data->regmap;

	if (buf == NULL || buf[0] == 0) {
		dev_err(&data->client->dev, "input buf invalid\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d %d", &on_time, &off_time) != 2) {
		dev_err(&data->client->dev, "input data format invalid\n");
		return -EINVAL;
	}

	if (on_time < 0 || on_time > 255) {
		dev_err(&data->client->dev, "input on time out of range\n");
		return -EINVAL;
	}

	if (off_time < 0 || off_time > 255) {
		dev_err(&data->client->dev, "input off time out of range\n");
		return -EINVAL;
	}

	mutex_lock(&data->lock);
	mode = mode_controls[data->mode_index].mode;
	if (mode != MODE_NORMAL) {
		ret = regmap_write(regmap,
			mode_controls[data->mode_index].reg_on_reg,
			on_time);
		ret |= regmap_write(regmap,
			mode_controls[data->mode_index].reg_off_reg,
			off_time);
	} else {
		dev_err(&data->client->dev, "mode in normal\n");
	}
	mutex_unlock(&data->lock);

	return ret == 0 ? size : ret;
}

static ssize_t cy8c_params_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct cy8c_data *data = NULL;
	int on_time, off_time;
	enum modes mode = 0;
	ssize_t ret = -EINVAL;
	data = container_of(led_cdev, struct cy8c_data, led);
	on_time = off_time = 0;

	mutex_lock(&data->lock);
	if (data->mode_index >= mode_controls_size) {
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	mode = mode_controls[data->mode_index].mode;

	if (mode != MODE_NORMAL) {
		ret = regmap_read(data->regmap,
			mode_controls[data->mode_index].reg_on_reg,
			&on_time);
		ret |= regmap_read(data->regmap,
			mode_controls[data->mode_index].reg_off_reg,
			&off_time);
	} else {
		dev_err(&data->client->dev, "mode in normal\n");
	}
	mutex_unlock(&data->lock);

	if (ret)
		return ret;
	else
		return sprintf(buf, "%d %d\n", on_time, off_time);
}

static DEVICE_ATTR(effects, S_IRUGO|S_IWUSR,
		cy8c_effects_show, cy8c_effects_set);
static DEVICE_ATTR(params, S_IRUGO|S_IWUSR,
		cy8c_params_show, cy8c_params_set);

static int cy8c_led_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct cy8c_data *data;
	struct regmap_config rconfig;
	struct dentry *d;
	int ret, reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* create debugfs for f/w loading purpose */
	d = debugfs_create_file("cy8c_led", S_IRUGO, NULL, data,
							&cy8c_debug_fops);
	if (!d)
		pr_err("Failed to create suspend_mode debug file\n");

	of_led_parse_pdata(client, data);

	data->led.brightness_set = set_led_brightness;

	/*TODO? Do we use a dedicated blink_set implementation?
	  data->led.blink_set = led_blink;*/

	/*TODO?: Do we need max-brightness to be read from DT?
	  data->led.max_brightness = <value-from-DT>*/

	memset(&rconfig, 0, sizeof(rconfig));
	rconfig.reg_bits = 8;
	rconfig.val_bits = 8;
	rconfig.cache_type = REGCACHE_NONE;
	rconfig.max_register = P1961_REG_MAX-1;

	/*This should happen before set clientdata*/
	data->regmap = regmap_init_i2c(client, &rconfig);
	if (!data->regmap) {
		dev_err(&client->dev, "Failed to allocate register map\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, data);

	ret = regmap_read(data->regmap, P1961_REG_APP_MAJOR_REV, &reg);
	if (ret) {
		dev_err(&client->dev, "Failed to read revision-major\n");
		return ret;
	}

	dev_dbg(&client->dev, "[nv-foster] rev: 0x%02x ", reg);

	ret = regmap_read(data->regmap, P1961_REG_APP_MINOR_REV, &reg);
	if (ret) {
		dev_err(&client->dev, "Failed to read revision-minor\n");
		return ret;
	}

	dev_dbg(&client->dev, "0x%02x\n", reg);

	ret = regmap_write(data->regmap, P1961_REG_LED_STATE,
						P1961_LED_STATE_SOLID);
	if (ret) {
		dev_err(&client->dev, "Failed to read revision-minor\n");
		return ret;
	}

	mutex_init(&data->lock);

	/* enable print in show/get */
	data->client = client;

	ret = led_classdev_register(&client->dev, &data->led);
	if (ret < 0)
		dev_err(&client->dev, "Failed to register foster led\n");
	else
		dev_info(&client->dev, "LED registered (%s)\n", data->led.name);

	ret = device_create_file(data->led.dev, &dev_attr_effects);
	if (ret)
		dev_err(&client->dev, "Failed to register effects sys node\n");
	ret = device_create_file(data->led.dev, &dev_attr_params);
	if (ret)
		dev_err(&client->dev, "Failed to register params sys node\n");

	return ret;
}

static int cy8c_led_remove(struct i2c_client *client)
{
	struct cy8c_data *data = i2c_get_clientdata(client);

	device_remove_file(data->led.dev, &dev_attr_effects);
	device_remove_file(data->led.dev, &dev_attr_params);
	led_classdev_unregister(&data->led);

	return 0;
}

static const struct i2c_device_id cy8c_led_id[] = {
	{"cy8c_led", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, cy8c_led_id);

#ifdef CONFIG_OF
static struct of_device_id cy8c_of_match[] = {
	{.compatible = "nvidia,cy8c_led", },
	{ },
};
#endif

static struct i2c_driver cy8c_led_driver = {
	.driver = {
		.name   = "cy8c_led",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(cy8c_of_match),
#endif
	},
	.probe      = cy8c_led_probe,
	.remove     = cy8c_led_remove,
	.id_table   = cy8c_led_id,
};

module_i2c_driver(cy8c_led_driver);

MODULE_AUTHOR("Vinayak Pane <vpane@nvidia.com>");
MODULE_DESCRIPTION("CY8C I2C based LED controller driver");
MODULE_LICENSE("GPL");
