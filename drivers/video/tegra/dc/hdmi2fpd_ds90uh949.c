/*
 * FPDLink Serializer driver
 *
 * Copyright (C) 2014-2015 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include <mach/dc.h>

#include "dc_priv.h"
#include "hdmi2fpd_ds90uh949.h"
#include "hdmi.h"

static struct tegra_dc_hdmi2fpd_data *hdmi2fpd;
static struct i2c_client *ds90uh949_i2c_client;

int hdmi2fpd_enable(struct tegra_dc_hdmi_data *hdmi)
{
	struct tegra_dc_hdmi2fpd_data *hdmi2fpd = tegra_hdmi_get_outdata(hdmi);
	int err;

	if (hdmi2fpd && hdmi2fpd->hdmi2fpd_enabled)
		return 0;

	mutex_lock(&hdmi2fpd->lock);

	/* Turn on serializer chip */
	if (hdmi2fpd->en_gpio > 0)
		gpio_set_value(hdmi2fpd->en_gpio, 1);

	mdelay(hdmi2fpd->power_on_delay);

	hdmi2fpd->hdmi2fpd_enabled = true;
	mutex_unlock(&hdmi2fpd->lock);
	return err;
}

void hdmi2fpd_disable(struct tegra_dc_hdmi_data *hdmi)
{
	struct tegra_dc_hdmi2fpd_data *hdmi2fpd = tegra_hdmi_get_outdata(hdmi);

	mutex_lock(&hdmi2fpd->lock);
	/* Turn off serializer chip */
	if (hdmi2fpd->en_gpio > 0)
		gpio_set_value(hdmi2fpd->en_gpio, 0);

	mdelay(hdmi2fpd->power_off_delay);

	hdmi2fpd->hdmi2fpd_enabled = false;
	mutex_unlock(&hdmi2fpd->lock);
}

#ifdef CONFIG_PM
void hdmi2fpd_suspend(struct tegra_dc_hdmi_data *hdmi)
{
	hdmi2fpd_disable(hdmi);
}

int hdmi2fpd_resume(struct tegra_dc_hdmi_data *hdmi)
{
	return hdmi2fpd_enable(hdmi);
}
#endif

static struct regmap_config hdmi2fpd_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int of_hdmi2fpd_parse_platform_data(struct tegra_dc_hdmi_data *hdmi,
						struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	enum of_gpio_flags flags;
	u32 temp;
	int err = 0;

	if (!np) {
		dev_err(&hdmi->dc->ndev->dev,
				"hdmi2fpd: device node not defined in DT\n");
		err = -EINVAL;
		goto err;
	}

	hdmi2fpd->en_gpio = of_get_named_gpio_flags(np,
			"ti,enable-gpio", 0, &flags);
	hdmi2fpd->en_gpio_flags = flags;

	if (!hdmi2fpd->en_gpio) {
		dev_err(&hdmi->dc->ndev->dev,
				"hdmi2fpd: gpio number not provided\n");
		err = -EINVAL;
		goto err;
	}

	if (!of_property_read_u32(np, "ti,power-on-delay", &temp))
		hdmi2fpd->power_on_delay = temp;

	if (!of_property_read_u32(np, "ti,power-off-delay", &temp))
		hdmi2fpd->power_off_delay = temp;
err:
	return err;
}

int hdmi2fpd_init(struct tegra_dc_hdmi_data *hdmi)
{
	int err = 0;

	if (hdmi2fpd) {
		tegra_hdmi_set_outdata(hdmi, hdmi2fpd);
		return err;
	}

	hdmi2fpd = devm_kzalloc(&hdmi->dc->ndev->dev,
				sizeof(*hdmi2fpd), GFP_KERNEL);
	if (!hdmi2fpd)
		return -ENOMEM;

	err = of_hdmi2fpd_parse_platform_data(hdmi, ds90uh949_i2c_client);
	if (err)
		return err;

	err = gpio_request(hdmi2fpd->en_gpio, "hdmi2fpd");
	if (err < 0) {
		pr_err("err %d: hdmi2fpd GPIO request failed\n", err);
		return err;
	}
	if (hdmi2fpd->en_gpio_flags & OF_GPIO_ACTIVE_LOW)
		gpio_direction_output(hdmi2fpd->en_gpio, 0);
	else
		gpio_direction_output(hdmi2fpd->en_gpio, 1);

	hdmi2fpd->regmap = devm_regmap_init_i2c(ds90uh949_i2c_client,
						&hdmi2fpd_regmap_config);
	if (IS_ERR(hdmi2fpd->regmap)) {
		err = PTR_ERR(hdmi2fpd->regmap);
		dev_err(&ds90uh949_i2c_client->dev,
				"Failed to allocate register map: %d\n", err);
		return err;
	}

	hdmi2fpd->hdmi = hdmi;

	tegra_hdmi_set_outdata(hdmi, hdmi2fpd);

	mutex_init(&hdmi2fpd->lock);

	return err;
}

void hdmi2fpd_destroy(struct tegra_dc_hdmi_data *hdmi)
{
	struct tegra_dc_hdmi2fpd_data *hdmi2fpd = tegra_hdmi_get_outdata(hdmi);

	if (!hdmi2fpd)
		return;

	hdmi2fpd = NULL;
	mutex_destroy(&hdmi2fpd->lock);
}

static int ds90uh949_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}
	ds90uh949_i2c_client = client;

	return 0;
}

static int ds90uh949_remove(struct i2c_client *client)
{
	ds90uh949_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id ds90uh949_id[] = {
	{ "ds90uh949", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds90uh949_id);

#ifdef CONFIG_OF
static struct of_device_id ds90uh949_of_match[] = {
	{.compatible = "ti,ds90uh949", },
	{ },
};
#endif

static struct i2c_driver ds90uh949_driver = {
	.driver = {
		.name = "ds90uh949",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(ds90uh949_of_match),
#endif
	},
	.probe    = ds90uh949_probe,
	.remove   = ds90uh949_remove,
	.id_table = ds90uh949_id,
};

static int __init ds90uh949_i2c_client_init(void)
{
	int err = 0;

	err = i2c_add_driver(&ds90uh949_driver);
	if (err)
		pr_err("ds90uh949: Failed to add i2c client driver\n");

	return err;
}

static void __exit ds90uh949_i2c_client_exit(void)
{
	i2c_del_driver(&ds90uh949_driver);
}

subsys_initcall(ds90uh949_i2c_client_init);
module_exit(ds90uh949_i2c_client_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ds90uh949 FPDLink Serializer driver");
MODULE_ALIAS("i2c:ds90uh949_ser");

