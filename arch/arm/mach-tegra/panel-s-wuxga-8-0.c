/*
 * arch/arm/mach-tegra/panel-s-wuxga-8-0.c
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <mach/dc.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include "board.h"
#include "board-panel.h"
#include "devices.h"

#define DSI_PANEL_RESET		1

static bool reg_requested;
static struct regulator *avdd_lcd_3v0;
static struct regulator *dvdd_lcd_1v8;
static struct regulator *avdd_lcd_1v2;
static struct regulator *vdd_ts_3v3;
static struct regulator *vdd_ts_1v8;
static struct regulator *vpp_lcd;
static struct regulator *vmm_lcd;
static struct device *dc_dev;
static u16 en_panel_rst;
static bool en_vmm_vpp_i2c_config;
static struct i2c_client *dsi_s_vmm_vpp_i2c_client;

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};

static u8 dsi_s_vmm_vpp_configure[][2] = {
	{0x03, 0xF2},
	{0x01, 0x07},
	{0x01, 0x0F},
};

static struct i2c_board_info i2c_vmm_vpp_pdata = {
	.type = "vmm-vpp-adapter",
	.addr = 0x21,
};

static struct i2c_driver dsi_s_vmm_vpp_i2c_slave_driver = {
	.driver = {
		.name = "vmm-vpp-adapter",
	},
};

static struct i2c_client *init_e3320_i2c_slave(struct device *dev, struct i2c_board_info *pdata,
	struct i2c_driver *i2c_slave_driver)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int bus = 0;
	int err = 0;
	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		pr_err("e3320: Can't get adapter for I2C bus\n");
		err = -EBUSY;
		goto err;
	}

	client = i2c_new_device(adapter, pdata);
	i2c_put_adapter(adapter);
	if (!client) {
		pr_err("e3320: Can't add i2c slave device...\n");
		err = -EBUSY;
		goto err;
	}

	err = i2c_add_driver(i2c_slave_driver);
	if (err) {
		pr_err("e3320: Can't add i2c slave driver!\n");
		err = -EBUSY;
		goto err_free;
	}
	return client;
err_free:
	i2c_unregister_device(client);
err:
	return ERR_PTR(err);

}

static int dsi_s_e3320_i2c_configure(struct i2c_client *client, u8 data[][2],
		u8 num_of_xfers)
{
	struct i2c_msg *i2c_msg_xfer;
	u32 cnt;
	int err = 0;

	i2c_msg_xfer = kzalloc(num_of_xfers * sizeof(i2c_msg_xfer), GFP_KERNEL);
	if (!i2c_msg_xfer)
		return -ENOMEM;

	for (cnt = 0; cnt < num_of_xfers; cnt++) {
		i2c_msg_xfer[cnt].addr = client->addr;
		i2c_msg_xfer[cnt].flags = I2C_WRITE;
		i2c_msg_xfer[cnt].len = 2;
		i2c_msg_xfer[cnt].buf = data[cnt];
	}

	for (cnt = 0; cnt < num_of_xfers; cnt++) {
		err = i2c_transfer(client->adapter, &i2c_msg_xfer[cnt], 1);
		if (err < 0)
			break;
		msleep(10);
	}

	kfree(i2c_msg_xfer);
	return err;
}

static int dsi_s_wuxga_8_0_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	avdd_lcd_3v0 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR_OR_NULL(avdd_lcd_3v0)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v0);
		avdd_lcd_3v0 = NULL;
		goto fail;
	}

	dvdd_lcd_1v8 = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR_OR_NULL(dvdd_lcd_1v8)) {
		pr_err("dvdd_lcd_1v8 regulator get failed\n");
		err = PTR_ERR(dvdd_lcd_1v8);
		dvdd_lcd_1v8 = NULL;
		goto fail;
	}

	avdd_lcd_1v2 = regulator_get(dev, "avdd_lcd_1v2");
	if (IS_ERR_OR_NULL(avdd_lcd_1v2)) {
		pr_err("avdd_lcd_1v2 regulator get failed\n");
		err = PTR_ERR(avdd_lcd_1v2);
		avdd_lcd_1v2 = NULL;
		goto fail;
	}

	if (en_vmm_vpp_i2c_config) {
		vdd_ts_3v3 = regulator_get(dev, "vdd_ts_3v3");
		if (IS_ERR_OR_NULL(vdd_ts_3v3)) {
			pr_err("vdd_ts_3v3 regulator get failed\n");
			err = PTR_ERR(vdd_ts_3v3);
			vdd_ts_3v3 = NULL;
			goto fail;
		}

		vdd_ts_1v8 = regulator_get(dev, "vdd_ts_1v8");
		if (IS_ERR_OR_NULL(vdd_ts_1v8)) {
			pr_err("vdd_ts_1v8 regulator get failed\n");
			err = PTR_ERR(vdd_ts_1v8);
			vdd_ts_1v8 = NULL;
			goto fail;
		}
		dsi_s_vmm_vpp_i2c_client = init_e3320_i2c_slave(dev, &i2c_vmm_vpp_pdata,
			&dsi_s_vmm_vpp_i2c_slave_driver);
		if (IS_ERR_OR_NULL(dsi_s_vmm_vpp_i2c_client)) {
			pr_err("e3320: Failed to get vmm vpp i2c adapter client\n");
			dsi_s_vmm_vpp_i2c_client = NULL;
			err = -EINVAL;
			goto fail;
		}
	} else {
		vpp_lcd = regulator_get(dev, "outp");
		if (IS_ERR_OR_NULL(vpp_lcd)) {
			pr_err("vpp_lcd regulator get failed\n");
			err = PTR_ERR(vpp_lcd);
			vpp_lcd = NULL;
			goto fail;
		}

		vmm_lcd = regulator_get(dev, "outn");
		if (IS_ERR_OR_NULL(vmm_lcd)) {
			pr_err("vmm_lcd regulator get failed\n");
			err = PTR_ERR(vmm_lcd);
			vmm_lcd = NULL;
			goto fail;
		}
	}

	reg_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_s_wuxga_8_0_enable(struct device *dev)
{
	int err = 0;

	err = tegra_panel_check_regulator_dt_support("s,wuxga-8-0",
		&panel_of);
	if (err < 0) {
		pr_err("display regulator dt check failed\n");
		goto fail;
	} else {
		en_vmm_vpp_i2c_config = panel_of.en_vmm_vpp_i2c_config;
	}

	err = dsi_s_wuxga_8_0_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}

	err = tegra_panel_gpio_get_dt("s,wuxga-8-0", &panel_of);
	if (err < 0) {
		pr_err("display gpio get failed\n");
		goto fail;
	}

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		en_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];
	else {
		pr_err("display reset gpio invalid\n");
		goto fail;
	}


	if (dvdd_lcd_1v8) {
		err = regulator_enable(dvdd_lcd_1v8);
		if (err < 0) {
			pr_err("dvdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	usleep_range(500, 1500);

	if (avdd_lcd_3v0) {
		err = regulator_enable(avdd_lcd_3v0);
		if (err < 0) {
			pr_err("avdd_lcd regulator enable failed\n");
			goto fail;
		}
	}

	usleep_range(500, 1500);

	if (en_vmm_vpp_i2c_config) {
		if (avdd_lcd_1v2) {
			err = regulator_enable(avdd_lcd_1v2);
			if (err < 0) {
				pr_err("avdd_lcd_1v2 regulator enable failed\n");
				goto fail;
			}
		}

		usleep_range(500, 1500);

		/*
		 * To avoid touch panel power on sequence violation, first
		 * enable 1.8V rail and finally enable 3.3V rail.
		 */
		if (vdd_ts_1v8) {
			err = regulator_enable(vdd_ts_1v8);
			if (err < 0) {
				pr_err("vdd_ts_1v8 regulator enable failed\n");
				goto fail;
			}
		}

		usleep_range(500, 6000);

		if (vdd_ts_3v3) {
			err = regulator_enable(vdd_ts_3v3);
			if (err < 0) {
				pr_err("vdd_ts_3v3 regulator enable failed\n");
				goto fail;
			}
		}

		usleep_range(500, 1500);


		/* VMM and VPP Driving high */
		err = dsi_s_e3320_i2c_configure(dsi_s_vmm_vpp_i2c_client, dsi_s_vmm_vpp_configure,
			ARRAY_SIZE(dsi_s_vmm_vpp_configure));
		if (err < 0) {
			pr_err("e3320: Failed to configure vmm vpp driving %d\n", err);
			goto fail;
		}
	} else {
		if (vpp_lcd) {
			err = regulator_enable(vpp_lcd);
			if (err < 0) {
				pr_err("vpp_lcd regulator enable failed\n");
				goto fail;
			}

			err = regulator_set_voltage(vpp_lcd, 5500000, 5500000);
			if (err < 0) {
				pr_err("vpp_lcd regulator failed changing voltage\n");
				goto fail;
			}
		}

		usleep_range(500, 1500);

		if (vmm_lcd) {
			err = regulator_enable(vmm_lcd);
			if (err < 0) {
				pr_err("vmm_lcd regulator enable failed\n");
				goto fail;
			}

			err = regulator_set_voltage(vmm_lcd, 5500000, 5500000);
			if (err < 0) {
				pr_err("vmm_lcd regulator failed changing voltage\n");
				goto fail;
			}
		}

		usleep_range(7000, 8000);
	}

#if DSI_PANEL_RESET
	if (!tegra_dc_initialized(dev)) {
		err = gpio_direction_output(en_panel_rst, 1);
		if (err < 0) {
			pr_err("setting display reset gpio value failed\n");
			goto fail;
		}
		err = gpio_direction_output(en_panel_rst, 0);
		if (err < 0) {
			pr_err("setting display reset gpio value 0 failed\n");
			goto fail;
		}
		err = gpio_direction_output(en_panel_rst, 1);
		if (err < 0) {
			pr_err("setting display reset gpio value 1 failed\n");
			goto fail;
		}
	}
#endif
	dc_dev = dev;
	return 0;
fail:
	return err;
}

static int dsi_s_wuxga_8_0_disable(struct device *dev)
{
	if (gpio_is_valid(en_panel_rst)) {
		/* Wait for 50ms before triggering panel reset */
		msleep(50);
		gpio_set_value(en_panel_rst, 0);
		usleep_range(500, 1000);
	} else
		pr_err("ERROR! display reset gpio invalid\n");

	if (vmm_lcd)
		regulator_disable(vmm_lcd);

	usleep_range(1000, 2000);

	if (vpp_lcd)
		regulator_disable(vpp_lcd);

	usleep_range(1500, 2000);

	if (vdd_ts_1v8)
		regulator_disable(vdd_ts_1v8);

	usleep_range(1500, 2000);

	if (vdd_ts_3v3)
		regulator_disable(vdd_ts_3v3);

	usleep_range(1500, 2000);

	if (avdd_lcd_1v2)
		regulator_disable(avdd_lcd_1v2);

	usleep_range(1500, 2000);

	if (avdd_lcd_3v0)
		regulator_disable(avdd_lcd_3v0);

	usleep_range(1500, 2000);

	if (dvdd_lcd_1v8)
		regulator_disable(dvdd_lcd_1v8);

	/* Min delay of 140ms required to avoid turning
	 * the panel on too soon after power off */
	msleep(140);

	dc_dev = NULL;
	return 0;
}

static int dsi_s_wuxga_8_0_postsuspend(void)
{
	return 0;
}

static int dsi_s_wuxga_8_0_bl_notify(struct device *dev, int brightness)
{
	int cur_sd_brightness;
	struct backlight_device *bl = NULL;
	struct pwm_bl_data *pb = NULL;
	bl = (struct backlight_device *)dev_get_drvdata(dev);
	pb = (struct pwm_bl_data *)dev_get_drvdata(&bl->dev);

	if (dc_dev)
		nvsd_check_prism_thresh(dc_dev, brightness);

	cur_sd_brightness = atomic_read(&sd_brightness);
	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else if (pb->bl_measured)
		brightness = pb->bl_measured[brightness];

	return brightness;
}

static int dsi_s_wuxga_8_0_check_fb(struct device *dev,
	struct fb_info *info)
{
	struct platform_device *pdev = NULL;
	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

static struct pwm_bl_data_dt_ops dsi_s_wuxga_8_0_pwm_bl_ops = {
	.notify = dsi_s_wuxga_8_0_bl_notify,
	.check_fb = dsi_s_wuxga_8_0_check_fb,
	.blnode_compatible = "s,wuxga-8-0-bl",
};
struct tegra_panel_ops dsi_s_wuxga_8_0_ops = {
	.enable = dsi_s_wuxga_8_0_enable,
	.disable = dsi_s_wuxga_8_0_disable,
	.postsuspend = dsi_s_wuxga_8_0_postsuspend,
	.pwm_bl_ops = &dsi_s_wuxga_8_0_pwm_bl_ops,
};
