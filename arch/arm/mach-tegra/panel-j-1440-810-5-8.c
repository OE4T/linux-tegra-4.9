/*
 * arch/arm/mach-tegra/panel-j-1440-810-5-8.c
 *
 * Copyright (c) 2013-2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <mach/dc.h>

#include "devices.h"
#include "board-panel.h"

#include "gpio-names.h"

static bool reg_requested;

static struct regulator *vdd_lcd_s_1v8;
static struct regulator *vdd_lcd_bl;
static struct regulator *vdd_lcd_bl_en;
static struct regulator *avdd_lcd_3v0_2v8;

static u16 en_panel_rst;
static u16 en_panel;

static int dsi_j_1440_810_5_8_reg_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	avdd_lcd_3v0_2v8 = regulator_get(dev, "avdd_lcd");
	if (IS_ERR(avdd_lcd_3v0_2v8)) {
		pr_err("avdd_lcd regulator get failed\n");
		err = PTR_ERR(avdd_lcd_3v0_2v8);
		avdd_lcd_3v0_2v8 = NULL;
		goto fail;
	}
	vdd_lcd_s_1v8 = regulator_get(dev, "dvdd_lcd");
	if (IS_ERR(vdd_lcd_s_1v8)) {
		pr_err("vdd_lcd_1v8_s regulator get failed\n");
		err = PTR_ERR(vdd_lcd_s_1v8);
		vdd_lcd_s_1v8 = NULL;
		goto fail;
	}

	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}
	reg_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_j_1440_810_5_8_enable(struct device *dev)
{
	int err = 0;

	err = dsi_j_1440_810_5_8_reg_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	err = tegra_panel_gpio_get_dt("j,1440-810-5-8", &panel_of);
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}
	/* If panel rst gpio is specified in device tree,
	 * use that.
	 */
	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		en_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN]))
		en_panel = panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN];

	gpio_direction_output(en_panel_rst, 0);
	gpio_direction_output(en_panel, 0);

	if (vdd_lcd_s_1v8) {
		err = regulator_enable(vdd_lcd_s_1v8);
		if (err < 0) {
			pr_err("vdd_lcd_1v8_s regulator enable failed\n");
			goto fail;
		}
	}
	usleep_range(3000, 5000);

	if (avdd_lcd_3v0_2v8) {
		err = regulator_enable(avdd_lcd_3v0_2v8);
		if (err < 0) {
			pr_err("avdd_lcd_3v0_2v8 regulator enable failed\n");
			goto fail;
		}
		regulator_set_voltage(avdd_lcd_3v0_2v8, 3100000, 3100000);
	}
	usleep_range(3000, 5000);

	if (vdd_lcd_bl) {
		err = regulator_enable(vdd_lcd_bl);
		if (err < 0) {
			pr_err("vdd_lcd_bl regulator enable failed\n");
			goto fail;
		}
	}

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}
	usleep_range(3000, 5000);

	gpio_direction_output(en_panel_rst, 1);
	msleep(20);
	gpio_set_value(en_panel, 1);
	msleep(20);

	return 0;
fail:
	return err;
}

static int dsi_j_1440_810_5_8_postpoweron(struct device *dev)
{
	msleep(80);
	return 0;
}

static int dsi_j_1440_810_5_8_disable(void)
{
	gpio_direction_output(en_panel_rst, 0);
	usleep_range(3000, 5000);

	if (vdd_lcd_bl)
		regulator_disable(vdd_lcd_bl);

	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);

	if (vdd_lcd_s_1v8)
		regulator_disable(vdd_lcd_s_1v8);

	if (avdd_lcd_3v0_2v8)
		regulator_disable(avdd_lcd_3v0_2v8);

	return 0;
}

static int dsi_j_1440_810_5_8_postsuspend(void)
{
	/* TODO */
	return 0;
}

static int dsi_j_1440_810_5_8_bl_notify(struct device *dev, int brightness)
{
	struct backlight_device *bl = NULL;
	struct pwm_bl_data *pb = NULL;
	int cur_sd_brightness = atomic_read(&sd_brightness);
	bl = (struct backlight_device *)dev_get_drvdata(dev);
	pb = (struct pwm_bl_data *)dev_get_drvdata(&bl->dev);

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else if (pb->bl_measured)
		brightness = pb->bl_measured[brightness];

	return brightness;
}

static int dsi_j_1440_810_5_8_check_fb(struct device *dev, struct fb_info *info)
{
	struct platform_device *pdev = NULL;
	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

struct pwm_bl_data_dt_ops dsi_j_1440_810_5_8_pwm_bl_ops = {
	.notify = dsi_j_1440_810_5_8_bl_notify,
	.check_fb = dsi_j_1440_810_5_8_check_fb,
	.blnode_compatible = "j,1440-810-5-8-bl",
};

struct tegra_panel_ops dsi_j_1440_810_5_8_ops = {
	.enable = dsi_j_1440_810_5_8_enable,
	.disable = dsi_j_1440_810_5_8_disable,
	.postsuspend = dsi_j_1440_810_5_8_postsuspend,
	.postpoweron = dsi_j_1440_810_5_8_postpoweron,
	.pwm_bl_ops = &dsi_j_1440_810_5_8_pwm_bl_ops,
};
