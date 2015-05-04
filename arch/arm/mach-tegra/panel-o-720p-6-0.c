/*
 * arch/arm/mach-tegra/panel-o-720p-6-0.c
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
#include "gpio-names.h"

static u16 en_panel_rst;
static u16 en_panel_p5v;
static u16 en_panel_n5v;
static u16 en_backlight;

static int dsi_o_720p_6_0_enable(struct device *dev)
{
	int err = 0;

	err = tegra_panel_gpio_get_dt("o,720-1280-6-0", &panel_of);
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}
	/* If panel rst gpio is specified in device tree,
	 * use that.
	 */
	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_RESET]))
		en_panel_rst = panel_of.panel_gpio[TEGRA_GPIO_RESET];
	else
		pr_warn("rst gpio is not defined in DT\n");

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN]))
		en_panel_p5v = panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN];
	else
		pr_warn("panel en-0 gpio is not defined in DT\n");

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN_1]))
		en_panel_n5v = panel_of.panel_gpio[TEGRA_GPIO_PANEL_EN_1];
	else
		pr_warn("panel en-1 gpio is not defined in DT\n");

	if (gpio_is_valid(panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE]))
		en_backlight = panel_of.panel_gpio[TEGRA_GPIO_BL_ENABLE];
	else
		pr_warn("en_backlight gpio is not defined in DT\n");

	gpio_direction_output(en_panel_rst, 0);

	gpio_direction_output(en_panel_p5v, 1);
	msleep(20);
	gpio_direction_output(en_panel_n5v, 1);
	msleep(20);

	return 0;
fail:
	return err;
}

static int dsi_o_720p_6_0_postpoweron(struct device *dev)
{
/*
 * Having reset control in postpoweron.
 *  - dc->out->postpoweron => reset control
 *  - dc->out_ops->postpoweron => dsi init command trigger
 */
	msleep(20);
	gpio_set_value(en_panel_rst, 1);
	msleep(20);
	gpio_direction_output(en_backlight, 1);

	return 0;
}

static int dsi_o_720p_6_0_disable(struct device *dev)
{
	gpio_set_value(en_backlight, 0);
	usleep_range(1000, 1020);
	gpio_set_value(en_panel_rst, 0);
	usleep_range(1000, 1020);
	gpio_direction_output(en_panel_n5v, 0);
	msleep(20);
	gpio_direction_output(en_panel_p5v, 0);
	msleep(20);

	return 0;
}

static int dsi_o_720p_6_0_bl_notify(struct device *dev, int brightness)
{
	/*
	 * Just return delivered brightness from OS
	 * in earlier bring-up stage.
	 */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	return brightness;
}

static int dsi_o_720p_6_0_bl_check_fb(struct device *dev, struct fb_info *info)
{
	struct platform_device *pdev = NULL;
	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "tegradc.0"));
	return info->device == &pdev->dev;
}

static struct pwm_bl_data_dt_ops dsi_o_720p_6_0_pwm_bl_ops = {
	.notify = dsi_o_720p_6_0_bl_notify,
	.check_fb = dsi_o_720p_6_0_bl_check_fb,
	.blnode_compatible = "o,720-1280-6-0-bl",
};

struct tegra_panel_ops dsi_o_720p_6_0_ops = {
	.enable = dsi_o_720p_6_0_enable,
	.disable = dsi_o_720p_6_0_disable,
	.postpoweron = dsi_o_720p_6_0_postpoweron,
	.pwm_bl_ops = &dsi_o_720p_6_0_pwm_bl_ops,
};
