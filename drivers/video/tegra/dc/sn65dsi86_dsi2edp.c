/*
 * drivers/video/tegra/dc/sn65dsi86_dsi2edp.c
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Bibek Basu <bbasu@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/swab.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/dc.h>
#include "dc_priv.h"
#include "sn65dsi86_dsi2edp.h"
#include "dsi.h"

static struct tegra_dc_dsi2edp_data *sn65dsi86_dsi2edp;
static struct i2c_client *sn65dsi86_i2c_client;

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};

static inline int sn65dsi86_reg_write(struct tegra_dc_dsi2edp_data *dsi2edp,
					unsigned int addr, unsigned int val)
{
	return regmap_write(dsi2edp->regmap, addr, val);
}

static inline void sn65dsi86_reg_read(struct tegra_dc_dsi2edp_data *dsi2edp,
					unsigned int addr, unsigned int *val)
{
	regmap_read(dsi2edp->regmap, addr, val);
}

static const struct regmap_config sn65dsi86_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int sn65dsi86_dsi2edp_init(struct tegra_dc_dsi_data *dsi)
{
	int err = 0;
	struct tegra_dc_dsi2edp_data *dsi2edp = sn65dsi86_dsi2edp;
	if (!sn65dsi86_dsi2edp)
		return -ENODEV;

	dsi2edp->dsi = dsi;
	dsi2edp->mode = &dsi->dc->mode;
	tegra_dsi_set_outdata(dsi, dsi2edp);

	if (dsi2edp->en_gpio) {
		err = gpio_request(dsi2edp->en_gpio, "dsi2dp");
		if (err < 0) {
			pr_err("err %d: dsi2dp GPIO request failed\n", err);
		} else {
			if (dsi2edp->en_gpio_flags & OF_GPIO_ACTIVE_LOW)
				gpio_direction_output(dsi2edp->en_gpio, 0);
			else
				gpio_direction_output(dsi2edp->en_gpio, 1);
		}
	}

	return 0;
}

static void sn65dsi86_dsi2edp_destroy(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp =
				tegra_dsi_get_outdata(dsi);

	if (!dsi2edp)
		return;

	if (dsi2edp->en_gpio) {
		if (dsi2edp->en_gpio_flags & OF_GPIO_ACTIVE_LOW)
			gpio_set_value(dsi2edp->en_gpio, 1);
		else
			gpio_set_value(dsi2edp->en_gpio, 0);
	}
	sn65dsi86_dsi2edp = NULL;
	mutex_destroy(&dsi2edp->lock);
}

static void sn65dsi86_dsi2edp_enable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);
	unsigned val = 0;

	if (dsi2edp && dsi2edp->dsi2edp_enabled)
		return;

	mutex_lock(&dsi2edp->lock);
	/* REFCLK 19.2MHz */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_PLL_REFCLK_CFG,
			dsi2edp->pll_refclk_cfg);
	usleep_range(10000, 12000);
	/* Single 4 DSI lanes */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_DSI_CFG1, 0x26);
	usleep_range(10000, 12000);
	/* DSI CLK FREQ 422.5MHz */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_DSI_CHA_CLK_RANGE,
			dsi2edp->dsi_cha_clk_range);
	usleep_range(10000, 12000);
	sn65dsi86_reg_read(dsi2edp, SN65DSI86_DSI_CHA_CLK_RANGE, &val);
	sn65dsi86_reg_read(dsi2edp, SN65DSI86_DSI_CHA_CLK_RANGE, &val);

	/* disable ASSR via TEST2 PULL UP */
	if (dsi2edp->disable_assr) {
		sn65dsi86_reg_write(dsi2edp, 0xFF, 0x07);
		sn65dsi86_reg_write(dsi2edp, 0x16, 0x01);
		sn65dsi86_reg_write(dsi2edp, 0xFF, 0x00);
	}

	/* enhanced framing */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_FRAMING_CFG, 0x04);
	usleep_range(10000, 12000);
	/* Pre0dB 2 lanes no SSC */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_DP_SSC_CFG,
			dsi2edp->dp_ssc_cfg);
	usleep_range(10000, 12000);
	/* L0mV HBR */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_DP_CFG, 0x80);
	usleep_range(10000, 12000);
	/* PLL ENABLE */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_PLL_EN, 0x01);
	usleep_range(10000, 12000);
	/* DP_PLL_LOCK */
	sn65dsi86_reg_read(dsi2edp, SN65DSI86_PLL_REFCLK_CFG, &val);
	usleep_range(10000, 12000);
	/* POST2 0dB */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_TRAINING_CFG, 0x00);
	usleep_range(10000, 12000);
	/* Semi-Auto TRAIN */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_ML_TX_MODE, 0x0a);
	usleep_range(10000, 12000);
	/* ADDR 0x96 CFR */
	sn65dsi86_reg_read(dsi2edp, SN65DSI86_ML_TX_MODE, &val);
	msleep(20);
	/* CHA_ACTIVE_LINE_LENGTH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_VIDEO_CHA_LINE_LOW, 0x80);
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_VIDEO_CHA_LINE_HIGH, 0x07);
	usleep_range(10000, 12000);
	/* CHA_VERTICAL_DISPLAY_SIZE */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_VERT_DISP_SIZE_LOW, 0x38);
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_VERT_DISP_SIZE_HIGH, 0x04);
	usleep_range(10000, 12000);
	/* CHA_HSYNC_PULSE_WIDTH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_HSYNC_PULSE_WIDTH_LOW,
			dsi2edp->h_pulse_width_low);
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_HSYNC_PULSE_WIDTH_HIGH,
			dsi2edp->h_pulse_width_high);
	usleep_range(10000, 12000);
	/* CHA_VSYNC_PULSE_WIDTH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_VSYNC_PULSE_WIDTH_LOW,
			dsi2edp->v_pulse_width_low);
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_VSYNC_PULSE_WIDTH_HIGH,
			0x80);
	usleep_range(10000, 12000);
	/* CHA_HORIZONTAL_BACK_PORCH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_HORIZONTAL_BACK_PORCH,
			dsi2edp->h_back_porch);
	usleep_range(10000, 12000);
	/* CHA_VERTICAL_BACK_PORCH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_VERTICAL_BACK_PORCH,
			dsi2edp->v_back_porch);
	usleep_range(10000, 12000);
	/* CHA_HORIZONTAL_FRONT_PORCH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_HORIZONTAL_FRONT_PORCH,
			dsi2edp->h_front_porch);
	usleep_range(10000, 12000);
	/* CHA_VERTICAL_FRONT_PORCH */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_CHA_VERTICAL_FRONT_PORCH,
			dsi2edp->v_front_porch);
	usleep_range(10000, 12000);
	/* DP-18BPP Enable */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_DP_18BPP_EN, 0x00);
	msleep(100);
	/* COLOR BAR */
	/* sn65dsi86_reg_write(dsi2edp, SN65DSI86_COLOR_BAR_CFG, 0x10);*/
	/* enhanced framing and Vstream enable */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_FRAMING_CFG, 0x0c);
	dsi2edp->dsi2edp_enabled = true;
	mutex_unlock(&dsi2edp->lock);
}

static void sn65dsi86_dsi2edp_disable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	mutex_lock(&dsi2edp->lock);
	/* enhanced framing and Vstream disable */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_FRAMING_CFG, 0x04);
	dsi2edp->dsi2edp_enabled = false;
	mutex_unlock(&dsi2edp->lock);

}

#ifdef CONFIG_PM
static void sn65dsi86_dsi2edp_suspend(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	mutex_lock(&dsi2edp->lock);
	/* configure GPIO1 for suspend */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_GPIO_CTRL_CFG, 0x02);
	dsi2edp->dsi2edp_enabled = false;
	mutex_unlock(&dsi2edp->lock);

}

static void sn65dsi86_dsi2edp_resume(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	mutex_lock(&dsi2edp->lock);
	/* disable configure GPIO1 for suspend */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_GPIO_CTRL_CFG, 0x00);
	/* enhanced framing and Vstream enable */
	sn65dsi86_reg_write(dsi2edp, SN65DSI86_FRAMING_CFG, 0x0c);
	dsi2edp->dsi2edp_enabled = true;
	mutex_unlock(&dsi2edp->lock);
}
#endif

struct tegra_dsi_out_ops tegra_dsi2edp_ops = {
	.init = sn65dsi86_dsi2edp_init,
	.destroy = sn65dsi86_dsi2edp_destroy,
	.enable = sn65dsi86_dsi2edp_enable,
	.disable = sn65dsi86_dsi2edp_disable,
#ifdef CONFIG_PM
	.suspend = sn65dsi86_dsi2edp_suspend,
	.resume = sn65dsi86_dsi2edp_resume,
#endif
};

static int of_dsi2edp_parse_platform_data(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct tegra_dc_dsi2edp_data *dsi2edp = sn65dsi86_dsi2edp;
	enum of_gpio_flags flags;
	int err = 0;
	u32 temp;

	if (!np) {
		dev_err(&client->dev, "dsi2edp: device node not defined\n");
		err = -EINVAL;
		goto err;
	}

	dsi2edp->en_gpio = of_get_named_gpio_flags(np,
			"ti,enable-gpio", 0, &flags);
	dsi2edp->en_gpio_flags = flags;
	if (!dsi2edp->en_gpio)
		dev_err(&client->dev, "dsi2edp: gpio number not provided\n");

	if (!of_property_read_u32(np, "ti,pll-refclk-cfg", &temp)) {
		dsi2edp->pll_refclk_cfg = temp;
	} else {
		/* TBC: default setting for backward compatibility */
		dsi2edp->pll_refclk_cfg = 0x02;
		dsi2edp->dsi_cha_clk_range = 0x55;
		dsi2edp->disable_assr = 0;
		dsi2edp->dp_ssc_cfg = 0x20;
		dsi2edp->h_pulse_width_low = 0x10;
		dsi2edp->h_pulse_width_high = 0x80;
		dsi2edp->v_pulse_width_low = 0x0e;
		dsi2edp->v_pulse_width_high = 0x80;
		dsi2edp->h_back_porch = 0x98;
		dsi2edp->v_back_porch = 0x13;
		dsi2edp->h_front_porch = 0x10;
		dsi2edp->v_front_porch = 0x03;
	}

	if (!of_property_read_u32(np, "ti,dsi-cha-clk-range", &temp)) {
		dsi2edp->dsi_cha_clk_range = temp;
	}

	if (!of_property_read_u32(np, "ti,disable-assr", &temp)) {
		dsi2edp->disable_assr = temp;
	}

	if (!of_property_read_u32(np, "ti,dp-ssc-cfg", &temp)) {
		dsi2edp->dp_ssc_cfg = temp;
	}

	if (!of_property_read_u32(np, "ti,h-pulse-width-low", &temp)) {
		dsi2edp->h_pulse_width_low = temp;
	}

	if (!of_property_read_u32(np, "ti,h-pulse-width-high", &temp)) {
		dsi2edp->h_pulse_width_high = temp;
	}

	if (!of_property_read_u32(np, "ti,v-pulse-width-low", &temp)) {
		dsi2edp->v_pulse_width_low = temp;
	}

	if (!of_property_read_u32(np, "ti,v-pulse-width-high", &temp)) {
		dsi2edp->v_pulse_width_high = temp;
	}

	if (!of_property_read_u32(np, "ti,h-back-porch", &temp)) {
		dsi2edp->h_back_porch = temp;
	}

	if (!of_property_read_u32(np, "ti,v-back-porch", &temp)) {
		dsi2edp->v_back_porch = temp;
	}

	if (!of_property_read_u32(np, "ti,h-front-porch", &temp)) {
		dsi2edp->h_front_porch = temp;
	}

	if (!of_property_read_u32(np, "ti,v-front-porch", &temp)) {
		dsi2edp->v_front_porch = temp;
	}

err:
	return err;
}

static int sn65dsi86_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct device_node *pri_pn = NULL;
	struct device_node *sec_pn = NULL;
	bool pri_bridge = 0;
	bool sec_bridge = 0;
	int err = 0;

	sn65dsi86_i2c_client = client;
	sn65dsi86_dsi2edp = devm_kzalloc(&client->dev,
					sizeof(*sn65dsi86_dsi2edp),
					GFP_KERNEL);
	if (!sn65dsi86_dsi2edp)
		return -ENOMEM;

	mutex_init(&sn65dsi86_dsi2edp->lock);
	sn65dsi86_dsi2edp->client_i2c = client;

	sn65dsi86_dsi2edp->regmap = devm_regmap_init_i2c(client,
						&sn65dsi86_regmap_config);
	if (IS_ERR(sn65dsi86_dsi2edp->regmap)) {
		err = PTR_ERR(sn65dsi86_dsi2edp->regmap);
		dev_err(&client->dev,
				"sn65dsi86_dsi2edp: regmap init failed\n");
		return err;
	}

	err = of_dsi2edp_parse_platform_data(client);
	if (err)
		return err;

	if (np) {
		/* TODO. We don't want probe itself for
		 * panels which don't use bridge.
		 * Until this bridge device is registered as a
		 * sub device of /host1x/dsi/panel in device tree,
		 * do no operation in probe in case bridge is not used.
		 * The reason to prepare this step to check with
		 * dsi2edp-bridge property is to consider
		 * the case that probe contains any actual operation.
		 */
		pri_pn =
			tegra_primary_panel_get_dt_node(NULL);
		sec_pn =
			tegra_secondary_panel_get_dt_node(NULL);
		if (pri_pn) {
			pri_bridge =
				of_property_read_bool(pri_pn,
				"nvidia,dsi-edp-bridge");
			of_node_put(pri_pn);
		}
		if (sec_pn) {
			sec_bridge = of_property_read_bool(sec_pn,
				"nvidia,dsi-edp-bridge");
			of_node_put(sec_pn);
		};

		if (!pri_bridge && !sec_bridge)
			return 0;
	}

	return 0;
}

static int sn65dsi86_i2c_remove(struct i2c_client *client)
{
	sn65dsi86_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id sn65dsi86_id_table[] = {
	{"sn65dsi86_dsi2edp", 0},
	{},
};

static const struct of_device_id sn65dsi86_dt_match[] = {
	{ .compatible = "ti,sn65dsi86" },
	{ }
};

static struct i2c_driver sn65dsi86_i2c_drv = {
	.driver = {
		.name = "sn65dsi86_dsi2edp",
		.of_match_table = of_match_ptr(sn65dsi86_dt_match),
		.owner = THIS_MODULE,
	},
	.probe = sn65dsi86_i2c_probe,
	.remove = sn65dsi86_i2c_remove,
	.id_table = sn65dsi86_id_table,
};

static int __init sn65dsi86_i2c_client_init(void)
{
	int err = 0;

	err = i2c_add_driver(&sn65dsi86_i2c_drv);
	if (err)
		pr_err("sn65dsi86_dsi2edp: Failed to add i2c client driver\n");

	return err;
}

static void __exit sn65dsi86_i2c_client_exit(void)
{
	i2c_del_driver(&sn65dsi86_i2c_drv);
}

subsys_initcall(sn65dsi86_i2c_client_init);
module_exit(sn65dsi86_i2c_client_exit);

MODULE_AUTHOR("Bibek Basu <bbasu@nvidia.com>");
MODULE_DESCRIPTION(" TI SN65DSI86 dsi bridge to edp");
MODULE_LICENSE("GPL");
