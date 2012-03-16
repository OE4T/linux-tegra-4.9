/*
 * drivers/video/tegra/dc/dsi2lvds.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
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

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <mach/dc.h>

#include "dc_priv.h"
#include "dsi2lvds.h"

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};

#define DSI2LVDS_REG_VAL(addr, val)	(addr), (val)

static u8 dsi2lvds_config_clk[] = {
	DSI2LVDS_REG_VAL(0x0d, 0x00), /* pLL disable */
	DSI2LVDS_REG_VAL(0x0a, 0x00), /* configure PLL */
	DSI2LVDS_REG_VAL(0x0b, 0x00), /* configure PLL */
	DSI2LVDS_REG_VAL(0x0d, 0x01), /* pLL enable */
};

static u8 dsi2lvds_config_dsi[] = {
	DSI2LVDS_REG_VAL(0x10, 0x80), /* default left right ganged mode */
};

static u8 dsi2lvds_config_lvds[] = {
	DSI2LVDS_REG_VAL(0x18, 0x70),
};

static u8 dsi2lvds_config_video[] = {
	DSI2LVDS_REG_VAL(0x20, 0x40),/* horizontal pixels on dsi channel A */
	DSI2LVDS_REG_VAL(0x21, 0x01),
	DSI2LVDS_REG_VAL(0x22, 0x40),/* horizontal pixels on dsi channel B */
	DSI2LVDS_REG_VAL(0x23, 0x01),
	DSI2LVDS_REG_VAL(0x24, 0xe0),/* vertical pixels on lvds channel A */
	DSI2LVDS_REG_VAL(0x25, 0x01),
	DSI2LVDS_REG_VAL(0x26, 0x00),/* vertical pixels on lvds channel B */
	DSI2LVDS_REG_VAL(0x27, 0x00),
	DSI2LVDS_REG_VAL(0x28, 0x00),/* Pixel clk delay from dsi to */
	DSI2LVDS_REG_VAL(0x29, 0x01),/* lvds channel A */
	DSI2LVDS_REG_VAL(0x2a, 0xe0),/* Pixel clk delay from dsi to */
	DSI2LVDS_REG_VAL(0x2b, 0x00),/* lvds channel B */
	DSI2LVDS_REG_VAL(0x2c, 0x60),/* hsync width channel A */
	DSI2LVDS_REG_VAL(0x2d, 0x00),
	DSI2LVDS_REG_VAL(0x2e, 0x00),/* hsync width channel B */
	DSI2LVDS_REG_VAL(0x2f, 0x00),
	DSI2LVDS_REG_VAL(0x30, 0x02),/* vsync width channel A */
	DSI2LVDS_REG_VAL(0x31, 0x00),
	DSI2LVDS_REG_VAL(0x32, 0x00),/* vsync width channel B */
	DSI2LVDS_REG_VAL(0x33, 0x00),
	DSI2LVDS_REG_VAL(0x34, 0x30),/* h back porch channel A */
	DSI2LVDS_REG_VAL(0x35, 0x00),/* h back porch channel B */
};

static u8 dsi2lvds_soft_reset[] = {
	DSI2LVDS_REG_VAL(0x09, 0x01),
};

static struct i2c_driver tegra_dc_dsi2lvds_i2c_slave_driver = {
	.driver = {
		.name = "dsi2lvds_bridge",
	},
};

static struct i2c_client *init_i2c_slave(struct tegra_dc *dc)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info p_data = {
		.type = "dsi2lvds_bridge",
		.addr = 0x2D,
	};
	int bus; /* TODO: Get i2c bus */
	int err = 0;

	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		dev_err(&dc->ndev->dev,
			"dsi2lvds: can't get adpater for bus %d\n", bus);
		err = -EBUSY;
		goto err;
	}

	client = i2c_new_device(adapter, &p_data);
	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&dc->ndev->dev,
			"dsi2lvds: can't add i2c slave device\n");
		err = -EBUSY;
		goto err;
	}

	err = i2c_add_driver(&tegra_dc_dsi2lvds_i2c_slave_driver);
	if (err) {
		dev_err(&dc->ndev->dev,
			"dsi2lvds: can't add i2c slave driver\n");
		goto err_free;
	}

	return client;
err:
	return ERR_PTR(err);
err_free:
	i2c_unregister_device(client);
	return ERR_PTR(err);
}

static int tegra_dc_dsi2lvds_init(struct tegra_dc *dc)
{
	int err = 0;
	struct tegra_dc_dsi2lvds_data *dsi2lvds;

	dsi2lvds = devm_kzalloc(&dc->ndev->dev, sizeof(*dsi2lvds), GFP_KERNEL);
	if (!dsi2lvds)
		return -ENOMEM;

	dsi2lvds->client_i2c = init_i2c_slave(dc);
	if (IS_ERR_OR_NULL(dsi2lvds->client_i2c)) {
		dev_err(&dc->ndev->dev,
			"dsi2lvds: i2c slave setup failure\n");
	}

	dsi2lvds->dc = dc;

	tegra_dc_dsi_set_outdata(dc, dsi2lvds);

	mutex_init(&dsi2lvds->lock);

	return err;
}

static void tegra_dc_dsi2lvds_destroy(struct tegra_dc *dc)
{
	struct tegra_dc_dsi2lvds_data *dsi2lvds = tegra_dc_dsi_get_outdata(dc);

	mutex_lock(&dsi2lvds->lock);
	i2c_del_driver(&tegra_dc_dsi2lvds_i2c_slave_driver);
	i2c_unregister_device(dsi2lvds->client_i2c);
	mutex_unlock(&dsi2lvds->lock);
	mutex_destroy(&dsi2lvds->lock);
}

static int dsi2lvds_i2c_transfer(struct tegra_dc_dsi2lvds_data *dsi2lvds,
					u8 transfers[], u32 no_of_tranfers,
					enum i2c_transfer_type type)
{
	struct i2c_msg *i2c_msg_transfer;
	struct i2c_client *client = dsi2lvds->client_i2c;
	int err = 0;
	u32 cnt = 0;

	i2c_msg_transfer = kzalloc
		(no_of_tranfers * sizeof(*i2c_msg_transfer), GFP_KERNEL);
	if (!i2c_msg_transfer)
		return -ENOMEM;

	for (cnt = 0; cnt < no_of_tranfers; cnt++) {
		i2c_msg_transfer[cnt].addr = client->addr;
		i2c_msg_transfer[cnt].flags = type;
		i2c_msg_transfer[cnt].len = 1;
		i2c_msg_transfer[cnt].buf = &transfers[cnt];
	}

	for (cnt = 0; cnt < no_of_tranfers; cnt += 2) {
		err = i2c_transfer(client->adapter, &i2c_msg_transfer[cnt], 1);
		if (err < 0) {
			dev_err(&dsi2lvds->dc->ndev->dev,
				"dsi2lvds: i2c transfer failed\n");
			break;
		}

		err =
		i2c_transfer(client->adapter, &i2c_msg_transfer[cnt + 1], 1);
		if (err < 0) {
			dev_err(&dsi2lvds->dc->ndev->dev,
				"dsi2lvds: i2c transfer failed\n");
			break;
		}

		mdelay(10);
	}

	kfree(i2c_msg_transfer);
	return err;
}

static void tegra_dc_dsi2lvds_enable(struct tegra_dc *dc)
{
	struct tegra_dc_dsi2lvds_data *dsi2lvds = tegra_dc_dsi_get_outdata(dc);
	int err = 0;

	mutex_lock(&dsi2lvds->lock);

	err = dsi2lvds_i2c_transfer(dsi2lvds, dsi2lvds_config_clk,
			ARRAY_SIZE(dsi2lvds_config_clk), I2C_WRITE);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi2lvds: Init clk failed\n");
		goto fail;
	}

	if (dc->out->dsi->ganged_mode_type == DSI2LVDS_ODD_EVEN) {
		u32 cnt;
		for (cnt = 0;
		cnt < ARRAY_SIZE(dsi2lvds_config_dsi); cnt += 2) {
			if (dsi2lvds_config_dsi[cnt] == 0x10)
				/* select odd even ganged mode */
				dsi2lvds_config_dsi[cnt + 1] &= ~(1 << 7);
		}
	}
	err = dsi2lvds_i2c_transfer(dsi2lvds, dsi2lvds_config_dsi,
			ARRAY_SIZE(dsi2lvds_config_dsi), I2C_WRITE);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi2lvds: Init dsi failed\n");
		goto fail;
	}

	err = dsi2lvds_i2c_transfer(dsi2lvds, dsi2lvds_config_lvds,
			ARRAY_SIZE(dsi2lvds_config_lvds), I2C_WRITE);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi2lvds: Init lvds failed\n");
		goto fail;
	}

	err = dsi2lvds_i2c_transfer(dsi2lvds, dsi2lvds_config_video,
			ARRAY_SIZE(dsi2lvds_config_video), I2C_WRITE);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi2lvds: Init video failed\n");
		goto fail;
	}

	err = dsi2lvds_i2c_transfer(dsi2lvds, dsi2lvds_soft_reset,
			ARRAY_SIZE(dsi2lvds_soft_reset), I2C_WRITE);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "dsi2lvds: Soft reset failed\n");
		goto fail;
	}
fail:
	mutex_unlock(&dsi2lvds->lock);
}

static void tegra_dc_dsi2lvds_disable(struct tegra_dc *dc)
{
	/* To be done */
}

#ifdef CONFIG_PM
static void tegra_dc_dsi2lvds_suspend(struct tegra_dc *dc)
{
	/* To be done */
}

static void tegra_dc_dsi2lvds_resume(struct tegra_dc *dc)
{
	/* To be done */
}
#endif

struct tegra_dc_out_ops tegra_dc_dsi2lvds_ops = {
	.init = tegra_dc_dsi2lvds_init,
	.destroy = tegra_dc_dsi2lvds_destroy,
	.enable = tegra_dc_dsi2lvds_enable,
	.disable = tegra_dc_dsi2lvds_disable,
#ifdef CONFIG_PM
	.suspend = tegra_dc_dsi2lvds_suspend,
	.resume = tegra_dc_dsi2lvds_resume,
#endif
};
