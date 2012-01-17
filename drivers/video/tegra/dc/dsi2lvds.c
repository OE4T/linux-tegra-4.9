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

#include "dc_priv.h"
#include "dsi2lvds.h"

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

	dsi2lvds->client = init_i2c_slave(dc);
	if (IS_ERR_OR_NULL(dsi2lvds->client)) {
		dev_err(&dc->ndev->dev,
			"dsi2lvds: i2c slave setup failure\n");
	}

	dsi2lvds->dc = dc;
	dsi2lvds->dsi = tegra_dc_get_outdata(dc);

	mutex_init(&dsi2lvds->lock);

	return err;
}

static void tegra_dc_dsi2lvds_destroy(struct tegra_dc *dc)
{
	struct tegra_dc_dsi2lvds_data *dsi2lvds = tegra_dc_dsi_get_outdata(dc);

	mutex_lock(&dsi2lvds->lock);
	i2c_del_driver(&tegra_dc_dsi2lvds_i2c_slave_driver);
	i2c_unregister_device(dsi2lvds->client);
	mutex_unlock(&dsi2lvds->lock);
	mutex_destroy(&dsi2lvds->lock);
}

static void tegra_dc_dsi2lvds_enable(struct tegra_dc *dc)
{
	/* To be done */
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
