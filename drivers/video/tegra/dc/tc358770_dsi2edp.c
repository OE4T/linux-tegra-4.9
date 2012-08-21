/*
 * drivers/video/tegra/dc/tc358770_dsi2edp.c
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
#include <linux/regmap.h>

#include <mach/dc.h>

#include "dc_priv.h"
#include "tc358770_dsi2edp.h"
#include "dsi.h"

static struct tegra_dc_dsi2edp_data *tc358770_dsi2edp;
static struct i2c_client *tc358770_i2c_client;

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};

static const struct regmap_config tc358770_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int tc358770_dsi2edp_init(struct tegra_dc_dsi_data *dsi)
{
	int err = 0;

	if (tc358770_dsi2edp) {
		tegra_dsi_set_outdata(dsi, tc358770_dsi2edp);
		return err;
	}

	tc358770_dsi2edp = devm_kzalloc(&dsi->dc->ndev->dev,
					sizeof(*tc358770_dsi2edp),
					GFP_KERNEL);
	if (!tc358770_dsi2edp)
		return -ENOMEM;

	tc358770_dsi2edp->dsi = dsi;
	tc358770_dsi2edp->client_i2c = tc358770_i2c_client;
	tc358770_dsi2edp->regmap = devm_regmap_init_i2c(tc358770_i2c_client,
						&tc358770_regmap_config);
	if (IS_ERR(tc358770_dsi2edp->regmap)) {
		err = PTR_ERR(tc358770_dsi2edp->regmap);
		dev_err(&dsi->dc->ndev->dev,
				"tc358770_dsi2edp: regmap init failed\n");
		goto fail;
	}

	tegra_dsi_set_outdata(dsi, tc358770_dsi2edp);

	mutex_init(&tc358770_dsi2edp->lock);

fail:
	return err;
}

static void tc358770_dsi2edp_destroy(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp =
				tegra_dsi_get_outdata(dsi);

	if (!dsi2edp)
		return;

	tc358770_dsi2edp = NULL;
	mutex_destroy(&dsi2edp->lock);
}

static void tc358770_dsi2edp_enable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	if (dsi2edp && dsi2edp->dsi2edp_enabled)
		return;

	mutex_lock(&dsi2edp->lock);

	/* TODO: Config bridge, use regmap_write */

	dsi2edp->dsi2edp_enabled = true;
	mutex_unlock(&dsi2edp->lock);
}

static void tc358770_dsi2edp_disable(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}

#ifdef CONFIG_PM
static void tc358770_dsi2edp_suspend(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	dsi2edp->dsi2edp_enabled = false;

	/* To be done */
}

static void tc358770_dsi2edp_resume(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}
#endif

struct tegra_dsi_out_ops tegra_dsi2edp_ops = {
	.init = tc358770_dsi2edp_init,
	.destroy = tc358770_dsi2edp_destroy,
	.enable = tc358770_dsi2edp_enable,
	.disable = tc358770_dsi2edp_disable,
#ifdef CONFIG_PM
	.suspend = tc358770_dsi2edp_suspend,
	.resume = tc358770_dsi2edp_resume,
#endif
};

static int tc358770_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	tc358770_i2c_client = client;

	return 0;
}

static int tc358770_i2c_remove(struct i2c_client *client)
{
	tc358770_i2c_client = NULL;

	return 0;
}

static struct i2c_driver tc358770_i2c_drv = {
	.driver = {
		.name = "tc358770_dsi2edp",
	},
	.probe = tc358770_i2c_probe,
	.remove = tc358770_i2c_remove,
};

static int __init tc358770_i2c_client_init(void)
{
	int err = 0;

	err = i2c_add_driver(&tc358770_i2c_drv);
	if (err)
		pr_err("tc358770_dsi2edp: Failed to add i2c client driver\n");

	return err;
}

static void __exit tc358770_i2c_client_exit(void)
{
	i2c_del_driver(&tc358770_i2c_drv);
}

module_init(tc358770_i2c_client_init);
module_exit(tc358770_i2c_client_exit);
