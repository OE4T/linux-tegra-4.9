/*
 * isl9238_charger.c -- ISL9238 Narrow o/p voltage DC charger driver
 *
 * Copyright (C) 2017 NVIDIA CORPORATION. All rights reserved.
 *
 * Author: Venkat Reddy Talla <vreddytalla@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

/* Register definitions */
#define ISL9238_CHG_CURR_LIMIT	0x14
#define ISL9238_MAX_SYS_VOLTAGE	0x15
#define ISL9238_T1_T2_TWO_LEVEL	0x38
#define ISL9238_CONTROL0_OPTIONS	0x39
#define ISL9238_INFO1_CHG_STATUS	0x3A
#define ISL9238_ADPTR_CURR_LIMIT2	0x3B
#define ISL9238_CONTROL1_OPTIONS	0x3C
#define ISL9238_CONTROL2_OPTIONS	0x3D
#define ISL9238_MIN_SYS_VOLTAGE	0x3E
#define ISL9238_ADPTR_CURR_LIMIT1	0x3F
#define ISL9238_AC_PROCHOT		0x47
#define ISL9238_DC_PROCHOT		0x48
#define ISL9238_REG_OTG_VOLTAGE	0x49
#define ISL9238_REG_OTG_CURRENT	0x4A
#define ISL9238_INPUT_VOLTAGE	0x4B
#define ISL9238_CONTROL3_OPTIONS	0x4C
#define ISL9238_INFO2_CHARGER_STATUS	0x4D
#define ISL9238_CONTROL4_OPTIONS	0x4E
#define ISL9238_MANUFACTURER_ID	0xFE
#define ISL9238_DEVICE_ID	0xFF

#define ISL9238_CHG_CURR_LIMIT_MASK	0x1FFC
#define ISL9238_MAX_SYS_VOLTAGE_MASK	0x7FF8
#define ISL9238_MIN_SYS_VOLTAGE_MASK	0x3F00
#define ISL9238_INPUT_VOLTAGE_MASK		0x3F00
#define ISL9238_ADPTR_CURR_LIMIT1_MASK	0x1FFC
#define ISL9238_ADPTR_CURR_LIMIT2_MASK	0x1FFC
#define ISL9238_AC_PROCHOT_MASK		0x1F80
#define ISL9238_DC_PROCHOT_MASK		0x3F00
#define ISL9238_OTG_MASK			0x800
#define ISL9238_OTG_ENABLE			0x800
#define ISL9238_OTG_DISABLE			0x0
#define ISL9238_REGULATION_MASK		0x4
#define ISL9238_BAT_LEARN_MASK		0x1000
#define ISL9238_TURBO_MODE_MASK		0x40
#define ISL9238_TURBO_MODE_DISABLE		0x40
#define ISL9238_AUTO_CHARGE_MODE_MASK	0x80

static const struct regmap_config isl9238_rmap_config = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.max_register		= ISL9238_DEVICE_ID,
	.cache_type		= REGCACHE_NONE,
};

struct isl9238_vbus_pdata {
	struct regulator_init_data *ridata;
};

struct isl9238_chg_pdata {
	struct isl9238_vbus_pdata *vbus_pdata;
	u32 charge_current_lim;
	u32 max_sys_voltage;
	u32 min_sys_voltage;
	u32 input_voltage_limit;
	u32 adapter_current_lim1;
	u32 adapter_current_lim2;
	u32 acprochot_threshold;
	u32 dcprochot_threshold;
	u32 adapter_duration_t1;
	u32 adapter_duration_t2;
	bool disable_input_regulation;
	bool enable_bat_learn_mode;
	bool disable_turbo_mode;
	bool enable_auto_charging;
};

struct isl9238_charger {
	struct device		*dev;
	struct i2c_client	*client;
	struct regmap		*rmap;
	int			irq;

	struct mutex			mutex;
	struct mutex			otg_mutex;

	struct isl9238_chg_pdata *chg_pdata;

	struct regulator_dev		*vbus_rdev;
	struct regulator_desc	vbus_reg_desc;
	struct regulator_init_data	*vbus_ridata;
	struct device_node		*vbus_np;
	bool	is_otg_connected;
};

static int isl9238_val_to_reg(int val, int offset, int div, int nbits,
	bool roundup)
{
	int max_val = offset + (BIT(nbits) - 1) * div;

	if (val <= offset)
		return 0;

	if (val >= max_val)
		return BIT(nbits) - 1;

	if (roundup)
		return DIV_ROUND_UP(val - offset, div);
	else
		return (val - offset) / div;
}

static int isl9238_otg_enable(struct regulator_dev *rdev)
{
	struct isl9238_charger *isl9238 = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&isl9238->otg_mutex);
	isl9238->is_otg_connected = true;

	ret = regmap_update_bits(isl9238->rmap, ISL9238_CONTROL1_OPTIONS,
			ISL9238_OTG_MASK, ISL9238_OTG_ENABLE);
	if (ret < 0) {
		dev_err(isl9238->dev, "OTG enable failed %d", ret);
		mutex_unlock(&isl9238->otg_mutex);
		return ret;
	}
	mutex_unlock(&isl9238->otg_mutex);

	dev_info(isl9238->dev, "OTG enabled\n");
	return ret;
}

static int isl9238_otg_disable(struct regulator_dev *rdev)
{
	struct isl9238_charger *isl9238 = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&isl9238->otg_mutex);

	isl9238->is_otg_connected = false;
	ret = regmap_update_bits(isl9238->rmap, ISL9238_CONTROL1_OPTIONS,
			ISL9238_OTG_MASK, ISL9238_OTG_DISABLE);
	if (ret < 0) {
		dev_err(isl9238->dev, "OTG disable failed %d", ret);
		mutex_unlock(&isl9238->otg_mutex);
		return ret;
	}
	mutex_unlock(&isl9238->otg_mutex);

	return ret;
}

static int isl9238_otg_is_enabled(struct regulator_dev *rdev)
{
	struct isl9238_charger *isl9238 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	ret = regmap_read(isl9238->rmap, ISL9238_CONTROL1_OPTIONS, &val);
	if (ret < 0) {
		dev_err(isl9238->dev, "CONTROL1 read failed %d", ret);
		return ret;
	}
	return (val & ISL9238_OTG_MASK) == ISL9238_OTG_ENABLE;
}

static struct regulator_ops isl9238_otg_ops = {
	.enable	= isl9238_otg_enable,
	.disable	= isl9238_otg_disable,
	.is_enabled	= isl9238_otg_is_enabled,
};

static struct regulator_desc isl9238_otg_reg_desc = {
	.name		= "isl9238-vbus",
	.ops		= &isl9238_otg_ops,
	.type		= REGULATOR_VOLTAGE,
	.enable_time	= 220000,
	.owner		= THIS_MODULE,
};

static int isl9238_init_vbus_regulator(struct isl9238_charger *isl9238,
		struct isl9238_chg_pdata *pdata)
{
	int ret = 0;
	struct regulator_config rconfig = { };

	if (!pdata->vbus_pdata) {
		dev_err(isl9238->dev, "No vbus platform data\n");
		return 0;
	}

	isl9238->vbus_reg_desc = isl9238_otg_reg_desc;
	isl9238->vbus_ridata = pdata->vbus_pdata->ridata;
	isl9238->vbus_ridata->constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;
	isl9238->vbus_ridata->constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_VOLTAGE;

	/* Register vbus regulator */
	rconfig.dev = isl9238->dev;
	rconfig.of_node =  isl9238->vbus_np;
	rconfig.init_data = isl9238->vbus_ridata;
	rconfig.driver_data = isl9238;
	isl9238->vbus_rdev = devm_regulator_register(isl9238->dev,
				&isl9238->vbus_reg_desc, &rconfig);
	if (IS_ERR(isl9238->vbus_rdev)) {
		ret = PTR_ERR(isl9238->vbus_rdev);
		dev_err(isl9238->dev,
			"VBUS regulator register failed %d\n", ret);
		return ret;
	}
	return 0;
}

static int isl9238_show_chip_version(struct isl9238_charger *isl9238)
{
	int ret;
	unsigned int val;

	ret = regmap_read(isl9238->rmap, ISL9238_MANUFACTURER_ID, &val);
	if (ret < 0) {
		dev_err(isl9238->dev, "REVISION_REG read failed: %d\n", ret);
		return ret;
	}

	dev_info(isl9238->dev, "ISL9238 Manufacture OTP id:0x%02X\n", val);

	ret = regmap_read(isl9238->rmap, ISL9238_DEVICE_ID, &val);
	if (ret < 0) {
		dev_err(isl9238->dev, "REVISION_REG read failed: %d\n", ret);
		return ret;
	}

	dev_info(isl9238->dev, "ISL9238 Device OTP id:0x%02X\n", val);
	return 0;
}

static int isl9238_charger_init(struct isl9238_charger *isl9238)
{
	unsigned int val;
	int ret;

	if (isl9238->chg_pdata->enable_auto_charging) {
		ret = regmap_update_bits(isl9238->rmap,
				ISL9238_CONTROL3_OPTIONS,
				ISL9238_AUTO_CHARGE_MODE_MASK, 0x0);
		if (ret < 0) {
		dev_err(isl9238->dev,
				"auto charge mode enable failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->charge_current_lim) {
		val = isl9238->chg_pdata->charge_current_lim &
					ISL9238_CHG_CURR_LIMIT_MASK;
		ret = regmap_write(isl9238->rmap, ISL9238_CHG_CURR_LIMIT, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"charge current update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->max_sys_voltage) {
		val = isl9238->chg_pdata->max_sys_voltage &
					ISL9238_MAX_SYS_VOLTAGE_MASK;
		ret = regmap_write(isl9238->rmap, ISL9238_MAX_SYS_VOLTAGE, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"max system voltage update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->min_sys_voltage) {
		val = isl9238->chg_pdata->min_sys_voltage &
					ISL9238_MIN_SYS_VOLTAGE_MASK;
		ret = regmap_write(isl9238->rmap, ISL9238_MIN_SYS_VOLTAGE, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"min system voltage update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->input_voltage_limit) {
		val = isl9238_val_to_reg(
			isl9238->chg_pdata->input_voltage_limit,
			0, 341, 6, 0);
		ret = regmap_write(isl9238->rmap, ISL9238_INPUT_VOLTAGE,
						val << 8);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"input voltage update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->adapter_current_lim1) {
		val = isl9238->chg_pdata->adapter_current_lim1 &
					ISL9238_ADPTR_CURR_LIMIT1_MASK;
		ret = regmap_write(isl9238->rmap,
				ISL9238_ADPTR_CURR_LIMIT1, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"adapter current_1 update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->adapter_current_lim2) {
		val = isl9238->chg_pdata->adapter_current_lim2 &
					ISL9238_ADPTR_CURR_LIMIT2_MASK;
		ret = regmap_write(isl9238->rmap,
				ISL9238_ADPTR_CURR_LIMIT2, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"adapter current_2 update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->acprochot_threshold) {
		val = isl9238->chg_pdata->acprochot_threshold &
					ISL9238_AC_PROCHOT_MASK;
		ret = regmap_write(isl9238->rmap, ISL9238_AC_PROCHOT, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"ac Prochot update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->dcprochot_threshold) {
		val = isl9238->chg_pdata->dcprochot_threshold &
					ISL9238_DC_PROCHOT_MASK;
		ret = regmap_write(isl9238->rmap, ISL9238_DC_PROCHOT, val);
		if (ret < 0) {
			dev_err(isl9238->dev,
				"dc Prochot update failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->disable_input_regulation) {
		ret = regmap_update_bits(isl9238->rmap,
				ISL9238_CONTROL0_OPTIONS,
				ISL9238_REGULATION_MASK, 0x0);
		if (ret < 0) {
		dev_err(isl9238->dev,
				"regulation disable failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->enable_bat_learn_mode) {
		ret = regmap_update_bits(isl9238->rmap,
				ISL9238_CONTROL1_OPTIONS,
				ISL9238_BAT_LEARN_MASK, ISL9238_BAT_LEARN_MASK);
		if (ret < 0) {
		dev_err(isl9238->dev,
				"bat learn mode enable failed %d\n", ret);
			return ret;
		}
	}

	if (isl9238->chg_pdata->disable_turbo_mode) {
		ret = regmap_update_bits(isl9238->rmap,
				ISL9238_CONTROL1_OPTIONS,
				ISL9238_TURBO_MODE_MASK,
				ISL9238_TURBO_MODE_DISABLE);
		if (ret < 0) {
		dev_err(isl9238->dev,
				"Turbo mode disable failed %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static struct isl9238_chg_pdata *isl9238_parse_dt_data(
		struct i2c_client *client, struct device_node **vbus_np)
{
	struct device_node *np = client->dev.of_node;
	struct device_node *vbus_reg_node;
	struct isl9238_chg_pdata *pdata;
	u32 pval;
	int ret;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "isl,charge-current-limit", &pval);
	if (!ret)
		pdata->charge_current_lim = pval/1000;

	ret = of_property_read_u32(np, "isl,max-system-voltage", &pval);
	if (!ret)
		pdata->max_sys_voltage = pval/1000;

	ret = of_property_read_u32(np, "isl,min-system-voltage", &pval);
	if (!ret)
		pdata->min_sys_voltage = pval/1000;

	ret = of_property_read_u32(np, "isl,input-voltage-limit", &pval);
	if (!ret)
		pdata->input_voltage_limit = pval/1000;

	ret = of_property_read_u32(np, "isl,adapter-current-limit1", &pval);
	if (!ret)
		pdata->adapter_current_lim1 = pval/1000;

	ret = of_property_read_u32(np, "isl,adapter-current-limit2", &pval);
	if (!ret)
		pdata->adapter_current_lim2 = pval/1000;

	ret = of_property_read_u32(np, "isl,acprochot-threshold", &pval);
	if (!ret)
		pdata->acprochot_threshold = pval;

	ret = of_property_read_u32(np, "isl,dcprochot-threshold", &pval);
	if (!ret)
		pdata->dcprochot_threshold = pval;

	pdata->disable_input_regulation = of_property_read_bool(np,
				"isl,disable-regulation");

	pdata->enable_bat_learn_mode = of_property_read_bool(np,
				"isl,enable-bat-learn-mode");

	pdata->disable_turbo_mode = of_property_read_bool(np,
				"isl,disable-turbo-mode");

	pdata->enable_auto_charging = of_property_read_bool(np,
				"isl,enable-auto-charging");

	vbus_reg_node = of_find_node_by_name(np, "vbus");
	if (vbus_reg_node) {
		pdata->vbus_pdata = devm_kzalloc(&client->dev,
			sizeof(*(pdata->vbus_pdata)), GFP_KERNEL);
		if (!pdata->vbus_pdata)
			return ERR_PTR(-ENOMEM);

		pdata->vbus_pdata->ridata = of_get_regulator_init_data(
					&client->dev, vbus_reg_node,
					&isl9238_otg_reg_desc);
		if (!pdata->vbus_pdata->ridata)
			return ERR_PTR(-EINVAL);
	}

	*vbus_np = vbus_reg_node;

	return pdata;
}

static int isl9238_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct isl9238_charger *isl9238;
	struct isl9238_chg_pdata *pdata = NULL;
	struct device_node *vbus_np = NULL;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	if (client->dev.of_node) {
		pdata = isl9238_parse_dt_data(client, &vbus_np);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			dev_err(&client->dev, "Dts Parsing failed, %d\n", ret);
			return ret;
		}
	}

	isl9238 = devm_kzalloc(&client->dev, sizeof(*isl9238), GFP_KERNEL);
	if (!isl9238)
		return -ENOMEM;

	isl9238->rmap = devm_regmap_init_i2c(client, &isl9238_rmap_config);
	if (IS_ERR(isl9238->rmap)) {
		ret = PTR_ERR(isl9238->rmap);
		dev_err(&client->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	isl9238->dev = &client->dev;
	i2c_set_clientdata(client, isl9238);
	isl9238->client = client;
	isl9238->chg_pdata = pdata;
	isl9238->vbus_np = vbus_np;

	mutex_init(&isl9238->mutex);
	mutex_init(&isl9238->otg_mutex);

	ret = isl9238_show_chip_version(isl9238);
	if (ret < 0) {
		dev_err(&client->dev, "version read failed %d\n", ret);
		goto scrub_mutex;
	}

	ret = isl9238_charger_init(isl9238);
	if (ret < 0) {
		dev_err(&client->dev, "Charger init failed: %d\n", ret);
		goto scrub_mutex;
	}

	ret = isl9238_init_vbus_regulator(isl9238, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "VBUS regulator init failed %d\n", ret);
		goto scrub_mutex;
	}

	return 0;

scrub_mutex:
	mutex_destroy(&isl9238->mutex);
	mutex_destroy(&isl9238->otg_mutex);
	return ret;
}

static const struct of_device_id isl9238_of_match[] = {
	{ .compatible = "isl,isl9238", },
	{},
};
MODULE_DEVICE_TABLE(of, isl9238_of_match);

static const struct i2c_device_id isl9238_id[] = {
	{.name = "isl9238",},
	{},
};

static struct i2c_driver isl9238_i2c_driver = {
	.driver = {
		.name = "isl9238",
		.owner = THIS_MODULE,
		.of_match_table = isl9238_of_match,
	},
	.probe = isl9238_probe,
	.id_table = isl9238_id,
};

static int __init isl9238_module_init(void)
{
	return i2c_add_driver(&isl9238_i2c_driver);
}
subsys_initcall(isl9238_module_init);

static void __exit isl9238_cleanup(void)
{
	i2c_del_driver(&isl9238_i2c_driver);
}
module_exit(isl9238_cleanup);

MODULE_DESCRIPTION("ISL9238 battery charger driver");
MODULE_AUTHOR("Venkat Reddy Talla <vreddytalla@nvidia.com>");
MODULE_LICENSE("GPL v2");
