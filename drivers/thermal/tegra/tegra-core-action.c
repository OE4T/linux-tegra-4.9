/*
 * tegra-core-action.c - connect Tegra CORE DVFS driver to thermal framework
 *
 * Copyright (C) 2012-2014 NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk-provider.h>

#include <soc/tegra/tegra-dvfs.h>

#define CORE_CDEV_TYPE_FLOOR	"CORE-floor"
#define CORE_CDEV_TYPE_CAP	"CORE-cap"
/**
 * struct tegra_core_cdev_data - CORE DVFS cooling device info
 * @cdev_floor: pointer to floor cooling device, after thermal registration
 * @cdev_cap: pointer to cap cooling device, after thermal registration
 * device
 */
struct tegra_core_cdev_data {
	struct thermal_cooling_device *cdev_floor;
	struct thermal_cooling_device *cdev_cap;
};

static struct tegra_core_cdev_data tegra_core_cdev_data;

/*
 * Thermal cooling device interface
 */

static int
tegra_core_cdev_get_max_state(struct thermal_cooling_device *cdev,
			      unsigned long *max_state)
{
	enum tegra_dvfs_core_thermal_type type;
	int num;

	type = (enum tegra_dvfs_core_thermal_type)cdev->devdata;
	num = tegra_dvfs_core_count_thermal_states(type);
	if (num <= 0)
		*max_state = 0;
	else
		*max_state = num;

	return 0;
}

static int
tegra_core_cdev_get_cur_state(struct thermal_cooling_device *cdev,
			      unsigned long *cur_state)
{
	enum tegra_dvfs_core_thermal_type type;

	type = (enum tegra_dvfs_core_thermal_type)cdev->devdata;
	*cur_state = tegra_dvfs_core_get_thermal_index(type);
	return 0;
}

static int
tegra_core_cdev_set_state(struct thermal_cooling_device *cdev,
			  unsigned long cur_state)
{
	enum tegra_dvfs_core_thermal_type type;

	type = (enum tegra_dvfs_core_thermal_type)cdev->devdata;
	return tegra_dvfs_core_update_thermal_index(type, cur_state);
}

static struct thermal_cooling_device_ops tegra_core_cooling_ops = {
	.get_max_state = tegra_core_cdev_get_max_state,
	.get_cur_state = tegra_core_cdev_get_cur_state,
	.set_cur_state = tegra_core_cdev_set_state,
};

static int tegra_core_register_therm(struct platform_device *pdev,
					 enum tegra_dvfs_core_thermal_type type)
{
	struct thermal_cooling_device *tcd;
	struct device *dev = &pdev->dev;
	int ret = 0;

	if (type == TEGRA_DVFS_CORE_THERMAL_FLOOR) {
		tcd = thermal_of_cooling_device_register(dev->of_node,
					CORE_CDEV_TYPE_FLOOR,
					(void *)TEGRA_DVFS_CORE_THERMAL_FLOOR,
					&tegra_core_cooling_ops);
		if (IS_ERR_OR_NULL(tcd)) {
			dev_err(dev,
				"Tegra CORE DVFS thermal reaction: failed to register floor\n");
			ret = -EINVAL;
			goto error;
		}
		tegra_core_cdev_data.cdev_floor = tcd;

		dev_info(dev, "Tegra CORE DVFS 'floor cooling device' registered\n");
	} else if (type == TEGRA_DVFS_CORE_THERMAL_CAP) {
		tcd = thermal_of_cooling_device_register(dev->of_node,
					CORE_CDEV_TYPE_CAP,
					(void *)TEGRA_DVFS_CORE_THERMAL_CAP,
					&tegra_core_cooling_ops);
		if (IS_ERR_OR_NULL(tcd)) {
			dev_err(dev,
				"Tegra CORE DVFS thermal reaction: failed to register cap\n");
			ret = -EINVAL;
			goto error;
		}
		tegra_core_cdev_data.cdev_cap = tcd;

		dev_info(dev, "Tegra CORE DVFS 'cap cooling device' registered\n");
	} else {
		ret = -EINVAL;
	}

error:
	return ret;
}

static int tegra_core_cdev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *cdev_type;
	int ret;


	ret = tegra_dvfs_core_count_thermal_states(
						TEGRA_DVFS_CORE_THERMAL_FLOOR);
	if (ret < 0)
		return -EPROBE_DEFER;

	cdev_type = of_get_property(dev->of_node, "cdev-type", NULL);
	if (IS_ERR_OR_NULL(cdev_type)) {
		dev_err(dev,
			"Tegra CORE DVFS thermal reaction: missing cdev type\n");
		ret = -ENOENT;
		goto error;
	}

	if (!strcmp(cdev_type, CORE_CDEV_TYPE_FLOOR))
		ret = tegra_core_register_therm(pdev,
						TEGRA_DVFS_CORE_THERMAL_FLOOR);
	else if (!strcmp(cdev_type, CORE_CDEV_TYPE_CAP))
		ret = tegra_core_register_therm(pdev,
						TEGRA_DVFS_CORE_THERMAL_CAP);
	else {
		ret = -ENOENT;
		dev_err(dev, "Tegra CORE DVFS thermal reaction: incorrect cdev type\n");
	}

error:
	return ret;
}

static int tegra_core_cdev_remove(struct platform_device *pdev)
{
	if (tegra_core_cdev_data.cdev_floor)
		thermal_cooling_device_unregister(
				tegra_core_cdev_data.cdev_floor);
	if (tegra_core_cdev_data.cdev_cap)
		thermal_cooling_device_unregister(
				tegra_core_cdev_data.cdev_cap);

	return 0;
}

static const struct of_device_id tegra_core_cdev_match[] = {
	{ .compatible = "nvidia,tegra-core-cdev-action", },
	{},
};

static struct platform_driver tegra_core_cdev_driver = {
	.driver = {
		.name   = "tegra_core_action",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_core_cdev_match,
	},
	.probe = tegra_core_cdev_probe,
	.remove = tegra_core_cdev_remove,
};

module_platform_driver(tegra_core_cdev_driver);

MODULE_DESCRIPTION("Tegra CORE DVFS thermal reaction driver");
MODULE_LICENSE("GPL");
