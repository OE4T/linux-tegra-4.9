/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/of_platform.h>
#include <linux/mailbox_controller.h>
#include <linux/tegra_ast.h>
#include <linux/tegra-ivc-bus.h>

#define NV(p) "nvidia," #p

#define TEGRA_IVC_BUS_HSP_DATA_ARRAY_SIZE	3

struct tegra_hsp_doorbell_priv {
	struct platform_device *hsp;
	enum tegra_hsp_master master;
	enum tegra_hsp_doorbell doorbell;
};

static int tegra_hsp_doorbell_probe(struct device *dev)
{
	struct tegra_hsp_doorbell_priv *db;
	struct platform_device *hsp;
	struct of_phandle_args dbspec;
	int ret;

	ret = of_parse_phandle_with_fixed_args(dev->of_node,
						NV(hsp-notifications), 2, 0,
						&dbspec);
	if (ret)
		return ret;

	db = devm_kmalloc(dev, sizeof(*db), GFP_KERNEL);
	if (unlikely(db == NULL)) {
		ret = -ENOMEM;
		goto out;
	}

	hsp = of_find_device_by_node(dbspec.np);
	if (hsp == NULL) {
		ret = -EPROBE_DEFER;
		goto out;
	}

	ret = sysfs_create_link(&dev->kobj, &hsp->dev.kobj, "hsp");
	if (ret) {
		platform_device_put(hsp);
		goto out;
	}

	db->hsp = hsp;
	db->master = dbspec.args[0];
	db->doorbell = dbspec.args[1];
	dev_set_drvdata(dev, db);
out:
	of_node_put(dbspec.np);
	return ret;
}

static void tegra_hsp_doorbell_remove(struct device *dev)
{
	struct tegra_hsp_doorbell_priv *db = dev_get_drvdata(dev);

	sysfs_delete_link(&dev->kobj, &db->hsp->dev.kobj, "hsp");
	platform_device_put(db->hsp);
}

static void tegra_hsp_doorbell_notify(void *data)
{
	struct device *dev = data;

	tegra_hsp_notify(dev);
}

static int tegra_hsp_doorbell_enable(struct device *dev)
{
	struct tegra_hsp_doorbell_priv *db = dev_get_drvdata(dev);
	int ret;

	/* Register for notifications from remote processor */
	ret = tegra_hsp_db_add_handler(db->master, tegra_hsp_doorbell_notify,
					dev);
	if (ret)
		dev_err(dev, "HSP doorbell %s error: %d\n", "register", ret);

	/* Allow remote processor to ring */
	ret = tegra_hsp_db_enable_master(db->master);
	if (ret)
		dev_err(dev, "HSP doorbell %s error: %d\n", "enable", ret);

	return ret;
}

static void tegra_hsp_doorbell_disable(struct device *dev)
{
	struct tegra_hsp_doorbell_priv *db = dev_get_drvdata(dev);
	int ret;

	ret = tegra_hsp_db_disable_master(db->master);
	if (ret)
		dev_err(dev, "HSP doorbell %s error: %d\n", "disable", ret);

	ret = tegra_hsp_db_del_handler(db->master);
	if (ret) {
		WARN_ON(1);
		dev_err(dev, "HSP doorbell %s error: %d\n", "deregister", ret);
	}
}

static void tegra_hsp_doorbell_ring(struct device *dev)
{
	struct tegra_hsp_doorbell_priv *db = dev_get_drvdata(dev);
	int ret = tegra_hsp_db_ring(db->doorbell);
	if (ret)
		dev_err(dev, "HSP doorbell %s error: %d\n", "ring", ret);
}

static const struct tegra_hsp_ops tegra_hsp_doorbell_ops = {
	.probe		= tegra_hsp_doorbell_probe,
	.remove		= tegra_hsp_doorbell_remove,
	.enable		= tegra_hsp_doorbell_enable,
	.disable	= tegra_hsp_doorbell_disable,
	.ring		= tegra_hsp_doorbell_ring,
};

static struct tegra_ivc_driver tegra_hsp_doorbell_driver = {
	.driver = {
		.name		= "tegra-hsp-doorbell",
		.bus		= &tegra_ivc_bus_type,
		.owner		= THIS_MODULE,
	},
	.dev_type	= &tegra_hsp_type,
	.ops.hsp	= &tegra_hsp_doorbell_ops,
};
tegra_ivc_module_driver(tegra_hsp_doorbell_driver);
MODULE_DESCRIPTION("NVIDIA Tegra HSP doorbell driver");
MODULE_LICENSE("GPL");
