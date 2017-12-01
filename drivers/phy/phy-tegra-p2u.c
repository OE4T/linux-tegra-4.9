/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_platform.h>

#define P2U_PERIODIC_EQ_CTRL_GEN3	0xc0
#define P2U_PERIODIC_EQ_CTRL_GEN3_INIT_PRESET_EQ_TRAIN_EN	BIT(1)
#define P2U_PERIODIC_EQ_CTRL_GEN4	0xc4
#define P2U_PERIODIC_EQ_CTRL_GEN4_INIT_PRESET_EQ_TRAIN_EN	BIT(1)
#define P2U_SLCG				0xbc
#define P2U_SLCG_MASTER			BIT(0)

struct tegra_p2u {
	void __iomem		*base;
	struct device		*dev;
};

static int tegra_p2u_power_off(struct phy *x)
{
	return 0;
}

static int tegra_p2u_power_on(struct phy *x)
{
	u32 val;
	struct tegra_p2u *phy = phy_get_drvdata(x);

	/* Disable SLCG */
	/* NOTE:- This needs to be removed after initial bringup */
	val = readl(phy->base + P2U_SLCG);
	writel(val | P2U_SLCG_MASTER, phy->base + P2U_SLCG);

	val = readl(phy->base + P2U_PERIODIC_EQ_CTRL_GEN3);
	val |= P2U_PERIODIC_EQ_CTRL_GEN3_INIT_PRESET_EQ_TRAIN_EN;
	writel(val, phy->base + P2U_PERIODIC_EQ_CTRL_GEN3);

	val = readl(phy->base + P2U_PERIODIC_EQ_CTRL_GEN4);
	val |= P2U_PERIODIC_EQ_CTRL_GEN4_INIT_PRESET_EQ_TRAIN_EN;
	writel(val, phy->base + P2U_PERIODIC_EQ_CTRL_GEN4);

	return 0;
}

static int tegra_p2u_init(struct phy *x)
{
	return 0;
}

static int tegra_p2u_exit(struct phy *x)
{
	return 0;
}

static const struct phy_ops ops = {
	.init		= tegra_p2u_init,
	.exit		= tegra_p2u_exit,
	.power_on	= tegra_p2u_power_on,
	.power_off	= tegra_p2u_power_off,
	.owner		= THIS_MODULE,
};

static int tegra_p2u_probe(struct platform_device *pdev)
{
	struct tegra_p2u *phy;
	struct phy *generic_phy;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct resource *res;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	platform_set_drvdata(pdev, phy);

	generic_phy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(generic_phy))
		return PTR_ERR(generic_phy);

	phy_set_drvdata(generic_phy, phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	return 0;
}

static int tegra_p2u_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id tegra_p2u_id_table[] = {
	{
		.compatible = "nvidia,phy-p2u",
	},
	{}
};
MODULE_DEVICE_TABLE(of, tegra_p2u_id_table);

static struct platform_driver tegra_p2u_driver = {
	.probe		= tegra_p2u_probe,
	.remove		= tegra_p2u_remove,
	.driver		= {
		.name	= "tegra-p2u",
		.of_match_table = tegra_p2u_id_table,
	},
};

module_platform_driver(tegra_p2u_driver);

MODULE_AUTHOR("Vidya Sagar <vidyas@nvidia.com>");
MODULE_DESCRIPTION("Nvidia Tegra PIPE_To_UPHY phy driver");
MODULE_LICENSE("GPL v2");
