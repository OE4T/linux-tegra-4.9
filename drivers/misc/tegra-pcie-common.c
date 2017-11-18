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

#include <linux/types.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/reset.h>

#define APPL_COMMON_CONTROL	0x0
#define APPL_COMMON_CONTROL_XBAR_CONFIG_SHIFT	24

struct tegra_pcie_common {
	struct device *dev;
	void __iomem		*common_base;
	struct reset_control	*common_apb_rst;
};

static int tegra_pcie_common_probe(struct platform_device *pdev)
{
	struct tegra_pcie_common *pcie_xbar;
	struct resource *common_res;
	u32 val;
	int err;

	pcie_xbar = devm_kzalloc(&pdev->dev, sizeof(*pcie_xbar), GFP_KERNEL);
	if (!pcie_xbar)
		return -ENOMEM;
	pcie_xbar->dev = &pdev->dev;
	platform_set_drvdata(pdev, pcie_xbar);

	common_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						  "common");
	if (!common_res) {
		dev_err(&pdev->dev, "missing common reg space\n");
		return PTR_ERR(common_res);
	}
	pcie_xbar->common_base = devm_ioremap_resource(&pdev->dev, common_res);
	if (IS_ERR(pcie_xbar->common_base)) {
		dev_err(&pdev->dev, "mapping common reg space failed\n");
		return PTR_ERR(pcie_xbar->common_base);
	}

	/* Program XBAR configuration */
	err = of_property_read_u32(pdev->dev.of_node, "nvidia,xbar", &val);
	if (err) {
		dev_err(&pdev->dev, "missing XBAR configuration\n");
		return err;
	}

	pcie_xbar->common_apb_rst = devm_reset_control_get(pcie_xbar->dev,
							  "common_apb_rst");
	if (IS_ERR(pcie_xbar->common_apb_rst)) {
		dev_err(pcie_xbar->dev, "common_apb_rst reset is missing\n");
		return PTR_ERR(pcie_xbar->common_apb_rst);
	}
	reset_control_deassert(pcie_xbar->common_apb_rst);

	writel(val << APPL_COMMON_CONTROL_XBAR_CONFIG_SHIFT,
	       pcie_xbar->common_base + APPL_COMMON_CONTROL);

	return 0;
}

static int tegra_pcie_common_remove(struct platform_device *pdev)
{
	struct tegra_pcie_common *pcie_xbar = platform_get_drvdata(pdev);

	reset_control_assert(pcie_xbar->common_apb_rst);
	return 0;
}

static const struct of_device_id tegra_pcie_common_id_table[] = {
	{ .compatible = "nvidia,tegra194-pcie-common" },
	{}
};
MODULE_DEVICE_TABLE(of, tegra_pcie_common_id_table);

#ifdef CONFIG_PM
static int tegra_pcie_common_suspend(struct device *dev)
{
	struct tegra_pcie_common *pcie_xbar = dev_get_drvdata(dev);

	reset_control_assert(pcie_xbar->common_apb_rst);
	return 0;
}

static int tegra_pcie_common_resume(struct device *dev)
{
	struct tegra_pcie_common *pcie_xbar = dev_get_drvdata(dev);

	reset_control_deassert(pcie_xbar->common_apb_rst);
	return 0;
}

static const struct dev_pm_ops tegra_pcie_common_pm_ops = {
	.suspend = tegra_pcie_common_suspend,
	.resume = tegra_pcie_common_resume,
};
#endif /* CONFIG_PM */

static struct platform_driver tegra_pcie_common_driver = {
	.probe		= tegra_pcie_common_probe,
	.remove		= tegra_pcie_common_remove,
	.driver		= {
		.name	= "tegra_pcie_common",
		.of_match_table = of_match_ptr(tegra_pcie_common_id_table),
#ifdef CONFIG_PM
		.pm    = &tegra_pcie_common_pm_ops,
#endif
	},
};

module_platform_driver(tegra_pcie_common_driver);

MODULE_AUTHOR("Vidya Sagar <vidyas@nvidia.com>");
MODULE_DESCRIPTION("Nvidia PCIe Common XBAR controller driver");
MODULE_LICENSE("GPL v2");
