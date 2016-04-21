/*
 * csi_mipi_cal.c - csi mipi calibration driver
 *
 * Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/tegra-soc.h>
#include <linux/reset.h>
#include <dt-bindings/soc/tegra186-powergate.h>
#include <linux/tegra-powergate.h>
#include "dc_priv.h"

#define DRV_NAME "tegra186_csi_mipical"

#define MIPI_CAL_MIPI_BIAS_PAD_CFG0_0	0x5c
#define MIPI_CAL_MIPI_BIAS_PAD_CFG1_0	0x60
#define MIPI_CAL_MIPI_BIAS_PAD_CFG2_0	0x64
#define MIPI_CAL_DSID_MIPI_CAL_CONFIG_2_0	0x78

struct tegra186_csi_mipical {
	struct clk *clk_mipical;
	struct clk *clk_uart_fs_mipical;
	struct regmap *regmap;
};

static const struct regmap_config tegra186_csi_mipical_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = MIPI_CAL_DSID_MIPI_CAL_CONFIG_2_0,
	.cache_type = REGCACHE_FLAT,
};

static const struct of_device_id tegra186_csi_mipical_of_match[] = {
	{ .compatible = "nvidia, tegra186_csi_mipical", .data = NULL },
	{},
};

static int tegra186_csi_mipical_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device_node *np = pdev->dev.of_node;
	struct tegra186_csi_mipical *csi_mipical;
	struct resource *mem, *memregion;
	void __iomem *regs;
	struct reset_control *rst = NULL;
	int ret = 0;
#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_POWERGATE)
	bool powergate = false;
#endif

	match = of_match_device(tegra186_csi_mipical_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		ret = -ENODEV;
		goto err;
	}

	csi_mipical = devm_kzalloc(&pdev->dev,
		sizeof(struct tegra186_csi_mipical), GFP_KERNEL);
	if (!csi_mipical) {
		dev_err(&pdev->dev, "Can't allocate csi mipical\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, csi_mipical);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err;
	}

	/* Don't return in the error condition since the same memory area can be
		requested by nvdisplay's mipi_cal driver before
		csi_mipical driver probe function called.
		TDB : This csi_mipical driver is temporal driver
		before the common mipi driver
		developed(Tracking : Bug 1686313) */
	memregion = devm_request_mem_region(&pdev->dev, mem->start,
					resource_size(mem), pdev->name);
	if (!memregion)
		dev_info(&pdev->dev, "Memory region already claimed\n");

	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err;
	}

	csi_mipical->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
		&tegra186_csi_mipical_regmap_config);
	if (IS_ERR(csi_mipical->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(csi_mipical->regmap);
		goto err;
	}

	regcache_cache_only(csi_mipical->regmap, false);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		csi_mipical->clk_mipical = of_clk_get_by_name(np, "mipi_cal");
		if (IS_ERR(csi_mipical->clk_mipical)) {
			dev_err(&pdev->dev, "Can't retrieve mipi_cal clock\n");
			ret = PTR_ERR(csi_mipical->clk_mipical);
			goto err;
		}

		csi_mipical->clk_uart_fs_mipical =
			of_clk_get_by_name(np, "uart_fs_mipi_cal");
		if (IS_ERR(csi_mipical->clk_uart_fs_mipical)) {
			dev_err(&pdev->dev, "Can't retrieve uart_fs_mipi_cal clock\n");
			ret = PTR_ERR(csi_mipical->clk_uart_fs_mipical);
			goto err_uart_fs_mipi_cal;
		}

		ret = clk_prepare_enable(csi_mipical->clk_mipical);
		if (ret) {
			dev_err(&pdev->dev, "fail to enable mipical clock\n");
			goto err_clk_enable_mipical;
		}

		ret = clk_prepare_enable(csi_mipical->clk_uart_fs_mipical);
		if (ret) {
			dev_err(&pdev->dev, "fail to enable uart_fs_mipi_cal clock\n");
			goto err_clk_enable_uart_fs_mipi_cal;
		}

		rst = of_reset_control_get(np, "mipi_cal");
		if (IS_ERR_OR_NULL(rst)) {
			dev_err(&pdev->dev,
				"mipi_cal: reset get control failed\n");
			ret = PTR_ERR(rst);
			goto err_mipi_cal_rst;
		}

		ret = reset_control_deassert(rst);
		if (ret) {
			dev_err(&pdev->dev,
				"mipi_cal: fail to deassert reset\n");
			ret = PTR_ERR(rst);
			goto err_mipi_cal_deassert_rst;
		}

		reset_control_put(rst);
	}

#ifdef CONFIG_PM
#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_POWERGATE)
	tegra_nvdisp_unpowergate_partition(TEGRA186_POWER_DOMAIN_DISP);
#endif
#endif

#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_POWERGATE)
	if (!tegra_powergate_is_powered(TEGRA186_POWER_DOMAIN_DISP)) {
		tegra_nvdisp_unpowergate_partition(TEGRA186_POWER_DOMAIN_DISP);
		powergate = true;
	}
#endif

	regmap_update_bits(csi_mipical->regmap,
		MIPI_CAL_MIPI_BIAS_PAD_CFG0_0, 1, 1);
	regmap_update_bits(csi_mipical->regmap,
		MIPI_CAL_MIPI_BIAS_PAD_CFG1_0, 1, 0);
	regmap_update_bits(csi_mipical->regmap,
		MIPI_CAL_MIPI_BIAS_PAD_CFG2_0, 1, 0);

#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_POWERGATE)
	if (powergate)
		tegra_nvdisp_powergate_partition(TEGRA186_POWER_DOMAIN_DISP);
#endif

	return 0;

err_mipi_cal_deassert_rst:
	reset_control_put(rst);
err_mipi_cal_rst:
	clk_disable_unprepare(csi_mipical->clk_uart_fs_mipical);
err_clk_enable_uart_fs_mipi_cal:
	clk_disable_unprepare(csi_mipical->clk_mipical);
err_clk_enable_mipical:
	devm_clk_put(&pdev->dev, csi_mipical->clk_uart_fs_mipical);
err_uart_fs_mipi_cal:
	devm_clk_put(&pdev->dev, csi_mipical->clk_mipical);
err:
	return ret;
}

#ifdef CONFIG_PM
static int tegra186_csi_mipical_suspend(
		struct platform_device *pdev, pm_message_t state)
{
#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_POWERGATE)
	tegra_nvdisp_powergate_partition(TEGRA186_POWER_DOMAIN_DISP);
#endif
	return 0;
}

static int tegra186_csi_mipical_resume(struct platform_device *pdev)
{
#if defined(CONFIG_TEGRA_NVDISPLAY) && defined(CONFIG_TEGRA_POWERGATE)
	struct tegra186_csi_mipical *csi_mipical;

	csi_mipical = dev_get_drvdata(&pdev->dev);
	tegra_nvdisp_unpowergate_partition(TEGRA186_POWER_DOMAIN_DISP);
	regmap_update_bits(csi_mipical->regmap,
		MIPI_CAL_MIPI_BIAS_PAD_CFG0_0, 1, 1);
	regmap_update_bits(csi_mipical->regmap,
		MIPI_CAL_MIPI_BIAS_PAD_CFG1_0, 1, 0);
	regmap_update_bits(csi_mipical->regmap,
		MIPI_CAL_MIPI_BIAS_PAD_CFG2_0, 1, 0);
#endif
	return 0;
}
#endif

static struct platform_driver tegra186_csi_mipi_cal_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra186_csi_mipical_of_match,
	},
	.probe = tegra186_csi_mipical_platform_probe,
#ifdef CONFIG_PM
	.suspend = tegra186_csi_mipical_suspend,
	.resume = tegra186_csi_mipical_resume,
#endif
};
module_platform_driver(tegra186_csi_mipi_cal_driver)

MODULE_AUTHOR("Junghyun Kim <juskim@nvidia.com>");
MODULE_DESCRIPTION("CSI MIPI Calibration Driver");
MODULE_LICENSE("GPLv2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra186_csi_mipical_of_match);
