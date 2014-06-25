/*
 * tegra124_virt_apbif_master.c - Tegra virtual APBIF master driver
 *
 * Copyright (c) 2011-2014 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <mach/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra30_xbar_alt.h"
#include "tegra124_virt_apbif_master.h"

#define DRV_NAME "tegra30-ahub-apbif"

int tegra30_apbif_i2s_rx_fifo_is_enabled(int i2s_id)
{
	int val = 0;
	return val;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_rx_fifo_is_enabled);

int tegra30_apbif_i2s_tx_fifo_is_enabled(int i2s_id)
{
	int val = 0;
	return val;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_tx_fifo_is_enabled);

int tegra30_apbif_i2s_rx_fifo_is_empty(int i2s_id)
{
	int val = 0;
	return val;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_rx_fifo_is_empty);

int tegra30_apbif_i2s_tx_fifo_is_empty(int i2s_id)
{
	int val = 0;
	return val;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_tx_fifo_is_empty);
int tegra30_apbif_i2s_underrun_interrupt_status_clear(int i2s_id)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_underrun_interrupt_status_clear);

int tegra30_apbif_i2s_overrun_interrupt_status_clear(int i2s_id)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_overrun_interrupt_status_clear);

int tegra30_apbif_i2s_underrun_interrupt_status(int i2s_id)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_underrun_interrupt_status);

int tegra30_apbif_i2s_overrun_interrupt_status(int i2s_id)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tegra30_apbif_i2s_overrun_interrupt_status);

static int tegra124_virt_apbif_runtime_suspend(struct device *dev)
{
	struct tegra124_virt_apbif *apbif = dev_get_drvdata(dev);

	clk_disable(apbif->clk);

	return 0;
}

static int tegra124_virt_apbif_runtime_resume(struct device *dev)
{
	struct tegra124_virt_apbif *apbif = dev_get_drvdata(dev);
	int ret;

	ret = clk_enable(apbif->clk);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	return 0;
}

#define CLK_LIST_MASK_TEGRA30 BIT(0)
#define CLK_LIST_MASK_TEGRA114 BIT(1)
#define CLK_LIST_MASK_TEGRA124 BIT(2)

#define CLK_LIST_MASK_TEGRA30_OR_LATER \
		(CLK_LIST_MASK_TEGRA30 | CLK_LIST_MASK_TEGRA114 |\
		CLK_LIST_MASK_TEGRA124)
#define CLK_LIST_MASK_TEGRA114_OR_LATER \
		(CLK_LIST_MASK_TEGRA114 | CLK_LIST_MASK_TEGRA124)

static const struct {
	const char *clk_name;
	unsigned int clk_list_mask;
} configlink_clocks[] = {
	{ "i2s0", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "i2s1", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "i2s2", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "i2s3", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "i2s4", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "dam0", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "dam1", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "dam2", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "spdif_in", CLK_LIST_MASK_TEGRA30_OR_LATER },
	{ "amx", CLK_LIST_MASK_TEGRA114_OR_LATER },
	{ "adx", CLK_LIST_MASK_TEGRA114_OR_LATER },
	{ "amx1", CLK_LIST_MASK_TEGRA124 },
	{ "adx1", CLK_LIST_MASK_TEGRA124 },
	{ "afc0", CLK_LIST_MASK_TEGRA124 },
	{ "afc1", CLK_LIST_MASK_TEGRA124 },
	{ "afc2", CLK_LIST_MASK_TEGRA124 },
	{ "afc3", CLK_LIST_MASK_TEGRA124 },
	{ "afc4", CLK_LIST_MASK_TEGRA124 },
	{ "afc5", CLK_LIST_MASK_TEGRA124 },
};

static struct of_dev_auxdata tegra124_virt_apbif_auxdata[] = {
	OF_DEV_AUXDATA("nvidia,tegra124-i2s", 0x70301000,
		"tegra30-i2s.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-i2s", 0x70301100,
		"tegra30-i2s.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-i2s", 0x70301200,
		"tegra30-i2s.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-i2s", 0x70301300,
		"tegra30-i2s.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-i2s", 0x70301400,
		"tegra30-i2s.4", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dam", 0x70302000,
		"tegra30-dam.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dam", 0x70302200,
		"tegra30-dam.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dam", 0x70302400,
		"tegra30-dam.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-spdif", 0x70306000,
		"tegra30-spdif", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-amx", 0x70303000,
		"tegra124-amx.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-virt-amx", 0x70303100,
		"tegra124-amx.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-adx", 0x70303800,
		"tegra124-adx.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-adx", 0x70303900,
		"tegra124-adx.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-afc", 0x70307000,
		"tegra124-afc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-afc", 0x70307100,
		"tegra124-afc.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-afc", 0x70307200,
		"tegra124-afc.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-afc", 0x70307300,
		"tegra124-afc.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-afc", 0x70307400,
		"tegra124-afc.4", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-afc", 0x70307500,
		"tegra124-afc.5", NULL),
	{},
};

static struct platform_device_info tegra30_xbar_device_info = {
	.name = "tegra30-ahub-xbar",
	.id = -1,
};

static const struct of_device_id tegra124_virt_apbif_virt_of_match[] = {
	{ .compatible = "nvidia,tegra124-virt-ahub-master" },
	{},
};

static int tegra124_virt_apbif_probe(struct platform_device *pdev)
{
	int i;
	struct clk *clk;
	int ret;
	struct tegra124_virt_apbif *apbif;

	/*
	 * The TEGRA_AHUB APBIF hosts a register bus: the "configlink".
	 * For this to operate correctly, all devices on this bus must
	 * be out of reset.
	 * Ensure that here.
	 */
	for (i = 0; i < ARRAY_SIZE(configlink_clocks); i++) {
		if (!(configlink_clocks[i].clk_list_mask &
					CLK_LIST_MASK_TEGRA124))
			continue;
		clk = devm_clk_get(&pdev->dev, configlink_clocks[i].clk_name);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Can't get clock %s\n",
				configlink_clocks[i].clk_name);
			ret = PTR_ERR(clk);
			goto err;
		}
		tegra_periph_reset_deassert(clk);
		devm_clk_put(&pdev->dev, clk);
	}

	apbif = devm_kzalloc(&pdev->dev, sizeof(*apbif), GFP_KERNEL);
	if (!apbif) {
		dev_err(&pdev->dev, "Can't allocate tegra124_virt_apbif\n");
		ret = -ENOMEM;
		goto err;
	}

	dev_set_drvdata(&pdev->dev, apbif);

	apbif->clk = devm_clk_get(&pdev->dev, "apbif");
	if (IS_ERR(apbif->clk)) {
		dev_err(&pdev->dev, "Can't retrieve clock\n");
		ret = PTR_ERR(apbif->clk);
		goto err;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra124_virt_apbif_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	tegra30_xbar_device_info.res = platform_get_resource(pdev,
						IORESOURCE_MEM, 1);
	if (!tegra30_xbar_device_info.res) {
		dev_err(&pdev->dev, "No memory resource for xbar\n");
		goto err_suspend;
	}
	tegra30_xbar_device_info.num_res = 1;
	tegra30_xbar_device_info.parent = &pdev->dev;
	platform_device_register_full(&tegra30_xbar_device_info);

	of_platform_populate(pdev->dev.of_node, NULL,
		tegra124_virt_apbif_auxdata, &pdev->dev);

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra124_virt_apbif_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err:
	return ret;
}

static int tegra124_virt_apbif_remove(struct platform_device *pdev)
{
	struct tegra124_virt_apbif *apbif = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra124_virt_apbif_runtime_suspend(&pdev->dev);

	devm_clk_put(&pdev->dev, apbif->clk);

	return 0;
}

static const struct dev_pm_ops tegra124_virt_apbif_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra124_virt_apbif_runtime_suspend,
			   tegra124_virt_apbif_runtime_resume, NULL)
};

static struct platform_driver tegra124_virt_apbif_driver = {
	.probe = tegra124_virt_apbif_probe,
	.remove = tegra124_virt_apbif_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra124_virt_apbif_virt_of_match,
		.pm = &tegra124_virt_apbif_pm_ops,
	},
};
module_platform_driver(tegra124_virt_apbif_driver);

MODULE_AUTHOR("Aniket Bahadarpurkar <aniketb@nvidia.com>");
MODULE_DESCRIPTION("Tegra124 virt APBIF master driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
