/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clk/tegra.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/tegra-soc.h>

static int tegra_clk_prepare(struct clk_hw *hw)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static void tegra_clk_unprepare(struct clk_hw *hw)
{
	/* Todo: Add IPC to BPMP here */
}

static int tegra_clk_enable(struct clk_hw *hw)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static void tegra_clk_disable(struct clk_hw *hw)
{
	/* Todo: Add IPC to BPMP here */
}

static int tegra_clk_is_enabled(struct clk_hw *hw)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static int tegra_clk_set_parent(struct clk_hw *hw, u8 index)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static u8 tegra_clk_get_parent(struct clk_hw *hw)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static int tegra_clk_set_rate(struct clk_hw *hw, unsigned long target_rate,
				unsigned long parent_rate)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static long tegra_clk_round_rate(struct clk_hw *hw, unsigned long target_rate,
				unsigned long *round_rate)
{
	/* Todo: Add IPC to BPMP and update return value accordingly */
	return 0;
}

static unsigned long tegra_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	return 0;
}

static const struct clk_ops tegra_clk_ops = {
	.prepare = tegra_clk_prepare,
	.unprepare = tegra_clk_unprepare,
	.enable = tegra_clk_enable,
	.disable = tegra_clk_disable,
	.is_enabled = tegra_clk_is_enabled,
	.set_parent = tegra_clk_set_parent,
	.get_parent = tegra_clk_get_parent,
	.set_rate = tegra_clk_set_rate,
	.round_rate = tegra_clk_round_rate,
	.recalc_rate = tegra_recalc_rate,
};

static struct clk * __init clk_register_tegra(struct device *dev,
	 const char *name, const char *parent_name, unsigned long flags)
{
	struct clk_hw *hw;
	struct clk *clk;
	struct clk_init_data init;

	hw = kzalloc(sizeof(struct clk_hw), GFP_KERNEL);
	if (!hw) {
		pr_err("%s: could not allocate clk_hw struct.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &tegra_clk_ops;
	init.flags = flags | CLK_IS_BASIC | CLK_IS_ROOT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	hw->init = &init;

	/* register the clock */
	clk = clk_register(dev, hw);

	if (IS_ERR(clk))
		kfree(hw);

	clk_register_clkdev(clk, name, NULL);

	return clk;
}

static const char *tegra_clk_name[] = {
	"cpu_d",
	"cpu_a",
	"ddr",
	"emc",
	"gpu",
	"pll_p",
	"sdr",
	"disp0",
	"disp1",
	"disp2",
	"sor0",
	"sor1",
	"dpaux",
	"dpaux1",
	"hdmi",
	"spi",
	"pll_dp",
	"sor_safe",
	"pll_d2",
	"emc.la",
	"gpcdma",
	"div-clk",
	"sdmmc4",
	"se",
	"entropy",
	/* Get complete list from BPMP */
};

#define MAXCLK (sizeof(tegra_clk_name) / sizeof(tegra_clk_name[0]))

static struct clk *tegra_clks[MAXCLK];


static struct of_device_id tegra18x_clock_of_match[] = {
	{ .compatible = "nvidia,tegra18x-car", },
	{ },
};

/* Needed for a nvdisp linsim clock hack */
#define CLK_RST_CONTROLLER_RST_DEV_NVDISPLAY0_CLR_0 0x800008
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_NVDISPLAY0_SET_0 0x80100

void enable_gpio_clk(void)
{
	void __iomem *clk_base = ioremap(0x5840000, 0x21000);

	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x1000);
	udelay(2);

	__raw_writel(0x0, clk_base + 0x10000);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x11000);

	udelay(2);
	iounmap(clk_base);

	clk_base = ioremap(0x5860000, 0x21000);
	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 1000);
	udelay(2);

	__raw_writel(0x0, clk_base + 0x10000);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x11000);
	udelay(2);
	iounmap(clk_base);

	clk_base = ioremap(0x5880000, 0x21000);
	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x1000);
	udelay(2);
	iounmap(clk_base);

	clk_base = ioremap(0x5fc0000, 0x21000);

	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 1000);
	udelay(2);
	iounmap(clk_base);
}

/* Needed for GPCDMA controller reset */
#define CLK_RST_CONTROLLER_RST_DEV_AXI_CBB_0 0x56A0000
void reset_gpcdma_controller(void)
{
	void __iomem *gpcdma_rst;

	gpcdma_rst = ioremap(CLK_RST_CONTROLLER_RST_DEV_AXI_CBB_0, 0x4);
	writel(0x6, gpcdma_rst);
	udelay(2);
	writel(0x4, gpcdma_rst);
}

static int tegra18x_clock_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	void __iomem *base = NULL;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra18x_clock_of_match, &pdev->dev);

		if (match) {
			res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		} else {
			ret = -1;
		}
	} else {
		ret = -1;
	}

	/* Nvdisp linsim clock hack */
	if (tegra_platform_is_linsim() && !ret) {
		base = ioremap(res->start,
				res->end - res->start + 1);
		writel(0x3ff, base + CLK_RST_CONTROLLER_RST_DEV_NVDISPLAY0_CLR_0);
		writel(0xf, base + CLK_RST_CONTROLLER_CLK_OUT_ENB_NVDISPLAY0_SET_0);
	}
	enable_gpio_clk();

	/* Reset GPCDMA controller */
	reset_gpcdma_controller();

	return ret;
}

static struct platform_driver platform_driver = {
	.probe = tegra18x_clock_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra18x_clock",
#ifdef CONFIG_OF
		.of_match_table = tegra18x_clock_of_match,
#endif
	},
};

static int __init tegra18x_clock_init(struct device_node *np)
{
	int i;

	for (i = 0; i < MAXCLK; i++) {
		tegra_clks[i] = clk_register_tegra(NULL, tegra_clk_name[i],
				NULL, 0);
		BUG_ON(IS_ERR(tegra_clks[i]));
	}

	return platform_driver_register(&platform_driver);
}

static void __exit tegra18x_clock_exit(void)
{
	platform_driver_unregister(&platform_driver);
}

CLK_OF_DECLARE(tegra18x, "nvidia,tegra18x-car", tegra18x_clock_init);
module_exit(tegra18x_clock_exit);
