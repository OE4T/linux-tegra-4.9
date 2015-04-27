/*
 * GPIO driver for NVIDIA Tegra186
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm.h>
#include <linux/syscore_ops.h>
#include <linux/tegra-soc.h>
#include <linux/irqchip/tegra.h>
#include <linux/platform_data/gpio-tegra.h>

static u32 address_map[32];

#define GPIO_ENB_CONFIG_REG	0x00
#define  GPIO_ENB_BIT		0x0
#define  GPIO_INOUT_BIT		0x1
#define  GPIO_TRG_TYPE_BIT(x)	(0x3 & x)
#define  GPIO_TRG_LVL_BIT	0x4
#define  GPIO_DEB_FUNC_BIT	0x5
#define  GPIO_INT_FUNC_BIT	0x6
#define  GPIO_TIMESTMP_FUNC_BIT 0x7

#define GPIO_DBC_THRES_REG	0x04
#define  GPIO_DBC_THRES_BIT(val) (val & 0xFF)
#define GPIO_INPUT_REG		0x08
#define GPIO_OUT_CTRL_REG	0x0c
#define GPIO_OUT_VAL_REG	0x10
#define GPIO_INT_CLEAR_REG	0x14

#define GPIO_REG_DIFF	0x20

static void __iomem *regs;

static inline u32 compute_reg_addr(u32 gpio)
{
	u32 port;
	u32 addr;
	port = gpio / 8;
	addr = address_map[port];
	return addr;
}

static inline void tegra_gpio_writel(u32 val, u32 reg)
{
	__raw_writel(val, regs + reg);
}

static inline u32 tegra_gpio_readl(u32 reg)
{
	return __raw_readl(regs + reg);
}

static void tegra_gpio_enable(int gpio)
{
	u32 val;
	u32 reg1;
	reg1 = compute_reg_addr(gpio);
	reg1 += (GPIO_REG_DIFF * (gpio % 8));
	val = readl(regs + reg1);
	val |= 0x1;
	writel(val, regs + reg1);
}

static int tegra_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void tegra_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 val;
	u32 reg1;
	reg1 = compute_reg_addr(offset);
	reg1 += (GPIO_REG_DIFF * (offset % 8));
	val = readl(regs + reg1);
	if (val & ((1 << GPIO_INOUT_BIT) | (1 << GPIO_ENB_BIT))) {
		reg1 = compute_reg_addr(offset);
		reg1 += GPIO_OUT_VAL_REG + (GPIO_REG_DIFF * (offset % 8));
		if (value)
			val = 0x1;
		else
			val = 0x0;
		writel(val, regs + reg1);
		reg1 = compute_reg_addr(offset);
		reg1 += GPIO_OUT_CTRL_REG + (GPIO_REG_DIFF * (offset % 8));
		writel(0, regs + reg1);
	}
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{

	u32 reg1;
	u32 val;
	reg1 = compute_reg_addr(offset);
	reg1 += (GPIO_REG_DIFF * (offset % 8));
	val = readl(regs + reg1);
	if (val & ((1 << GPIO_INOUT_BIT))) {
		reg1 = compute_reg_addr(offset);
		reg1 += GPIO_OUT_VAL_REG + (GPIO_REG_DIFF * (offset % 8));
		return readl(regs + reg1) & 0x1;
	}
	reg1 = compute_reg_addr(offset);
	reg1 += GPIO_INPUT_REG + (GPIO_REG_DIFF * (offset % 8));

	return readl(regs + reg1);
}

static void set_gpio_direction_mode(unsigned offset, bool mode)
{
	u32 reg1;
	u32 val;
	reg1 = compute_reg_addr(offset);
	reg1 += GPIO_ENB_CONFIG_REG + (GPIO_REG_DIFF * (offset % 8));

	val = readl(regs + reg1);
	if (mode)
		val |= 1 << GPIO_INOUT_BIT;
	else
		val &= ~(1 << GPIO_INOUT_BIT);

	writel(val, regs + reg1);
}

static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	int ret;

	tegra_gpio_enable(offset);
	set_gpio_direction_mode(offset, 0);

	ret = pinctrl_gpio_direction_input(chip->base + offset);
	if (ret < 0)
		dev_err(chip->dev,
			"Tegra gpio input: pinctrl input failed: %d\n", ret);

	return 0;
}

static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	int ret;

	tegra_gpio_enable(offset);

	set_gpio_direction_mode(offset, 1);

	ret = pinctrl_gpio_direction_output(chip->base + offset);
	if (ret < 0)
		dev_err(chip->dev,
			"Tegra gpio output: pinctrl output failed: %d\n", ret);

	tegra_gpio_set(chip, offset, value);
	return 0;
}

static int tegra_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				unsigned debounce)
{
	u32 reg1;
	u32 val;
	unsigned max_dbc;
	unsigned debounce_ms = DIV_ROUND_UP(debounce, 1000);

	reg1 = compute_reg_addr(offset);
	reg1 += (GPIO_REG_DIFF * (offset % 8));
	val = readl(regs + reg1);
	if (val & (1 << GPIO_ENB_BIT)) {
		val |= (1 << GPIO_DEB_FUNC_BIT);
		writel(val, regs + reg1);

		debounce_ms = max(debounce_ms, 255U);
		reg1 = compute_reg_addr(offset);
		reg1 += GPIO_DBC_THRES_REG + (GPIO_REG_DIFF * (offset % 8));

		max_dbc = readl(regs + reg1);
		max_dbc = (max_dbc < debounce_ms) ? debounce_ms : max_dbc;

		writel(max_dbc, regs + reg1);
		return 0;
	}
	return -ENOSYS;
}

static struct gpio_chip tegra_gpio_chip = {
	.label			= "tegra-gpio",
	.request		= tegra_gpio_request,
	.free			= tegra_gpio_free,
	.direction_input	= tegra_gpio_direction_input,
	.get			= tegra_gpio_get,
	.direction_output	= tegra_gpio_direction_output,
	.set			= tegra_gpio_set,
	.set_debounce		= tegra_gpio_set_debounce,
	.base			= 0,
};

struct tegra_gpio_soc_config {
	bool debounce_support;
};

static struct tegra_gpio_soc_config tegra186_gpio_config = {
	.debounce_support = true,
};

static struct of_device_id tegra_gpio_of_match[] = {
	{ .compatible = "nvidia,tegra186-gpio", .data = &tegra186_gpio_config },
	{ },
};

static void read_gpio_mapping_data(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int a;
	int nstates;
	int i;

	if (!np)
		return;
	nstates = of_property_count_u32(np, "nvidia,gpio_mapping");

	for (i = 0; i < nstates; i++) {
		of_property_read_u32_index(np, "nvidia,gpio_mapping", i, &a);
		address_map[i] = a;
	}

}

static int tegra_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct tegra_gpio_soc_config *config;
	struct resource *res;

	read_gpio_mapping_data(pdev);
	match = of_match_device(tegra_gpio_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}
	config = (struct tegra_gpio_soc_config *)match->data;

	tegra_gpio_chip.dev = &pdev->dev;

	tegra_gpio_chip.ngpio = 32 * 8; /* 32 total number of gpio ports */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing MEM resource\n");
		return -ENODEV;
	}
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	tegra_gpio_chip.of_node = pdev->dev.of_node;
	gpiochip_add(&tegra_gpio_chip);

	return 0;
}

static struct platform_driver tegra_gpio_driver = {
	.driver		= {
		.name	= "tegra-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = tegra_gpio_of_match,
	},
	.probe		= tegra_gpio_probe,
};

static int __init tegra_gpio_init(void)
{
	return platform_driver_register(&tegra_gpio_driver);
}
postcore_initcall(tegra_gpio_init);

MODULE_AUTHOR("Suresh Mangipudi <smangipudi@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra186 GPIO driver");
MODULE_LICENSE("GPL v2");
