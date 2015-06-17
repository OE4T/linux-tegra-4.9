/*
 * GPIO driver for NVIDIA Tegra186
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Suresh Mangipudi <smangipudi@nvidia.com>
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
#include <linux/platform_data/gpio-tegra.h>
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

#define GPIO_SCR_REG	0x04
#define GPIO_SCR_DIFF	0x08

#define GPIO_CONTROLLERS_DIFF 0x1000

#define DEFAULT_IRQ_OFFSET 32

#define GPIO_INT_LVL_NO_TRIGGER	0x0
#define GPIO_INT_LVL_LEVEL_TRIGGER	0x1
#define GPIO_INT_LVL_SINGLE_EDGE_TRIGGER	0x2
#define GPIO_INT_LVL_BOTH_EDGE_TRIGGER	0x3

#define TRIGGER_LEVEL_LOW		0x0
#define TRIGGER_LEVEL_HIGH		0x1

#define GPIO_INT_STATUS_OFFSET		0x100
#define GPIO_STATUS_G1			0x04

#define MAX_GPIO_CONTROLLERS 7
#define MAX_GPIO_PORTS 8

struct tegra_gpio_controller {
	int controller;
	int irq;
	spinlock_t lvl_lock[4];
};

struct tegra_gpio {
	struct device *dev;
	const struct tegra_pinctrl_soc_data *soc;

	int nbanks;
	void __iomem **regs;
	int *regs_size;
	unsigned int *reg_base;
};

static struct tegra_gpio *tegra_gpio;

/* Below table is mapping for contoller and ports contained
 * in each port of the give gpio controller
 */
static int tegra186_gpio_map
	[MAX_GPIO_CONTROLLERS][MAX_GPIO_PORTS] = {
	{13, 14, 16, 19, 8, 17, -1, -1}, /* gpio cntrlr 0 */
	{7, 11, 23, 24, -1, -1, -1, -1}, /* gpio cntrlr 1 */
	{0, 4, 5, 27, -1, -1, -1, -1,}, /* gpio cntrlr 2 */
	{1, 2, 3, -1, -1, -1, -1, -1,}, /* gpio cntrlr 3 */
	{15, 6, -1, -1, -1, -1, -1, -1},/* gpio cntrlr 4 */
	{9, 10, 28, 12, -1, -1, -1, -1},/* gpio cntrlr 5 */
	{31, 18, 20, 30, 21, 22, 26, 25},/* AON gpio cntrlr */
};

static u32 address_map[32][2];
static u32 tegra_gpio_bank_count;
static struct tegra_gpio_controller *tegra_gpio_controllers;

static struct irq_domain *irq_domain;

static inline u32 controller_index(u32 gpio)
{
	int i, j;
	u32 temp;
	temp = gpio/8;
	for (i = 0; i < MAX_GPIO_CONTROLLERS; i++) {
		for (j = 0; j < MAX_GPIO_PORTS; j++) {
			if (tegra186_gpio_map[i][j] == temp)
				return i;
		}
	}
	return -1;
}

static inline u32 tegra_gpio_readl(u32 gpio, u32 reg_offset)
{
	u32 temp_addr;
	temp_addr = address_map[gpio / 8][1] + (GPIO_REG_DIFF * (gpio % 8))
				+ reg_offset;
	return __raw_readl((tegra_gpio->regs[address_map[gpio / 8][0]])
		+ temp_addr);
}

static inline void tegra_gpio_writel(u32 val, u32 gpio, u32 reg_offset)
{
	u32 temp_addr;
	temp_addr = address_map[gpio / 8][1] + (GPIO_REG_DIFF * (gpio % 8))
				+ reg_offset;
	__raw_writel(val, (tegra_gpio->regs[address_map[gpio / 8][0]])
		+ temp_addr);
}


static void tegra_gpio_enable(int gpio)
{
	u32 val;
	val = tegra_gpio_readl(gpio, GPIO_ENB_CONFIG_REG);
	val |= 0x1;
	tegra_gpio_writel(val, gpio, GPIO_ENB_CONFIG_REG);
}

static void tegra_gpio_disable(int gpio)
{
	tegra_gpio_writel(0, gpio, GPIO_ENB_CONFIG_REG);
}

static int tegra_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void tegra_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
	tegra_gpio_disable(offset);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	u32 val;

	val = tegra_gpio_readl(offset, GPIO_ENB_CONFIG_REG);
	if (val & ((1 << GPIO_INOUT_BIT) | (1 << GPIO_ENB_BIT))) {
		if (value)
				val = 0x1;
			else
				val = 0x0;
		tegra_gpio_writel(val, offset, GPIO_OUT_VAL_REG);
		tegra_gpio_writel(0, offset, GPIO_OUT_CTRL_REG);
	}
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 val;
	val = tegra_gpio_readl(offset, GPIO_ENB_CONFIG_REG);
	if (val & ((1 << GPIO_INOUT_BIT)))
		return tegra_gpio_readl(offset, GPIO_OUT_VAL_REG) & 0x1;

	return tegra_gpio_readl(offset, GPIO_INPUT_REG) & 0x1;
}

static void set_gpio_direction_mode(unsigned offset, bool mode)
{
	u32 val;
	val = tegra_gpio_readl(offset, GPIO_ENB_CONFIG_REG);
	if (mode)
		val |= 1 << GPIO_INOUT_BIT;
	else
		val &= ~(1 << GPIO_INOUT_BIT);
	tegra_gpio_writel(val, offset, GPIO_ENB_CONFIG_REG);

	val = tegra_gpio_readl(offset, GPIO_INPUT_REG);

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
	u32 val;
	unsigned max_dbc;
	unsigned debounce_ms = DIV_ROUND_UP(debounce, 1000);

	val = tegra_gpio_readl(offset, GPIO_ENB_CONFIG_REG);
	if (val & (1 << GPIO_ENB_BIT)) {
		val |= (1 << GPIO_DEB_FUNC_BIT);
		tegra_gpio_writel(val, offset, GPIO_ENB_CONFIG_REG);

		debounce_ms = max(debounce_ms, 255U);
		max_dbc = tegra_gpio_readl(offset, GPIO_DBC_THRES_REG);
		max_dbc = (max_dbc < debounce_ms) ? debounce_ms : max_dbc;

		tegra_gpio_writel(max_dbc, offset, GPIO_DBC_THRES_REG);
		return 0;
	}
	return -ENOSYS;
}

static int tegra_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{

	return irq_find_mapping(irq_domain, offset);
}

static void tegra_gpio_irq_ack(struct irq_data *d)
{
	int gpio = d->hwirq;
	tegra_gpio_writel(1, gpio, GPIO_INT_CLEAR_REG);
	/* FPGA platforms have a serializer between the GPIO
	   block and interrupt controller. Allow time for
	   clearing of the GPIO interrupt to propagate to the
	   interrupt controller before re-enabling the IRQ
	   to prevent double interrupts. */
	if (tegra_platform_is_fpga())
		udelay(15);
}

static void tegra_gpio_irq_mask(struct irq_data *d)
{
	int gpio = d->hwirq;
	u32 val;

	val = tegra_gpio_readl(gpio, GPIO_ENB_CONFIG_REG);
	val &= ~(1 << GPIO_INT_FUNC_BIT);
	tegra_gpio_writel(val, gpio, GPIO_ENB_CONFIG_REG);
}

static void tegra_gpio_irq_unmask(struct irq_data *d)
{
	int gpio = d->hwirq;
	u32 val;

	val = tegra_gpio_readl(gpio, GPIO_ENB_CONFIG_REG);
	val |= 1 << GPIO_INT_FUNC_BIT;
	tegra_gpio_writel(val, gpio, GPIO_ENB_CONFIG_REG);
}

static int tegra_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	int gpio = d->hwirq;
	u32 lvl_type = 0;
	u32 trg_type = 0;
	u32 val;
	u32 wake = tegra_gpio_to_wake(d->hwirq);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		trg_type = TRIGGER_LEVEL_HIGH;
		lvl_type = GPIO_INT_LVL_SINGLE_EDGE_TRIGGER;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		trg_type = TRIGGER_LEVEL_LOW;
		lvl_type = GPIO_INT_LVL_SINGLE_EDGE_TRIGGER;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_INT_LVL_BOTH_EDGE_TRIGGER;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		trg_type = TRIGGER_LEVEL_HIGH;
		lvl_type = GPIO_INT_LVL_LEVEL_TRIGGER;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		trg_type = TRIGGER_LEVEL_LOW;
		lvl_type = GPIO_INT_LVL_LEVEL_TRIGGER;
		break;

	default:
		return -EINVAL;
	}

	trg_type = trg_type << 0x4;
	lvl_type = lvl_type << 0x2;

	/* Program the values */
	val = tegra_gpio_readl(gpio, GPIO_ENB_CONFIG_REG);
	val |= trg_type | lvl_type;
	tegra_gpio_writel(val, gpio, GPIO_ENB_CONFIG_REG);

	tegra_gpio_enable(gpio);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__irq_set_handler_locked(d->irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__irq_set_handler_locked(d->irq, handle_edge_irq);

	tegra_pm_irq_set_wake_type(wake, type);

	return 0;
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
	.to_irq			= tegra_gpio_to_irq,
	.base			= 0,
};

#define tegra_gpio_irq_set_wake NULL

static struct irq_chip tegra_gpio_irq_chip = {
	.name		= "GPIO",
	.irq_ack	= tegra_gpio_irq_ack,
	.irq_mask	= tegra_gpio_irq_mask,
	.irq_unmask	= tegra_gpio_irq_unmask,
	.irq_set_type	= tegra_gpio_irq_set_type,
	.irq_set_wake	= tegra_gpio_irq_set_wake,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};

static void tegra_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	int pin;
	u32 i;
	unsigned long val;
	u32 gpio;
	u32 temp;
	u32 reg;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	switch (irq) {
	case DEFAULT_IRQ_OFFSET + 47:
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[0][i] != -1) {
				temp = address_map[tegra186_gpio_map[0][i]][1];
				reg = tegra186_gpio_map[0][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[0][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	case DEFAULT_IRQ_OFFSET + 50:
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[1][i] != -1) {
				temp = address_map[tegra186_gpio_map[1][i]][1];
				reg = tegra186_gpio_map[1][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[1][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	case DEFAULT_IRQ_OFFSET + 53:
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[2][i] != -1) {
				temp = address_map[tegra186_gpio_map[2][i]][1];
				reg = tegra186_gpio_map[2][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[2][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	case DEFAULT_IRQ_OFFSET + 56:
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[3][i] != -1) {
				temp = address_map[tegra186_gpio_map[3][i]][1];
				reg = tegra186_gpio_map[3][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[3][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	case DEFAULT_IRQ_OFFSET + 59:
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[4][i] != -1) {
				temp = address_map[tegra186_gpio_map[4][i]][1];
				reg = tegra186_gpio_map[4][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[4][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	case DEFAULT_IRQ_OFFSET + 61:
		/* AON */
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[6][i] != -1) {
				temp = address_map[tegra186_gpio_map[6][i]][1];
				reg = tegra186_gpio_map[6][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[6][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	case DEFAULT_IRQ_OFFSET + 180:
		for (i = 0; i < MAX_GPIO_PORTS; i++) {
			if (tegra186_gpio_map[5][i] != -1) {
				temp = address_map[tegra186_gpio_map[5][i]][1];
				reg = tegra186_gpio_map[5][i];
				val = __raw_readl(
					tegra_gpio->regs[address_map[reg][0]] +
					temp + GPIO_INT_STATUS_OFFSET +
					GPIO_STATUS_G1);
				gpio = tegra186_gpio_map[5][i] * 8;
				for_each_set_bit(pin, &val, 8)
					generic_handle_irq(
						gpio_to_irq(gpio + pin));
			}
		}
		break;
	default:
		break;
	}
	chained_irq_exit(chip, desc);
}

static struct of_device_id tegra_gpio_of_match[] = {
	{ .compatible = "nvidia,tegra186-gpio", NULL },
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
	nstates = nstates / 2;
	for (i = 0; i < nstates; i++) {
		of_property_read_u32_index(np, "nvidia,gpio_mapping",
					i * 2, &a);
		address_map[i][0] = a;
		of_property_read_u32_index(np, "nvidia,gpio_mapping",
					i * 2 + 1, &a);
		address_map[i][1] = a;
	}

}

static int tegra_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct resource *res;
	struct tegra_gpio_controller *controller;
	u32 i, j;
	int gpio;
	u32 ret;

	match = of_match_device(tegra_gpio_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	read_gpio_mapping_data(pdev);

	for (;;) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ,
					tegra_gpio_bank_count);
		if (!res)
			break;
		tegra_gpio_bank_count++;
	}

	if (!tegra_gpio_bank_count) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		return -ENODEV;
	}

	tegra_gpio_chip.dev = &pdev->dev;

	tegra_gpio_chip.ngpio = 32 * 8; /* 32 total number of gpio ports */

	tegra_gpio_controllers = devm_kzalloc(&pdev->dev,
			tegra_gpio_bank_count * sizeof(*tegra_gpio_controllers),
			GFP_KERNEL);
	if (!tegra_gpio_controllers) {
		dev_err(&pdev->dev, "Couldn't allocate bank structure\n");
		return -ENODEV;
	}

	irq_domain = irq_domain_add_linear(pdev->dev.of_node,
					   tegra_gpio_chip.ngpio,
					   &irq_domain_simple_ops, NULL);
	if (!irq_domain)
		return -ENODEV;

	tegra_gpio = devm_kzalloc(&pdev->dev, sizeof(*tegra_gpio), GFP_KERNEL);
	if (!tegra_gpio) {
		dev_err(&pdev->dev, "Can't alloc tegra_gpio\n");
		return -ENOMEM;
	}
	tegra_gpio->dev = &pdev->dev;

	for (i = 0;; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;
	}
	tegra_gpio->nbanks = i;

	tegra_gpio->regs = devm_kzalloc(&pdev->dev,
				tegra_gpio->nbanks * sizeof(*tegra_gpio->regs),
				 GFP_KERNEL);
	if (!tegra_gpio->regs) {
		dev_err(&pdev->dev, "Can't alloc regs pointer\n");
		return -ENODEV;
	}

	tegra_gpio->reg_base = devm_kzalloc(
			&pdev->dev,
			tegra_gpio->nbanks * sizeof(*tegra_gpio->reg_base),
			GFP_KERNEL);
	if (!tegra_gpio->reg_base) {
		dev_err(&pdev->dev, "Can't alloc reg_base pointer\n");
		return -ENOMEM;
	}

	for (i = 0; i < tegra_gpio_bank_count; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, i);
		if (!res) {
			dev_err(&pdev->dev, "Missing IRQ resource\n");
			return -ENODEV;
		}
		controller = &tegra_gpio_controllers[i];
		controller->controller = i;
		controller->irq = res->start;
	}

	for (i = 0; i < tegra_gpio->nbanks; i++) {
		void __iomem *base;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(&pdev->dev, "Missing MEM resource\n");
			return -ENODEV;
		}

		base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(base)) {
			ret = PTR_ERR(base);
			dev_err(&pdev->dev,
				"memregion/iomap address request failed: %d\n",
				ret);
			return ret;
		}
		tegra_gpio->reg_base[i] = res->start;
		tegra_gpio->regs[i] = base;
	}

	if (!res) {
		dev_err(&pdev->dev, "Missing MEM resource\n");
		return -ENODEV;
	}

	tegra_gpio_chip.of_node = pdev->dev.of_node;
	gpiochip_add(&tegra_gpio_chip);

	for (gpio = 0; gpio < tegra_gpio_chip.ngpio; gpio++) {
		int irq = irq_create_mapping(irq_domain, gpio);
		/* No validity check; all Tegra GPIOs are valid IRQs */
		/* Map pin to the GPIO controller*/

		controller = &tegra_gpio_controllers[controller_index(gpio)];

		irq_set_chip_data(irq, controller);
		irq_set_chip_and_handler(irq, &tegra_gpio_irq_chip,
					 handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}
	if (0) {
		/* program SCR reg to get the interrupts */
		for (i = 0; i < (tegra_gpio_bank_count - 1); i++) {
			u32 val;
			int k;
			val = 0x1000 * i;
			/* 64 since 8 ports * 8 pins */
			for (k = 0; k < 64; k++) {
				val = 0;
				val = readl(tegra_gpio->regs[0] +
				(k * GPIO_SCR_DIFF) + GPIO_SCR_REG);
				/* program bit 1 and bit 27 */
				val |= (1 << 1) | (1 << 27);
				writel(val, tegra_gpio->regs[0] +
				(k * GPIO_SCR_DIFF) + GPIO_SCR_REG);
			}
		}
	}
	for (i = 0; i < tegra_gpio_bank_count; i++) {
		controller = &tegra_gpio_controllers[i];

		for (j = 0; j < 4; j++)
			spin_lock_init(&controller->lvl_lock[j]);

		irq_set_handler_data(controller->irq, controller);
		irq_set_chained_handler(controller->irq,
			tegra_gpio_irq_handler);
	}

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
