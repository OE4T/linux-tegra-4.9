/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <asm/traps.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/platform/tegra/bridge_mca.h>

#define BUS_ADDR_MASK 0x3fffffff
#define BUS_ERROR_TYPE_MASK 0x3e0

static LIST_HEAD(bridge_list);
static DEFINE_RAW_SPINLOCK(bridge_lock);

static void bus_print_error(struct bridge_mca_bank *bank) {
	int bus_addr;
	int bus_status;
	int error_type;

	bus_addr = readl(bank->vaddr) & BUS_ADDR_MASK;
	writel(1, bank->vaddr);
	bus_status = readl(bank->vaddr);
	error_type = bus_status & BUS_ERROR_TYPE_MASK;

	pr_crit("**************************************\n");
	pr_crit("Machine check error in %s@0x%llx:\n",
		bank->name, bank->bank);
	pr_crit("Bus addr: 0x%x\n", bus_addr);
	pr_crit("Error status 0x%x: %s\n", bus_status,
		(error_type > bank->max_error ? "Unknown" :
		bank->errors[error_type].desc));
	pr_crit("**************************************\n");
}


static int bridge_serr_hook(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	struct bridge_mca_bank *bank = priv;

	if(readl(bank->vaddr))
		bus_print_error(bank);
	return 0;
}

static struct tegra_bridge_data axi2apb_data = {
	.name = "AXI2APB",
	.offset = 0x2ec,
	.errors = t18x_axi_errors,
	.max_error = ARRAY_SIZE(t18x_axi_errors)
};

static struct tegra_bridge_data axip2p_data = {
	.name = "AXIP2P",
	.offset = 0x11c,
	.errors = t18x_axi_errors,
	.max_error = ARRAY_SIZE(t18x_axi_errors)
};

static struct of_device_id tegra18_bridge_match[] = {
	{.compatible	= "nvidia,tegra186-AXI2APB-bridge",
	 .data		= &axi2apb_data},
	{.compatible	= "nvidia,tegra186-AXIP2P-bridge",
	 .data		= &axip2p_data},
	{},
};

MODULE_DEVICE_TABLE(of, tegra18_bridge_match);

static int tegra18_bridge_probe(struct platform_device *pdev)
{
	struct resource *res_base;
	struct bridge_mca_bank *bank;
	const struct tegra_bridge_data *bdata;
	struct serr_hook *hook;
	const struct of_device_id *match;
	unsigned long flags;

	match = of_match_device(of_match_ptr(tegra18_bridge_match),
				&pdev->dev);

	if (!match) {
		dev_err(&pdev->dev, "No device match found");
		return -ENODEV;
	}

	bdata = match->data;

	res_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_base) {
		dev_err(&pdev->dev, "Could not find base address");
		return -ENOENT;
	}

	bank = devm_kzalloc(&pdev->dev, sizeof(*bank), GFP_KERNEL);
	bank->bank = res_base->start;
	bank->vaddr = devm_ioremap_resource(&pdev->dev, res_base);
	if(IS_ERR(bank->vaddr))
		return -EPERM;
	bank->vaddr += bdata->offset;
	bank->name = bdata->name;
	bank->errors = bdata->errors;
	bank->max_error = bdata->max_error;

	hook = devm_kzalloc(&pdev->dev, sizeof(*hook), GFP_KERNEL);
	hook->fn = bridge_serr_hook;
	hook->priv = bank;
	bank->hook = hook;

	raw_spin_lock_irqsave(&bridge_lock, flags);
	list_add(&bank->node, &bridge_list);
	raw_spin_unlock_irqrestore(&bridge_lock, flags);

	register_serr_hook(hook);

	return 0;
}

static int tegra18_bridge_remove(struct platform_device *pdev)
{
	struct resource *res_base;
	struct bridge_mca_bank *bank;
	unsigned long flags;

	res_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_base)
		return 0;

	raw_spin_lock_irqsave(&bridge_lock, flags);
	list_for_each_entry(bank, &bridge_list, node) {
		if (bank->bank == res_base->start) {
			unregister_serr_hook(bank->hook);
			list_del(&bank->node);
			break;
		}
	}
	raw_spin_unlock_irqrestore(&bridge_lock, flags);

	return 0;
}

static struct platform_driver platform_driver = {
	.probe		= tegra18_bridge_probe,
	.remove		= tegra18_bridge_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra18-bridge",
		.of_match_table = of_match_ptr(tegra18_bridge_match),
	},
};

module_platform_driver(platform_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Bridge Machine Check / SError handler");
