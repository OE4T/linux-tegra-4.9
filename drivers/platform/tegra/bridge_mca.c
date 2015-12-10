/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/ioport.h>

#define BUS_ADDR_MASK 0x3fffffff
#define BUS_ERROR_TYPE_MASK 0x3e0
#define BUS_ERROR_TYPE_SHIFT 5

static LIST_HEAD(bridge_list);
static DEFINE_RAW_SPINLOCK(bridge_lock);

static void bus_print_error(struct bridge_mca_bank *bank) {
	int bus_addr;
	int bus_status;
	int error_type;
	int count = 0;
	struct resource *res = NULL;
	u64 addr;

	if (bank->error_fifo_count(bank->vaddr) == 0)
		return;

	pr_crit("**************************************\n");
	pr_crit("CPU%d Machine check error in %s@0x%llx:\n",
		smp_processor_id(), bank->name, bank->bank);

	while (bank->error_fifo_count(bank->vaddr)) {
		bus_status = bank->error_status(bank->vaddr);
		bus_addr = bank->error_status(bank->vaddr);
		if (((bus_status >> 16) & 0xffff) == 0xdead &&
		    ((bus_addr >> 16) & 0xffff) == 0xdead)
			break;
		bus_addr &= BUS_ADDR_MASK;
		error_type = (bus_status & BUS_ERROR_TYPE_MASK) >>
			     BUS_ERROR_TYPE_SHIFT;

		addr = bus_addr;
		res = locate_resource(&iomem_resource, addr);
		if (res == NULL)
			pr_crit("Bus addr[%d]: 0x%x (Unknown)\n",
				count, bus_addr);
		else
			pr_crit("Bus addr[%d]: 0x%x -- %s + 0x%llx\n",
				count, bus_addr, res->name,
				bus_addr - res->start);

		pr_crit("Error status[%d] 0x%x: %s\n", count, bus_status,
			(error_type >= bank->max_error ? "Unknown" :
			 bank->errors[error_type].desc));
		count += 1;
	}

	pr_crit("**************************************\n");
}


static int bridge_serr_hook(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	struct bridge_mca_bank *bank = priv;

	if (!bank->seen_error &&
	    bank->error_fifo_count(bank->vaddr)) {
		bus_print_error(bank);
		bank->seen_error = 1;
	}
	return 0;
}

#define AXI2APB_ERROR_STATUS	0x2ec
#define AXI2APB_FIFO_STATUS3	0x2f8
#define AXI2APB_FIFO_ERROR_SHIFT	13
#define AXI2APB_FIFO_ERROR_MASK		0x1f

static unsigned int axi2apb_error_status(void __iomem *addr)
{
	unsigned int error_status;

	error_status = readl(addr+AXI2APB_ERROR_STATUS);

	writel(0xFFFFFFFF, addr+AXI2APB_ERROR_STATUS);
	return error_status;
}

static unsigned int axi2apb_error_fifo_count(void __iomem *addr)
{
	unsigned int fifo_status;

	fifo_status = readl(addr+AXI2APB_FIFO_STATUS3);

	fifo_status >>= AXI2APB_FIFO_ERROR_SHIFT;
	fifo_status &= AXI2APB_FIFO_ERROR_MASK;
	return fifo_status;
}

static struct tegra_bridge_data axi2apb_data = {
	.name = "AXI2APB",
	.error_status = axi2apb_error_status,
	.error_fifo_count = axi2apb_error_fifo_count,
	.errors = t18x_axi_errors,
	.max_error = ARRAY_SIZE(t18x_axi_errors)
};

#define AXIP2P_ERROR_STATUS	0x11c
#define AXIP2P_FIFO_STATUS	0x120
#define AXIP2P_FIFO_ERROR_SHIFT 26
#define AXIP2P_FIFO_ERROR_MASK  0x3f

static unsigned int axip2p_error_status(void __iomem *addr)
{
	unsigned int error_status;

	error_status = readl(addr+AXIP2P_ERROR_STATUS);

	writel(0xFFFFFFFF, addr+AXIP2P_ERROR_STATUS);
	return error_status;
}

static unsigned int axip2p_error_fifo_count(void __iomem *addr)
{
	unsigned int fifo_status;

	fifo_status = readl(addr+AXIP2P_FIFO_STATUS);

	fifo_status >>= AXIP2P_FIFO_ERROR_SHIFT;
	fifo_status &= AXIP2P_FIFO_ERROR_MASK;
	return fifo_status;
}

static struct tegra_bridge_data axip2p_data = {
	.name = "AXIP2P",
	.error_status = axip2p_error_status,
	.error_fifo_count = axip2p_error_fifo_count,
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

	bank->name = bdata->name;
	bank->error_status = bdata->error_status;
	bank->error_fifo_count = bdata->error_fifo_count;
	bank->errors = bdata->errors;
	bank->max_error = bdata->max_error;
	bank->seen_error = 0;

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
