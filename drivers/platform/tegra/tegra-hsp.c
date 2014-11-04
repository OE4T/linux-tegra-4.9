/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/tegra-hsp.h>

#define HSP_DB_REG_TRIGGER		0x0
#define HSP_DB_REG_ENABLE		0x4
#define HSP_DB_REG_RAW			0x8
#define HSP_DB_REG_PENDING		0xc

struct hsp_top {
	int nr_sm;
	int nr_as;
	int nr_ss;
	int nr_db;
	int nr_si;
	int status;
	void __iomem *base;
	spinlock_t lock;
};

static struct hsp_top hsp_top;
static void __iomem *db_bases[HSP_NR_DBS];
static irq_handler_t db_irq_cb;

#define HSP_DB_OFF(i, d) \
	(d.base + ((1 + (d.nr_sm>>1) + d.nr_ss + d.nr_as)<<16) + (i) * 0x100)

static inline u32 hsp_readl(void __iomem *base, int reg)
{
	return readl(base + reg);
}

static inline void hsp_writel(void __iomem *base, int reg, u32 val)
{
	writel(val, base + reg);
	(void)readl(base + reg);
}

static irqreturn_t dbell_irq(int irq, void *data)
{
	u32 reg;
	reg = hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING);
	hsp_writel(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING, reg);
	return likely(db_irq_cb) ?
		db_irq_cb(irq, (void *)((uintptr_t)reg)) : IRQ_HANDLED;
}

/**
 * tegra_hsp_db_enable_master: turn on the interrupt from <master>
 * @master:	 HSP master
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_enable_master(enum tegra_hsp_db_master master)
{
	u32 reg;
	unsigned long flags;
	BUG_ON(hsp_top.status != HSP_INIT_OKAY);
	if (master >= HSP_DB_NR_MASTERS)
		return -EINVAL;
	spin_lock_irqsave(&hsp_top.lock, flags);
	reg = hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE);
	reg |= (1 << master);
	hsp_writel(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE, reg);
	spin_unlock_irqrestore(&hsp_top.lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_enable_master);

/**
 * tegra_hsp_db_ring: ring the <dbell>
 * @dbell:	 HSP dbell to be rung
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_ring(enum tegra_hsp_doorbell dbell)
{
	BUG_ON(hsp_top.status != HSP_INIT_OKAY);
	if (dbell >= HSP_NR_DBS)
		return -EINVAL;
	hsp_writel(db_bases[dbell], HSP_DB_REG_TRIGGER, 1);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_ring);

/**
 * tegra_hsp_db_enabled: check if CPU can ring the dbell
 * @dbell:	 HSP dbell to be checked
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_enabled(enum tegra_hsp_doorbell dbell)
{
	int reg;
	BUG_ON(hsp_top.status != HSP_INIT_OKAY);
	if (dbell >= HSP_NR_DBS)
		return -EINVAL;
	reg = hsp_readl(db_bases[dbell], HSP_DB_REG_ENABLE);
	return reg & (1 << (16 + HSP_DB_MASTER_CCPLEX));
}
EXPORT_SYMBOL(tegra_hsp_db_enabled);

/**
 * tegra_hsp_db_listen: provide a callback for CPU doorbell ISR
 * @callback:	doorbell ISR callback
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_listen(irq_handler_t callback)
{
	if (!callback)
		return -EINVAL;
	db_irq_cb = callback;
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_listen);

/**
 * tegra_hsp_db_get_pending: get pending signals to ccplex
 *
 * Returns a mask of pending signals.
 */
u32 tegra_hsp_db_get_pending(void)
{
	BUG_ON(hsp_top.status != HSP_INIT_OKAY);
	return hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING);
}
EXPORT_SYMBOL(tegra_hsp_db_get_pending);

/**
 * tegra_hsp_db_clr_pending: clear all pending signals of ccplex
 */
void tegra_hsp_db_clr_pending(u32 mask)
{
	BUG_ON(hsp_top.status != HSP_INIT_OKAY);
	hsp_writel(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING, mask);
}
EXPORT_SYMBOL(tegra_hsp_db_clr_pending);

/**
 * tegra_hsp_get_init_status: get the result of tegra_hsp_probe
 */
enum tegra_hsp_init_status tegra_hsp_get_init_status(void)
{
	return hsp_top.status;
}
EXPORT_SYMBOL(tegra_hsp_get_init_status);

static int tegra_hsp_probe(struct platform_device *pdev)
{
	int i;
	int irq;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	if (!np) {
		dev_err(dev, "DT data required.\n");
		return -EINVAL;
	}

	hsp_top.status = HSP_INIT_FAILED;

	ret |= of_property_read_u32(np, "num-SM", &hsp_top.nr_sm);
	ret |= of_property_read_u32(np, "num-AS", &hsp_top.nr_as);
	ret |= of_property_read_u32(np, "num-SS", &hsp_top.nr_ss);
	ret |= of_property_read_u32(np, "num-DB", &hsp_top.nr_db);
	ret |= of_property_read_u32(np, "num-SI", &hsp_top.nr_si);

	if (ret) {
		dev_err(dev, "failed to parse HSP config.\n");
		return -EINVAL;
	}

	hsp_top.base = of_iomap(np, 0);
	if (!hsp_top.base) {
		dev_err(dev, "failed to map HSP IO space.\n");
		return -EINVAL;
	}

	for (i = HSP_DB_DPMU; i < HSP_NR_DBS; i++) {
		db_bases[i] = HSP_DB_OFF(i, hsp_top);
		dev_dbg(dev, "db[%d]: %p\n", i, db_bases[i]);
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		dev_err(dev, "failed to parse doorbell interrupt.\n");
		return -EINVAL;
	}

	if (devm_request_irq(dev, irq, dbell_irq, 0, dev_name(dev), NULL)) {
		dev_err(dev, "failed to request doorbell interrupt.\n");
		return -EINVAL;
	}

	spin_lock_init(&hsp_top.lock);

	hsp_top.status = HSP_INIT_OKAY;

	return 0;
}

static const struct of_device_id tegra_hsp_of_match[] = {
	{ .compatible = "nvidia,tegra186-hsp", },
	{},
};

static struct platform_driver tegra_hsp_driver = {
	.probe	= tegra_hsp_probe,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_hsp",
		.of_match_table = of_match_ptr(tegra_hsp_of_match),
	},
};

static int __init tegra_hsp_init(void)
{
	return platform_driver_register(&tegra_hsp_driver);
}
core_initcall(tegra_hsp_init);
