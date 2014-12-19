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

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/tegra-hsp.h>

#define HSP_DB_REG_TRIGGER		0x0
#define HSP_DB_REG_ENABLE		0x4
#define HSP_DB_REG_RAW			0x8
#define HSP_DB_REG_PENDING		0xc

enum tegra_hsp_init_status {
	HSP_INIT_PENDING,
	HSP_INIT_FAILED,
	HSP_INIT_OKAY,
};

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

struct db_handler_info {
	db_handler_t	handler;
	void			*data;
};

static struct hsp_top hsp_top;
static void __iomem *db_bases[HSP_NR_DBS];

static DEFINE_SPINLOCK(db_handlers_lock);
static struct db_handler_info db_handlers[HSP_LAST_MASTER + 1];

#define hsp_db_offset(i, d) \
	(d.base + ((1 + (d.nr_sm>>1) + d.nr_ss + d.nr_as)<<16) + (i) * 0x100)

#define MASTER_NS_SHIFT	16
#define MASTER_NS_LIMIT	((1 << MASTER_NS_SHIFT) - 1)

#define ns_masters(m)	(m >> MASTER_NS_SHIFT)
#define master_bit(m)	(1 << (m + MASTER_NS_SHIFT))
#define bad_master(m)	(m < HSP_FIRST_MASTER || m > HSP_LAST_MASTER)
#define hsp_ready() (hsp_top.status == HSP_INIT_OKAY)

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
	ulong reg;
	int master;
	struct db_handler_info *info;
	reg = (ulong)hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING);
	hsp_writel(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING, reg);
	reg = ns_masters(reg);
	spin_lock(&db_handlers_lock);
	for_each_set_bit(master, &reg, HSP_LAST_MASTER + 1) {
		info = &db_handlers[master];
		if (info)
			info->handler(master, info->data);
	}
	spin_unlock(&db_handlers_lock);
	return IRQ_HANDLED;
}

/**
 * tegra_hsp_db_get_enabled_masters: get masters that can ring CCPLEX
 * @master:	 HSP master
 *
 * Returns the mask of enabled masters of CCPLEX.
 */
int tegra_hsp_db_get_enabled_masters(void)
{
	if (!hsp_ready())
		return -EINVAL;
	return hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE);
}
EXPORT_SYMBOL(tegra_hsp_db_get_enabled_masters);

/**
 * tegra_hsp_db_enable_master: allow <master> to ring CCPLEX
 * @master:	 HSP master
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_enable_master(enum tegra_hsp_master master)
{
	u32 reg;
	unsigned long flags;
	if (!hsp_ready() || master > HSP_LAST_MASTER)
		return -EINVAL;
	spin_lock_irqsave(&hsp_top.lock, flags);
	reg = hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE);
	reg |= master_bit(master);
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
	if (!hsp_ready() || dbell >= HSP_NR_DBS)
		return -EINVAL;
	hsp_writel(db_bases[dbell], HSP_DB_REG_TRIGGER, 1);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_ring);

/**
 * tegra_hsp_db_can_ring: check if CCPLEX can ring the <dbell>
 * @dbell:	 HSP dbell to be checked
 *
 * Returns 1 if CCPLEX can ring <dbell> otherwise 0.
 */
int tegra_hsp_db_can_ring(enum tegra_hsp_doorbell dbell)
{
	int reg;
	if (!hsp_ready() || dbell >= HSP_NR_DBS)
		return 0;
	reg = hsp_readl(db_bases[dbell], HSP_DB_REG_ENABLE);
	return !!(reg & master_bit(HSP_MASTER_CCPLEX));
}
EXPORT_SYMBOL(tegra_hsp_db_can_ring);

static int tegra_hsp_db_get_signals(u32 reg)
{
	u32 val;
	if (!hsp_ready())
		return -EINVAL;
	val = hsp_readl(db_bases[HSP_DB_CCPLEX], reg);
	return val >> MASTER_NS_SHIFT;
}

static int tegra_hsp_db_clr_signals(u32 reg, u32 mask)
{
	if (!hsp_ready() || mask > MASTER_NS_LIMIT)
		return -EINVAL;
	hsp_writel(db_bases[HSP_DB_CCPLEX], reg, mask);
	return 0;
}

/**
 * tegra_hsp_db_get_pending: get pending rings to CCPLEX
 *
 * Returns a mask of pending signals.
 */
int tegra_hsp_db_get_pending(void)
{
	return tegra_hsp_db_get_signals(HSP_DB_REG_PENDING);
}
EXPORT_SYMBOL(tegra_hsp_db_get_pending);

/**
 * tegra_hsp_db_clr_pending: clear rings of ccplex based on <mask>
 * @mask:	mask of masters to be cleared
 */
int tegra_hsp_db_clr_pending(u32 mask)
{
	return tegra_hsp_db_clr_signals(HSP_DB_REG_PENDING, mask);
}
EXPORT_SYMBOL(tegra_hsp_db_clr_pending);

/**
 * tegra_hsp_db_get_raw: get unmasked rings to CCPLEX
 *
 * Returns a mask of unmasked pending signals.
 */
int tegra_hsp_db_get_raw(void)
{
	return tegra_hsp_db_get_signals(HSP_DB_REG_RAW);
}
EXPORT_SYMBOL(tegra_hsp_db_get_raw);

/**
 * tegra_hsp_db_clr_raw: clear unmasked rings of ccplex based on <mask>
 * @mask:	mask of masters to be cleared
 */
int tegra_hsp_db_clr_raw(u32 mask)
{
	return tegra_hsp_db_clr_signals(HSP_DB_REG_RAW, mask);
}
EXPORT_SYMBOL(tegra_hsp_db_clr_raw);

/**
 * tegra_hsp_db_add_handler: register an CCPLEX doorbell IRQ handler
 * @ master:	master id
 * @ handler:	IRQ handler
 * @ data:		custom data
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_add_handler(int master, db_handler_t handler, void *data)
{
	ulong flags;
	if (!handler || bad_master(master))
		return -EINVAL;
	spin_lock_irqsave(&db_handlers_lock, flags);
	db_handlers[master].handler = handler;
	db_handlers[master].data = data;
	spin_unlock_irqrestore(&db_handlers_lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_add_handler);

/**
 * tegra_hsp_db_del_handler: unregister an CCPLEX doorbell IRQ handler
 * @handler:	IRQ handler
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_del_handler(int master)
{
	ulong flags;
	if (bad_master(master))
		return -EINVAL;
	spin_lock_irqsave(&db_handlers_lock, flags);
	db_handlers[master].handler = NULL;
	db_handlers[master].data = NULL;
	spin_unlock_irqrestore(&db_handlers_lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_del_handler);

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

	for (i = HSP_FIRST_DB; i <= HSP_LAST_DB; i++) {
		db_bases[i] = hsp_db_offset(i, hsp_top);
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

	hsp_top.status = HSP_INIT_OKAY;

	return ret;
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
