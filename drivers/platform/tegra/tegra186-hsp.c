/*
 * Copyright (c) 2016 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <linux/tegra-hsp.h>

#define NV(p) "nvidia," #p

struct tegra_hsp {
	void __iomem *base;
	struct hlist_head sm_pairs;
	struct mutex lock;
	u8 n_sm;
	u8 n_as;
	u8 n_ss;
	u8 n_db;
	u8 n_si;
	u8 si_index;
};

#define TEGRA_HSP_IE(si)		(0x100 + (4 * (si)))
#define TEGRA_HSP_IE_SM_EMPTY(sm)	(0x1u << (sm))
#define TEGRA_HSP_IE_SM_FULL(sm)	(0x100u << (sm))
#define TEGRA_HSP_IE_DB(db)		(0x10000u << (db))
#define TEGRA_HSP_IE_AS(as)		(0x1000000u << (as))
#define TEGRA_HSP_DIMENSIONING		0x380
#define TEGRA_HSP_SM(sm)		(0x10000 + (0x8000 * (sm)))
#define TEGRA_HSP_SM_FULL		0x80000000u

static void __iomem *tegra_hsp_reg(struct device *dev, u32 offset)
{
	struct tegra_hsp *hsp = dev_get_drvdata(dev);

	return hsp->base + offset;
}

static void tegra_hsp_ie_writel(struct device *dev, u32 value)
{
	struct tegra_hsp *hsp = dev_get_drvdata(dev);

	writel(value, tegra_hsp_reg(dev, TEGRA_HSP_IE(hsp->si_index)));
}

static void __iomem *tegra_hsp_sm_reg(struct device *dev, u32 sm)
{
	return tegra_hsp_reg(dev, TEGRA_HSP_SM(sm));
}

static irqreturn_t tegra_hsp_isr(int irq, void *data)
{
	struct device *dev = data;
	struct tegra_hsp *hsp = dev_get_drvdata(dev);
	struct tegra_hsp_sm_pair *pair;

	rcu_read_lock();
	hlist_for_each_entry_rcu(pair, &hsp->sm_pairs, node) {
		void __iomem *reg = tegra_hsp_sm_reg(dev, pair->index);
		u32 value = readl(reg);

		if (!(value & TEGRA_HSP_SM_FULL))
			continue;

		if (pair->notify_full != NULL) {
			value &= ~TEGRA_HSP_SM_FULL;
			value = pair->notify_full(pair, value);
			value &= ~TEGRA_HSP_SM_FULL;
		} else
			value = 0;

		writel(value, reg);
	}
	rcu_read_unlock();

	return IRQ_HANDLED;
}

static int tegra_hsp_get_shared_irq(struct device *dev, u8 *offset,
					irq_handler_t handler, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_hsp *hsp = dev_get_drvdata(dev);
	int ret = -ENODEV;
	unsigned i;

	for (i = 0; i < hsp->n_si; i++) {
		char irqname[8];
		int irq;

		sprintf(irqname, "shared%X", i);
		irq = platform_get_irq_byname(pdev, irqname);
		if (irq < 0) {
			ret = irq;
			continue;
		}

		*offset = i;

		ret = devm_request_threaded_irq(dev, irq, NULL, handler,
						IRQF_ONESHOT, dev_name(dev),
						data);
		if (ret)
			continue;

		dev_dbg(&pdev->dev, "using shared IRQ %u (%d)\n", i, irq);
		return irq;
	}

	if (ret != -EPROBE_DEFER)
		dev_err(dev, "cannot get shared IRQ: %d\n", ret);
	return ret;
}

static void tegra_hsp_update_ie(struct device *dev)
{
	struct tegra_hsp *hsp = dev_get_drvdata(dev);
	struct tegra_hsp_sm_pair *pair;
	u32 ie = 0;

	WARN_ON(!mutex_is_locked(&hsp->lock));

	hlist_for_each_entry(pair, &hsp->sm_pairs, node)
		ie |= TEGRA_HSP_IE_SM_FULL(pair->index);

	tegra_hsp_ie_writel(dev, ie);
}

static int tegra_hsp_sm_pair_request(struct device *dev, u32 index,
					struct tegra_hsp_sm_pair *pair)
{
	struct tegra_hsp *hsp = dev_get_drvdata(dev);

	if (hsp == NULL)
		return -EPROBE_DEFER;
	if (index >= hsp->n_sm)
		return -ENODEV;

	pair->dev = get_device(dev);
	pair->index = index;

	mutex_lock(&hsp->lock);
	hlist_add_head_rcu(&pair->node, &hsp->sm_pairs);
	tegra_hsp_update_ie(dev);
	mutex_unlock(&hsp->lock);
	return 0;
}

int of_tegra_hsp_sm_pair_request(struct device *dev, u32 index,
					struct tegra_hsp_sm_pair *pair)
{
	struct platform_device *pdev;
	struct of_phandle_args smspec;
	int ret;

	ret = of_parse_phandle_with_fixed_args(dev->of_node,
						NV(hsp-shared-mailbox), 1,
						index, &smspec);
	if (ret)
		return ret;

	pdev = of_find_device_by_node(smspec.np);
	index = smspec.args[0];
	of_node_put(smspec.np);

	if (pdev == NULL)
		return -EPROBE_DEFER;

	ret = tegra_hsp_sm_pair_request(&pdev->dev, index, pair);
	platform_device_put(pdev);
	return ret;
}
EXPORT_SYMBOL(of_tegra_hsp_sm_pair_request);

void tegra_hsp_sm_pair_free(struct tegra_hsp_sm_pair *pair)
{
	struct device *dev = pair->dev;
	struct tegra_hsp *hsp;

	if (pair->dev == NULL)
		return;

	hsp = dev_get_drvdata(dev);

	mutex_lock(&hsp->lock);
	hlist_del_rcu(&pair->node);
	tegra_hsp_update_ie(dev);
	mutex_unlock(&hsp->lock);

	/* Make sure that the structure is no longer referenced.
	 * This also implies that callbacks are no longer pending. */
	synchronize_rcu();
	put_device(dev);
}
EXPORT_SYMBOL(tegra_hsp_sm_pair_free);

void tegra_hsp_sm_pair_write(struct tegra_hsp_sm_pair *pair,
				u32 value)
{
	struct device *dev = pair->dev;
	void __iomem *reg = tegra_hsp_sm_reg(dev, pair->index ^ 1);

	writel(TEGRA_HSP_SM_FULL | value, reg);
}
EXPORT_SYMBOL(tegra_hsp_sm_pair_write);

static const struct of_device_id tegra_hsp_of_match[] = {
	{ .compatible = NV(tegra186-hsp), },
	{ },
};

static int tegra_hsp_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *r;
	struct tegra_hsp *hsp;
	int ret;
	u32 reg;

	if (np == NULL)
		return -ENXIO;

	hsp = devm_kzalloc(&pdev->dev, sizeof(*hsp), GFP_KERNEL);
	if (unlikely(hsp == NULL))
		return -ENOMEM;

	platform_set_drvdata(pdev, hsp);
	INIT_HLIST_HEAD(&hsp->sm_pairs);
	mutex_init(&hsp->lock);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL)
		return -EINVAL;

	if (resource_size(r) < 0x10000) {
		dev_err(&pdev->dev, "memory range too short");
		return -EINVAL;
	}

	hsp->base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (hsp->base == NULL)
		return -ENOMEM;

	reg = readl(tegra_hsp_reg(&pdev->dev, TEGRA_HSP_DIMENSIONING));
	hsp->n_sm = reg & 0xf;
	hsp->n_ss = (reg >> 4) & 0xf;
	hsp->n_as = (reg >> 8) & 0xf;
	hsp->n_db = (reg >> 12) & 0xf;
	hsp->n_si = (reg >> 16) & 0xf;

	if ((resource_size(r) >> 16) < (1 + (hsp->n_sm / 2) + hsp->n_ss +
					hsp->n_as + (hsp->n_db > 0))) {
		dev_err(&pdev->dev, "memory range too short");
		return -EINVAL;
	}

	ret = tegra_hsp_get_shared_irq(&pdev->dev, &hsp->si_index,
					tegra_hsp_isr, &pdev->dev);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "no shared interrupts\n");
		return ret;
	}

	return 0;
}

static int tegra_hsp_remove(struct platform_device *pdev)
{
	struct tegra_hsp *hsp = platform_get_drvdata(pdev);

	WARN_ON(!hlist_empty(&hsp->sm_pairs));
	return 0;
}

static struct platform_driver tegra_hsp_driver = {
	.probe	= tegra_hsp_probe,
	.remove	= tegra_hsp_remove,
	.driver = {
		.name	= "tegra186-hsp",
		.owner	= THIS_MODULE,
		.suppress_bind_attrs = true,
		.of_match_table = of_match_ptr(tegra_hsp_of_match),
	},
};
module_platform_driver(tegra_hsp_driver);
MODULE_AUTHOR("Remi Denis-Courmont <remid@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra 186 HSP driver");
MODULE_LICENSE("GPL");
