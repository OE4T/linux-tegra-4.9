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
#include <linux/rculist.h>

#include <linux/tegra-hsp.h>

#define NV(p) "nvidia," #p

struct tegra_hsp {
	void __iomem *base;
	struct mutex lock;
	u8 n_sm;
	u8 n_as;
	u8 n_ss;
	u8 n_db;
	u8 n_si;
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

static void __iomem *tegra_hsp_ie_reg(struct device *dev, u8 si)
{
	return tegra_hsp_reg(dev, TEGRA_HSP_IE(si));
}

static void __iomem *tegra_hsp_sm_reg(struct device *dev, u32 sm)
{
	return tegra_hsp_reg(dev, TEGRA_HSP_SM(sm));
}

static irqreturn_t tegra_hsp_full_isr(int irq, void *data)
{
	struct tegra_hsp_irq *hi = data;
	struct tegra_hsp_sm_pair *pair =
		container_of(hi, struct tegra_hsp_sm_pair, full);
	struct device *dev = pair->dev;
	void __iomem *reg = tegra_hsp_sm_reg(dev, pair->index);
	u32 value = readl(reg);

	if (!(value & TEGRA_HSP_SM_FULL))
		return IRQ_NONE;

	if (pair->notify_full != NULL)
		value = pair->notify_full(pair, value & ~TEGRA_HSP_SM_FULL);
	else
		value = 0;

	/* Write new value to empty the mailbox and clear the interrupt */
	writel(value & ~TEGRA_HSP_SM_FULL, reg);
	return IRQ_HANDLED;
}

static irqreturn_t tegra_hsp_empty_isr(int irq, void *data)
{
	struct tegra_hsp_irq *hi = data;
	struct tegra_hsp_sm_pair *pair =
		container_of(hi, struct tegra_hsp_sm_pair, empty);
	struct device *dev = pair->dev;
	void __iomem *reg = tegra_hsp_sm_reg(dev, pair->index ^ 1);
	u32 value = readl(reg);

	if (value & TEGRA_HSP_SM_FULL)
		return IRQ_NONE;

	disable_irq_nosync(irq);
	pair->notify_empty(pair, value);
	return IRQ_HANDLED;
}

static int tegra_hsp_get_shared_irq(struct device *dev, irq_handler_t handler,
					unsigned long flags, u32 ie_shift,
					struct tegra_hsp_irq *hi)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_hsp *hsp = dev_get_drvdata(dev);
	int ret = -ENODEV;
	unsigned i;

	flags |= IRQF_PROBE_SHARED;

	for (i = 0; i < hsp->n_si; i++) {
		void __iomem *reg = tegra_hsp_ie_reg(dev, i);
		char irqname[8];

		sprintf(irqname, "shared%X", i);
		hi->irq = platform_get_irq_byname(pdev, irqname);
		if (IS_ERR_VALUE(hi->irq))
			continue;

		hi->si_index = i;
		hi->ie_shift = ie_shift;

		ret = request_threaded_irq(hi->irq, NULL, handler, flags,
						dev_name(dev), hi);
		if (ret)
			continue;

		dev_dbg(&pdev->dev, "using shared IRQ %u (%d)\n", i, hi->irq);

		/* Update interrupt masks (for shared interrupts only) */
		mutex_lock(&hsp->lock);
		writel(readl(reg) | (1u << ie_shift), reg);
		mutex_unlock(&hsp->lock);
		return 0;
	}

	if (ret != -EPROBE_DEFER)
		dev_err(dev, "cannot get shared IRQ: %d\n", ret);
	return ret;
}

static int tegra_hsp_get_sm_irq(struct device *dev, u8 sm, bool empty,
				struct tegra_hsp_irq *hi)
{
	struct platform_device *pdev = to_platform_device(dev);
	irq_handler_t handler = empty ? tegra_hsp_empty_isr
					: tegra_hsp_full_isr;
	unsigned long flags = (empty ? 0 : IRQF_SHARED) | IRQF_ONESHOT;
	u32 ie_shift = (empty ? 0 : 8) + sm;
	char name[7];

	/* Look for dedicated internal IRQ */
	sprintf(name, empty ? "empty%X" : "full%X", sm);
	hi->irq = platform_get_irq_byname(pdev, name);
	if (!IS_ERR_VALUE(hi->irq)) {
		hi->si_index = 0xff;

		if (request_threaded_irq(hi->irq, NULL, handler, flags,
						dev_name(dev), hi) == 0)
			return 0;
	}

	/* Look for a free shared IRQ */
	return tegra_hsp_get_shared_irq(dev, handler, flags, ie_shift, hi);
}

static void tegra_hsp_free_irq(struct device *dev, struct tegra_hsp_irq *hi)
{
	struct tegra_hsp *hsp = dev_get_drvdata(dev);

	if (hi->si_index != 0xff) {
		void __iomem *reg = tegra_hsp_ie_reg(dev, hi->si_index);

		mutex_lock(&hsp->lock);
		writel(readl(reg) & ~(1u << hi->ie_shift), reg);
		mutex_unlock(&hsp->lock);
	}
	free_irq(hi->irq, hi);
}

static int tegra_hsp_sm_pair_request(struct device *dev, u32 index,
					struct tegra_hsp_sm_pair *pair)
{
	struct tegra_hsp *hsp = dev_get_drvdata(dev);
	int err;

	if (hsp == NULL)
		return -EPROBE_DEFER;
	if (index >= hsp->n_sm)
		return -ENODEV;

	WARN_ON(pair->dev != NULL);
	pair->dev = get_device(dev);
	pair->index = index;

	/* Get empty interrupt if necessary */
	if (pair->notify_empty != NULL) {
		err = tegra_hsp_get_sm_irq(dev, pair->index ^ 1, true,
						&pair->empty);
		if (err)
			goto error;
	}

	/* Get full interrupt */
	err = tegra_hsp_get_sm_irq(dev, pair->index, false, &pair->full);
	if (err) {
		if (pair->notify_empty != NULL)
			tegra_hsp_free_irq(dev, &pair->empty);
		goto error;
	}

	return 0;

error:
	put_device(pair->dev);
	pair->dev = NULL;
	return err;
}

/**
 * of_tegra_sm_pair_request - request a Tegra HSP shared mailbox pair from DT.
 *
 * @np: device node
 * @index: mailbox pair entry offset in the DT property
 * @pair: initialized HSP mailbox structure
 *
 * Looks up a shared mailbox pair in device tree by index. The device node
 * needs a nvidia,hsp-shared-mailbox property, containing pairs of
 * OF phandle and mailbox number. The OF phandle points to the Tegra HSP
 * platform device. The mailbox number refers to the consumer side mailbox.
 * The producer side mailbox is the other one in the same (even-odd) pair.
 */
int of_tegra_hsp_sm_pair_request(const struct device_node *np, u32 index,
					struct tegra_hsp_sm_pair *pair)
{
	struct platform_device *pdev;
	struct of_phandle_args smspec;
	int ret;

	ret = of_parse_phandle_with_fixed_args(np, NV(hsp-shared-mailbox), 1,
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

/**
 * of_tegra_sm_pair_by_name - request a Tegra HSP shared mailbox pair from DT.
 *
 * @np: device node
 * @name: mailbox pair entry name
 * @pair: initialized HSP mailbox structure
 *
 * Looks up a shared mailbox pair in device tree by name. The device node needs
 * nvidia,hsp-shared-mailbox and nvidia-hsp-shared-mailbox-names properties.
 */
int of_tegra_hsp_sm_pair_by_name(const struct device_node *np, char const *name,
					struct tegra_hsp_sm_pair *pair)
{
	int index;

	/* The of_property_match_string() prototype will be fixed in _next */
	/* Until then, we have to cast to non-const */
	/* If match fails, index will be -1 and parse_phandles fails */
	index = of_property_match_string((struct device_node *)np,
			NV(hsp-shared-mailbox-names), name);

	return of_tegra_hsp_sm_pair_request(np, index, pair);
}
EXPORT_SYMBOL(of_tegra_hsp_sm_pair_by_name);

/**
 * tegra_hsp_sm_pair_free - free a Tegra HSP shared mailbox pair.
 */
void tegra_hsp_sm_pair_free(struct tegra_hsp_sm_pair *pair)
{
	struct device *dev = pair->dev;
	struct tegra_hsp *hsp;

	if (pair->dev == NULL)
		return;

	hsp = dev_get_drvdata(dev);

	/* Make sure that the structure is no longer referenced.
	 * This also implies that callbacks are no longer pending. */
	tegra_hsp_free_irq(dev, &pair->full);
	if (pair->notify_empty != NULL)
		tegra_hsp_free_irq(dev, &pair->empty);

	put_device(dev);
	pair->dev = NULL;
}
EXPORT_SYMBOL(tegra_hsp_sm_pair_free);

/**
 * tegra_hsp_sm_pair_write - fill a Tegra HSP shared mailbox
 *
 * @pair: shared mailbox pair
 * @value: value to fill mailbox with (only 31-bits low order bits are used)
 *
 * This writes a value to the producer side mailbox of a mailbox pair.
 * The mailbox must be empty (especially if notify_empty callback is non-nul).
 */
void tegra_hsp_sm_pair_write(const struct tegra_hsp_sm_pair *pair,
				u32 value)
{
	struct device *dev = pair->dev;
	void __iomem *reg = tegra_hsp_sm_reg(dev, pair->index ^ 1);

	/* Ensure any pending empty ISR invocation has disabled the IRQ */
	if (pair->notify_empty != NULL) {
		might_sleep();
		synchronize_irq(pair->empty.irq);
	}

	writel(TEGRA_HSP_SM_FULL | value, reg);

	if (pair->notify_empty != NULL)
		enable_irq(pair->empty.irq);
}
EXPORT_SYMBOL(tegra_hsp_sm_pair_write);

bool tegra_hsp_sm_pair_is_empty(const struct tegra_hsp_sm_pair *pair)
{
	struct device *dev = pair->dev;
	u32 cvalue, pvalue;

	/* Ensure any pending full ISR invocation has emptied the mailbox */
	might_sleep();
	synchronize_irq(pair->full.irq);

	pvalue = readl(tegra_hsp_sm_reg(dev, pair->index ^ 1));
	cvalue = readl(tegra_hsp_sm_reg(dev, pair->index));
	return ((pvalue|cvalue) & TEGRA_HSP_SM_FULL) == 0;
}
EXPORT_SYMBOL(tegra_hsp_sm_pair_is_empty);

static const struct of_device_id tegra_hsp_of_match[] = {
	{ .compatible = NV(tegra186-hsp), },
	{ },
};

static int tegra_hsp_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *r;
	struct tegra_hsp *hsp;
	u32 reg;

	if (np == NULL)
		return -ENXIO;

	hsp = devm_kzalloc(&pdev->dev, sizeof(*hsp), GFP_KERNEL);
	if (unlikely(hsp == NULL))
		return -ENOMEM;

	platform_set_drvdata(pdev, hsp);
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

	return 0;
}

static struct platform_driver tegra_hsp_driver = {
	.probe	= tegra_hsp_probe,
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
