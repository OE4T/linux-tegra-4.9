/*
 * drivers/video/tegra/host/vi/vi_irq.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/nvhost.h>

#include "nvhost_acm.h"
#include "vi.h"

int vi_enable_irq(struct vi *tegra_vi)
{
	int val;
	int err = 0;

	err = nvhost_module_busy(tegra_vi->ndev);
	if (err)
		return err;

	if (tegra_vi->ndev->id) {
		/* Reset VI status register */
		val = host1x_readl(tegra_vi->ndev,
					CSI_CSI_PIXEL_PARSER_B_STATUS_0);

		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_B_STATUS_0,
				val);

		/* Enable FIFO Overflow Interrupt */
		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK_0,
				PPB_FIFO_OVRF);
	} else {
		/* Reset VI status register */
		val = host1x_readl(tegra_vi->ndev,
					CSI_CSI_PIXEL_PARSER_A_STATUS_0);

		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_A_STATUS_0,
				val);

		/* Enable FIFO Overflow Interrupt */
		host1x_writel(tegra_vi->ndev,
			CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK_0,
				PPA_FIFO_OVRF);

		/* interrupts are associated only with master dev vi.0 */
		enable_irq(tegra_vi->vi_irq);
	}

	nvhost_module_idle(tegra_vi->ndev);

	return 0;
}
EXPORT_SYMBOL(vi_enable_irq);

int vi_disable_irq(struct vi *tegra_vi)
{
	int val;
	int err = 0;

	err = nvhost_module_busy(tegra_vi->ndev);
	if (err)
		return err;

	if (tegra_vi->ndev->id) {
		/* Disable FIFO Overflow Interrupt */
		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK_0,
				0);

		/* Reset status register */
		val = host1x_readl(tegra_vi->ndev,
					CSI_CSI_PIXEL_PARSER_B_STATUS_0);

		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_B_STATUS_0,
				val);
	} else {
		/* interrupts are associated only with master dev vi.0 */
		disable_irq(tegra_vi->vi_irq);

		/* Disable FIFO Overflow Interrupt */
		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK_0,
				0);

		/* Reset status register */
		val = host1x_readl(tegra_vi->ndev,
					CSI_CSI_PIXEL_PARSER_A_STATUS_0);

		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_A_STATUS_0,
				val);
	}

	nvhost_module_idle(tegra_vi->ndev);

	return 0;
}
EXPORT_SYMBOL(vi_disable_irq);

static irqreturn_t vi_checkwd(struct vi *tegra_vi, int stream)
{
	int err_addr, wd_addr, val;

	switch (stream) {
	case 0:
		err_addr = VI_CSI_0_ERROR_STATUS;
		wd_addr = VI_CSI_0_WD_CTRL;
		break;
	case 1:
		err_addr = VI_CSI_1_ERROR_STATUS;
		wd_addr = VI_CSI_1_WD_CTRL;
		break;
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	case 2:
		err_addr = VI_CSI_2_ERROR_STATUS;
		wd_addr = VI_CSI_2_WD_CTRL;
		break;
	case 3:
		err_addr = VI_CSI_3_ERROR_STATUS;
		wd_addr = VI_CSI_3_WD_CTRL;
		break;
#endif
	default:
		return IRQ_NONE;
	}

	val = host1x_readl(tegra_vi->ndev, err_addr);
	if (val & 0x20) {
		host1x_writel(tegra_vi->ndev, wd_addr, 0);
		host1x_writel(tegra_vi->ndev, err_addr, 0x20);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t vi_isr(int irq, void *dev_id)
{
	struct vi *tegra_vi = (struct vi *)dev_id;
	int i, val;
	irqreturn_t result;

	for (i = 0; i < NUM_VI_WATCHDOG; i++) {
		result = vi_checkwd(tegra_vi, i);
		if (result == IRQ_HANDLED)
			goto handled;
	}

	dev_dbg(&tegra_vi->ndev->dev, "%s: ++", __func__);

	if (tegra_vi->ndev->id) {
		val = host1x_readl(tegra_vi->ndev,
					CSI_CSI_PIXEL_PARSER_B_STATUS_0);

		/* changes required as per t124 register spec */
		if (val & PPB_FIFO_OVRF)
			atomic_inc(&(tegra_vi->vi_out.overflow));

		/* Reset interrupt status register */
		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_B_STATUS_0,
				val);
	} else {
		val = host1x_readl(tegra_vi->ndev,
					CSI_CSI_PIXEL_PARSER_A_STATUS_0);

		/* changes required as per t124 register spec */
		if (val & PPA_FIFO_OVRF)
			atomic_inc(&(tegra_vi->vi_out.overflow));

		/* Reset interrupt status register */
		host1x_writel(tegra_vi->ndev,
				CSI_CSI_PIXEL_PARSER_A_STATUS_0,
				val);
	}

	schedule_work(&tegra_vi->stats_work);

handled:
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(vi_isr);

void vi_stats_worker(struct work_struct *work)
{
	struct vi *tegra_vi = container_of(work, struct vi, stats_work);

	dev_dbg(&tegra_vi->ndev->dev,
		"%s: vi[%d]_out dropped data %u times", __func__,
		tegra_vi->ndev->id,
		atomic_read(&(tegra_vi->vi_out.overflow)));
}
EXPORT_SYMBOL(vi_stats_worker);

int vi_intr_init(struct vi *tegra_vi)
{
	struct platform_device *ndev = tegra_vi->ndev;

	/* Interrupt resources are only associated with
	 * master dev vi.0 so irq must be programmed
	 * with it only.
	 */
	if (tegra_vi->ndev->id == 0) {
		int ret;

		dev_dbg(&tegra_vi->ndev->dev, "%s: ++", __func__);

		tegra_vi->vi_irq = platform_get_irq(ndev, 0);
		if (IS_ERR_VALUE(tegra_vi->vi_irq)) {
			dev_err(&tegra_vi->ndev->dev, "missing camera irq\n");
			return -ENXIO;
		}

		ret = request_irq(tegra_vi->vi_irq,
				vi_isr,
				IRQF_SHARED,
				dev_name(&tegra_vi->ndev->dev),
				tegra_vi);
		if (ret) {
			dev_err(&tegra_vi->ndev->dev, "failed to get irq\n");
			return -EBUSY;
		}

		disable_irq(tegra_vi->vi_irq);
	}

	return 0;
}
EXPORT_SYMBOL(vi_intr_init);

int vi_intr_free(struct vi *tegra_vi)
{
	dev_dbg(&tegra_vi->ndev->dev, "%s: ++", __func__);

	if (tegra_vi->ndev->id == 0)
		free_irq(tegra_vi->vi_irq, tegra_vi);

	return 0;
}
EXPORT_SYMBOL(vi_intr_free);

