/*
 * PVA ISR code for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/irq.h>

#include "bus_client.h"
#include "dev.h"
#include "pva.h"
#include "pva_regs.h"

static irqreturn_t pva_mbox_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct pva *pva = dev_id;
	struct pva_mbox_status mb_status;
	struct platform_device *pdev = pva->pdev;
	u32 status7 = host1x_readl(pdev, hsp_sm7_r());
	u32 status6 = host1x_readl(pdev, hsp_sm6_r());
	u32 status5 = host1x_readl(pdev, hsp_sm5_r());

	spin_lock_irqsave(&pva->lock, flags);

	memset(&mb_status, 0, sizeof(struct pva_mbox_status));

	if (status5)
		host1x_writel(pdev, hsp_sm5_r(), 0x0);

	if (status6)
		host1x_writel(pdev, hsp_sm6_r(), 0x0);

	if (status7) {
		/* Interrupt bits are cleared in pva_read_mbox_status() */
		pva_read_mbox_status(pdev, status7, &mb_status);
		pva_process_mbox_status(pdev, &mb_status);
	}

	spin_unlock_irqrestore(&pva->lock, flags);
	return IRQ_HANDLED;
}

int pva_register_isr(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct pva *pva = pdata->private_data;
	int err;

	pva->irq = platform_get_irq(dev, 0);
	if (pva->irq <= 0) {
		dev_err(&dev->dev, "no irq\n");
		return -ENOENT;
	}

	spin_lock_init(&pva->lock);

	err = request_threaded_irq(pva->irq, NULL, pva_mbox_isr, IRQF_ONESHOT,
			  "pva-isr", pva);
	if (err) {
		pr_err("%s: request_irq(%d) failed(%d)\n", __func__,
		pva->irq, err);
		return err;
	}

	disable_irq(pva->irq);

	return 0;
}
