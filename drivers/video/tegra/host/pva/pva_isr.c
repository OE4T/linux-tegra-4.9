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

static irqreturn_t pva_isr_work(int irq, void *dev_id)
{
	struct pva *pva = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&pva->lock, flags);

	/* TO DO: check the mailbox status register
	 * and do appropriate action
	 */

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

	err = request_threaded_irq(pva->irq, NULL, pva_isr_work, IRQF_ONESHOT,
			  "pva-isr", pva);
	if (err) {
		pr_err("%s: request_irq(%d) failed(%d)\n", __func__,
		pva->irq, err);
		return err;
	}

	disable_irq(pva->irq);

	return 0;
}
