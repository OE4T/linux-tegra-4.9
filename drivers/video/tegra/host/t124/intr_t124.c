/*
 * drivers/video/tegra/host/t124/intr_t124.c
 *
 * Tegra Graphics Host Interrupt Management
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/mach/irq.h>

#include "../nvhost_intr.h"
#include "../dev.h"

#include "t124.h"
#include "hardware_t124.h"

#include "chip_support.h"

static void syncpt_thresh_mask(struct irq_data *data)
{
	(void)data;
}

static void syncpt_thresh_unmask(struct irq_data *data)
{
	(void)data;
}

static void syncpt_thresh_cascade(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *sync_regs = irq_desc_get_handler_data(desc);
	unsigned long reg;
	int i, id;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	for (i = 0; i < INT_SYNCPT_THRESH_NR / 32; i++) {
		reg = readl(sync_regs +
			host1x_sync_syncpt_thresh_cpu0_int_status_0_r() + i * 4);

		for_each_set_bit(id, &reg, 32)
			generic_handle_irq(id + INT_SYNCPT_THRESH_BASE + i *32);
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip syncpt_thresh_irq = {
	.name		= "syncpt",
	.irq_mask	= syncpt_thresh_mask,
	.irq_unmask	= syncpt_thresh_unmask
};

static void t124_intr_init_host_sync(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	int i, irq;

	writel(0xffffffffUL,
		sync_regs + host1x_sync_syncpt_thresh_int_disable_0_r());
	writel(0xffffffffUL,
		sync_regs + host1x_sync_syncpt_thresh_cpu0_int_status_0_r());

	for (i = 0; i < INT_SYNCPT_THRESH_NR; i++) {
		irq = INT_SYNCPT_THRESH_BASE + i;
		irq_set_chip_and_handler(irq, &syncpt_thresh_irq,
			handle_simple_irq);
		irq_set_chip_data(irq, sync_regs);
		set_irq_flags(irq, IRQF_VALID);
	}
	irq_set_chained_handler(INT_HOST1X_MPCORE_SYNCPT,
		syncpt_thresh_cascade);
	irq_set_handler_data(INT_HOST1X_MPCORE_SYNCPT, sync_regs);

	nvhost_dbg_fn("");
	/* disable the ip_busy_timeout. this prevents write drops, etc.
	 * there's no real way to recover from a hung client anyway.
	 */
	writel(0, sync_regs + host1x_sync_ip_busy_timeout_0_r());

	/* increase the auto-ack timout to the maximum value.
	 *  T124?
	 */
	writel(0xff, sync_regs + host1x_sync_ctxsw_timeout_cfg_0_r());
}

static void t124_intr_set_host_clocks_per_usec(struct nvhost_intr *intr, u32 cpm)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;

	nvhost_dbg_fn("");
	/* write microsecond clock register */
	writel(cpm, sync_regs + host1x_sync_usec_clk_0_r());
}

static void t124_intr_set_syncpt_threshold(struct nvhost_intr *intr,
					  u32 id, u32 thresh)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;

	nvhost_dbg_fn("");
	thresh &= 0xffff;
	writel(thresh, sync_regs + (host1x_sync_syncpt_int_thresh_0_0_r() +
				    id * 4));
}

static void t124_intr_enable_syncpt_intr(struct nvhost_intr *intr, u32 id)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 reg_offset = (id / 32) * 4;

	nvhost_dbg_fn("");
	BUG_ON(reg_offset > (nvhost_syncpt_nb_pts(&dev->syncpt) / 32) * 4);
	writel(BIT(id & (32 - 1)), sync_regs +
		host1x_sync_syncpt_thresh_int_enable_cpu0_0_r() + reg_offset);
}

static void t124_intr_disable_syncpt_intr(struct nvhost_intr *intr, u32 id)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 reg_offset = (id / 32) * 4;

	nvhost_dbg_fn("");
	BUG_ON(reg_offset > (nvhost_syncpt_nb_pts(&dev->syncpt) / 32) * 4);
	writel(BIT(id & (32 - 1)),
	       sync_regs +
		host1x_sync_syncpt_thresh_int_disable_0_r() + reg_offset);
	writel(BIT(id & (32 - 1)),
	       sync_regs +
		host1x_sync_syncpt_thresh_cpu0_int_status_0_r() + reg_offset);
}

static void t124_intr_disable_all_syncpt_intrs(struct nvhost_intr *intr)
{
	struct nvhost_master *dev = intr_to_dev(intr);
	void __iomem *sync_regs = dev->sync_aperture;
	u32 reg_offset;

	nvhost_dbg_fn("");

	for (reg_offset = 0;
	     reg_offset <= (nvhost_syncpt_nb_pts(&dev->syncpt) / 32) * 4;
	     reg_offset += 4) {

		/* disable interrupts for both cpu's */
		writel(0, sync_regs +
		       host1x_sync_syncpt_thresh_int_disable_0_r() +
		       reg_offset);

		/* clear status for both cpu's */
		writel(0xfffffffful, sync_regs +
			host1x_sync_syncpt_thresh_cpu0_int_status_0_r() +
		       reg_offset);

		writel(0xfffffffful, sync_regs +
			host1x_sync_syncpt_thresh_cpu1_int_status_0_r() +
		       reg_offset);
	}
}

/**
 * Sync point threshold interrupt service function
 * Handles sync point threshold triggers, in interrupt context
 */
irqreturn_t t124_intr_syncpt_thresh_isr(int irq, void *dev_id)
{
	struct nvhost_intr_syncpt *syncpt = dev_id;
	unsigned int id = syncpt->id;
	struct nvhost_intr *intr = intr_syncpt_to_intr(syncpt);
	void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;

	u32 reg_offset = (id / 32) * 4;
	id &= 32 - 1;

	nvhost_dbg_fn("");
	writel(BIT(id),
	       sync_regs +
		host1x_sync_syncpt_thresh_int_disable_0_r() + reg_offset);
	writel(BIT(id),
	       sync_regs +
		host1x_sync_syncpt_thresh_cpu0_int_status_0_r() + reg_offset);

	return IRQ_WAKE_THREAD;
}

/**
 * Host general interrupt service function
 * Handles read / write failures
 */
static irqreturn_t t124_intr_host1x_isr(int irq, void *dev_id)
{
	struct nvhost_intr *intr = dev_id;
	void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;
	u32 stat;
	u32 ext_stat;
	u32 addr;
	nvhost_dbg_fn("");

	stat = readl(sync_regs + host1x_sync_hintstatus_0_r());
	ext_stat = readl(sync_regs + host1x_sync_hintstatus_ext_0_r());

	if (nvhost_sync_hintstatus_ext_ip_read_int(ext_stat)) {
		addr = readl(sync_regs +
			     host1x_sync_ip_read_timeout_addr_0_r());
		pr_err("Host read timeout at address %x\n", addr);
	}

	if (nvhost_sync_hintstatus_ext_ip_write_int(ext_stat)) {
		addr = readl(sync_regs +
			     host1x_sync_ip_write_timeout_addr_0_r());
		pr_err("Host write timeout at address %x\n", addr);
	}

	writel(ext_stat, sync_regs + host1x_sync_hintstatus_ext_0_r());
	writel(stat, sync_regs + host1x_sync_hintstatus_0_r());

	return IRQ_HANDLED;
}

static int t124_intr_request_host_general_irq(struct nvhost_intr *intr)
{
	void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;
	int err;
	nvhost_dbg_fn("");

	if (intr->host_general_irq_requested)
		return 0;

	/* master disable for general (not syncpt) host interrupts */
	writel(0, sync_regs + host1x_sync_intmask_0_r());

	/* clear status & extstatus */
	writel(0xfffffffful, sync_regs + host1x_sync_hintstatus_ext_0_r());
	writel(0xfffffffful, sync_regs + host1x_sync_hintstatus_ext_0_r());

	err = request_irq(intr->host_general_irq, t124_intr_host1x_isr, 0,
			  "host_status", intr);
	if (err)
		return err;
	/* enable extra interrupt sources IP_READ_INT and IP_WRITE_INT */
	writel(BIT(30) | BIT(31), sync_regs + host1x_sync_hintmask_ext_0_r());

	/* enable extra interrupt sources */
	writel(BIT(31), sync_regs + host1x_sync_hintmask_0_r());

	/* enable host module interrupt to CPU0 */
	writel(BIT(0), sync_regs + host1x_sync_intc0mask_0_r());

	/* master enable for general (not syncpt) host interrupts */
	writel(BIT(0), sync_regs + host1x_sync_intmask_0_r());

	intr->host_general_irq_requested = true;

	return err;
}

static void t124_intr_free_host_general_irq(struct nvhost_intr *intr)
{
	nvhost_dbg_fn("");
	if (intr->host_general_irq_requested) {
		void __iomem *sync_regs = intr_to_dev(intr)->sync_aperture;

		/* master disable for general (not syncpt) host interrupts */
		writel(0, sync_regs + host1x_sync_intmask_0_r());

		free_irq(intr->host_general_irq, intr);
		intr->host_general_irq_requested = false;
	}
}

static int t124_request_syncpt_irq(struct nvhost_intr_syncpt *syncpt)
{
	int err;
	if (syncpt->irq_requested)
		return 0;

	err = request_threaded_irq(syncpt->irq,
				   t124_intr_syncpt_thresh_isr, nvhost_syncpt_thresh_fn,
				   0, syncpt->thresh_irq_name, syncpt);
	if (err)
		return err;

	syncpt->irq_requested = 1;
	return 0;
}

int nvhost_init_t124_intr_support(struct nvhost_chip_support *op)
{
	op->intr.init_host_sync = t124_intr_init_host_sync;
	op->intr.set_host_clocks_per_usec =
	  t124_intr_set_host_clocks_per_usec;
	op->intr.set_syncpt_threshold = t124_intr_set_syncpt_threshold;
	op->intr.enable_syncpt_intr = t124_intr_enable_syncpt_intr;
	op->intr.disable_syncpt_intr = t124_intr_disable_syncpt_intr;
	op->intr.disable_all_syncpt_intrs =
	  t124_intr_disable_all_syncpt_intrs;
	op->intr.request_host_general_irq =
	  t124_intr_request_host_general_irq;
	op->intr.free_host_general_irq =
	  t124_intr_free_host_general_irq;
	op->intr.request_syncpt_irq =
		t124_request_syncpt_irq;

	return 0;
}
