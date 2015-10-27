/*
 * VI NOTIFY driver for T186
 *
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include "dev.h"
#include "nvhost_acm.h"
#include "vi/vi_notify.h"
#include "vi/vi4.h"

#define VI_CFG_INTERRUPT_STATUS_0		0x0044
#define VI_CFG_INTERRUPT_MASK_0			0x0048

#define VI_ISPBUFA_ERROR_0			0x1000

#define VI_FMLITE_ERROR_0			0x313C

#define VI_NOTIFY_FIFO_TAG_0_0			0x4000
#define VI_NOTIFY_FIFO_TIMESTAMP_0_0		(VI_NOTIFY_FIFO_TAG_0_0 + 4)
#define VI_NOTIFY_FIFO_DATA_0_0			(VI_NOTIFY_FIFO_TAG_0_0 + 8)
#define VI_NOTIFY_TAG_CLASSIFY_NO_OUTPUT_0	0x6000
#define VI_NOTIFY_TAG_CLASSIFY_HIGH_0		0x6004
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_0		0x6008
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0	0x600C
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_TEST_0	0x6010
#define VI_NOTIFY_OCCUPANCY_0			0x6014
#define VI_NOTIFY_OCCUPANCY_URGENT_0		0x6018
#define VI_NOTIFY_HIGHPRIO_0			0x601C
#define VI_NOTIFY_ERROR_0			0x6020

#define VI_CH_REG(n, r)				((n+1) * 0x10000 + (r))
#define VI_CH_CONTROL(n)			VI_CH_REG(n, 0x1c)

#define VI_HOST_PKTINJECT_STALL_ERR_MASK	0x00000080
#define VI_CSIMUX_FIFO_OVFL_ERR_MASK		0x00000040
#define VI_ATOMP_PACKER_OVFL_ERR_MASK		0x00000020
#define VI_FMLITE_BUF_OVFL_ERR_MASK		0x00000010
#define VI_NOTIFY_FIFO_OVFL_ERR_MASK		0x00000008
#define VI_ISPBUFA_ERR_MASK			0x00000001

#define VI_CH_CONTROL_ENABLE			0x01
#define VI_CH_CONTROL_SINGLESHOT		0x02

#define VI_NOTIFY_TAG_CHANSEL_COLLISION		0x0C
#define VI_NOTIFY_TAG_CHANSEL_SHORT_FRAME	0x0D
#define VI_NOTIFY_TAG_CHANSEL_LOAD_FRAMED	0x0E
#define VI_NOTIFY_TAG_ATOMP_FS			0x10
#define VI_NOTIFY_TAG_ATOMP_FE			0x11
#define VI_NOTIFY_TAG_ISPBUF_FS			0x1B
#define VI_NOTIFY_TAG_ISPBUF_FE			0x1C

#define VI_NOTIFY_TAG_DATA_FE			0x20
#define VI_NOTIFY_TAG_DATA_LOAD_FRAMED		0x08000000

static void nvhost_vi_notify_dump_status(struct platform_device *pdev)
{
	u32 r = host1x_readl(pdev, VI_NOTIFY_OCCUPANCY_0);

	dev_dbg(&pdev->dev, "Occupancy: %u/%u (max: %u)\n",
		(r >> 10) & 0x3ff, r & 0x3ff,  (r >> 20) & 0x3ff);
	dev_dbg(&pdev->dev, "Urgent:    %u\n",
		host1x_readl(pdev, VI_NOTIFY_HIGHPRIO_0));
	dev_dbg(&pdev->dev, "Error:   0x%08X\n",
		host1x_readl(pdev, VI_NOTIFY_ERROR_0));
}

/* Interrupt handlers */
static irqreturn_t nvhost_vi_error_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);
	struct nvhost_vi_notify_dev *hvnd = &vi->notify;
	u32 r;

	r = host1x_readl(pdev, VI_NOTIFY_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_NOTIFY_ERROR_0, 1);
		dev_err(&pdev->dev, "notify buffer overflow\n");
		atomic_inc(&hvnd->notify_overflow);

		vi_notify_dev_error(hvnd->vnd);
	}

	r = host1x_readl(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0, r);
		dev_err(&pdev->dev, "safety error mask 0x%08X\n", r);
	}

	r = host1x_readl(pdev, VI_FMLITE_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_FMLITE_ERROR_0, 1);
		dev_err(&pdev->dev, "FM-Lite buffer overflow\n");
		atomic_inc(&hvnd->fmlite_overflow);
	}

	r = host1x_readl(pdev, VI_CFG_INTERRUPT_STATUS_0);
	if (r) {
		host1x_writel(pdev, VI_CFG_INTERRUPT_STATUS_0, 1);
		dev_err(&pdev->dev, "master error\n");
		atomic_inc(&hvnd->overflow);
	}

	return IRQ_HANDLED;
}

static irqreturn_t nvhost_vi_prio_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	u32 count = host1x_readl(pdev, VI_NOTIFY_HIGHPRIO_0);

	/* Not clear what to do with prioritized events: There are no ways to
	 * dequeue them out-of-band. Let the regular ISR deal with them. */
	dev_dbg(&pdev->dev, "priority count: %u", count);
	host1x_writel(pdev, VI_NOTIFY_HIGHPRIO_0, count);
	return IRQ_HANDLED;
}

static irqreturn_t nvhost_vi_notify_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);
	struct nvhost_vi_notify_dev *hvnd = &vi->notify;

	for (;;) {
		struct vi_notify_msg msg;
		u32 v;
		u8 ch;
		u8 tag;

		msg.tag = host1x_readl(pdev, VI_NOTIFY_FIFO_TAG_0_0);
		nvhost_vi_notify_dump_status(pdev);

		if (!VI_NOTIFY_TAG_VALID(msg.tag))
			break;

		tag = VI_NOTIFY_TAG_TAG(msg.tag);

		msg.stamp = host1x_readl(pdev, VI_NOTIFY_FIFO_TIMESTAMP_0_0);
		msg.data = host1x_readl(pdev, VI_NOTIFY_FIFO_DATA_0_0);
		msg.reserve = 0;

		switch (tag) {
		case VI_NOTIFY_TAG_CHANSEL_COLLISION:
		case VI_NOTIFY_TAG_CHANSEL_SHORT_FRAME:
		case VI_NOTIFY_TAG_CHANSEL_LOAD_FRAMED:
			if (!(msg.data & VI_NOTIFY_TAG_DATA_LOAD_FRAMED))
				break;
			ch = msg.data >> 28; /* yes, really */
			hvnd->ld_mask |= (1 << ch);
			break;

		case VI_NOTIFY_TAG_ATOMP_FS:
		case VI_NOTIFY_TAG_ISPBUF_FS:
			ch = VI_NOTIFY_TAG_CHANNEL(msg.tag);
			if (ch >= 12)
				break;
			hvnd->ld_mask &= ~(1 << ch);
			break;

		case VI_NOTIFY_TAG_ATOMP_FE:
		case VI_NOTIFY_TAG_ISPBUF_FE:
			ch = VI_NOTIFY_TAG_CHANNEL(msg.tag);
			if (ch >= 12)
				break;

			v = host1x_readl(pdev, VI_CH_CONTROL(ch));
			if (hvnd->ld_mask & (1 << ch)) {
				hvnd->ld_mask &= ~(1 << ch);
				v |= VI_CH_CONTROL_ENABLE;
				host1x_writel(pdev, VI_CH_CONTROL(ch), v);
			} else if (!(v & VI_CH_CONTROL_SINGLESHOT)) {
				v &= ~VI_CH_CONTROL_ENABLE;
				host1x_writel(pdev, VI_CH_CONTROL(ch), v);
			}
			break;

		default:
			break;
		}

		vi_notify_dev_recv(hvnd->vnd, &msg);
	}

	return IRQ_HANDLED;
}

static int nvhost_vi_get_irq(struct platform_device *pdev, unsigned num,
				irq_handler_t isr)
{
	int err, irq;

	irq = platform_get_irq(pdev, num);
	if (IS_ERR_VALUE(irq)) {
		dev_err(&pdev->dev, "missing IRQ\n");
		return irq;
	}

	err = devm_request_threaded_irq(&pdev->dev, irq, NULL, isr,
					IRQF_ONESHOT, dev_name(&pdev->dev),
					pdev);
	if (err) {
		dev_err(&pdev->dev, "cannot get IRQ %d\n", irq);
		return err;
	}

	disable_irq(irq);
	return irq;
}

/* VI Notify back-end */
static int nvhost_vi_notify_probe(struct device *dev,
					struct vi_notify_dev *vnd)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);
	struct nvhost_vi_notify_dev *hvnd = &vi->notify;

	hvnd->vnd = vnd;
	hvnd->mask = 0;
	hvnd->ld_mask = 0;

	if (vi->debug_dir != NULL) {
		debugfs_create_atomic_t("overflow", S_IRUGO, vi->debug_dir,
					&hvnd->overflow);
		debugfs_create_atomic_t("notify-overflow", S_IRUGO,
					vi->debug_dir, &hvnd->notify_overflow);
		debugfs_create_atomic_t("fmlite-overflow", S_IRUGO,
					vi->debug_dir, &hvnd->fmlite_overflow);
	}

	hvnd->error_irq = nvhost_vi_get_irq(pdev, 0, nvhost_vi_error_isr);
	if (IS_ERR_VALUE(hvnd->error_irq))
		return hvnd->error_irq;

	hvnd->prio_irq = nvhost_vi_get_irq(pdev, 1, nvhost_vi_prio_isr);
	if (IS_ERR_VALUE(hvnd->prio_irq))
		return hvnd->prio_irq;

	hvnd->norm_irq = nvhost_vi_get_irq(pdev, 2, nvhost_vi_notify_isr);
	if (IS_ERR_VALUE(hvnd->norm_irq))
		return hvnd->norm_irq;

	return 0;
}

static void nvhost_vi_notify_dump_classify(struct platform_device *pdev)
{
	u32 r;

#define DUMP_TAG(x, y) \
do { \
	r = host1x_readl(pdev, VI_NOTIFY_TAG_CLASSIFY_##x##_0); \
	dev_dbg(&pdev->dev, "Classify " y ": 0x%08X\n", r); \
} while (0)

	DUMP_TAG(NO_OUTPUT,	"no output");
	DUMP_TAG(HIGH,		"high prio");
	DUMP_TAG(SAFETY,	"safety   ");
	DUMP_TAG(SAFETY_ERROR,	"error    ");
	DUMP_TAG(SAFETY_TEST,	"test     ");

	nvhost_vi_notify_dump_status(pdev);
}

static int nvhost_vi_notify_classify(struct device *dev,
					u32 ign_mask, u32 pri_mask)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);
	struct nvhost_vi_notify_dev *hvnd = &vi->notify;
	int err;

	if (ign_mask != 0xffffffff)
		/* Unmask events handled by the interrupt handler */
		ign_mask &= ~((1u << VI_NOTIFY_TAG_CHANSEL_COLLISION)
			| (1u << VI_NOTIFY_TAG_CHANSEL_SHORT_FRAME)
			| (1u << VI_NOTIFY_TAG_CHANSEL_LOAD_FRAMED)
			| (1u << VI_NOTIFY_TAG_ATOMP_FE)
			| (1u << VI_NOTIFY_TAG_ISPBUF_FE));

	if (hvnd->mask == 0) {
		err = nvhost_module_busy(pdev);
		if (err) {
			WARN_ON(1);
			return err;
		}

		enable_irq(hvnd->error_irq);
		enable_irq(hvnd->prio_irq);
		enable_irq(hvnd->norm_irq);
		host1x_writel(pdev, VI_CFG_INTERRUPT_MASK_0,
				VI_HOST_PKTINJECT_STALL_ERR_MASK |
				VI_CSIMUX_FIFO_OVFL_ERR_MASK |
				VI_ATOMP_PACKER_OVFL_ERR_MASK |
				VI_FMLITE_BUF_OVFL_ERR_MASK |
				VI_NOTIFY_FIFO_OVFL_ERR_MASK |
				VI_ISPBUFA_ERR_MASK);
	}

	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_NO_OUTPUT_0, ign_mask);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_HIGH_0, pri_mask);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_0, 0);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_TEST_0, 0);
	host1x_writel(pdev, VI_NOTIFY_OCCUPANCY_URGENT_0, 512);
	nvhost_vi_notify_dump_classify(pdev);

	hvnd->mask = ~ign_mask;

	if (hvnd->mask == 0) {
		host1x_writel(pdev, VI_CFG_INTERRUPT_MASK_0, 0);
		disable_irq(hvnd->norm_irq);
		disable_irq(hvnd->prio_irq);
		disable_irq(hvnd->error_irq);
		nvhost_module_idle(pdev);
	}

	return 0;
}

struct vi_notify_driver nvhost_vi_notify_driver = {
	.owner = THIS_MODULE,
	.probe = nvhost_vi_notify_probe,
	.classify = nvhost_vi_notify_classify,
};
