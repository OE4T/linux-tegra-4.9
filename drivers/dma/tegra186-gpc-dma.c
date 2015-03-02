/*
 * DMA driver for Nvidia's Tegra186 GPC DMA controller.
 *
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

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/clk/tegra.h>
#include <linux/tegra_pm_domains.h>

#include "dmaengine.h"

/* CSR register */
#define TEGRA_GPCDMA_CHAN_CSR			0x00
#define TEGRA_GPCDMA_CSR_ENB			BIT(31)
#define TEGRA_GPCDMA_CSR_IE_EOC			BIT(30)
#define TEGRA_GPCDMA_CSR_ONCE			BIT(27)
#define TEGRA_GPCDMA_CSR_FC_MODE_NO_MMIO	(0 << 24)
#define TEGRA_GPCDMA_CSR_FC_MODE_ONE_MMIO	(2 << 24)
#define TEGRA_GPCDMA_CSR_FC_MODE_TWO_MMIO	(3 << 24)
#define TEGRA_GPCDMA_CSR_DMA_IO2MEM_NO_FC	(0 << 21)
#define TEGRA_GPCDMA_CSR_DMA_IO2MEM_FC		(1 << 21)
#define TEGRA_GPCDMA_CSR_DMA_MEM2IO_NO_FC	(2 << 21)
#define TEGRA_GPCDMA_CSR_DMA_MEM2IO_FC		(3 << 21)
#define TEGRA_GPCDMA_CSR_DMA_MEM2MEM		(4 << 21)
#define TEGRA_GPCDMA_CSR_DMA_FIXED_PAT		(6 << 21)
#define TEGRA_GPCDMA_CSR_REQ_SEL_SHIFT		16
#define TEGRA_GPCDMA_CSR_IRQ_MASK		15
#define TEGRA_GPCDMA_CSR_WEIGHT_SHIFT		10

/* STATUS register */
#define TEGRA_GPCDMA_CHAN_STATUS		0x004
#define TEGRA_GPCDMA_STATUS_BUSY		BIT(31)
#define TEGRA_GPCDMA_STATUS_ISE_EOC		BIT(30)
#define TEGRA_GPCDMA_STATUS_PING_PONG		BIT(28)
#define TEGRA_GPCDMA_STATUS_DMA_ACTIVITY	BIT(27)
#define TEGRA_GPCDMA_STATUS_CHANNEL_PAUSE	BIT(26)
#define TEGRA_GPCDMA_STATUS_CHANNEL_RX		BIT(25)
#define TEGRA_GPCDMA_STATUS_CHANNEL_TX		BIT(24)
#define TEGRA_GPCDMA_STATUS_IRQ_INTR_STA	BIT(23)
#define TEGRA_GPCDMA_STATUS_IRQ_STA		BIT(21)
#define TEGRA_GPCDMA_STATUS_IRQ_TRIG_STA	BIT(20)

#define TEGRA_GPCDMA_CHAN_CSRE			0x008
#define TEGRA_GPCDMA_CHAN_CSRE_PAUSE		(1 << 31)

/* Source address */
#define TEGRA_GPCDMA_CHAN_SRC_PTR		0x00C

/* Destination address */
#define TEGRA_GPCDMA_CHAN_DST_PTR		0x010

/* High address pointer */
#define TEGRA_GPCDMA_HIGH_ADDR_PTR		0x014
#define TEGRA_GPCDMA_HIGH_ADDR_SCR_PTR_SHIFT	0
#define TEGRA_GPCDMA_HIGH_ADDR_DST_PTR_SHIFT	16

/* MC sequence register */
#define TEGRA_GPCDMA_CHAN_MCSEQ		0x18
#define TEGRA_GPCDMA_MCSEQ_DATA_SWAP		BIT(31)
#define TEGRA_GPCDMA_MCSEQ_REQ_COUNT_SHIFT	25
#define TEGRA_GPCDMA_MCSEQ_BURST_4		(1 << 23)
#define TEGRA_GPCDMA_MCSEQ_BURST_8		(2 << 23)
#define TEGRA_GPCDMA_MCSEQ_BURST_16		(3 << 23)
#define TEGRA_GPCDMA_MCSEQ_WRAP1_SHIFT		20
#define TEGRA_GPCDMA_MCSEQ_WRAP0_SHIFT		17
#define TEGRA_GPCDMA_MCSEQ_WRAP_NONE		0
#define TEGRA_GPCDMA_MCSEQ_MC_PROT_SHIFT	14
#define TEGRA_GPCDMA_MCSEQ_STREAM_ID1_SHIFT	7
#define TEGRA_GPCDMA_MCSEQ_STREAM_ID0_SHIFT	0


/* MMIO sequence register */
#define TEGRA_GPCDMA_CHAN_MMIOSEQ		0x01c
#define TEGRA_GPCDMA_MMIOSEQ_DBL_BUF		BIT(31)
#define TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_8	(0 << 28)
#define TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_16	(1 << 28)
#define TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_32	(2 << 28)
#define TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_64	(3 << 28)
#define TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_128	(4 << 28)
#define TEGRA_GPCDMA_MMIOSEQ_DATA_SWAP		BIT(27)
/* Burst sizes from 1 word to 15 words are supported */
#define TEGRA_GPCDMA_MMIOSEQ_BURST_SHIFT	23
#define TEGRA_GPCDMA_MMIOSEQ_MASTER_ID_SHIFT	19
#define TEGRA_GPCDMA_MMIOSEQ_WRAP_WORD_SHIFT	16
#define TEGRA_GPCDMA_MMIOSEQ_MMIO_PROT_SHIFT	7

/* Channel BCOUNT */
#define TEGRA_GPCDMA_CHAN_BCOUNT		0x20

/* Transfer count */
#define TEGRA_GPCDMA_CHAN_XFER_COUNT		0x24

/* DMA byte count status */
#define TEGRA_GPCDMA_CHAN_DMA_BYTE_STATUS	0x28

/* Error Status Register */
#define TEGRA_GPCDMA_CHAN_ERR_STATUS		0x30

/* Fixed Pattern */
#define TEGRA_GPCDMA_CHAN_FIXED_PATTERN		0x34

#define TEGRA_GPCDMA_CHAN_TZ			0x38
#define TEGRA_GPCDMA_CHAN_TZ_MMIO_PROT_1	BIT(0)
#define TEGRA_GPCDMA_CHAN_TZ_MC_PROT_1		BIT(1)

#define TEGRA_GPCDMA_CHAN_SPARE			0x3c
#define TEGRA_GPCDMA_CHAN_SPARE_EN_LEGACY_FC	BIT(16)

/*
 * If any burst is in flight and DMA paused then this is the time to complete
 * on-flight burst and update DMA status register.
 */
#define TEGRA_GPCDMA_BURST_COMPLETE_TIME	20

/* Channel base address offset from GPCDMA base address */
#define TEGRA_GPCDMA_CHANNEL_BASE_ADD_OFFSET	0x10000

struct tegra_dma;

/*
 * tegra_dma_chip_data Tegra chip specific DMA data
 * @nr_channels: Number of channels available in the controller.
 * @channel_reg_size: Channel register size.
 * @max_dma_count: Maximum DMA transfer count supported by DMA controller.
 */
struct tegra_dma_chip_data {
	int nr_channels;
	int channel_reg_size;
	int max_dma_count;
};

/* DMA channel registers */
struct tegra_dma_channel_regs {
	unsigned long	csr;
	unsigned long	src_ptr;
	unsigned long	dst_ptr;
	unsigned long	mc_seq;
	unsigned long	mmio_seq;
	unsigned long	bcount;
};

/*
 * tegra_dma_sg_req: Dma request details to configure hardware. This
 * contains the details for one transfer to configure DMA hw.
 * The client's request for data transfer can be broken into multiple
 * sub-transfer as per requester details and hw support.
 * This sub transfer get added in the list of transfer and point to Tegra
 * DMA descriptor which manages the transfer details.
 */
struct tegra_dma_sg_req {
	struct tegra_dma_channel_regs	ch_regs;
	int				req_len;
	bool				configured;
	bool				skipped;
	bool				last_sg;
	bool				half_done;
	struct list_head		node;
	struct tegra_dma_desc		*dma_desc;
};

/*
 * tegra_dma_desc: Tegra DMA descriptors which manages the client requests.
 * This descriptor keep track of transfer status, callbacks and request
 * counts etc.
 */
struct tegra_dma_desc {
	struct dma_async_tx_descriptor	txd;
	int				bytes_requested;
	int				bytes_transferred;
	enum dma_status			dma_status;
	struct list_head		node;
	struct list_head		tx_list;
	struct list_head		cb_node;
	int				cb_count;
};

struct tegra_dma_channel;

typedef void (*dma_isr_handler)(struct tegra_dma_channel *tdc,
				bool to_terminate);

/* tegra_dma_channel: Channel specific information */
struct tegra_dma_channel {
	struct dma_chan		dma_chan;
	char			name[30];
	bool			config_init;
	int			id;
	int			irq;
	unsigned long		chan_base_offset;
	spinlock_t		lock;
	bool			busy;
	struct tegra_dma	*tdma;

	/* Different lists for managing the requests */
	struct list_head	free_sg_req;
	struct list_head	pending_sg_req;
	struct list_head	free_dma_desc;
	struct list_head	cb_desc;

	/* ISR handler and tasklet for bottom half of isr handling */
	dma_isr_handler		isr_handler;
	struct tasklet_struct	tasklet;
	dma_async_tx_callback	callback;
	void			*callback_param;

	/* Channel-slave specific configuration */
	unsigned int slave_id;
	struct dma_slave_config dma_sconfig;
	struct tegra_dma_channel_regs	channel_reg;
};

/* tegra_dma: Tegra DMA specific information */
struct tegra_dma {
	struct dma_device		dma_dev;
	struct device			*dev;
	struct clk			*dma_clk;
	spinlock_t			global_lock;
	void __iomem			*base_addr;
	const struct tegra_dma_chip_data *chip_data;

	/* Last member of the structure */
	struct tegra_dma_channel channels[0];
};

static inline void tdma_write(struct tegra_dma *tdma, u32 reg, u32 val)
{
	writel(val, tdma->base_addr + reg);
}

static inline u32 tdma_read(struct tegra_dma *tdma, u32 reg)
{
	return readl(tdma->base_addr + reg);
}

static inline void tdc_write(struct tegra_dma_channel *tdc,
		u32 reg, u32 val)
{
	writel(val, tdc->tdma->base_addr + tdc->chan_base_offset + reg);
}

static inline u32 tdc_read(struct tegra_dma_channel *tdc, u32 reg)
{
	return readl(tdc->tdma->base_addr + tdc->chan_base_offset + reg);
}

static inline struct tegra_dma_channel *to_tegra_dma_chan(struct dma_chan *dc)
{
	return container_of(dc, struct tegra_dma_channel, dma_chan);
}

static inline struct tegra_dma_desc *txd_to_tegra_dma_desc(
		struct dma_async_tx_descriptor *td)
{
	return container_of(td, struct tegra_dma_desc, txd);
}

static inline struct device *tdc2dev(struct tegra_dma_channel *tdc)
{
	return &tdc->dma_chan.dev->device;
}

static dma_cookie_t tegra_dma_tx_submit(struct dma_async_tx_descriptor *tx);
#ifdef CONFIG_PM_RUNTIME
static int tegra_dma_runtime_suspend(struct device *dev);
static int tegra_dma_runtime_resume(struct device *dev);
#endif

/* Get DMA desc from free list, if not there then allocate it.  */
static struct tegra_dma_desc *tegra_dma_desc_get(
		struct tegra_dma_channel *tdc)
{
	struct tegra_dma_desc *dma_desc;
	unsigned long flags;

	spin_lock_irqsave(&tdc->lock, flags);

	/* Do not allocate if desc are waiting for ack */
	list_for_each_entry(dma_desc, &tdc->free_dma_desc, node) {
		if (async_tx_test_ack(&dma_desc->txd)) {
			list_del(&dma_desc->node);
			spin_unlock_irqrestore(&tdc->lock, flags);
			dma_desc->txd.flags = 0;
			return dma_desc;
		}
	}

	spin_unlock_irqrestore(&tdc->lock, flags);

	/* Allocate DMA desc */
	dma_desc = kzalloc(sizeof(*dma_desc), GFP_ATOMIC);
	if (!dma_desc) {
		dev_err(tdc2dev(tdc), "dma_desc alloc failed\n");
		return NULL;
	}

	dma_async_tx_descriptor_init(&dma_desc->txd, &tdc->dma_chan);
	dma_desc->txd.tx_submit = tegra_dma_tx_submit;
	dma_desc->txd.flags = 0;
	return dma_desc;
}

static void tegra_dma_desc_put(struct tegra_dma_channel *tdc,
		struct tegra_dma_desc *dma_desc)
{
	unsigned long flags;

	spin_lock_irqsave(&tdc->lock, flags);
	if (!list_empty(&dma_desc->tx_list))
		list_splice_init(&dma_desc->tx_list, &tdc->free_sg_req);
	list_add_tail(&dma_desc->node, &tdc->free_dma_desc);
	spin_unlock_irqrestore(&tdc->lock, flags);
}

static struct tegra_dma_sg_req *tegra_dma_sg_req_get(
		struct tegra_dma_channel *tdc)
{
	struct tegra_dma_sg_req *sg_req = NULL;
	unsigned long flags;

	spin_lock_irqsave(&tdc->lock, flags);
	if (!list_empty(&tdc->free_sg_req)) {
		sg_req = list_first_entry(&tdc->free_sg_req,
					typeof(*sg_req), node);
		list_del(&sg_req->node);
		spin_unlock_irqrestore(&tdc->lock, flags);
		return sg_req;
	}
	spin_unlock_irqrestore(&tdc->lock, flags);

	sg_req = kzalloc(sizeof(struct tegra_dma_sg_req), GFP_ATOMIC);
	if (!sg_req)
		dev_err(tdc2dev(tdc), "sg_req alloc failed\n");
	return sg_req;
}

static int tegra_dma_slave_config(struct dma_chan *dc,
		struct dma_slave_config *sconfig)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);

	if (!list_empty(&tdc->pending_sg_req)) {
		dev_err(tdc2dev(tdc), "Configuration not allowed\n");
		return -EBUSY;
	}

	memcpy(&tdc->dma_sconfig, sconfig, sizeof(*sconfig));
	if (!tdc->slave_id)
		tdc->slave_id = sconfig->slave_id;
	tdc->config_init = true;
	return 0;
}

static void tegra_dma_pause(struct tegra_dma_channel *tdc,
	bool wait_for_burst_complete)
{
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSRE,
		TEGRA_GPCDMA_CHAN_CSRE_PAUSE);
	if (wait_for_burst_complete)
		udelay(TEGRA_GPCDMA_BURST_COMPLETE_TIME);
}

static void tegra_dma_resume(struct tegra_dma_channel *tdc)
{
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSRE, 0);
}

static void tegra_dma_stop(struct tegra_dma_channel *tdc)
{
	u32 csr;
	u32 status;

	/* Disable interrupts */
	csr = tdc_read(tdc, TEGRA_GPCDMA_CHAN_CSR);
	csr &= ~TEGRA_GPCDMA_CSR_IE_EOC;
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSR, csr);

	/* Disable DMA */
	csr &= ~TEGRA_GPCDMA_CSR_ENB;
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSR, csr);

	/* Clear interrupt status if it is there */
	status = tdc_read(tdc, TEGRA_GPCDMA_CHAN_STATUS);
	if (status & TEGRA_GPCDMA_STATUS_ISE_EOC) {
		dev_dbg(tdc2dev(tdc), "%s():clearing interrupt\n", __func__);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_STATUS, status);
	}
	tdc->busy = false;
}

static void tegra_dma_start(struct tegra_dma_channel *tdc,
		struct tegra_dma_sg_req *sg_req)
{
	struct tegra_dma_channel_regs *ch_regs = &sg_req->ch_regs;

	tdc_write(tdc, TEGRA_GPCDMA_CHAN_BCOUNT, ch_regs->bcount);

	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSR, 0);
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSR, ch_regs->csr);
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_MMIOSEQ, ch_regs->mmio_seq);
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_MCSEQ, ch_regs->mc_seq);
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_SRC_PTR, ch_regs->src_ptr);
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_DST_PTR, ch_regs->dst_ptr);

	/* Start DMA */
	tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSR,
				ch_regs->csr | TEGRA_GPCDMA_CSR_ENB);
}

static void tdc_start_head_req(struct tegra_dma_channel *tdc)
{
	struct tegra_dma_sg_req *sg_req;

	if (list_empty(&tdc->pending_sg_req))
		return;

	sg_req = list_first_entry(&tdc->pending_sg_req,
					typeof(*sg_req), node);
	tegra_dma_start(tdc, sg_req);
	sg_req->configured = true;
	sg_req->skipped = false;
	tdc->busy = true;
}

static void tegra_dma_abort_all(struct tegra_dma_channel *tdc)
{
	struct tegra_dma_sg_req *sgreq;
	struct tegra_dma_desc *dma_desc;

	while (!list_empty(&tdc->pending_sg_req)) {
		sgreq = list_first_entry(&tdc->pending_sg_req,
						typeof(*sgreq), node);
		list_move_tail(&sgreq->node, &tdc->free_sg_req);
		if (sgreq->last_sg) {
			dma_desc = sgreq->dma_desc;
			dma_desc->dma_status = DMA_ERROR;
			list_add_tail(&dma_desc->node, &tdc->free_dma_desc);

			/* Add in cb list if it is not there. */
			if (!dma_desc->cb_count)
				list_add_tail(&dma_desc->cb_node,
							&tdc->cb_desc);
			dma_desc->cb_count++;
		}
	}
	tdc->isr_handler = NULL;
}

static void handle_once_dma_done(struct tegra_dma_channel *tdc,
	bool to_terminate)
{
	struct tegra_dma_sg_req *sgreq;
	struct tegra_dma_desc *dma_desc;

	tdc->busy = false;
	sgreq = list_first_entry(&tdc->pending_sg_req, typeof(*sgreq), node);
	dma_desc = sgreq->dma_desc;
	dma_desc->bytes_transferred += sgreq->req_len;

	list_del(&sgreq->node);
	if (sgreq->last_sg) {
		dma_desc->dma_status = DMA_SUCCESS;
		dma_cookie_complete(&dma_desc->txd);
		if (!dma_desc->cb_count)
			list_add_tail(&dma_desc->cb_node, &tdc->cb_desc);
		dma_desc->cb_count++;
		list_add_tail(&dma_desc->node, &tdc->free_dma_desc);
	}
	list_add_tail(&sgreq->node, &tdc->free_sg_req);

	/* Do not start DMA if it is going to be terminate */
	if (list_empty(&tdc->pending_sg_req) && (!to_terminate)) {
		clk_disable(tdc->tdma->dma_clk);
		pm_runtime_put(tdc->tdma->dev);
	}

	if (to_terminate || list_empty(&tdc->pending_sg_req))
		return;

	tdc_start_head_req(tdc);
	return;
}

static void tegra_dma_tasklet(unsigned long data)
{
	struct tegra_dma_channel *tdc = (struct tegra_dma_channel *)data;
	dma_async_tx_callback callback = NULL;
	void *callback_param = NULL;
	struct tegra_dma_desc *dma_desc;
	unsigned long flags;
	int cb_count;

	spin_lock_irqsave(&tdc->lock, flags);
	while (!list_empty(&tdc->cb_desc)) {
		dma_desc  = list_first_entry(&tdc->cb_desc,
					typeof(*dma_desc), cb_node);
		list_del(&dma_desc->cb_node);
		callback = dma_desc->txd.callback;
		callback_param = dma_desc->txd.callback_param;
		cb_count = dma_desc->cb_count;
		dma_desc->cb_count = 0;
		spin_unlock_irqrestore(&tdc->lock, flags);
		while (cb_count-- && callback)
			callback(callback_param);
		spin_lock_irqsave(&tdc->lock, flags);
	}
	spin_unlock_irqrestore(&tdc->lock, flags);
}

static irqreturn_t tegra_dma_isr(int irq, void *dev_id)
{
	struct tegra_dma_channel *tdc = dev_id;
	unsigned long status;
	unsigned long flags;

	spin_lock_irqsave(&tdc->lock, flags);

	status = tdc_read(tdc, TEGRA_GPCDMA_CHAN_STATUS);
	if (status & TEGRA_GPCDMA_STATUS_ISE_EOC) {
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_STATUS, status);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_STATUS, TEGRA_GPCDMA_STATUS_ISE_EOC);
		tdc->isr_handler(tdc, false);
		tasklet_schedule(&tdc->tasklet);
		spin_unlock_irqrestore(&tdc->lock, flags);
		return IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&tdc->lock, flags);
	dev_info(tdc2dev(tdc),
		"Interrupt already served status 0x%08lx\n", status);
	return IRQ_NONE;
}

static dma_cookie_t tegra_dma_tx_submit(struct dma_async_tx_descriptor *txd)
{
	struct tegra_dma_desc *dma_desc = txd_to_tegra_dma_desc(txd);
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(txd->chan);
	unsigned long flags;
	dma_cookie_t cookie;

	spin_lock_irqsave(&tdc->lock, flags);
	dma_desc->dma_status = DMA_IN_PROGRESS;
	cookie = dma_cookie_assign(&dma_desc->txd);
	list_splice_tail_init(&dma_desc->tx_list, &tdc->pending_sg_req);
	spin_unlock_irqrestore(&tdc->lock, flags);
	return cookie;
}

static void tegra_dma_issue_pending(struct dma_chan *dc)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&tdc->lock, flags);
	if (list_empty(&tdc->pending_sg_req)) {
		dev_err(tdc2dev(tdc), "No DMA request\n");
		goto end;
	}

	pm_runtime_get(tdc->tdma->dev);
	ret = clk_enable(tdc->tdma->dma_clk);
	if (ret < 0) {
		dev_err(tdc2dev(tdc), "clk_enable failed: %d\n", ret);
		return;
	}

	if (!tdc->busy)
		tdc_start_head_req(tdc);
end:
	spin_unlock_irqrestore(&tdc->lock, flags);
	return;
}

static void tegra_dma_terminate_all(struct dma_chan *dc)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);
	struct tegra_dma_sg_req *sgreq;
	struct tegra_dma_desc *dma_desc;
	unsigned long flags;
	unsigned long status;
	unsigned long bcount = 0;
	bool was_busy;

	spin_lock_irqsave(&tdc->lock, flags);
	if (list_empty(&tdc->pending_sg_req)) {
		spin_unlock_irqrestore(&tdc->lock, flags);
		return;
	}

	if (!tdc->busy)
		goto skip_dma_stop;

	/* Pause DMA before checking the queue status */
	tegra_dma_pause(tdc, true);

	status = tdc_read(tdc, TEGRA_GPCDMA_CHAN_STATUS);
	bcount = tdc_read(tdc, TEGRA_GPCDMA_CHAN_XFER_COUNT);
	if (status & TEGRA_GPCDMA_STATUS_ISE_EOC) {
		dev_dbg(tdc2dev(tdc), "%s():handling isr\n", __func__);
		tdc->isr_handler(tdc, true);
		status = tdc_read(tdc, TEGRA_GPCDMA_CHAN_STATUS);
		bcount = tdc_read(tdc, TEGRA_GPCDMA_CHAN_XFER_COUNT);
	}

	was_busy = tdc->busy;
	tegra_dma_stop(tdc);

	if (!list_empty(&tdc->pending_sg_req) && was_busy) {
		sgreq = list_first_entry(&tdc->pending_sg_req,
					typeof(*sgreq), node);
		sgreq->dma_desc->bytes_transferred += sgreq->req_len - status;
	}
	tegra_dma_resume(tdc);
	clk_disable(tdc->tdma->dma_clk);
	pm_runtime_put(tdc->tdma->dev);

skip_dma_stop:
	tegra_dma_abort_all(tdc);

	while (!list_empty(&tdc->cb_desc)) {
		dma_desc  = list_first_entry(&tdc->cb_desc,
					typeof(*dma_desc), cb_node);
		list_del(&dma_desc->cb_node);
		dma_desc->cb_count = 0;
	}
	spin_unlock_irqrestore(&tdc->lock, flags);
}

static enum dma_status tegra_dma_tx_status(struct dma_chan *dc,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);
	struct tegra_dma_desc *dma_desc;
	struct tegra_dma_sg_req *sg_req;
	enum dma_status ret;
	unsigned long flags;
	unsigned int residual;

	spin_lock_irqsave(&tdc->lock, flags);

	ret = dma_cookie_status(dc, cookie, txstate);
	if (ret == DMA_SUCCESS) {
		spin_unlock_irqrestore(&tdc->lock, flags);
		return ret;
	}

	/* Check on wait_ack desc status */
	list_for_each_entry(dma_desc, &tdc->free_dma_desc, node) {
		if (dma_desc->txd.cookie == cookie) {
			residual =  dma_desc->bytes_requested -
					(dma_desc->bytes_transferred %
						dma_desc->bytes_requested);
			dma_set_residue(txstate, residual);
			ret = dma_desc->dma_status;
			spin_unlock_irqrestore(&tdc->lock, flags);
			return ret;
		}
	}

	/* Check in pending list */
	list_for_each_entry(sg_req, &tdc->pending_sg_req, node) {
		dma_desc = sg_req->dma_desc;
		if (dma_desc->txd.cookie == cookie) {
			residual =  dma_desc->bytes_requested -
					(dma_desc->bytes_transferred %
						dma_desc->bytes_requested);
			dma_set_residue(txstate, residual);
			ret = dma_desc->dma_status;
			spin_unlock_irqrestore(&tdc->lock, flags);
			return ret;
		}
	}

	dev_dbg(tdc2dev(tdc), "cookie %d does not found\n", cookie);
	spin_unlock_irqrestore(&tdc->lock, flags);
	return ret;
}

static int tegra_dma_device_control(struct dma_chan *dc, enum dma_ctrl_cmd cmd,
			unsigned long arg)
{
	switch (cmd) {
	case DMA_SLAVE_CONFIG:
		return tegra_dma_slave_config(dc,
				(struct dma_slave_config *)arg);

	case DMA_TERMINATE_ALL:
		tegra_dma_terminate_all(dc);
		return 0;

	default:
		break;
	}

	return -ENXIO;
}

static inline int get_bus_width(struct tegra_dma_channel *tdc,
		enum dma_slave_buswidth slave_bw)
{
	switch (slave_bw) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_8;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_16;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		return TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_32;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		return TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_64;
	default:
		dev_warn(tdc2dev(tdc),
			"slave bw is not supported, using 32bits\n");
		return TEGRA_GPCDMA_MMIOSEQ_BUS_WIDTH_32;
	}
}

static inline int get_burst_size(struct tegra_dma_channel *tdc,
	u32 burst_size, enum dma_slave_buswidth slave_bw, int len)
{
	int burst_byte;
	int burst_mc_width;

	/*
	 * burst_size from client is in terms of the bus_width.
	 * convert that into words.
	 */
	burst_byte = burst_size * slave_bw;
	burst_mc_width = burst_byte / 4;

	/* If burst size is 0 then calculate the burst size based on length */
	if (!burst_mc_width) {
		if (len & 0xF)
			return TEGRA_GPCDMA_MCSEQ_BURST_4;
		else if ((len >> 4) & 0x1)
			return TEGRA_GPCDMA_MCSEQ_BURST_8;
		else
			return TEGRA_GPCDMA_MCSEQ_BURST_16;
	}
	if (burst_mc_width < 4)
		return TEGRA_GPCDMA_MCSEQ_BURST_4;
	else if (burst_mc_width < 8)
		return TEGRA_GPCDMA_MCSEQ_BURST_8;
	else
		return TEGRA_GPCDMA_MCSEQ_BURST_16;
}

static int get_transfer_param(struct tegra_dma_channel *tdc,
	enum dma_transfer_direction direction, unsigned long *apb_addr,
	unsigned long *mmio_seq, unsigned long *csr, unsigned int *burst_size,
	enum dma_slave_buswidth *slave_bw)
{

	switch (direction) {
	case DMA_MEM_TO_DEV:
		*apb_addr = tdc->dma_sconfig.dst_addr;
		*mmio_seq = get_bus_width(tdc, tdc->dma_sconfig.dst_addr_width);
		*burst_size = tdc->dma_sconfig.dst_maxburst;
		*slave_bw = tdc->dma_sconfig.dst_addr_width;
		*csr = TEGRA_GPCDMA_CSR_DMA_MEM2IO_FC;
		return 0;

	case DMA_DEV_TO_MEM:
		*apb_addr = tdc->dma_sconfig.src_addr;
		*mmio_seq = get_bus_width(tdc, tdc->dma_sconfig.src_addr_width);
		*burst_size = tdc->dma_sconfig.src_maxburst;
		*slave_bw = tdc->dma_sconfig.src_addr_width;
		*csr = TEGRA_GPCDMA_CSR_DMA_IO2MEM_FC;
		return 0;
	default:
		dev_err(tdc2dev(tdc), "Dma direction is not supported\n");
		return -EINVAL;
	}
	return -EINVAL;
}

static struct dma_async_tx_descriptor *tegra_dma_prep_slave_sg(
	struct dma_chan *dc, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags,
	void *context)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);
	struct tegra_dma_desc *dma_desc;
	unsigned int i;
	struct scatterlist *sg;
	unsigned long csr, mc_seq, apb_ptr, mmio_seq;
	struct list_head req_list;
	struct tegra_dma_sg_req *sg_req = NULL;
	u32 burst_size;
	enum dma_slave_buswidth slave_bw;
	int ret;

	if (!tdc->config_init) {
		dev_err(tdc2dev(tdc), "dma channel is not configured\n");
		return NULL;
	}
	if (sg_len < 1) {
		dev_err(tdc2dev(tdc), "Invalid segment length %d\n", sg_len);
		return NULL;
	}

	ret = get_transfer_param(tdc, direction, &apb_ptr, &mmio_seq, &csr,
				&burst_size, &slave_bw);
	if (ret < 0)
		return NULL;

	INIT_LIST_HEAD(&req_list);

	csr |= TEGRA_GPCDMA_CSR_ONCE;
	csr |= tdc->slave_id << TEGRA_GPCDMA_CSR_REQ_SEL_SHIFT;
	if (flags & DMA_PREP_INTERRUPT)
		csr |= TEGRA_GPCDMA_CSR_IE_EOC;

	mc_seq = TEGRA_GPCDMA_MCSEQ_WRAP_NONE <<
			TEGRA_GPCDMA_MCSEQ_WRAP0_SHIFT;
	mc_seq |= TEGRA_GPCDMA_MCSEQ_WRAP_NONE <<
			TEGRA_GPCDMA_MCSEQ_WRAP1_SHIFT;
	mmio_seq |= (1 << TEGRA_GPCDMA_MMIOSEQ_WRAP_WORD_SHIFT);

	dma_desc = tegra_dma_desc_get(tdc);
	if (!dma_desc) {
		dev_err(tdc2dev(tdc), "Dma descriptors not available\n");
		return NULL;
	}
	INIT_LIST_HEAD(&dma_desc->tx_list);
	INIT_LIST_HEAD(&dma_desc->cb_node);
	dma_desc->cb_count = 0;
	dma_desc->bytes_requested = 0;
	dma_desc->bytes_transferred = 0;
	dma_desc->dma_status = DMA_IN_PROGRESS;

	/* Make transfer requests */
	for_each_sg(sgl, sg, sg_len, i) {
		u32 len, mem;

		mem = sg_dma_address(sg);
		len = sg_dma_len(sg);

		if ((len & 3) || (mem & 3) ||
				(len > tdc->tdma->chip_data->max_dma_count)) {
			dev_err(tdc2dev(tdc),
				"Dma length/memory address is not supported\n");
			tegra_dma_desc_put(tdc, dma_desc);
			return NULL;
		}

		sg_req = tegra_dma_sg_req_get(tdc);
		if (!sg_req) {
			dev_err(tdc2dev(tdc), "Dma sg-req not available\n");
			tegra_dma_desc_put(tdc, dma_desc);
			return NULL;
		}

		mc_seq |= get_burst_size(tdc, burst_size, slave_bw, len);
		dma_desc->bytes_requested += len;

		if (direction == DMA_MEM_TO_DEV) {
			sg_req->ch_regs.src_ptr = mem;
			sg_req->ch_regs.dst_ptr = apb_ptr;
		} else if (direction == DMA_DEV_TO_MEM) {
			sg_req->ch_regs.src_ptr = apb_ptr;
			sg_req->ch_regs.dst_ptr = mem;
		}
		sg_req->ch_regs.bcount = len;
		sg_req->ch_regs.csr = csr;
		sg_req->ch_regs.mmio_seq = mmio_seq;
		sg_req->ch_regs.mc_seq = mc_seq;
		sg_req->configured = false;
		sg_req->skipped = false;
		sg_req->last_sg = false;
		sg_req->dma_desc = dma_desc;
		sg_req->req_len = len;

		list_add_tail(&sg_req->node, &dma_desc->tx_list);
	}
	sg_req->last_sg = true;
	if (flags & DMA_CTRL_ACK)
		dma_desc->txd.flags = DMA_CTRL_ACK;

	/*
	 * Make sure that mode should not be conflicting with currently
	 * configured mode.
	 */
	if (!tdc->isr_handler)
		tdc->isr_handler = handle_once_dma_done;

	return &dma_desc->txd;
}

static int tegra_dma_alloc_chan_resources(struct dma_chan *dc)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);

	clk_prepare(tdc->tdma->dma_clk);
	dma_cookie_init(&tdc->dma_chan);
	tdc->config_init = false;
	return 0;
}

static void tegra_dma_free_chan_resources(struct dma_chan *dc)
{
	struct tegra_dma_channel *tdc = to_tegra_dma_chan(dc);
	struct tegra_dma_desc *dma_desc;
	struct tegra_dma_sg_req *sg_req;
	struct list_head dma_desc_list;
	struct list_head sg_req_list;
	unsigned long flags;

	INIT_LIST_HEAD(&dma_desc_list);
	INIT_LIST_HEAD(&sg_req_list);

	dev_dbg(tdc2dev(tdc), "Freeing channel %d\n", tdc->id);

	if (tdc->busy)
		tegra_dma_terminate_all(dc);
	clk_unprepare(tdc->tdma->dma_clk);
	spin_lock_irqsave(&tdc->lock, flags);
	list_splice_init(&tdc->pending_sg_req, &sg_req_list);
	list_splice_init(&tdc->free_sg_req, &sg_req_list);
	list_splice_init(&tdc->free_dma_desc, &dma_desc_list);
	INIT_LIST_HEAD(&tdc->cb_desc);
	tdc->config_init = false;
	tdc->isr_handler = NULL;
	spin_unlock_irqrestore(&tdc->lock, flags);

	while (!list_empty(&dma_desc_list)) {
		dma_desc = list_first_entry(&dma_desc_list,
					typeof(*dma_desc), node);
		list_del(&dma_desc->node);
		kfree(dma_desc);
	}

	while (!list_empty(&sg_req_list)) {
		sg_req = list_first_entry(&sg_req_list, typeof(*sg_req), node);
		list_del(&sg_req->node);
		kfree(sg_req);
	}
	tdc->slave_id = 0;
}

static struct dma_chan *tegra_dma_of_xlate(struct of_phandle_args *dma_spec,
					   struct of_dma *ofdma)
{
	struct tegra_dma *tdma = ofdma->of_dma_data;
	struct dma_chan *chan;
	struct tegra_dma_channel *tdc;

	chan = dma_get_any_slave_channel(&tdma->dma_dev);
	if (!chan)
		return NULL;

	tdc = to_tegra_dma_chan(chan);
	tdc->slave_id = dma_spec->args[0];

	return chan;
}

#if defined(CONFIG_OF)
static const struct tegra_dma_chip_data tegra186_dma_chip_data = {
	.nr_channels = 32,
	.channel_reg_size = 0x10000,
	.max_dma_count = 1024UL * 64,
};

static const struct of_device_id tegra_dma_of_match[] = {
	{
		.compatible = "nvidia,tegra186-gpcdma",
		.data = &tegra186_dma_chip_data,
	}, {
	},
};
MODULE_DEVICE_TABLE(of, tegra_dma_of_match);
#endif

static struct platform_device_id tegra_dma_devtype[] = {
	{
		.name = "tegra186-gpcdma",
		.driver_data = (unsigned long)&tegra186_dma_chip_data,
	},
};

static struct device *dma_device;

static int tegra_dma_probe(struct platform_device *pdev)
{
	struct resource	*res;
	struct tegra_dma *tdma;
	int ret;
	int i;
	const struct tegra_dma_chip_data *cdata = NULL;
	struct tegra_dma_chip_data *chip_data = NULL;
	int start_chan_idx;
	int nr_chans;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_device(of_match_ptr(tegra_dma_of_match),
					&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Error: No device match found\n");
			return -ENODEV;
		}
		cdata = match->data;

		ret = of_property_read_u32(pdev->dev.of_node, "dma-channels",
							&nr_chans);
		if (!ret) {
			/* Allocate chip data and update number of channels */
			chip_data =
				devm_kzalloc(&pdev->dev,
					sizeof(struct tegra_dma_chip_data),
								GFP_KERNEL);
			if(!chip_data) {
				dev_err(&pdev->dev, "Error: memory allocation failed\n");
				return -ENOMEM;
			}
			memcpy(chip_data, cdata,
				sizeof(struct tegra_dma_chip_data));
			chip_data->nr_channels = nr_chans;
			cdata = chip_data;
		}
		ret = of_property_read_u32(pdev->dev.of_node,
				"nvidia,start-dma-channel-index",
						&start_chan_idx);
		if (ret)
			start_chan_idx = 0;
	} else {
		/* If no device tree then fallback to tegra186 data */
		cdata = (struct tegra_dma_chip_data *)pdev->id_entry->driver_data;
	}

	tdma = devm_kzalloc(&pdev->dev, sizeof(*tdma) + cdata->nr_channels *
			sizeof(struct tegra_dma_channel), GFP_KERNEL);
	if (!tdma) {
		dev_err(&pdev->dev, "Error: memory allocation failed\n");
		return -ENOMEM;
	}

	tdma->dev = &pdev->dev;
	tdma->chip_data = cdata;
	platform_set_drvdata(pdev, tdma);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No mem resource for DMA\n");
		return -EINVAL;
	}

	tdma->base_addr = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(tdma->base_addr))
		return PTR_ERR(tdma->base_addr);

	tdma->dma_clk = devm_clk_get(&pdev->dev, "gpcdma");
	if (IS_ERR(tdma->dma_clk)) {
		dev_err(&pdev->dev, "Error: Missing controller clock\n");
		return PTR_ERR(tdma->dma_clk);
	}

	spin_lock_init(&tdma->global_lock);

	dma_device = &pdev->dev;

#ifdef CONFIG_PM_RUNTIME
	tegra_pd_add_device(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra_dma_runtime_resume(&pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "dma_runtime_resume failed %d\n",
				ret);
			goto err_pm_disable;
		}
	}
#endif
	/* Enable clock before accessing registers */
	ret = clk_prepare_enable(tdma->dma_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "clk_prepare_enable failed: %d\n", ret);
		goto err_pm_disable;
	}

	/* Reset DMA controller */
	tegra_periph_reset_assert(tdma->dma_clk);
	udelay(2);
	tegra_periph_reset_deassert(tdma->dma_clk);

	INIT_LIST_HEAD(&tdma->dma_dev.channels);
	for (i = 0; i < cdata->nr_channels; i++) {
		struct tegra_dma_channel *tdc = &tdma->channels[i];

		tdc->chan_base_offset = TEGRA_GPCDMA_CHANNEL_BASE_ADD_OFFSET +
				start_chan_idx * cdata->channel_reg_size +
					i * cdata->channel_reg_size;
		res = platform_get_resource(pdev, IORESOURCE_IRQ,
							start_chan_idx + i);
		if (!res) {
			ret = -EINVAL;
			dev_err(&pdev->dev, "No irq resource for chan %d\n", i);
			goto err_irq;
		}
		tdc->irq = res->start;
		snprintf(tdc->name, sizeof(tdc->name), "gpcdma.%d", i);
		ret = devm_request_irq(&pdev->dev, tdc->irq,
				tegra_dma_isr, 0, tdc->name, tdc);
		if (ret) {
			dev_err(&pdev->dev,
				"request_irq failed with err %d channel %d\n",
				i, ret);
			goto err_irq;
		}

		tdc->dma_chan.device = &tdma->dma_dev;
		dma_cookie_init(&tdc->dma_chan);
		list_add_tail(&tdc->dma_chan.device_node,
				&tdma->dma_dev.channels);
		tdc->tdma = tdma;
		tdc->id = i;

		tasklet_init(&tdc->tasklet, tegra_dma_tasklet,
				(unsigned long)tdc);
		spin_lock_init(&tdc->lock);

		INIT_LIST_HEAD(&tdc->pending_sg_req);
		INIT_LIST_HEAD(&tdc->free_sg_req);
		INIT_LIST_HEAD(&tdc->free_dma_desc);
		INIT_LIST_HEAD(&tdc->cb_desc);
	}

	dma_cap_set(DMA_SLAVE, tdma->dma_dev.cap_mask);
	dma_cap_set(DMA_PRIVATE, tdma->dma_dev.cap_mask);

	tdma->dma_dev.dev = &pdev->dev;
	tdma->dma_dev.device_alloc_chan_resources =
					tegra_dma_alloc_chan_resources;
	tdma->dma_dev.device_free_chan_resources =
					tegra_dma_free_chan_resources;
	tdma->dma_dev.device_prep_slave_sg = tegra_dma_prep_slave_sg;
	tdma->dma_dev.device_control = tegra_dma_device_control;
	tdma->dma_dev.device_tx_status = tegra_dma_tx_status;
	tdma->dma_dev.device_issue_pending = tegra_dma_issue_pending;

	ret = dma_async_device_register(&tdma->dma_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"GPC DMA driver registration failed %d\n", ret);
		goto err_irq;
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 tegra_dma_of_xlate, tdma);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"GPC DMA OF registration failed %d\n", ret);
		goto err_unregister_dma_dev;
	}

	dev_info(&pdev->dev, "GPC DMA driver register %d channels\n",
			cdata->nr_channels);
	return 0;

err_unregister_dma_dev:
	dma_async_device_unregister(&tdma->dma_dev);
err_irq:
	while (--i >= 0) {
		struct tegra_dma_channel *tdc = &tdma->channels[i];
		tasklet_kill(&tdc->tasklet);
	}

err_pm_disable:
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra_dma_runtime_suspend(&pdev->dev);
	tegra_pd_remove_device(&pdev->dev);
#endif
	return ret;
}

static int tegra_dma_remove(struct platform_device *pdev)
{
	struct tegra_dma *tdma = platform_get_drvdata(pdev);
	int i;
	struct tegra_dma_channel *tdc;

	dma_async_device_unregister(&tdma->dma_dev);

	for (i = 0; i < tdma->chip_data->nr_channels; ++i) {
		tdc = &tdma->channels[i];
		tasklet_kill(&tdc->tasklet);
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra_dma_runtime_suspend(&pdev->dev);

	tegra_pd_remove_device(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int tegra_dma_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_dma *tdma = platform_get_drvdata(pdev);

	clk_disable(tdma->dma_clk);
	return 0;
}

static int tegra_dma_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_dma *tdma = platform_get_drvdata(pdev);
	int ret;

	ret = clk_enable(tdma->dma_clk);
	if (ret < 0) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int tegra_dma_pm_suspend(struct device *dev)
{
	struct tegra_dma *tdma = dev_get_drvdata(dev);
	int i;
	int ret;

	ret = tegra_dma_runtime_resume(dev);
	if (ret < 0)
		return ret;

	for (i = 0; i < tdma->chip_data->nr_channels; i++) {
		struct tegra_dma_channel *tdc = &tdma->channels[i];
		struct tegra_dma_channel_regs *ch_reg = &tdc->channel_reg;

		ch_reg->csr = tdc_read(tdc, TEGRA_GPCDMA_CHAN_CSR);
		ch_reg->src_ptr = tdc_read(tdc, TEGRA_GPCDMA_CHAN_SRC_PTR);
		ch_reg->dst_ptr = tdc_read(tdc, TEGRA_GPCDMA_CHAN_DST_PTR);
		ch_reg->mc_seq = tdc_read(tdc, TEGRA_GPCDMA_CHAN_MCSEQ);
		ch_reg->mmio_seq = tdc_read(tdc, TEGRA_GPCDMA_CHAN_MMIOSEQ);
		ch_reg->bcount = tdc_read(tdc, TEGRA_GPCDMA_CHAN_BCOUNT);
	}
	tegra_dma_runtime_suspend(dev);
	return 0;
}

static int tegra_dma_pm_resume(struct device *dev)
{
	struct tegra_dma *tdma = dev_get_drvdata(dev);
	int i;
	int ret;

	ret = tegra_dma_runtime_resume(dev);
	if (ret < 0)
		return ret;

	for (i = 0; i < tdma->chip_data->nr_channels; i++) {
		struct tegra_dma_channel *tdc = &tdma->channels[i];
		struct tegra_dma_channel_regs *ch_reg = &tdc->channel_reg;

		tdc_write(tdc, TEGRA_GPCDMA_CHAN_BCOUNT, ch_reg->bcount);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_MMIOSEQ, ch_reg->mmio_seq);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_DST_PTR, ch_reg->dst_ptr);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_MCSEQ, ch_reg->mc_seq);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_SRC_PTR, ch_reg->src_ptr);
		tdc_write(tdc, TEGRA_GPCDMA_CHAN_CSR,
			(ch_reg->csr & ~TEGRA_GPCDMA_CSR_ENB));
	}
	tegra_dma_runtime_suspend(dev);
	return 0;
}

int tegra_gpcdma_save(void)
{
	return tegra_dma_pm_suspend(dma_device);
}

int tegra_gpcdma_restore(void)
{
	return tegra_dma_pm_resume(dma_device);
}
#endif

static const struct dev_pm_ops tegra_dma_dev_pm_ops = {
#ifdef CONFIG_PM_SLEEP
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = tegra_dma_runtime_suspend,
	.runtime_resume = tegra_dma_runtime_resume,
#endif
	SET_SYSTEM_SLEEP_PM_OPS(tegra_dma_pm_suspend, tegra_dma_pm_resume)
#endif
};

static struct platform_driver tegra_dmac_driver = {
	.driver = {
		.name	= "tegra-gpcdma",
		.owner = THIS_MODULE,
		.pm	= &tegra_dma_dev_pm_ops,
		.of_match_table = of_match_ptr(tegra_dma_of_match),
	},
	.probe		= tegra_dma_probe,
	.remove		= tegra_dma_remove,
	.id_table	= tegra_dma_devtype,
};

module_platform_driver(tegra_dmac_driver);

MODULE_ALIAS("platform:tegra186-gpcdma");
MODULE_DESCRIPTION("NVIDIA Tegra GPC DMA Controller driver");
MODULE_AUTHOR("Pavan Kunapuli <pkunapuli@nvidia.com>");

/* STATUS register */
MODULE_LICENSE("GPL v2");
