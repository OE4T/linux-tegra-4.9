/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/device.h>
#include <linux/tegra-hsp.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tegra_ast.h>
#include <linux/tegra-aon.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>

#define AON_SLAVE_ADDR			0x80000000
#define IVC_MIN_FRAME_SIZE		4

#define SMBOX1_OFFSET			0x8000
#define SMBOX_IVC_READY_MSG		0xAAAA5555

struct tegra_aon {
	struct mbox_controller mbox;
	int hsp_master;
};

struct tegra_aon_ivc_chan {
	char *name;
	int hsp_db;
	u32 nframes;
	u32 framesz;
	void *tx_base;
	void *rx_base;
	struct ivc ivc;
	bool last_tx_done;
};

static uint64_t aon_ipcbuf_phys = 0;
static uint64_t aon_ipcbuf_size = 0;

static void tegra_aon_notify_remote(struct ivc *ivc)
{
	struct tegra_aon_ivc_chan *ivc_chan;

	ivc_chan = container_of(ivc, struct tegra_aon_ivc_chan, ivc);
	tegra_hsp_db_ring(ivc_chan->hsp_db);
}

static void hsp_irq_handler(int master, void *data)
{
	struct tegra_aon *aon;
	struct mbox_chan *mbox_chan;
	struct tegra_aon_ivc_chan *ivc_chan;
	struct tegra_aon_mbox_msg msg;
	u32 *buf;
	int i;

	aon = (struct tegra_aon *)data;
	for (i = 0; i < aon->mbox.num_chans; i++) {
		mbox_chan = &aon->mbox.chans[i];
		ivc_chan = (struct tegra_aon_ivc_chan *)mbox_chan->con_priv;
		while (tegra_ivc_can_read(&ivc_chan->ivc)) {
			buf = tegra_ivc_read_get_next_frame(&ivc_chan->ivc);
			msg.length = ivc_chan->framesz;
			msg.data = buf;
			mbox_chan_received_data(mbox_chan, &msg);
			tegra_ivc_read_advance(&ivc_chan->ivc);
		}
	}
}

#define NV(p) "nvidia," p

static int parse_channel(struct device *dev, struct mbox_chan *mbox_chan,
			 struct device_node *np, int hsp_db)
{
	struct tegra_aon_ivc_chan *ivc_chan;
	uint64_t tx_offset, rx_offset, ch_size;
	int ret = 0;

	/* Sanity check */
	if (!mbox_chan || !np || !dev)
		return -EINVAL;

	ivc_chan = devm_kzalloc(dev, sizeof(*ivc_chan), GFP_KERNEL);
	if (!ivc_chan) {
		dev_err(dev, "out of memory.\n");
		return -ENOMEM;
	}

	ivc_chan->name = devm_kstrdup(dev, np->name, GFP_KERNEL);
	if (!ivc_chan->name)
		return -ENOMEM;

	ret = of_property_read_u64(np, "ivc-tx", &tx_offset);
	ret |= of_property_read_u64(np, "ivc-rx", &rx_offset);
	ret |= of_property_read_u64(np, "ivc-channel-size", &ch_size);
	ret |= of_property_read_u32(np, "ivc-num-frames", &ivc_chan->nframes);
	ret |= of_property_read_u32(np, "ivc-frame-size", &ivc_chan->framesz);
	if (ret) {
		dev_err(dev, "invalid ivc channel property.\n");
		return ret;
	}

	if (ivc_chan->framesz < IVC_MIN_FRAME_SIZE ||
		ivc_chan->framesz > ch_size) {
		dev_err(dev,
			"ivc-frame-size exceeds range (%d-%lld).\n",
			IVC_MIN_FRAME_SIZE, ch_size);
		return -EINVAL;
	}

	if (!ivc_chan->nframes) {
		dev_err(dev, "invalid <ivc-num-frames> property.\n");
		return -EINVAL;
	}

	if(ch_size <
	   ((ivc_chan->nframes * ivc_chan->framesz) + (2 * IVC_ALIGN))) {
		dev_err(dev, "ivc channel size too small.\n");
		return -EINVAL;
	}

	ivc_chan->tx_base = phys_to_virt(aon_ipcbuf_phys + tx_offset);
	ivc_chan->rx_base = phys_to_virt(aon_ipcbuf_phys + rx_offset);

	memset(ivc_chan->tx_base, 0, ch_size);
	memset(ivc_chan->rx_base, 0, ch_size);

	/* Allocate the IVC links */
	ret = tegra_ivc_init(&ivc_chan->ivc,
			     (uintptr_t)ivc_chan->rx_base,
			     (uintptr_t)ivc_chan->tx_base,
			     ivc_chan->nframes, ivc_chan->framesz,
			     dev,
			     tegra_aon_notify_remote);
	if (ret) {
		dev_err(dev,
			"failed to instantiate IVC.\n");
		return ret;
	}
	ivc_chan->hsp_db = hsp_db;
	mbox_chan->con_priv = ivc_chan;

	dev_dbg(dev, "%s: TX: 0x%lx-0x%lx\n",
		 ivc_chan->name,
		 (ulong)ivc_chan->tx_base,
		 (ulong)ivc_chan->tx_base + (u32)ch_size);
	dev_dbg(dev, "%s: RX: 0x%lx-0x%lx\n",
		 ivc_chan->name,
		 (ulong)ivc_chan->rx_base,
		 (ulong)ivc_chan->rx_base + (u32)ch_size);

	return ret;
}

static int tegra_aon_mbox_send_data(struct mbox_chan *mbox_chan, void *data)
{
	struct tegra_aon_ivc_chan *ivc_chan;
	struct tegra_aon_mbox_msg *msg;
	u32 bytes;
	int ret;

	msg = (struct tegra_aon_mbox_msg *)data;
	ivc_chan = (struct tegra_aon_ivc_chan *)mbox_chan->con_priv;
	bytes = tegra_ivc_write(&ivc_chan->ivc, msg->data, msg->length);
	ret = (bytes != msg->length) ? -EBUSY : 0;
	ivc_chan->last_tx_done = (bytes != msg->length) ? false : true;

	return ret;
}

static int tegra_aon_mbox_startup(struct mbox_chan *mbox_chan)
{
	return 0;
}

static void tegra_aon_mbox_shutdown(struct mbox_chan *mbox_chan)
{
}

static bool tegra_aon_mbox_last_tx_done(struct mbox_chan *mbox_chan)
{
	struct tegra_aon_ivc_chan *ivc_chan;

	ivc_chan = (struct tegra_aon_ivc_chan *)mbox_chan->con_priv;

	return ivc_chan->last_tx_done;
}

static struct mbox_chan_ops tegra_aon_mbox_chan_ops = {
	.send_data = tegra_aon_mbox_send_data,
	.startup = tegra_aon_mbox_startup,
	.shutdown = tegra_aon_mbox_shutdown,
	.last_tx_done = tegra_aon_mbox_last_tx_done,
};

static int tegra_aon_probe(struct platform_device *pdev)
{
	struct tegra_aon *aon;
	struct tegra_ast *ast0, *ast1;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct device_node *dn = dev->of_node;
	void __iomem *smbox_base;
	u32 hsp_data[2];
	int i, num_chans;
	int ret = 0;

	dev_dbg(&pdev->dev, "tegra aon driver probe Start\n");

	if (!aon_ipcbuf_phys || !aon_ipcbuf_size) {
		dev_err(dev, "no carveout memory.\n");
		return -ENOMEM;
	}

	aon = devm_kzalloc(&pdev->dev, sizeof(*aon), GFP_KERNEL);
	if (!aon)
		return -ENOMEM;
	platform_set_drvdata(pdev, aon);

	smbox_base = of_iomap(dn, 0);
	if (!smbox_base) {
		dev_err(dev, "failed to map smbox IO space.\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_array(dn, NV("notifications"), hsp_data, 2);
	if (ret) {
		dev_err(dev, "missing <notification> property\n");
		goto exit;
	}

	aon->hsp_master = hsp_data[0];

	/* Listen to the remote's notification */
	ret = tegra_hsp_db_add_handler(aon->hsp_master, hsp_irq_handler, aon);
	if (ret)
		goto exit;
	/* Allow remote to ring CCPLEX's doorbell */
	ret = tegra_hsp_db_enable_master(aon->hsp_master);
	if (ret)
		goto exit;

	num_chans = of_get_child_count(dn);
	if (num_chans <= 0) {
		dev_err(dev, "no ivc channels\n");
		ret = -EINVAL;
		goto exit;
	}

	aon->mbox.dev = &pdev->dev;
	aon->mbox.chans = devm_kzalloc(&pdev->dev, num_chans *
					sizeof(*aon->mbox.chans), GFP_KERNEL);
	if (!aon->mbox.chans) {
		ret = -ENOMEM;
		goto exit;
	}

	aon->mbox.num_chans = num_chans;
	aon->mbox.ops = &tegra_aon_mbox_chan_ops;
	aon->mbox.txdone_poll = true;
	aon->mbox.txpoll_period = 1;

	/* Initialize ASTs */
	ast0 = tegra_ast_add_ref(dn, NV("ast0"), 0);
	if (IS_ERR(ast0)) {
		dev_warn(dev, "ast0 not found");
		ret = -EPROBE_DEFER;
		goto exit;
	}

	ast1 = tegra_ast_add_ref(dn, NV("ast1"), 0);
	if (IS_ERR(ast1)) {
		dev_warn(dev, "ast1 not found");
		ret = -EPROBE_DEFER;
		goto exit;
	}

	tegra_ast_region_enable(ast0, 2,
		AON_SLAVE_ADDR, (aon_ipcbuf_size - 1), (u64)aon_ipcbuf_phys);

	tegra_ast_region_enable(ast1, 2,
		AON_SLAVE_ADDR, (aon_ipcbuf_size - 1), (u64)aon_ipcbuf_phys);

	i = 0;
	/* Parse out all channels from DT */
	for_each_child_of_node(dn, child) {
		ret = parse_channel(dev, &aon->mbox.chans[i++], child,
				    hsp_data[1]);
		if (ret) {
			dev_err(dev, "failed to parse a channel\n");
			goto exit;
		}
	}

	ret = mbox_controller_register(&aon->mbox);
	if (ret) {
		dev_err(&pdev->dev, "failed to register mailbox: %d\n", ret);
		goto exit;
	}

	writel(SMBOX_IVC_READY_MSG, smbox_base + SMBOX1_OFFSET);
	dev_dbg(&pdev->dev, "tegra aon driver probe OK\n");

exit:
	iounmap(smbox_base);
	return ret;
}

static int tegra_aon_remove(struct platform_device *pdev)
{
	struct tegra_aon *aon = platform_get_drvdata(pdev);

	mbox_controller_unregister(&aon->mbox);
	tegra_hsp_db_del_handler(aon->hsp_master);

	return 0;
}

static const struct of_device_id tegra_aon_of_match[] = {
	{ .compatible = NV("tegra186-aon"), },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_aon_of_match);

static struct platform_driver tegra_aon_driver = {
	.probe	= tegra_aon_probe,
	.remove = tegra_aon_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_aon",
		.of_match_table = of_match_ptr(tegra_aon_of_match),
	},
};
module_platform_driver(tegra_aon_driver);

static int __init tegra_aon_ipc_setup(struct reserved_mem *rmem)
{
	aon_ipcbuf_phys = rmem->base;
	aon_ipcbuf_size = rmem->size;
	pr_info("aon ipc buffer at phys %llx size %llu\n",
		aon_ipcbuf_phys, aon_ipcbuf_size);

	return 0;
}
RESERVEDMEM_OF_DECLARE(aon_ipc, NV("aon-ipc-carveout"), tegra_aon_ipc_setup);
