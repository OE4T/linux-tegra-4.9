/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/of_platform.h>
#include <linux/mailbox_controller.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <asm/cacheflush.h>

#include <linux/tegra_ast.h>
#include <linux/tegra_camera_rtcpu.h>

#include <dt-bindings/memory/tegra-swgroup.h>
#include <dt-bindings/memory/tegra186-swgroup.h>

/* Register specifics */
#define TEGRA_APS_FRSC_SC_CTL_0			0x0
#define TEGRA_SCE_APS_FRSC_SC_MODEIN_0		0x14
#define TEGRA_SCE_EVP_RESET_ADDR_0		0x20
#define TEGRA_SCEPM_R5_CTRL_0			0x40

#define TEGRA_SCE_R5R_SC_DISABLE		0x5
#define TEGRA_SCE_FN_MODEIN			0x29527
#define TEGRA_SCE_FWLOADDONE			0x2

#define TEGRA_CAM_RTCPU_HSP_DATA_ARRAY_SIZE	3

static const char * const sce_clock_names[] = {
	"sce-apb",
};

static const char * const sce_reset_names[] = {
	"sce-apb",
	"sce-nsysporeset",
	"sce-nreset",
	"sce-dbgresetn",
	"sce-presetdbgn",
	"sce-actmon",
	"sce-pm",
	"sce-dma",
	"sce-hsp",
	"tsctnsce",
	"sce-tke",
	"sce-gte",
	"sce-cfg",
};

static const char * const ape_clock_names[] = {
	"ahub",
	"apb2ape",
	"ape",
	"adsp",
	"adspneon",
};

static const char * const ape_reset_names[] = {
	"adspdbg",
	"adspintf",
	"adspneon",
	"adspperiph",
	"adspscu",
	"adspwdt",
	"ape",
	"adsp",
};

static struct clk *sce_clocks[ARRAY_SIZE(sce_clock_names)];
static struct clk *ape_clocks[ARRAY_SIZE(sce_clock_names)];
static struct reset_control *sce_resets[ARRAY_SIZE(sce_reset_names)];
static struct reset_control *ape_resets[ARRAY_SIZE(ape_reset_names)];

enum tegra_cam_rtcpu_id {
	TEGRA_CAM_RTCPU_SCE,
	TEGRA_CAM_RTCPU_APE,
};

struct tegra_cam_rtcpu_pdata {
	const char *rtcpu_name;
	const char * const *clock_names;
	const char * const *reset_names;
	struct clk **clocks;
	struct reset_control **resets;
	enum tegra_cam_rtcpu_id id;
	u32 sid;
	u32 num_clocks;
	u32 num_resets;
};

static const struct tegra_cam_rtcpu_pdata sce_pdata = {
	.rtcpu_name = "sce",
	.clock_names = sce_clock_names,
	.reset_names = sce_reset_names,
	.clocks = sce_clocks,
	.resets = sce_resets,
	.id = TEGRA_CAM_RTCPU_SCE,
	.sid = TEGRA_SID_SCE,
	.num_clocks = ARRAY_SIZE(sce_clock_names),
	.num_resets = ARRAY_SIZE(sce_reset_names),
};

static const struct tegra_cam_rtcpu_pdata ape_pdata = {
	.rtcpu_name = "ape",
	.clock_names = ape_clock_names,
	.reset_names = ape_reset_names,
	.clocks = ape_clocks,
	.resets = ape_resets,
	.id = TEGRA_CAM_RTCPU_APE,
	.sid = TEGRA_SID_APE,
	.num_clocks = ARRAY_SIZE(ape_clock_names),
	.num_resets = ARRAY_SIZE(ape_reset_names),
};

#define NV(p) "nvidia," #p

static struct of_device_id tegra_cam_rtcpu_of_match[] = {
	{
		.compatible = NV(tegra186-sce-ivc), .data = &sce_pdata
	},
	{
		.compatible = NV(tegra186-ape-ivc), .data = &ape_pdata
	},
	{},
};
MODULE_DEVICE_TABLE(of, tegra_cam_rtcpu_of_match);

struct tegra_cam_rtcpu {
	struct mbox_controller mbox;
	int hsp_master;
	int hsp_db;
	union {
		struct {
			void __iomem *sce_cfg_base;
			void __iomem *sce_arsce_evp;
			void __iomem *sce_pm_base;
		} __packed rtcpu_sce;
	};
	struct work_struct ch_rx_work;
	const struct tegra_cam_rtcpu_pdata *rtcpu_pdata;
};

struct tegra_cam_rtcpu_ivc_chan {
	struct ivc ivc;
	char *name;
	struct tegra_cam_rtcpu *cam_rtcpu;
	bool last_tx_done;
};

static void tegra_cam_rtcpu_ivc_notify(struct ivc *ivc)
{
	int ret;
	struct tegra_cam_rtcpu_ivc_chan *ivc_chan;

	ivc_chan = container_of(ivc, struct tegra_cam_rtcpu_ivc_chan, ivc);
	ret = tegra_hsp_db_ring(ivc_chan->cam_rtcpu->hsp_db);
	if (ret)
		pr_err("tegra_hsp_db_ring failed: %d\n", ret);
}

static void cam_rtcpu_rx_worker(struct work_struct *work)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct mbox_chan *mbox_chan;
	struct ivc *ivc;
	struct tegra_ivc_mbox_msg msg;
	int i;

	cam_rtcpu = container_of(work, struct tegra_cam_rtcpu, ch_rx_work);

	for (i = 0; i < cam_rtcpu->mbox.num_chans; i++) {
		mbox_chan = &cam_rtcpu->mbox.chans[i];
		ivc = mbox_chan->con_priv;

		while (tegra_ivc_can_read(ivc)) {
			msg.data = tegra_ivc_read_get_next_frame(ivc);
			msg.length = ivc->frame_size;
			mbox_chan_received_data(mbox_chan, &msg);
			tegra_ivc_read_advance(ivc);
		}
	}
}

static void tegra_cam_rtcpu_isr(int irq, void *data)
{
	struct tegra_cam_rtcpu *cam_rtcpu = data;

	schedule_work(&cam_rtcpu->ch_rx_work);
}

static int tegra_cam_rtcpu_parse_channel(struct device *dev,
			uintptr_t ivc_base, dma_addr_t ivc_dma, u32 ivc_size,
			struct mbox_chan *mbox_chan,
			struct device_node *ch_node)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct tegra_cam_rtcpu_ivc_chan *ivc_chan;
	struct {
		u32 rx, tx;
	} start, end;
	int ret;
	u32 nframes, frame_size;

	cam_rtcpu = dev_get_drvdata(dev);

	ret = of_property_read_u32_array(ch_node, "reg", &start.rx, 2);
	if (ret) {
		dev_err(dev, "missing <%s> property\n", "reg");
		return ret;
	}
	ret = of_property_read_u32(ch_node, NV(frame-count), &nframes);
	if (ret) {
		dev_err(dev, "missing <%s> property\n", NV(frame-count));
		return ret;
	}
	ret = of_property_read_u32(ch_node, NV(frame-size), &frame_size);
	if (ret) {
		dev_err(dev, "missing <%s> property\n", NV(frame-size));
		return ret;
	}

	end.rx = start.rx + tegra_ivc_total_queue_size(nframes * frame_size);
	end.tx = start.tx + tegra_ivc_total_queue_size(nframes * frame_size);

	if (end.rx > ivc_size) {
		dev_err(dev, "%s buffer exceeds ivc size\n", "rx");
		return -ENOMEM;
	}
	if (end.tx > ivc_size) {
		dev_err(dev, "%s buffer exceeds ivc size\n", "tx");
		return -ENOMEM;
	}

	if (start.tx < start.rx ? end.tx > start.rx : end.rx > start.tx) {
		dev_err(dev, "rx and tx buffers overlap on channel %s\n",
			ch_node->name);
		return -ENOMEM;
	}

	ivc_chan = devm_kzalloc(dev, sizeof(*ivc_chan), GFP_KERNEL);
	if (!ivc_chan) {
		dev_err(dev, "failed to allocate CAM RTCPU IVC channel\n");
		return -ENOMEM;
	}

	ivc_chan->name = devm_kstrdup(dev, ch_node->name, GFP_KERNEL);
	if (!ivc_chan->name)
		return -ENOMEM;

	ivc_chan->cam_rtcpu = cam_rtcpu;
	mbox_chan->con_priv = ivc_chan;

	/* Init IVC */
	ret = tegra_ivc_init_with_dma_handle(&ivc_chan->ivc,
				ivc_base + start.rx,
				(u64)ivc_dma + start.rx,
				ivc_base + start.tx,
				(u64)ivc_dma + start.tx,
				nframes,
				frame_size,
				dev,
				tegra_cam_rtcpu_ivc_notify);
	if (ret) {
		dev_err(dev, "failed to init IVC.\n");
		return ret;
	}

	dev_dbg(dev, "%s: RX: 0x%x-0x%x TX: 0x%x-0x%x\n",
		ivc_chan->name, start.rx, end.rx, start.tx, end.tx);

	return ret;
}

static int tegra_cam_rtcpu_check_overlap(struct device *dev,
			struct tegra_cam_rtcpu_ivc_chan *ch0,
			struct tegra_cam_rtcpu_ivc_chan *ch1)
{
	unsigned s0, s1;
	uintptr_t tx0, rx0, tx1, rx1;

	if (ch0 == NULL || ch1 == NULL)
		return -EINVAL;

	tx0 = (uintptr_t)ch0->ivc.tx_channel;
	rx0 = (uintptr_t)ch0->ivc.rx_channel;
	s0 = ch0->ivc.nframes * ch0->ivc.frame_size;
	s0 = tegra_ivc_total_queue_size(s0);

	tx1 = (uintptr_t)ch1->ivc.tx_channel;
	rx1 = (uintptr_t)ch1->ivc.rx_channel;
	s1 = ch1->ivc.nframes * ch1->ivc.frame_size;
	s1 = tegra_ivc_total_queue_size(s1);

	if ((tx0 < tx1 ? tx0 + s0 > tx1 : tx1 + s1 > tx0) ||
		(rx0 < tx1 ? rx0 + s0 > tx1 : tx1 + s1 > rx0) ||
		(rx0 < rx1 ? rx0 + s0 > rx1 : rx1 + s1 > rx0) ||
		(tx0 < rx1 ? tx0 + s0 > rx1 : rx1 + s1 > tx0)) {
		dev_err(dev, "ivc buffers overlap on channels %s and %s\n",
			ch0->name, ch1->name);
		return -EINVAL;
	}

	return 0;
}

static int tegra_cam_rtcpu_validate_channels(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct tegra_cam_rtcpu_ivc_chan *i_chan, *j_chan;
	int i, j;
	int ret;

	cam_rtcpu = dev_get_drvdata(dev);

	for (i = 0; i < cam_rtcpu->mbox.num_chans; i++) {
		i_chan = cam_rtcpu->mbox.chans[i].con_priv;

		if (i_chan == NULL)
			return -EINVAL;

		for (j = i + 1; j < cam_rtcpu->mbox.num_chans; j++) {
			j_chan = cam_rtcpu->mbox.chans[j].con_priv;

			ret = tegra_cam_rtcpu_check_overlap(dev,
							i_chan, j_chan);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_parse_channels(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct device_node *reg_node, *ch_node;
	struct tegra_ast *ast0;
	struct tegra_ast *ast1;
	struct {
		u32 va, size;
	} ivc;
	uintptr_t ivc_base;
	dma_addr_t ivc_dma;
	int ret, i, region;

	cam_rtcpu = dev_get_drvdata(dev);

	/* Get AST handles */
	ast0 = tegra_ast_add_ref(dev->of_node, NV(ast), 0);
	if (IS_ERR(ast0)) {
		dev_err(dev, "%s:%d not found", NV(ast), 0);
		return PTR_ERR(ast0);
	}

	ast1 = tegra_ast_add_ref(dev->of_node, NV(ast), 1);
	if (IS_ERR(ast1)) {
		dev_err(dev, "%s:%d not found", NV(ast), 1);
		return PTR_ERR(ast1);
	}

	i = 0;

	/* Use AST vmindex 0 for all regions, set default stream ID */
	ret = tegra_ast_set_streamid(ast0, 0, cam_rtcpu->rtcpu_pdata->sid);
	ret = tegra_ast_set_streamid(ast1, 0, cam_rtcpu->rtcpu_pdata->sid);

	/* AST regions 0 and 1 are used for DRAM and SYSRAM carveouts */
	region = 2;

	/* Parse out all nodes with a region */
	for_each_child_of_node(dev->of_node, reg_node) {
		if (strcmp(reg_node->name, "ivc-channels"))
			continue;

		/* IVC VA addr and size */
		ret = of_property_read_u32_array(reg_node, "reg", &ivc.va, 2);
		if (ret) {
			dev_err(dev, "Invalid <%s> property\n", "reg");
			return -EINVAL;
		}

		/* IVC buffer size must be a power of 2 */
		if (ivc.size & (ivc.size - 1)) {
			dev_err(dev, "Invalid region size 0x%08X\n", ivc.size);
			return -EINVAL;
		}

		/* Allocate RAM for IVC */
		ivc_base = (uintptr_t)dmam_alloc_coherent(dev, ivc.size,
				&ivc_dma, GFP_KERNEL | __GFP_ZERO);
		if (ivc_base == 0)
			return -ENOMEM;

		ret = tegra_ast_region_enable(ast0, region,
				ivc.va, ivc.size - 1, ivc_dma);
		if (ret)
			return ret;

		ret = tegra_ast_region_enable(ast1, region,
				ivc.va, ivc.size - 1, ivc_dma);
		if (ret)
			return ret;

		region++;

		for_each_child_of_node(reg_node, ch_node) {
			ret = tegra_cam_rtcpu_parse_channel(dev,
					ivc_base, ivc_dma, ivc.size,
					&cam_rtcpu->mbox.chans[i++], ch_node);
			if (ret)
				return ret;
		}
	}

	return tegra_cam_rtcpu_validate_channels(dev);
}

static int tegra_cam_rtcpu_count_channels(struct device_node *dev_node)
{
	int num = 0;
	struct device_node *ch_sub_node, *ch_node;

	for_each_child_of_node(dev_node, ch_sub_node) {
		if (strcmp(ch_sub_node->name, "ivc-channels"))
			continue;

		for_each_child_of_node(ch_sub_node, ch_node) {
			num++;
		}
	}

	return num;
}

static int tegra_cam_rtcpu_mbox_send_data(struct mbox_chan *mbox_chan,
					void *data)
{
	struct tegra_cam_rtcpu_ivc_chan *ivc_chan;
	struct tegra_ivc_mbox_msg *msg;
	u32 bytes;
	int ret;

	ivc_chan = mbox_chan->con_priv;
	msg = data;
	bytes = tegra_ivc_write(&ivc_chan->ivc, msg->data, msg->length);

	ret = (bytes == msg->length) ? 0 : -EBUSY;
	ivc_chan->last_tx_done = (ret == 0);

	return ret;
}

static int tegra_cam_rtcpu_mbox_startup(struct mbox_chan *mbox_chan)
{
	return 0;
}

static void tegra_cam_rtcpu_mbox_shutdown(struct mbox_chan *mbox_chan)
{
}

static bool tegra_cam_rtcpu_mbox_last_tx_done(struct mbox_chan *mbox_chan)
{
	struct tegra_cam_rtcpu_ivc_chan *ivc_chan;

	ivc_chan = mbox_chan->con_priv;

	return ivc_chan->last_tx_done;
}

static struct mbox_chan_ops tegra_cam_rtcpu_mbox_chan_ops = {
	.send_data = tegra_cam_rtcpu_mbox_send_data,
	.startup = tegra_cam_rtcpu_mbox_startup,
	.shutdown = tegra_cam_rtcpu_mbox_shutdown,
	.last_tx_done = tegra_cam_rtcpu_mbox_last_tx_done,
};

static int tegra_cam_rtcpu_get_clks_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	/* Get clocks and resets */
	for (i = 0; i < pdata->num_clocks; i++) {
		pdata->clocks[i] = devm_clk_get(dev, pdata->clock_names[i]);
		if (IS_ERR(pdata->clocks[i])) {
			dev_err(dev, "clock %s not found: %ld\n",
				pdata->clock_names[i],
				PTR_ERR(pdata->clocks[i]));
			if (PTR_ERR(pdata->clocks[i]) == -EPROBE_DEFER)
				return PTR_ERR(pdata->clocks[i]);
		}
	}

	for (i = 0; i < pdata->num_resets; i++) {
		pdata->resets[i] =
			devm_reset_control_get(dev, pdata->reset_names[i]);
		if (IS_ERR(pdata->resets[i])) {
			dev_err(dev, "reset %s not found: %ld\n",
				pdata->reset_names[i],
				PTR_ERR(pdata->resets[i]));
			if (PTR_ERR(pdata->resets[i]) == -EPROBE_DEFER)
				return PTR_ERR(pdata->resets[i]);
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_apply_clks(struct device *dev,
				int (*func)(struct clk *clk))
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i, ret;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	for (i = 0; i < pdata->num_clocks; i++) {
		if (IS_ERR(pdata->clocks[i]))
			continue;
		ret = (*func)(pdata->clocks[i]);
		if (ret) {
			dev_err(dev, "clock %s failed: %d\n",
				pdata->clock_names[i], ret);
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_apply_resets(struct device *dev,
			int (*func)(struct reset_control *))
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i, ret;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	for (i = 0; i < pdata->num_resets; i++) {
		if (IS_ERR(pdata->resets[i]))
			continue;
		ret = (*func)(pdata->resets[i]);
		if (ret) {
			dev_err(dev, "reset %s failed: %d\n",
				pdata->reset_names[i], ret);
		}
	}

	return 0;
}

static void tegra_cam_rtcpu_boot(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);
	u32 reg_val;

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* Disable SCE R5R and smartcomp in camera mode */
		writel(TEGRA_SCE_R5R_SC_DISABLE,
			cam_rtcpu->rtcpu_sce.sce_cfg_base +
					TEGRA_APS_FRSC_SC_CTL_0);

		/* Enable JTAG/Coresight */
		writel(TEGRA_SCE_FN_MODEIN,
			cam_rtcpu->rtcpu_sce.sce_cfg_base +
					TEGRA_SCE_APS_FRSC_SC_MODEIN_0);

		/* Set FW load done bit */
		reg_val = readl(cam_rtcpu->rtcpu_sce.sce_pm_base +
				TEGRA_SCEPM_R5_CTRL_0);
		writel(TEGRA_SCE_FWLOADDONE | reg_val,
			cam_rtcpu->rtcpu_sce.sce_pm_base +
			TEGRA_SCEPM_R5_CTRL_0);

		dev_info(dev, "booting sce");
	} /* else the rtcpu is ape. */
}

static int tegra_cam_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct device *dev = &pdev->dev;
	struct device_node *dev_node = dev->of_node;
	u32 hsp_data[TEGRA_CAM_RTCPU_HSP_DATA_ARRAY_SIZE];
	int num_chans;
	int ret;
	const struct of_device_id *match;

	dev_dbg(dev, "probing\n");

	cam_rtcpu = devm_kzalloc(dev, sizeof(*cam_rtcpu), GFP_KERNEL);
	if (!cam_rtcpu)
		return -ENOMEM;
	platform_set_drvdata(pdev, cam_rtcpu);

	match = of_match_device(tegra_cam_rtcpu_of_match, dev);
	if (match == NULL) {
		dev_err(dev, "Device match not found\n");
		return -ENODEV;
	}

	cam_rtcpu->rtcpu_pdata = match->data;

	ret = of_property_read_u32_array(dev_node, NV(hsp-notifications),
				hsp_data, ARRAY_SIZE(hsp_data));
	if (ret) {
		dev_err(dev, "Missing <%s> property\n",
			NV(hsp-notifications));
		return ret;
	}

	/* hsp_data[0] is HSP phandle */
	cam_rtcpu->hsp_master = hsp_data[1];
	cam_rtcpu->hsp_db = hsp_data[2];

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* RTCPU base addr / SCE EVP addr */
		cam_rtcpu->rtcpu_sce.sce_arsce_evp = of_iomap(dev_node, 0);
		if (!cam_rtcpu->rtcpu_sce.sce_arsce_evp) {
			dev_err(dev, "failed to map SCE EVP space.\n");
			return -EINVAL;
		}

		/* SCE PM addr */
		cam_rtcpu->rtcpu_sce.sce_pm_base = of_iomap(dev_node, 1);
		if (!cam_rtcpu->rtcpu_sce.sce_pm_base) {
			dev_err(dev, "failed to map SCE PM space.\n");
			return -EINVAL;
		}

		/* SCE CFG addr */
		cam_rtcpu->rtcpu_sce.sce_cfg_base = of_iomap(dev_node, 2);
		if (!cam_rtcpu->rtcpu_sce.sce_cfg_base) {
			dev_err(dev, "failed to map SCE CFG space.\n");
			return -EINVAL;
		}
	}

	ret = tegra_cam_rtcpu_get_clks_resets(dev);
	if (ret) {
		dev_err(dev, "failed to get clocks/resets: %d\n", ret);
		return ret;
	}

	num_chans = tegra_cam_rtcpu_count_channels(dev_node);
	if (num_chans <= 0) {
		dev_err(dev, "no ivc channels\n");
		return -EINVAL;
	}

	cam_rtcpu->mbox.dev = dev;
	cam_rtcpu->mbox.chans = devm_kzalloc(dev,
				num_chans * sizeof(*cam_rtcpu->mbox.chans),
				GFP_KERNEL);
	if (!cam_rtcpu->mbox.chans)
		return -ENOMEM;

	cam_rtcpu->mbox.num_chans = num_chans;
	cam_rtcpu->mbox.ops = &tegra_cam_rtcpu_mbox_chan_ops;
	cam_rtcpu->mbox.txdone_poll = true;
	cam_rtcpu->mbox.txpoll_period = 1;

	ret = tegra_cam_rtcpu_parse_channels(dev);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "ivc-channels set up failed: %d\n", ret);
		else
			dev_err(dev, "ivc-channels set up deferred\n");
		return ret;
	}

	/* Init IVC channels ch_rx_work */
	INIT_WORK(&cam_rtcpu->ch_rx_work, cam_rtcpu_rx_worker);

	ret = mbox_controller_register(&cam_rtcpu->mbox);
	if (ret) {
		dev_err(dev, "failed to register mailbox: %d\n", ret);
		return ret;
	}

	/* Listen to the remote's notification */
	ret = tegra_hsp_db_add_handler(cam_rtcpu->hsp_master,
					tegra_cam_rtcpu_isr, cam_rtcpu);
	if (ret) {
		dev_err(dev, "failed to add db handler: %d\n", ret);
		goto fail;
	}

	/* Allow remote to ring CCPLEX's doorbell */
	ret = tegra_hsp_db_enable_master(cam_rtcpu->hsp_master);
	if (ret) {
		dev_err(dev, "failed to enable db master: %d\n", ret);
		goto fail;
	}

	ret = tegra_cam_rtcpu_apply_clks(dev, clk_prepare_enable);
	if (ret) {
		dev_err(dev, "failed to turn on clocks: %d\n", ret);
		goto fail;
	}

	tegra_cam_rtcpu_apply_resets(dev, reset_control_deassert);

	tegra_cam_rtcpu_boot(dev);

	dev_dbg(dev, "probe successful\n");

	return 0;

fail:
	tegra_hsp_db_del_handler(cam_rtcpu->hsp_master);
	mbox_controller_unregister(&cam_rtcpu->mbox);
	return ret;
}

static int tegra_cam_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = platform_get_drvdata(pdev);

	mbox_controller_unregister(&cam_rtcpu->mbox);
	tegra_hsp_db_del_handler(cam_rtcpu->hsp_master);
	tegra_ast_del_ref();

	return 0;
}

static int tegra_cam_rtcpu_resume(struct device *dev)
{
	/*
	 * TODO:
	 * Call tegra_cam_rtcpu_clks_resets_on() and tegra_cam_rtcpu_boot()?
	 */

	return 0;
}

static int tegra_cam_rtcpu_suspend(struct device *dev)
{
	/*
	 * TODO:
	 *
	 * 1. Signal(through new control channel) RTCPU to finish off pending
	 *    tasks and assert standbyWFI.
	 * 2. Wait until acknowledgement from RTCPU for standbyWFI assert.
	 * 3. Once ack recieved check SCEPM_PWR_STATUS_0 register for
	 *    WFIPIPESTOPPED to confirm standbyWFI assert is done.
	 * 4. If OK proceed for SC7.
	 * 5. Disable clocks and assert resets.
	 */

	return 0;
}

const struct dev_pm_ops tegra_cam_rtcpu_pm_ops = {
	.suspend = tegra_cam_rtcpu_suspend,
	.resume = tegra_cam_rtcpu_resume,
};

static struct platform_driver tegra_cam_rtcpu_driver = {
	.driver = {
		.name	= "tegra186-cam-rtcpu",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_cam_rtcpu_of_match),
#ifdef CONFIG_PM
		.pm = &tegra_cam_rtcpu_pm_ops,
#endif
	},
	.probe = tegra_cam_rtcpu_probe,
	.remove = tegra_cam_rtcpu_remove,
};
module_platform_driver(tegra_cam_rtcpu_driver);

MODULE_DESCRIPTION("CAMERA RTCPU driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
