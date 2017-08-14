/*
 * NVCSI driver for T186
 *
 * Copyright (c) 2014-2017, NVIDIA Corporation.  All rights reserved.
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
#include <linux/device.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <asm/ioctls.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/nvhost_nvcsi_ioctl.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <media/mc_common.h>
#include <media/csi.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t186/t186.h"
#include "nvcsi.h"
#include "camera/csi/csi4_fops.h"

#define DESKEW_TIMEOUT_USEC 300

static struct tegra_csi_device *mc_csi;
static struct tegra_csi_data t18_nvcsi_data = {
	.info = (struct nvhost_device_data *)&t18_nvcsi_info,
	.csi_fops = &csi4_fops,
};

static const struct of_device_id tegra_nvcsi_of_match[] = {
	{
		.compatible = "nvidia,tegra186-nvcsi",
		.data = &t18_nvcsi_data
	},
	{ },
};

struct nvcsi_private {
	struct platform_device *pdev;
	unsigned int active_lanes;
};
/* debugfs variables */
static unsigned int debugfs_deskew_clk_stats_low[NVCSI_PHY_CIL_NUM_LANE];
static unsigned int debugfs_deskew_clk_stats_high[NVCSI_PHY_CIL_NUM_LANE];
static unsigned int debugfs_deskew_data_stats_low[NVCSI_PHY_CIL_NUM_LANE];
static unsigned int debugfs_deskew_data_stats_high[NVCSI_PHY_CIL_NUM_LANE];
static unsigned long long input_stats;
static int nvcsi_deskew_debugfs_init(struct nvcsi *nvcsi);
static void nvcsi_deskew_debugfs_remove(struct nvcsi *nvcsi);
/* interrupt variables */
static int deskew_active;
static unsigned int int_lanes;

static int nvcsi_deskew_apply_helper(unsigned int active_lanes);

static inline void enable_int_mask(unsigned int active_lanes,
				   unsigned int enable)
{
	unsigned int phy_num = 0;
	unsigned int cil_lanes = 0, cila_io_lanes = 0, cilb_io_lanes = 0;
	unsigned int remaining_lanes = active_lanes;
	unsigned int val = 0, newval = 0;

	while (remaining_lanes) {
		cil_lanes = (active_lanes & (0x000f << (phy_num * 4)))
				>> (phy_num * 4);
		cila_io_lanes =  cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_A_IO0
			| NVCSI_PHY_0_NVCSI_CIL_A_IO1);
		cilb_io_lanes = cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_B_IO0
			| NVCSI_PHY_0_NVCSI_CIL_B_IO1);
		remaining_lanes &= ~(0xf << (phy_num * 4));
		if (cila_io_lanes) {
			newval = ((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO0)
				!= 0 ?
				intr_dphy_cil_deskew_calib_done_lane0 : 0) |
				((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO1)
				!= 0 ?
				intr_dphy_cil_deskew_calib_done_lane1 : 0) |
				intr_dphy_cil_deskew_calib_done_ctrl;
			newval = enable ? newval : enable;
			/* ONLY handles deskew done interrupt, also
			 * turn off any other interrupt for NVCSI
			 * because ISR does not handle them*/
			host1x_writel(mc_csi->pdev,
				     NVCSI_PHY_0_CILA_INTR_MASK +
				     NVCSI_PHY_OFFSET * phy_num,
				     ~newval);
			dev_dbg(mc_csi->dev, "addr %x prev %x mask %x new %x\n",
				NVCSI_PHY_0_CILA_INTR_MASK +
				NVCSI_PHY_OFFSET * phy_num, val, newval,
				~newval);
			/* Disable single bit err when detecting leader
			 * pattern
			 */
			newval = CFG_ERR_STATUS2VI_MASK_VC3 |
			         CFG_ERR_STATUS2VI_MASK_VC2 |
			         CFG_ERR_STATUS2VI_MASK_VC1 |
			         CFG_ERR_STATUS2VI_MASK_VC0;
			newval = enable ? newval : enable;
			host1x_writel(mc_csi->pdev,
				      NVCSI_STREAM_0_ERROR_STATUS2VI_MASK +
				      NVCSI_PHY_OFFSET * phy_num,
				      newval);
		}
		if (cilb_io_lanes) {
			newval = ((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO0)
				!= 0 ?
				intr_dphy_cil_deskew_calib_done_lane0 : 0) |
				((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO1)
				!= 0 ?
				intr_dphy_cil_deskew_calib_done_lane1 : 0) |
				intr_dphy_cil_deskew_calib_done_ctrl;
			newval = enable ? newval : enable;
			/* ONLY handles deskew done interrupt, also
			 * turn off any other interrupt for NVCSI
			 * because ISR does not handle them*/
			host1x_writel(mc_csi->pdev,
				     NVCSI_PHY_0_CILB_INTR_MASK +
				     NVCSI_PHY_OFFSET * phy_num,
				     ~newval);
			dev_dbg(mc_csi->dev, "addr %x prev %x mask %x new %x\n",
				NVCSI_PHY_0_CILB_INTR_MASK +
				NVCSI_PHY_OFFSET * phy_num, val, newval,
				~newval);
			/* Disable single bit err when detecting leader
			 * pattern
			 */
			newval = CFG_ERR_STATUS2VI_MASK_VC3 |
			         CFG_ERR_STATUS2VI_MASK_VC2 |
			         CFG_ERR_STATUS2VI_MASK_VC1 |
			         CFG_ERR_STATUS2VI_MASK_VC0;
			newval = enable ? newval : enable;
			host1x_writel(mc_csi->pdev,
				      NVCSI_STREAM_1_ERROR_STATUS2VI_MASK +
				      NVCSI_PHY_OFFSET * phy_num,
				      newval);
		}
		phy_num++;
	}
}
static irqreturn_t nvcsi_thread_isr(int irq, void *arg)
{
	unsigned int i, vala, valb;

	for (i = 0; i < NVCSI_PHY_NUM_BRICKS; i++) {
		vala = host1x_readl(mc_csi->pdev,
			NVCSI_PHY_0_CILA_INTR_STATUS + NVCSI_PHY_OFFSET * i);
		host1x_writel(mc_csi->pdev,
			NVCSI_PHY_0_CILA_INTR_STATUS + NVCSI_PHY_OFFSET * i,
			vala);
		valb = host1x_readl(mc_csi->pdev,
			NVCSI_PHY_0_CILB_INTR_STATUS + NVCSI_PHY_OFFSET * i);
		host1x_writel(mc_csi->pdev,
			NVCSI_PHY_0_CILB_INTR_STATUS + NVCSI_PHY_OFFSET * i,
			valb);

		int_lanes |= (vala & intr_dphy_cil_deskew_calib_done_lane0) ?
			    NVCSI_PHY_0_NVCSI_CIL_A_IO0 << (i * 4) : 0;
		int_lanes |= (vala & intr_dphy_cil_deskew_calib_done_lane1) ?
			    NVCSI_PHY_0_NVCSI_CIL_A_IO1 << (i * 4) : 0;
		int_lanes |= (valb & intr_dphy_cil_deskew_calib_done_lane0) ?
			    NVCSI_PHY_0_NVCSI_CIL_B_IO0 << (i * 4) : 0;
		int_lanes |= (valb & intr_dphy_cil_deskew_calib_done_lane1) ?
			    NVCSI_PHY_0_NVCSI_CIL_B_IO1 << (i * 4) : 0;
		dev_dbg(mc_csi->dev, "inta %x intb %x", vala, valb);
	}
	deskew_active = nvcsi_deskew_apply_helper(int_lanes);
	return IRQ_HANDLED;
}


int nvcsi_finalize_poweron(struct platform_device *pdev)
{
	struct nvcsi *nvcsi = nvhost_get_private_data(pdev);
	int ret;

	if (nvcsi->regulator) {
		ret = regulator_enable(nvcsi->regulator);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable csi regulator failed.");
			return ret;
		}
	}

	return 0;
}

int nvcsi_prepare_poweroff(struct platform_device *pdev)
{
	struct nvcsi *nvcsi = nvhost_get_private_data(pdev);
	int ret;

	if (nvcsi->regulator) {
		ret = regulator_disable(nvcsi->regulator);
		if (ret)
			dev_err(&pdev->dev, "failed to disabled csi regulator failed.");
	}

	return 0;
}

static int nvcsi_probe_regulator(struct nvcsi *nvcsi)
{
	struct device *dev = &nvcsi->pdev->dev;
	struct regulator *regulator;
	const char *regulator_name;
	int err;

	err = of_property_read_string(dev->of_node, "nvidia,csi_regulator",
				      &regulator_name);
	if (err)
		return err;

	regulator = devm_regulator_get(dev, regulator_name);
	if (IS_ERR(regulator))
		return PTR_ERR(regulator);

	nvcsi->regulator = regulator;

	return 0;
}

static int nvcsi_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;
	struct nvcsi *nvcsi = NULL;
	struct tegra_csi_data *data = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvcsi_of_match, &dev->dev);
		if (match) {
			data = (struct tegra_csi_data *) match->data;
			pdata = (struct nvhost_device_data *)data->info;
		}
	} else {
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;
	}

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		err = -ENODATA;
		goto err_get_pdata;
	}

	nvcsi = devm_kzalloc(&dev->dev, sizeof(*nvcsi), GFP_KERNEL);
	if (!nvcsi) {
		err = -ENOMEM;
		goto err_alloc_nvcsi;
	}

	nvcsi->pdev = dev;
	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);
	pdata->private_data = nvcsi;
	mc_csi = &nvcsi->csi;
	mutex_init(&nvcsi->deskew_lock);
	err = nvcsi_probe_regulator(nvcsi);
	if (err)
		dev_info(&dev->dev, "failed to get regulator (%d)\n", err);

	nvcsi->irq = platform_get_irq(dev, 0);
	if (nvcsi->irq < 0) {
		dev_err(&dev->dev, "No IRQ available\n");
		goto err_get_resources;
	} else {
		err = devm_request_threaded_irq(&dev->dev, nvcsi->irq,
				NULL, nvcsi_thread_isr, IRQF_ONESHOT,
				dev_name(&dev->dev), dev);
		if (err) {
			dev_err(&dev->dev, "Cannot setup irq\n");
			nvcsi->irq = -ENXIO;
			goto err_get_resources;
		} else
			disable_irq(nvcsi->irq);
	}

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto err_get_resources;

	err = nvhost_module_init(dev);
	if (err)
		goto err_module_init;

	err = nvhost_client_device_init(dev);
	if (err)
		goto err_client_device_init;

	if (data)
		nvcsi->csi.fops = data->csi_fops;

	err = tegra_csi_media_controller_init(&nvcsi->csi, dev);
	if (err < 0)
		goto err_mediacontroller_init;

	nvcsi_deskew_debugfs_init(nvcsi);
	return 0;

err_mediacontroller_init:
err_client_device_init:
	nvhost_module_deinit(dev);
err_module_init:
err_get_resources:
err_alloc_nvcsi:
err_get_pdata:

	return err;
}

static int __exit nvcsi_remove(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct nvcsi *nvcsi = (struct nvcsi *)pdata->private_data;

	mc_csi = NULL;
	nvcsi_deskew_debugfs_remove(nvcsi);
	tegra_csi_media_controller_remove(&nvcsi->csi);

	return 0;
}

static struct platform_driver nvcsi_driver = {
	.probe = nvcsi_probe,
	.remove = __exit_p(nvcsi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvcsi",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvcsi_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

#ifdef CONFIG_PM_GENERIC_DOMAINS
static struct of_device_id tegra_nvcsi_domain_match[] = {
	{.compatible = "nvidia,tegra186-ve-pd",
	.data = (struct nvhost_device_data *)&t18_nvcsi_info},
	{},
};
#endif
static inline void nvcsi_phy_write(unsigned int phy_num,
				   unsigned int addr_offset, unsigned int val)
{
	unsigned int addr;

	addr = NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0 + NVCSI_PHY_OFFSET * phy_num
		+addr_offset;
	dev_dbg(mc_csi->dev, "%s: addr %x val %x\n", __func__, addr,
			val);
	host1x_writel(mc_csi->pdev, addr, val);
}
static inline unsigned int nvcsi_phy_readl(unsigned int phy_num,
					   unsigned int addr_offset)
{
	unsigned int addr;
	int val;

	addr = NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0 + NVCSI_PHY_OFFSET * phy_num
		+ addr_offset;
	val = host1x_readl(mc_csi->pdev, addr);
	dev_dbg(mc_csi->dev, "%s: addr %x val %x\n", __func__, addr,
			val);
	return val;
}
int nvcsi_deskew_setup(unsigned int active_lanes)
{
	unsigned int phy_num = 0;
	unsigned int cil_lanes = 0, cila_io_lanes = 0, cilb_io_lanes = 0;
	unsigned int remaining_lanes = active_lanes;
	unsigned int val = 0, newval = 0;

	if (active_lanes >> NVCSI_PHY_CIL_NUM_LANE) {
		dev_err(mc_csi->dev, "%s Invalid active lanes\n", __func__);
		return -EINVAL;
	}

	dev_dbg(mc_csi->dev, "%s: active_lanes: %x\n", __func__, active_lanes);
	while (remaining_lanes) {
		cil_lanes = (active_lanes & (0x000f << (phy_num * 4)))
				>> (phy_num * 4);
		cila_io_lanes =  cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_A_IO0
			| NVCSI_PHY_0_NVCSI_CIL_A_IO1);
		cilb_io_lanes = cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_B_IO0
			| NVCSI_PHY_0_NVCSI_CIL_B_IO1);
		remaining_lanes &= ~(0xf << (phy_num * 4));
		if (cila_io_lanes) {
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_CONTROL_0_OFFSET);
			val = (val & (~(DESKEW_COMPARE | DESKEW_SETTLE |
					CLK_SETTLE | THS_SETTLE)))
				| (0x4 << DESKEW_COMPARE_SHIFT)
				| (0X6 << DESKEW_SETTLE_SHIFT)
				| (0x19 << CLK_SETTLE_SHIFT)
				| (0x16 << THS_SETTLE_SHIFT);
			nvcsi_phy_write(phy_num, NVCSI_CIL_A_CONTROL_0_OFFSET,
					val);
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET);
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET,
					val | CLK_INADJ_SWEEP_CTRL);
			val = nvcsi_phy_readl(phy_num,
					NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET);
			newval = ((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO0)
				!= 0 ? DATA_INADJ_SWEEP_CTRL0 : 0) |
				((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO1)
				!= 0 ? DATA_INADJ_SWEEP_CTRL1 : 0);
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET,
					((val & ~(DATA_INADJ_SWEEP_CTRL0 |
						DATA_INADJ_SWEEP_CTRL1)) |
					newval));
		}
		if (cilb_io_lanes) {
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_CONTROL_0_OFFSET);
			val = (val & (~(DESKEW_COMPARE | DESKEW_SETTLE |
					CLK_SETTLE | THS_SETTLE)))
				| (0x4 << DESKEW_COMPARE_SHIFT)
				| (0X6 << DESKEW_SETTLE_SHIFT)
				| (0x19 << CLK_SETTLE_SHIFT)
				| (0x16 << THS_SETTLE_SHIFT);
			nvcsi_phy_write(phy_num, NVCSI_CIL_B_CONTROL_0_OFFSET,
					val);
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET);
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET,
					val | CLK_INADJ_SWEEP_CTRL);
			val = nvcsi_phy_readl(phy_num,
					NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET);
			newval = ((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO0)
				!= 0 ? DATA_INADJ_SWEEP_CTRL0 : 0) |
				((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO1)
				!= 0 ? DATA_INADJ_SWEEP_CTRL1 : 0);
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET,
					((val & ~(DATA_INADJ_SWEEP_CTRL0 |
						DATA_INADJ_SWEEP_CTRL1)) |
					newval));
		}
		phy_num++;
	}
	return 0;
}
EXPORT_SYMBOL(nvcsi_deskew_setup);

static inline unsigned int checkpass(unsigned long long stat)
{
	/* Due to bug 200098288, use
	 * mask101 = 0x5; mask111 = 0x7; mask1010 = 0xa;
	 * to check if current trimmer setting results in passing
	 * Algorithm is explained in NVCSI_CIL_IAS chapter 5.3
	 */

	return ((stat & 0x5) == 0x5 || ((stat & 0x7) == 0x7)
		|| (stat & 0xa) == 0xa);
}

/* compute_boundary:
 * This function find the flipping point when the trimmer settings starts
 * to pass/fail.
 * Each graph represent the 64-bit status,trimmer setting 0~0x3F
 * from Right to Left.
 *
 * pf: pass to fail, fp: fail to pass
 *
 *   pf     fp
 * __|------|__
 *     pf     fp = 0
 * ____|-------
 * pf=0x3f   fp
 * ----------|__
 */
static unsigned int compute_boundary(unsigned long long stat, unsigned int *x,
				     unsigned int *w)
{
	unsigned int was_pass, i = 0;
	int pf = -1, fp = -1;
	unsigned long long last_stat;

	was_pass = checkpass(stat);
	fp = (was_pass == 1 ? 0 : -1);
	last_stat = stat;

	while (i < 64) {
		if ((was_pass == 1) && (!(last_stat & 1))) {
			if (!checkpass(last_stat)) {
				pf = i;
				was_pass = 0;
				dev_dbg(mc_csi->dev, "pf %d\n", pf);
			}
		} else if ((was_pass == 0) && (last_stat & 1)) {
			if (checkpass(last_stat)) {
				fp = i;
				was_pass = 1;
				dev_dbg(mc_csi->dev, "fp %d\n", fp);
		       }
		}
		i++;
		last_stat >>= 1;
	}

	dev_dbg(mc_csi->dev, "fp %d pf %d\n", fp, pf);
	if (fp == -1 && pf == -1) {
		dev_dbg(mc_csi->dev, "No passing record found, please retry\n");
		return -EINVAL;
	} else if (pf == -1 && was_pass == 1)
		pf = 0x3f;

	*x = pf;
	*w = fp;
	dev_dbg(mc_csi->dev, "%s: stats %llx, f2p %d, p2f %d",
		__func__, stat, fp, pf);

	return 0;
}
static unsigned int error_boundary(unsigned int phy_num, unsigned int cil_bit,
			    unsigned int *x, unsigned int *w,
			    unsigned int *y, unsigned int *z)
{
	unsigned int stats_low = 0, stats_high = 0, stats_offset = 0;
	unsigned long long result = 0;
	unsigned int is_cilb = 0, is_io1 = 0;

	is_cilb = (cil_bit > 1);
	is_io1 = (cil_bit % 2);
	stats_offset = is_cilb * NVCSI_CIL_B_OFFSET + is_io1 * 8;
	/* step #1 clk lane */
	stats_low = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_0_0_OFFSET);
	stats_high = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_0_0_OFFSET);
	result = ((unsigned long long)stats_high) << 32 | stats_low;

	debugfs_deskew_clk_stats_low[cil_bit + phy_num * 4] = stats_low;
	debugfs_deskew_clk_stats_high[cil_bit + phy_num * 4] = stats_high;

	if (compute_boundary(result, x, w))
		return -EINVAL;
	/* step #2 data lane */
	stats_low = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_0_0_OFFSET);
	stats_high = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_0_0_OFFSET);
	result = ((unsigned long long)stats_high) << 32 | stats_low;

	debugfs_deskew_data_stats_low[cil_bit + phy_num * 4] = stats_low;
	debugfs_deskew_data_stats_high[cil_bit + phy_num * 4] = stats_high;

	if (compute_boundary(result, y, z))
		return -EINVAL;

	return 0;
}
static void compute_trimmer(unsigned int *x, unsigned int *w,
			    unsigned int *y, unsigned int *z,
			    unsigned int *d, unsigned int *c)
{
	int mid[4], base = 0;
	unsigned int i = 0;


	/* NVCSI_CIL_IAS Chapter 5.3 */
	for (i = 0; i < 4; i++) {
		if (w[i] < z[i]) {
			y[i] = 0;
			z[i] = 0;
		} else if (w[i] > z[i]) {
			x[i] = 0;
			w[i] = 0;
		}
		mid[i] = ((y[i] + z[i]) - (x[i] + w[i])) >> 1;
		base = mid[i] < base ? mid[i] : base;
	}
	*c = -base;
	for (i = 0; i < 4; i++)
		d[i] = mid[i] - base;

	/* debug prints */
	for (i = 0; i < 4; i++)
		dev_dbg(mc_csi->dev, "x %u w %u y %u z %u d %u\n",
				x[i], w[i], y[i], z[i], d[i]);
	dev_dbg(mc_csi->dev, "clk %u\n", *c);
}
static void set_trimmer(unsigned int phy_num, unsigned int cila,
			unsigned int cilb,
			unsigned int *d, unsigned int c)
{
	unsigned int val = 0, val1 = 0;

	if (cila && cilb) {
		/* 4-lane */
		val = SW_SET_DPHY_INADJ_CLK |
		      SW_SET_DPHY_INADJ_IO0 |
		      SW_SET_DPHY_INADJ_IO1 |
		      (c << DPHY_INADJ_CLK_SHIFT) |
		      (d[PHY_0_CIL_A_IO0] << DPHY_INADJ_IO0_SHIFT) |
		      (d[PHY_0_CIL_A_IO1] << DPHY_INADJ_IO1_SHIFT);
		val1 = SW_SET_DPHY_INADJ_IO0 |
		       SW_SET_DPHY_INADJ_IO1 |
		       (d[PHY_0_CIL_B_IO0] << DPHY_INADJ_IO0_SHIFT)|
		       (d[PHY_0_CIL_B_IO1] << DPHY_INADJ_IO1_SHIFT);
		nvcsi_phy_write(phy_num, NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET,
				val);
		nvcsi_phy_write(phy_num, NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET,
				val1);
		dev_dbg(mc_csi->dev, "cila %x cilb %x\n", val, val1);
		return;
	}
	/* TODO:
	 * 2-lane and 1-lane cases cannot be verified since there
	 * is no such sensor supported yet
	 */
	if (cila) {
		/* 2-lane and 1-lane*/
		val1 = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET);
		if (cila & NVCSI_PHY_0_NVCSI_CIL_A_IO0) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO0 | DPHY_INADJ_IO0);
			val |= SW_SET_DPHY_INADJ_IO0 |
			      (d[PHY_0_CIL_A_IO0] << DPHY_INADJ_IO0_SHIFT);

		}
		if (cila & NVCSI_PHY_0_NVCSI_CIL_A_IO1) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO1 | DPHY_INADJ_IO1);
			val |= SW_SET_DPHY_INADJ_IO1 |
			      (d[PHY_0_CIL_A_IO1] << DPHY_INADJ_IO1_SHIFT);
		}
		val1 &= ~(SW_SET_DPHY_INADJ_CLK | DPHY_INADJ_CLK);
		val |= SW_SET_DPHY_INADJ_CLK | (c << DPHY_INADJ_CLK_SHIFT);

		nvcsi_phy_write(phy_num, NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET,
				val | val1);
		dev_dbg(mc_csi->dev, "cila %x\n", val | val1);
	} else {
		/* 2-lane and 1-lane*/
		val1 = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET);
		if (cilb & NVCSI_PHY_0_NVCSI_CIL_B_IO0) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO0 | DPHY_INADJ_IO0);
			val |= SW_SET_DPHY_INADJ_IO0 |
				(d[PHY_0_CIL_B_IO0] << DPHY_INADJ_IO0_SHIFT);
		}
		if (cilb & NVCSI_PHY_0_NVCSI_CIL_B_IO1) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO1 | DPHY_INADJ_IO1);
			val |= SW_SET_DPHY_INADJ_IO1 |
				(d[PHY_0_CIL_B_IO1] << DPHY_INADJ_IO1_SHIFT);
		}
		val1 &= ~(SW_SET_DPHY_INADJ_CLK | DPHY_INADJ_CLK);
		val |= SW_SET_DPHY_INADJ_CLK | (c << DPHY_INADJ_CLK_SHIFT);
		nvcsi_phy_write(phy_num, NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET,
				val | val1);
		dev_dbg(mc_csi->dev, "cilb %x\n", val | val1);
	}
}
int nvcsi_deskew_apply_check(unsigned int active_lanes)
{
	unsigned long timeout;
	struct nvcsi *nvcsi = nvhost_get_private_data(mc_csi->pdev);

	dev_dbg(mc_csi->dev, "%s: active_lanes: %x\n", __func__, active_lanes);
	mutex_lock(&nvcsi->deskew_lock);
	deskew_active = 1;
	enable_int_mask(active_lanes, 1);
	enable_irq(nvcsi->irq);
	timeout = jiffies + usecs_to_jiffies(DESKEW_TIMEOUT_USEC);
	while (time_before(jiffies, timeout)) {
		if (deskew_active != 1) {
			disable_irq(nvcsi->irq);
			enable_int_mask(active_lanes, 0);
			dev_dbg(mc_csi->dev, "%s:deskew applied on %x ret %d\n",
					__func__, int_lanes, deskew_active);
			mutex_unlock(&nvcsi->deskew_lock);
			return ((int_lanes & active_lanes) == active_lanes) ?
				deskew_active : -EINVAL;
		}
		usleep_range(10, 50);
	}
	disable_irq(nvcsi->irq);
	enable_int_mask(active_lanes, 0);
	mutex_unlock(&nvcsi->deskew_lock);
	dev_info(mc_csi->dev, "Deskew timeout\n");
	return -ETIMEDOUT;
}
EXPORT_SYMBOL(nvcsi_deskew_apply_check);

static int nvcsi_deskew_apply_helper(unsigned int active_lanes)
{
	unsigned int phy_num = -1;
	unsigned int cil_lanes = 0, cila_io_lanes = 0, cilb_io_lanes = 0;
	unsigned int remaining_lanes = active_lanes;
	unsigned int val = 0;
	unsigned int i, j;

	dev_dbg(mc_csi->dev, "%s: interrupt lane: %x\n",
		__func__, active_lanes);
	while (remaining_lanes) {
		unsigned int x[4] = {0, 0, 0, 0};
		unsigned int w[4] = {0, 0, 0, 0};
		unsigned int y[4] = {0, 0, 0, 0};
		unsigned int z[4] = {0, 0, 0, 0};
		unsigned int d_trimmer[4] = {0, 0, 0, 0};
		unsigned int clk_trimmer = 0;

		phy_num++;
		cil_lanes = (active_lanes & (0xf << (phy_num * 4)))
				>> (phy_num * 4);
		remaining_lanes &= ~(0xf << (phy_num * 4));
		if (!cil_lanes)
			continue;
		cila_io_lanes =  cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_A_IO0
			| NVCSI_PHY_0_NVCSI_CIL_A_IO1);
		cilb_io_lanes = cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_B_IO0
			| NVCSI_PHY_0_NVCSI_CIL_B_IO1);
		if (cila_io_lanes) {
			/* Step 0 : check if deskew has error */
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_DPHY_DESKEW_STATUS_0_OFFSET);
			/* If there is error in DESKEW_STATUS, user
			 * should request sensor to re-send deskew sequence
			 */
			if (val & DPHY_CALIB_ERR_IO1 ||
			    val & DPHY_CALIB_ERR_IO0) {
				dev_err(mc_csi->dev, "deskew calib status err");
				return -EINVAL;
			}

			if (!((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO0) &&
			    (val & DPHY_CALIB_DONE_IO0)))
				return -EINVAL;
			if (!((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO1) &&
			    (val & DPHY_CALIB_DONE_IO1)))
				return -EINVAL;
		}
		if (cilb_io_lanes) {
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_DPHY_DESKEW_STATUS_0_OFFSET);
			/* If there is error in DESKEW_STATUS, user
			 * should request sensor to re-send deskew sequence
			 */
			if (val & DPHY_CALIB_ERR_IO1 ||
			    val & DPHY_CALIB_ERR_IO0) {
				dev_err(mc_csi->dev, "deskew calib status err");
				return -EINVAL;
			}

			if (!((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO0) &&
			    (val & DPHY_CALIB_DONE_IO0)))
				return -EINVAL;
			if (!((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO1) &&
			    (val & DPHY_CALIB_DONE_IO1)))
				return -EINVAL;
		}
		/* Step 1: Read status registers and compute error boundaries */
		for (i = NVCSI_PHY_0_NVCSI_CIL_A_IO0, j = 0;
		     i <= NVCSI_PHY_0_NVCSI_CIL_B_IO1;
		     i <<= 1, j++) {
			if ((cil_lanes & i) == 0)
				continue;
			if (error_boundary(phy_num, j,
						&x[j], &w[j],
						&y[j], &z[j]))
				return -EINVAL;
		}
		/*step 2: compute trimmer value based on error boundaries */
		compute_trimmer(x, w, y, z, d_trimmer, &clk_trimmer);
		/*step 3: Apply trimmer settings */
		set_trimmer(phy_num, cila_io_lanes, cilb_io_lanes,
				d_trimmer, clk_trimmer);
	}
	return 0;
}

static int dbgfs_deskew_stats(struct seq_file *s, void *data)
{
	unsigned int i = 0;

	seq_puts(s, "clk stats\n");
	for (i = 0; i < NVCSI_PHY_CIL_NUM_LANE; i++) {
		seq_printf(s, "%08x %08x\n",
			debugfs_deskew_clk_stats_high[i],
			debugfs_deskew_clk_stats_low[i]);
	}
	seq_puts(s, "data stats\n");
	for (i = 0; i < NVCSI_PHY_CIL_NUM_LANE; i++) {
		seq_printf(s, "%08x %08x\n",
				debugfs_deskew_data_stats_high[i],
				debugfs_deskew_data_stats_low[i]);
	}
	return 0;
}
static int dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbgfs_deskew_stats, inode->i_private);
}
static const struct file_operations dbg_show_ops = {
	.open		= dbgfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static int dbgfs_calc_bound(struct seq_file *s, void *data)
{
	unsigned int x, w;

	seq_printf(s, "input: %llx\n", input_stats);
	compute_boundary(input_stats, &x, &w);
	seq_printf(s, "setting: x %u w %u\n", x, w);

	return 0;
}
static int dbg_calc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbgfs_calc_bound, inode->i_private);
}
static const struct file_operations dbg_calc_ops = {
	.open		= dbg_calc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release
};

static void nvcsi_deskew_debugfs_remove(struct nvcsi *nvcsi)
{
	debugfs_remove_recursive(nvcsi->dir);
}
static int nvcsi_deskew_debugfs_init(struct nvcsi *nvcsi)
{
	struct dentry *val;

	nvcsi->dir = debugfs_create_dir("deskew", NULL);
	if (!nvcsi->dir)
		return -ENOMEM;

	val = debugfs_create_file("stats", S_IRUGO, nvcsi->dir, mc_csi,
				&dbg_show_ops);
	if (!val)
		goto err_debugfs;

	val = debugfs_create_x64("input_status", S_IRUGO | S_IWUSR,
				nvcsi->dir, &input_stats);
	if (!val)
		goto err_debugfs;
	val = debugfs_create_file("calc_bound", S_IRUGO | S_IWUSR,
				nvcsi->dir, mc_csi, &dbg_calc_ops);
	if (!val)
		goto err_debugfs;
	return 0;
err_debugfs:
	dev_err(mc_csi->dev, "Fail to create debugfs\n");
	debugfs_remove_recursive(nvcsi->dir);
	return -ENOMEM;
}
static long nvcsi_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct nvcsi_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;
	int ret;

	switch (cmd) {
	case NVHOST_NVCSI_IOCTL_SET_NVCSI_CLK: {
		long rate;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (get_user(rate, (long __user *)arg))
			return -EFAULT;

		return nvhost_module_set_rate(pdev, priv, rate, 0,
						NVHOST_CLOCK);
		}
	case NVHOST_NVCSI_IOCTL_DESKEW_SETUP: {
		unsigned int active_lanes;

		dev_dbg(mc_csi->dev, "ioctl: deskew_setup\n");
		priv->active_lanes = get_user(active_lanes,
				(long __user *)arg);
		ret = nvcsi_deskew_setup(priv->active_lanes);
		return ret;
		}
	case NVHOST_NVCSI_IOCTL_DESKEW_APPLY: {
		dev_dbg(mc_csi->dev, "ioctl: deskew_apply\n");
		ret = nvcsi_deskew_apply_check(priv->active_lanes);
		return ret;
		}
	}
	return -ENOIOCTLCMD;
}

static int nvcsi_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct nvcsi_private *priv;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL))
		return -ENOMEM;

	priv->pdev = pdev;

	if (nvhost_module_add_client(pdev, priv)) {
		kfree(priv);
		return -ENOMEM;
	}
	file->private_data = priv;
	priv->active_lanes = 0;
	return nonseekable_open(inode, file);
}

static int nvcsi_release(struct inode *inode, struct file *file)
{
	struct nvcsi_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;

	nvhost_module_remove_client(pdev, priv);
	kfree(priv);
	return 0;
}

const struct file_operations tegra_nvcsi_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvcsi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvcsi_ioctl,
#endif
	.open = nvcsi_open,
	.release = nvcsi_release,
};

static int __init nvcsi_init(void)
{

#ifdef CONFIG_PM_GENERIC_DOMAINS
	int ret;

	ret = nvhost_domain_init(tegra_nvcsi_domain_match);
	if (ret)
		return ret;
#endif

	return platform_driver_register(&nvcsi_driver);
}

static void __exit nvcsi_exit(void)
{
	platform_driver_unregister(&nvcsi_driver);
}

late_initcall(nvcsi_init);
module_exit(nvcsi_exit);
