/*
 * Tegra CSI4 device common APIs
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/clk/tegra.h>
#include "nvhost_acm.h"
#include "camera/csi/csi.h"
#include "camera/vi/t18x_registers.h"

static void csi4_write(struct tegra_csi_channel *chan,
		unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;

	writel(val, csi->iomem_base + addr);
}

static u32 csi4_read(struct tegra_csi_channel *chan,
		unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;

	return readl(csi->iomem_base + addr);
}

static void csi4_stream_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;
	u32 cilb_offset = (index & 0x1) ? 0x800 : 0x0;

	writel(val, csi->iomem[index >> 1] + cilb_offset + addr);
}

static u32 csi4_stream_read(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;
	u32 cilb_offset = (index & 0x1) ? 0x800 : 0x0;

	return readl(csi->iomem[index >> 1] + cilb_offset + addr);
}

static void csi4_phy_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;

	writel(val, csi->iomem_base  + 0x18000 + (0x10000 * index) + addr);
}

static u32 csi4_phy_read(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;

	return readl(csi->iomem_base + 0x18000 + (0x10000 * index) + addr);
}

static bool csi4_check_reg(struct tegra_csi_channel *chan)
{
	struct tegra_csi_device *csi = chan->csi;
	bool ret = true;

	dev_dbg(csi->dev, "%s\n", __func__);
	csi4_write(chan, CFG_NVCSI_INCR_SYNCPT_CNTRL, 0x101);
	if (csi4_read(chan, CFG_NVCSI_INCR_SYNCPT_CNTRL) != 0x101)
		ret = false;
	dev_dbg(csi->dev, "checkCamReg::Check %s!\n",
			ret ? "pass" : "false");

	csi4_write(chan, CFG_NVCSI_INCR_SYNCPT_CNTRL, 0x0);
	if (csi4_read(chan, CFG_NVCSI_INCR_SYNCPT_CNTRL) != 0x0)
		ret = false;
	csi4_write(chan, CFG_NVCSI_INCR_SYNCPT_CNTRL, 0x0);
	dev_dbg(csi->dev, "checkCamReg::Check %s!\n",
			ret ? "pass" : "false");

	return ret;
}

static bool csi4_init(struct tegra_csi_channel *chan)
{
	struct tegra_csi_device *csi = chan->csi;
	int i;

	dev_dbg(csi->dev, "%s\n", __func__);

	if (!csi4_check_reg(chan))
		return false;

	for (i = 0; i < CSI_PORTS; i++) {
		csi4_stream_write(chan, i, CIL_INTR_STATUS, 0xffffffff);
		csi4_stream_write(chan, i, CIL_ERR_INTR_STATUS, 0xffffffff);
	}
	for (i = 0; i < CSI_PORTS; i++) {
		csi4_stream_write(chan, i, CIL_INTR_MASK, 0xffffffff);
		csi4_stream_write(chan, i, CIL_ERR_INTR_MASK, 0xffffffff);
	}

	for (i = 0; i < CSI_PORTS; i++) {
		csi4_stream_write(chan, i, INTR_STATUS, 0x3ffff);
		csi4_stream_write(chan, i, ERR_INTR_STATUS, 0x7ffff);
	}
	for (i = 0; i < CSI_PORTS; i++) {
		csi4_stream_write(chan, i, ERROR_STATUS2VI_MASK, 0x0);
		csi4_stream_write(chan, i, INTR_MASK, 0x0);
		csi4_stream_write(chan, i, ERR_INTR_MASK, 0x0);
	}

	return true;
}

static bool csi4_stream_config(struct tegra_csi_channel *chan, int port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int val;

	dev_dbg(csi->dev, "%s\n", __func__);

	csi4_stream_write(chan, port_num, PPFSM_TIMEOUT_CTRL, 0);
	csi4_stream_write(chan, port_num, PH_CHK_CTRL,
			CFG_PH_CRC_CHK_EN | CFG_PH_ECC_CHK_EN);
	csi4_stream_write(chan, port_num, VC0_DPCM_CTRL, 0);
	csi4_stream_write(chan, port_num, VC0_DT_OVERRIDE, 0);

	val = csi4_stream_read(chan, port_num, VC0_DPCM_CTRL);
	dev_dbg(csi->dev, "%s (%d) read VC0_DPCM_CTRL = %08x\n",
			__func__, port_num, val);

	return true;
}

static bool csi4_phy_config(struct tegra_csi_channel *chan, int phy_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int val;

	dev_dbg(csi->dev, "%s\n", __func__);

	csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET,
			SW_RESET1_EN | SW_RESET0_EN);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET,
			SW_RESET1_EN | SW_RESET0_EN);

	csi4_phy_write(chan, phy_num, NVCSI_CIL_PHY_CTRL, 0);
	val = csi4_phy_read(chan, phy_num, NVCSI_CIL_CONFIG);
	dev_dbg(csi->dev, "csi_phy 0 read NVCSI_CIL_CONFIG val = %08x\n", val);

	csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG, chan->numlanes);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_PAD_CONFIG, 0);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_A_PAD_CONFIG, 0x700011);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_B_PAD_CONFIG, 0x700011);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_A_CONTROL, 0x110a14);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_B_CONTROL, 0x110a14);

	csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET, 0x0);
	csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET, 0x0);
	return true;
}

static bool csi4_stream_check_status(struct tegra_csi_channel *chan)
{
	struct tegra_csi_device *csi = chan->csi;
	int err = 0, i;

	dev_dbg(csi->dev, "%s\n", __func__);
	for (i = 0; i < CSI_PORTS; i++) {
		err = csi4_stream_read(chan, i, ERROR_STATUS2VI_VC0);
		if (err)
			dev_err(csi->dev,
					"%s (%d) ERROR_STATUS2VI_VC0 = 0x%08x\n",
					__func__, i, err);

		err = csi4_stream_read(chan, i, ERROR_STATUS2VI_VC1);
		if (err)
			dev_err(csi->dev,
					"%s (%d) ERROR_STATUS2VI_VC1 = 0x%08x\n",
					__func__, i, err);

		err = csi4_stream_read(chan, i, ERROR_STATUS2VI_VC2);
		if (err)
			dev_err(csi->dev,
					"%s (%d) ERROR_STATUS2VI_VC2 = 0x%08x\n",
					__func__, i, err);

		err = csi4_stream_read(chan, i, ERROR_STATUS2VI_VC3);
		if (err)
			dev_err(csi->dev,
					"%s (%d) ERROR_STATUS2VI_VC2 = 0x%08x\n",
					__func__, i, err);

		err = csi4_stream_read(chan, i, INTR_STATUS);
		if (err)
			dev_err(csi->dev,
					"%s (%d) INTR_STATUS 0x%08x\n",
					__func__, i, err);

		err = csi4_stream_read(chan, i, ERR_INTR_STATUS);
		if (err)
			dev_err(csi->dev,
					"%s (%d) ERR_INTR_STATUS 0x%08x\n",
					__func__, i, err);
	}
	return false;
}

static bool csi4_cil_check_status(struct tegra_csi_channel *chan)
{
	struct tegra_csi_device *csi = chan->csi;
	int err = 0, i;

	dev_dbg(csi->dev, "%s %d\n", __func__, __LINE__);
	for (i = 0; i < CSI_PORTS; i++) {
		err = csi4_stream_read(chan, i, CIL_INTR_STATUS);
		if (err)
			dev_err(csi->dev,
					"%s (%d) CIL_INTR_STATUS 0x%08x\n",
					__func__, i, err);
		err = csi4_stream_read(chan, i, CIL_ERR_INTR_STATUS);
		if (err)
			dev_err(csi->dev,
					"%s (%d) CIL_ERR_INTR_STATUS 0x%08x\n",
					__func__, i, err);
	}
	return false;
}


int csi4_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:nvhost module is busy\n", __func__);

	return err;
}

int csi4_power_off(struct tegra_csi_device *csi)
{
	nvhost_module_idle(csi->pdev);

	return 0;
}

void csi4_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	dev_dbg(csi->dev, "%s port_num=%d, lanes=%d\n",
			__func__, port_num, chan->numlanes);

	/* TODO - add fops for iomem setting */
	csi->iomem[0] = csi->iomem_base + TEGRA_CSI_STREAM_0_BASE;
	csi->iomem[1] = csi->iomem_base + TEGRA_CSI_STREAM_2_BASE;
	csi->iomem[2] = csi->iomem_base + TEGRA_CSI_STREAM_4_BASE;

	csi4_init(chan);
	csi4_stream_config(chan, port_num);
	csi4_phy_config(chan, (port_num & 0x6) >> 1);
	csi4_stream_write(chan, port_num, PP_EN_CTRL, CFG_PP_EN);
}

void csi4_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	csi4_stream_check_status(chan);
	csi4_cil_check_status(chan);
}
