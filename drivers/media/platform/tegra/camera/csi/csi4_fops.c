/*
 * Tegra CSI4 device common APIs
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk/tegra.h>
#include <linux/of_graph.h>
#include <linux/string.h>

#include <media/csi.h>
#include <media/csi4_registers.h>
#include <media/tegra_camera_core.h>
#include <media/mc_common.h>

#include "linux/nvhost_ioctl.h"
#include "mipical/mipi_cal.h"
#include "nvcsi/nvcsi.h"
#include "nvhost_acm.h"

#include "csi4_fops.h"

#define DEFAULT_TPG_FREQ	102000000

static void csi4_stream_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;
	u32 cilb_offset = (index & 0x1) ? CSI4_STREAM_OFFSET : 0x0;

	writel(val, csi->iomem[index >> 1] + cilb_offset + addr);
}

static u32 csi4_stream_read(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;
	u32 cilb_offset = (index & 0x1) ? CSI4_STREAM_OFFSET : 0x0;

	return readl(csi->iomem[index >> 1] + cilb_offset + addr);
}

static void csi4_phy_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;

	writel(val, csi->iomem_base +
		CSI4_BASE_ADDRESS + (CSI4_PHY_OFFSET * index) + addr);
}

static u32 csi4_phy_read(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;

	return readl(csi->iomem_base +
		CSI4_BASE_ADDRESS + (CSI4_PHY_OFFSET * index) + addr);
}

static void csi4_stream_init(struct tegra_csi_channel *chan, int port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	dev_dbg(csi->dev, "%s\n", __func__);

	csi4_stream_write(chan, port_num, CIL_INTR_STATUS, 0xffffffff);
	csi4_stream_write(chan, port_num, CIL_ERR_INTR_STATUS, 0xffffffff);
	csi4_stream_write(chan, port_num, CIL_INTR_MASK, 0xffffffff);
	csi4_stream_write(chan, port_num, CIL_ERR_INTR_MASK, 0xffffffff);
	csi4_stream_write(chan, port_num, INTR_STATUS, 0x3ffff);
	csi4_stream_write(chan, port_num, ERR_INTR_STATUS, 0x7ffff);
	csi4_stream_write(chan, port_num, ERROR_STATUS2VI_MASK, 0x0);
	csi4_stream_write(chan, port_num, INTR_MASK, 0x0);
	csi4_stream_write(chan, port_num, ERR_INTR_MASK, 0x0);
}

static void csi4_stream_config(struct tegra_csi_channel *chan, int port_num)
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
}


static void csi4_phy_config(
	struct tegra_csi_channel *chan, int csi_port,
	int csi_lanes, bool enable)
{
	struct tegra_csi_device *csi = chan->csi;
	struct camera_common_data *s_data = chan->s_data;
	const struct sensor_mode_properties *mode = NULL;
	int phy_num = (csi_port & 0x6) >> 1;
	bool cil_a = (csi_port & 0x1) ? false : true;
	int cil_config;
	/* Clocks for the CSI interface */
	const unsigned int cil_clk_mhz = TEGRA_CSICIL_CLK_MHZ;
	const unsigned int csi_clk_mhz = csi->clk_freq / 1000000;
	/* Calculated clock settling times for cil and csi clocks */
	unsigned int cil_settletime = 0;
	unsigned int csi_settletime;

	dev_dbg(csi->dev, "%s\n", __func__);

	/* set to DPHY */
	csi4_phy_write(chan, phy_num, NVCSI_CIL_PHY_CTRL, DPHY);

	/* read current NVCSI_CIL_CONFIG setting */
	cil_config = csi4_phy_read(chan, phy_num, NVCSI_CIL_CONFIG);
	dev_dbg(csi->dev, "NVCSI_CIL_CONFIG = %08x\n", cil_config);

	if (cil_a) {
		/* soft reset for data lane */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET,
			SW_RESET1_EN | SW_RESET0_EN);
		/* reset CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			cil_config & ~DATA_LANE_A);
		/* disable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_A_PAD_CONFIG,
			PD_CLK | PD_IO0 | PD_IO1 | SPARE_IO0 | SPARE_IO1);

		/* setting up CIL B for 4 lane */
		if (csi_lanes > 2) {
			/* soft reset for data lane */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
			/* reset CSI lane number */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
				cil_config & ~DATA_LANE_B);
			/* disable clock lane*/
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_PAD_CONFIG,
				PD_CLK | PD_IO0 | PD_IO1 |
				SPARE_IO0 | SPARE_IO1);
		}

		/* power down de-serializer is CIL B is not in use*/
		if ((cil_config & DATA_LANE_B) == 0)
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_PAD_CONFIG, PDVCLAMP);
	} else {
		/* soft reset for data lane */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET,
			SW_RESET1_EN | SW_RESET0_EN);
		/* reset CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			cil_config & ~DATA_LANE_B);
		/* disable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_B_PAD_CONFIG,
			PD_CLK | PD_IO0 | PD_IO1 | SPARE_IO0 | SPARE_IO1);

		/* power down de-serializer if CIL A is not in use*/
		if ((cil_config & DATA_LANE_A) == 0)
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_PAD_CONFIG, PDVCLAMP);
	}

	if (!enable)
		return;

	/* power on de-serializer */
	csi4_phy_write(chan, phy_num, NVCSI_CIL_PAD_CONFIG, 0);

	/* Attempt to find the cil_settingtime from the device tree */
	if (s_data) {
		mode = &s_data->sensor_props.sensor_modes[s_data->mode];
		cil_settletime = mode->signal_properties.cil_settletime;
	} else if (chan->of_node) {
		int err = 0;
		const char *str;

		err = of_property_read_string(chan->of_node, "cil_settletime",
			&str);
		if (!err) {
			err = kstrtou32(str, 10, &cil_settletime);
			if (err)
				cil_settletime = 0;
		}
	}

	/* calculate MIPI settling times */
	dev_dbg(csi->dev, "cil core clock: %u, csi clock: %u", cil_clk_mhz,
		csi_clk_mhz);

	csi_settletime = tegra_csi_clk_settling_time(csi, cil_clk_mhz);
	/* If cil_settletime is 0, calculate a settling time */
	if (!cil_settletime) {
		cil_settletime = tegra_csi_ths_settling_time(csi,
			cil_clk_mhz,
			csi_clk_mhz);
	}

	dev_dbg(csi->dev, "csi settle time: %u, cil settle time: %u",
		csi_settletime, cil_settletime);

	if (cil_a) {
		/* set CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			(cil_config & ~DATA_LANE_A) |
			(csi_lanes << DATA_LANE_A_OFFSET));
		/* enable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_A_PAD_CONFIG,
			E_INPUT_LP_CLK | E_INPUT_LP_IO0 | E_INPUT_LP_IO1);
		/* setup settle time */
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_A_CONTROL,
			DEFAULT_DESKEW_COMPARE | DEFAULT_DESKEW_SETTLE |
			csi_settletime << CLK_SETTLE_SHIFT |
			T18X_BYPASS_LP_SEQ |
			cil_settletime << THS_SETTLE_SHIFT);
		/* release soft reset */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET, 0x0);

		/* setting up CIL B for 4 lane */
		if (csi_lanes > 2) {
			/* set CSI lane number */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
				csi_lanes << DATA_LANE_A_OFFSET);
			/* enable clock lane*/
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_PAD_CONFIG,
				E_INPUT_LP_IO0 | E_INPUT_LP_IO1 | PD_CLK);
			/* setup settle time */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_CONTROL,
				DEFAULT_DESKEW_COMPARE |
				DEFAULT_DESKEW_SETTLE |
				csi_settletime << CLK_SETTLE_SHIFT |
				T18X_BYPASS_LP_SEQ |
				cil_settletime << THS_SETTLE_SHIFT);
			/* release soft reset */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_SW_RESET, 0x0);
		}
	} else {
		/* set CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			(cil_config & ~DATA_LANE_B) |
			(csi_lanes << DATA_LANE_B_OFFSET));
		/* enable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_B_PAD_CONFIG,
			E_INPUT_LP_CLK | E_INPUT_LP_IO0 | E_INPUT_LP_IO1);
		/* setup settle time */
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_B_CONTROL,
			DEFAULT_DESKEW_COMPARE | DEFAULT_DESKEW_SETTLE |
			csi_settletime << CLK_SETTLE_SHIFT |
			T18X_BYPASS_LP_SEQ |
			cil_settletime << THS_SETTLE_SHIFT);
		/* release soft reset */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET, 0x0);
	}
}

static void csi4_stream_check_status(
	struct tegra_csi_channel *chan, int port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int status = 0;

	dev_dbg(csi->dev, "%s\n", __func__);
	if (!chan->pg_mode) {
		status = csi4_stream_read(chan, port_num, ERROR_STATUS2VI_VC0);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC0 = 0x%08x\n",
				__func__, port_num, status);

		status = csi4_stream_read(chan, port_num, ERROR_STATUS2VI_VC1);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC1 = 0x%08x\n",
				__func__, port_num, status);

		status = csi4_stream_read(chan, port_num, ERROR_STATUS2VI_VC2);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC2 = 0x%08x\n",
				__func__, port_num, status);

		status = csi4_stream_read(chan, port_num, ERROR_STATUS2VI_VC3);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC2 = 0x%08x\n",
				__func__, port_num, status);
	}
	status = csi4_stream_read(chan, port_num, INTR_STATUS);
	if (status)
		dev_err(csi->dev,
				"%s (%d) INTR_STATUS 0x%08x\n",
				__func__, port_num, status);

	status = csi4_stream_read(chan, port_num, ERR_INTR_STATUS);
	if (status)
		dev_err(csi->dev,
				"%s (%d) ERR_INTR_STATUS 0x%08x\n",
				__func__, port_num, status);
}

static void csi4_cil_check_status(struct tegra_csi_channel *chan, int port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int status = 0;

	dev_dbg(csi->dev, "%s %d\n", __func__, __LINE__);

	status = csi4_stream_read(chan, port_num, CIL_INTR_STATUS);
	if (status)
		dev_err(csi->dev,
			"%s (%d) CIL_INTR_STATUS 0x%08x\n",
			__func__, port_num, status);

	status = csi4_stream_read(chan, port_num, CIL_ERR_INTR_STATUS);
	if (status)
		dev_err(csi->dev,
			"%s (%d) CIL_ERR_INTR_STATUS 0x%08x\n",
			__func__, port_num, status);
}


static int csi4_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:cannot enable csi\n", __func__);

	return err;
}

static int csi4_power_off(struct tegra_csi_device *csi)
{
	nvhost_module_idle(csi->pdev);

	return 0;
}

static void csi4_tpg_stop_streaming(struct tegra_csi_channel *chan,
				int ports_index)
{
	unsigned int csi_port = chan->ports[ports_index].num;
	struct tegra_csi_device *csi = chan->csi;

	dev_dbg(csi->dev, "%s\n", __func__);
	csi4_stream_check_status(chan, csi_port);
	csi4_cil_check_status(chan, csi_port);
	csi4_stream_write(chan, csi_port, PP_EN_CTRL, 0);
	csi4_stream_write(chan, csi_port, TPG_EN_0, 0);
	csi4_stream_write(chan, csi_port, PG_CTRL, PG_DISABLE);

	mutex_lock(&csi->source_update);
	if (csi->tpg_active != 0) {
		mutex_unlock(&csi->source_update);
		return;
	}
	mutex_unlock(&csi->source_update);
	nvhost_module_remove_client(csi->pdev, csi);
}
static int csi4_tpg_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &chan->ports[port_num];
	struct tegra_csi_device *csi = chan->csi;
	unsigned int val, csi_port, csi_lanes;
	int ret = 0;

	if (!port->core_format) {
		dev_err(csi->dev, "Fail to find tegra video fmt");
		return -EINVAL;
	}

	mutex_lock(&csi->source_update);
	if (csi->tpg_active == 1) {
		mutex_unlock(&csi->source_update);
		ret = nvhost_module_add_client(csi->pdev, csi);
		if (ret)
			return ret;
		nvhost_module_set_rate(csi->pdev, csi,
				DEFAULT_TPG_FREQ, 0, NVHOST_CLOCK);
	} else
		mutex_unlock(&csi->source_update);

	csi_port = port->num;
	csi_lanes = port->lanes;
	dev_dbg(csi->dev, "%s CSI port=%d, # lanes=%d\n",
			__func__, csi_port, csi_lanes);

	csi4_stream_write(chan, csi_port, PH_CHK_CTRL, 0);
	csi4_stream_write(chan, csi_port, INTR_MASK, PH_ECC_MULTI_BIT_ERR |
			PD_CRC_ERR_VC0 | PH_ECC_SINGLE_BIT_ERR_VC0);
	csi4_stream_write(chan, csi_port, ERR_INTR_MASK, PH_ECC_MULTI_BIT_ERR |
			PD_CRC_ERR_VC0 | PH_ECC_SINGLE_BIT_ERR_VC0);
	csi4_stream_write(chan, csi_port, ERROR_STATUS2VI_MASK,
			CFG_ERR_STATUS2VI_MASK_VC0 |
			CFG_ERR_STATUS2VI_MASK_VC1 |
			CFG_ERR_STATUS2VI_MASK_VC2 |
			CFG_ERR_STATUS2VI_MASK_VC3);
	/* calculate PG blank */
	csi4_stream_write(chan, csi_port, PG_BLANK,
			((port->v_blank & PG_VBLANK_MASK) << PG_VBLANK_OFFSET) |
			((port->h_blank & PG_HBLANK_MASK) << PG_HBLANK_OFFSET));
	csi4_stream_write(chan, csi_port, PG_PHASE, 0x0);
	csi4_stream_write(chan, csi_port, PG_RED_FREQ,
			(0x10 << PG_VERT_INIT_FREQ_OFFSET)|
			(0x10 << PG_HOR_INIT_FREQ_OFFSET));
	csi4_stream_write(chan, csi_port, PG_RED_FREQ_RATE, 0x0);
	csi4_stream_write(chan, csi_port, PG_GREEN_FREQ,
			(0x10 << PG_VERT_INIT_FREQ_OFFSET)|
			(0x10 << PG_HOR_INIT_FREQ_OFFSET));
	csi4_stream_write(chan, csi_port, PG_GREEN_FREQ_RATE, 0x0);
	csi4_stream_write(chan, csi_port, PG_BLUE_FREQ,
			(0x10 << PG_VERT_INIT_FREQ_OFFSET)|
			(0x10 << PG_HOR_INIT_FREQ_OFFSET));
	csi4_stream_write(chan, csi_port, PG_BLUE_FREQ_RATE, 0x0);
	/* calculate PG IMAGE SIZE and DT */
	mutex_lock(&chan->format_lock);
	val = port->format.height << HEIGHT_OFFSET |
		(port->format.width *
		(port->core_format->vf_code == TEGRA_VF_RAW10 ? 10 : 24) / 8);
	mutex_unlock(&chan->format_lock);
	csi4_stream_write(chan, csi_port, PG_IMAGE_SIZE, val);
	csi4_stream_write(chan, csi_port, PG_IMAGE_DT,
			port->core_format->img_dt);
	csi4_stream_write(chan, csi_port, PP_EN_CTRL, CFG_PP_EN);
	csi4_stream_write(chan, csi_port, TPG_EN_0, cfg_tpg_en);

	csi4_stream_write(chan, csi_port, PG_CTRL,
			((chan->pg_mode - 1) << PG_MODE_OFFSET) | PG_ENABLE);
	return 0;
}
static int csi4_hw_init(struct tegra_csi_device *csi)
{
	csi->iomem[0] = csi->iomem_base + TEGRA_CSI_STREAM_0_BASE;
	csi->iomem[1] = csi->iomem_base + TEGRA_CSI_STREAM_2_BASE;
	csi->iomem[2] = csi->iomem_base + TEGRA_CSI_STREAM_4_BASE;

	return 0;
}
static int csi4_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int csi_port, csi_lanes, ret = 0;

	csi_port = chan->ports[port_num].num;
	csi_lanes = chan->ports[port_num].lanes;
	dev_dbg(csi->dev, "%s ports index=%d, lanes=%d\n",
			__func__, csi_port, csi_lanes);

	if (chan->pg_mode)
		ret = csi4_tpg_start_streaming(chan, port_num);
	else {
		csi4_stream_init(chan, csi_port);
		csi4_stream_config(chan, csi_port);
		/* enable DPHY */
		csi4_phy_config(chan, csi_port, csi_lanes, true);
		csi4_stream_write(chan, csi_port, PP_EN_CTRL, CFG_PP_EN);
	}
	return ret;
}

static void csi4_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	int csi_port, csi_lanes;

	dev_dbg(csi->dev, "%s ports index=%d, lanes=%d\n",
			__func__, port_num, chan->numlanes);

	csi_port = chan->ports[port_num].num;
	csi_lanes = chan->ports[port_num].lanes;

	if (chan->pg_mode)
		csi4_tpg_stop_streaming(chan, port_num);
	else {
		/* disable DPHY */
		csi4_phy_config(chan, csi_port, csi_lanes, false);
		csi4_stream_check_status(chan, csi_port);
		csi4_cil_check_status(chan, csi_port);
	}
}

static void csi4_override_format(struct tegra_csi_channel *chan,
		enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &chan->ports[port_num];
	unsigned int val;
	int csi_port;

	if (!chan->pg_mode) {
		dev_err(chan->csi->dev, "%s non PG format update failed\n",
				__func__);
		return;
	}

	/* calculate PG IMAGE SIZE and DT */
	mutex_lock(&chan->format_lock);
	val = port->format.height << HEIGHT_OFFSET |
		(port->format.width *
		(port->core_format->vf_code == TEGRA_VF_RAW10 ? 10 : 24) / 8);
	mutex_unlock(&chan->format_lock);

	csi_port = chan->ports[port_num].num;
	csi4_stream_write(chan, csi_port, PG_IMAGE_SIZE, val);
}

static int csi4_mipi_cal(struct tegra_csi_channel *chan)
{
	unsigned int lanes, num_ports, port, addr;
	unsigned int cila, cilb;
	struct tegra_csi_device *csi = chan->csi;

	lanes = 0;
	num_ports = 0;
	port = 0;
	while (num_ports < chan->numports) {
		port = chan->ports[num_ports].num;
		dev_dbg(csi->dev, "csi port:%d\n", port);

		if (chan->numlanes <= 2) {
			lanes |= CSIA << port;
			addr = (port % 2 == 0 ?
				NVCSI_CIL_A_SW_RESET : NVCSI_CIL_B_SW_RESET);
			csi4_phy_write(chan, port >> 1, addr,
				SW_RESET1_EN | SW_RESET0_EN);
		} else {
			lanes |= (CSIA | CSIB) << port;
			cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
				(0x01 << E_INPUT_LP_IO1_SHIFT) |
				(0x01 << E_INPUT_LP_CLK_SHIFT) |
				(0x00 << PD_CLK_SHIFT) |
				(0x00 << PD_IO0_SHIFT) |
				(0x00 << PD_IO1_SHIFT);
			cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
				(0x01 << E_INPUT_LP_IO1_SHIFT) |
				(0x01 << PD_CLK_SHIFT) |
				(0x00 << PD_IO0_SHIFT) |
				(0x00 << PD_IO1_SHIFT);
			csi4_phy_write(chan, port >> 1,
				NVCSI_CIL_A_BASE + PAD_CONFIG_0, cila);
			csi4_phy_write(chan, port >> 1,
				NVCSI_CIL_B_BASE + PAD_CONFIG_0, cilb);
			csi4_phy_write(chan, port >> 1, NVCSI_CIL_A_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
			csi4_phy_write(chan, port >> 1, NVCSI_CIL_B_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
		}
		num_ports++;
	}
	if (!lanes) {
		dev_err(csi->dev, "Selected no CSI lane, cannot do calibration");
		return -EINVAL;
	}
	return tegra_mipi_calibration(lanes);
}
struct tegra_csi_fops csi4_fops = {
	.csi_power_on = csi4_power_on,
	.csi_power_off = csi4_power_off,
	.csi_start_streaming = csi4_start_streaming,
	.csi_stop_streaming = csi4_stop_streaming,
	.csi_override_format = csi4_override_format,
	.mipical = csi4_mipi_cal,
	.hw_init = csi4_hw_init,
};
