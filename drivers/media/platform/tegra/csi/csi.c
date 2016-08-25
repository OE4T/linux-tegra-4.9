/*
 * NVIDIA Tegra CSI Device
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/camera_common.h>

#include "dev.h"
#include "vi/vi.h"
#include "csi/csi.h"
#include "camera/mc_common.h"
#include "mipical/mipi_cal.h"
#include "linux/nvhost.h"
#include "nvcsi/nvcsi.h"

#define DEBUG 0

static void csi_write(struct tegra_csi_device *csi, unsigned int addr,
			u32 val, u8 port)
{
	dev_dbg(csi->dev, "%s:port %d offset 0x%08x val:0x%08x\n",
				__func__, port, addr, val);
	writel(val, (csi->iomem[port] + addr));
}

static u32 csi_read(struct tegra_csi_device *csi, unsigned int addr,
			u8 port)
{
	dev_dbg(csi->dev, "%s:port %d offset 0x%08x\n", __func__, port, addr);
	return readl((csi->iomem[port] + addr));
}

/* Pixel parser registers accessors */
static void pp_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
#if DEBUG
	pr_debug("%s:offset 0x%08x val:0x%08x\n", __func__, addr, val);
#endif
	writel(val, port->pixel_parser + addr);
}

static u32 pp_read(struct tegra_csi_port *port, u32 addr)
{
#if DEBUG
	pr_debug("%s:offset 0x%08x\n", __func__, addr);
#endif
	return readl(port->pixel_parser + addr);
}

/* CSI CIL registers accessors */
static void cil_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
#if DEBUG
	pr_debug("%s:offset 0x%08x val:0x%08x\n", __func__, addr, val);
#endif
	writel(val, port->cil + addr);
}

static u32 cil_read(struct tegra_csi_port *port, u32 addr)
{
#if DEBUG
	pr_debug("%s:offset 0x%08x\n", __func__, addr);
#endif
	return readl(port->cil + addr);
}

/* Test pattern generator registers accessor */
static void tpg_write(struct tegra_csi_port *port,
			unsigned int addr, u32 val)
{
	writel(val, port->tpg + addr);
}

static int csi_get_clks(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	csi->clk = devm_clk_get(&pdev->dev, "csi");
	if (IS_ERR(csi->clk)) {
		dev_err(&pdev->dev, "Failed to get csi clock\n");
		return PTR_ERR(csi->clk);
	}

	csi->tpg_clk = devm_clk_get(&pdev->dev, "pll_d");
	if (IS_ERR(csi->tpg_clk)) {
		dev_err(&pdev->dev, "Failed to get tpg clock\n");
		return PTR_ERR(csi->tpg_clk);
	}

	csi->cil[0] = devm_clk_get(&pdev->dev, "cilab");
	if (IS_ERR(csi->cil[0])) {
		dev_err(&pdev->dev, "Failed to get cilab clock\n");
		return PTR_ERR(csi->cil[0]);
	}

	csi->cil[1] = devm_clk_get(&pdev->dev, "cilcd");
	if (IS_ERR(csi->cil[1])) {
		dev_err(&pdev->dev, "Failed to get cilcd clock\n");
		return PTR_ERR(csi->cil[1]);
	}

	csi->cil[2] = devm_clk_get(&pdev->dev, "cile");
	if (IS_ERR(csi->cil[2])) {
		dev_err(&pdev->dev, "Failed to get cile clock\n");
		return PTR_ERR(csi->cil[2]);
	}

	return 0;
}

static int set_csi_properties(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	struct camera_common_data *s_data = &csi->s_data[0];

	/*
	* These values are only used for tpg mode
	* With sensor, CSI power and clock info are provided
	* by the sensor sub device
	*/
	s_data->csi_port = 0;
	s_data->numlanes = 12;
	csi->clk_freq = TEGRA_CLOCK_CSI_PORT_MAX;

	return 0;
}

void set_csi_portinfo(struct tegra_csi_device *csi,
	unsigned int port, unsigned int numlanes)
{
	struct camera_common_data *s_data = &csi->s_data[port];

	s_data->csi_port = port;
	s_data->numlanes = numlanes;
	s_data->def_clk_freq = TEGRA_CLOCK_CSI_PORT_MAX;
}
EXPORT_SYMBOL(set_csi_portinfo);


static void set_csi_registers(struct tegra_csi_device *csi,
		void __iomem *regbase)
{
	csi->iomem[0] = (regbase + TEGRA_CSI_PIXEL_PARSER_0_BASE);
	csi->iomem[1] = (regbase + TEGRA_CSI_PIXEL_PARSER_2_BASE);
	csi->iomem[2] = (regbase + TEGRA_CSI_PIXEL_PARSER_4_BASE);

	/* calculate per channel tegra_csi_port info based on
	 * tegra_csi_port_num
	 *
	 */
#if 0
	for (j = 0; j < 3; j++) {
		for (i = 0; i < 2; i++) {
			idx = (j << 1) + i;
			/* Initialize port register bases */
			csi->ports[idx].pixel_parser = csi->iomem[j] +
				i * TEGRA_CSI_PORT_OFFSET;
			csi->ports[idx].cil = csi->iomem[j] +
				TEGRA_CSI_CIL_OFFSET +
				i * TEGRA_CSI_PORT_OFFSET;
			csi->ports[idx].tpg = csi->iomem[j] +
				TEGRA_CSI_TPG_OFFSET +
				i * TEGRA_CSI_PORT_OFFSET;

			csi->ports[idx].num = idx;
			csi->ports[idx].lanes = 2;
		}
	}
#endif
}

static int clock_start(struct tegra_csi_device *csi,
		       struct clk *clk, unsigned int freq)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		dev_err(csi->dev, "csi clk enable error %d\n", err);
	err = clk_set_rate(clk, freq);
	if (err)
		dev_err(csi->dev, "csi clk set rate error %d\n", err);

	return err;
}

void tegra_csi_pad_control(struct tegra_csi_device *csi,
				unsigned char *port_num, int enable)
{
	int i, port;

	if (enable) {
		for (i = 0; csi_port_is_valid(port_num[i]); i++)
			port = port_num[i];
	} else {
		for (i = 0; csi_port_is_valid(port_num[i]); i++)
			port = port_num[i];
	}
}
EXPORT_SYMBOL(tegra_csi_pad_control);

int tegra_csi_channel_power(struct tegra_csi_device *csi,
				unsigned char *port_num, int enable)
{
	int err = 0;
	int i, cil_num, port;

	if (enable) {
		for (i = 0; csi_port_is_valid(port_num[i]); i++) {
			port = port_num[i];
			cil_num = port >> 1;
			err = clock_start(csi,
				csi->cil[cil_num], csi->clk_freq);
			if (err)
				dev_err(csi->dev, "cil clk start error\n");
		}
	} else {
		for (i = 0; csi_port_is_valid(port_num[i]); i++) {
			port = port_num[i];
			cil_num = port >> 1;
			clk_disable_unprepare(csi->cil[cil_num]);
		}
	}

	return err;
}
EXPORT_SYMBOL(tegra_csi_channel_power);

int tegra_csi_power(struct tegra_csi_device *csi, int enable)
{
	int err = 0;

	if (enable) {
		/* set clk and power */
		err = clk_prepare_enable(csi->clk);
		if (err)
			dev_err(csi->dev, "csi clk enable error\n");

		if (csi->pg_mode) {
			err = clock_start(csi, csi->tpg_clk,
						TEGRA_CLOCK_TPG_MAX);
			if (err)
				dev_err(csi->dev, "tpg clk start error\n");
			else {
				tegra_clk_cfg_ex(csi->tpg_clk,
					TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
				tegra_clk_cfg_ex(csi->tpg_clk,
					TEGRA_CLK_PLLD_DSI_OUT_ENB, 1);
				tegra_clk_cfg_ex(csi->tpg_clk,
					TEGRA_CLK_MIPI_CSI_OUT_ENB, 0);
			}
		}
	} else {
		if (csi->pg_mode) {
			tegra_clk_cfg_ex(csi->tpg_clk,
					 TEGRA_CLK_MIPI_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(csi->tpg_clk,
					 TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
			tegra_clk_cfg_ex(csi->tpg_clk,
					 TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
			clk_disable_unprepare(csi->tpg_clk);
		}
		clk_disable_unprepare(csi->clk);
	}

	return err;
}
EXPORT_SYMBOL(tegra_csi_power);

/*
 * -----------------------------------------------------------------------------
 * CSI Subdevice Video Operations
 * -----------------------------------------------------------------------------
 */

/* Test Pattern Generator setup */
void tegra_csi_tpg_start_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			((csi->pg_mode - 1) << PG_MODE_OFFSET) |
			PG_ENABLE);
	tpg_write(port, TEGRA_CSI_PG_PHASE, 0x0);
	tpg_write(port, TEGRA_CSI_PG_RED_FREQ,
			(0x10 << PG_RED_VERT_INIT_FREQ_OFFSET) |
			(0x10 << PG_RED_HOR_INIT_FREQ_OFFSET));
	tpg_write(port, TEGRA_CSI_PG_RED_FREQ_RATE, 0x0);
	tpg_write(port, TEGRA_CSI_PG_GREEN_FREQ,
			(0x10 << PG_GREEN_VERT_INIT_FREQ_OFFSET) |
			(0x10 << PG_GREEN_HOR_INIT_FREQ_OFFSET));
	tpg_write(port, TEGRA_CSI_PG_GREEN_FREQ_RATE, 0x0);
	tpg_write(port, TEGRA_CSI_PG_BLUE_FREQ,
			(0x10 << PG_BLUE_VERT_INIT_FREQ_OFFSET) |
			(0x10 << PG_BLUE_HOR_INIT_FREQ_OFFSET));
	tpg_write(port, TEGRA_CSI_PG_BLUE_FREQ_RATE, 0x0);
}

void tegra_csi_start_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	csi_write(csi, TEGRA_CSI_CLKEN_OVERRIDE, 0, port_num >> 1);

	/* Clean up status */
	pp_write(port, TEGRA_CSI_PIXEL_PARSER_STATUS, 0xFFFFFFFF);
	cil_write(port, TEGRA_CSI_CIL_STATUS, 0xFFFFFFFF);
	cil_write(port, TEGRA_CSI_CILX_STATUS, 0xFFFFFFFF);

	cil_write(port, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);

	/* CIL PHY registers setup */
	cil_write(port, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
	cil_write(port, TEGRA_CSI_CIL_PHY_CONTROL,
			BYPASS_LP_SEQ | 0xA);

	/*
	 * The CSI unit provides for connection of up to six cameras in
	 * the system and is organized as three identical instances of
	 * two MIPI support blocks, each with a separate 4-lane
	 * interface that can be configured as a single camera with 4
	 * lanes or as a dual camera with 2 lanes available for each
	 * camera.
	 */
	if (port->lanes == 4) {
		int port_val = ((port_num >> 1) << 1);
		struct tegra_csi_port *port_a = &csi->ports[port_val];
		struct tegra_csi_port *port_b = &csi->ports[port_val + 1];

		cil_write(port_a, TEGRA_CSI_CIL_PAD_CONFIG0,
				BRICK_CLOCK_A_4X);
		cil_write(port_b, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
		cil_write(port_b, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);
		cil_write(port_a, TEGRA_CSI_CIL_PHY_CONTROL,
				BYPASS_LP_SEQ | 0xA);
		cil_write(port_b, TEGRA_CSI_CIL_PHY_CONTROL,
				BYPASS_LP_SEQ | 0xA);
		csi_write(csi, TEGRA_CSI_PHY_CIL_COMMAND,
				CSI_A_PHY_CIL_ENABLE | CSI_B_PHY_CIL_ENABLE,
				port_num >> 1);
	} else {
		u32 val = csi_read(csi, TEGRA_CSI_PHY_CIL_COMMAND,
					port_num >> 1);
		int port_val = ((port_num >> 1) << 1);
		struct tegra_csi_port *port_a = &csi->ports[port_val];

		cil_write(port_a, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
		val |= ((port->num & 0x1) == PORT_A) ? CSI_A_PHY_CIL_ENABLE :
			CSI_B_PHY_CIL_ENABLE;
		csi_write(csi, TEGRA_CSI_PHY_CIL_COMMAND, val,
				port_num >> 1);
	}

	/* CSI pixel parser registers setup */
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
			(0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
			CSI_PP_SINGLE_SHOT_ENABLE | CSI_PP_RST);
	pp_write(port, TEGRA_CSI_PIXEL_PARSER_INTERRUPT_MASK, 0x0);
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_CONTROL0,
			CSI_PP_PACKET_HEADER_SENT |
			CSI_PP_DATA_IDENTIFIER_ENABLE |
			CSI_PP_WORD_COUNT_SELECT_HEADER |
			CSI_PP_CRC_CHECK_ENABLE |  CSI_PP_WC_CHECK |
			CSI_PP_OUTPUT_FORMAT_STORE | CSI_PPA_PAD_LINE_NOPAD |
			CSI_PP_HEADER_EC_DISABLE | CSI_PPA_PAD_FRAME_NOPAD |
			(port->num & 1));
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_CONTROL1,
			(0x1 << CSI_PP_TOP_FIELD_FRAME_OFFSET) |
			(0x1 << CSI_PP_TOP_FIELD_FRAME_MASK_OFFSET));
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_GAP,
			0x14 << PP_FRAME_MIN_GAP_OFFSET);
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_EXPECTED_FRAME, 0x0);
	pp_write(port, TEGRA_CSI_INPUT_STREAM_CONTROL,
			(0x3f << CSI_SKIP_PACKET_THRESHOLD_OFFSET) |
			(port->lanes - 1));
#if DEBUG
	/* 0x454140E1 - register setting for line counter */
	/* 0x454340E1 - tracks frame start, line starts, hpa headers */
	pp_write(port, TEGRA_CSI_DEBUG_CONTROL, 0x454340E1);
#endif
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
			(0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
			CSI_PP_SINGLE_SHOT_ENABLE | CSI_PP_ENABLE);
}
EXPORT_SYMBOL(tegra_csi_start_streaming);

int tegra_csi_error(struct tegra_csi_channel *chan,
			enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port;
	u32 val;
	int err = 0, i;

	for (i = 0; i < chan->numports; i++) {
		port = &chan->ports[i];
		/*
		 * only uncorrectable header error and multi-bit
		 * transmission errors are checked as they cannot be
		 * corrected automatically
		 */
		val = pp_read(port, TEGRA_CSI_PIXEL_PARSER_STATUS);
		err |= val & 0x4000;
		pp_write(port, TEGRA_CSI_PIXEL_PARSER_STATUS, val);

		val = cil_read(port, TEGRA_CSI_CIL_STATUS);
		err |= val & 0x02;
		cil_write(port, TEGRA_CSI_CIL_STATUS, val);

		val = cil_read(port, TEGRA_CSI_CILX_STATUS);
		err |= val & 0x00020020;
		cil_write(port, TEGRA_CSI_CILX_STATUS, val);
	}

	return err;
}

void tegra_csi_status(struct tegra_csi_channel *chan,
			enum tegra_csi_port_num port_num)
{
	int i;
	u32 val;
	struct tegra_csi_port *port;

	for (i = 0; i < chan->numports; i++) {
		port = &chan->ports[i];
		val = pp_read(port, TEGRA_CSI_PIXEL_PARSER_STATUS);

		dev_dbg(chan->csi->dev, "TEGRA_CSI_PIXEL_PARSER_STATUS 0x%08x\n",
			val);

		val = cil_read(port, TEGRA_CSI_CIL_STATUS);
		dev_dbg(chan->csi->dev, "TEGRA_CSI_CIL_STATUS 0x%08x\n", val);

		val = cil_read(port, TEGRA_CSI_CILX_STATUS);
		dev_dbg(chan->csi->dev, "TEGRA_CSI_CILX_STATUS 0x%08x\n", val);

#if DEBUG
		val = pp_read(port, TEGRA_CSI_DEBUG_COUNTER_0);
		dev_dbg(chan->csi->dev,
			"TEGRA_CSI_DEBUG_COUNTER_0 0x%08x\n", val);
		val = pp_read(port, TEGRA_CSI_DEBUG_COUNTER_1);
		dev_dbg(chan->csi->dev,
			"TEGRA_CSI_DEBUG_COUNTER_1 0x%08x\n", val);
		val = pp_read(port,
			TEGRA_CSI_DEBUG_COUNTER_2);
		dev_dbg(chan->csi->dev,
			"TEGRA_CSI_DEBUG_COUNTER_2 0x%08x\n", val);
#endif
	}
}
EXPORT_SYMBOL(tegra_csi_status);

void tegra_csi_error_recover(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port;
	struct tegra_csi_device *csi;
	int i;

	csi = chan->csi;

	for (i = 0; i < chan->numports; i++) {
		port = &chan->ports[i];

		if (port->lanes == 4) {
			int port_val = ((port_num >> 1) << 1);
			struct tegra_csi_port *port_a = &chan->ports[port_val];
			struct tegra_csi_port *port_b =
				&chan->ports[port_val+1];

			tpg_write(port_a, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				PG_ENABLE);
			tpg_write(port_b, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				PG_ENABLE);
			cil_write(port_a, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
			cil_write(port_b, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
			csi_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x1,
					port_num >> 1);
			/* sleep for clock cycles to drain the Rx FIFO */
			usleep_range(10, 20);
			cil_write(port_a, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
			cil_write(port_b, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
			csi_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x0,
				port_num >> 1);
			tpg_write(port_a, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				PG_DISABLE);
			tpg_write(port_b, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				PG_DISABLE);
		} else {
			tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				PG_ENABLE);
			cil_write(port, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
			csi_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x1,
				port_num >> 1);
			/* sleep for clock cycles to drain the Rx FIFO */
			usleep_range(10, 20);
			cil_write(port, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
			csi_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x0,
				port_num >> 1);
			tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				PG_DISABLE);
		}
	}
}

void tegra_csi_stop_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	if (csi->pg_mode)
		tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_DISABLE);

	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
			(0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
			CSI_PP_DISABLE);
}
EXPORT_SYMBOL(tegra_csi_stop_streaming);

static struct tegra_csi_device *get_csi(void)
{
	struct device_node *np;
	struct platform_device *dev;
	struct nvhost_device_data *pdata;

	np = of_find_node_by_name(NULL, "nvcsi");
	if (!np) {
		pr_err("%s: Can not find nvcsi node\n", __func__);
		return NULL;
	}
	dev = of_find_device_by_node(np);
	if (!dev) {
		pr_err("%s:Can not find device\n", __func__);
		return NULL;
	}
	pdata = platform_get_drvdata(dev);
	if (!pdata) {
		pr_err("%s:Can not find driver\n", __func__);
		return NULL;
	}
	return &((struct nvcsi *)pdata->private_data)->csi;
}

int csi_mipi_cal(struct tegra_channel *chan, char is_bypass)
{
	unsigned int lanes, cur_lanes;
	unsigned int cila, cilb;
	struct tegra_csi_device *csi;
	int j;

	csi = get_csi();
	if (!csi)
		return -EINVAL;

	lanes = 0;

	if (chan->numlanes == 2 && chan->total_ports == 1) {
		cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << E_INPUT_LP_CLK) |
			(0x00 << PD_CLK) |
			(0x00 << PD_IO0) |
			(0x00 << PD_IO1);
		cilb = cila;
		switch (chan->port[0]) {
		case PORT_A:
			lanes = CSIA;
			host1x_writel(csi->pdev, NVCSI_PHY_0_BASE +
					NVCSI_CIL_A_BASE + PAD_CONFIG_0,
					cila);
			break;
		case PORT_B:
			lanes = CSIB;
			host1x_writel(csi->pdev, NVCSI_PHY_0_BASE +
					NVCSI_CIL_B_BASE + PAD_CONFIG_0,
					cilb);
			break;
		case PORT_C:
			lanes = CSIC;
			host1x_writel(csi->pdev, NVCSI_PHY_1_BASE +
					NVCSI_CIL_A_BASE + PAD_CONFIG_0,
					cila);
			break;
		case PORT_D:
			lanes = CSID;
			host1x_writel(csi->pdev, NVCSI_PHY_1_BASE +
					NVCSI_CIL_B_BASE + PAD_CONFIG_0,
					cilb);
			break;
		case PORT_E:
			lanes = CSIE;
			host1x_writel(csi->pdev, NVCSI_PHY_2_BASE +
					NVCSI_CIL_A_BASE + PAD_CONFIG_0,
					cila);
			break;
		case PORT_F:
			lanes = CSIF;
			host1x_writel(csi->pdev, NVCSI_PHY_2_BASE +
					NVCSI_CIL_B_BASE + PAD_CONFIG_0,
					cilb);
			break;
		}
	} else if (chan->numlanes == 4 && chan->total_ports == 1) {
		cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << E_INPUT_LP_CLK) |
			(0x00 << PD_CLK) |
			(0x00 << PD_IO0) |
			(0x00 << PD_IO1);
		cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << PD_CLK) |
			(0x00 << PD_IO0) |
			(0x00 << PD_IO1);
		switch (chan->port[0]) {
		case PORT_A:
		case PORT_B:
			lanes = CSIA|CSIB;
			host1x_writel(csi->pdev, NVCSI_PHY_0_BASE +
					NVCSI_CIL_A_BASE + PAD_CONFIG_0,
					cila);
			host1x_writel(csi->pdev, NVCSI_PHY_0_BASE +
					NVCSI_CIL_B_BASE + PAD_CONFIG_0,
					cilb);
			break;
		case PORT_C:
		case PORT_D:
			lanes = CSIC|CSID;
			host1x_writel(csi->pdev, NVCSI_PHY_1_BASE +
					NVCSI_CIL_A_BASE + PAD_CONFIG_0,
					cila);
			host1x_writel(csi->pdev, NVCSI_PHY_1_BASE +
					NVCSI_CIL_B_BASE + PAD_CONFIG_0,
					cilb);
			break;
		case PORT_E:
		case PORT_F:
			lanes = CSIE|CSIF;
			host1x_writel(csi->pdev, NVCSI_PHY_2_BASE +
					NVCSI_CIL_A_BASE + PAD_CONFIG_0,
					cila);
			host1x_writel(csi->pdev, NVCSI_PHY_2_BASE +
					NVCSI_CIL_B_BASE + PAD_CONFIG_0,
					cilb);
			break;
		}
	} else if (chan->numlanes == 8) {
		cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << E_INPUT_LP_CLK) |
			(0x00 << PD_CLK) |
			(0x00 << PD_IO0) |
			(0x00 << PD_IO1);
		cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << PD_CLK) |
			(0x00 << PD_IO0) |
			(0x00 << PD_IO1);
		cur_lanes = 0;
		for (j = 0; j < chan->valid_ports; ++j) {
			switch (chan->port[j]) {
			case PORT_A:
			case PORT_B:
				cur_lanes = CSIA|CSIB;
				host1x_writel(csi->pdev, NVCSI_PHY_0_BASE +
						NVCSI_CIL_A_BASE + PAD_CONFIG_0,
						cila);
				host1x_writel(csi->pdev, NVCSI_PHY_0_BASE +
						NVCSI_CIL_B_BASE + PAD_CONFIG_0,
						cilb);
				break;
			case PORT_C:
			case PORT_D:
				cur_lanes = CSIC|CSID;
				host1x_writel(csi->pdev, NVCSI_PHY_1_BASE +
						NVCSI_CIL_A_BASE + PAD_CONFIG_0,
						cila);
				host1x_writel(csi->pdev, NVCSI_PHY_1_BASE +
						NVCSI_CIL_B_BASE + PAD_CONFIG_0,
						cilb);
				break;
			case PORT_E:
			case PORT_F:
				cur_lanes = CSIE|CSIF;
				host1x_writel(csi->pdev, NVCSI_PHY_2_BASE +
						NVCSI_CIL_A_BASE + PAD_CONFIG_0,
						cila);
				host1x_writel(csi->pdev, NVCSI_PHY_2_BASE +
						NVCSI_CIL_B_BASE + PAD_CONFIG_0,
						cilb);
				break;
			default:
				dev_err(csi->dev, "csi_port number: %d",
						chan->port[0]);
				break;
			}
			lanes |= cur_lanes;
		}
	}

	if (!lanes) {
		dev_err(csi->dev, "Selected no CSI lane, cannot do calibration");
		return -EINVAL;
	}
	return tegra_mipi_calibration(lanes);
}
EXPORT_SYMBOL(csi_mipi_cal);

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_csi_device *csi;
	struct tegra_channel *chan;
	int index;

	csi = to_csi(subdev);
	if (!csi)
		return -EINVAL;
	if (enable)
		nvhost_module_busy(csi->pdev);
	else
		nvhost_module_idle(csi->pdev);
	return 0;

	if (csi->pg_mode)
		return 0;

	chan = subdev->host_priv;
	for (index = 0; index < chan->valid_ports; index++) {
		enum tegra_csi_port_num port_num = chan->port[index];

		if (enable)
			tegra_csi_start_streaming(csi, port_num);
		else
			tegra_csi_stop_streaming(csi, port_num);
	}

	return 0;
}

/*
 * Only use this subdevice media bus ops for test pattern generator,
 * because CSI device is an separated subdevice which has 6 source
 * pads to generate test pattern.
 */
static struct v4l2_mbus_framefmt tegra_csi_tpg_fmts[] = {
	{
		TEGRA_DEF_WIDTH,
		TEGRA_DEF_HEIGHT,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		V4L2_FIELD_NONE,
		V4L2_COLORSPACE_SRGB
	},
	{
		TEGRA_DEF_WIDTH,
		TEGRA_DEF_HEIGHT,
		MEDIA_BUS_FMT_RGBA8888_4X8_LE,
		V4L2_FIELD_NONE,
		V4L2_COLORSPACE_SRGB
	}

};

static struct v4l2_frmsize_discrete tegra_csi_tpg_sizes[] = {
	{1280, 720},
	{1920, 1080},
	{3840, 2160}
};

static int tegra_csi_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse) __maybe_unused;
static int tegra_csi_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse)
{
	int i;
	struct tegra_csi_device *csi = to_csi(sd);

	if (!csi->pg_mode) {
		dev_err(csi->dev, "CSI is not in TPG mode\n");
		return -EINVAL;
	}

	if (fse->index >= ARRAY_SIZE(tegra_csi_tpg_sizes))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tegra_csi_tpg_fmts); i++) {
		const struct tegra_video_format *format =
		      tegra_core_get_format_by_code(tegra_csi_tpg_fmts[i].code);
		if (format && format->fourcc == fse->code)
			break;
	}
	if (i == ARRAY_SIZE(tegra_csi_tpg_fmts))
		return -EINVAL;

	fse->min_width = fse->max_width =
			tegra_csi_tpg_sizes[fse->index].width;
	fse->min_height = fse->max_height =
			tegra_csi_tpg_sizes[fse->index].height;
	return 0;
}

#define TPG_PIXEL_OUTPUT_RATE 182476800

static int tegra_csi_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie) __maybe_unused;
static int tegra_csi_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int i;
	struct tegra_csi_device *csi = to_csi(sd);

	if (!csi->pg_mode) {
		dev_err(csi->dev, "CSI is not in TPG mode\n");
		return -EINVAL;
	}

	/* One resolution just one framerate */
	if (fie->index > 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tegra_csi_tpg_fmts); i++) {
		const struct tegra_video_format *format =
		      tegra_core_get_format_by_code(tegra_csi_tpg_fmts[i].code);
		if (format && format->fourcc == fie->code)
			break;
	}
	if (i == ARRAY_SIZE(tegra_csi_tpg_fmts))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(tegra_csi_tpg_sizes); i++) {
		if (tegra_csi_tpg_sizes[i].width == fie->width &&
		    tegra_csi_tpg_sizes[i].height == fie->height)
			break;
	}
	if (i == ARRAY_SIZE(tegra_csi_tpg_sizes))
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = TPG_PIXEL_OUTPUT_RATE /
		   (fie->width * fie->height);
	return 0;
}

static int tegra_csi_try_mbus_fmt(struct v4l2_subdev *sd,
				  struct v4l2_mbus_framefmt *mf)
{
	int i, j;
	struct tegra_csi_device *csi = to_csi(sd);
	static struct v4l2_frmsize_discrete *sizes;

	if (!csi->pg_mode) {
		dev_err(csi->dev, "CSI is not in TPG mode\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(tegra_csi_tpg_fmts); i++) {
		struct v4l2_mbus_framefmt *fmt = &tegra_csi_tpg_fmts[i];

		if (mf->code == fmt->code && mf->field == fmt->field &&
		    mf->colorspace == fmt->colorspace) {
			for (j = 0; j < ARRAY_SIZE(tegra_csi_tpg_sizes); j++) {
				sizes = &tegra_csi_tpg_sizes[j];
				if (mf->width == sizes->width &&
				    mf->height == sizes->height)
					return 0;
			}
		}
	}

	memcpy(mf, tegra_csi_tpg_fmts, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tegra_csi_g_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	return tegra_csi_try_mbus_fmt(sd, fmt);
}

static int tegra_csi_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tegra_csi_device *csi = to_csi(sd);

	*status = !!csi->pg_mode;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int tegra_csi_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt) __maybe_unused;
static int tegra_csi_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt mbus_fmt;
	int ret;

	ret = tegra_csi_g_mbus_fmt(subdev, &mbus_fmt);
	if (ret)
		return ret;

	fmt->format = mbus_fmt;

	return 0;
}

static int tegra_csi_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt) __maybe_unused;
static int tegra_csi_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	int ret;
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct v4l2_mbus_framefmt *format;

	ret = tegra_csi_try_mbus_fmt(subdev, &fmt->format);
	if (ret)
		return ret;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	format = &chan->ports[0].format;
	memcpy(format, &fmt->format, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops tegra_csi_video_ops = {
	.s_stream	= tegra_csi_s_stream,
	.g_input_status = tegra_csi_g_input_status,
};

static struct v4l2_subdev_pad_ops tegra_csi_pad_ops = {
#if 0
	.get_fmt	= tegra_csi_get_format,
	.set_fmt	= tegra_csi_set_format,
	.enum_frame_size = tegra_csi_enum_framesizes,
	.enum_frame_interval = tegra_csi_enum_frameintervals,
#endif
};

static struct v4l2_subdev_ops tegra_csi_ops = {
	.video  = &tegra_csi_video_ops,
	.pad	= &tegra_csi_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations tegra_csi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Platform Device Driver
 */

static int tegra_csi_get_port_info(struct tegra_csi_channel *chan,
				struct device_node *node, unsigned int index)
{
	struct device_node *ep = NULL;
	struct device_node *ports;
	struct device_node *port;
	struct device_node *chan_dt;

	int value = 0xFFFF;
	int ret = 0, i;

	for_each_child_of_node(node, chan_dt) {
		if (!chan_dt->name || of_node_cmp(chan_dt->name, "channel"))
			continue;
		ret = of_property_read_u32(chan_dt, "reg", &value);
		if (value == index)
			break;
	}

	chan->subdev.of_node = chan_dt;
	ports = of_get_child_by_name(chan_dt, "ports");
	if (ports == NULL)
		return -EINVAL;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;
		ret = of_property_read_u32(port, "reg", &value);
		if (ret < 0)
			continue;
		if (value != 0)
			continue;

		for_each_child_of_node(port, ep) {
			if (!ep->name || of_node_cmp(ep->name, "endpoint"))
				continue;
			ret = of_property_read_u32(ep, "csi-port", &value);
			if (ret < 0)
				dev_err(chan->csi->dev, "No csi port info\n");
			chan->port[0] = value;

			ret = of_property_read_u32(ep, "bus-width", &value);
			if (ret < 0)
				dev_err(chan->csi->dev, "No bus width info\n");
			chan->numlanes = value;
			chan->numports = value >> 1;
			if (value > 12) {
				dev_err(chan->csi->dev, "Invalid num lanes\n");
				return -EINVAL;
			}
			/*
			 * for numlanes greater than 4 multiple CSI bricks
			 * are needed to capture the image, the logic below
			 * checks for numlanes > 4 and add a new CSI brick
			 * as a valid port. Loops around the three CSI
			 * bricks to add as many ports necessary.
			 */
			value -= 4;
			for (i = 1; value > 0; i++, value -= 4) {
				int next_port = chan->port[i-1] + 2;

				next_port = (next_port % (PORT_F + 1));
				chan->port[i] = next_port;
			}
		}
	}
	return 0;
}

static int tegra_tpg_channel_init(struct tegra_csi_device *csi,
				    struct platform_device *pdev,
				    unsigned int index)
{
	struct tegra_csi_channel *chan = &csi->chans[index];

	chan->numlanes = 2;
	chan->numports = 1;
	return 0;
}

int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	csi->dev = &pdev->dev;
	err = set_csi_properties(csi, pdev);
	if (err)
		return err;

	set_csi_registers(csi, pdata->aperture[0]);

	err = csi_get_clks(csi, pdev);
	if (err)
		dev_err(&pdev->dev, "Failed to get CSI clks\n");

	return err;
}

static int tegra_csi_channel_init_one(struct tegra_csi_device *csi,
					unsigned int index)
{
	struct tegra_csi_channel *chan = &csi->chans[index];
	struct v4l2_subdev *sd;
	int i, ret;

	chan->csi = csi;
	sd = &chan->subdev;
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, &tegra_csi_ops);
	sd->dev = csi->dev;
	snprintf(sd->name, sizeof(sd->name), "%s-%d",
			dev_name(csi->dev), index);
	v4l2_set_subdevdata(sd, csi);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &tegra_csi_media_ops;

	chan->ports = devm_kzalloc(csi->dev,
			chan->numports * sizeof(struct tegra_csi_port),
			GFP_KERNEL);
	/* Initialize the default format */
	for (i = 0; i < chan->numports; i++) {
		chan->ports[i].format.code = TEGRA_VF_DEF;
		chan->ports[i].format.field = V4L2_FIELD_NONE;
		chan->ports[i].format.colorspace = V4L2_COLORSPACE_SRGB;
		chan->ports[i].format.width = TEGRA_DEF_WIDTH;
		chan->ports[i].format.height = TEGRA_DEF_HEIGHT;
	}
	if (csi->pg_mode) {
		chan->port[0] = index;
		chan->ports[0].num = index;
		chan->ports[0].lanes = 2;

		chan->pads = devm_kzalloc(csi->dev, sizeof(*chan->pads),
				GFP_KERNEL);
		if (!chan->pads)
			return -ENOMEM;
		chan->pg_mode = true;
		chan->pads[0].flags = MEDIA_PAD_FL_SOURCE;
	} else {
		chan->pads = devm_kzalloc(csi->dev, 2 * sizeof(*chan->pads),
			GFP_KERNEL);
		if (!chan->pads)
			return -ENOMEM;
		chan->pads[0].flags = MEDIA_PAD_FL_SINK;
		chan->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	}
	/* Initialize media entity */
	ret = media_entity_init(&sd->entity,
			chan->pg_mode ? 1 : 2,
			chan->pads, 0);
	if (ret < 0)
		return ret;


	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(csi->dev, "failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}
	return 0;
}

static int tegra_csi_channels_init(struct tegra_csi_device *csi)
{
	int i, ret;

	for (i = 0; i < csi->num_channels; i++) {
		ret = tegra_csi_channel_init_one(csi, i);
		if (ret)
			return ret;
	}
	return 0;
}

static int csi_parse_dt(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	int err = 0, i;
	int num_channels = 0;
	struct device_node *node = pdev->dev.of_node;

	err = of_property_read_u32(node, "num-channels", &num_channels);
	if (err) {
		dev_err(csi->dev, " Faile to find num of channels, set to 1\n");
		num_channels = 1;
	}

	csi->num_channels = num_channels;
	csi->chans = devm_kzalloc(csi->dev,
		(sizeof(*csi->chans) * csi->num_channels),
		GFP_KERNEL);

	if (!csi->chans)
		return -ENOMEM;

	for (i = 0; i < csi->num_channels; i++) {
		csi->chans[i].csi = csi;
		tegra_csi_get_port_info(&csi->chans[i], node, i);
	}

	return 0;
}

int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				    struct platform_device *pdev)
{
	int ret, i;

	csi->dev = &pdev->dev;
	csi->pdev = pdev;
	if (csi->pg_mode) {
		csi->chans = devm_kzalloc(&pdev->dev,
				csi->num_channels * sizeof(*csi->chans),
				GFP_KERNEL);
		for (i = 0 ; i < csi->num_channels; i++)
			ret = tegra_tpg_channel_init(csi, pdev, i);
	} else {
		ret = csi_parse_dt(csi, pdev);
		if (ret < 0)
			return ret;
	}
	ret = tegra_csi_channels_init(csi);
	ret = tegra_csi_init(csi, pdev);
	if (ret < 0)
		dev_err(&pdev->dev, "Failed to init csi property,clks\n");

	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_init);

int tegra_csi_media_controller_remove(struct tegra_csi_device *csi)
{
	struct tegra_csi_channel *chan;
	struct v4l2_subdev *sd;
	int i;

	for (i = 0; i < csi->num_channels; i++) {
		chan = &csi->chans[i];
		sd = &chan->subdev;
		v4l2_async_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_remove);
