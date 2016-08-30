/*
 * Tegra CSI2 device common APIs
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/clk/tegra.h>
#include "camera/csi/csi.h"

static void csi_write(struct tegra_csi_device *csi, unsigned int addr,
			u32 val, u8 port)
{
	writel(val, (csi->iomem[port] + addr));
}

static u32 csi_read(struct tegra_csi_device *csi, unsigned int addr,
			u8 port)
{
	return readl((csi->iomem[port] + addr));
}

/* Pixel parser registers accessors */
static void pp_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
	writel(val, port->pixel_parser + addr);
}

static u32 pp_read(struct tegra_csi_port *port, u32 addr)
{
	return readl(port->pixel_parser + addr);
}

/* CSI CIL registers accessors */
static void cil_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
	writel(val, port->cil + addr);
}

static u32 cil_read(struct tegra_csi_port *port, u32 addr)
{
	return readl(port->cil + addr);
}

/* Test pattern generator registers accessor */
static void tpg_write(struct tegra_csi_port *port,
			unsigned int addr, u32 val)
{
	writel(val, port->tpg + addr);
}

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

		dev_dbg(chan->csi->dev,
			"TEGRA_CSI_PIXEL_PARSER_STATUS 0x%08x\n",
			val);

		val = cil_read(port, TEGRA_CSI_CIL_STATUS);
		dev_dbg(chan->csi->dev,
			"TEGRA_CSI_CIL_STATUS 0x%08x\n", val);

		val = cil_read(port, TEGRA_CSI_CILX_STATUS);
		dev_dbg(chan->csi->dev,
			"TEGRA_CSI_CILX_STATUS 0x%08x\n", val);
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
			struct tegra_csi_port *port_a =
				&chan->ports[port_val];
			struct tegra_csi_port *port_b =
				&chan->ports[port_val+1];

			tpg_write(port_a,
				TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_ENABLE);
			tpg_write(port_b,
				TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_ENABLE);
			cil_write(port_a,
				TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
			cil_write(port_b,
				TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
			csi_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x1,
					port_num >> 1);
			/* sleep for clock cycles to drain the Rx FIFO */
			usleep_range(10, 20);
			cil_write(port_a,
				TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
			cil_write(port_b,
				TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
			csi_write(csi,
				TEGRA_CSI_CSI_SW_STATUS_RESET,
				0x0, port_num >> 1);
			tpg_write(port_a,
				TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_DISABLE);
			tpg_write(port_b,
				TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_DISABLE);
		} else {
			tpg_write(port,
				TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_ENABLE);
			cil_write(port,
				TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
			csi_write(csi,
				TEGRA_CSI_CSI_SW_STATUS_RESET,
				0x1, port_num >> 1);
			/* sleep for clock cycles to drain the Rx FIFO */
			usleep_range(10, 20);
			cil_write(port,
				TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
			csi_write(csi,
				TEGRA_CSI_CSI_SW_STATUS_RESET,
				0x0, port_num >> 1);
			tpg_write(port,
				TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_DISABLE);
		}
	}
}


static int csi2_tpg_start_streaming(struct tegra_csi_device *csi,
			      enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
		       ((csi->pg_mode - 1) << PG_MODE_OFFSET) |
		       PG_ENABLE);
	tpg_write(port, TEGRA_CSI_PG_BLANK,
			port->v_blank << PG_VBLANK_OFFSET |
			port->h_blank);
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
	return 0;
}

int csi2_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
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

	if (csi->pg_mode)
		csi2_tpg_start_streaming(csi, port_num);

	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
			(0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
			CSI_PP_SINGLE_SHOT_ENABLE | CSI_PP_ENABLE);
	return 0;
}

void csi2_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &csi->ports[port_num];

	if (csi->pg_mode)
		tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL, PG_DISABLE);

	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
			(0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
			CSI_PP_DISABLE);
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

int csi2_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

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

	return err;
}

int csi2_power_off(struct tegra_csi_device *csi)
{
	int err = 0;

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

	return err;
}
