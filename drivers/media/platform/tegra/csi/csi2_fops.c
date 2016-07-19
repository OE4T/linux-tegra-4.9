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
#include "csi/csi.h"

#define DEBUG 0

static void csi2_write(struct tegra_csi_device *csi, unsigned int addr,
			   u32 val, u8 port)
{
	dev_dbg(csi->dev, "%s:port %d offset 0x%08x val:0x%08x\n",
				__func__, port, addr, val);
	writel(val, (csi->iomem[port] + addr));
}

static u32 csi2_read(struct tegra_csi_device *csi, unsigned int addr,
			 u8 port)
{
	dev_dbg(csi->dev, "%s:port %d offset 0x%08x\n", __func__, port, addr);
	return readl((csi->iomem[port] + addr));
}

/* Pixel parser registers accessors */
static void csi2_pp_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
	pr_debug("%s:offset 0x%08x val:0x%08x\n", __func__, addr, val);
	writel(val, port->pixel_parser + addr);
}

static u32 csi2_pp_read(struct tegra_csi_port *port, u32 addr)
{
	pr_debug("%s:offset 0x%08x\n", __func__, addr);
	return readl(port->pixel_parser + addr);
}

/* CSI CIL registers accessors */
static void csi2_cil_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
	pr_debug("%s:offset 0x%08x val:0x%08x\n", __func__, addr, val);
	writel(val, port->cil + addr);
}

static u32 csi2_cil_read(struct tegra_csi_port *port, u32 addr)
{
	pr_debug("%s:offset 0x%08x\n", __func__, addr);
	return readl(port->cil + addr);
}

/* Test pattern generator registers accessor */
static void csi2_tpg_write(struct tegra_csi_port *port,
			       unsigned int addr, u32 val)
{
	writel(val, port->tpg + addr);
}

void csi2_tpg_start_streaming(struct tegra_csi_device *csi,
			      enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	csi2_tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
		       ((csi->pg_mode - 1) << PG_MODE_OFFSET) |
		       PG_ENABLE);
	csi2_tpg_write(port, TEGRA_CSI_PG_PHASE, 0x0);
	csi2_tpg_write(port, TEGRA_CSI_PG_RED_FREQ,
		       (0x10 << PG_RED_VERT_INIT_FREQ_OFFSET) |
		       (0x10 << PG_RED_HOR_INIT_FREQ_OFFSET));
	csi2_tpg_write(port, TEGRA_CSI_PG_RED_FREQ_RATE, 0x0);
	csi2_tpg_write(port, TEGRA_CSI_PG_GREEN_FREQ,
		       (0x10 << PG_GREEN_VERT_INIT_FREQ_OFFSET) |
		       (0x10 << PG_GREEN_HOR_INIT_FREQ_OFFSET));
	csi2_tpg_write(port, TEGRA_CSI_PG_GREEN_FREQ_RATE, 0x0);
	csi2_tpg_write(port, TEGRA_CSI_PG_BLUE_FREQ,
		       (0x10 << PG_BLUE_VERT_INIT_FREQ_OFFSET) |
		       (0x10 << PG_BLUE_HOR_INIT_FREQ_OFFSET));
	csi2_tpg_write(port, TEGRA_CSI_PG_BLUE_FREQ_RATE, 0x0);
}

void csi2_start_streaming(struct tegra_csi_device *csi,
			  enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	csi2_write(csi, TEGRA_CSI_CLKEN_OVERRIDE, 0, port_num >> 1);

	/* Clean up status */
	csi2_pp_write(port, TEGRA_CSI_PIXEL_PARSER_STATUS, 0xFFFFFFFF);
	csi2_cil_write(port, TEGRA_CSI_CIL_STATUS, 0xFFFFFFFF);
	csi2_cil_write(port, TEGRA_CSI_CILX_STATUS, 0xFFFFFFFF);

	csi2_cil_write(port, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);

	/* CIL PHY registers setup */
	csi2_cil_write(port, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
	csi2_cil_write(port, TEGRA_CSI_CIL_PHY_CONTROL,
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

		csi2_cil_write(port_a, TEGRA_CSI_CIL_PAD_CONFIG0,
			       BRICK_CLOCK_A_4X);
		csi2_cil_write(port_b, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
		csi2_cil_write(port_b, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);
		csi2_cil_write(port_a, TEGRA_CSI_CIL_PHY_CONTROL,
			       BYPASS_LP_SEQ | 0xA);
		csi2_cil_write(port_b, TEGRA_CSI_CIL_PHY_CONTROL,
			       BYPASS_LP_SEQ | 0xA);
		csi2_write(csi, TEGRA_CSI_PHY_CIL_COMMAND,
			   CSI_A_PHY_CIL_ENABLE | CSI_B_PHY_CIL_ENABLE,
			   port_num >> 1);
	} else {
		u32 val = csi2_read(csi, TEGRA_CSI_PHY_CIL_COMMAND,
					port_num >> 1);
		int port_val = ((port_num >> 1) << 1);
		struct tegra_csi_port *port_a = &csi->ports[port_val];

		csi2_cil_write(port_a, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
		val |= ((port->num & 0x1) == PORT_A) ? CSI_A_PHY_CIL_ENABLE :
			CSI_B_PHY_CIL_ENABLE;
		csi2_write(csi, TEGRA_CSI_PHY_CIL_COMMAND, val,
			   port_num >> 1);
	}

	/* CSI pixel parser registers setup */
	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
		      (0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
		      CSI_PP_SINGLE_SHOT_ENABLE | CSI_PP_RST);
	csi2_pp_write(port, TEGRA_CSI_PIXEL_PARSER_INTERRUPT_MASK, 0x0);
	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_CONTROL0,
		      CSI_PP_PACKET_HEADER_SENT |
		      CSI_PP_DATA_IDENTIFIER_ENABLE |
		      CSI_PP_WORD_COUNT_SELECT_HEADER |
		      CSI_PP_CRC_CHECK_ENABLE |  CSI_PP_WC_CHECK |
		      CSI_PP_OUTPUT_FORMAT_STORE | CSI_PPA_PAD_LINE_NOPAD |
		      CSI_PP_HEADER_EC_DISABLE | CSI_PPA_PAD_FRAME_NOPAD |
		      (port->num & 1));
	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_CONTROL1,
		      (0x1 << CSI_PP_TOP_FIELD_FRAME_OFFSET) |
		      (0x1 << CSI_PP_TOP_FIELD_FRAME_MASK_OFFSET));
	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_GAP,
		      0x14 << PP_FRAME_MIN_GAP_OFFSET);
	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_EXPECTED_FRAME, 0x0);
	csi2_pp_write(port, TEGRA_CSI_INPUT_STREAM_CONTROL,
		      (0x3f << CSI_SKIP_PACKET_THRESHOLD_OFFSET) |
		      (port->lanes - 1));
#if DEBUG
	/* 0x454140E1 - register setting for line counter */
	/* 0x454340E1 - tracks frame start, line starts, hpa headers */
	csi2_pp_write(port, TEGRA_CSI_DEBUG_CONTROL, 0x454340E1);
#endif
	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
		      (0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
		      CSI_PP_SINGLE_SHOT_ENABLE | CSI_PP_ENABLE);
}

int csi2_error(struct tegra_csi_device *csi,
	       enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];
	u32 val;
	int err = 0;

	/*
	 * only uncorrectable header error and multi-bit
	 * transmission errors are checked as they cannot be
	 * corrected automatically
	*/
	val = csi2_pp_read(port, TEGRA_CSI_PIXEL_PARSER_STATUS);
	err |= val & 0x4000;
	csi2_pp_write(port, TEGRA_CSI_PIXEL_PARSER_STATUS, val);

	val = csi2_cil_read(port, TEGRA_CSI_CIL_STATUS);
	err |= val & 0x02;
	csi2_cil_write(port, TEGRA_CSI_CIL_STATUS, val);

	val = csi2_cil_read(port, TEGRA_CSI_CILX_STATUS);
	err |= val & 0x00020020;
	csi2_cil_write(port, TEGRA_CSI_CILX_STATUS, val);

	return err;
}

void csi2_status(struct tegra_csi_device *csi,
		 enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];
	u32 val = csi2_pp_read(port, TEGRA_CSI_PIXEL_PARSER_STATUS);

	dev_dbg(csi->dev, "TEGRA_CSI_PIXEL_PARSER_STATUS 0x%08x\n",
		val);

	val = csi2_cil_read(port, TEGRA_CSI_CIL_STATUS);
	dev_dbg(csi->dev, "TEGRA_CSI_CIL_STATUS 0x%08x\n", val);

	val = csi2_cil_read(port, TEGRA_CSI_CILX_STATUS);
	dev_dbg(csi->dev, "TEGRA_CSI_CILX_STATUS 0x%08x\n", val);

#if DEBUG
	val = csi2_pp_read(port, TEGRA_CSI_DEBUG_COUNTER_0);
	dev_dbg(csi->dev, "TEGRA_CSI_DEBUG_COUNTER_0 0x%08x\n", val);
	val = csi2_pp_read(port, TEGRA_CSI_DEBUG_COUNTER_1);
	dev_dbg(csi->dev, "TEGRA_CSI_DEBUG_COUNTER_1 0x%08x\n", val);
	val = csi2_pp_read(port, TEGRA_CSI_DEBUG_COUNTER_2);
	dev_dbg(csi->dev, "TEGRA_CSI_DEBUG_COUNTER_2 0x%08x\n", val);
#endif
}

void csi2_error_recover(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	if (port->lanes == 4) {
		int port_val = ((port_num >> 1) << 1);
		struct tegra_csi_port *port_a = &csi->ports[port_val];
		struct tegra_csi_port *port_b = &csi->ports[port_val+1];
		csi2_tpg_write(port_a, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_ENABLE);
		csi2_tpg_write(port_b, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_ENABLE);
		csi2_cil_write(port_a, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
		csi2_cil_write(port_b, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
		csi2_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x1,
			   port_num >> 1);
		/* sleep for clock cycles to drain the Rx FIFO */
		usleep_range(10, 20);
		csi2_cil_write(port_a, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
		csi2_cil_write(port_b, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
		csi2_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x0,
			   port_num >> 1);
		csi2_tpg_write(port_a, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_DISABLE);
		csi2_tpg_write(port_b, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_DISABLE);
	} else {
		csi2_tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_ENABLE);
		csi2_cil_write(port, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x1);
		csi2_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x1,
			   port_num >> 1);
		/* sleep for clock cycles to drain the Rx FIFO */
		usleep_range(10, 20);
		csi2_cil_write(port, TEGRA_CSI_CIL_SW_SENSOR_RESET, 0x0);
		csi2_write(csi, TEGRA_CSI_CSI_SW_STATUS_RESET, 0x0,
			   port_num >> 1);
		csi2_tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_DISABLE);
	}
}

void csi2_stop_streaming(struct tegra_csi_device *csi,
			 enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	if (csi->pg_mode)
		csi2_tpg_write(port, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
			       PG_DISABLE);

	csi2_pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
		      (0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
		      CSI_PP_DISABLE);
}
