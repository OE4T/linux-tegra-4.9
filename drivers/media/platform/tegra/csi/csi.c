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

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/camera_common.h>

#include "dev.h"
#include "vi/vi.h"
#include "csi.h"

#define DEBUG 0

static void csi_write(struct tegra_csi_device *csi, unsigned int addr,
		u32 val, u8 port)
{
#if DEBUG
	dev_info(csi->dev, "%s:port %d offset 0x%08x val:0x%08x\n",
				__func__, port, addr, val);
#endif
	writel(val, (csi->iomem[port] + addr));
}

static u32 csi_read(struct tegra_csi_device *csi, unsigned int addr,
		u8 port)
{
#if DEBUG
	dev_info(csi->dev, "%s:port %d offset 0x%08x\n", __func__, port, addr);
#endif
	return readl((csi->iomem[port] + addr));
}

/* Pixel parser registers accessors */
static void pp_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
#if DEBUG
	pr_info("%s:offset 0x%08x val:0x%08x\n", __func__, addr, val);
#endif
	writel(val, port->pixel_parser + addr);
}

static u32 pp_read(struct tegra_csi_port *port, u32 addr)
{
#if DEBUG
	pr_info("%s:offset 0x%08x\n", __func__, addr);
#endif
	return readl(port->pixel_parser + addr);
}

/* CSI CIL registers accessors */
static void cil_write(struct tegra_csi_port *port, u32 addr, u32 val)
{
#if DEBUG
	pr_info("%s:offset 0x%08x val:0x%08x\n", __func__, addr, val);
#endif
	writel(val, port->cil + addr);
}

static u32 cil_read(struct tegra_csi_port *port, u32 addr)
{
#if DEBUG
	pr_info("%s:offset 0x%08x\n", __func__, addr);
#endif
	return readl(port->cil + addr);
}

/* Test pattern generator registers accessor */
static void tpg_write(struct tegra_csi_port *port, unsigned int addr, u32 val)
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

	csi->pg_mode = 0;
	/*
	* These values are only used for tpg mode
	* With sensor, CSI power and clock info are provided
	* by the sensor sub device
	*/
	s_data->csi_port = 0;
	s_data->numlanes = 12;
	csi->clk_freq = TEGRA_CLOCK_CSI_PORT_MAX;

	if (!csi->ports) {
		int port_num = (s_data->numlanes >> 1);
		csi->ports = devm_kzalloc(&pdev->dev,
			(port_num * sizeof(struct tegra_csi_port)),
			GFP_KERNEL);
		if (!csi->ports)
			return -ENOMEM;
		csi->num_ports = port_num;
	}

	return 0;
}

void set_csi_portinfo(struct tegra_csi_device *csi,
	unsigned int port, unsigned int numlanes)
{
	struct camera_common_data *s_data = &csi->s_data[port];

	s_data->csi_port = port;
	s_data->numlanes = numlanes;
	s_data->def_clk_freq = TEGRA_CLOCK_CSI_PORT_MAX;
	csi->ports[port].lanes = numlanes;
}
EXPORT_SYMBOL(set_csi_portinfo);

static void set_csi_registers(struct tegra_csi_device *csi,
		void __iomem *regbase)
{
	int i, j, idx;

	csi->iomem[0] = (regbase + TEGRA_CSI_PIXEL_PARSER_0_BASE);
	csi->iomem[1] = (regbase + TEGRA_CSI_PIXEL_PARSER_2_BASE);
	csi->iomem[2] = (regbase + TEGRA_CSI_PIXEL_PARSER_4_BASE);

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
}

static int clock_start(struct clk *clk, unsigned int freq)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		pr_err("csi clk enable error\n");
	err = clk_set_rate(clk, freq);
	if (err)
		pr_err("cil clk set error\n");

	return err;
}

int tegra_csi_power(struct tegra_csi_device *csi, int port_num, int enable)
{
	int err = 0;
	int port = port_num >> 1;

	if (enable) {
		/* set clk and power */
		err = clk_prepare_enable(csi->clk);
		if (err)
			pr_err("csi clk error\n");

		err = clock_start(csi->cil[port], csi->clk_freq);
		if (err)
			pr_err("cil clk error\n");

		if (csi->pg_mode) {
			err = clock_start(csi->tpg_clk, TEGRA_CLOCK_TPG_MAX);
			if (err)
				pr_err("tpg clk set error\n");
			else {
				tegra_clk_cfg_ex(csi->tpg_clk,
					TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
				tegra_clk_cfg_ex(csi->tpg_clk,
					TEGRA_CLK_PLLD_DSI_OUT_ENB, 1);
				tegra_clk_cfg_ex(csi->tpg_clk,
					TEGRA_CLK_MIPI_CSI_OUT_ENB, 0);
			}
			camera_common_dpd_disable(&csi->s_data[port]);
		}
	} else {
		if (csi->pg_mode) {
			camera_common_dpd_enable(&csi->s_data[port]);
			tegra_clk_cfg_ex(csi->tpg_clk,
					 TEGRA_CLK_MIPI_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(csi->tpg_clk,
					 TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
			tegra_clk_cfg_ex(csi->tpg_clk,
					 TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
			clk_disable_unprepare(csi->tpg_clk);
		}
		clk_disable_unprepare(csi->cil[port]);
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

void tegra_csi_start_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	csi_write(csi, TEGRA_CSI_CLKEN_OVERRIDE, 0, port_num>>1);

	/* Clean up status */
	pp_write(port, TEGRA_CSI_PIXEL_PARSER_STATUS, 0xFFFFFFFF);
	cil_write(port, TEGRA_CSI_CIL_STATUS, 0xFFFFFFFF);
	cil_write(port, TEGRA_CSI_CILX_STATUS, 0xFFFFFFFF);

	cil_write(port, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);

	/* CIL PHY registers setup */
	cil_write(port, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
	cil_write(port, TEGRA_CSI_CIL_PHY_CONTROL, 0xA);

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
		struct tegra_csi_port *port_b = &csi->ports[port_val+1];

		cil_write(port_a, TEGRA_CSI_CIL_PAD_CONFIG0,
			  BRICK_CLOCK_A_4X);
		cil_write(port_b, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
		cil_write(port_b, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);
		cil_write(port_a, TEGRA_CSI_CIL_PHY_CONTROL, 0xA);
		cil_write(port_b, TEGRA_CSI_CIL_PHY_CONTROL, 0xA);
		csi_write(csi, TEGRA_CSI_PHY_CIL_COMMAND,
			CSI_A_PHY_CIL_ENABLE | CSI_B_PHY_CIL_ENABLE,
			port_num>>1);
	} else {
		u32 val = csi_read(csi, TEGRA_CSI_PHY_CIL_COMMAND, port_num>>1);
		int port_val = ((port_num >> 1) << 1);
		struct tegra_csi_port *port_a = &csi->ports[port_val];

		cil_write(port_a, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
		val |= ((port->num & 0x1) == PORT_A) ? CSI_A_PHY_CIL_ENABLE :
			CSI_B_PHY_CIL_ENABLE;
		csi_write(csi, TEGRA_CSI_PHY_CIL_COMMAND, val, port_num>>1);
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
		 CSI_PP_OUTPUT_FORMAT_STORE |
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

	/* Test Pattern Generator setup */
	if (csi->pg_mode) {
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

#if DEBUG
	/* 0x454140E1 - register setting for line counter */
	pp_write(port, TEGRA_CSI_DEBUG_CONTROL, 0x454140E1);
#endif
	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
		 (0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
		 CSI_PP_SINGLE_SHOT_ENABLE | CSI_PP_ENABLE);
}
EXPORT_SYMBOL(tegra_csi_start_streaming);

void tegra_csi_status(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];
	u32 val = pp_read(port, TEGRA_CSI_PIXEL_PARSER_STATUS);

	dev_info(csi->dev, "TEGRA_CSI_PIXEL_PARSER_STATUS 0x%08x\n",
		val);

	val = cil_read(port, TEGRA_CSI_CIL_STATUS);
	dev_info(csi->dev, "TEGRA_CSI_CIL_STATUS 0x%08x\n", val);

	val = cil_read(port, TEGRA_CSI_CILX_STATUS);
	dev_info(csi->dev, "TEGRA_CSI_CILX_STATUS 0x%08x\n", val);

#if DEBUG
	val = pp_read(port, TEGRA_CSI_DEBUG_COUNTER_0);
	dev_info(csi->dev, "TEGRA_CSI_DEBUG_COUNTER_0 0x%08x\n", val);
	val = pp_read(port, TEGRA_CSI_DEBUG_COUNTER_1);
	dev_info(csi->dev, "TEGRA_CSI_DEBUG_COUNTER_1 0x%08x\n", val);
	val = pp_read(port, TEGRA_CSI_DEBUG_COUNTER_2);
	dev_info(csi->dev, "TEGRA_CSI_DEBUG_COUNTER_2 0x%08x\n", val);
#endif
}
EXPORT_SYMBOL(tegra_csi_status);

void tegra_csi_stop_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_port *port = &csi->ports[port_num];

	pp_write(port, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND,
		 (0xF << CSI_PP_START_MARKER_FRAME_MAX_OFFSET) |
		 CSI_PP_DISABLE);
}
EXPORT_SYMBOL(tegra_csi_stop_streaming);

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_csi_device *csi = to_csi(subdev);
	struct tegra_channel *chan = subdev->host_priv;
	enum tegra_csi_port_num port_num = chan->port;

	if (enable)
		tegra_csi_start_streaming(csi, port_num);
	else
		tegra_csi_stop_streaming(csi, port_num);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static struct v4l2_mbus_framefmt *
__tegra_csi_get_pad_format(struct tegra_csi_device *csi,
		      struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, u32 which)
{
	enum tegra_csi_port_num port_num = (pad >> 1);

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
		return v4l2_subdev_get_try_format(&csi->subdev, cfg, pad);
#endif
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &csi->ports[port_num].format;
	default:
		return NULL;
	}
}

static int tegra_csi_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct tegra_csi_device *csi = to_csi(subdev);

	fmt->format = *__tegra_csi_get_pad_format(csi, cfg,
					fmt->pad, fmt->which);
	return 0;
}

static int tegra_csi_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct tegra_csi_device *csi = to_csi(subdev);
	struct v4l2_mbus_framefmt *__format;
	enum tegra_csi_port_num port_num = (fmt->pad >> 1);

	__format = __tegra_csi_get_pad_format(csi, cfg,
					fmt->pad, fmt->which);
	if (__format)
		csi->ports[port_num].format = *__format;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops tegra_csi_video_ops = {
	.s_stream = tegra_csi_s_stream,
};

static struct v4l2_subdev_pad_ops tegra_csi_pad_ops = {
	.get_fmt		= tegra_csi_get_format,
	.set_fmt		= tegra_csi_set_format,
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

static int tegra_csi_parse_of(struct tegra_csi_device *csi,
			struct platform_device *pdev)
{
	struct device_node *node = csi->dev->of_node;
	struct device_node *ports;
	struct device_node *port;
	struct device_node *ep;
	unsigned int lanes, pad_num, port_num;
	int ret;

	ret = of_property_read_u32(node, "num-ports", &port_num);
	if (ret < 0)
		return ret;

	csi->ports = devm_kzalloc(&pdev->dev,
		(port_num * sizeof(struct tegra_csi_port)), GFP_KERNEL);
	if (!csi->ports)
		return -ENOMEM;
	csi->num_ports = port_num;

	csi->pads = devm_kzalloc(&pdev->dev,
		(port_num * 2 * sizeof(struct media_pad)), GFP_KERNEL);
	if (!csi->pads)
		return -ENOMEM;

	ports = of_get_child_by_name(node, "ports");
	if (ports == NULL)
		ports = node;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;

		ret = of_property_read_u32(port, "reg", &pad_num);
		if (ret < 0)
			continue;
		port_num = (pad_num >> 1);
		csi->ports[port_num].num = port_num;

		for_each_child_of_node(port, ep) {
			if (!ep->name || of_node_cmp(ep->name, "endpoint"))
				continue;

			/* Get number of data lanes for the first endpoint */
			ret = of_property_read_u32(ep, "bus-width", &lanes);
			if (ret < 0)
				lanes = 4;
			csi->ports[port_num].lanes = lanes;
		}
	}

	return 0;
}

int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = pdev->dev.platform_data;

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
EXPORT_SYMBOL(tegra_csi_init);

int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				struct platform_device *pdev)
{
	struct v4l2_subdev *subdev;
	int ret, i;

	csi->dev = &pdev->dev;

	ret = tegra_csi_parse_of(csi, pdev);
	if (ret < 0)
		return ret;

	ret = tegra_csi_init(csi, pdev);
	if (ret < 0)
		return ret;

	/* Initialize V4L2 subdevice and media entity */
	subdev = &csi->subdev;
	v4l2_subdev_init(subdev, &tegra_csi_ops);
	subdev->dev = &pdev->dev;
	strlcpy(subdev->name, dev_name(&pdev->dev), sizeof(subdev->name));
	v4l2_set_subdevdata(subdev, csi);
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	subdev->entity.ops = &tegra_csi_media_ops;

	for (i = 0; i < csi->num_ports; i++) {
		/* Initialize the default format */
		csi->ports[i].format.code = TEGRA_VF_DEF;
		csi->ports[i].format.field = V4L2_FIELD_NONE;
		csi->ports[i].format.colorspace = V4L2_COLORSPACE_SRGB;
		csi->ports[i].format.width = TEGRA_DEF_WIDTH;
		csi->ports[i].format.height = TEGRA_DEF_HEIGHT;

		csi->pads[i * 2].flags = MEDIA_PAD_FL_SINK;
		csi->pads[i * 2 + 1].flags = MEDIA_PAD_FL_SOURCE;
	}

	/* Initialize media entity */
	ret = media_entity_init(&subdev->entity, csi->num_ports * 2,
			csi->pads, 0);
	if (ret < 0)
		return ret;

	ret = v4l2_async_register_subdev(subdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register subdev\n");
		goto error;
	}

	return 0;

error:
	media_entity_cleanup(&subdev->entity);
	return ret;
}
EXPORT_SYMBOL(tegra_csi_media_controller_init);

int tegra_csi_media_controller_remove(struct tegra_csi_device *csi)
{
	struct v4l2_subdev *subdev = &csi->subdev;

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_remove);
