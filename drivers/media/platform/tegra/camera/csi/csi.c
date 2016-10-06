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
#include "camera/csi/csi.h"
#include "camera/vi/mc_common.h"
#include "mipical/mipi_cal.h"
#include "linux/nvhost.h"
#include "nvcsi/nvcsi.h"

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

static void update_blank_intervals(struct tegra_csi_channel *chan,
		int portnum, int fmtindex)
{
	struct tegra_csi_port *port = &chan->ports[portnum];
	const struct tpg_frmfmt *tegra_csi_tpg_frmfmt =
						chan->csi->tpg_frmfmt_table;

	port->framerate = tegra_csi_tpg_frmfmt[fmtindex].framerate;
	port->h_blank = tegra_csi_tpg_frmfmt[fmtindex].h_blank;
	port->v_blank = tegra_csi_tpg_frmfmt[fmtindex].v_blank;
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
	csi->iomem_base = regbase;
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

int tegra_csi_power(struct tegra_csi_device *csi, int enable)
{
	int err = 0;

	if (enable) {
		err = csi->fops->csi_power_on(csi);
		if (!err)
			atomic_inc(&csi->power_ref);
	} else {
		err = csi->fops->csi_power_off(csi);
		if (!err)
			atomic_dec(&csi->power_ref);
	}
	return err;
}
EXPORT_SYMBOL(tegra_csi_power);

int tegra_csi_s_power(struct v4l2_subdev *subdev, int enable)
{
	int err = 0;
	struct tegra_csi_device *csi = to_csi(subdev);

	err = tegra_csi_power(csi, enable);

	return err;
}

/*
 * -----------------------------------------------------------------------------
 * CSI Subdevice Video Operations
 * -----------------------------------------------------------------------------
 */

int tegra_csi_start_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	return csi->fops->csi_start_streaming(chan, port_num);
}
EXPORT_SYMBOL(tegra_csi_start_streaming);

void tegra_csi_stop_streaming(struct tegra_csi_channel *chan,
				enum tegra_csi_port_num port_num)
{
	struct tegra_csi_device *csi = chan->csi;

	csi->fops->csi_stop_streaming(chan, port_num);
}
EXPORT_SYMBOL(tegra_csi_stop_streaming);

int csi_mipi_cal(struct tegra_channel *chan, char is_bypass)
{
	unsigned int lanes, cur_lanes;
	unsigned int cila, cilb;
	struct tegra_csi_device *csi;
	int j;

	csi = tegra_get_mc_csi();
	if (!csi)
		return -EINVAL;

	lanes = 0;

	if (chan->numlanes == 2 && chan->total_ports == 1) {
		cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << E_INPUT_LP_CLK_SHIFT) |
			(0x00 << PD_CLK_SHIFT) |
			(0x00 << PD_IO0_SHIFT) |
			(0x00 << PD_IO1_SHIFT);
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
			(0x01 << E_INPUT_LP_CLK_SHIFT) |
			(0x00 << PD_CLK_SHIFT) |
			(0x00 << PD_IO0_SHIFT) |
			(0x00 << PD_IO1_SHIFT);
		cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << PD_CLK_SHIFT) |
			(0x00 << PD_IO0_SHIFT) |
			(0x00 << PD_IO1_SHIFT);
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
			(0x01 << E_INPUT_LP_CLK_SHIFT) |
			(0x00 << PD_CLK_SHIFT) |
			(0x00 << PD_IO0_SHIFT) |
			(0x00 << PD_IO1_SHIFT);
		cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
			(0x01 << E_INPUT_LP_IO1_SHIFT) |
			(0x01 << PD_CLK_SHIFT) |
			(0x00 << PD_IO0_SHIFT) |
			(0x00 << PD_IO1_SHIFT);
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
		dev_err(csi->dev, "Selected no CSI lane, can't do calibration");
		return -EINVAL;
	}
	return tegra_mipi_calibration(lanes);
}
EXPORT_SYMBOL(csi_mipi_cal);

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_csi_device *csi;
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct tegra_channel *tegra_chan = v4l2_get_subdev_hostdata(subdev);
	int i, ret = 0;

	if (tegra_chan->bypass)
		return 0;

	csi = to_csi(subdev);
	if (!csi)
		return -EINVAL;

	for (i = 0; i < tegra_chan->valid_ports; i++) {
		if (enable) {
			ret = tegra_csi_start_streaming(chan, i);
			if (ret)
				tegra_csi_stop_streaming(chan, i);
		} else
			tegra_csi_stop_streaming(chan, i);
	}

	return ret;
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
		MEDIA_BUS_FMT_RGB888_1X32_PADHI,
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
		struct v4l2_subdev_frame_size_enum *fse)
{
	int i;
	struct tegra_csi_channel *chan = to_csi_chan(sd);

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

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

static int tegra_csi_get_fmtindex(struct tegra_csi_channel *chan,
				int width, int height, int pixel_format)
{
	int i;
	const struct tpg_frmfmt *tegra_csi_tpg_frmfmt =
						chan->csi->tpg_frmfmt_table;

	for (i = 0; i < chan->csi->tpg_frmfmt_table_size; i++) {
		if (tegra_csi_tpg_frmfmt[i].frmsize.width == width &&
		    tegra_csi_tpg_frmfmt[i].frmsize.height == height &&
		    tegra_csi_tpg_frmfmt[i].pixel_format == pixel_format)
			break;
	}

	if (i == chan->csi->tpg_frmfmt_table_size)
		return -EINVAL;

	return i;
}

static int tegra_csi_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int index;
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	const struct tegra_video_format *format;
	const struct tpg_frmfmt *tegra_csi_tpg_frmfmt =
						chan->csi->tpg_frmfmt_table;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

	/* One resolution just one framerate */
	if (fie->index > 0)
		return -EINVAL;
	format = tegra_core_get_format_by_fourcc(fie->code);
	if (!format)
		return -EINVAL;
	index = tegra_csi_get_fmtindex(chan, fie->width, fie->height,
					format->fourcc);
	if (index < 0)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = tegra_csi_tpg_frmfmt[index].framerate;

	return 0;
}

static int tegra_csi_try_mbus_fmt(struct v4l2_subdev *sd,
				  struct v4l2_mbus_framefmt *mf)
{
	int i, j;
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	static struct v4l2_frmsize_discrete *sizes;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

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
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	struct v4l2_mbus_framefmt *format = &chan->ports[0].format;

	if (!chan->pg_mode) {
		dev_err(chan->csi->dev, "CSI is not in TPG mode\n");
		return -EINVAL;
	}

	mutex_lock(&chan->format_lock);
	memcpy(fmt, format, sizeof(struct v4l2_mbus_framefmt));
	mutex_unlock(&chan->format_lock);

	return 0;
}

static int csi_is_power_on(struct tegra_csi_device *csi)
{
	return atomic_read(&csi->power_ref);
}
static int tegra_csi_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tegra_csi_device *csi = to_csi(sd);

	/* Set status to 0 if power is on
	 * Set status to 1 if power is off
	 */
	*status = !csi_is_power_on(csi);

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static int tegra_csi_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct v4l2_mbus_framefmt *mbus_fmt = &fmt->format;
	int ret;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;
	ret = tegra_csi_g_mbus_fmt(subdev, mbus_fmt);
	if (ret)
		return ret;

	return 0;
}

static int tegra_csi_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	int ret;
	struct tegra_csi_channel *chan = to_csi_chan(subdev);
	struct v4l2_mbus_framefmt *format = &fmt->format;
	const struct tegra_video_format *vf;
	struct tegra_channel *vi_chan = v4l2_get_subdev_hostdata(subdev);
	int index, i;

	if (!chan->pg_mode)
		return -ENOIOCTLCMD;

	ret = tegra_csi_try_mbus_fmt(subdev, format);
	if (ret)
		return ret;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	vf = tegra_core_get_format_by_code(format->code);
	if (!vf) {
		dev_err(chan->csi->dev, "Fail to find tegra video fmt");
		mutex_unlock(&chan->format_lock);
		return -EINVAL;
	}
	index = tegra_csi_get_fmtindex(chan, format->width,
				format->height, vf->fourcc);
	if (index < 0) {
		dev_err(chan->csi->dev, "Fail to find matching fmt");
		return -EINVAL;
	}

	mutex_lock(&chan->format_lock);
	for (i = 0; i < vi_chan->valid_ports; i++) {
		memcpy(&chan->ports[i].format,
		       &fmt->format, sizeof(struct v4l2_mbus_framefmt));
		chan->ports[i].core_format = vf;
		update_blank_intervals(chan, i, index);
	}
	mutex_unlock(&chan->format_lock);

	return 0;
}

static int tegra_csi_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *vfi)
{
	struct tegra_csi_channel *chan = to_csi_chan(sd);
	struct tegra_csi_port *port = &chan->ports[0];

	if (!port->framerate)
		return -EINVAL;

	vfi->interval.numerator = 1;
	vfi->interval.denominator = port->framerate;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static struct v4l2_subdev_video_ops tegra_csi_video_ops = {
	.s_stream	= tegra_csi_s_stream,
	.g_input_status = tegra_csi_g_input_status,
	.g_frame_interval = tegra_csi_g_frame_interval,
};

static struct v4l2_subdev_pad_ops tegra_csi_pad_ops = {
	.get_fmt	= tegra_csi_get_format,
	.set_fmt	= tegra_csi_set_format,
	.enum_frame_size = tegra_csi_enum_framesizes,
	.enum_frame_interval = tegra_csi_enum_frameintervals,
};

static struct v4l2_subdev_core_ops tegra_csi_core_ops = {
	.s_power	= tegra_csi_s_power,
};

static struct v4l2_subdev_ops tegra_csi_ops = {
	.core	= &tegra_csi_core_ops,
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

	memset(&chan->port[0], INVALID_CSI_PORT, TEGRA_CSI_BLOCKS);
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

	for (i = 0; csi_port_is_valid(chan->port[i]); i++)
		chan->numports++;

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

static int tegra_csi_channel_init_one(struct tegra_csi_channel *chan)
{
	struct v4l2_subdev *sd;
	int numlanes = 0;
	struct tegra_csi_device *csi = chan->csi;
	int i, ret;

	sd = &chan->subdev;
	/* Initialize V4L2 subdevice and media entity */
	v4l2_subdev_init(sd, &tegra_csi_ops);
	sd->dev = chan->csi->dev;
	v4l2_set_subdevdata(sd, csi);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &tegra_csi_media_ops;
	chan->ports = devm_kzalloc(csi->dev,
			chan->numports * sizeof(struct tegra_csi_port),
			GFP_KERNEL);
	if (!chan->ports)
		return -ENOMEM;

	mutex_init(&chan->format_lock);

	/* Initialize the default format */
	for (i = 0; i < chan->numports; i++) {
		mutex_lock(&chan->format_lock);
		chan->ports[i].format.code = TEGRA_VF_DEF;
		chan->ports[i].format.field = V4L2_FIELD_NONE;
		chan->ports[i].format.colorspace = V4L2_COLORSPACE_SRGB;
		chan->ports[i].format.width = TEGRA_DEF_WIDTH;
		chan->ports[i].format.height = TEGRA_DEF_HEIGHT;
		mutex_unlock(&chan->format_lock);
	}
	if (chan->pg_mode) {
		/* If CSI has 2 existing channels, chan->id will start
		 * from 2 for the first TPG channel, which uses PORT_A(0).
		 * To get the correct PORT number, subtract existing number of
		 * channels from chan->id.
		 */
		chan->port[0] = chan->id - csi->num_channels;
		WARN_ON(chan->port[0] > TPG_CHANNELS);
		chan->ports[0].num = chan->id - csi->num_channels;
		chan->ports->lanes = 2;
		chan->pads = devm_kzalloc(csi->dev, sizeof(*chan->pads),
				GFP_KERNEL);
		if (!chan->pads)
			return -ENOMEM;
		chan->pads[0].flags = MEDIA_PAD_FL_SOURCE;
	} else {
		chan->pads = devm_kzalloc(csi->dev, 2 * sizeof(*chan->pads),
			GFP_KERNEL);
		if (!chan->pads)
			return -ENOMEM;
		chan->pads[0].flags = MEDIA_PAD_FL_SINK;
		chan->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	}
	snprintf(sd->name, sizeof(sd->name), "%s-%d",
			 chan->pg_mode?"tpg":dev_name(csi->dev), chan->port[0]);
	/* Initialize media entity */
	ret = media_entity_init(&sd->entity,
			chan->pg_mode ? 1 : 2,
			chan->pads, 0);
	if (ret < 0) {
		return ret;
	}

	for (i = 0; i < chan->numports; i++) {
		numlanes = chan->numlanes - (i * MAX_CSI_BLOCK_LANES);
		WARN_ON(numlanes < 0);
		numlanes = numlanes > MAX_CSI_BLOCK_LANES ?
			MAX_CSI_BLOCK_LANES : numlanes;
		chan->ports[i].lanes = numlanes;
		chan->ports[i].num = chan->port[i];
	}

	if (!chan->pg_mode) {
		ret = v4l2_async_register_subdev(sd);
		if (ret < 0) {
			dev_err(csi->dev, "failed to register subdev\n");
			media_entity_cleanup(&sd->entity);
		}
	}
	return ret;
}

static int tegra_csi_channels_init(struct tegra_csi_device *csi)
{
	int ret;
	struct tegra_csi_channel *it;

	list_for_each_entry(it, &csi->csi_chans, list) {
		ret = tegra_csi_channel_init_one(it);
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
	struct tegra_csi_channel *item;

	err = of_property_read_u32(node, "num-channels", &num_channels);
	if (err) {
		dev_err(csi->dev, " Faile to find num of channels, set to 0\n");
		num_channels = 0;
	}

	csi->num_channels = num_channels;
	for (i = 0; i < num_channels; i++) {
		item = devm_kzalloc(csi->dev, sizeof(*item), GFP_KERNEL);
		if (!item)
			return -ENOMEM;
		list_add_tail(&item->list, &csi->csi_chans);
		item->csi = csi;
		item->id = i;
		err = tegra_csi_get_port_info(item, node, item->id);
		if (err)
			return err;
	}

	return 0;
}

int tpg_csi_media_controller_init(struct tegra_csi_device *csi, int pg_mode)
{
	int i, err;
	struct tegra_csi_channel *item;

	if (!csi)
		return -EINVAL;
	for (i = 0; i < TPG_CHANNELS; i++) {
		item = devm_kzalloc(csi->dev, sizeof(*item), GFP_KERNEL);
		if (!item) {
			err = -ENOMEM;
			goto channel_init_error;
		}
		if (i == 0)
			csi->tpg_start = item;
		list_add_tail(&item->list, &csi->csi_chans);
		item->numlanes = 2;
		item->numports = 1;
		item->csi = csi;
		item->pg_mode = pg_mode;
		item->id = csi->num_channels + i;
		err = tegra_csi_channel_init_one(item);
		if (err)
			goto channel_init_error;
	}
	csi->num_channels += TPG_CHANNELS;

	return err;

channel_init_error:
	if (csi->tpg_start)
		tpg_csi_media_controller_cleanup(csi);
	dev_err(csi->dev, "%s: Error\n", __func__);
	return err;
}
EXPORT_SYMBOL(tpg_csi_media_controller_init);

void tpg_csi_media_controller_cleanup(struct tegra_csi_device *csi)
{
	struct tegra_csi_channel *item;
	struct tegra_csi_channel *itemn;
	struct v4l2_subdev *sd;

	list_for_each_entry_safe(item, itemn, &csi->csi_chans, list) {
		if (!item->pg_mode)
			continue;
		sd = &item->subdev;
		v4l2_device_unregister_subdev(sd);
		list_del(&item->list);
		devm_kfree(csi->dev, item);
	}
	csi->num_channels -= TPG_CHANNELS;
	csi->tpg_start = NULL;
}
EXPORT_SYMBOL(tpg_csi_media_controller_cleanup);
int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				    struct platform_device *pdev)
{
	int ret;

	csi->dev = &pdev->dev;
	csi->pdev = pdev;
	atomic_set(&csi->power_ref, 0);
	INIT_LIST_HEAD(&csi->csi_chans);
	ret = csi_parse_dt(csi, pdev);
	if (ret < 0)
		return ret;
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

	list_for_each_entry(chan, &csi->csi_chans, list) {
		sd = &chan->subdev;
		v4l2_async_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
	return 0;
}
EXPORT_SYMBOL(tegra_csi_media_controller_remove);
