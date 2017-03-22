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
#include "csi/csi.h"

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
		for (i = 0; csi_port_is_valid(port_num[i]); i++) {
			port = port_num[i];
			camera_common_dpd_disable(&csi->s_data[port]);
		}
	} else {
		for (i = 0; csi_port_is_valid(port_num[i]); i++) {
			port = port_num[i];
			camera_common_dpd_enable(&csi->s_data[port]);
		}
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
			camera_common_dpd_disable(&csi->s_data[port]);
		}
	} else {
		for (i = 0; csi_port_is_valid(port_num[i]); i++) {
			port = port_num[i];
			cil_num = port >> 1;
			camera_common_dpd_enable(&csi->s_data[port]);
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
	csi->fops->soc_tpg_start_streaming(csi, port_num);
}

void tegra_csi_start_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	csi->fops->soc_start_streaming(csi, port_num);
}
EXPORT_SYMBOL(tegra_csi_start_streaming);

int tegra_csi_error(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num)
{
	return csi->fops->soc_error(csi, port_num);
}

void tegra_csi_status(struct tegra_csi_device *csi,
			enum tegra_csi_port_num port_num)
{
	csi->fops->soc_status(csi, port_num);
}
EXPORT_SYMBOL(tegra_csi_status);

void tegra_csi_error_recover(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	csi->fops->soc_error_recover(csi, port_num);
}

void tegra_csi_stop_streaming(struct tegra_csi_device *csi,
				enum tegra_csi_port_num port_num)
{
	csi->fops->soc_stop_streaming(csi, port_num);
}
EXPORT_SYMBOL(tegra_csi_stop_streaming);

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_csi_device *csi = to_csi(subdev);
	struct tegra_channel *chan;
	int index;

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
			   struct v4l2_subdev_format *fmt)
{
	int i, ret;
	struct tegra_csi_device *csi = to_csi(subdev);

	ret = tegra_csi_try_mbus_fmt(subdev, &fmt->format);
	if (ret)
		return ret;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	for (i = 0; i < csi->num_ports; i++) {
		struct v4l2_mbus_framefmt *format = &csi->ports[i].format;
		memcpy(format, &fmt->format, sizeof(struct v4l2_mbus_framefmt));
	}

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
	.get_fmt	= tegra_csi_get_format,
	.set_fmt	= tegra_csi_set_format,
	.enum_frame_size = tegra_csi_enum_framesizes,
	.enum_frame_interval = tegra_csi_enum_frameintervals,
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

static int tegra_tpg_csi_parse_data(struct tegra_csi_device *csi,
				    struct platform_device *pdev)
{
	int i;

	csi->ports = devm_kzalloc(&pdev->dev,
		(csi->num_ports * sizeof(struct tegra_csi_port)), GFP_KERNEL);
	if (!csi->ports)
		return -ENOMEM;

	csi->pads = devm_kzalloc(&pdev->dev,
		(csi->num_ports * sizeof(struct media_pad)), GFP_KERNEL);
	if (!csi->pads)
		return -ENOMEM;

	for (i = 0; i < csi->num_ports; i++) {
		csi->ports[i].num = i;
		csi->ports[i].lanes = 2;
	}

	return 0;
}


int tegra_csi_init(struct tegra_csi_device *csi,
		struct platform_device *pdev)
{
	if (csi->fops != NULL && csi->fops->soc_init != NULL)
		return csi->fops->soc_init(csi, pdev);
	return -EINVAL;
}

int tegra_csi_media_controller_init(struct tegra_csi_device *csi,
				    struct platform_device *pdev)
{
	struct v4l2_subdev *subdev;
	int ret, i;

	csi->dev = &pdev->dev;

	if (csi->pg_mode)
		ret = tegra_tpg_csi_parse_data(csi, pdev);
	else
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

		if (csi->pg_mode)
			csi->pads[i].flags = MEDIA_PAD_FL_SOURCE;
		else {
			csi->pads[i * 2].flags = MEDIA_PAD_FL_SINK;
			csi->pads[i * 2 + 1].flags = MEDIA_PAD_FL_SOURCE;
		}
	}

	/* Initialize media entity */
	ret = media_entity_init(&subdev->entity,
				csi->pg_mode ? csi->num_ports :
				csi->num_ports * 2,
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
