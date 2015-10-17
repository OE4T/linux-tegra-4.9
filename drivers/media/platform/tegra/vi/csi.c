/*
 * NVIDIA Tegra CSI Device
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include "vi.h"

/* Each CSI has 2 ports, CSI_A and CSI_B */
#define TEGRA_CSI_PORTS_NUM	2

/* Each port has one source pad and one sink pad */
#define TEGRA_CSI_PADS_NUM	4

enum tegra_csi_port_num {
	PORT_A = 0,
	PORT_B = 1,
};

struct tegra_csi_port {
	void __iomem *pixel_parser;
	void __iomem *cil;
	void __iomem *tpg;

	/* One pair of sink/source pad has one format */
	struct v4l2_mbus_framefmt format;
	const struct tegra_video_format *core_format;
	unsigned int lanes;

	enum tegra_csi_port_num num;
};

struct tegra_csi_device {
	struct v4l2_subdev subdev;
	struct device *dev;
	void __iomem *iomem;
	struct clk *clk;

	struct tegra_csi_port ports[TEGRA_CSI_PORTS_NUM];
	struct media_pad pads[TEGRA_CSI_PADS_NUM];
};

static inline struct tegra_csi_device *to_csi(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct tegra_csi_device, subdev);
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Video Operations
 */

static int tegra_csi_s_stream(struct v4l2_subdev *subdev, int enable)
{
	/* Need programming CSI only for VI mode */
	return 0;
}


/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

static struct v4l2_mbus_framefmt *
__tegra_csi_get_pad_format(struct tegra_csi_device *csi,
		      struct v4l2_subdev_fh *cfg,
		      unsigned int pad, u32 which)
{
	enum tegra_csi_port_num port_num = pad < 2 ? PORT_A : PORT_B;

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
		return v4l2_subdev_get_try_format(cfg, pad);
#endif
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &csi->ports[port_num].format;
	default:
		return NULL;
	}
}

static int tegra_csi_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_fh *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct tegra_csi_device *csi = to_csi(subdev);

	fmt->format = *__tegra_csi_get_pad_format(csi, cfg,
					fmt->pad, fmt->which);
	return 0;
}

static int tegra_csi_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_fh *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct tegra_csi_device *csi = to_csi(subdev);
	struct v4l2_mbus_framefmt *__format;
	enum tegra_csi_port_num port_num = fmt->pad < 2 ? PORT_A : PORT_B;

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
	.pad    = &tegra_csi_pad_ops,
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

static int tegra_csi_parse_of(struct tegra_csi_device *csi)
{
	struct device_node *node = csi->dev->of_node;
	struct device_node *ports;
	struct device_node *port;
	struct device_node *ep;
	unsigned int lanes, pad_num, port_num;
	int ret;

	ports = of_get_child_by_name(node, "ports");
	if (ports == NULL)
		ports = node;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;

		ret = of_property_read_u32(port, "reg", &pad_num);
		if (ret < 0)
			continue;
		port_num = pad_num < 2 ? PORT_A : PORT_B;
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

static int tegra_csi_probe(struct platform_device *pdev)
{
	struct v4l2_subdev *subdev;
	struct tegra_csi_device *csi;
	int ret, i;

	csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = &pdev->dev;

	ret = tegra_csi_parse_of(csi);
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

	for (i = 0; i < TEGRA_CSI_PORTS_NUM; i++) {
		/* Initialize the default format */
		csi->ports[i].format.code = TEGRA_VF_DEF;
		csi->ports[i].format.field = V4L2_FIELD_NONE;
		csi->ports[i].format.colorspace = V4L2_COLORSPACE_SRGB;
		csi->ports[i].format.width = TEGRA_DEF_WIDTH;
		csi->ports[i].format.height = TEGRA_DEF_HEIGHT;

		csi->ports[i].num = i;

		csi->pads[i * 2].flags = MEDIA_PAD_FL_SINK;
		csi->pads[i * 2 + 1].flags = MEDIA_PAD_FL_SOURCE;
	}

	/* Initialize media entity */
	ret = media_entity_init(&subdev->entity, ARRAY_SIZE(csi->pads),
			csi->pads, 0);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, csi);

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

static int tegra_csi_remove(struct platform_device *pdev)
{
	struct tegra_csi_device *csi = platform_get_drvdata(pdev);
	struct v4l2_subdev *subdev = &csi->subdev;

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	return 0;
}

static const struct of_device_id tegra_csi_of_id_table[] = {
	{ .compatible = "nvidia,tegra210-csi" },
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_csi_of_id_table);

static struct platform_driver tegra_csi_driver = {
	.driver = {
		.name		= "tegra-csi",
		.of_match_table	= tegra_csi_of_id_table,
	},
	.probe			= tegra_csi_probe,
	.remove			= tegra_csi_remove,
};
module_platform_driver(tegra_csi_driver);

MODULE_AUTHOR("Bryan Wu <pengw@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra CSI Device Driver");
MODULE_LICENSE("GPL v2");
