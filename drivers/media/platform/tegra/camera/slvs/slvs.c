/*
 * NVIDIA Tegra SLVS(-EC) Subdevice for V4L2
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Pekka Pessi <ppessi@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/camera_common.h>
#include <media/mc_common.h>
#include <media/slvs.h>

#include "dev.h"
#include "linux/nvhost.h"
#include <linux/version.h>

#define SLVSEC_NUM_LANES 8

struct slvsec_stream_params {
	u8 rate;
	u8 lanes[8];
	u8 num_lanes;
	bool enable_header_crc;
	bool enable_payload_crc;
	u32 watchdog_period;
	u32 symbols;

	struct slvsec_cil_stream_uphy {
		bool rate_gen_2;
		bool term_other;
		bool term_data;
		bool aux_term_other;
		bool aux_term_data;
		bool aux_term_10kohm;
		bool skip_sleep;
		bool dedicated_calibration;
		u8 aux_idle_mode;
		bool aux_idle_detect;
	} uphy;
};

struct slvsec_lane_params {
	bool stream1;
	bool half_pad_code;
	bool half_start_code;
	bool switch_polarity;
};

struct slvec_uphy_params {
	u32 blank_timeout;
	u8 sleep_to_iddq;
	u8 iddq_to_sleep;
	u8 dataen_to_sleep;
	u8 sleep_to_dataen;
	u8 dataen_step;
	u8 sleep_step;
	u8 sleep_cali;
	u8 cali_dataen;
	u8 cali_sleep;
	bool always_on_iopll;
	bool standby_off_iopll;
};

struct slvsec_params {
	struct slvsec_lane_params lanes[SLVSEC_NUM_LANES];
	struct slvec_uphy_params uphy;
};

struct tegra_mc_slvs {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *slvs_base;
	void __iomem *syncgen_base;

	atomic_t power_ref;
	atomic_t sensor_active;

	struct slvsec_params params;

	struct tegra_slvs_stream *streams;
	int num_streams;

	struct dentry *debugdir;
};

struct tegra_slvs_stream {
	struct tegra_mc_slvs *slvs;
	struct v4l2_subdev subdev;
	struct media_pad pads[2];

	u32 id;
	struct slvsec_stream_params params;
	atomic_t is_streaming;
	wait_queue_head_t cil_waitq;
};

#define to_slvs(_sd) \
	container_of(_sd, struct tegra_slvs_stream, subdev)->slvs

static int tegra_slvs_s_power(struct v4l2_subdev *sd, int enable)
{
	struct tegra_mc_slvs *slvs = v4l2_get_subdevdata(sd);
	int err = 0;

	if (enable) {
		err = nvhost_module_busy(slvs->pdev);
		if (err == 0)
			atomic_inc(&slvs->power_ref);
		else
			dev_err(&slvs->pdev->dev,
				"s_power: failed (%d)\n", err);
	} else {
		nvhost_module_idle(slvs->pdev);
		atomic_dec(&slvs->power_ref);
	}

	return err;
}

#define SLVS_STREAM0_OFFSET	0x10000
#define SLVS_STREAM_PAGE_SIZE	0x10000
#define SLVS_CIL_OFFSET		0x30000
#define SLVS_CIL_PAGE_SIZE	0x01000
#define SLVS_CIL_LANE_OFFSET	0x31800
#define SLVS_CIL_LANE_SIZE	0x00400

#define SLVS_STRM_CTRL			0x0000
#define SLVS_STRM_RST_CTRL		0x0004
#define SLVS_STRM_CH0_CFG		0x000C
#define SLVS_STRM_CH1_CFG		0x0010
#define SLVS_STRM_CH0_EMBD_CFG		0x0014
#define SLVS_STRM_CH1_EMBD_CFG		0x0018
#define SLVS_STRM_TIMEOUT_CTRL		0x001C
#define SLVS_STRM_INTR_MASK		0x0044
#define SLVS_STRM_INTR_MASK_CH0		0x003C
#define SLVS_STRM_INTR_MASK_CH1		0x0040
#define SLVS_STRM_VI_ERR_MASK_CH0	0x0034
#define SLVS_STRM_VI_ERR_MASK_CH1	0x0038

#define SLVS_CIL_STRM_RST_CTRL		0x04
#define SLVS_CIL_STRM_CTRL		0x08
#define SLVS_CIL_STRM_SYM_DEFINE	0x0c
#define SLVS_CIL_STRM_UPHY_MODE		0x10

#define SLVS_CIL_STRM_UPHY_CAL_EN	BIT(3)

#define SLVS_CIL_STRM_INTR_MASK		0x1c
#define SLVS_CIL_STRM_INTR_STATUS	0x20
#define SLVS_CIL_STRM_INTR_CAL_DONE	BIT(3)
#define SLVS_CIL_STRM_CAL_STATUS	0x24
#define SLVS_CIL_STRM_CAL_STATUS_DONE	BIT(0)

#define SLVS_LANE_RST_CTRL		0x04
#define SLVS_LANE_CTRL			0x08
#define SLVS_LANE_STATUS		0x0C

#define TEGRA_IMAGE_DT_RAW16	46

/*
 * SLVS-EC register accessors
 */
static inline void slvs_core_write(struct tegra_mc_slvs *slvs,
		u32 offset, u32 val)
{
	writel(val, slvs->slvs_base + offset);
}

static inline u32 slvs_core_read(struct tegra_mc_slvs *slvs,
		u32 offset)
{
	return readl(slvs->slvs_base + offset);
}

static inline void slvs_stream_write(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_STREAM0_OFFSET +
		id * SLVS_STREAM_PAGE_SIZE +
		offset);
}

static inline u32 slvs_stream_read(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_STREAM0_OFFSET +
		id * SLVS_STREAM_PAGE_SIZE +
		offset);
}

static inline void slvs_cil_stream_write(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_CIL_OFFSET +
		id * SLVS_CIL_PAGE_SIZE +
		offset);
}

static inline u32 slvs_cil_stream_read(struct tegra_mc_slvs *slvs,
				u32 id, u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_CIL_OFFSET +
		id * SLVS_CIL_PAGE_SIZE +
		offset);
}

static inline void slvs_cil_lane_write(struct tegra_mc_slvs *slvs,
				u32 lane, u32 offset, u32 val)
{
	writel(val, slvs->slvs_base +
		SLVS_CIL_LANE_OFFSET +
		lane * SLVS_CIL_LANE_SIZE +
		offset);
}

static inline u32 slvs_cil_lane_read(struct tegra_mc_slvs *slvs,
				u32 lane, u32 offset)
{
	return readl(slvs->slvs_base +
		SLVS_CIL_LANE_OFFSET +
		lane * SLVS_CIL_LANE_SIZE +
		offset);
}

static int slvs_stream_config(const struct tegra_channel *chan)
{
	u32 data_type = chan->fmtinfo->img_dt;
	u32 width = chan->format.width;
	u32 pixel_width = chan->fmtinfo->width;
	u32 stuffing = 16 - ((width * pixel_width + 7U) / 8U) % 16;
	u32 pd_length = ((width * pixel_width + 7U) / 8U);
	u32 line_length = pd_length + stuffing;

	switch (data_type) {
	case TEGRA_IMAGE_DT_RAW8:
	case TEGRA_IMAGE_DT_RAW10:
	case TEGRA_IMAGE_DT_RAW12:
	case TEGRA_IMAGE_DT_RAW14:
	case TEGRA_IMAGE_DT_RAW16:
		break;
	default:
		WARN(1, "Unsupported data type %u", data_type);
		return -EINVAL;
	}

	if (WARN_ON(pd_length >= 0x20000))
		return -EINVAL;
	if (WARN(line_length % 16 != 0,
			"bad slvs-ec stuffing: width=%u pixel=%ub",
			width, pixel_width))
		return -EINVAL;

	return (data_type << 22) | (stuffing << 18) | (pd_length);
}

static int slvs_cil_stream_uphy_mode(const struct tegra_slvs_stream *stream)
{
	const struct slvsec_cil_stream_uphy *uphy = &stream->params.uphy;

	return (uphy->rate_gen_2 << 11) |
		(uphy->term_other << 10) |
		(uphy->term_data << 9) |
		(uphy->aux_term_other << 8) |
		(uphy->aux_term_data << 7) |
		(uphy->aux_term_10kohm << 6) |
		(uphy->skip_sleep << 5) |
		(uphy->dedicated_calibration << 4) |
		(uphy->aux_idle_mode << 1) |
		(uphy->aux_idle_detect << 0);
}

/*
 * -----------------------------------------------------------------------------
 * SLVS Subdevice Video Operations
 * -----------------------------------------------------------------------------
 */
static inline bool tegra_slvs_is_cal_done(struct tegra_slvs_stream *stream)
{
	u32 val = slvs_cil_stream_read(stream->slvs, stream->id,
				SLVS_CIL_STRM_CAL_STATUS);

	return (val & SLVS_CIL_STRM_CAL_STATUS_DONE) == 0;
}

static int tegra_slvs_start_streaming(struct tegra_slvs_stream *stream)
{
	struct tegra_mc_slvs *slvs = stream->slvs;
	struct tegra_channel *tegra_chan;
	long timeout = msecs_to_jiffies(5000);
	u32 id = stream->id;
	int i, cfg;
	u32 val;

	if (WARN_ON(id != 0))
		return -ENXIO;

	tegra_chan = v4l2_get_subdev_hostdata(&stream->subdev);
	cfg = slvs_stream_config(tegra_chan);
	if (cfg < 0)
		return cfg;

	/* XXX - where we should dig embedded data type ? */

	/* Reset stream */
	slvs_stream_write(slvs, id, SLVS_STRM_RST_CTRL, 1);
	/* Reset all lanes */
	for (i = 0; i < SLVSEC_NUM_LANES; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_RST_CTRL, 1);

	/* Reset CIL */
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_RST_CTRL, 1);

	/* Program core */
	slvs_stream_write(slvs, id, SLVS_STRM_RST_CTRL, 0);
	val = (stream->params.enable_header_crc << 2)
		| ((stream->params.watchdog_period != 0 << 1))
		| 1;
	slvs_stream_write(slvs, id, SLVS_STRM_CTRL, val);
	val = cfg | (stream->params.enable_payload_crc << 28);
	slvs_stream_write(slvs, id, SLVS_STRM_CH0_CFG, val);
	/* XXX - how this should be configured ? */
	slvs_stream_write(slvs, id, SLVS_STRM_CH1_CFG, 0x1a800000);

	/* Mask errors and interrupts */
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_MASK, 0xf);
	slvs_stream_write(slvs, id, SLVS_STRM_VI_ERR_MASK_CH0, 0xf);
	slvs_stream_write(slvs, id, SLVS_STRM_VI_ERR_MASK_CH1, 0xf);
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_MASK_CH0, 0xf);
	slvs_stream_write(slvs, id, SLVS_STRM_INTR_MASK_CH1, 0xf);

	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_RST_CTRL, 0);
	for (i = 0; i < SLVSEC_NUM_LANES; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_RST_CTRL, 0);

	/* Enable cal done interrupt */
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CAL_STATUS,
			SLVS_CIL_STRM_INTR_CAL_DONE);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_INTR_MASK,
			SLVS_CIL_STRM_INTR_CAL_DONE);

	/* Program CIL */
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_SYM_DEFINE,
			stream->params.symbols);
	val = (stream->params.num_lanes << 1) | 1;
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, val);
	val = slvs_cil_stream_uphy_mode(stream);
	val |= SLVS_CIL_STRM_UPHY_CAL_EN;
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_UPHY_MODE, val);

	/* Enable lanes */
	for (i = 0; i < stream->params.num_lanes; i++) {
		/* XXX id=1 => STRM1_LANE_BEG_NUM */
		slvs_cil_lane_write(slvs, i, SLVS_LANE_CTRL, 1);
	}

	/* Wait for CAL_DONE */
	timeout = wait_event_interruptible_timeout(
		stream->cil_waitq,
		tegra_slvs_is_cal_done(stream),
		timeout);

	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_INTR_MASK, 0);

	if (timeout == 0)
		return -ETIMEDOUT;

	/* Enable syncgen */

	return 0;
}

static void tegra_slvs_cil_notify(struct tegra_slvs_stream *stream,
	u32 events)
{
	(void)events;

	if (tegra_slvs_is_cal_done(stream)) {
		slvs_cil_stream_write(stream->slvs, stream->id,
				SLVS_CIL_STRM_INTR_MASK, 0);
		wake_up(&stream->cil_waitq);
	}
}

static int tegra_slvs_stop_streaming(struct tegra_slvs_stream *stream)
{
	struct tegra_mc_slvs *slvs = stream->slvs;
	u32 id = stream->id;
	int i;

	slvs_stream_write(slvs, id, SLVS_STRM_CTRL, 0);
	slvs_cil_stream_write(slvs, id, SLVS_CIL_STRM_CTRL, 0);
	for (i = 0; i < stream->params.num_lanes; i++)
		slvs_cil_lane_write(slvs, i, SLVS_LANE_CTRL, 0);

	return 0;
}

static int tegra_slvs_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct tegra_channel *tegra_chan = v4l2_get_subdev_hostdata(subdev);
	struct tegra_slvs_stream *stream;
	int ret;

	stream = container_of(subdev, struct tegra_slvs_stream, subdev);

	if (atomic_read(&stream->is_streaming) == enable)
		return 0;

	if (tegra_chan->bypass)
		ret = 0;
	else if (enable)
		ret = tegra_slvs_start_streaming(stream);
	else
		ret = tegra_slvs_stop_streaming(stream);

	atomic_set(&stream->is_streaming, enable);

	return ret;
}

static int tegra_slvs_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tegra_mc_slvs *slvs = v4l2_get_subdevdata(sd);

	/*
	 * Set status to 0 if power is on
	 * Set status to 1 if power is off
	 */
	*status = atomic_read(&slvs->power_ref) == 0;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Pad Operations
 */

/* -----------------------------------------------------------------------------
 * V4L2 Subdevice Operations
 */
static const struct v4l2_subdev_video_ops tegra_slvs_video_ops = {
	.s_stream	= tegra_slvs_s_stream,
	.g_input_status = tegra_slvs_g_input_status,
};

static const struct v4l2_subdev_core_ops tegra_slvs_core_ops = {
	.s_power	= tegra_slvs_s_power,
};

static const struct v4l2_subdev_ops tegra_slvs_ops = {
	.core	= &tegra_slvs_core_ops,
	.video  = &tegra_slvs_video_ops,
};

static const struct media_entity_operations tegra_slvs_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * Media controller
 */
static int tegra_slvs_init_stream(struct tegra_mc_slvs *slvs,
				struct tegra_slvs_stream *stream)
{
	struct v4l2_subdev *sd;
	int err;

	atomic_set(&stream->is_streaming, 0);

	/* Initialize V4L2 subdevice and media entity */
	sd = &stream->subdev;
	v4l2_subdev_init(sd, &tegra_slvs_ops);
	sd->dev = slvs->dev;
	v4l2_set_subdevdata(sd, slvs);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &tegra_slvs_media_ops;
	snprintf(sd->name, sizeof(sd->name), "%s-stream-%u",
		dev_name(slvs->dev), stream->id);

	/* Initialize media entity */
	/* XXX - use media_entity_pads_init() directly? */
	err = tegra_media_entity_init(&sd->entity,
				ARRAY_SIZE(stream->pads),
				stream->pads, true, false);
	if (err < 0) {
		dev_err(slvs->dev, "stream@%u: failed to register pads\n",
			stream->id);
		return err;
	}

	err = v4l2_async_register_subdev(sd);
	if (err < 0) {
		dev_err(slvs->dev, "stream@%u: failed to register subdev\n",
			stream->id);
		return err;
	}

	return 0;
}

static int tegra_slvs_init_streams(struct tegra_mc_slvs *slvs)
{
	int i;
	int err;

	for (i = 0; i < slvs->num_streams; i++) {
		err = tegra_slvs_init_stream(slvs, &slvs->streams[i]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int tegra_slvs_count_streams(struct device_node *np)
{
	struct device_node *streams_node, *stream_node;
	int count = 0;

	streams_node = of_get_child_by_name(np, "streams");

	for_each_available_child_of_node(streams_node, stream_node) {
		if (stream_node->name &&
			of_node_cmp(stream_node->name, "stream") == 0)
			count++;
	}

	of_node_put(streams_node);

	return count;
}

static struct device_node *tegra_slvs_get_stream_node(struct device_node *np,
						int num)
{
	struct device_node *streams_node, *stream_node;
	int count = 0;

	streams_node = of_get_child_by_name(np, "streams");

	for_each_available_child_of_node(streams_node, stream_node) {
		if (stream_node->name &&
			of_node_cmp(stream_node->name, "stream") == 0) {
			if (num == count)
				break;
			count++;
		}
	}

	of_node_put(streams_node);

	return stream_node;
}

static int tegra_slvs_parse_stream_dt(struct tegra_slvs_stream *stream,
				struct device_node *np)
{
	struct slvsec_stream_params *params = &stream->params;
	int i, err;
	bool boolparam;
	u32 numparam;

	err = of_property_read_u32(np, "reg", &numparam);
	if (err < 0)
		stream->id = numparam;

	err = of_property_count_elems_of_size(np, "lanes", sizeof(u32));
	if (err < 0)
		return err;
	/* We can have 1, 2, 4, 6 or 8 lanes */
	if (err <= 0 || err > SLVSEC_NUM_LANES || ((err % 2) == 1 && err > 1))
		return -EINVAL;
	params->num_lanes = err;

	for (i = 0; i < params->num_lanes; i++) {
		u32 lane;
		err = of_property_read_u32_index(np, "lanes", i, &lane);
		if (err)
			return err;
		if (lane > SLVSEC_NUM_LANES)
			return -EINVAL;
		params->lanes[i] = lane;
	}

	boolparam = of_property_read_bool(np, "nvidia,disable-header-crc");
	params->enable_header_crc = !boolparam;
	boolparam = of_property_read_bool(np, "nvidia,disable-payload-crc");
	params->enable_payload_crc = !boolparam;

	of_property_read_u32(np, "watchdog-period", &params->watchdog_period);

	err = of_property_read_u32(np, "nvidia,symbols", &params->symbols);
	if (err != 0)
		params->symbols = 0x000360aa;

	boolparam = of_property_read_bool(np, "nvidia,uphy-rate-gen2");
	params->uphy.rate_gen_2 = boolparam;

	boolparam =  of_property_read_bool(np, "nvidia,uphy-term-other");
	params->uphy.term_other = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-term-data");
	params->uphy.term_data = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-term-other");
	params->uphy.aux_term_other = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-term-data");
	params->uphy.aux_term_data = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-term-10kohm");
	params->uphy.aux_term_10kohm = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-skip-sleep");
	params->uphy.skip_sleep = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,"
					"uphy-dedicated-calibration");
	params->uphy.dedicated_calibration = boolparam;

	boolparam = of_property_read_bool(np, "nvidia,uphy-aux-idle-detect");
	params->uphy.aux_idle_detect = boolparam;

	err = of_property_read_u32(np, "nvidia,uphy-aux-idle-mode", &numparam);
	if (err == 0)
		params->uphy.aux_idle_mode = numparam & 0x03;

	return 0;
}

static int tegra_slvs_parse_dt(struct tegra_mc_slvs *slvs)
{
	struct device_node *node = slvs->pdev->dev.of_node;
	int num_streams = tegra_slvs_count_streams(node);
	struct device_node *stream_node;
	struct tegra_slvs_stream *stream;
	int i;

	if (num_streams <= 0) {
		dev_info(slvs->dev, "no streams defined");
		return -ENODEV;
	}

	dev_info(slvs->dev, "slvs has %d available streams", num_streams);

	slvs->num_streams = num_streams;
	slvs->streams = devm_kzalloc(slvs->dev,
				num_streams * sizeof(*stream), GFP_KERNEL);
	if (slvs->streams == NULL)
		return -ENOMEM;

	for (i = 0; i < num_streams; i++) {

		stream_node = tegra_slvs_get_stream_node(node, i);
		if (stream_node == NULL)
			goto error;

		stream = &slvs->streams[i];
		stream->slvs = slvs;

		stream->subdev.of_node = stream_node;

		stream->pads[0].flags = MEDIA_PAD_FL_SINK;
		stream->pads[1].flags = MEDIA_PAD_FL_SOURCE;

		tegra_slvs_parse_stream_dt(stream, stream_node);
	}

	return 0;

error:
	for (i = 0; i < num_streams; i++)
		of_node_put(slvs->streams[i].subdev.of_node);

	return -ENODEV;
}

void tegra_slvs_media_controller_cil_notify(struct tegra_mc_slvs *slvs,
					u32 id, u32 events)
{
	int i;

	if (IS_ERR_OR_NULL(slvs))
		return;

	for (i = 0; i < slvs->num_streams; i++) {
		if (slvs->streams[i].id == id)
			tegra_slvs_cil_notify(&slvs->streams[i], events);
	}
}
EXPORT_SYMBOL(tegra_slvs_media_controller_cil_notify);

struct tegra_mc_slvs *tegra_slvs_media_controller_init(
	struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct tegra_mc_slvs *slvs;
	int err;

	slvs = devm_kzalloc(&pdev->dev, sizeof(*slvs), GFP_KERNEL);
	slvs->dev = &pdev->dev;
	slvs->pdev = pdev;
	slvs->slvs_base = pdata->aperture[0];

	atomic_set(&slvs->power_ref, 0);

	err = tegra_slvs_parse_dt(slvs);
	if (err < 0)
		return ERR_PTR(err);

	/*
	 * if there is no slvs channels listed in DT,
	 * no need to init the channel and graph
	 */
	if (slvs->num_streams > 0) {
		err = tegra_slvs_init_streams(slvs);
		if (err < 0)
			dev_err(&pdev->dev, "Failed to init slvs streams\n");
		else
			dev_info(&pdev->dev, "Initialized slvs streams for V4L2\n");
	}

	return slvs;
}
EXPORT_SYMBOL(tegra_slvs_media_controller_init);

void tegra_slvs_media_controller_remove(struct tegra_mc_slvs *slvs)
{
	int i;

	if (IS_ERR_OR_NULL(slvs))
		return;

	for (i = 0; i < slvs->num_streams; i++) {
		struct tegra_slvs_stream *stream = &slvs->streams[i];
		v4l2_async_unregister_subdev(&stream->subdev);
		media_entity_cleanup(&stream->subdev.entity);

		of_node_put(stream->subdev.of_node);
	}
}
EXPORT_SYMBOL(tegra_slvs_media_controller_remove);
