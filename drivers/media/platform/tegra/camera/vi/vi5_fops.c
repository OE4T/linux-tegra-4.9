/*
 * Tegra Video Input 5 device common APIs
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frank@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <media/mc_common.h>
#include <media/capture.h>
#include <linux/nvhost.h>
#include "nvhost_acm.h"
#include "linux/nvhost_ioctl.h"
#include "vi5_formats.h"
#include "vi5_fops.h"
#include <soc/tegra/chip-id.h>

static void vi5_init_video_formats(struct tegra_channel *chan)
{
	int i;

	dev_info(&chan->video.dev, "vi5_init_video_formats\n");

	chan->num_video_formats = ARRAY_SIZE(vi5_video_formats);
	for (i = 0; i < chan->num_video_formats; i++)
		chan->video_formats[i] = &vi5_video_formats[i];
}

static int tegra_vi5_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegra_channel *chan = container_of(ctrl->handler,
				struct tegra_channel, ctrl_handler);
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_WRITE_ISPFORMAT:
		chan->write_ispformat = ctrl->val;
		break;
	default:
		dev_err(&chan->video.dev, "%s:Not valid ctrl\n", __func__);
		return -EINVAL;
	}

	return err;
}

static const struct v4l2_ctrl_ops vi5_ctrl_ops = {
	.s_ctrl	= tegra_vi5_s_ctrl,
};

static const struct v4l2_ctrl_config vi5_custom_ctrls[] = {
	{
		.ops = &vi5_ctrl_ops,
		.id = TEGRA_CAMERA_CID_WRITE_ISPFORMAT,
		.name = "Write ISP format",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.def = 1,
		.min = 1,
		.max = 1,
		.step = 1,
	},
};

static int vi5_add_ctrls(struct tegra_channel *chan)
{
	int i;

	dev_info(&chan->video.dev, "vi5_add_ctrls\n");

	/* Add vi5 custom controls */
	for (i = 0; i < ARRAY_SIZE(vi5_custom_ctrls); i++) {
		v4l2_ctrl_new_custom(&chan->ctrl_handler,
			&vi5_custom_ctrls[i], NULL);
		if (chan->ctrl_handler.error) {
			dev_err(chan->vi->dev,
				"Failed to add %s ctrl\n",
				vi5_custom_ctrls[i].name);
			return chan->ctrl_handler.error;
		}
	}

	return 0;
}

static int vi5_channel_start_streaming(struct vb2_queue *vq, u32 count)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);

	dev_info(&chan->video.dev, "vi5_channel_start_streaming\n");

	tegra_channel_set_stream(chan, true);

	return 0;
}

static int vi5_channel_stop_streaming(struct vb2_queue *vq)
{
	struct tegra_channel *chan = vb2_get_drv_priv(vq);

	dev_info(&chan->video.dev, "vi5_channel_stop_streaming\n");

	tegra_channel_set_stream(chan, false);

	return 0;
}
static int t19x_sim_gpio(struct device *dev, char on)
{
#define VI_THI_SIZE	0x10000UL
#define VI_THI_BASE	0x15f00000
#define VI_CFG_VGP1_0	0x5068
#define VGP1_OUTPUT_DATA	1
#define VGP1_OUTPUT_ENABLE	(1 << 16)

	void *vi_thi_base = NULL;
	unsigned int val;

	vi_thi_base = ioremap_nocache(VI_THI_BASE, VI_THI_SIZE);
	if (!vi_thi_base) {
		dev_err(dev, "Can't map vi-thi base");
		return -EBUSY;
	}
	val = readl(vi_thi_base + VI_CFG_VGP1_0);
	if (on)
		writel(val | VGP1_OUTPUT_DATA | VGP1_OUTPUT_ENABLE,
			vi_thi_base + VI_CFG_VGP1_0);
	else
		writel(val & (~VGP1_OUTPUT_DATA) & (~VGP1_OUTPUT_ENABLE),
			vi_thi_base + VI_CFG_VGP1_0);
	iounmap(vi_thi_base);
	return 0;
}

static int vi5_power_on(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi = chan->vi;

	dev_info(&chan->video.dev, "vi5_power_on\n");

	ret = nvhost_module_add_client(vi->ndev, &chan->video);
	if (ret < 0)
		return ret;

	ret = nvhost_module_busy(vi->ndev);
	if (ret < 0)
		return ret;

	if (atomic_add_return(1, &chan->power_on_refcnt) == 1) {
		if (!tegra_platform_is_silicon())
			t19x_sim_gpio(vi->dev, 1);
		ret = tegra_channel_set_power(chan, 1);
		if (ret < 0) {
			dev_err(&chan->video.dev, "Failed to power on subdevices\n");
			return ret;
		}
	}

	return 0;
}

static void vi5_power_off(struct tegra_channel *chan)
{
	int ret = 0;
	struct tegra_mc_vi *vi = chan->vi;

	dev_info(&chan->video.dev, "vi5_power_off\n");

	if (atomic_dec_and_test(&chan->power_on_refcnt)) {
		ret = tegra_channel_set_power(chan, 0);
		if (ret < 0)
			dev_err(&chan->video.dev, "Failed to power off subdevices\n");
		if (!tegra_platform_is_silicon())
			t19x_sim_gpio(vi->dev, 0);
	}

	nvhost_module_idle(vi->ndev);
	nvhost_module_remove_client(vi->ndev, &chan->video);
}

struct tegra_vi_fops vi5_fops = {
	.vi_power_on = vi5_power_on,
	.vi_power_off = vi5_power_off,
	.vi_start_streaming = vi5_channel_start_streaming,
	.vi_stop_streaming = vi5_channel_stop_streaming,
	.vi_add_ctrls = vi5_add_ctrls,
	.vi_init_video_formats = vi5_init_video_formats,
};
