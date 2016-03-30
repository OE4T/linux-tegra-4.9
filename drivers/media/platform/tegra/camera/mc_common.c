/*
 * Tegra Video Input device common APIs
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <media/tegra_v4l2_camera.h>
#include <media/camera_common.h>
#include <media/v4l2-event.h>

#include "dev.h"
#include "camera/mc_common.h"
#include "vi/vi.h"
#include "camera/registers.h"


void vi_write(struct tegra_mc_vi *vi, unsigned int addr, u32 val)
{
	writel(val, vi->iomem + addr);
}

/* In TPG mode, VI only support 2 formats */
static void vi_tpg_fmts_bitmap_init(struct tegra_mc_vi *vi)
{
	int index;

	bitmap_zero(vi->tpg_fmts_bitmap, MAX_FORMAT_NUM);

	index = tegra_core_get_idx_by_code(MEDIA_BUS_FMT_SRGGB10_1X10);
	bitmap_set(vi->tpg_fmts_bitmap, index, 1);

	index = tegra_core_get_idx_by_code(MEDIA_BUS_FMT_RGB888_1X32_PADHI);
	bitmap_set(vi->tpg_fmts_bitmap, index, 1);
}

int tegra_vi_power_on(struct tegra_mc_vi *vi)
{
	int ret;

	ret = nvhost_module_busy_ext(vi->ndev);
	if (ret) {
		dev_err(vi->dev, "%s:nvhost module is busy\n", __func__);
		return ret;
	}

	if (vi->reg) {
		ret = regulator_enable(vi->reg);
		if (ret) {
			dev_err(vi->dev, "%s: enable csi regulator failed.\n",
					__func__);
			goto error_regulator_fail;
		}
	}

	vi_write(vi, TEGRA_VI_CFG_CG_CTRL, 1);

	/* unpowergate VE */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
	if (ret) {
		dev_err(vi->dev, "failed to unpower gate VI\n");
		goto error_unpowergate;
	}

	/* clock settings */
	clk_prepare_enable(vi->clk);
	ret = clk_set_rate(vi->clk, TEGRA_CLOCK_VI_MAX);
	if (ret) {
		dev_err(vi->dev, "failed to set vi clock\n");
		goto error_clk_set_rate;
	}

	return 0;

error_clk_set_rate:
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);
error_unpowergate:
	regulator_disable(vi->reg);
error_regulator_fail:
	nvhost_module_idle_ext(vi->ndev);

	return ret;
}

void tegra_vi_power_off(struct tegra_mc_vi *vi)
{
	clk_disable_unprepare(vi->clk);
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	regulator_disable(vi->reg);
	nvhost_module_idle_ext(vi->ndev);
}

/* -----------------------------------------------------------------------------
 * Media Controller and V4L2
 */

static const char *const vi_pattern_strings[] = {
	"Disabled",
	"Black/White Direct Mode",
	"Color Patch Mode",
};

static int vi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegra_mc_vi *vi = container_of(ctrl->handler, struct tegra_mc_vi,
					   ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		/*
		 * TPG control is only avaiable to TPG driver,
		 * it can't be changed to 0 to disable TPG mode.
		 */
		if (ctrl->val) {
			dev_info(&vi->ndev->dev, "Set TPG mode to %d\n",
				 ctrl->val);
			vi->pg_mode = ctrl->val;
			vi->csi->pg_mode = vi->pg_mode;
		} else
			dev_warn(&vi->ndev->dev,
				 "TPG mode can't be disabled for TPG driver\n");
		break;
	default:
		dev_err(vi->dev, "%s:Not valid ctrl\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops vi_ctrl_ops = {
	.s_ctrl	= vi_s_ctrl,
};

void tegra_vi_v4l2_cleanup(struct tegra_mc_vi *vi)
{
	v4l2_ctrl_handler_free(&vi->ctrl_handler);
	v4l2_device_unregister(&vi->v4l2_dev);
	if (!vi->pg_mode)
		media_device_unregister(&vi->media_dev);
}
EXPORT_SYMBOL(tegra_vi_v4l2_cleanup);

static void tegra_vi_notify(struct v4l2_subdev *sd,
					  unsigned int notification, void *arg)
{
	struct tegra_mc_vi *vi = container_of(sd->v4l2_dev,
			struct tegra_mc_vi, v4l2_dev);
	unsigned ch, i;

	if (notification != V4L2_DEVICE_NOTIFY_EVENT)
		return;

	for (ch = 0; ch < vi->num_channels; ch++) {
		struct tegra_channel *chan = &vi->chans[ch];

		for (i = 0; i < chan->num_subdevs; i++)
			if (sd == chan->subdev[i])
				v4l2_event_queue(&chan->video, arg);
	}
}

int tegra_vi_v4l2_init(struct tegra_mc_vi *vi)
{
	int ret;

	vi_tpg_fmts_bitmap_init(vi);

	/*
	 * TPG mode need to reuse the real media_device struct of tegra_vi,
	 * so bypass the media_device_register() here.
	 */
	if (vi->pg_mode) {
		struct vi *tegra_vi = tegra_vi_get();
		if (tegra_vi)
			memcpy(&vi->media_dev, &tegra_vi->mc_vi.media_dev,
					sizeof(struct media_device));
	} else {
		vi->media_dev.dev = vi->dev;
		strlcpy(vi->media_dev.model, "NVIDIA Tegra Video Input Device",
				sizeof(vi->media_dev.model));
		vi->media_dev.hw_revision = 3;

		ret = media_device_register(&vi->media_dev);
		if (ret < 0) {
			dev_err(vi->dev, "media device registration failed (%d)\n",
					ret);
			return ret;
		}
	}

	vi->v4l2_dev.mdev = &vi->media_dev;
	vi->v4l2_dev.notify = tegra_vi_notify;
	ret = v4l2_device_register(vi->dev, &vi->v4l2_dev);
	if (ret < 0) {
		dev_err(vi->dev, "V4L2 device registration failed (%d)\n",
			ret);
		goto register_error;
	}

	v4l2_ctrl_handler_init(&vi->ctrl_handler, 1);
	vi->pattern = v4l2_ctrl_new_std_menu_items(&vi->ctrl_handler,
				&vi_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(vi_pattern_strings) - 1,
				0, vi->pg_mode, vi_pattern_strings);

	if (vi->ctrl_handler.error) {
		dev_err(vi->dev, "failed to add controls\n");
		ret = vi->ctrl_handler.error;
		goto ctrl_error;
	}
	vi->v4l2_dev.ctrl_handler = &vi->ctrl_handler;

	ret = v4l2_ctrl_handler_setup(&vi->ctrl_handler);
	if (ret < 0) {
		dev_err(vi->dev, "failed to set controls\n");
		goto ctrl_error;
	}
	return 0;


ctrl_error:
	v4l2_ctrl_handler_free(&vi->ctrl_handler);
	v4l2_device_unregister(&vi->v4l2_dev);
register_error:
	if (!vi->pg_mode)
		media_device_unregister(&vi->media_dev);
	return ret;
}

static int vi_get_clks(struct tegra_mc_vi *vi, struct platform_device *pdev)
{
	int ret = 0;

	vi->clk = devm_clk_get(&pdev->dev, "vi");
	if (IS_ERR(vi->clk)) {
		dev_err(&pdev->dev, "Failed to get vi clock\n");
		return PTR_ERR(vi->clk);
	}

	return ret;
}

static int vi_parse_dt(struct tegra_mc_vi *vi, struct platform_device *dev)
{
	int err = 0;
	int num_channels = 0;
	struct device_node *node = dev->dev.of_node;

	err = of_property_read_u32(node, "num-channels", &num_channels);
	if (err) {
		dev_err(&dev->dev,
			"Failed to find num of channels, reset to 1\n");
		/* Needed this WAR to have ap_sim sanity */
		num_channels = 1;
	}

	vi->num_channels = num_channels;
	vi->chans = devm_kzalloc(&dev->dev,
			(sizeof(struct tegra_channel) * num_channels),
			GFP_KERNEL);
	if (!vi->chans)
		return -ENOMEM;

	return 0;
}

static void set_vi_register_base(struct tegra_mc_vi *mc_vi,
			void __iomem *regbase)
{
	mc_vi->iomem = regbase;
}

int tegra_vi_media_controller_init(struct tegra_mc_vi *mc_vi,
				   struct platform_device *pdev)
{
	int err = 0;
	struct nvhost_device_data *pdata = pdev->dev.platform_data;

	set_vi_register_base(mc_vi, pdata->aperture[0]);

	err = vi_get_clks(mc_vi, pdev);
	if (err)
		return err;

	if (mc_vi->pg_mode) {
		mc_vi->chans = devm_kzalloc(&pdev->dev,
					    sizeof(struct tegra_channel) *
					    mc_vi->num_channels,
					    GFP_KERNEL);
		if (!mc_vi->chans)
			return -ENOMEM;
	} else {
		err = vi_parse_dt(mc_vi, pdev);
		if (err)
			goto mc_init_fail;
	}

	mc_vi->ndev = pdev;
	mc_vi->dev = &pdev->dev;
	INIT_LIST_HEAD(&mc_vi->entities);

	err = tegra_vi_v4l2_init(mc_vi);
	if (err < 0)
		goto mc_init_fail;

	/* Init Tegra VI channels */
	err = tegra_vi_channels_init(mc_vi);
	if (err < 0)
		goto channels_error;

	/* Setup media links between VI and external sensor subdev. */
	if (mc_vi->pg_mode)
		err = tegra_vi_tpg_graph_init(mc_vi);
	else
		err = tegra_vi_graph_init(mc_vi);
	if (err < 0)
		goto graph_error;

	return 0;

graph_error:
	tegra_vi_channels_cleanup(mc_vi);
channels_error:
	tegra_vi_v4l2_cleanup(mc_vi);
mc_init_fail:
	dev_err(&pdev->dev, "%s: failed\n", __func__);
	return err;
}

void tegra_vi_media_controller_cleanup(struct tegra_mc_vi *mc_vi)
{
	tegra_vi_graph_cleanup(mc_vi);
	tegra_vi_channels_cleanup(mc_vi);
	tegra_vi_v4l2_cleanup(mc_vi);
}
