/*
 * Tegra Video Input device common APIs
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#include "vi.h"
#include "vi_common.h"

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
	struct vi *vi = container_of(ctrl->handler, struct vi,
					   ctrl_handler);
	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		vi->pg_mode = ctrl->val;
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops vi_ctrl_ops = {
	.s_ctrl	= vi_s_ctrl,
};

void tegra_vi_v4l2_cleanup(struct vi *vi)
{
	v4l2_ctrl_handler_free(&vi->ctrl_handler);
	v4l2_device_unregister(&vi->v4l2_dev);
	media_device_unregister(&vi->media_dev);
}

int tegra_vi_v4l2_init(struct vi *vi)
{
	int ret;

	vi->pg_mode = 0;
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

	vi->v4l2_dev.mdev = &vi->media_dev;
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
					0, 0, vi_pattern_strings);

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
	media_device_unregister(&vi->media_dev);
	return ret;
}
