/*
 * tegracam_core - tegra camera framework initialization
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/types.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>

static int v4l2sd_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct camera_common_sensor_ops *sensor_ops = s_data->ops;
	struct tegracam_device *tc_dev = to_tegracam_device(s_data);
	int err;

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	if (enable) {
		err = sensor_ops->set_mode(tc_dev);
		if (err) {
			dev_err(&client->dev, "Error writing mode\n");
			return err;
		}

		if (s_data->override_enable) {
			err = tegracam_ctrl_set_overrides(
					s_data->tegracam_ctrl_hdl);
			if (err) {
				dev_err(&client->dev,
					"overrides cannot be set\n");
				return err;
			}
		}

		err = sensor_ops->start_streaming(tc_dev);
		if (err) {
			dev_err(&client->dev, "Error turning on streaming\n");
			return err;
		}
	} else {
		err = sensor_ops->stop_streaming(tc_dev);
		if (err) {
			dev_err(&client->dev, "Error turning off streaming\n");
			return err;
		}
	}

	return 0;
}

static int v4l2sd_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct camera_common_power_rail *pw = s_data->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops v4l2sd_video_ops = {
	.s_stream	= v4l2sd_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = v4l2sd_g_input_status,
};

static struct v4l2_subdev_core_ops v4l2sd_core_ops = {
	.s_power	= camera_common_s_power,
};

static int v4l2sd_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int v4l2sd_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	/* TODO: Add set mode for blob collection */

	return ret;
}

static struct v4l2_subdev_pad_ops v4l2sd_pad_ops = {
	.set_fmt = v4l2sd_set_fmt,
	.get_fmt = v4l2sd_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops v4l2sd_ops = {
	.core	= &v4l2sd_core_ops,
	.video	= &v4l2sd_video_ops,
	.pad = &v4l2sd_pad_ops,
};

static const struct media_entity_operations media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

int tegracam_v4l2subdev_register(struct tegracam_device *tc_dev,
				bool is_sensor)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct tegracam_ctrl_handler *ctrl_hdl = s_data->tegracam_ctrl_hdl;
	struct v4l2_subdev *sd = NULL;
	struct device *dev = tc_dev->dev;
	int err = 0;

	/* init v4l2 subdevice for registration */
	sd = &s_data->subdev;
	if (!sd || !tc_dev->client) {
		dev_err(dev, "Invalid subdev context\n");
		return -ENODEV;
	}

	if (!tc_dev->tcctrl_ops) {
		dev_err(dev, "uninitialized control ops\n");
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(sd, tc_dev->client, &v4l2sd_ops);

	ctrl_hdl->ctrl_ops = tc_dev->tcctrl_ops;
	err = tegracam_ctrl_handler_init(ctrl_hdl);
	if (err) {
		dev_err(dev, "Failed to init ctrls %s\n", tc_dev->name);
		return err;
	}
	tc_dev->numctrls = ctrl_hdl->ctrl_ops->numctrls;
	s_data->numctrls = tc_dev->numctrls;
	sd->ctrl_handler = s_data->ctrl_handler = &ctrl_hdl->ctrl_handler;
	s_data->ctrls = ctrl_hdl->ctrls;
	sd->internal_ops = tc_dev->v4l2sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	tc_dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.ops = &media_ops;
	err = tegra_media_entity_init(&sd->entity,
			1, &tc_dev->pad, true, is_sensor);
	if (err < 0) {
		dev_err(dev, "unable to init media entity\n");
		return err;
	}
#endif

	return v4l2_async_register_subdev(sd);
}
EXPORT_SYMBOL_GPL(tegracam_v4l2subdev_register);

void tegracam_v4l2subdev_unregister(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct v4l2_subdev *sd = &s_data->subdev;

	v4l2_ctrl_handler_free(s_data->ctrl_handler);
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
}
EXPORT_SYMBOL_GPL(tegracam_v4l2subdev_unregister);
