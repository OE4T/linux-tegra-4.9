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

struct tegracam_device *to_tegracam_device(struct camera_common_data *data)
{
	/* fix this by moving subdev to base struct */
	return (struct tegracam_device *)data->tegracam_ctrl_hdl->tc_dev;
}
EXPORT_SYMBOL_GPL(to_tegracam_device);

void tegracam_set_privdata(struct tegracam_device * tc_dev, void *priv)
{
	tc_dev->priv = priv;

	/* TODO: cleanup needed for priv once sensors adapt this driver */
	tc_dev->s_data->priv = priv;
}
EXPORT_SYMBOL_GPL(tegracam_set_privdata);

void *tegracam_get_privdata(struct tegracam_device *tc_dev)
{
	return tc_dev->priv;
}
EXPORT_SYMBOL_GPL(tegracam_get_privdata);

int tegracam_device_register(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tegracam_ctrl_handler *ctrl_hdl = NULL;
	struct camera_common_power_rail *pw_rail = NULL;
	struct camera_common_data *s_data = NULL;
	struct sensor_mode_properties *sensor_mode = NULL;
	struct sensor_signal_properties *signal_props = NULL;
	struct sensor_image_properties *image_props = NULL;
	u32 mode_idx = 0;
	int err = 0;

	s_data = devm_kzalloc(dev,
		sizeof(struct camera_common_data), GFP_KERNEL);
	s_data->dev = dev;

	ctrl_hdl = devm_kzalloc(dev,
		sizeof(struct tegracam_ctrl_handler), GFP_KERNEL);
	ctrl_hdl->tc_dev = tc_dev;
	s_data->tegracam_ctrl_hdl = ctrl_hdl;

	pw_rail = devm_kzalloc(dev,
		sizeof(struct camera_common_power_rail), GFP_KERNEL);
	s_data->power = pw_rail;

	s_data->regmap = devm_regmap_init_i2c(tc_dev->client,
					tc_dev->dev_regmap_config);
	if (IS_ERR(s_data->regmap)) {
		dev_err(dev,
			"regmap init failed: %ld\n", PTR_ERR(s_data->regmap));
		return -ENODEV;
	}

	if (!tc_dev->sensor_ops) {
		dev_err(dev, "sensor ops not initialized\n");
		return -EINVAL;
	}
	s_data->ops = tc_dev->sensor_ops;

	s_data->pdata = tc_dev->sensor_ops->parse_dt(tc_dev);
	if (!s_data->pdata) {
		dev_err(dev, "unable to get platform data\n");
		return -EFAULT;
	}
	tc_dev->s_data = s_data;
	err = tc_dev->sensor_ops->power_get(tc_dev);
	if (err) {
		dev_err(dev, "unable to power get\n");
		return -EFAULT;
	}

	err = camera_common_initialize(s_data, tc_dev->name);
	if (err) {
		dev_err(dev, "Failed to initialize %s\n", tc_dev->name);
		return err;
	}

	/* TODO: updated default mode from DT ?? */
	mode_idx = s_data->mode_prop_idx = 0;
	/* init format context */
	/*TODO: compile frmfmt array from DT */
	s_data->frmfmt = tc_dev->sensor_ops->frmfmt_table;
	s_data->numfmts = tc_dev->sensor_ops->numfrmfmts;
	sensor_mode = &s_data->sensor_props.sensor_modes[mode_idx];
	signal_props = &sensor_mode->signal_properties;
	image_props = &sensor_mode->image_properties;

	s_data->def_mode = s_data->frmfmt[mode_idx].mode;
	s_data->colorfmt =
		camera_common_find_pixelfmt(image_props->pixel_format);
	s_data->def_width = s_data->fmt_width =
		s_data->frmfmt[mode_idx].size.width;
	s_data->def_height = s_data->fmt_height =
		s_data->frmfmt[mode_idx].size.height;
	s_data->def_clk_freq = signal_props->mclk_freq * 1000;

	return 0;
}
EXPORT_SYMBOL_GPL(tegracam_device_register);

void tegracam_device_unregister(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;

	tc_dev->sensor_ops->power_put(tc_dev);
	camera_common_cleanup(s_data);
}
EXPORT_SYMBOL_GPL(tegracam_device_unregister);
