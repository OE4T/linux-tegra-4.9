/*
 * imx390.c - imx390 sensor driver
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/max9295.h>
#include <media/max9296.h>

#include <media/tegra_v4l2_camera.h>
#include <media/camera_common.h>
#include "imx390_mode_tbls.h"

#define IMX390_DEFAULT_MODE	IMX390_MODE_1920X1080_CROP_30FPS
#define IMX390_MIN_GAIN         (0)
#define IMX390_MAX_GAIN         (30)
#define IMX390_MAX_GAIN_REG         (100)
#define IMX390_DEFAULT_GAIN     IMX390_MIN_GAIN

#define IMX390_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB12_1X12
#define IMX390_MIN_FRAME_LENGTH	(1125)
#define IMX390_MAX_FRAME_LENGTH	(0x1FFFF)
#define IMX390_MIN_SHS1_1080P_HDR	(5)

#define IMX390_MIN_SHS2_1080P_HDR	(82)
#define IMX390_MAX_SHS2_1080P_HDR	(IMX390_MAX_FRAME_LENGTH - 5)
#define IMX390_MAX_SHS1_1080P_HDR	(IMX390_MAX_SHS2_1080P_HDR / 16)

#define IMX390_FRAME_LENGTH_ADDR_MSB		0x200A
#define IMX390_FRAME_LENGTH_ADDR_MID		0x2009
#define IMX390_FRAME_LENGTH_ADDR_LSB		0x2008
#define IMX390_COARSE_TIME_SHS1_ADDR_MSB	0x3022
#define IMX390_COARSE_TIME_SHS1_ADDR_MID	0x3021
#define IMX390_COARSE_TIME_SHS1_ADDR_LSB	0x3020
#define IMX390_COARSE_TIME_SHS2_ADDR_MSB	0x3025
#define IMX390_COARSE_TIME_SHS2_ADDR_MID	0x3024
#define IMX390_COARSE_TIME_SHS2_ADDR_LSB	0x3023
#define IMX390_GAIN_ADDR			0x3014
#define IMX390_GROUP_HOLD_ADDR			0x0001
#define IMX390_ANALOG_GAIN_SP1H_ADDR            0x0018
#define IMX390_ANALOG_GAIN_SP1L_ADDR            0x001A

#define IMX390_DEFAULT_WIDTH	1936
#define IMX390_DEFAULT_HEIGHT	1100
#define IMX390_DEFAULT_CLK_FREQ	24000000

struct imx390 {
	struct camera_common_power_rail	power;
	int	numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct i2c_client	*i2c_client;
	struct v4l2_subdev	*subdev;
	struct device		*ser_dev;
	struct device		*dser_dev;
	struct gmsl_link_data	gmsl_data;
	struct media_pad	pad;
	u32	frame_length;
	s32	group_hold_prev;
	bool	group_hold_en;
	s64 last_wdr_et_val;
	struct regmap	*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int imx390_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx390_ctrl_ops = {
	.s_ctrl = imx390_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX390_MIN_GAIN * FIXED_POINT_SCALING_FACTOR,
		.max = IMX390_MAX_GAIN * FIXED_POINT_SCALING_FACTOR,
		.def = IMX390_MAX_GAIN * FIXED_POINT_SCALING_FACTOR,
		.step = 1,
	},
	{
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		.max = 1000000LL * FIXED_POINT_SCALING_FACTOR / 1000000,
		.def = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
		.step = 1,
	},
	{
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = 1 * FIXED_POINT_SCALING_FACTOR,
		.max = 30 * FIXED_POINT_SCALING_FACTOR,
		.def = 30 * FIXED_POINT_SCALING_FACTOR,
		.step = 1,
	},
	{
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &imx390_ctrl_ops,
		.id = TEGRA_CAMERA_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
};

static inline void imx390_get_frame_length_regs(imx390_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX390_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 16) & 0x01;

	(regs + 1)->addr = IMX390_FRAME_LENGTH_ADDR_MID;
	(regs + 1)->val = (frame_length >> 8) & 0xff;

	(regs + 2)->addr = IMX390_FRAME_LENGTH_ADDR_LSB;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void imx390_get_coarse_time_regs_shs1(imx390_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX390_COARSE_TIME_SHS1_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x01;

	(regs + 1)->addr = IMX390_COARSE_TIME_SHS1_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = IMX390_COARSE_TIME_SHS1_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;

}

static inline void imx390_get_coarse_time_regs_shs2(imx390_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX390_COARSE_TIME_SHS2_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x01;

	(regs + 1)->addr = IMX390_COARSE_TIME_SHS2_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;

	(regs + 2)->addr = IMX390_COARSE_TIME_SHS2_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;

}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx390_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx390_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct imx390 *priv = (struct imx390 *)s_data->priv;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		pr_err("%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx390_write_table(struct imx390 *priv,
				const imx390_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX390_TABLE_WAIT_MS,
					 IMX390_TABLE_END);
}

static int imx390_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);
	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

	usleep_range(30, 50);

	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);

	usleep_range(30, 50);

	/*exit reset mode: XCLR */
	if (pw->reset_gpio) {
		gpio_set_value(pw->reset_gpio, 0);
		usleep_range(30, 50);
		gpio_set_value(pw->reset_gpio, 1);
		usleep_range(30, 50);
	}

	msleep(20);

	max9296_poweron(priv->dser_dev);
	msleep(20);
	max9295_poweron(priv->ser_dev);

	pw->state = SWITCH_ON;
	return 0;

}

static int imx390_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			pr_err("%s failed.\n", __func__);
		return err;
	}

	/* enter reset mode: XCLR */
	usleep_range(1, 2);
	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

	if (pw->dvdd)
		regulator_disable(pw->dvdd);

	max9295_poweroff(priv->ser_dev);
	msleep(20);
	max9296_poweroff(priv->dser_dev);

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int imx390_power_get(struct imx390 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ?
		    priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,
			"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(&priv->i2c_client->dev,
				"unable to get parent clcok %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}

	if (!err)
		pw->reset_gpio = pdata->reset_gpio;

	/* digital 1.2v */
	if (pdata->regulators.dvdd != NULL)
		err |= camera_common_regulator_get(&priv->i2c_client->dev,
				&pw->dvdd, pdata->regulators.dvdd);

	pw->state = SWITCH_OFF;

	return err;
}
static int imx390_set_coarse_time(struct imx390 *priv, s64 val);
static int imx390_set_coarse_time_hdr(struct imx390 *priv, s64 val);
static int imx390_set_exposure(struct imx390 *priv, s64 val);
static int imx390_set_gain(struct imx390 *priv, s64 val);
static int imx390_set_frame_rate(struct imx390 *priv, s64 val);

static int imx390_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	struct v4l2_ext_controls ctrls;
	struct v4l2_ext_control control[3];
	int err;

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	if (!enable) {
		err =  imx390_write_table(priv,
			mode_table[IMX390_MODE_STOP_STREAM]);

		if (err)
			return err;
		return 0;
	}

	max9295_stream_setup(priv->ser_dev);
	msleep(20);
	max9296_stream_setup(priv->dser_dev);
	msleep(20);
	max9295_streamon(priv->ser_dev);
	msleep(20);
	max9296_streamon(priv->dser_dev);
	msleep(20);

	err = imx390_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;

	if (s_data->override_enable) {
		/* write list of override regs for the asking gain, */
		/* frame rate and exposure time    */
		memset(&ctrls, 0, sizeof(ctrls));

		ctrls.count = 3;
		ctrls.controls = control;

		control[0].id = TEGRA_CAMERA_CID_GAIN;
		control[1].id = TEGRA_CAMERA_CID_FRAME_RATE;
		control[2].id = TEGRA_CAMERA_CID_EXPOSURE;

		err = v4l2_g_ext_ctrls(&priv->ctrl_handler, &ctrls);
		if (err == 0) {
			err |= imx390_set_gain(priv, control[0].value64);
			if (err)
				dev_err(&client->dev,
					"%s: error gain override\n", __func__);

			err |= imx390_set_frame_rate(priv, control[1].value64);
			if (err)
				dev_err(&client->dev,
					"%s: error frame length override\n",
					__func__);

			err |= imx390_set_exposure(priv, control[2].value64);
			if (err)
				dev_err(&client->dev,
					"%s: error exposure override\n",
					__func__);
		} else {
			dev_err(&client->dev, "%s: faile to get overrides\n",
				__func__);
		}
	}

	err = imx390_write_table(priv, mode_table[IMX390_MODE_START_STREAM]);
	if (err)
		goto exit;

	msleep(20);

	return 0;
exit:
	dev_err(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx390_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx390 *priv = (struct imx390 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx390_subdev_video_ops = {
	.s_stream	= imx390_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = imx390_g_input_status,
};

static struct v4l2_subdev_core_ops imx390_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int imx390_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int imx390_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops imx390_subdev_pad_ops = {
	.set_fmt = imx390_set_fmt,
	.get_fmt = imx390_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops imx390_subdev_ops = {
	.core	= &imx390_subdev_core_ops,
	.video	= &imx390_subdev_video_ops,
	.pad = &imx390_subdev_pad_ops,
};

const struct of_device_id imx390_of_match[] = {
	{ .compatible = "nvidia,imx390",},
	{ },
};

static struct camera_common_sensor_ops imx390_common_ops = {
	.power_on = imx390_power_on,
	.power_off = imx390_power_off,
	.write_reg = imx390_write_reg,
	.read_reg = imx390_read_reg,
};

static int imx390_set_group_hold(struct imx390 *priv, s32 val)
{
	int err;
	int gh_en = switch_ctrl_qmenu[val];

	priv->group_hold_prev = val;

	if (gh_en == SWITCH_ON) {

		err = imx390_write_reg(priv->s_data,
				       IMX390_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
	} else if (gh_en == SWITCH_OFF) {
		err = imx390_write_reg(priv->s_data,
				       IMX390_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
	}

	return 0;
fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: Group hold control error\n", __func__);
	return err;
}

static int imx390_set_gain(struct imx390 *priv, s64 val)
{
	imx390_reg reg;
	int err;
	s64 gain64;
	u8 gain;

	/* translate value */
	gain64 = (s64)(val * 10 / FIXED_POINT_SCALING_FACTOR);
	gain = (u8)(gain64 / 3);

	dev_dbg(&priv->i2c_client->dev,
		"%s:  gain reg: %d, db: %lld\n",  __func__, gain, gain64);

	if (gain < IMX390_MIN_GAIN)
		gain = IMX390_MIN_GAIN;
	else if (gain > IMX390_MAX_GAIN_REG)
		gain = IMX390_MAX_GAIN_REG;

	reg.val = gain;
	reg.addr = IMX390_ANALOG_GAIN_SP1H_ADDR;
	err = imx390_write_reg(priv->s_data, reg.addr, reg.val);
	if (err)
		goto fail;

	reg.addr = IMX390_ANALOG_GAIN_SP1L_ADDR;
	reg.val = gain/8;

	err = imx390_write_reg(priv->s_data, reg.addr, reg.val);
	if (err)
		goto fail;

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		"%s: GAIN control error\n", __func__);
	return err;
}

static int imx390_set_frame_rate(struct imx390 *priv, s64 val)
{
	imx390_reg reg_list[3];
	int err;
	s64 frame_length;
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	int i = 0;

	frame_length = mode->signal_properties.pixel_clock.val *
		FIXED_POINT_SCALING_FACTOR /
		mode->image_properties.line_length / val;

	priv->frame_length = (u32) frame_length;
	if (priv->frame_length > IMX390_MAX_FRAME_LENGTH)
		priv->frame_length = IMX390_MAX_FRAME_LENGTH;

	dev_dbg(&priv->i2c_client->dev,
		"%s: val: %lld, , frame_length: %d\n", __func__,
		val, priv->frame_length);

	imx390_get_frame_length_regs(reg_list, priv->frame_length);

	for (i = 0; i < 3; i++) {
		err = imx390_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx390_set_exposure(struct imx390 *priv, s64 val)
{
	int err;
	struct v4l2_control control;
	int hdr_en;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: val: %lld\n", __func__, val);

	/* check hdr enable ctrl */
	control.id = TEGRA_CAMERA_CID_HDR_EN;
	err = camera_common_g_ctrl(priv->s_data, &control);
	if (err < 0) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return err;
	}

	hdr_en = switch_ctrl_qmenu[control.value];
	if (hdr_en == SWITCH_ON) {
		err = imx390_set_coarse_time_hdr(priv, val);
		if (err)
			dev_dbg(&priv->i2c_client->dev,
			"%s: error coarse time SHS1 SHS2 override\n", __func__);
	} else {
		err = imx390_set_coarse_time(priv, val);
		if (err)
			dev_dbg(&priv->i2c_client->dev,
			"%s: error coarse time SHS1 override\n", __func__);
	}

	return err;
}

static int imx390_set_coarse_time(struct imx390 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	imx390_reg reg_list[3];
	int err;
	u32 coarse_time_shs1;
	u32 reg_shs1;
	int i = 0;

	coarse_time_shs1 = mode->signal_properties.pixel_clock.val * val /
		mode->image_properties.line_length / FIXED_POINT_SCALING_FACTOR;

	if (priv->frame_length == 0)
		priv->frame_length = IMX390_MIN_FRAME_LENGTH;

	reg_shs1 = priv->frame_length - coarse_time_shs1 - 1;

	dev_dbg(&priv->i2c_client->dev,
		 "%s: coarse1:%d, shs1:%d, FL:%d\n", __func__,
		 coarse_time_shs1, reg_shs1, priv->frame_length);

	imx390_get_coarse_time_regs_shs1(reg_list, reg_shs1);

	for (i = 0; i < 3; i++) {
		err = imx390_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: set coarse time error\n", __func__);
	return err;
}

static int imx390_set_coarse_time_hdr(struct imx390 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	imx390_reg reg_list_shs1[3];
	imx390_reg reg_list_shs2[3];
	u32 coarse_time_shs1;
	u32 coarse_time_shs2;
	u32 reg_shs1;
	u32 reg_shs2;
	int err;
	int i = 0;

	if (priv->frame_length == 0)
		priv->frame_length = IMX390_MIN_FRAME_LENGTH;

	priv->last_wdr_et_val = val;

	/*WDR, update SHS1 as short ET, and SHS2 is 16x of short*/
	coarse_time_shs1 = mode->signal_properties.pixel_clock.val * val /
		mode->image_properties.line_length /
		FIXED_POINT_SCALING_FACTOR / 16;
	if (coarse_time_shs1 < IMX390_MIN_SHS1_1080P_HDR)
		coarse_time_shs1 = IMX390_MIN_SHS1_1080P_HDR;
	if (coarse_time_shs1 > IMX390_MAX_SHS1_1080P_HDR)
		coarse_time_shs1 = IMX390_MAX_SHS1_1080P_HDR;

	coarse_time_shs2 = (coarse_time_shs1 - IMX390_MIN_SHS1_1080P_HDR) * 16 +
				IMX390_MIN_SHS2_1080P_HDR;

	reg_shs1 = priv->frame_length - coarse_time_shs1 - 1;
	reg_shs2 = priv->frame_length - coarse_time_shs2 - 1;

	imx390_get_coarse_time_regs_shs1(reg_list_shs1, reg_shs1);
	imx390_get_coarse_time_regs_shs2(reg_list_shs2, reg_shs2);

	dev_dbg(&priv->i2c_client->dev,
		"%s: coarse1:%d, shs1:%d, coarse2:%d, shs2: %d, FL:%d\n",
		__func__,
		 coarse_time_shs1, reg_shs1,
		 coarse_time_shs2, reg_shs2,
		 priv->frame_length);

	for (i = 0; i < 3; i++) {
		err = imx390_write_reg(priv->s_data, reg_list_shs1[i].addr,
			 reg_list_shs1[i].val);
		if (err)
			goto fail;

		err = imx390_write_reg(priv->s_data, reg_list_shs2[i].addr,
			 reg_list_shs2[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(&priv->i2c_client->dev,
		 "%s: set WDR coarse time error\n", __func__);
	return err;
}

static int imx390_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx390 *priv =
		container_of(ctrl->handler, struct imx390, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = imx390_set_gain(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		err = imx390_set_exposure(priv, *ctrl->p_new.p_s64);
	case TEGRA_CAMERA_CID_FRAME_RATE:
		err = imx390_set_frame_rate(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		err = imx390_set_group_hold(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx390_ctrls_init(struct imx390 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(&client->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->numctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, imx390_of_match);

static struct camera_common_pdata *imx390_parse_dt(struct imx390 *priv,
				struct i2c_client *client,
				struct camera_common_data *s_data)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct device_node *ser_node;
	struct i2c_client *ser_i2c = NULL;
	struct device_node *dser_node;
	struct i2c_client *dser_i2c = NULL;
	struct device_node *gmsl;
	int value = 0xFFFF;
	const char *str_value;
	const char *str_value1[2];
	int  i;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(imx390_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(&client->dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio =
			of_get_named_gpio(node, "reset-gpios", 0);
	if (err) {
		dev_err(&client->dev,
			"reset-gpios not found %d\n", err);
		goto error;
	}

	of_property_read_string(node, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);

	ser_node = of_parse_phandle(node, "nvidia,gmsl-ser-device", 0);
	if (ser_node == NULL) {
		dev_err(&client->dev,
			"missing %s handle\n",
				"nvidia,gmsl-ser-device");
		goto error;
	}

	ser_i2c = of_find_i2c_device_by_node(ser_node);
	of_node_put(ser_node);

	if (ser_i2c == NULL) {
		dev_err(&client->dev, "missing serializer dev handle\n");
		goto error;
	}
	if (ser_i2c->dev.driver == NULL) {
		dev_err(&client->dev, "missing serializer driver\n");
		goto error;
	}

	priv->ser_dev = &ser_i2c->dev;

	dser_node = of_parse_phandle(node, "nvidia,gmsl-dser-device", 0);
	if (dser_node == NULL) {
		dev_err(&client->dev,
			"missing %s handle\n",
				"nvidia,gmsl-dser-device");
		goto error;
	}

	dser_i2c = of_find_i2c_device_by_node(dser_node);
	of_node_put(dser_node);

	if (dser_i2c == NULL) {
		dev_err(&client->dev, "missing deserializer dev handle\n");
		goto error;
	}
	if (dser_i2c->dev.driver == NULL) {
		dev_err(&client->dev, "missing deserializer driver\n");
		goto error;
	}

	priv->dser_dev = &dser_i2c->dev;

	/* populate gmsl_data from DT */
	gmsl = of_get_child_by_name(node, "gmsl-link");
	if (gmsl == NULL) {
		dev_err(&client->dev, "missing gmsl-link device node\n");
		err = -EINVAL;
		goto error;
	}

	err = of_property_read_string(gmsl, "dst-csi-port", &str_value);
	if (err < 0) {
		dev_err(&client->dev, "No dst-csi-port found\n");
		goto error;
	}
	priv->gmsl_data.dst_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	err = of_property_read_string(gmsl, "src-csi-port", &str_value);
	if (err < 0) {
		dev_err(&client->dev, "No src-csi-port found\n");
		goto error;
	}
	priv->gmsl_data.src_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	err = of_property_read_string(gmsl, "csi-mode", &str_value);
	if (err < 0) {
		dev_err(&client->dev, "No csi-mode found\n");
		goto error;
	}

	if (!strcmp(str_value, "1x4")) {
		priv->gmsl_data.csi_mode = GMSL_CSI_1X4_MODE;
	} else if (!strcmp(str_value, "2x4")) {
		priv->gmsl_data.csi_mode = GMSL_CSI_2X4_MODE;
	} else if (!strcmp(str_value, "4x2")) {
		priv->gmsl_data.csi_mode = GMSL_CSI_4X2_MODE;
	} else if (!strcmp(str_value, "2x2")) {
		priv->gmsl_data.csi_mode = GMSL_CSI_2X2_MODE;
	} else {
		dev_err(&client->dev, "invalid csi mode\n");
		goto error;
	}

	err = of_property_read_u32(gmsl, "st-vc", &value);
	if (err < 0) {
		dev_err(&client->dev, "No st-vc info\n");
		goto error;
	}
	priv->gmsl_data.st_vc = value;

	err = of_property_read_u32(gmsl, "vc-id", &value);
	if (err < 0) {
		dev_err(&client->dev, "No vc-id info\n");
		goto error;
	}
	priv->gmsl_data.dst_vc = value;

	err = of_property_read_u32(gmsl, "num-lanes", &value);
	if (err < 0) {
		dev_err(&client->dev, "No num-lanes info\n");
		goto error;
	}
	priv->gmsl_data.num_csi_lanes = value;

	priv->gmsl_data.num_streams =
			of_property_count_strings(gmsl, "streams");
	if (priv->gmsl_data.num_streams <= 0) {
		dev_err(&client->dev, "No streams found\n");
		err = -EINVAL;
		goto error;
	}

	for (i = 0; i < priv->gmsl_data.num_streams; i++) {
		of_property_read_string_index(gmsl, "streams", i,
						&str_value1[i]);
		if (!str_value1[i]) {
			dev_err(&client->dev, "invalid stream info\n");
			goto error;
		}
		if (strcmp(str_value1[i], "raw12")) {
			priv->gmsl_data.streams[i].st_data_type =
							GMSL_CSI_DT_RAW_12;
		}
		else if (strcmp(str_value1[i], "ued-u1")) {
			priv->gmsl_data.streams[i].st_data_type =
							GMSL_CSI_DT_UED_U1;
		} else {
			dev_err(&client->dev, "invalid stream data type\n");
			goto error;
		}
	}

	priv->gmsl_data.s_dev = &client->dev;

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}

static int imx390_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx390_subdev_internal_ops = {
	.open = imx390_open,
};

static const struct media_entity_operations imx390_media_ops = {
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = v4l2_subdev_link_validate,
#endif
};

static int imx390_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct device_node *node = client->dev.of_node;
	struct imx390 *priv;
	int err;

	dev_info(&client->dev, "probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&client->dev,
			    sizeof(struct imx390) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	common_data->ops = &imx390_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->frmfmt = &imx390_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
					  IMX390_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(imx390_frmfmt);
	common_data->def_mode = IMX390_DEFAULT_MODE;
	common_data->def_width = IMX390_DEFAULT_WIDTH;
	common_data->def_height = IMX390_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = IMX390_DEFAULT_CLK_FREQ;

	priv->pdata = imx390_parse_dt(priv, client, common_data);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;
	priv->last_wdr_et_val = 0;

	/* Pair sensor dev to serializer dev */
	max9295_dev_pair(priv->ser_dev, &priv->gmsl_data);

	/* Add sensor dev to deserializer */
	max9296_dev_add(priv->dser_dev, &priv->gmsl_data);

	err = imx390_power_get(priv);
	if (err)
		return err;

	err = camera_common_initialize(common_data, "imx390");
	if (err) {
		dev_err(&client->dev, "Failed to initialize imx390.\n");
		return err;
	}

	v4l2_i2c_subdev_init(priv->subdev, client, &imx390_subdev_ops);

	err = imx390_ctrls_init(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx390_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &imx390_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity, 1,
		&priv->pad, true, true);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

	dev_info(&client->dev, "Detected IMX390 sensor\n");

	return 0;
}

static int imx390_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx390 *priv = (struct imx390 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id imx390_id[] = {
	{ "imx390", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx390_id);

static struct i2c_driver imx390_i2c_driver = {
	.driver = {
		.name = "imx390",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx390_of_match),
	},
	.probe = imx390_probe,
	.remove = imx390_remove,
	.id_table = imx390_id,
};

module_i2c_driver(imx390_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX390");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com");
MODULE_LICENSE("GPL v2");
