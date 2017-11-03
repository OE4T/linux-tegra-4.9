/*
 * imx318.c - imx318 sensor driver
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <media/tegra_v4l2_camera.h>
#include <media/camera_common.h>
#include <media/imx318.h>

#include "../platform/tegra/camera/camera_gpio.h"
#include "imx318_mode_tbls.h"


#define IMX318_MIN_GAIN			0
#define IMX318_MAX_GAIN			24
#define IMX318_DEFAULT_GAIN		IMX318_MIN_GAIN

#define IMX318_MIN_FRAME_LENGTH		0x0
#define IMX318_MAX_FRAME_LENGTH		0xfff5
#define IMX318_DEFAULT_FRAME_LENGTH	0x17b0

#define IMX318_MAX_COARSE_DIFF			10
#define IMX318_MIN_COARSE_INTEG_TIME	1
#define IMX318_MAX_COARSE_INTEG_TIME	\
	(IMX318_MAX_FRAME_LENGTH-IMX318_MAX_COARSE_DIFF)
#define IMX318_DEFAULT_COARSE_INTEG_TIME	\
	(IMX318_DEFAULT_FRAME_LENGTH-IMX318_MAX_COARSE_DIFF)

#define IMX318_DEFAULT_WIDTH	5488
#define IMX318_DEFAULT_HEIGHT	4112
#define IMX318_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SBGGR10_1X10
#define IMX318_DEFAULT_CLK_FREQ	24000000

#define IMX318_DEFAULT_MODE	IMX318_MODE_CPHY_4k_30FPS
#define IMX318_GAIN_ADDR_MSB					0x0204
#define IMX318_GAIN_ADDR_LSB					0x0205
#define IMX318_FRAME_LEGNTH_CTRL_EN				0x0350
#define IMX318_FRAME_LENGTH_ADDR_MSB			0x0340
#define IMX318_FRAME_LENGTH_ADDR_LSB			0x0341
#define IMX318_COARSE_INTEG_TIME_ADDR_MSB		0x0202
#define IMX318_COARSE_INTEG_TIME_ADDR_LSB		0x0203
#define IMX318_ST_COARSE_INTEG_TIME_ADDR_MSB	0x0224
#define IMX318_ST_COARSE_INTEG_TIME_ADDR_LSB	0x0225
#define IMX318_GROUP_HOLD_ADDR					0x0104
#define IMX318_HDR_EN_ADDR						0x3011


struct imx318 {
	struct mutex imx318_camera_lock;
	struct camera_common_power_rail	power;
	int	numctrls;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct camera_common_eeprom_data eeprom[IMX318_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[IMX318_EEPROM_SIZE];
	struct i2c_client	*i2c_client;
	struct v4l2_subdev	*subdev;
	struct media_pad	pad;
	s32				group_hold_prev;
	u32				frame_length;
	s32	group_hold_en;
	struct regmap	*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static int imx318_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx318_ctrl_ops = {
	.s_ctrl = imx318_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx318_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX318_MIN_GAIN,
		.max = IMX318_MAX_GAIN,
		.def = IMX318_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &imx318_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX318_MIN_COARSE_INTEG_TIME,
		.max = IMX318_MAX_COARSE_INTEG_TIME,
		.def = IMX318_DEFAULT_COARSE_INTEG_TIME,
		.step = 1,
	},
	{
		.ops = &imx318_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX318_MIN_FRAME_LENGTH,
		.max = IMX318_MAX_FRAME_LENGTH,
		.def = IMX318_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &imx318_ctrl_ops,
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
		.ops = &imx318_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EEPROM_DATA,
		.name = "EEPROM Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = IMX318_EEPROM_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &imx318_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = IMX318_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};

static inline void imx318_get_frame_length_regs(imx318_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX318_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX318_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void imx318_get_coarse_time_regs(imx318_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX318_COARSE_INTEG_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX318_COARSE_INTEG_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx318_get_gain_reg(imx318_reg *regs,
				u8 gain)
{
	regs->addr = IMX318_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x01;
	(regs + 1)->addr = IMX318_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx318_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx318 *priv = (struct imx318 *)s_data->priv;
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx318_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct imx318 *priv = (struct imx318 *)s_data->priv;
	struct device *dev = &priv->i2c_client->dev;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx318_write_table(struct imx318 *priv,
				const imx318_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
						table,
						NULL, 0,
						IMX318_TABLE_WAIT_MS,
						IMX318_TABLE_END);
}

static int imx318_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx318 *priv = (struct imx318 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);
	usleep_range(15, 20);
	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto imx318_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto imx318_iovdd_fail;

	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);
	if (err)
		goto imx318_dvdd_fail;

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);

	usleep_range(19000, 19010);
	pw->state = SWITCH_ON;

	return 0;

imx318_dvdd_fail:
	regulator_disable(pw->iovdd);

imx318_iovdd_fail:
	regulator_disable(pw->avdd);

imx318_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);
	return -ENODEV;
}

static int imx318_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct imx318 *priv = (struct imx318 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;
	struct device *dev = &priv->i2c_client->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_off) {
		err = priv->pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio)
			gpio_set_value(pw->reset_gpio, 0);
		usleep_range(15, 20);
		if (pw->avdd)
			regulator_disable(pw->avdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->dvdd)
			regulator_disable(pw->dvdd);
	}

	pw->state = SWITCH_OFF;
	return 0;
}

static int imx318_power_put(struct imx318 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (priv->pdata && priv->pdata->use_cam_gpio)
		cam_gpio_deregister(&priv->i2c_client->dev, pw->pwdn_gpio);
	else {
		gpio_free(pw->pwdn_gpio);
		gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int imx318_power_get(struct imx318 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	struct device *dev = &priv->i2c_client->dev;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0, ret = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ?
			pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clock %s",
				parentclk_name);
		} else
			clk_set_parent(pw->mclk, parent);
	}


	/* analog 2.8v */
	err |= camera_common_regulator_get(dev,
			&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(dev,
			&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	err |= camera_common_regulator_get(dev,
			&pw->dvdd, pdata->regulators.dvdd);

	if (!err)
		pw->reset_gpio = pdata->reset_gpio;

	ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (ret < 0)
		dev_dbg(dev, "%s can't request reset_gpio %d\n", __func__, ret);

	pw->state = SWITCH_OFF;
	return err;
}

static int imx318_set_coarse_time(struct imx318 *priv, s64 val);
static int imx318_set_gain(struct imx318 *priv, s64 val);
static int imx318_set_frame_length(struct imx318 *priv, s32 val);

static int imx318_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx318 *priv = (struct imx318 *)s_data->priv;
	struct v4l2_control control;
	int err;

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	if (!enable) {
		err = imx318_write_table(priv,
			mode_table[IMX318_MODE_STOP_STREAM]);
		if (err)
			return err;

		/*
		 * Wait for one frame to make sure sensor is set to
		 * software standby in V-blank
		 *
		 * delay = frame length rows * Tline (10 us)
		 */
		usleep_range(priv->frame_length * 10,
			priv->frame_length * 10 + 1000);
		return 0;
	}

	err = imx318_write_table(priv, mode_table[IMX318_MODE_COMMON]);
	if (err)
		goto exit;
	err = imx318_write_table(priv, mode_table[IMX318_DEFAULT_MODE]);
	if (err)
		goto exit;

	if (s_data->override_enable) {
		/*
		 * write list of override regs for the asking frame length,
		 * coarse integration time, and gain. Failures to write
		 * overrides are non-fatal
		 */
		control.id = TEGRA_CAMERA_CID_GAIN;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx318_set_gain(priv, control.value);
		if (err)
			dev_dbg(&client->dev, "%s: warning gain override failed\n",
				__func__);

		control.id = TEGRA_CAMERA_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx318_set_frame_length(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: warning frame length override failed\n",
				__func__);

		control.id = TEGRA_CAMERA_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx318_set_coarse_time(priv, control.value);
		if (err)
			dev_dbg(&client->dev,
				"%s: warning coarse time override failed\n",
				__func__);
	}

	err = imx318_write_table(priv, mode_table[IMX318_MODE_START_STREAM]);
	if (err)
		goto exit;

	usleep_range(10000, 10010); /* from tegrashell */
	return 0;
exit:
	dev_err(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx318_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx318 *priv = (struct imx318 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx318_subdev_video_ops = {
	.s_stream	= imx318_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = imx318_g_input_status,
};

static struct v4l2_subdev_core_ops imx318_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int imx318_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int imx318_set_fmt(struct v4l2_subdev *sd,
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

static struct v4l2_subdev_pad_ops imx318_subdev_pad_ops = {
	.set_fmt = imx318_set_fmt,
	.get_fmt = imx318_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops imx318_subdev_ops = {
	.core	= &imx318_subdev_core_ops,
	.video	= &imx318_subdev_video_ops,
	.pad = &imx318_subdev_pad_ops,
};

static const struct of_device_id imx318_of_match[] = {
	{ .compatible = "nvidia,imx318",},
	{ },
};

static struct camera_common_sensor_ops imx318_common_ops = {
	.power_on = imx318_power_on,
	.power_off = imx318_power_off,
	.write_reg = imx318_write_reg,
	.read_reg = imx318_read_reg,
};

static int imx318_set_group_hold(struct imx318 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	int err;
	int gh_en = switch_ctrl_qmenu[val];

	priv->group_hold_prev = val;
	if (gh_en == SWITCH_ON) {

		err = imx318_write_reg(priv->s_data,
					IMX318_GROUP_HOLD_ADDR, 0x1);
		if (err)
			goto fail;
	} else if (gh_en == SWITCH_OFF) {
		err = imx318_write_reg(priv->s_data,
					IMX318_GROUP_HOLD_ADDR, 0x0);
		if (err)
			goto fail;
	}
	return 0;
fail:
	dev_dbg(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int imx318_set_gain(struct imx318 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	imx318_reg reg_list[1];
	int err;
	u16 gain;
	int i;

	if (!priv->group_hold_prev)
		imx318_set_group_hold(priv, 1);

	if (val < IMX318_MIN_GAIN)
		val = IMX318_MIN_GAIN;
	else if (val > IMX318_MAX_GAIN)
		val = IMX318_MAX_GAIN;

	/* translate value */
	/*
	 * TODO: Current translation uses analog gain range equation
	 * against the gain DB values. Need to revise this and update
	 * for proper gain translation
	 */
	if (val == 0)
		gain = 0;
	else
		gain = (u16)(512 - (512/val));

	dev_dbg(dev, "%s: gain reg: %d, times: %lld\n", __func__, gain, val);

	imx318_get_gain_reg(reg_list, gain);

	for (i = 0; i < 2; i++) {
		err = imx318_write_reg(priv->s_data, reg_list[i].addr,
				reg_list[i].val);
		if (err) {
			dev_dbg(dev, "%s: GAIN control error\n", __func__);
			return err;
		}
	}

	return 0;
}

static int imx318_set_frame_length(struct imx318 *priv, s32 val)
{
	struct device *dev = &priv->i2c_client->dev;
	imx318_reg reg_list[2];
	int err;
	u32 frame_length;
	int i;

	if (!priv->group_hold_prev)
		imx318_set_group_hold(priv, 1);

	frame_length = (u32)val;

	imx318_get_frame_length_regs(reg_list, frame_length);
	dev_dbg(dev, "%s: val: %d\n", __func__, frame_length);

	for (i = 0; i < 2; i++) {
		err = imx318_write_reg(priv->s_data, reg_list[i].addr,
				reg_list[i].val);
		if (err) {
			dev_dbg(dev, "%s: FRAME_LENGTH control error\n",
				__func__);
			return err;
		}
	}

	priv->frame_length = frame_length;
	return 0;
}

static int imx318_set_coarse_time(struct imx318 *priv, s64 val)
{
	struct device *dev = &priv->i2c_client->dev;
	imx318_reg reg_list[3];
	int err;
	u32 coarse_time;
	int i;

	if (!priv->group_hold_prev)
		imx318_set_group_hold(priv, 1);

	coarse_time = (u32)val;

	imx318_get_coarse_time_regs(reg_list, coarse_time);
	dev_dbg(dev, "%s: val: %d\n", __func__, coarse_time);

	for (i = 0; i < 2; i++) {
		err = imx318_write_reg(priv->s_data, reg_list[i].addr,
				reg_list[i].val);
		if (err) {
			dev_dbg(dev, "%s: COARSE_TIME control error\n",
				__func__);
			return err;
		}
	}

	return 0;
}

static int imx318_eeprom_device_release(struct imx318 *priv)
{
	int i;

	for (i = 0; i < IMX318_EEPROM_NUM_BLOCKS; i++) {
		if (priv->eeprom[i].i2c_client != NULL) {
			i2c_unregister_device(priv->eeprom[i].i2c_client);
			priv->eeprom[i].i2c_client = NULL;
		}
	}

	return 0;
}

static int imx318_eeprom_device_init(struct imx318 *priv)
{
	char *dev_name = "eeprom_imx318";
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;

	if (!priv->pdata->has_eeprom)
		return -EINVAL;

	for (i = 0; i < IMX318_EEPROM_NUM_BLOCKS; i++) {
		priv->eeprom[i].adap = i2c_get_adapter(
				priv->i2c_client->adapter->nr);
		memset(&priv->eeprom[i].brd, 0, sizeof(priv->eeprom[i].brd));
		strncpy(priv->eeprom[i].brd.type, dev_name,
				sizeof(priv->eeprom[i].brd.type));
		priv->eeprom[i].brd.addr = IMX318_EEPROM_ADDRESS + i;
		priv->eeprom[i].i2c_client = i2c_new_device(
				priv->eeprom[i].adap, &priv->eeprom[i].brd);

		priv->eeprom[i].regmap = devm_regmap_init_i2c(
			priv->eeprom[i].i2c_client, &eeprom_regmap_config);
		if (IS_ERR(priv->eeprom[i].regmap)) {
			err = PTR_ERR(priv->eeprom[i].regmap);
			imx318_eeprom_device_release(priv);
			return err;
		}
	}

	return 0;
}

static int imx318_read_eeprom(struct imx318 *priv)
{
	int err, i;
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler,
			TEGRA_CAMERA_CID_EEPROM_DATA);
	if (!ctrl) {
		dev_err(&priv->i2c_client->dev,
			"could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < IMX318_EEPROM_NUM_BLOCKS; i++) {
		err = regmap_bulk_read(priv->eeprom[i].regmap, 0,
			&priv->eeprom_buf[i * IMX318_EEPROM_BLOCK_SIZE],
			IMX318_EEPROM_BLOCK_SIZE);
		if (err)
			return err;
	}

	for (i = 0; i < IMX318_EEPROM_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x",
			priv->eeprom_buf[i]);
	return 0;
}

/* TODO Validate id from sensor */
static int imx318_fuse_id_setup(struct imx318 *priv)
{
	int err;
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(priv->subdev);
	struct device *dev = &client->dev;
	struct camera_common_data *s_data = to_camera_common_data(dev);
	struct v4l2_ctrl *ctrl;
	u8 fuse_id[IMX318_FUSE_ID_SIZE];
	u8 bak = 0;

	for (i = 0; i < IMX318_FUSE_ID_SIZE; i++) {
		err |= imx318_read_reg(s_data,
			IMX318_FUSE_ID_START_ADDR + i, &bak);
		if (!err)
			fuse_id[i] = bak;
		else {
			dev_err(dev, "%s: can not read fuse id\n", __func__);
			return -EINVAL;
		}
	}

	ctrl = v4l2_ctrl_find(&priv->ctrl_handler, TEGRA_CAMERA_CID_FUSE_ID);
	if (!ctrl) {
		dev_err(dev, "could not find device ctrl.\n");
		return -EINVAL;
	}

	for (i = 0; i < IMX318_FUSE_ID_SIZE; i++)
		sprintf(&ctrl->p_new.p_char[i*2], "%02x",
			fuse_id[i]);
	ctrl->p_cur.p_char = ctrl->p_new.p_char;
	dev_info(&client->dev, "%s, fuse id: %s\n", __func__,
		ctrl->p_cur.p_char);

	return 0;
}

static int imx318_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx318 *priv =
		container_of(ctrl->handler, struct imx318, ctrl_handler);
	struct device *dev = &priv->i2c_client->dev;
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = imx318_set_gain(priv, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_FRAME_LENGTH:
		err = imx318_set_frame_length(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME:
		err = imx318_set_coarse_time(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		err = imx318_set_group_hold(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		dev_err(dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx318_ctrls_init(struct imx318 *priv, bool eeprom_ctrl)
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

	err = camera_common_s_power(priv->subdev, true);
	if (err) {
		dev_err(&client->dev,
			"Error %d during power on sensor\n", err);
		err = -ENODEV;
		goto error;
	}

	if (eeprom_ctrl) {
		err = imx318_read_eeprom(priv);
		if (err) {
			dev_err(&client->dev,
				"Error %d reading eeprom data\n", err);
			goto error_hw;
		}
	}

	err = imx318_fuse_id_setup(priv);
	if (err) {
		dev_err(&client->dev,
			"Error %d reading fuse id data\n", err);
		goto error_hw;
	}

	camera_common_s_power(priv->subdev, false);
	return 0;

error_hw:
	camera_common_s_power(priv->subdev, false);
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

MODULE_DEVICE_TABLE(of, imx318_of_match);

static struct camera_common_pdata *imx318_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;
	struct camera_common_pdata *ret = NULL;

	if (!np)
		return NULL;

	match = of_match_device(imx318_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk",
				&board_priv_pdata->mclk_name);
	if (err)
		dev_err(&client->dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (board_priv_pdata->reset_gpio < 0) {
		if (board_priv_pdata->reset_gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(&client->dev, "reset-gpios not found %d\n", err);
		goto error;
	}

	err = of_property_read_string(np, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(&client->dev, "avdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(&client->dev, "iovdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(np, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	if (err) {
		dev_err(&client->dev, "dvdd-reg not in DT\n");
		goto error;
	}
	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return ret;

}

static int imx318_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx318_subdev_internal_ops = {
	.open = imx318_open,
};

static const struct media_entity_operations imx318_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx318_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct imx318 *priv;
	char debugfs_name[10];
	int err;

	dev_info(&client->dev, "[IMX318]: probing v4l2 sensor at addr 0x%0x.\n",
		client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	common_data = devm_kzalloc(&client->dev,
				sizeof(struct camera_common_data), GFP_KERNEL);

	priv = devm_kzalloc(&client->dev,
			sizeof(struct imx318) + sizeof(struct v4l2_ctrl *) *
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

	priv->pdata = imx318_parse_dt(client);
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops = &imx318_common_ops;
	common_data->ctrl_handler = &priv->ctrl_handler;
	common_data->dev = &client->dev;
	common_data->frmfmt = &imx318_frmfmt[0];
	common_data->colorfmt = camera_common_find_datafmt(
							IMX318_DEFAULT_DATAFMT);
	common_data->power = &priv->power;
	common_data->ctrls = priv->ctrls;
	common_data->priv = (void *)priv;
	common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts = ARRAY_SIZE(imx318_frmfmt);
	common_data->def_mode = IMX318_DEFAULT_MODE;
	common_data->def_width = IMX318_DEFAULT_WIDTH;
	common_data->def_height = IMX318_DEFAULT_HEIGHT;
	common_data->fmt_width = common_data->def_width;
	common_data->fmt_height = common_data->def_height;
	common_data->def_clk_freq = IMX318_DEFAULT_CLK_FREQ;

	priv->i2c_client = client;
	priv->s_data = common_data;
	priv->subdev = &common_data->subdev;
	priv->subdev->dev = &client->dev;
	priv->s_data->dev = &client->dev;

	err = imx318_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(&client->dev, common_data);
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}
	sprintf(debugfs_name, "imx318_%c", common_data->csi_port + 'a');
	dev_dbg(&client->dev, "%s: name %s\n", __func__, debugfs_name);

	camera_common_create_debugfs(common_data, debugfs_name);

	v4l2_i2c_subdev_init(priv->subdev, client, &imx318_subdev_ops);

	/* eeprom interface */
	err = imx318_eeprom_device_init(priv);
	if (err && priv->pdata->has_eeprom)
		dev_err(&client->dev,
			"Failed to allocate eeprom reg map: %d\n", err);

	err = imx318_ctrls_init(priv, !err);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx318_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &imx318_media_ops;
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

	dev_info(&client->dev, "Detected IMX318 sensor\n");

	return 0;
}

static int imx318_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx318 *priv = (struct imx318 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx318_power_put(priv);
	camera_common_remove_debugfs(s_data);
	return 0;
}

static const struct i2c_device_id imx318_id[] = {
	{ "imx318", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx318_id);

static struct i2c_driver imx318_i2c_driver = {
	.driver = {
		.name = "imx318",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx318_of_match),
	},
	.probe = imx318_probe,
	.remove = imx318_remove,
	.id_table = imx318_id,
};

module_i2c_driver(imx318_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX318");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
