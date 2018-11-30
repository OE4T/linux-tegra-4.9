/*
 * imx318.c - imx318 sensor driver
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <media/tegracam_core.h>
#include <media/imx318.h>

#include "../platform/tegra/camera/camera_gpio.h"
#include "imx318_mode_tbls.h"

#define IMX318_MAX_COARSE_DIFF			10

/* IMX318 sensor register address */
#define IMX318_GAIN_ADDR_MSB				0x0204
#define IMX318_GAIN_ADDR_LSB				0x0205
#define IMX318_FRAME_LEGNTH_CTRL_EN			0x0350
#define IMX318_FRAME_LENGTH_ADDR_MSB			0x0340
#define IMX318_FRAME_LENGTH_ADDR_LSB			0x0341
#define IMX318_COARSE_INTEG_TIME_ADDR_MSB		0x0202
#define IMX318_COARSE_INTEG_TIME_ADDR_LSB		0x0203
#define IMX318_ST_COARSE_INTEG_TIME_ADDR_MSB		0x0224
#define IMX318_ST_COARSE_INTEG_TIME_ADDR_LSB		0x0225
#define IMX318_GROUP_HOLD_ADDR				0x0104

static const struct of_device_id imx318_of_match[] = {
	{ .compatible = "nvidia,imx318",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx318_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_EEPROM_DATA,
	TEGRA_CAMERA_CID_FUSE_ID,
};

struct imx318 {
	struct mutex imx318_camera_lock;
	struct camera_common_eeprom_data eeprom[IMX318_EEPROM_NUM_BLOCKS];
	u8				eeprom_buf[IMX318_EEPROM_SIZE];
	u8				fuse_id[IMX318_FUSE_ID_SIZE];
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	u32				frame_length;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
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
				s16 gain)
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
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx318_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx318_write_table(struct imx318 *priv,
				const imx318_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap,
						table,
						NULL, 0,
						IMX318_TABLE_WAIT_MS,
						IMX318_TABLE_END);
}

static int imx318_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	err = imx318_write_reg(s_data,
				IMX318_GROUP_HOLD_ADDR, val);
	if (err) {
		dev_dbg(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int imx318_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	imx318_reg reg_list[2];
	int err;
	s16 gain;
	int i;

	if (val < mode->control_properties.min_gain_val)
		val = mode->control_properties.min_gain_val;
	else if (val > mode->control_properties.max_gain_val)
		val = mode->control_properties.max_gain_val;

	/* translate value */
	gain = (s16)((512 * mode->control_properties.gain_factor) / val);
	gain = 512 - gain;

	if (gain < 0)
		gain = 0;

	dev_dbg(dev,
		"%s: gain reg: %d, times: %lld\n", __func__, gain, val);

	imx318_get_gain_reg(reg_list, gain);

	for (i = 0; i < 2; i++) {
		err = imx318_write_reg(s_data, reg_list[i].addr,
				reg_list[i].val);
		if (err) {
			dev_dbg(dev,
				"%s:%d GAIN control error\n", __func__, i);
			return err;
		}
	}

	return 0;
}

static int imx318_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx318 *priv = (struct imx318 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	imx318_reg reg_list[2];
	int err;
	u32 frame_length;
	int i;

	frame_length = (u32)(mode->signal_properties.pixel_clock.val *
		(u64)mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val);

	imx318_get_frame_length_regs(reg_list, frame_length);
	dev_dbg(dev,
		"%s: val:%d\n", __func__, frame_length);

	for (i = 0; i < 2; i++) {
		err = imx318_write_reg(s_data, reg_list[i].addr,
				reg_list[i].val);
		if (err)
			dev_dbg(dev,
				"%s: FRAME_LENGTH control error\n", __func__);
			return err;
	}

	priv->frame_length = frame_length;
	return 0;
}

static int imx318_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx318 *priv = (struct imx318 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	imx318_reg reg_list[3];
	int err = 0;
	u32 coarse_time;
	s32 max_coarse_time = priv->frame_length - IMX318_MAX_COARSE_DIFF;
	int i;

	coarse_time = mode->signal_properties.pixel_clock.val *
		val / mode->image_properties.line_length /
		mode->control_properties.exposure_factor;

	dev_dbg(dev, "%s: val: %d\n", __func__, coarse_time);
	if (coarse_time > max_coarse_time) {
		coarse_time = max_coarse_time;
		dev_dbg(dev, "%s:exposure limited by framelength val:%d\n",
				__func__, max_coarse_time);
	}

	imx318_get_coarse_time_regs(reg_list, coarse_time);

	for (i = 0; i < 2; i++) {
		err = imx318_write_reg(s_data, reg_list[i].addr,
				reg_list[i].val);
		if (err) {
			dev_dbg(dev,
				"%s: COARSE_TIME control error\n", __func__);
			return err;
		}
	}

	return err;
}

static int imx318_fill_string_ctrl(struct tegracam_device *tc_dev,
				struct v4l2_ctrl *ctrl)
{
	struct imx318 *priv = tc_dev->priv;
	int i;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_EEPROM_DATA:
		for (i = 0; i < IMX318_EEPROM_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->eeprom_buf[i]);
		break;
	case TEGRA_CAMERA_CID_FUSE_ID:
		for (i = 0; i < IMX318_FUSE_ID_SIZE; i++)
			sprintf(&ctrl->p_new.p_char[i*2], "%02x",
				priv->fuse_id[i]);
		break;
	default:
		return -EINVAL;
	}

	ctrl->p_cur.p_char = ctrl->p_new.p_char;

	return 0;
}

static struct tegracam_ctrl_ops imx318_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = { IMX318_EEPROM_STR_SIZE,
				IMX318_FUSE_ID_STR_SIZE},
	.set_gain = imx318_set_gain,
	.set_exposure = imx318_set_exposure,
	.set_frame_rate = imx318_set_frame_rate,
	.set_group_hold = imx318_set_group_hold,
	.fill_string_ctrl = imx318_fill_string_ctrl,
};

static int imx318_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
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
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
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
		/*
		 * Sleep for atleast 5ms after power off, discussions with
		 * sony revealed that this has to be investigated for the
		 * right delay and rootcause, for not add a WAR to pass testing
		 */
		usleep_range(5000, 6000);
	}

	pw->state = SWITCH_OFF;
	return 0;
}

static int imx318_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (pdata && pdata->use_cam_gpio)
		cam_gpio_deregister(s_data->dev, pw->pwdn_gpio);
	else {
		gpio_free(pw->pwdn_gpio);
		gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int imx318_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
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

static struct camera_common_pdata *imx318_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;
	struct camera_common_pdata *ret = NULL;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx318_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk",
				&board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found %d\n", err);
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	err = of_property_read_string(np, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(dev, "avdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(dev, "iovdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(np, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "dvdd-reg not in DT\n");
		goto error;
	}
	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;

}

static int imx318_set_mode(struct tegracam_device *tc_dev)
{
	struct imx318 *priv = (struct imx318 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int err;

	err = imx318_write_table(priv, mode_table[IMX318_MODE_COMMON]);
	if (err)
		return err;

	err = imx318_write_table(priv, mode_table[s_data->mode]);
	if (err)
		return err;

	return 0;
}

static int imx318_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx318 *priv = (struct imx318 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx318_write_table(priv, mode_table[IMX318_MODE_START_STREAM]);
	if (err)
		return err;

	usleep_range(10000, 10010); /* from tegrashell */

	return 0;
}

static int imx318_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx318 *priv = (struct imx318 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx318_write_table(priv, mode_table[IMX318_MODE_STOP_STREAM]);
	if (err)
		return err;
	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * delay = frame length rows * Tline (10 us)
	 */
	usleep_range(priv->frame_length * 10, priv->frame_length * 10 + 1000);

	return 0;
}

static struct camera_common_sensor_ops imx318_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx318_frmfmt),
	.frmfmt_table = imx318_frmfmt,
	.power_on = imx318_power_on,
	.power_off = imx318_power_off,
	.write_reg = imx318_write_reg,
	.read_reg = imx318_read_reg,
	.parse_dt = imx318_parse_dt,
	.power_get = imx318_power_get,
	.power_put = imx318_power_put,
	.set_mode = imx318_set_mode,
	.start_streaming = imx318_start_streaming,
	.stop_streaming = imx318_stop_streaming,
};

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
	struct camera_common_pdata *pdata = priv->s_data->pdata;
	static struct regmap_config eeprom_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	int i;
	int err;

	if (!pdata || !pdata->has_eeprom)
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

	for (i = 0; i < IMX318_EEPROM_NUM_BLOCKS; i++) {
		err = regmap_bulk_read(priv->eeprom[i].regmap, 0,
			&priv->eeprom_buf[i * IMX318_EEPROM_BLOCK_SIZE],
			IMX318_EEPROM_BLOCK_SIZE);
		if (err)
			return err;
	}

	return 0;
}

static int imx318_read_fuse_id(struct imx318 *priv)
{
	/* fuse id is stored in eeprom */
	memcpy(priv->fuse_id,
			&priv->eeprom_buf[IMX318_FUSE_ID_START_ADDR],
			IMX318_FUSE_ID_SIZE);

	return 0;
}

static int imx318_board_setup(struct imx318 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	bool eeprom_ctrl = 0;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	/* eeprom interface */
	err = imx318_eeprom_device_init(priv);
	if (err && s_data->pdata->has_eeprom)
		dev_err(dev,
			"Failed to allocate eeprom reg map: %d\n", err);
	eeprom_ctrl = !err;

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx318_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}

	if (eeprom_ctrl) {
		err = imx318_read_eeprom(priv);
		if (err) {
			dev_err(dev,
				"Error %d reading eeprom data\n", err);
			goto error;
		}
		err = imx318_read_fuse_id(priv);
		if (err) {
			dev_err(dev,
				"Error %d reading fuse id data\n", err);
			goto error;
		}
	}

error:
	imx318_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
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

static int imx318_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx318 *priv;
	int err;

	dev_info(dev, "[IMX318]: probing v4l2 sensor at addr 0x%0x.\n",
		client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx318), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx318", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx318_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx318_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx318_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx318_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "Detected IMX318 sensor\n");

	return 0;
}

static int imx318_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx318 *priv = (struct imx318 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

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
