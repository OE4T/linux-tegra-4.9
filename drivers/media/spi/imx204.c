/*
 * imx204.c - imx204 sensor driver
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/regmap.h>
#include <media/camera_common.h>
#include <media/imx204.h>
#include "imx204_mode_tbls.h"

#include "../platform/tegra/camera/camera_gpio.h"

#define IMX204_MAX_COARSE_DIFF		10

#define IMX204_GAIN_SHIFT		8
#define IMX204_MIN_GAIN		(1 << IMX204_GAIN_SHIFT)
#define IMX204_MAX_GAIN		(27 << IMX204_GAIN_SHIFT)
#define IMX204_MIN_FRAME_LENGTH	(0x0E51)
#define IMX204_MAX_FRAME_LENGTH	(0xffff)
#define IMX204_MIN_EXPOSURE_COARSE	(0x0001)
#define IMX204_MAX_EXPOSURE_COARSE	\
	(IMX204_MAX_FRAME_LENGTH-IMX204_MAX_COARSE_DIFF)

#define IMX204_DEFAULT_GAIN		IMX204_MIN_GAIN
#define IMX204_DEFAULT_FRAME_LENGTH	(0x0E51)
#define IMX204_DEFAULT_EXPOSURE_COARSE	\
	(IMX204_DEFAULT_FRAME_LENGTH-IMX204_MAX_COARSE_DIFF)
#define	IMX204_DEFAULT_MODE	IMX204_MODE_5208x3924
#ifdef T19x_uFPGA
#define IMX204_DEFAULT_WIDTH	1784
#define IMX204_DEFAULT_HEIGHT	1318
#else
#define IMX204_DEFAULT_WIDTH	5184
#define IMX204_DEFAULT_HEIGHT	3924
#endif
#define IMX204_DEFAULT_DATAFMT	MEDIA_BUS_FMT_SRGGB12_1X12
#define IMX204_DEFAULT_CLK_FREQ	72000000

#define IMX204_CHIP_ID 0x81
#define VGP1_OFFSET	0x5068
#define VGP5_OFFSET	0x5078

struct imx204 {
	struct camera_common_power_rail	power;
	struct v4l2_ctrl_handler	ctrl_handler;
	int				num_ctrls;
	struct spi_device		*spi;
	struct device			*dev;
	struct v4l2_subdev		*subdev;
	struct media_pad		pad;
#ifdef T19x_uFPGA
	void __iomem *vi_base;
#endif

	s32				group_hold_prev;
	bool				group_hold_en;
	struct regmap			*regmap;
	struct camera_common_data	*s_data;
	struct camera_common_pdata	*pdata;
	struct v4l2_ctrl		*ctrls[];
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 24,
	.val_bits = 8,
};

static int imx204_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int imx204_s_ctrl(struct v4l2_ctrl *ctrl);

static const struct v4l2_ctrl_ops imx204_ctrl_ops = {
	.g_volatile_ctrl = imx204_g_volatile_ctrl,
	.s_ctrl		= imx204_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &imx204_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX204_MIN_GAIN,
		.max = IMX204_MAX_GAIN,
		.def = IMX204_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &imx204_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX204_MIN_FRAME_LENGTH,
		.max = IMX204_MAX_FRAME_LENGTH,
		.def = IMX204_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &imx204_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX204_MIN_EXPOSURE_COARSE,
		.max = IMX204_MAX_EXPOSURE_COARSE,
		.def = IMX204_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx204_ctrl_ops,
		.id = TEGRA_CAMERA_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = IMX204_MIN_EXPOSURE_COARSE,
		.max = IMX204_MAX_EXPOSURE_COARSE,
		.def = IMX204_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &imx204_ctrl_ops,
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
		.ops = &imx204_ctrl_ops,
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

static inline void imx204_get_frame_length_regs(imx204_reg *regs,
				u32 frame_length)
{
	// TODO
}

static inline void imx204_get_coarse_time_regs(imx204_reg *regs,
				u32 coarse_time)
{
	// TODO
}

static inline void imx204_get_coarse_time_short_regs(imx204_reg *regs,
				u32 coarse_time)
{
	// TODO
}

static inline void imx204_get_gain_reg(imx204_reg *regs,
				u16 gain)
{
	// TODO
}

static inline void imx204_get_gain_short_reg(imx204_reg *regs,
				u16 gain)
{
	// TODO
}

static int test_mode;
module_param(test_mode, int, 0644);

#ifndef T19x_uFPGA
static void imx204_gpio_set(struct imx204 *priv,
			    unsigned int gpio, int val)
{
	if (priv->pdata->use_cam_gpio)
		cam_gpio_ctrl(priv->dev, gpio, val, 1);
	else {
		if (gpio_cansleep(gpio))
			gpio_set_value_cansleep(gpio, val);
		else
			gpio_set_value(gpio, val);
	}
}
#endif

static inline int imx204_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	struct imx204 *priv = (struct imx204 *)s_data->priv;
	u32 addr32 = ((0x81<<16)|(addr));

	return regmap_read(priv->regmap, addr32, (unsigned int *) val);
}

static int imx204_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err;
	struct imx204 *priv = (struct imx204 *)s_data->priv;
#ifdef USE_REGMAP
	u32 addr32 = ((0x81<<16)|(addr));

	err = regmap_write(priv->regmap, addr32, val);
	if (err)
		dev_err(priv->dev, "%s:spi write failed, %x = %x\n",
			__func__, addr, val);
#else
	u8 data[4];

	data[0] = 0x81;
	data[1] = (addr >> 8) & 0xff;
	data[2] = addr & 0xff;
	data[3] = val;
	err = spi_write(priv->spi, &data[0], 4);
	if (err < 0)
		dev_err(priv->dev, "spi_write failed %d\n", err);
#endif

	return err;
}

static int imx204_write_reg32(struct camera_common_data *s_data,
				u32 addr, u8 val)
{
	int err = 0;
	struct imx204 *priv = (struct imx204 *)s_data->priv;
#ifdef USE_REGMAP
	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(priv->dev, "%s:spi write failed, %x = %x\n",
			__func__, addr, val);

#else
	unsigned char buffer[4];
	buffer[0] = (addr>>16)&0xFF;
	buffer[1] = (addr>>8)&0xFF;
	buffer[2] = (addr)&0xFF;
	buffer[3] = (val)&0xFF;
	spi_write(priv->spi, buffer, 4);
#endif
	return err;
}

static int imx204_write_table(struct imx204 *priv,
			      const imx204_reg table[])
{
#ifdef USE_REGMAP
	return regmap_util_write_table_8(priv->regmap,
					 table,
					 NULL, 0,
					 IMX204_TABLE_WAIT_MS,
					 IMX204_TABLE_END);
#else
	int cnt = 0;
	int status = 0;

	while (table[cnt].addr != IMX204_TABLE_END) {
		if (table[cnt].addr == IMX204_TABLE_WAIT_MS) {
			msleep_range(table[cnt].val);
#ifdef T19x_uFPGA
		} else if (table[cnt].addr == IMX204_TABLE_ENABLE_GTX) {
			/* toggle VGP5 to enable GTX on uFPGA */
			writel(0x10001, priv->vi_base + VGP5_OFFSET);
			writel(0x0, priv->vi_base + VGP5_OFFSET);
#endif
		} else {
			status = imx204_write_reg32(priv->s_data,
					table[cnt].addr, table[cnt].val);
			if (status < 0) {
				dev_err(priv->dev, "REG0x%X=0x%X ERROR = %d\n",
						table[cnt].addr,
						table[cnt].val,
						status);
				return status;
			}
		}
		cnt++;
	}
	return status;
#endif
}

static int imx204_power_on(struct camera_common_data *s_data)
{
	struct imx204 *priv = (struct imx204 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

#ifdef T19x_uFPGA
	/* VGP1 is used as reset GPIO on uFPGA */
	writel(0x10001, priv->vi_base + VGP1_OFFSET);
#else
	// TODO
#endif
	pw->state = SWITCH_ON;
	return 0;
}

static int imx204_power_off(struct camera_common_data *s_data)
{
	struct imx204 *priv = (struct imx204 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

#ifdef T19x_uFPGA
	/* VGP1 is used as reset GPIO on uFPGA */
	writel(0, priv->vi_base + VGP1_OFFSET);
#else
	// TODO
#endif
	pw->state = SWITCH_OFF;
	return 0;
}

static int imx204_power_put(struct imx204 *priv)
{
	// TODO

	return 0;
}

static int imx204_power_get(struct imx204 *priv)
{
	struct camera_common_power_rail *pw = &priv->power;

	// TODO

	pw->state = SWITCH_OFF;
	return 0;
}

static int imx204_get_framesync(struct camera_common_data *s_data,
		struct camera_common_framesync *vshs)
{
	vshs->inck = 72000;
	vshs->xhs = 300;
	vshs->xvs = 4004;
	vshs->fps = 59940;

	return 0;
}


static int imx204_set_gain(struct imx204 *priv, s32 val);
static int imx204_set_frame_length(struct imx204 *priv, s32 val);
static int imx204_set_coarse_time(struct imx204 *priv, s32 val);
static int imx204_set_coarse_time_short(struct imx204 *priv, s32 val);

static int imx204_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	struct imx204 *priv = (struct imx204 *)s_data->priv;
	struct v4l2_control control;
	int err;
	int sensor_mode = s_data->mode;

	dev_info(priv->dev, "%s++ enable = %d\n", __func__, enable);

#ifdef T19x_uFPGA
	sensor_mode = IMX204_MODE_1784x1318_FPGA;
#endif

	if (!enable) {
		return imx204_write_table(priv,
			mode_table[IMX204_MODE_STOP_STREAM]);
	}

	err = imx204_write_table(priv, mode_table[sensor_mode]);
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
		err |= imx204_set_gain(priv, control.value);
		if (err)
			dev_dbg(priv->dev, "%s: warning gain override failed\n",
				__func__);

		control.id = TEGRA_CAMERA_CID_FRAME_LENGTH;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx204_set_frame_length(priv, control.value);
		if (err)
			dev_dbg(priv->dev,
				"%s: warning frame length override failed\n",
				__func__);

		control.id = TEGRA_CAMERA_CID_COARSE_TIME;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx204_set_coarse_time(priv, control.value);
		if (err)
			dev_dbg(priv->dev,
				"%s: warning coarse time override failed\n",
				__func__);

		control.id = TEGRA_CAMERA_CID_COARSE_TIME_SHORT;
		err = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err |= imx204_set_coarse_time_short(priv, control.value);
		if (err)
			dev_dbg(sd->dev,
				"%s: warning coarse time short override failed\n",
				__func__);
	}

#ifndef T19x_uFPGA
	err = imx204_write_table(priv, mode_table[IMX204_MODE_START_STREAM]);
	if (err)
		goto exit;
#endif

	dev_info(priv->dev, "%s--\n", __func__);
	return 0;
exit:
	dev_info(priv->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx204_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct camera_common_data *s_data = to_camera_common_data(sd->dev);
	struct imx204 *priv = (struct imx204 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops imx204_subdev_video_ops = {
	.s_stream	= imx204_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = imx204_g_input_status,
};

static struct v4l2_subdev_core_ops imx204_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int imx204_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}
static int imx204_set_fmt(struct v4l2_subdev *sd,
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

static struct v4l2_subdev_pad_ops imx204_subdev_pad_ops = {
	.set_fmt = imx204_set_fmt,
	.get_fmt = imx204_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size	= camera_common_enum_framesizes,
	.enum_frame_interval	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops imx204_subdev_ops = {
	.core	= &imx204_subdev_core_ops,
	.video	= &imx204_subdev_video_ops,
	.pad	= &imx204_subdev_pad_ops,
};
static struct of_device_id imx204_of_match[] = {
	{ .compatible = "nvidia,imx204-spi", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx204_of_match);

static struct camera_common_sensor_ops imx204_common_ops = {
	.power_on = imx204_power_on,
	.power_off = imx204_power_off,
	.write_reg = imx204_write_reg,
	.read_reg = imx204_read_reg,
	.get_framesync = imx204_get_framesync,
};

static int imx204_set_group_hold(struct imx204 *priv, s32 val)
{
	// TODO
	return 0;
}

static int imx204_set_gain(struct imx204 *priv, s32 val)
{
	// TODO
	return 0;
}

static int imx204_set_frame_length(struct imx204 *priv, s32 val)
{
	// TODO
	return 0;
}

static int imx204_set_coarse_time(struct imx204 *priv, s32 val)
{
	// TODO
	return 0;
}

static int imx204_set_coarse_time_short(struct imx204 *priv, s32 val)
{
	// TODO
	return 0;
}

static int imx204_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx204 *priv =
		container_of(ctrl->handler, struct imx204, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	default:
			dev_err(priv->dev, "%s: unknown ctrl id.\n", __func__);
			return -EINVAL;
	}

	return err;
}

static int imx204_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx204 *priv =
		container_of(ctrl->handler, struct imx204, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = imx204_set_gain(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_FRAME_LENGTH:
		err = imx204_set_frame_length(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME:
		err = imx204_set_coarse_time(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_COARSE_TIME_SHORT:
		err = imx204_set_coarse_time_short(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		err = imx204_set_group_hold(priv, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		dev_err(priv->dev, "%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}
static int imx204_ctrls_init(struct imx204 *priv)
{
	struct v4l2_ctrl *ctrl;
	int num_ctrls;
	int err;
	int i;

	dev_dbg(priv->dev, "%s++\n", __func__);

	num_ctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

	for (i = 0; i < num_ctrls; i++) {
		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
			&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(priv->dev, "Failed to init %s ctrl\n",
				ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(priv->dev,
				ctrl_config_list[i].max + 1, GFP_KERNEL);
		}
		priv->ctrls[i] = ctrl;
	}

	priv->num_ctrls = num_ctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(priv->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(priv->dev,
			"Error %d setting default controls\n", err);
		goto error;
	}

	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}

static struct camera_common_pdata *imx204_parse_dt(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;

	match = of_match_device(imx204_of_match, &spi->dev);
	if (!match) {
		dev_err(&spi->dev, "Failed to find matching dt id\n");
		return ERR_PTR(-ENODEV);
	}

	board_priv_pdata = devm_kzalloc(&spi->dev,
			   sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata) {
		dev_err(&spi->dev, "Failed to allocate pdata\n");
		return ERR_PTR(-ENODEV);
	}

	of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);

	of_property_read_string(np, "avdd-reg",
			&board_priv_pdata->regulators.avdd);
	of_property_read_string(np, "dvdd-reg",
			&board_priv_pdata->regulators.dvdd);
	of_property_read_string(np, "iovdd-reg",
			&board_priv_pdata->regulators.iovdd);

	return board_priv_pdata;
}

static int imx204_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	dev_dbg(sd->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx204_subdev_internal_ops = {
	.open = imx204_open,
};

static const struct media_entity_operations imx204_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx204_probe(struct spi_device *spi)
{
	struct camera_common_data *common_data;
	struct imx204 *priv;
	struct device_node *node = spi->dev.of_node;
	char debugfs_name[10];
	int err;
	int ret = 0;

	pr_info("[IMX204]: probing v4l2 sensor.\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	common_data = devm_kzalloc(&spi->dev,
			    sizeof(struct camera_common_data), GFP_KERNEL);
	if (!common_data) {
		dev_err(&spi->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	priv = devm_kzalloc(&spi->dev,
			    sizeof(struct imx204) + sizeof(struct v4l2_ctrl *) *
			    ARRAY_SIZE(ctrl_config_list),
			    GFP_KERNEL);
	if (!priv) {
		dev_err(&spi->dev, "unable to allocate memory!\n");
		return -ENOMEM;
	}

	spi->mode = SPI_MODE_3 | SPI_LSB_FIRST;
	spi->bits_per_word = 8;
	spi->irq = -1;
	err = spi_setup(spi);
	if (err) {
		dev_err(&spi->dev, "unable to setup SPI!\n");
		return err;
	}

	priv->regmap = devm_regmap_init_spi(spi, &sensor_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&spi->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pdata = imx204_parse_dt(spi);
	if (!priv->pdata) {
		dev_err(&spi->dev, "unable to get platform data\n");
		return -EFAULT;
	}

	common_data->ops		= &imx204_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->dev		= &spi->dev;
#ifdef T19x_uFPGA
	common_data->frmfmt		= &imx204_frmfmt[3];
	common_data->colorfmt		= camera_common_find_datafmt(
					  IMX204_DEFAULT_DATAFMT);
#else
	common_data->frmfmt		= &imx204_frmfmt[0];
	common_data->colorfmt		= camera_common_find_datafmt(
					  IMX204_DEFAULT_DATAFMT);
#endif
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(imx204_frmfmt);
	common_data->def_mode		= IMX204_DEFAULT_MODE;
	common_data->def_width		= IMX204_DEFAULT_WIDTH;
	common_data->def_height		= IMX204_DEFAULT_HEIGHT;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;
	common_data->def_clk_freq	= IMX204_DEFAULT_CLK_FREQ;

	priv->dev				= &spi->dev;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &spi->dev;
	priv->s_data->dev		= &spi->dev;
	priv->spi				= spi;

	err = imx204_power_get(priv);
	if (err)
		return err;

	err = camera_common_parse_ports(&spi->dev, common_data);
	if (err) {
		dev_err(&spi->dev, "Failed to find port info\n");
		return err;
	}

	sprintf(debugfs_name, "imx204_%c", common_data->csi_port + 'a');
	dev_dbg(&spi->dev, "%s: name %s\n", __func__, debugfs_name);
	camera_common_create_debugfs(common_data, "imx204");

	v4l2_spi_subdev_init(&common_data->subdev, spi, &imx204_subdev_ops);

	err = imx204_ctrls_init(priv);
	if (err)
		return err;

	priv->subdev->internal_ops = &imx204_subdev_internal_ops;
	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &imx204_media_ops;
	err = tegra_media_entity_init(&priv->subdev->entity, 1,
				&priv->pad, true, true);
	if (err < 0) {
		dev_err(&spi->dev, "unable to init media entity\n");
		return err;
	}
#endif
	err = v4l2_async_register_subdev(priv->subdev);
	if (err)
		return err;

#ifdef T19x_uFPGA
	/* VGP1 is used as reset GPIO on uFPGA */
	priv->vi_base = ioremap(0x15f00000, 0x10000);
	if (!priv->vi_base)
		dev_err(priv->dev, "can't get vi base address\n");
#endif

	return ret;
}

static int
imx204_remove(struct spi_device *spi)
{
	struct camera_common_data *s_data = to_camera_common_data(&spi->dev);
	struct imx204 *priv = (struct imx204 *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	imx204_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct spi_device_id imx204_id[] = {
	{ "imx204", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, imx204_id);

static struct spi_driver imx204_spi_driver = {
	.driver = {
		.name = "imx204_spi",
		.owner = THIS_MODULE,
		.of_match_table = imx204_of_match,
	},
	.probe = imx204_probe,
	.remove = imx204_remove,
	.id_table = imx204_id,
};

module_spi_driver(imx204_spi_driver);

MODULE_DESCRIPTION("SPI driver for Sony IMX204");
MODULE_AUTHOR("Frank Chen<frankc@nvidia.com>");
MODULE_LICENSE("GPL v2");
