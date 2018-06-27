/*
 * max9295.c - max9295 IO Expander driver
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

#include <media/camera_common.h>
#include <linux/module.h>
#include <media/max9295.h>

/* register specifics */
#define MAX9295_MIPI_RX0_ADDR 0x330
#define MAX9295_MIPI_RX1_ADDR 0x331
#define MAX9295_MIPI_RX2_ADDR 0x332
#define MAX9295_MIPI_RX3_ADDR 0x333

#define MAX9295_PIPE_X_DT_ADDR 0x314
#define MAX9295_PIPE_Y_DT_ADDR 0x316
#define MAX9295_PIPE_Z_DT_ADDR 0x318
#define MAX9295_PIPE_U_DT_ADDR 0x31A

#define MAX9295_CTRL0_ADDR 0x10
#define MAX9295_SRC_CTRL_ADDR 0x2B0
#define MAX9295_SRC_PWDN_ADDR 0x02BE
#define MAX9295_SRC_OUT_RCLK_ADDR 0x3F1
#define MAX9295_START_PIPE_ADDR 0x311
#define MAX9295_PIPE_EN_ADDR 0x2
#define MAX9295_CSI_PORT_SEL_ADDR 0x308

#define MAX9295_STREAM_PIPE_UNUSED 0x22
#define MAX9295_CSI_MODE_1X4 0x00
#define MAX9295_CSI_MODE_2X2 0x03
#define MAX9295_CSI_MODE_2X4 0x06

#define MAX9295_CSI_PORT_B(num_lanes) (((num_lanes) << 4) & 0xF0)
#define MAX9295_CSI_PORT_A(num_lanes) ((num_lanes) & 0x0F)

#define MAX9295_CSI_1X4_MODE_LANE_MAP1 0xE0
#define MAX9295_CSI_1X4_MODE_LANE_MAP2 0x04

#define MAX9295_CSI_2X4_MODE_LANE_MAP1 0xEE
#define MAX9295_CSI_2X4_MODE_LANE_MAP2 0xE4

#define MAX9295_CSI_2X2_MODE_LANE_MAP1 MAX9295_CSI_2X4_MODE_LANE_MAP1
#define MAX9295_CSI_2X2_MODE_LANE_MAP2 MAX9295_CSI_2X4_MODE_LANE_MAP2

#define MAX9295_PIPE_X_ST_ID 0x0
#define MAX9295_PIPE_Y_ST_ID 0x1
#define MAX9295_PIPE_Z_ST_ID 0x2
#define MAX9295_PIPE_U_ST_ID 0x3

#define MAX9295_PIPE_X_DT_ROUTE_EN 0xC0
#define MAX9295_PIPE_Y_DT_ROUTE_EN 0x40
#define MAX9295_PIPE_Z_DT_ROUTE_EN 0x40
#define MAX9295_PIPE_U_DT_ROUTE_EN 0x40

#define MAX9295_PIPE_X_START_B 0x80
#define MAX9295_PIPE_Y_START_B 0x40
#define MAX9295_PIPE_Z_START_B 0x20
#define MAX9295_PIPE_U_START_B 0x10

#define MAX9295_PIPE_X_START_A 0x1
#define MAX9295_PIPE_Y_START_A 0x2
#define MAX9295_PIPE_Z_START_A 0x4
#define MAX9295_PIPE_U_START_A 0x8

#define MAX9295_CLK_SELX 0x1
#define MAX9295_CLK_SELY 0x2
#define MAX9295_CLK_SELZ 0x4
#define MAX9295_CLK_SELU 0x8

#define MAX9295_START_PORT_A 0x10
#define MAX9295_START_PORT_B 0x20

#define MAX9295_CSI_LN2 0x1
#define MAX9295_CSI_LN4 0x3

#define MAX9295_EN_LINE_INFO 0x40

#define MAX9295_VID_TX_EN_X 0x10
#define MAX9295_VID_TX_EN_Y 0x20
#define MAX9295_VID_TX_EN_Z 0x40
#define MAX9295_VID_TX_EN_U 0x80

#define MAX9295_VID_INIT 0x3

#define MAX9295_SRC_RCLK 0x89

#define MAX9295_RESET_ALL 0x80
#define MAX9295_RESET_LINK 0x40
#define MAX9295_RESET_ONESHOT 0x20
#define MAX9295_REG_EN 0x4
#define MAX9295_AUTO_LINK 0x1
#define MAX9295_RESET_SRC 0x12
#define MAX9295_PWDN_GPIO 0x90

struct max9295 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct gmsl_link_data *gmsl_data;
	struct mutex lock;
};

static int max9295_write_reg(struct device *dev, u16 addr, u8 val)
{
	struct max9295 *priv = dev_get_drvdata(dev);
	int err;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev, "%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int max9295_read_reg(struct max9295 *priv,
	u16 addr, u8* val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	if (err)
		dev_err(&i2c_client->dev, "%s:i2c read failed, 0x%x = %x\n",
			__func__, addr, *val);

	*val = reg_val & 0xFF;

	return err;
}

int max9295_stream_setup(struct device *dev)
{
	struct max9295 *priv = dev_get_drvdata(dev);
	int err = 0;
	u32 csi_mode;
	u32 lane_map1;
	u32 lane_map2;
	u32 port;
	u32 rx1_lanes;

	mutex_lock(&priv->lock);

	switch (priv->gmsl_data->csi_mode) {
	case GMSL_CSI_1X4_MODE:
		csi_mode = MAX9295_CSI_MODE_1X4;
		lane_map1 = MAX9295_CSI_1X4_MODE_LANE_MAP1;
		lane_map2 = MAX9295_CSI_1X4_MODE_LANE_MAP2;
		rx1_lanes = MAX9295_CSI_LN4;
		break;
	case GMSL_CSI_2X2_MODE:
		csi_mode = MAX9295_CSI_MODE_2X2;
		lane_map1 = MAX9295_CSI_2X2_MODE_LANE_MAP1;
		lane_map2 = MAX9295_CSI_2X2_MODE_LANE_MAP2;
		rx1_lanes = MAX9295_CSI_LN2;
		break;
	case GMSL_CSI_2X4_MODE:
		csi_mode = MAX9295_CSI_MODE_2X4;
		lane_map1 = MAX9295_CSI_2X4_MODE_LANE_MAP1;
		lane_map2 = MAX9295_CSI_2X4_MODE_LANE_MAP2;
		rx1_lanes = MAX9295_CSI_LN4;
		break;
	default:
		dev_err(dev, "%s: invalid csi mode\n", __func__);
		err = -EINVAL;
		goto error;
	}

	port = (priv->gmsl_data->src_csi_port == GMSL_CSI_PORT_B) ?
			MAX9295_CSI_PORT_B(rx1_lanes) :
			MAX9295_CSI_PORT_A(rx1_lanes);

	max9295_write_reg(dev, MAX9295_MIPI_RX0_ADDR, csi_mode);
	max9295_write_reg(dev, MAX9295_MIPI_RX1_ADDR, port);
	max9295_write_reg(dev, MAX9295_MIPI_RX2_ADDR, lane_map1);
	max9295_write_reg(dev, MAX9295_MIPI_RX3_ADDR, lane_map2);

	switch (priv->gmsl_data->num_streams) {
	case 2:
		max9295_write_reg(dev, MAX9295_PIPE_X_DT_ADDR,
			(MAX9295_PIPE_X_DT_ROUTE_EN |
				priv->gmsl_data->streams[0].st_data_type));
		priv->gmsl_data->streams[0].st_id_sel = MAX9295_PIPE_X_ST_ID;

		max9295_write_reg(dev, MAX9295_PIPE_Z_DT_ADDR,
			(MAX9295_PIPE_Z_DT_ROUTE_EN |
				priv->gmsl_data->streams[1].st_data_type));
		priv->gmsl_data->streams[1].st_id_sel = MAX9295_PIPE_Z_ST_ID;

		max9295_write_reg(dev,
			MAX9295_PIPE_Y_DT_ADDR, MAX9295_STREAM_PIPE_UNUSED);
		max9295_write_reg(dev,
			MAX9295_PIPE_U_DT_ADDR, MAX9295_STREAM_PIPE_UNUSED);
		priv->gmsl_data->streams[2].st_id_sel = GMSL_ST_ID_UNUSED;
		priv->gmsl_data->streams[3].st_id_sel = GMSL_ST_ID_UNUSED;
		break;
	case 4:
		max9295_write_reg(dev, MAX9295_PIPE_X_DT_ADDR,
			(MAX9295_PIPE_X_DT_ROUTE_EN |
				priv->gmsl_data->streams[0].st_data_type));
		priv->gmsl_data->streams[0].st_id_sel = MAX9295_PIPE_X_ST_ID;

		max9295_write_reg(dev, MAX9295_PIPE_Y_DT_ADDR,
			(MAX9295_PIPE_Y_DT_ROUTE_EN |
				priv->gmsl_data->streams[1].st_data_type));
		priv->gmsl_data->streams[1].st_id_sel = MAX9295_PIPE_Y_ST_ID;

		max9295_write_reg(dev, MAX9295_PIPE_Z_DT_ADDR,
			(MAX9295_PIPE_Z_DT_ROUTE_EN |
				priv->gmsl_data->streams[2].st_data_type));
		priv->gmsl_data->streams[2].st_id_sel = MAX9295_PIPE_Z_ST_ID;

		max9295_write_reg(dev, MAX9295_PIPE_U_DT_ADDR,
			(MAX9295_PIPE_U_DT_ROUTE_EN |
				priv->gmsl_data->streams[3].st_data_type));
		priv->gmsl_data->streams[3].st_id_sel = MAX9295_PIPE_U_ST_ID;
		break;
	default:
		dev_err(dev, "%s: stream count not supported\n", __func__);
		err = -EINVAL;
		goto error;
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9295_stream_setup);

int max9295_poweron(struct device *dev)
{
	max9295_write_reg(dev, MAX9295_CTRL0_ADDR,
		(MAX9295_RESET_ONESHOT |
			MAX9295_REG_EN | MAX9295_AUTO_LINK));
	msleep(20);
	max9295_write_reg(dev, MAX9295_SRC_PWDN_ADDR, MAX9295_PWDN_GPIO);
	max9295_write_reg(dev, MAX9295_SRC_OUT_RCLK_ADDR, MAX9295_SRC_RCLK);
	max9295_write_reg(dev, MAX9295_SRC_CTRL_ADDR, MAX9295_RESET_SRC);

	return 0;
}
EXPORT_SYMBOL(max9295_poweron);

int max9295_poweroff(struct device *dev)
{
	max9295_write_reg(dev, MAX9295_CTRL0_ADDR, MAX9295_RESET_ALL);

	return 0;
}
EXPORT_SYMBOL(max9295_poweroff);

int max9295_dev_pair(struct device *dev, struct gmsl_link_data *pdata)
{
	struct max9295 *priv;
	int err = 0;

	if (!dev || !pdata || !pdata->s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);
	mutex_lock(&priv->lock);
	if (priv->gmsl_data) {
		dev_err(dev, "%s: device already paired\n", __func__);
		err = -EINVAL;
		goto error;
	}
	priv->gmsl_data = pdata;

error:
	mutex_unlock(&priv->lock);
	return 0;
}
EXPORT_SYMBOL(max9295_dev_pair);

int max9295_dev_unpair(struct device *dev, struct device *s_dev)
{
	struct max9295 *priv = NULL;
	int err = 0;

	if (!dev || !s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);

	mutex_lock(&priv->lock);

	if (!priv->gmsl_data) {
		dev_err(dev, "%s: device is not paired\n", __func__);
		err = -ENOMEM;
		goto error;
	}

	if (priv->gmsl_data->s_dev != s_dev) {
		dev_err(dev, "%s: invalid device\n", __func__);
		err = -EINVAL;
		goto error;
	}

	priv->gmsl_data = NULL;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9295_dev_unpair);

int max9295_streamon(struct device *dev)
{
	struct max9295 *priv = dev_get_drvdata(dev);
	u32 st_pipe = 0;
	u32 port_sel = 0;
	u32 pipe_en = 0;
	int i = 0;

	mutex_lock(&priv->lock);
	for (i = 0; i < priv->gmsl_data->num_streams; i++) {
		if (priv->gmsl_data->streams[i].st_id_sel ==
				MAX9295_PIPE_X_ST_ID)
			port_sel = MAX9295_CLK_SELX;
		else if (priv->gmsl_data->streams[i].st_id_sel ==
				MAX9295_PIPE_Y_ST_ID)
			port_sel |= MAX9295_CLK_SELY;
		else if (priv->gmsl_data->streams[i].st_id_sel ==
				MAX9295_PIPE_Z_ST_ID)
			port_sel |= MAX9295_CLK_SELZ;
		else if (priv->gmsl_data->streams[i].st_id_sel ==
				MAX9295_PIPE_U_ST_ID)
			port_sel = MAX9295_CLK_SELU;
	}

	if (priv->gmsl_data->src_csi_port == GMSL_CSI_PORT_B) {
		st_pipe = (MAX9295_PIPE_X_START_B | MAX9295_PIPE_Y_START_B |
			MAX9295_PIPE_Z_START_B | MAX9295_PIPE_U_START_B);
		port_sel |= (MAX9295_EN_LINE_INFO | MAX9295_START_PORT_B);
	} else {
		st_pipe = MAX9295_PIPE_X_START_A | MAX9295_PIPE_Y_START_A |
			MAX9295_PIPE_Z_START_A | MAX9295_PIPE_U_START_A;
		port_sel |= (MAX9295_EN_LINE_INFO | MAX9295_START_PORT_A);
	}

	pipe_en = (MAX9295_VID_TX_EN_X | MAX9295_VID_TX_EN_Y |
		MAX9295_VID_TX_EN_Z | MAX9295_VID_TX_EN_U | MAX9295_VID_INIT);

	max9295_write_reg(dev, MAX9295_START_PIPE_ADDR, st_pipe);
	max9295_write_reg(dev, MAX9295_CSI_PORT_SEL_ADDR, port_sel);
	max9295_write_reg(dev, MAX9295_PIPE_EN_ADDR, pipe_en);

	mutex_unlock(&priv->lock);

	return 0;
}
EXPORT_SYMBOL(max9295_streamon);

int max9295_streamoff(struct device *dev)
{
	max9295_write_reg(dev, MAX9295_START_PIPE_ADDR, 0x0);
	max9295_write_reg(dev, MAX9295_PIPE_EN_ADDR, 0x0);

	return 0;
}
EXPORT_SYMBOL(max9295_streamoff);

static  struct regmap_config max9295_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max9295_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max9295 *priv;
	int err = 0;
	u8 val = 0;

	dev_info(&client->dev, "[MAX9295]: probing GMSL IO Expander\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&max9295_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	mutex_init(&priv->lock);

	max9295_read_reg(priv, 0x0010, &val);
	dev_info(&client->dev, "max9295 id = 0x%x\n", val);

	dev_set_drvdata(&client->dev, priv);

	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}

static int max9295_remove(struct i2c_client *client)
{
	if (client != NULL) {
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max9295_id[] = {
	{ "max9295", 0 },
	{ },
};

const struct of_device_id max9295_of_match[] = {
	{ .compatible = "nvidia,max9295", },
	{ },
};
MODULE_DEVICE_TABLE(of, max9295_of_match);
MODULE_DEVICE_TABLE(i2c, max9295_id);

static struct i2c_driver max9295_i2c_driver = {
	.driver = {
		.name = "max9295",
		.owner = THIS_MODULE,
	},
	.probe = max9295_probe,
	.remove = max9295_remove,
	.id_table = max9295_id,
};

static int __init max9295_init(void)
{
	return i2c_add_driver(&max9295_i2c_driver);
}

static void __exit max9295_exit(void)
{
	i2c_del_driver(&max9295_i2c_driver);
}

module_init(max9295_init);
module_exit(max9295_exit);

MODULE_DESCRIPTION("IO Expander driver max9295");
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com>");
MODULE_LICENSE("GPL v2");
