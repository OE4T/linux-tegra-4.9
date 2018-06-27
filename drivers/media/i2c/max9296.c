/*
 * max9296.c - max9296 IO Expander driver
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
#include <media/max9296.h>

/* register specifics */
#define MAX9296_DST_CSI_MODE_ADDR 0x330
#define MAX9296_LANE_MAP1_ADDR 0x333
#define MAX9296_LANE_MAP2_ADDR 0x334

#define MAX9296_LANE_CTRL0_ADDR 0x40A
#define MAX9296_LANE_CTRL1_ADDR 0x44A
#define MAX9296_LANE_CTRL2_ADDR 0x48A
#define MAX9296_LANE_CTRL3_ADDR 0x4CA

#define MAX9296_TX11_PIPE_Y_EN_ADDR 0x44B
#define MAX9296_TX45_PIPE_Y_DST_CTRL_ADDR 0x46D
#define MAX9296_PIPE_X_ST_SEL_ADDR 0x50
#define MAX9296_PIPE_Y_ST_SEL_ADDR 0x51
#define MAX9296_PIPE_Z_ST_SEL_ADDR 0x52
#define MAX9296_PIPE_U_ST_SEL_ADDR 0x53

#define MAX9296_PIPE_Y_SRC_0_MAP_ADDR 0x44D
#define MAX9296_PIPE_Y_DST_0_MAP_ADDR 0x44E
#define MAX9296_PIPE_Y_SRC_1_MAP_ADDR 0x44F
#define MAX9296_PIPE_Y_DST_1_MAP_ADDR 0x450
#define MAX9296_PIPE_Y_SRC_2_MAP_ADDR 0x451
#define MAX9296_PIPE_Y_DST_2_MAP_ADDR 0x452

#define MAX9296_PWDN_PHYS_ADDR 0x332
#define MAX9296_PHY1_CLK_ADDR 0x320
#define MAX9296_CTRL0_ADDR 0x10

#define MAX9296_MAP_SRC_0 0x1
#define MAX9296_MAP_SRC_1 0x2
#define MAX9296_MAP_SRC_2 0x4
#define MAX9296_MAP_SRC_3 0x8
#define MAX9296_MAP_SRC_4 0x10
#define MAX9296_MAP_SRC_5 0x20
#define MAX9296_MAP_SRC_6 0x40
#define MAX9296_MAP_SRC_7 0x80

#define MAX9296_MAP_DST_0 0x1
#define MAX9296_MAP_DST_1 0x4
#define MAX9296_MAP_DST_2 0x10
#define MAX9296_MAP_DST_3 0x40

#define MAX9296_DT_FS 0x0
#define MAX9296_DT_FE 0x1

#define MAX9295_PIPE_MAX_STREAMS 16

#define MAX9296_DT_MAP(vc, dt) \
	((vc << 6) | (dt & 0x3F))

#define MAX9296_CSI_MODE_4X2 0x1
#define MAX9296_CSI_MODE_2X4 0x4

#define MAX9296_LANE_MAP1_4X2 0x44
#define MAX9296_LANE_MAP2_4X2 0x44
#define MAX9296_LANE_MAP1_2X4 0x4E
#define MAX9296_LANE_MAP2_2X4 0xE4

#define MAX9296_LANE_CTRL_MAP(num_lanes) \
	(((num_lanes) << 6) & 0xF0)

#define MAX9296_PHY0_STDBY 0x00
#define MAX9296_PHY1_STDBY 0x10
#define MAX9296_PHY2_STDBY 0x20
#define MAX9296_PHY3_STDBY 0x30
#define MAX9296_ALLPHYS_STDBY \
	(MAX9296_PHY0_STDBY | MAX9296_PHY1_STDBY | \
		MAX9296_PHY2_STDBY | MAX9296_PHY3_STDBY)

#define MAX9296_ALLPHYS_NOSTDBY 0xF0
#define MAX9296_ST_ID_SEL_INVALID 0xF

#define MAX9296_PHY1_CLK 0x2C

#define MAX9296_RESET_ALL 0x80
#define MAX9296_RESET_LINK 0x40
#define MAX9296_RESET_ONESHOT 0x20
#define MAX9296_REG_EN 0x4
#define MAX9296_AUTO_LINK 0x10
#define MAX9296_LINK_CFG 0x1

#define MAX9296_MAX_SOURCES 2

struct map_dt_grp {
	u32 st_vc;
	u32 dst_vc;
	u32 dt;
};

struct max9296_source_data {
	struct device *src_dev;
	struct gmsl_link_data *gmsl_data;
};

struct max9296_pipe_id {
	u32 x_id;
	u32 y_id;
	u32 z_id;
	u32 u_id;
};

struct max9296 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct max9296_pipe_id pipes;
	u32 num_sources;
	struct max9296_source_data
		sources[MAX9296_MAX_SOURCES];
	struct mutex lock;
};

static int max9296_write_reg(struct device *dev,
	u16 addr, u8 val)
{
	struct max9296 *priv;
	int err;

	priv = dev_get_drvdata(dev);

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		dev_err(dev,
		"%s:i2c write failed, 0x%x = %x\n",
		__func__, addr, val);

	return err;
}

static int max9296_read_reg(struct max9296 *priv,
	u16 addr, u8* val)
{
	struct i2c_client *i2c_client = priv->i2c_client;
	int err;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	if (err)
		dev_err(&i2c_client->dev,
			"%s:i2c read failed, 0x%x = %x\n",
			__func__, addr, *val);

	*val = reg_val & 0xFF;

	return err;
}

int max9296_poweron(struct device *dev)
{
	max9296_write_reg(dev, MAX9296_CTRL0_ADDR,
		(MAX9296_RESET_ONESHOT |
			MAX9296_REG_EN | MAX9296_AUTO_LINK | MAX9296_LINK_CFG));
	msleep(10);
	max9296_write_reg(dev,
		MAX9296_PWDN_PHYS_ADDR, MAX9296_ALLPHYS_NOSTDBY);

	return 0;
}
EXPORT_SYMBOL(max9296_poweron);

int max9296_poweroff(struct device *dev)
{
	max9296_write_reg(dev, MAX9296_CTRL0_ADDR, MAX9296_RESET_ALL);

	return 0;
}
EXPORT_SYMBOL(max9296_poweroff);

int max9296_dev_add(struct device *dev, struct gmsl_link_data *pdata)
{
	struct max9296 *priv = NULL;
	int err;

	if (!dev || !pdata || !pdata->s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);
	mutex_lock(&priv->lock);
	if (priv->num_sources >= MAX9296_MAX_SOURCES) {
		dev_err(dev,
		"%s: MAX9296 inputs size exhausted\n", __func__);
		err = -ENOMEM;
		goto error;
	}

	priv->sources[priv->num_sources++].gmsl_data = pdata;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_dev_add);

int max9296_dev_remove(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = NULL;
	int err = 0;
	int i = 0;
	bool src_rm = false;

	if (!dev || !s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);
	mutex_lock(&priv->lock);

	if (priv->num_sources == 0) {
		dev_err(dev, "%s: no source found\n", __func__);
		err = -ENODATA;
		goto error;
	}

	for (i = 0; i < priv->num_sources; i++) {
		if (s_dev == priv->sources[i].gmsl_data->s_dev) {
			priv->sources[i].gmsl_data = NULL;
			src_rm = true;
		}
	}

	if (src_rm) {
		priv->num_sources--;
	} else {
		dev_err(dev,
			"%s: requested device not found\n", __func__);
		err = -EINVAL;
		goto error;
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_dev_remove);

int max9296_stream_setup(struct device *dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_link_data *gmsl_data;
	int err;
	int i = 0;
	u32 map_y_en = 0;
	u32 map_y_dst_en = 0;
	struct map_dt_grp map_dt[MAX9295_PIPE_MAX_STREAMS];
	u32 csi_mode = 0;
	u32 num_maps = 0;
	u32 lane_mp1 = 0;
	u32 lane_mp2 = 0;
	u32 lane_a = 0;
	u32 lane_b = 0;
	u32 lane_4x2 = 0;
	u32 src_mp = 0;
	u32 dst_mp = 0;

	mutex_lock(&priv->lock);

	switch (priv->num_sources) {
	/* TODO: Program 2nd source */
	case 1:
		gmsl_data = priv->sources[0].gmsl_data;

		if ((gmsl_data->csi_mode == GMSL_CSI_1X4_MODE ||
				gmsl_data->csi_mode == GMSL_CSI_2X4_MODE)) {
			csi_mode = MAX9296_CSI_MODE_2X4;
			lane_mp1 = MAX9296_LANE_MAP1_2X4;
			lane_mp2 = MAX9296_LANE_MAP2_2X4;
			if (gmsl_data->dst_csi_port == GMSL_CSI_PORT_A)
				lane_a = MAX9296_LANE_CTRL_MAP(
					gmsl_data->num_csi_lanes-1);
			else
				lane_b = MAX9296_LANE_CTRL_MAP(
					gmsl_data->num_csi_lanes-1);
		}
		else if ((gmsl_data->csi_mode == GMSL_CSI_2X2_MODE ||
				gmsl_data->csi_mode == GMSL_CSI_4X2_MODE)) {
			csi_mode = MAX9296_CSI_MODE_4X2;
			lane_mp1 = MAX9296_LANE_MAP1_4X2;
			lane_mp2 = MAX9296_LANE_MAP2_4X2;
		}

		for (i = 0; i < gmsl_data->num_streams; i++) {
			/* TODO: add support for other formats as well */
			if (gmsl_data->streams[i].st_data_type ==
				GMSL_CSI_DT_RAW_12) {
				map_y_en |=
					(MAX9296_MAP_SRC_0 |
					MAX9296_MAP_SRC_1 | MAX9296_MAP_SRC_2);
				map_y_dst_en |=
					(MAX9296_MAP_DST_0 |
					MAX9296_MAP_DST_1 | MAX9296_MAP_DST_2);
				map_dt[0].st_vc = map_dt[1].st_vc =
					map_dt[2].st_vc = gmsl_data->st_vc;
				map_dt[0].dst_vc = map_dt[1].dst_vc =
					map_dt[2].dst_vc = gmsl_data->dst_vc;
				map_dt[0].dt =
					gmsl_data->streams[i].st_data_type;
				map_dt[1].dt = MAX9296_DT_FS;
				map_dt[2].dt = MAX9296_DT_FE;
				num_maps = 3;
				gmsl_data->streams[i].pipe_id = GMSL_PIPE_Y_ID;
				break;
			}
		}

		if (map_y_en == 0) {
			dev_err(dev, "%s: no mapping found\n", __func__);
			err = -EINVAL;
			goto error;
		}

		max9296_write_reg(dev,
			MAX9296_TX11_PIPE_Y_EN_ADDR, map_y_en);
		max9296_write_reg(dev,
				MAX9296_TX45_PIPE_Y_DST_CTRL_ADDR,
				map_y_dst_en);

		for (i = 0; i < num_maps; i++) {
			switch (i) {
			case 0:
				src_mp = MAX9296_PIPE_Y_SRC_0_MAP_ADDR;
				dst_mp = MAX9296_PIPE_Y_DST_0_MAP_ADDR;
				break;
			case 1:
				src_mp = MAX9296_PIPE_Y_SRC_1_MAP_ADDR;
				dst_mp = MAX9296_PIPE_Y_DST_1_MAP_ADDR;
				break;
			case 2:
				src_mp = MAX9296_PIPE_Y_SRC_2_MAP_ADDR;
				dst_mp = MAX9296_PIPE_Y_DST_2_MAP_ADDR;
				break;
			}
			max9296_write_reg(dev, src_mp,
				MAX9296_DT_MAP(map_dt[i].st_vc, map_dt[i].dt));
			max9296_write_reg(dev, dst_mp,
				MAX9296_DT_MAP(map_dt[i].dst_vc, map_dt[i].dt));
		}
		break;
	default:
		dev_err(dev, "%s: invalid num sources\n", __func__);
		err = -EINVAL;
		goto error;
	}

	max9296_write_reg(dev,
		MAX9296_DST_CSI_MODE_ADDR, csi_mode);
	max9296_write_reg(dev,
		MAX9296_LANE_MAP1_ADDR, lane_mp1);
	max9296_write_reg(dev,
		MAX9296_LANE_MAP2_ADDR, lane_mp2);

	if (csi_mode == MAX9296_CSI_MODE_2X4) {
		if (lane_a) {
			max9296_write_reg(dev,
				MAX9296_LANE_CTRL1_ADDR, lane_a);
			max9296_write_reg(dev,
				MAX9296_PHY1_CLK_ADDR, MAX9296_PHY1_CLK);
		}
		if (lane_b)
			max9296_write_reg(dev,
				MAX9296_LANE_CTRL2_ADDR, lane_b);
	} else {
		max9296_write_reg(dev,
			MAX9296_LANE_CTRL0_ADDR, lane_4x2);
		max9296_write_reg(dev,
			MAX9296_LANE_CTRL1_ADDR, lane_4x2);
		max9296_write_reg(dev,
			MAX9296_LANE_CTRL2_ADDR, lane_4x2);
		max9296_write_reg(dev,
			MAX9296_LANE_CTRL3_ADDR, lane_4x2);
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_stream_setup);

int max9296_streamon(struct device *dev)
{
        struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_link_data *gmsl_data;
	int i = 0;
	int j = 0;

	mutex_lock(&priv->lock);
	for (i = 0; i < priv->num_sources; i++) {
		gmsl_data = priv->sources[i].gmsl_data;
		for (j = 0; j < gmsl_data->num_streams; j++) {
			switch (gmsl_data->streams[j].pipe_id) {
			case GMSL_PIPE_X_ID:
				if (priv->pipes.x_id ==
					MAX9296_ST_ID_SEL_INVALID) {
					priv->pipes.x_id =
						gmsl_data->streams[j].st_id_sel;
					max9296_write_reg(dev,
						MAX9296_PIPE_X_ST_SEL_ADDR,
						priv->pipes.x_id);
				}
				break;
			case GMSL_PIPE_Y_ID:
				if (priv->pipes.y_id ==
					MAX9296_ST_ID_SEL_INVALID) {
					priv->pipes.y_id =
						gmsl_data->streams[j].st_id_sel;
					max9296_write_reg(dev,
						MAX9296_PIPE_Y_ST_SEL_ADDR,
						priv->pipes.y_id);
				}
				break;
			case GMSL_PIPE_Z_ID:
				if (priv->pipes.z_id ==
					MAX9296_ST_ID_SEL_INVALID) {
					priv->pipes.z_id =
						gmsl_data->streams[j].st_id_sel;
					max9296_write_reg(dev,
						MAX9296_PIPE_Z_ST_SEL_ADDR,
						priv->pipes.z_id);
				}
				break;
			case GMSL_PIPE_U_ID:
				if (priv->pipes.u_id ==
					MAX9296_ST_ID_SEL_INVALID) {
					priv->pipes.u_id =
						gmsl_data->streams[j].st_id_sel;
					max9296_write_reg(dev,
						MAX9296_PIPE_U_ST_SEL_ADDR,
						priv->pipes.u_id);
				}
				break;
			}
		}
	}

	mutex_unlock(&priv->lock);

	return 0;
}
EXPORT_SYMBOL(max9296_streamon);

int max9296_streamoff(struct device *dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);

	if (priv->pipes.x_id != MAX9296_ST_ID_SEL_INVALID)
		max9296_write_reg(dev,
			MAX9296_PIPE_X_ST_SEL_ADDR, 0x0);
	if (priv->pipes.y_id != MAX9296_ST_ID_SEL_INVALID)
		max9296_write_reg(dev,
			MAX9296_PIPE_Y_ST_SEL_ADDR, 0x0);
	if (priv->pipes.z_id != MAX9296_ST_ID_SEL_INVALID)
		max9296_write_reg(dev,
			MAX9296_PIPE_Z_ST_SEL_ADDR, 0x0);
	if (priv->pipes.u_id != MAX9296_ST_ID_SEL_INVALID)
		max9296_write_reg(dev,
			MAX9296_PIPE_U_ST_SEL_ADDR, 0x0);

	return 0;
}
EXPORT_SYMBOL(max9296_streamoff);

static  struct regmap_config max9296_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max9296_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct max9296 *priv;
	int err = 0;
	u8 val = 0;

	dev_info(&client->dev, "[MAX9296]: probing GMSL dserializer\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
				&max9296_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	priv->pipes.x_id = priv->pipes.y_id =
		priv->pipes.z_id = priv->pipes.u_id = MAX9296_ST_ID_SEL_INVALID;

	mutex_init(&priv->lock);

	max9296_read_reg(priv, 0x0010, &val);
	dev_info(&client->dev, "max9296 id = 0x%x\n", val);

	dev_set_drvdata(&client->dev, priv);

	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}


static int max9296_remove(struct i2c_client *client)
{
	if (client != NULL) {
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max9296_id[] = {
	{ "max9296", 0 },
	{ },
};

const struct of_device_id max9296_of_match[] = {
	{ .compatible = "nvidia,max9296", },
	{ },
};
MODULE_DEVICE_TABLE(of, max9296_of_match);
MODULE_DEVICE_TABLE(i2c, max9296_id);

static struct i2c_driver max9296_i2c_driver = {
	.driver = {
		.name = "max9296",
		.owner = THIS_MODULE,
	},
	.probe = max9296_probe,
	.remove = max9296_remove,
	.id_table = max9296_id,
};

static int __init max9296_init(void)
{
	return i2c_add_driver(&max9296_i2c_driver);
}

static void __exit max9296_exit(void)
{
	i2c_del_driver(&max9296_i2c_driver);
}

module_init(max9296_init);
module_exit(max9296_exit);

MODULE_DESCRIPTION("IO Expander driver max9296");
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com");
MODULE_LICENSE("GPL v2");
