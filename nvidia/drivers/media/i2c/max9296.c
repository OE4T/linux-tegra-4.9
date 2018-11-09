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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
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

#define MAX9296_TX11_PIPE_X_EN_ADDR 0x40B
#define MAX9296_TX45_PIPE_X_DST_CTRL_ADDR 0x42D

#define MAX9296_PIPE_X_SRC_0_MAP_ADDR 0x40D
#define MAX9296_PIPE_X_DST_0_MAP_ADDR 0x40E
#define MAX9296_PIPE_X_SRC_1_MAP_ADDR 0x40F
#define MAX9296_PIPE_X_DST_1_MAP_ADDR 0x410
#define MAX9296_PIPE_X_SRC_2_MAP_ADDR 0x411
#define MAX9296_PIPE_X_DST_2_MAP_ADDR 0x412

#define MAX9296_PIPE_X_ST_SEL_ADDR 0x50

#define MAX9296_PWDN_PHYS_ADDR 0x332
#define MAX9296_PHY1_CLK_ADDR 0x320
#define MAX9296_CTRL0_ADDR 0x10

#define MAX9296_CSI_MODE_4X2 0x1
#define MAX9296_CSI_MODE_2X4 0x4
#define MAX9296_LANE_MAP1_4X2 0x44
#define MAX9296_LANE_MAP2_4X2 0x44
#define MAX9296_LANE_MAP1_2X4 0x4E
#define MAX9296_LANE_MAP2_2X4 0xE4

#define MAX9296_LANE_CTRL_MAP(num_lanes) \
	(((num_lanes) << 6) & 0xF0)

#define MAX9296_ALLPHYS_NOSTDBY 0xF0
#define MAX9296_ST_ID_SEL_INVALID 0xF

#define MAX9296_PHY1_CLK 0x2C

#define MAX9296_RESET_ALL 0x80

#define MAX9296_MAX_SOURCES 2
#define MAX9296_MAX_PIPES 0x4

#define MAX9296_PIPE_X 0
#define MAX9296_PIPE_Y 1
#define MAX9296_PIPE_Z 2
#define MAX9296_PIPE_U 3

#define MAX9296_CSI_CTRL_0 0
#define MAX9296_CSI_CTRL_1 1
#define MAX9296_CSI_CTRL_2 2
#define MAX9296_CSI_CTRL_3 3

#define MAX9296_INVAL_ST_ID 0xFF

struct max9296_source_ctx {
	struct gmsl_link_ctx *g_ctx;
	bool st_enabled;
};

struct pipe_ctx {
	u32 id;
	u32 dt_type;
	u32 dst_csi_ctrl;
	u32 st_count;
	u32 st_id_sel;
};

struct max9296 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	u32 num_src;
	u32 max_src;
	struct max9296_source_ctx sources[MAX9296_MAX_SOURCES];
	struct mutex lock;
	u32 sdev_ref;
	bool power_on;
	bool lane_setup;
	bool link_ex;
	struct pipe_ctx pipe[MAX9296_MAX_PIPES];
	u8 csi_mode;
	u8 lane_mp1;
	u8 lane_mp2;
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

	/* delay before next i2c command as required for SERDES link */
	usleep_range(100, 110);

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

static void max9296_reset_ctx(struct max9296 *priv)
{
	int i;

	priv->power_on = false;
	priv->link_ex = false;
	priv->lane_setup = false;
	for (i = 0; i < priv->num_src; i++)
		priv->sources[i].st_enabled = false;
}

int max9296_link_ex(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	u32 link;
	int err = 0;
	int i;

	mutex_lock(&priv->lock);
	for (i = 0; i < priv->num_src; i++) {
		if (priv->sources[i].g_ctx->s_dev == s_dev) {
			if (priv->sources[i].st_enabled)
				goto ret;
			break;
		}
	}

	if (i == priv->num_src) {
		dev_err(dev, "sdev not registered\n");
		err = -EINVAL;
		goto ret;
	}

	link = priv->sources[i].g_ctx->serdes_csi_link;

	if (!priv->power_on) {
		if (link == GMSL_SERDES_CSI_LINK_A) {
			max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x01);
			max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x21);
			/* delay to settle link */
			msleep(100);
		} else if (link == GMSL_SERDES_CSI_LINK_B) {
			max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x02);
			max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x22);
			/* delay to settle link */
			msleep(100);
		} else { /* Extend for DES having more than two GMSL links */
			dev_err(dev, "%s: invalid gmsl link\n", __func__);
			err = -EINVAL;
			goto ret;
		}
		priv->link_ex = true;
	}

ret:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_link_ex);

int max9296_setup_control(struct device *dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int err = 0;

	if (!priv->link_ex) {
		dev_err(dev, "%s: invalid state\n", __func__);
		err = -EINVAL;
		goto error;
	}

	if (!priv->power_on) {
		/* Enable splitter mode */
		max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x03);
		max9296_write_reg(dev, MAX9296_CTRL0_ADDR, 0x23);
		/* delay to settle link */
		msleep(100);
		max9296_write_reg(dev,
			MAX9296_PWDN_PHYS_ADDR, MAX9296_ALLPHYS_NOSTDBY);

		priv->power_on = true;
	}

	priv->sdev_ref++;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_setup_control);

int max9296_reset_control(struct device *dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int err = 0;

	if (!priv->sdev_ref) {
		dev_err(dev, "%s: invalid state\n", __func__);
		err = -EINVAL;
		goto error;
	}

	priv->sdev_ref--;

	if (priv->sdev_ref == 0) {
		max9296_reset_ctx(priv);
		max9296_write_reg(dev, MAX9296_CTRL0_ADDR, MAX9296_RESET_ALL);
		/* delay to settle reset */
		msleep(100);
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_reset_control);

int max9296_sdev_register(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
	struct max9296 *priv = NULL;
	int i;
	int err = 0;

	if (!dev || !g_ctx || !g_ctx->s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);

	mutex_lock(&priv->lock);

	if (priv->num_src > priv->max_src) {
		dev_err(dev,
			"%s: MAX9296 inputs size exhausted\n", __func__);
		err = -ENOMEM;
		goto error;
	}

	/* Check csi mode compatibility */
	if (!((priv->csi_mode == MAX9296_CSI_MODE_2X4) ?
			((g_ctx->csi_mode == GMSL_CSI_1X4_MODE) ||
				(g_ctx->csi_mode == GMSL_CSI_2X4_MODE)) :
			((g_ctx->csi_mode == GMSL_CSI_2X2_MODE) ||
				(g_ctx->csi_mode == GMSL_CSI_4X2_MODE)))) {
		dev_err(dev,
			"%s: csi mode not supported\n", __func__);
		err = -EINVAL;
		goto error;
	}

	for (i = 0; i < priv->num_src; i++) {
		if (g_ctx->serdes_csi_link ==
			priv->sources[i].g_ctx->serdes_csi_link) {
			dev_err(dev,
				"%s: serdes csi link is in use\n", __func__);
			err = -EINVAL;
			goto error;
		}
		/*
		 * All sdevs should have same num-csi-lanes regardless of
		 * dst csi port selected.
		 * Later if there is any usecase which requires each port
		 * to be configured with different num-csi-lanes, then this
		 * check should be performed per port.
		 */
		if (g_ctx->num_csi_lanes !=
				priv->sources[i].g_ctx->num_csi_lanes) {
			dev_err(dev,
				"%s: csi num lanes mismatch\n", __func__);
			err = -EINVAL;
			goto error;
		}
	}

	priv->sources[priv->num_src].g_ctx = g_ctx;
	priv->sources[priv->num_src].st_enabled = false;

	priv->num_src++;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_sdev_register);

int max9296_sdev_unregister(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = NULL;
	int err = 0;
	int i = 0;

	if (!dev || !s_dev) {
		dev_err(dev, "%s: invalid input params\n", __func__);
		return -EINVAL;
	}

	priv = dev_get_drvdata(dev);
	mutex_lock(&priv->lock);

	if (priv->num_src == 0) {
		dev_err(dev, "%s: no source found\n", __func__);
		err = -ENODATA;
		goto error;
	}

	for (i = 0; i < priv->num_src; i++) {
		if (s_dev == priv->sources[i].g_ctx->s_dev) {
			priv->sources[i].g_ctx = NULL;
			priv->num_src--;
			break;
		}
	}

	if (i == priv->num_src) {
		dev_err(dev,
			"%s: requested device not found\n", __func__);
		err = -EINVAL;
		goto error;
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_sdev_unregister);

static int max9296_get_available_pipe(struct device *dev,
				u32 st_data_type, u32 dst_csi_port)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX9296_MAX_PIPES; i++) {
		/*
		 * TODO: Enable a pipe for multi stream configuration having
		 * similar stream data type. For now use st_count as a flag
		 * for 1 to 1 mapping in pipe and stream data type, same can
		 * be extended as count for many to 1 mapping. Would also need
		 * few more checks such as input stream id select, dst port etc.
		 */
		if ((priv->pipe[i].dt_type == st_data_type) &&
			((dst_csi_port == GMSL_CSI_PORT_A) ?
				(priv->pipe[i].dst_csi_ctrl ==
					MAX9296_CSI_CTRL_0) ||
				(priv->pipe[i].dst_csi_ctrl ==
					MAX9296_CSI_CTRL_1) :
				(priv->pipe[i].dst_csi_ctrl ==
					MAX9296_CSI_CTRL_2) ||
				(priv->pipe[i].dst_csi_ctrl ==
					MAX9296_CSI_CTRL_3)) &&
			(!priv->pipe[i].st_count))
			break;
	}

	if (i == MAX9296_MAX_PIPES) {
		dev_err(dev, "%s: all pipes are busy\n", __func__);
		return -ENOMEM;
	}

	return i;
}

struct reg_pair {
	u16 addr;
	u8 val;
};

static int max9296_setup_pipeline(struct device *dev,
		struct gmsl_link_ctx *g_ctx)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_stream *g_stream;
	struct reg_pair *map_list;
	u32 arr_sz = 0;
	int pipe_id = 0;
	u32 i = 0;
	u32 j = 0;
	u32 vc_idx = 0;

	for (i = 0; i < g_ctx->num_streams; i++) {
		/* Base data type mapping: pipeX/RAW12/CSICNTR1 */
		struct reg_pair map_pipe_raw12[] = {
			/* addr, val */
			{MAX9296_TX11_PIPE_X_EN_ADDR, 0x7},
			{MAX9296_TX45_PIPE_X_DST_CTRL_ADDR, 0x15},
			{MAX9296_PIPE_X_SRC_0_MAP_ADDR, 0x2C},
			{MAX9296_PIPE_X_DST_0_MAP_ADDR, 0x2C},
			{MAX9296_PIPE_X_SRC_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_DST_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_SRC_2_MAP_ADDR, 0x01},
			{MAX9296_PIPE_X_DST_2_MAP_ADDR, 0x01},
		};

		/* Base data type mapping: pipeX/EMBED/CSICNTR1 */
		struct reg_pair map_pipe_embed[] = {
			/* addr, val */
			{MAX9296_TX11_PIPE_X_EN_ADDR, 0x7},
			{MAX9296_TX45_PIPE_X_DST_CTRL_ADDR, 0x15},
			{MAX9296_PIPE_X_SRC_0_MAP_ADDR, 0x12},
			{MAX9296_PIPE_X_DST_0_MAP_ADDR, 0x12},
			{MAX9296_PIPE_X_SRC_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_DST_1_MAP_ADDR, 0x00},
			{MAX9296_PIPE_X_SRC_2_MAP_ADDR, 0x01},
			{MAX9296_PIPE_X_DST_2_MAP_ADDR, 0x01},
		};

		g_stream = &g_ctx->streams[i];

		if (g_stream->st_data_type == GMSL_CSI_DT_RAW_12) {
			map_list = map_pipe_raw12;
			arr_sz = ARRAY_SIZE(map_pipe_raw12);
		} else if (g_stream->st_data_type == GMSL_CSI_DT_EMBED) {
			map_list = map_pipe_embed;
			arr_sz = ARRAY_SIZE(map_pipe_embed);
		} else if (g_stream->st_data_type == GMSL_CSI_DT_UED_U1) {
			dev_dbg(dev,
				"%s: No mapping for GMSL_CSI_DT_UED_U1\n",
				__func__);
			continue;
		} else {
			dev_err(dev, "%s: Invalid data type\n", __func__);
			return -EINVAL;
		}

		pipe_id = max9296_get_available_pipe(dev,
				g_stream->st_data_type, g_ctx->dst_csi_port);
		if (pipe_id < 0)
			return pipe_id;

		for (j = 0, vc_idx = 3; j < arr_sz; j++, vc_idx += 2) {
			/* update pipe configuration */
			map_list[j].addr += (0x40 * pipe_id);
			/* update vc id configuration */
			if (vc_idx < arr_sz)
				map_list[vc_idx].val |=
					(g_ctx->dst_vc << 6);

			max9296_write_reg(dev, map_list[j].addr,
						map_list[j].val);
		}

		/* Enable stream id select input */
		if (g_stream->st_id_sel == GMSL_ST_ID_UNUSED) {
			dev_err(dev, "%s: Invalid stream st_id_sel\n",
				__func__);
			return -EINVAL;
		}
		max9296_write_reg(dev,
			(MAX9296_PIPE_X_ST_SEL_ADDR + pipe_id),
			g_stream->st_id_sel);

		/* Update pipe internals */
		priv->pipe[pipe_id].st_count++;
		priv->pipe[pipe_id].st_id_sel = g_stream->st_id_sel;
	}

	return 0;
}

int max9296_setup_streaming(struct device *dev, struct device *s_dev)
{
	struct max9296 *priv = dev_get_drvdata(dev);
	struct gmsl_link_ctx *g_ctx;
	int err = 0;
	int i = 0;
	u16 lane_ctrl_addr;

	mutex_lock(&priv->lock);
	for (i = 0; i < priv->num_src; i++) {
		if (priv->sources[i].g_ctx->s_dev == s_dev) {
			if (priv->sources[i].st_enabled)
				goto error;
			break;
		}
	}

	if (i == priv->num_src) {
		dev_err(dev, "sdev not registered\n");
		err = -EINVAL;
		goto error;
	}

	g_ctx = priv->sources[i].g_ctx;

	err = max9296_setup_pipeline(dev, g_ctx);
	if (err)
		goto error;

	/* Derive CSI lane map register */
	switch (g_ctx->dst_csi_port) {
	case GMSL_CSI_PORT_A:
	case GMSL_CSI_PORT_D:
		lane_ctrl_addr = MAX9296_LANE_CTRL1_ADDR;
		break;
	case GMSL_CSI_PORT_B:
	case GMSL_CSI_PORT_E:
		lane_ctrl_addr = MAX9296_LANE_CTRL2_ADDR;
		break;
	case GMSL_CSI_PORT_C:
		lane_ctrl_addr = MAX9296_LANE_CTRL0_ADDR;
		break;
	case GMSL_CSI_PORT_F:
		lane_ctrl_addr = MAX9296_LANE_CTRL3_ADDR;
		break;
	default:
		dev_err(dev, "invalid gmsl csi port!\n");
		err = -EINVAL;
		goto error;
	};

	/*
	 * rewrite num_lanes to same dst port should not be an issue,
	 * as the device compatibility is already
	 * checked during sdev registration against the des properties.
	 */
	max9296_write_reg(dev, lane_ctrl_addr,
		MAX9296_LANE_CTRL_MAP(g_ctx->num_csi_lanes-1));

	if (!priv->lane_setup) {
		max9296_write_reg(dev,
			MAX9296_DST_CSI_MODE_ADDR, priv->csi_mode);
		max9296_write_reg(dev,
			MAX9296_LANE_MAP1_ADDR, priv->lane_mp1);
		max9296_write_reg(dev,
			MAX9296_LANE_MAP2_ADDR, priv->lane_mp2);
		max9296_write_reg(dev,
			MAX9296_PHY1_CLK_ADDR, MAX9296_PHY1_CLK);

		priv->lane_setup = true;
	}

	priv->sources[i].st_enabled = true;

error:
	mutex_unlock(&priv->lock);
	return err;
}
EXPORT_SYMBOL(max9296_setup_streaming);

const struct of_device_id max9296_of_match[] = {
	{ .compatible = "nvidia,max9296", },
	{ },
};
MODULE_DEVICE_TABLE(of, max9296_of_match);

static int max9296_parse_dt(struct max9296 *priv,
				struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	int err = 0;
	const char *str_value;
	int value;
	const struct of_device_id *match;

	if (!node)
		return -EINVAL;

	match = of_match_device(max9296_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return -EFAULT;
	}

	err = of_property_read_string(node, "csi-mode", &str_value);
	if (err < 0) {
		dev_err(&client->dev, "csi-mode property not found\n");
		return err;
	}

	if (!strcmp(str_value, "2x4")) {
		priv->csi_mode = MAX9296_CSI_MODE_2X4;
		priv->lane_mp1 = MAX9296_LANE_MAP1_2X4;
		priv->lane_mp2 = MAX9296_LANE_MAP2_2X4;
	} else if (!strcmp(str_value, "4x2")) {
		priv->csi_mode = MAX9296_CSI_MODE_4X2;
		priv->lane_mp1 = MAX9296_LANE_MAP1_4X2;
		priv->lane_mp2 = MAX9296_LANE_MAP2_4X2;
	} else {
		dev_err(&client->dev, "invalid csi mode\n");
		return -EINVAL;
	}

	err = of_property_read_u32(node, "max-src", &value);
	if (err < 0) {
		dev_err(&client->dev, "No max-src info\n");
		return err;
	}
	priv->max_src = value;

	return 0;
}

static void max9296_pipes_init(struct max9296 *priv)
{
	/*
	 * This is default pipes combination. add more mappings
	 * for other combinations and requirements.
	 */
	struct pipe_ctx pipe_defaults[] = {
		{MAX9296_PIPE_X, GMSL_CSI_DT_RAW_12,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID},
		{MAX9296_PIPE_Y, GMSL_CSI_DT_RAW_12,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID},
		{MAX9296_PIPE_Z, GMSL_CSI_DT_EMBED,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID},
		{MAX9296_PIPE_U, GMSL_CSI_DT_EMBED,
			MAX9296_CSI_CTRL_1, 0, MAX9296_INVAL_ST_ID}
	};

	/*
	 * Add DT props for num-streams and stream sequence, and based on that
	 * set the appropriate pipes defaults.
	 * For now default it supports "2 RAW12 and 2 EMBED" 1:1 mappings.
	 */
	memcpy(priv->pipe, pipe_defaults, sizeof(pipe_defaults));
}

static struct regmap_config max9296_regmap_config = {
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

	priv->power_on = false;
	priv->lane_setup = false;
	priv->link_ex = false;

	err = max9296_parse_dt(priv, client);
	if (err) {
		dev_err(&client->dev, "unable to parse dt\n");
		return -EFAULT;
	}

	max9296_pipes_init(priv);

	if (priv->max_src > MAX9296_MAX_SOURCES) {
		dev_err(&client->dev,
			"max sources more than currently supported\n");
		return -EINVAL;
	}

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

MODULE_DEVICE_TABLE(i2c, max9296_id);

static struct i2c_driver max9296_i2c_driver = {
	.driver = {
		.name = "max9296",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max9296_of_match),
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
