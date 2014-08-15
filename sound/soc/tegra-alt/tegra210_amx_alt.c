/*
 * tegra210_amx_alt.c - Tegra210 AMX driver
 *
 * Copyright (c) 2014 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_device.h>

#include "tegra210_xbar_alt.h"
#include "tegra210_amx_alt.h"

#define DRV_NAME "tegra210-amx"

/**
 * tegra210_amx_set_master_stream - set master stream and dependency
 * @amx: struct of tegra210_amx
 * @stream_id: one of input stream id to be a master
 * @dependency: master dependency for tansferring
 *		0 - wait on all, 1 - wait on any
 *
 * This dependency matter on starting point not every frame.
 * Once amx starts to run, it is work as wait on all.
 */
static void tegra210_amx_set_master_stream(struct tegra210_amx *amx,
				unsigned int stream_id,
				unsigned int dependency)
{
	unsigned int mask, val;

	mask = (TEGRA210_AMX_CTRL_MSTR_RX_NUM_MASK |
		TEGRA210_AMX_CTRL_RX_DEP_MASK);

	val = (stream_id << TEGRA210_AMX_CTRL_MSTR_RX_NUN_SHIFT) |
		(dependency << TEGRA210_AMX_CTRL_RX_DEP_SHIFT);

	regmap_update_bits(amx->regmap, TEGRA210_AMX_CTRL, mask, val);
}

/**
 * tegra210_amx_enable_instream - enable input stream
 * @amx: struct of tegra210_amx
 * @stream_id: amx input stream id for enabling
 */
static void tegra210_amx_enable_instream(struct tegra210_amx *amx,
					unsigned int stream_id)
{
	int reg;

	reg = TEGRA210_AMX_CTRL;

	regmap_update_bits(amx->regmap, reg,
			TEGRA210_AMX_RX_ENABLE << stream_id,
			TEGRA210_AMX_RX_ENABLE << stream_id);
}

/**
 * tegra210_amx_disable_instream - disable input stream
 * @amx: struct of tegra210_amx
 * @stream_id: amx input stream id for disabling
 */
static void tegra210_amx_disable_instream(struct tegra210_amx *amx,
					unsigned int stream_id)
{
	int reg;

	reg = TEGRA210_AMX_CTRL;

	regmap_update_bits(amx->regmap, reg,
			TEGRA210_AMX_RX_ENABLE << stream_id,
			TEGRA210_AMX_RX_DISABLE);
}

/**
 * tegra210_amx_set_out_byte_mask - set byte mask for output frame
 * @amx: struct of tegra210_amx
 * @mask1: enable for bytes 31 ~ 0
 * @mask2: enable for bytes 63 ~ 32
 */
static void tegra210_amx_set_out_byte_mask(struct tegra210_amx *amx,
					unsigned int mask1,
					unsigned int mask2)
{
	regmap_write(amx->regmap, TEGRA210_AMX_OUT_BYTE_EN0, mask1);
	regmap_write(amx->regmap, TEGRA210_AMX_OUT_BYTE_EN1, mask2);
}

/**
 * tegra210_amx_set_map_table - set map table not RAM
 * @amx: struct of tegra210_amx
 * @out_byte_addr: byte address in one frame
 * @stream_id: input stream id
 * @nth_word: n-th word in the input stream
 * @nth_byte: n-th byte in the word
 */
static void tegra210_amx_set_map_table(struct tegra210_amx *amx,
				unsigned int out_byte_addr,
				unsigned int stream_id,
				unsigned int nth_word,
				unsigned int nth_byte)
{
	unsigned char *bytes_map = (unsigned char *)&amx->map;

	bytes_map[out_byte_addr] =
			(stream_id << TEGRA210_AMX_MAP_STREAM_NUMBER_SHIFT) |
			(nth_word << TEGRA210_AMX_MAP_WORD_NUMBER_SHIFT) |
			(nth_byte << TEGRA210_AMX_MAP_BYTE_NUMBER_SHIFT);
}

/**
 * tegra210_amx_write_map_ram - write map information in RAM
 * @amx: struct of tegra210_amx
 * @addr: n-th word of input stream
 * @val : bytes mapping information of the word
 */
static void tegra210_amx_write_map_ram(struct tegra210_amx *amx,
				unsigned int addr,
				unsigned int val)
{
	unsigned int reg;

	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL,
		(addr << TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL_RAM_ADDR_SHIFT));

	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_DATA, val);

	regmap_read(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, &reg);
	reg |= TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL_ADDR_INIT_EN;

	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, reg);

	regmap_read(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, &reg);
	reg |= TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL_RW_WRITE;

	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, reg);
}

static void tegra210_amx_update_map_ram(struct tegra210_amx *amx)
{
	int i;

	for (i = 0; i < TEGRA210_AMX_RAM_DEPTH; i++)
		tegra210_amx_write_map_ram(amx, i, amx->map[i]);
}

static int tegra210_amx_sw_reset(struct tegra210_amx *amx,
				int timeout)
{
	unsigned int val;
	int wait = timeout;

	regmap_update_bits(amx->regmap, TEGRA210_AMX_SOFT_RESET,
		TEGRA210_AMX_SOFT_RESET_SOFT_RESET_MASK,
		TEGRA210_AMX_SOFT_RESET_SOFT_EN);

	do {
		regmap_read(amx->regmap, TEGRA210_AMX_SOFT_RESET, &val);
		wait--;
		if (!wait)
			return -EINVAL;
	} while (val & 0x00000001);

	regmap_update_bits(amx->regmap, TEGRA210_AMX_SOFT_RESET,
			TEGRA210_AMX_SOFT_RESET_SOFT_RESET_MASK,
			TEGRA210_AMX_SOFT_RESET_SOFT_DEFAULT);

	return 0;
}

static int tegra210_amx_get_status(struct tegra210_amx *amx)
{
	unsigned int val;

	regmap_read(amx->regmap, TEGRA210_AMX_STATUS, &val);
	val = (val & 0x00000001);

	return val;
}

static int tegra210_amx_stop(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct device *dev = codec->dev;
	struct tegra210_amx *amx = dev_get_drvdata(dev);
	int dcnt = 10, ret = 0;

	/* wait until AMX status is disabled */
	while (tegra210_amx_get_status(amx) && dcnt--)
		udelay(100);

	/* HW needs sw reset to make sure previous transaction be clean */
	ret = tegra210_amx_sw_reset(amx, 0xffff);
	if (ret) {
		dev_err(dev, "Failed at AMX%d sw reset\n", dev->id);
		return ret;
	}

	return (dcnt < 0) ? -ETIMEDOUT : 0;
}

static int tegra210_amx_runtime_suspend(struct device *dev)
{
	struct tegra210_amx *amx = dev_get_drvdata(dev);

	regcache_cache_only(amx->regmap, true);

	clk_disable_unprepare(amx->clk_amx);

	return 0;
}

#ifdef TEGRA210_AMX_MAP_READ
static unsigned int tegra210_amx_read_map_ram(struct tegra210_amx *amx,
						unsigned int addr)
{
	unsigned int val, wait;
	wait = 0xffff;

	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL,
		(addr << TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL_RAM_ADDR_SHIFT));

	regmap_read(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, &val);
	val |= TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL_ADDR_INIT_EN;
	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, val);
	regmap_read(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, &val);
	val &= ~(TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL_RW_WRITE);
	regmap_write(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, val);

	do {
		regmap_read(amx->regmap,
				TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL, &val);
		wait--;
		if (!wait)
			return -EINVAL;
	} while (val & 0x80000000);

	regmap_read(amx->regmap, TEGRA210_AMX_AHUBRAMCTL_AMX_DATA, &val);

	return val;
}
#endif

static int tegra210_amx_runtime_resume(struct device *dev)
{
	struct tegra210_amx *amx = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(amx->clk_amx);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	regcache_cache_only(amx->regmap, false);

	return 0;
}

static int tegra210_amx_set_audio_cif(struct tegra210_amx *amx,
				struct snd_pcm_hw_params *params,
				unsigned int reg)
{
	int channels, audio_bits;
	struct tegra210_xbar_cif_conf cif_conf;

	channels = params_channels(params);
	if (channels < 1 || channels > 16)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		audio_bits = TEGRA210_AUDIOCIF_BITS_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		return -EINVAL;
	}

	cif_conf.threshold = 0;
	cif_conf.audio_channels = channels;
	cif_conf.client_channels = channels;
	cif_conf.audio_bits = audio_bits;
	cif_conf.client_bits = audio_bits;
	cif_conf.expand = 0;
	cif_conf.stereo_conv = 0;
	cif_conf.replicate = 0;
	cif_conf.truncate = 0;
	cif_conf.mono_conv = 0;

	amx->soc_data->set_audio_cif(amx->regmap, reg, &cif_conf);

	return 0;
}


static int tegra210_amx_in_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_amx *amx = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = tegra210_amx_set_audio_cif(amx, params,
				TEGRA210_AMX_AXBAR_RX1_CIF_CTRL +
				(dai->id * TEGRA210_AMX_AUDIOCIF_CH_STRIDE));

	return ret;
}

static int tegra210_amx_in_trigger(struct snd_pcm_substream *substream,
				 int cmd,
				 struct snd_soc_dai *dai)
{
	struct tegra210_amx *amx = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		tegra210_amx_enable_instream(amx, dai->id);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		tegra210_amx_disable_instream(amx, dai->id);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra210_amx_out_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_amx *amx = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = tegra210_amx_set_audio_cif(amx, params,
				TEGRA210_AMX_AXBAR_TX_CIF_CTRL);

	return ret;
}

int tegra210_amx_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct device *dev = dai->dev;
	struct tegra210_amx *amx = snd_soc_dai_get_drvdata(dai);
	unsigned int byte_mask1 = 0, byte_mask2 = 0;
	unsigned int in_stream_idx, in_ch_idx, in_byte_idx;
	int i, ret = 0;

	/* HW needs sw reset to make sure previous transaction be clean */
	tegra210_amx_sw_reset(amx, 0xffff);
	if (ret) {
		dev_err(dev, "Failed at AMX sw reset\n");
		return ret;
	}

	if ((tx_num < 1) || (tx_num > 64)) {
		dev_err(dev, "Doesn't support %d tx_num, need to be 1 to 64\n",
			tx_num);
		return -EINVAL;
	}

	if (!tx_slot) {
		dev_err(dev, "tx_slot is NULL\n");
		return -EINVAL;
	}

	tegra210_amx_set_master_stream(amx, 0,
				TEGRA210_AMX_WAIT_ON_ANY);

	memset(amx->map, 0, sizeof(amx->map));

	for (i = 0; i < tx_num; i++) {
		if (tx_slot[i] != 0) {
			/* getting mapping information */
			/* n-th input stream : 0 to 3 */
			in_stream_idx = (tx_slot[i] >> 16) & 0x3;
			/* n-th audio channel of input stream : 1 to 16 */
			in_ch_idx = (tx_slot[i] >> 8) & 0x1f;
			/* n-th byte of audio channel : 0 to 3 */
			in_byte_idx = tx_slot[i] & 0x3;
			tegra210_amx_set_map_table(amx, i, in_stream_idx,
					in_ch_idx - 1,
					in_byte_idx);

			/* making byte_mask */
			if (i > 32)
				byte_mask2 |= (1 << (i - 32));
			else
				byte_mask1 |= (1 << i);
		}
	}

	tegra210_amx_update_map_ram(amx);
	tegra210_amx_set_out_byte_mask(amx, byte_mask1, byte_mask2);

	return 0;
}

static int tegra210_amx_codec_probe(struct snd_soc_codec *codec)
{
	struct tegra210_amx *amx = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = amx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_dai_ops tegra210_amx_out_dai_ops = {
	.hw_params	= tegra210_amx_out_hw_params,
	.set_channel_map = tegra210_amx_set_channel_map,
};

static struct snd_soc_dai_ops tegra210_amx_in_dai_ops = {
	.hw_params	= tegra210_amx_in_hw_params,
	.trigger	= tegra210_amx_in_trigger,
};

#define IN_DAI(id)						\
	{							\
		.name = "IN" #id,				\
		.playback = {					\
			.stream_name = "IN" #id " Receive",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = &tegra210_amx_in_dai_ops,		\
	}

#define OUT_DAI(sname, dai_ops)					\
	{							\
		.name = #sname,					\
		.capture = {					\
			.stream_name = #sname " Transmit",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = dai_ops,					\
	}

static struct snd_soc_dai_driver tegra210_amx_dais[] = {
	IN_DAI(1),
	IN_DAI(2),
	IN_DAI(3),
	IN_DAI(4),
	OUT_DAI(OUT, &tegra210_amx_out_dai_ops),
};

static const struct snd_soc_dapm_widget tegra210_amx_widgets[] = {
	SND_SOC_DAPM_AIF_IN("IN1", NULL, 0, TEGRA210_AMX_CTRL, 0, 0),
	SND_SOC_DAPM_AIF_IN("IN2", NULL, 0, TEGRA210_AMX_CTRL, 1, 0),
	SND_SOC_DAPM_AIF_IN("IN3", NULL, 0, TEGRA210_AMX_CTRL, 2, 0),
	SND_SOC_DAPM_AIF_IN("IN4", NULL, 0, TEGRA210_AMX_CTRL, 3, 0),
	SND_SOC_DAPM_AIF_OUT_E("OUT", NULL, 0, TEGRA210_AMX_ENABLE,
				TEGRA210_AMX_ENABLE_SHIFT, 0,
				tegra210_amx_stop, SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route tegra210_amx_routes[] = {
	{ "IN1",       NULL, "IN1 Receive" },
	{ "IN2",       NULL, "IN2 Receive" },
	{ "IN3",       NULL, "IN3 Receive" },
	{ "IN4",       NULL, "IN4 Receive" },
	{ "OUT",       NULL, "IN1" },
	{ "OUT",       NULL, "IN2" },
	{ "OUT",       NULL, "IN3" },
	{ "OUT",       NULL, "IN4" },
	{ "OUT Transmit", NULL, "OUT" },
};

static struct snd_soc_codec_driver tegra210_amx_codec = {
	.probe = tegra210_amx_codec_probe,
	.dapm_widgets = tegra210_amx_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra210_amx_widgets),
	.dapm_routes = tegra210_amx_routes,
	.num_dapm_routes = ARRAY_SIZE(tegra210_amx_routes),
};

static bool tegra210_amx_wr_reg(struct device *dev,
				unsigned int reg)
{
	switch (reg) {
	case TEGRA210_AMX_AXBAR_RX_INT_MASK:
	case TEGRA210_AMX_AXBAR_RX_INT_SET:
	case TEGRA210_AMX_AXBAR_RX_INT_CLEAR:
	case TEGRA210_AMX_AXBAR_RX1_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_RX2_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_RX3_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_RX4_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_TX_INT_MASK:
	case TEGRA210_AMX_AXBAR_TX_INT_SET:
	case TEGRA210_AMX_AXBAR_TX_INT_CLEAR:
	case TEGRA210_AMX_AXBAR_TX_CIF_CTRL:
	case TEGRA210_AMX_ENABLE:
	case TEGRA210_AMX_SOFT_RESET:
	case TEGRA210_AMX_CG:
	case TEGRA210_AMX_CTRL:
	case TEGRA210_AMX_OUT_BYTE_EN0:
	case TEGRA210_AMX_OUT_BYTE_EN1:
	case TEGRA210_AMX_CYA:
	case TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL:
	case TEGRA210_AMX_AHUBRAMCTL_AMX_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_amx_rd_reg(struct device *dev,
				unsigned int reg)
{
	switch (reg) {
	case TEGRA210_AMX_AXBAR_RX_STATUS:
	case TEGRA210_AMX_AXBAR_RX_INT_STATUS:
	case TEGRA210_AMX_AXBAR_RX_INT_MASK:
	case TEGRA210_AMX_AXBAR_RX_INT_SET:
	case TEGRA210_AMX_AXBAR_RX_INT_CLEAR:
	case TEGRA210_AMX_AXBAR_RX1_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_RX2_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_RX3_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_RX4_CIF_CTRL:
	case TEGRA210_AMX_AXBAR_TX_STATUS:
	case TEGRA210_AMX_AXBAR_TX_INT_STATUS:
	case TEGRA210_AMX_AXBAR_TX_INT_MASK:
	case TEGRA210_AMX_AXBAR_TX_INT_SET:
	case TEGRA210_AMX_AXBAR_TX_INT_CLEAR:
	case TEGRA210_AMX_AXBAR_TX_CIF_CTRL:
	case TEGRA210_AMX_ENABLE:
	case TEGRA210_AMX_SOFT_RESET:
	case TEGRA210_AMX_CG:
	case TEGRA210_AMX_STATUS:
	case TEGRA210_AMX_INT_STATUS:
	case TEGRA210_AMX_CTRL:
	case TEGRA210_AMX_OUT_BYTE_EN0:
	case TEGRA210_AMX_OUT_BYTE_EN1:
	case TEGRA210_AMX_CYA:
	case TEGRA210_AMX_DBG:
	case TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL:
	case TEGRA210_AMX_AHUBRAMCTL_AMX_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_amx_volatile_reg(struct device *dev,
				unsigned int reg)
{
	switch (reg) {
	case TEGRA210_AMX_AXBAR_RX_STATUS:
	case TEGRA210_AMX_AXBAR_RX_INT_STATUS:
	case TEGRA210_AMX_AXBAR_RX_INT_SET:
	case TEGRA210_AMX_AXBAR_TX_STATUS:
	case TEGRA210_AMX_AXBAR_TX_INT_STATUS:
	case TEGRA210_AMX_AXBAR_TX_INT_SET:
	case TEGRA210_AMX_SOFT_RESET:
	case TEGRA210_AMX_STATUS:
	case TEGRA210_AMX_INT_STATUS:
	case TEGRA210_AMX_AHUBRAMCTL_AMX_CTRL:
	case TEGRA210_AMX_AHUBRAMCTL_AMX_DATA:
		return true;
	default:
		break;
	};

	return false;
}

static const struct regmap_config tegra210_amx_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_AMX_AHUBRAMCTL_AMX_DATA,
	.writeable_reg = tegra210_amx_wr_reg,
	.readable_reg = tegra210_amx_rd_reg,
	.volatile_reg = tegra210_amx_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

static const struct tegra210_amx_soc_data soc_data_tegra210 = {
	.set_audio_cif = tegra210_xbar_set_cif
};

static const struct of_device_id tegra210_amx_of_match[] = {
	{ .compatible = "nvidia,tegra210-amx", .data = &soc_data_tegra210 },
	{},
};

static int tegra210_amx_platform_probe(struct platform_device *pdev)
{
	struct tegra210_amx *amx;
	struct resource *mem, *memregion;
	void __iomem *regs;
	int ret;
	const struct of_device_id *match;
	struct tegra210_amx_soc_data *soc_data;

	match = of_match_device(tegra210_amx_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		ret = -ENODEV;
		goto err;
	}
	soc_data = (struct tegra210_amx_soc_data *)match->data;

	amx = devm_kzalloc(&pdev->dev, sizeof(struct tegra210_amx), GFP_KERNEL);
	if (!amx) {
		dev_err(&pdev->dev, "Can't allocate tegra210_amx\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, amx);

	amx->soc_data = soc_data;

	amx->clk_amx = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(amx->clk_amx)) {
		dev_err(&pdev->dev, "Can't retrieve tegra210_amx clock\n");
		ret = PTR_ERR(amx->clk_amx);
		goto err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_clk_put;
	}

	memregion = devm_request_mem_region(&pdev->dev, mem->start,
					    resource_size(mem), DRV_NAME);
	if (!memregion) {
		dev_err(&pdev->dev, "Memory region already claimed\n");
		ret = -EBUSY;
		goto err_clk_put;
	}

	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_clk_put;
	}

	amx->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &tegra210_amx_regmap_config);
	if (IS_ERR(amx->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(amx->regmap);
		goto err_clk_put;
	}
	regcache_cache_only(amx->regmap, true);

	if (of_property_read_u32(pdev->dev.of_node,
				"nvidia,ahub-amx-id",
				&pdev->dev.id) < 0) {
		dev_err(&pdev->dev,
			"Missing property nvidia,ahub-amx-id\n");
		ret = -ENODEV;
		goto err_clk_put;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_amx_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = snd_soc_register_codec(&pdev->dev, &tegra210_amx_codec,
				     tegra210_amx_dais,
				     ARRAY_SIZE(tegra210_amx_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_amx_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_clk_put:
	devm_clk_put(&pdev->dev, amx->clk_amx);
err:
	return ret;
}

static int tegra210_amx_platform_remove(struct platform_device *pdev)
{
	struct tegra210_amx *amx = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_amx_runtime_suspend(&pdev->dev);

	devm_clk_put(&pdev->dev, amx->clk_amx);

	return 0;
}

static const struct dev_pm_ops tegra210_amx_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_amx_runtime_suspend,
			   tegra210_amx_runtime_resume, NULL)
};

static struct platform_driver tegra210_amx_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_amx_of_match,
		.pm = &tegra210_amx_pm_ops,
	},
	.probe = tegra210_amx_platform_probe,
	.remove = tegra210_amx_platform_remove,
};
module_platform_driver(tegra210_amx_driver);

MODULE_AUTHOR("Songhee Baek <sbaek@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 AMX ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra210_amx_of_match);
