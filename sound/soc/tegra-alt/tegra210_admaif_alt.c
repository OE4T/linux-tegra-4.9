/*
 * tegra210_admaif_alt.c - Tegra ADMAIF driver
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
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <mach/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra_pcm_alt.h"
#include "tegra210_xbar_alt.h"
#include "tegra210_admaif_alt.h"

#define DRV_NAME "tegra210-ape-admaif"

static bool tegra210_admaif_wr_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;

	switch (reg) {
	case TEGRA210_ADMAIF_XBAR_TX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_TX_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL:
	case TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET:
	case TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL:
	case TEGRA210_ADMAIF_XBAR_RX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL:
	case TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET:
	case TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL:
	case TEGRA210_ADMAIF_GLOBAL_ENABLE:
		return true;
	default:
		break;
	};

	return false;
}

static bool tegra210_admaif_rd_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;

	switch (reg) {
	case TEGRA210_ADMAIF_XBAR_RX_STATUS:
	case TEGRA210_ADMAIF_XBAR_RX_INT_STATUS:
	case TEGRA210_ADMAIF_XBAR_RX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET:
	case TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL:
	case TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL:
	case TEGRA210_ADMAIF_XBAR_TX_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_INT_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_ENABLE:
	case TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET:
	case TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL:
	case TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL:
	case TEGRA210_ADMAIF_GLOBAL_ENABLE:
		return true;
	default:
		return false;
	};
}

static bool tegra210_admaif_volatile_reg(struct device *dev, unsigned int reg)
{
	reg = reg % TEGRA210_ADMAIF_CHANNEL_REG_STRIDE;

	switch (reg) {
	case TEGRA210_ADMAIF_XBAR_RX_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_STATUS:
	case TEGRA210_ADMAIF_XBAR_RX_INT_STATUS:
	case TEGRA210_ADMAIF_XBAR_TX_INT_STATUS:
	case TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET:
	case TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET:
		return true;
	default:
		break;
	};

	return false;
}

static const struct regmap_config tegra210_admaif_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_ADMAIF_LAST_REG,
	.writeable_reg = tegra210_admaif_wr_reg,
	.readable_reg = tegra210_admaif_rd_reg,
	.volatile_reg = tegra210_admaif_volatile_reg,
	.cache_type = REGCACHE_FLAT,
};

static void tegra210_admaif_global_enable(struct tegra210_admaif *admaif,
					int enable)
{
	if (enable) {
		regmap_update_bits(admaif->regmap,
				TEGRA210_ADMAIF_GLOBAL_ENABLE, 1, 1);
		admaif->refcnt++;
	} else {
		admaif->refcnt--;

		if (!admaif->refcnt)
			regmap_update_bits(admaif->regmap,
				TEGRA210_ADMAIF_GLOBAL_ENABLE, 1, 0);
	}
}

static int tegra210_admaif_sw_reset(struct snd_soc_dai *dai,
				int direction, int timeout)
{
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int sw_reset_reg, val;
	int wait = timeout;

	if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
		sw_reset_reg = TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	} else {
		sw_reset_reg = TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	}

	regmap_update_bits(admaif->regmap, sw_reset_reg, 1, 1);

	do {
		regmap_read(admaif->regmap, sw_reset_reg, &val);
		wait--;
		if (!wait)
			return -EINVAL;
	} while (val & 0x00000001);

	return 0;
}

static int tegra210_admaif_get_status(struct snd_soc_dai *dai,
				int direction)
{
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int status_reg, val;

	if (direction == SNDRV_PCM_STREAM_PLAYBACK) {
		status_reg = TEGRA210_ADMAIF_XBAR_RX_STATUS +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	} else {
		status_reg = TEGRA210_ADMAIF_XBAR_TX_STATUS +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	}
	regmap_read(admaif->regmap, status_reg, &val);
	val = (val & 0x00000001);

	return val;
}

static int tegra210_admaif_runtime_suspend(struct device *dev)
{
	struct tegra210_admaif *admaif = dev_get_drvdata(dev);

	regcache_cache_only(admaif->regmap, true);
	pm_runtime_put_sync(dev->parent);

	return 0;
}

static int tegra210_admaif_runtime_resume(struct device *dev)
{
	struct tegra210_admaif *admaif = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_get_sync(dev->parent);
	if (ret < 0) {
		dev_err(dev, "parent get_sync failed: %d\n", ret);
		return ret;
	}

	regcache_cache_only(admaif->regmap, false);
	regcache_sync(admaif->regmap);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra210_admaif_suspend(struct device *dev)
{
	struct tegra210_admaif *admaif = dev_get_drvdata(dev);

	regcache_mark_dirty(admaif->regmap);
	return 0;
}
#endif

static int tegra210_admaif_set_pack_mode(struct regmap *map, unsigned int reg,
					int valid_bit)
{
	switch (valid_bit) {
	case DATA_8BIT:
		regmap_update_bits(map, reg,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_MASK,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN);
		regmap_update_bits(map, reg,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_MASK,
			0);
		break;
	case DATA_16BIT:
		regmap_update_bits(map, reg,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_MASK,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN);
		regmap_update_bits(map, reg,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_MASK,
			0);
		break;
	case DATA_32BIT:
		regmap_update_bits(map, reg,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_MASK,
			0);
		regmap_update_bits(map, reg,
			TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_MASK,
			0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra210_admaif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	struct tegra210_xbar_cif_conf cif_conf;
	unsigned int reg, fifo_ctrl, fifo_size;
	int valid_bit;

	cif_conf.audio_channels = params_channels(params);
	cif_conf.client_channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_16;
		valid_bit = DATA_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_32;
		valid_bit  = DATA_32BIT;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}

	cif_conf.threshold = 0;
	cif_conf.expand = 0;
	cif_conf.stereo_conv = 0;
	cif_conf.replicate = 0;
	cif_conf.truncate = 0;
	cif_conf.mono_conv = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		reg = TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		fifo_ctrl = TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		fifo_size = 3;
	} else {
		reg = TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		fifo_ctrl = TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL +
			(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		fifo_size = 3;
	}

	tegra210_admaif_set_pack_mode(admaif->regmap, reg, valid_bit);
	admaif->soc_data->set_audio_cif(admaif->regmap, reg, &cif_conf);

	regmap_update_bits(admaif->regmap, fifo_ctrl,
			TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_MASK,
			fifo_size << TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT);

	regmap_update_bits(admaif->regmap, fifo_ctrl,
		TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_MASK,
		(0x4 * dai->id)
			<< TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT);

	return 0;
}

static void tegra210_admaif_start_playback(struct snd_soc_dai *dai)
{
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int reg;

	tegra210_admaif_global_enable(admaif, 1);

	reg = TEGRA210_ADMAIF_XBAR_TX_ENABLE +
		(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	regmap_update_bits(admaif->regmap, reg,
				TEGRA210_ADMAIF_XBAR_TX_ENABLE_MASK,
				TEGRA210_ADMAIF_XBAR_TX_EN);
}

static void tegra210_admaif_stop_playback(struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int reg;
	int dcnt = 10, ret;

	tegra210_admaif_global_enable(admaif, 0);
	reg = TEGRA210_ADMAIF_XBAR_TX_ENABLE +
		(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	regmap_update_bits(admaif->regmap, reg,
				TEGRA210_ADMAIF_XBAR_TX_ENABLE_MASK,
				0);

	/* wait until ADMAIF TX status is disabled */
	while (tegra210_admaif_get_status(dai, SNDRV_PCM_STREAM_PLAYBACK) &&
			dcnt--)
		udelay(100);

	/* HW needs sw reset to make sure previous transaction be clean */
	ret = tegra210_admaif_sw_reset(dai, SNDRV_PCM_STREAM_PLAYBACK, 0xffff);
	if (ret)
		dev_err(dev, "Failed at ADMAIF%d_TX sw reset\n", dev->id);
}

static void tegra210_admaif_start_capture(struct snd_soc_dai *dai)
{
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int reg;

	tegra210_admaif_global_enable(admaif, 1);
	reg = TEGRA210_ADMAIF_XBAR_RX_ENABLE +
		(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	regmap_update_bits(admaif->regmap, reg,
				TEGRA210_ADMAIF_XBAR_RX_ENABLE_MASK,
				TEGRA210_ADMAIF_XBAR_RX_EN);
}

static void tegra210_admaif_stop_capture(struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int reg;
	int dcnt = 10, ret;

	tegra210_admaif_global_enable(admaif, 0);
	reg = TEGRA210_ADMAIF_XBAR_RX_ENABLE +
		(dai->id * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
	regmap_update_bits(admaif->regmap, reg,
				TEGRA210_ADMAIF_XBAR_RX_ENABLE_MASK,
				0);

	/* wait until ADMAIF RX status is disabled */
	while (tegra210_admaif_get_status(dai, SNDRV_PCM_STREAM_CAPTURE) &&
			dcnt--)
		udelay(100);

	/* HW needs sw reset to make sure previous transaction be clean */
	ret = tegra210_admaif_sw_reset(dai, SNDRV_PCM_STREAM_CAPTURE, 0xffff);
	if (ret)
		dev_err(dev, "Failed at ADMAIF%d_RX sw reset\n", dev->id);
}

static int tegra210_admaif_trigger(struct snd_pcm_substream *substream, int cmd,
				 struct snd_soc_dai *dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra210_admaif_start_playback(dai);
		else
			tegra210_admaif_start_capture(dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra210_admaif_stop_playback(dai);
		else
			tegra210_admaif_stop_capture(dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops tegra210_admaif_dai_ops = {
	.hw_params	= tegra210_admaif_hw_params,
	.trigger	= tegra210_admaif_trigger,
};

static int tegra210_admaif_enable(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct device *dev = codec->dev;
	struct tegra210_admaif *admaif = dev_get_drvdata(dev);

	/* Note: ADMAIF channel is enabled/disabled by ADSP */
	tegra210_admaif_global_enable(admaif, !!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra210_admaif_dai_probe(struct snd_soc_dai *dai)
{
	struct tegra210_admaif *admaif = snd_soc_dai_get_drvdata(dai);

	dai->capture_dma_data = &admaif->capture_dma_data[dai->id];
	dai->playback_dma_data = &admaif->playback_dma_data[dai->id];

	return 0;
}

#define ADMAIF_DAI(id)							\
	{							\
		.name = "ADMAIF" #id,				\
		.probe = tegra210_admaif_dai_probe,		\
		.playback = {					\
			.stream_name = "Playback " #id,		\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.capture = {					\
			.stream_name = "Capture " #id,		\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = &tegra210_admaif_dai_ops,			\
	}

static struct snd_soc_dai_driver tegra210_admaif_dais[10] = {
	ADMAIF_DAI(1),
	ADMAIF_DAI(2),
	ADMAIF_DAI(3),
	ADMAIF_DAI(4),
	ADMAIF_DAI(5),
	ADMAIF_DAI(6),
	ADMAIF_DAI(7),
	ADMAIF_DAI(8),
	ADMAIF_DAI(9),
	ADMAIF_DAI(10),
};

#define ADMAIF_CODEC_DAI(id)						\
	{								\
		.name = "ADMAIF" #id " FIFO",				\
		.playback = {						\
			.stream_name = "ADMAIF" #id " FIFO Transmit",	\
			.channels_min = 2,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_96000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
		.capture = {						\
			.stream_name = "ADMAIF" #id " FIFO Receive",	\
			.channels_min = 2,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_96000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
		.ops = &tegra210_admaif_dai_ops,			\
	},								\
	{								\
		.name = "ADMAIF" #id " CIF",				\
		.playback = {						\
			.stream_name = "ADMAIF" #id " CIF Transmit",	\
			.channels_min = 2,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_96000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
		.capture = {						\
			.stream_name = "ADMAIF" #id " CIF Receive",	\
			.channels_min = 2,				\
			.channels_max = 2,				\
			.rates = SNDRV_PCM_RATE_8000_96000,		\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,		\
		},							\
	}

static struct snd_soc_dai_driver tegra210_admaif_codec_dais[] = {
	ADMAIF_CODEC_DAI(1),
	ADMAIF_CODEC_DAI(2),
	ADMAIF_CODEC_DAI(3),
	ADMAIF_CODEC_DAI(4),
	ADMAIF_CODEC_DAI(5),
	ADMAIF_CODEC_DAI(6),
	ADMAIF_CODEC_DAI(7),
	ADMAIF_CODEC_DAI(8),
	ADMAIF_CODEC_DAI(9),
	ADMAIF_CODEC_DAI(10),
};

#define ADMAIF_WIDGETS(id)					\
	SND_SOC_DAPM_AIF_IN_E("ADMAIF" #id " FIFO RX", NULL, 0,	\
		SND_SOC_NOPM, 0, 0,				\
		tegra210_admaif_enable,				\
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),	\
	SND_SOC_DAPM_AIF_OUT_E("ADMAIF" #id " FIFO TX", NULL, 0,\
		SND_SOC_NOPM, 0, 0,				\
		tegra210_admaif_enable,				\
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),	\
	SND_SOC_DAPM_AIF_IN("ADMAIF" #id " CIF RX", NULL, 0,	\
		SND_SOC_NOPM, 0, 0),				\
	SND_SOC_DAPM_AIF_OUT("ADMAIF" #id " CIF TX", NULL, 0,	\
		SND_SOC_NOPM, 0, 0)

static const struct snd_soc_dapm_widget tegra210_admaif_widgets[] = {
	ADMAIF_WIDGETS(1),
	ADMAIF_WIDGETS(2),
	ADMAIF_WIDGETS(3),
	ADMAIF_WIDGETS(4),
	ADMAIF_WIDGETS(5),
	ADMAIF_WIDGETS(6),
	ADMAIF_WIDGETS(7),
	ADMAIF_WIDGETS(8),
	ADMAIF_WIDGETS(9),
	ADMAIF_WIDGETS(10)
};

#define ADMAIF_ROUTES(id)						\
	{ "ADMAIF" #id " FIFO RX",      NULL, "ADMAIF" #id " FIFO Transmit" }, \
	{ "ADMAIF" #id " CIF TX",       NULL, "ADMAIF" #id " FIFO RX" },\
	{ "ADMAIF" #id " CIF Receive",  NULL, "ADMAIF" #id " CIF TX" }, \
	{ "ADMAIF" #id " CIF RX",       NULL, "ADMAIF" #id " CIF Transmit" },  \
	{ "ADMAIF" #id " FIFO TX",      NULL, "ADMAIF" #id " CIF RX" }, \
	{ "ADMAIF" #id " FIFO Receive", NULL, "ADMAIF" #id " FIFO TX" } \

static const struct snd_soc_dapm_route tegra210_admaif_routes[] = {
	ADMAIF_ROUTES(1),
	ADMAIF_ROUTES(2),
	ADMAIF_ROUTES(3),
	ADMAIF_ROUTES(4),
	ADMAIF_ROUTES(5),
	ADMAIF_ROUTES(6),
	ADMAIF_ROUTES(7),
	ADMAIF_ROUTES(8),
	ADMAIF_ROUTES(9),
	ADMAIF_ROUTES(10)
};

static int tegra210_admaif_codec_probe(struct snd_soc_codec *codec)
{
	struct tegra210_admaif *admaif = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = admaif->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_codec_driver tegra210_admaif_codec = {
	.probe = tegra210_admaif_codec_probe,
	.dapm_widgets = tegra210_admaif_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra210_admaif_widgets),
	.dapm_routes = tegra210_admaif_routes,
	.num_dapm_routes = ARRAY_SIZE(tegra210_admaif_routes),
	.idle_bias_off = 1,
};

static const struct snd_soc_component_driver tegra210_admaif_dai_driver = {
	.name		= DRV_NAME,
};

static struct tegra210_admaif_soc_data soc_data_tegra210 = {
	.num_ch = 10,
	.set_audio_cif = tegra210_xbar_set_cif,
};

static const struct of_device_id tegra210_admaif_of_match[] = {
	{ .compatible = "nvidia,tegra210-admaif", .data = &soc_data_tegra210 },
	{},
};

static int tegra210_admaif_probe(struct platform_device *pdev)
{
	int i;

	int ret;
	struct tegra210_admaif *admaif;
	void __iomem *regs;
	struct resource *res;
	const struct of_device_id *match;
	struct tegra210_admaif_soc_data *soc_data;

	match = of_match_device(tegra210_admaif_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}
	soc_data = (struct tegra210_admaif_soc_data *)match->data;

	admaif = devm_kzalloc(&pdev->dev, sizeof(*admaif), GFP_KERNEL);
	if (!admaif) {
		dev_err(&pdev->dev, "Can't allocate tegra210_admaif\n");
		ret = -ENOMEM;
		goto err;
	}

	dev_set_drvdata(&pdev->dev, admaif);

	admaif->refcnt = 0;

	admaif->soc_data = soc_data;

	admaif->capture_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				admaif->soc_data->num_ch,
			GFP_KERNEL);
	if (!admaif->capture_dma_data) {
		dev_err(&pdev->dev, "Can't allocate tegra_alt_pcm_dma_params\n");
		ret = -ENOMEM;
		goto err;
	}

	admaif->playback_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				admaif->soc_data->num_ch,
			GFP_KERNEL);
	if (!admaif->playback_dma_data) {
		dev_err(&pdev->dev, "Can't allocate tegra_alt_pcm_dma_params\n");
		ret = -ENOMEM;
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory resource for admaif\n");
		ret = -ENODEV;
		goto err;
	}

	regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!regs) {
		dev_err(&pdev->dev, "request/iomap region failed\n");
		ret = -ENODEV;
		goto err;
	}

	admaif->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					&tegra210_admaif_regmap_config);
	if (IS_ERR(admaif->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(admaif->regmap);
		goto err;
	}
	regcache_cache_only(admaif->regmap, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_admaif_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	for (i = 0; i < admaif->soc_data->num_ch; i++) {
		admaif->playback_dma_data[i].addr = res->start +
				TEGRA210_ADMAIF_XBAR_TX_FIFO_WRITE +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);

		admaif->capture_dma_data[i].addr = res->start +
				TEGRA210_ADMAIF_XBAR_RX_FIFO_READ +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);

		admaif->playback_dma_data[i].wrap = 4;
		admaif->playback_dma_data[i].width = 32;
		admaif->playback_dma_data[i].req_sel = i + 1;
		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(i * 2) + 1,
				&admaif->playback_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
			ret = -ENODEV;
			goto err_suspend;
		}

		admaif->capture_dma_data[i].wrap = 4;
		admaif->capture_dma_data[i].width = 32;
		admaif->capture_dma_data[i].req_sel = i + 1;
		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(i * 2),
				&admaif->capture_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
			ret = -ENODEV;
			goto err_suspend;
		}
	}

	ret = snd_soc_register_component(&pdev->dev,
					&tegra210_admaif_dai_driver,
					tegra210_admaif_dais,
					ARRAY_SIZE(tegra210_admaif_dais));
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAIs %d: %d\n",
			i, ret);
		ret = -ENOMEM;
		goto err_suspend;
	}

	ret = snd_soc_register_codec(&pdev->dev, &tegra210_admaif_codec,
				tegra210_admaif_codec_dais,
				ARRAY_SIZE(tegra210_admaif_codec_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err_unregister_dais;
	}

	ret = tegra_alt_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_codec;
	}

	return 0;

err_unregister_codec:
	snd_soc_unregister_codec(&pdev->dev);
err_unregister_dais:
	snd_soc_unregister_component(&pdev->dev);
err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_admaif_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err:
	return ret;
}

static int tegra210_admaif_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);

	tegra_alt_pcm_platform_unregister(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_admaif_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops tegra210_admaif_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_admaif_runtime_suspend,
			   tegra210_admaif_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(tegra210_admaif_suspend, NULL)
};

static struct platform_driver tegra210_admaif_driver = {
	.probe = tegra210_admaif_probe,
	.remove = tegra210_admaif_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_admaif_of_match,
		.pm = &tegra210_admaif_pm_ops,
	},
};
module_platform_driver(tegra210_admaif_driver);

MODULE_AUTHOR("Songhee Baek <sbaek@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 ADMAIF driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
