/*
 * tegra_grenada.c - Tegra VCM30 T124 Machine driver
 *
 * Copyright (c) 2013-2014 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/wm8731.h"
#include "../codecs/ad193x.h"

#include "tegra_asoc_utils_alt.h"
#include "ahub_unit_fpga_clock.h"

#define DRV_NAME "tegra-snd-grenada"
struct tegra_grenada {
	struct tegra_asoc_audio_clock_info audio_clock;
};

static int tegra_grenada_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return 0;
}

static int tegra_grenada_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void tegra_grenada_shutdown(struct snd_pcm_substream *substream)
{
	return;
}

static int tegra_grenada_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct snd_soc_dai *i2s_dai = card->rtd[10].cpu_dai;
	unsigned int srate;
	int err;
	AD1937_EXTRA_INFO ad1937_info;

	srate = 48000;

	err = snd_soc_dai_set_sysclk(i2s_dai, 0, srate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "i2s clock not set\n");
		return err;
	}

#if SYSTEM_FPGA
	i2c_pinmux_setup();
	i2c_clk_setup(10 - 1);
	program_cdc_pll(2, CLK_OUT_12_2888_MHZ);
	program_io_expander();
	i2s_clk_setup(I2S1, 0, 23);
	i2s_clk_setup(I2S2, 0, 0);
	i2s_clk_setup(I2S5, 0, 3);

	i2s_pinmux_setup(I2S1, 0);
	i2s_pinmux_setup(I2S2, 0);
	i2s_pinmux_setup(I2S5, 1);

#else
	/* PLL, I2S and Codec programming */
	program_cdc_pll(2, CLK_OUT_12_2888_MHZ);
	program_io_expander();
	i2c_clk_divider(10 - 1);

	/* I2S1: 23 for 8Khz, 3 for 48Khz */
	i2s_clk_divider(I2S1, 23);
	i2s_clk_divider(I2S2, 0);
	i2s_clk_divider(I2S5, 3);
#endif
	program_max_codec();
	ad1937_info.codecId = AD1937_X_ADDRESS;
	ad1937_info.clkgenId = CLK_OUT_FROM_TEGRA;
	ad1937_info.dacMasterEn = AUDIO_DAC_SLAVE_MODE;
	ad1937_info.daisyEn = 0;
	OnAD1937CaptureAndPlayback(AUDIO_CODEC_SLAVE_MODE,
				AUDIO_INTERFACE_TDM_FORMAT,
				I2S_DATAWIDTH_16,
				64, 0, 0,
				AUDIO_SAMPLE_RATE_48_00,
				&ad1937_info);

	ad1937_info.codecId = AD1937_Y_ADDRESS;
	ad1937_info.clkgenId = CLK_OUT_FROM_TEGRA;
	ad1937_info.dacMasterEn = AUDIO_DAC_SLAVE_MODE;
	ad1937_info.daisyEn = 0;
	OnAD1937CaptureAndPlayback(AUDIO_CODEC_SLAVE_MODE,
				AUDIO_INTERFACE_I2S_FORMAT,
				I2S_DATAWIDTH_16,
				256, 0, 0,
				AUDIO_SAMPLE_RATE_48_00,
				&ad1937_info);
	return 0;
}

static int tegra_grenada_spdif_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	unsigned int mclk, srate;
	int err;

	srate = 48000;
	mclk = srate * 512;

	err = snd_soc_dai_set_sysclk(card->rtd[61].cpu_dai, 0, srate,
					SND_SOC_CLOCK_OUT);
	err = snd_soc_dai_set_sysclk(card->rtd[61].cpu_dai, 0, srate,
					SND_SOC_CLOCK_IN);
	return 0;
}

static int tegra_grenada_amx_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct snd_soc_dai *amx_dai = card->rtd[20].cpu_dai;
	unsigned int tx_slot[32], i, j;

	for (i = 0, j = 0; i < 32; i += 8) {
		tx_slot[i] = 0;
		tx_slot[i + 1] = 0;
		tx_slot[i + 2] = (j << 16) | (1 << 8) | 0;
		tx_slot[i + 3] = (j << 16) | (1 << 8) | 1;
		tx_slot[i + 4] = 0;
		tx_slot[i + 5] = 0;
		tx_slot[i + 6] = (j << 16) | (2 << 8) | 0;
		tx_slot[i + 7] = (j << 16) | (2 << 8) | 1;
		j++;
	}

	if (amx_dai->driver->ops->set_channel_map)
		amx_dai->driver->ops->set_channel_map(amx_dai,
							32, tx_slot, 0, 0);

	return 0;
}

static int tegra_grenada_adx_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct snd_soc_dai *adx_dai = card->rtd[21].codec_dai;
	unsigned int rx_slot[32], i, j;

	for (i = 0, j = 0; i < 32; i += 8) {
		rx_slot[i] = 0;
		rx_slot[i + 1] = 0;
		rx_slot[i + 2] = (j << 16) | (1 << 8) | 0;
		rx_slot[i + 3] = (j << 16) | (1 << 8) | 1;
		rx_slot[i + 4] = 0;
		rx_slot[i + 5] = 0;
		rx_slot[i + 6] = (j << 16) | (2 << 8) | 0;
		rx_slot[i + 7] = (j << 16) | (2 << 8) | 1;
		j++;
	}

	if (adx_dai->driver->ops->set_channel_map)
		adx_dai->driver->ops->set_channel_map(adx_dai,
							0, 0, 32, rx_slot);

	return 0;
}

static int tegra_grenada_sfc_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int in_srate, out_srate;
	int err;

	in_srate = 48000;
	out_srate = 8000;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, out_srate,
					SND_SOC_CLOCK_OUT);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, in_srate,
					SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops tegra_grenada_ops = {
	.hw_params = tegra_grenada_hw_params,
	.startup = tegra_grenada_startup,
	.shutdown = tegra_grenada_shutdown,
};

static const struct snd_soc_dapm_widget tegra_grenada_dapm_widgets[] = {
	SND_SOC_DAPM_HP("x Headphone", NULL),
	SND_SOC_DAPM_HP("y Headphone", NULL),
	SND_SOC_DAPM_HP("z Headphone", NULL),
	SND_SOC_DAPM_HP("s Headphone", NULL),
	SND_SOC_DAPM_MIC("x Mic", NULL),
	SND_SOC_DAPM_MIC("y Mic", NULL),
	SND_SOC_DAPM_MIC("z Mic", NULL),
	SND_SOC_DAPM_MIC("s Mic", NULL),
};

static const struct snd_soc_dapm_route tegra_grenada_audio_map[] = {
};

static int tegra_grenada_remove(struct snd_soc_card *card)
{
	return 0;
}

static const struct snd_soc_pcm_stream tdm_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 8,
	.channels_max = 8,
};

static const struct snd_soc_pcm_stream amx_adx_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream i2s_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_dai_link tegra_grenada_links[] = {
	{
		/* 0 */
		.name = "ADMAIF1 CIF",
		.stream_name = "ADMAIF1 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF1",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF1",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 1 */
		.name = "ADMAIF2 CIF",
		.stream_name = "ADMAIF2 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF2",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF2",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 2 */
		.name = "ADMAIF3 CIF",
		.stream_name = "ADMAIF3 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF3",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF3",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 3 */
		.name = "ADMAIF4 CIF",
		.stream_name = "ADMAIF4 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF4",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF4",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 4 */
		.name = "ADMAIF5 CIF",
		.stream_name = "ADMAIF5 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF5",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF5",
		.ops = &tegra_grenada_ops,
	},

	{
		/* 5 */
		.name = "ADMAIF6 CIF",
		.stream_name = "ADMAIF6 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF6",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF6",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 6 */
		.name = "ADMAIF7 CIF",
		.stream_name = "ADMAIF7 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF7",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF7",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 7 */
		.name = "ADMAIF8 CIF",
		.stream_name = "ADMAIF8 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF8",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF8",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 8 */
		.name = "ADMAIF9 CIF",
		.stream_name = "ADMAIF9 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF9",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF9",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 9 */
		.name = "ADMAIF10 CIF",
		.stream_name = "ADMAIF10 CIF",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF10",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF10",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 10 */
		.name = "x spdif-dit",
		.stream_name = "Playback",
		/* .cpu_of_node = I2S1 */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = SPDIF dummy */
		.codec_dai_name = "dit-hifi",
		.init = tegra_grenada_init,
		.params = &i2s_link_params,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
	},
	{
		/* 11 */
		.name = "I2S1 CIF",
		.stream_name = "I2S1 CIF",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "I2S1",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 12 */
		.name = "y spdif-dit",
		.stream_name = "Playback",
		/* .cpu_of_node = I2S2 */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = SPDIF dummy */
		.codec_dai_name = "dit-hifi",
		.params = &tdm_link_params,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
	},
	{
		/* 13 */
		.name = "I2S2 CIF",
		.stream_name = "I2S2 CIF",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "I2S2",
		/* .codec_of_node = I2S2 */
		.codec_dai_name = "CIF",
		.params = &tdm_link_params,
	},
	{
		/* 14 */
		.name = "z spdif-dit",
		.stream_name = "Playback",
		/* .cpu_of_node = I2S5 */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = SPDIF dummy */
		.codec_dai_name = "dit-hifi",
		.params = &i2s_link_params,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
	},
	{
		/* 15 */
		.name = "I2S5 CIF",
		.stream_name = "I2S5 CIF",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "I2S5",
		/* .codec_of_node = I2S5 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 16 */
		.name = "AMX1 IN1",
		.stream_name = "AMX1 IN",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AMX1-1",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "IN1",
		.params = &amx_adx_link_params,
	},
	{
		/* 17 */
		.name = "AMX1 IN2",
		.stream_name = "AMX1 IN",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AMX1-2",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "IN2",
		.params = &amx_adx_link_params,
	},
	{
		/* 18 */
		.name = "AMX1 IN3",
		.stream_name = "AMX1 IN",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AMX1-3",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "IN3",
		.params = &amx_adx_link_params,
	},
	{
		/* 19 */
		.name = "AMX1 IN4",
		.stream_name = "AMX1 IN",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AMX1-4",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "IN4",
		.params = &amx_adx_link_params,
	},
	{
		/* 20 */
		.name = "AMX1 CIF",
		.stream_name = "AMX1 CIF",
		/* .cpu_of_node = AMX1 OUT */
		.cpu_dai_name = "OUT",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "AMX1",
		.init = tegra_grenada_amx_dai_init,
		.params = &tdm_link_params,
	},
	{
		/* 21 */
		.name = "ADX1 CIF",
		.stream_name = "ADX1 IN",
		.cpu_dai_name = "ADX1",
		.codec_dai_name = "IN",
		.init = tegra_grenada_adx_dai_init,
		.params = &tdm_link_params,
	},
	{
		/* 22 */
		.name = "ADX1 OUT1",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX1-1",
		.params = &amx_adx_link_params,
	},
	{
		/* 23 */
		.name = "ADX1 OUT2",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX1-2",
		.params = &amx_adx_link_params,
	},
	{
		/* 24 */
		.name = "ADX1 OUT3",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX1-3",
		.params = &amx_adx_link_params,
	},
	{
		/* 25 */
		.name = "ADX1 OUT4",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX1-4",
		.params = &amx_adx_link_params,
	},
	{
		/* 26 */
		.name = "MIXER1 RX1",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-1",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX1",
		.params = &amx_adx_link_params,
	},
	{
		/* 27 */
		.name = "MIXER1 RX2",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-2",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX2",
		.params = &amx_adx_link_params,
	},
	{
		/* 28 */
		.name = "MIXER1 RX3",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-3",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX3",
		.params = &amx_adx_link_params,
	},
	{
		/* 29 */
		.name = "MIXER1 RX4",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-4",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX4",
		.params = &amx_adx_link_params,
	},
	{
		/* 30 */
		.name = "MIXER1 RX5",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-5",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX5",
		.params = &amx_adx_link_params,
	},
	{
		/* 31 */
		.name = "MIXER1 RX6",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-6",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX6",
		.params = &amx_adx_link_params,
	},
	{
		/* 32 */
		.name = "MIXER1 RX7",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-7",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX7",
		.params = &amx_adx_link_params,
	},
	{
		/* 33 */
		.name = "MIXER1 RX8",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-8",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX8",
		.params = &amx_adx_link_params,
	},
	{
		/* 34 */
		.name = "MIXER1 RX9",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-9",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX9",
		.params = &amx_adx_link_params,
	},
	{
		/* 35 */
		.name = "MIXER1 RX10",
		.stream_name = "MIXER1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MIXER1-10",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "RX10",
		.params = &amx_adx_link_params,
	},
	{
		/* 36 */
		.name = "MIXER1 TX1",
		.stream_name = "MIXER1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "TX1",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "MIXER1-1",
		.params = &amx_adx_link_params,
	},
	{
		/* 37 */
		.name = "MIXER1 TX2",
		.stream_name = "MIXER1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "TX2",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "MIXER1-2",
		.params = &amx_adx_link_params,
	},
	{
		/* 38 */
		.name = "MIXER1 TX3",
		.stream_name = "MIXER1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "TX3",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "MIXER1-3",
		.params = &amx_adx_link_params,
	},
	{
		/* 39 */
		.name = "MIXER1 TX4",
		.stream_name = "MIXER1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "TX4",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "MIXER1-4",
		.params = &amx_adx_link_params,
	},
	{
		/* 40 */
		.name = "MIXER1 TX5",
		.stream_name = "MIXER1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "TX5",
		/* .codec_of_node = AMX1 */
		.codec_dai_name = "MIXER1-5",
		.params = &amx_adx_link_params,
	},
	{
		/* 41 */
		.name = "SFC1 RX",
		.stream_name = "SFC1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "SFC1",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
		.init = tegra_grenada_sfc_init,
	},
	{
		/* 42 */
		.name = "SFC1 TX",
		.stream_name = "SFC1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "SFC1",
		.params = &i2s_link_params,
	},
	{
		/* 43 */
		.name = "SFC2 RX",
		.stream_name = "SFC2 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "SFC2",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 44 */
		.name = "SFC2 TX",
		.stream_name = "SFC2 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "SFC2",
		.params = &i2s_link_params,
	},
	{
		/* 45 */
		.name = "SFC3 RX",
		.stream_name = "SFC3 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "SFC3",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 46 */
		.name = "SFC3 TX",
		.stream_name = "SFC3 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "SFC3",
		.params = &i2s_link_params,
	},
	{
		/* 47 */
		.name = "SFC4 RX",
		.stream_name = "SFC4 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "SFC4",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 48 */
		.name = "SFC4 TX",
		.stream_name = "SFC4 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "SFC4",
		.params = &i2s_link_params,
	},
	{
		/* 49 */
		.name = "AFC1 RX",
		.stream_name = "AFC1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC1",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC IN",
		.params = &i2s_link_params,
	},
	{
		/* 50 */
		.name = "AFC1 TX",
		.stream_name = "AFC1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC1",
		.params = &i2s_link_params,
	},
	{
		/* 51 */
		.name = "AFC2 RX",
		.stream_name = "AFC2 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC2",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC IN",
		.params = &i2s_link_params,
	},
	{
		/* 52 */
		.name = "AFC2 TX",
		.stream_name = "AFC2 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC2",
		.params = &i2s_link_params,
	},
	{
		/* 53 */
		.name = "AFC3 RX",
		.stream_name = "AFC3 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC3",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC IN",
		.params = &i2s_link_params,
	},
	{
		/* 54 */
		.name = "AFC3 TX",
		.stream_name = "AFC3 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC3",
		.params = &i2s_link_params,
	},
	{
		/* 55 */
		.name = "AFC4 RX",
		.stream_name = "AFC4 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC4",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC IN",
		.params = &i2s_link_params,
	},
	{
		/* 56 */
		.name = "AFC4 TX",
		.stream_name = "AFC4 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC4",
		.params = &i2s_link_params,
	},
	{
		/* 57 */
		.name = "AFC5 RX",
		.stream_name = "AFC5 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC5",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC IN",
		.params = &i2s_link_params,
	},
	{
		/* 58 */
		.name = "AFC5 TX",
		.stream_name = "AFC5 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC5",
		.params = &i2s_link_params,
	},
	{
		/* 59 */
		.name = "AFC6 RX",
		.stream_name = "AFC6 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC6",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC IN",
		.params = &i2s_link_params,
	},
	{
		/* 60 */
		.name = "AFC6 TX",
		.stream_name = "AFC6 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "AFC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "AFC6",
		.params = &i2s_link_params,
	},
	{
		/* 61 */
		.name = "s spdif-dit",
		.stream_name = "playback",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "dit-hifi",
		.init = tegra_grenada_spdif_init,
		.params = &i2s_link_params,
	},
	{
		/* 62 */
		.name = "SPDIF1-1 CIF",
		.stream_name = "SPDIF1-1 CIF",
		.cpu_dai_name = "SPDIF1-1",
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 63 */
		.name = "MVC1 RX",
		.stream_name = "MVC1 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MVC1",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "MVC IN",
		.params = &i2s_link_params,
	},
	{
		/* 64 */
		.name = "MVC1 TX",
		.stream_name = "MVC1 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MVC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "MVC1",
		.params = &i2s_link_params,
	},
	{
		/* 65 */
		.name = "MVC2 RX",
		.stream_name = "MVC2 RX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MVC2",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "MVC IN",
		.params = &i2s_link_params,
	},
	{
		/* 66 */
		.name = "MVC2 TX",
		.stream_name = "AFC2 TX",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "MVC OUT",
		/* .codec_of_node = I2S1 */
		.codec_dai_name = "MVC2",
		.params = &i2s_link_params,
	},
	{
		/* 67 */
		.name = "ADMAIF1 CODEC",
		.stream_name = "ADMAIF1 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF1 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF1",
		.params = &i2s_link_params,
	},
	{
		/* 68 */
		.name = "ADMAIF2 CODEC",
		.stream_name = "ADMAIF2 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF2 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF2",
		.params = &i2s_link_params,
	},
	{
		/* 69 */
		.name = "ADMAIF3 CODEC",
		.stream_name = "ADMAIF3 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF3 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF3",
		.params = &i2s_link_params,
	},
	{
		/* 70 */
		.name = "ADMAIF4 CODEC",
		.stream_name = "ADMAIF4 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF4 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF4",
		.params = &i2s_link_params,
	},
	{
		/* 71 */
		.name = "ADMAIF5 CODEC",
		.stream_name = "ADMAIF5 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF5 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF5",
		.params = &i2s_link_params,
	},
	{
		/* 72 */
		.name = "ADMAIF6 CODEC",
		.stream_name = "ADMAIF6 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF6 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF6",
		.params = &i2s_link_params,
	},
	{
		/* 73 */
		.name = "ADMAIF7 CODEC",
		.stream_name = "ADMAIF7 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF7 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF7",
		.params = &i2s_link_params,
	},
	{
		/* 74 */
		.name = "ADMAIF8 CODEC",
		.stream_name = "ADMAIF8 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF8 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF8",
		.params = &i2s_link_params,
	},
	{
		/* 75 */
		.name = "ADMAIF9 CODEC",
		.stream_name = "ADMAIF9 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF9 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF9",
		.params = &i2s_link_params,
	},
	{
		/* 76 */
		.name = "ADMAIF10 CODEC",
		.stream_name = "ADMAIF10 CODEC",
		/* .cpu_of_node = AHUB ADMAIF */
		.cpu_dai_name = "ADMAIF10 CIF",
		/* .codec_of_node = AHUB XBAR */
		.codec_dai_name = "ADMAIF10",
		.params = &i2s_link_params,
	},
	{
		/* 77 */
		.name = "ADSP ADMAIF1",
		.stream_name = "ADSP ADMAIF1",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF1",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF1 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 78 */
		.name = "ADSP ADMAIF2",
		.stream_name = "ADSP ADMAIF2",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF2",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF2 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 79 */
		.name = "ADSP ADMAIF3",
		.stream_name = "ADSP ADMAIF3",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF3",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF3 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 80 */
		.name = "ADSP ADMAIF4",
		.stream_name = "ADSP ADMAIF4",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF4",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF4 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 81 */
		.name = "ADSP ADMAIF5",
		.stream_name = "ADSP ADMAIF5",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF5",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF5 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 82 */
		.name = "ADSP ADMAIF6",
		.stream_name = "ADSP ADMAIF6",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF6",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF6 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 83 */
		.name = "ADSP ADMAIF7",
		.stream_name = "ADSP ADMAIF7",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF7",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF7 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 84 */
		.name = "ADSP ADMAIF8",
		.stream_name = "ADSP ADMAIF8",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF8",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF8 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 85 */
		.name = "ADSP ADMAIF9",
		.stream_name = "ADSP ADMAIF9",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF9",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF9 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 86 */
		.name = "ADSP ADMAIF10",
		.stream_name = "ADSP ADMAIF10",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP-ADMAIF10",
		/* .codec_of_node = AHUB ADMAIF */
		.codec_dai_name = "ADMAIF10 FIFO",
		.params = &i2s_link_params,
	},
	{
		/* 87 */
		.name = "ADSP PCM",
		.stream_name = "ADSP PCM",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP PCM",
		/* .codec_of_node = ADSP */
		.codec_dai_name = "ADSP-FE1",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 88 */
		.name = "ADSP COMPR1",
		.stream_name = "ADSP COMPR1",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP COMPR1",
		/* .codec_of_node = ADSP */
		.codec_dai_name = "ADSP-FE2",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 89 */
		.name = "ADSP COMPR2",
		.stream_name = "ADSP COMPR2",
		/* .cpu_of_node = ADSP */
		.cpu_dai_name = "ADSP COMPR2",
		/* .codec_of_node = ADSP */
		.codec_dai_name = "ADSP-FE3",
		.ops = &tegra_grenada_ops,
	},
	{
		/* 90 */
		.name = "dmic-x spdif-dit",
		.stream_name = "Playback",
		/* .cpu_of_node = DMIC1 */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = SPDIF dummy */
		.codec_dai_name = "dit-hifi",
		.params = &i2s_link_params,
	},
	{
		/* 91 */
		.name = "DMIC1 CIF",
		.stream_name = "DMIC1 CIF",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DMIC1",
		/* .codec_of_node = DMIC1 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 92 */
		.name = "dmic-y spdif-dit",
		.stream_name = "Playback",
		/* .cpu_of_node = DMIC2 */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = SPDIF dummy */
		.codec_dai_name = "dit-hifi",
		.params = &i2s_link_params,
	},
	{
		/* 93 */
		.name = "DMIC2 CIF",
		.stream_name = "DMIC2 CIF",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DMIC2",
		/* .codec_of_node = DMIC2 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
	{
		/* 94 */
		.name = "dmic-z spdif-dit",
		.stream_name = "Playback",
		/* .cpu_of_node = DMIC3 */
		.cpu_dai_name = "DAP",
		/* .codec_of_node = SPDIF dummy */
		.codec_dai_name = "dit-hifi",
		.params = &i2s_link_params,
	},
	{
		/* 95 */
		.name = "DMIC3 CIF",
		.stream_name = "DMIC3 CIF",
		/* .cpu_of_node = AHUB XBAR */
		.cpu_dai_name = "DMIC3",
		/* .codec_of_node = DMIC3 */
		.codec_dai_name = "CIF",
		.params = &i2s_link_params,
	},
};

static struct snd_soc_codec_conf ad193x_codec_conf[] = {
	{
		.dev_name = "spdif-dit.0",
		.name_prefix = "x",
	},
	{
		.dev_name = "spdif-dit.1",
		.name_prefix = "y",
	},
	{
		.dev_name = "spdif-dit.2",
		.name_prefix = "z",
	},
	{
		.dev_name = "spdif-dit.3",
		.name_prefix = "s",
	},
	{
		.dev_name = "tegra210-i2s.0",
		.name_prefix = "I2S1",
	},
	{
		.dev_name = "tegra210-i2s.1",
		.name_prefix = "I2S2",
	},
	{
		.dev_name = "tegra210-i2s.4",
		.name_prefix = "I2S5",
	},
	{
		.dev_name = "tegra210-sfc.0",
		.name_prefix = "SFC1",
	},
	{
		.dev_name = "tegra210-sfc.1",
		.name_prefix = "SFC2",
	},
	{
		.dev_name = "tegra210-sfc.2",
		.name_prefix = "SFC3",
	},
	{
		.dev_name = "tegra210-sfc.3",
		.name_prefix = "SFC4",
	},
	{
		.dev_name = "tegra210-afc.0",
		.name_prefix = "AFC1",
	},
	{
		.dev_name = "tegra210-afc.1",
		.name_prefix = "AFC2",
	},
	{
		.dev_name = "tegra210-afc.2",
		.name_prefix = "AFC3",
	},
	{
		.dev_name = "tegra210-afc.3",
		.name_prefix = "AFC4",
	},
	{
		.dev_name = "tegra210-afc.4",
		.name_prefix = "AFC5",
	},
	{
		.dev_name = "tegra210-afc.5",
		.name_prefix = "AFC6",
	},
	{
		.dev_name = "tegra210-mvc.0",
		.name_prefix = "MVC1",
	},
	{
		.dev_name = "tegra210-mvc.1",
		.name_prefix = "MVC2",
	},
};

static struct snd_soc_card snd_soc_tegra_grenada = {
	.name = "tegra-grenada",
	.owner = THIS_MODULE,
	.dai_link = tegra_grenada_links,
	.num_links = ARRAY_SIZE(tegra_grenada_links),
	.remove = tegra_grenada_remove,
	.dapm_widgets = tegra_grenada_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_grenada_dapm_widgets),
	.codec_conf = ad193x_codec_conf,
	.num_configs = ARRAY_SIZE(ad193x_codec_conf),
	.fully_routed = true,
};

static int tegra_grenada_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_grenada;
	struct tegra_grenada *machine;
	int ret, i;

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_grenada),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_grenada struct\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	if (np) {
		ret = snd_soc_of_parse_card_name(card, "nvidia,model");
		if (ret)
			goto err;

		/* APBIF dai links */
		tegra_grenada_links[0].cpu_of_node = of_parse_phandle(np,
					"nvidia,admaif", 0);
		if (!tegra_grenada_links[0].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,admaif' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		tegra_grenada_links[0].platform_of_node =
				tegra_grenada_links[0].cpu_of_node;

		tegra_grenada_links[0].codec_of_node = of_parse_phandle(np,
					"nvidia,xbar", 0);
		if (!tegra_grenada_links[0].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,xbar' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		ret = snd_soc_of_parse_audio_routing(card,
					"nvidia,audio-routing");
		if (ret)
			goto err;

		for (i = 1; i < 10; i++) {
			tegra_grenada_links[i].cpu_of_node =
				tegra_grenada_links[0].cpu_of_node;
			tegra_grenada_links[i].codec_of_node =
					tegra_grenada_links[0].codec_of_node;
			tegra_grenada_links[i].platform_of_node =
					tegra_grenada_links[0].cpu_of_node;
		}

		/* audio codec DAI */
		tegra_grenada_links[10].codec_of_node = of_parse_phandle(np,
					"nvidia,audio-codec-x", 0);
		if (!tegra_grenada_links[10].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,audio-codec-x' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[12].codec_of_node = of_parse_phandle(np,
					"nvidia,audio-codec-y", 0);
		if (!tegra_grenada_links[12].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,audio-codec-y' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[14].codec_of_node = of_parse_phandle(np,
					"nvidia,audio-codec-z", 0);
		if (!tegra_grenada_links[14].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,audio-codec-z' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		/* I2S dai links */
		tegra_grenada_links[10].cpu_of_node = of_parse_phandle(np,
					"nvidia,i2s-controller-1", 0);
		if (!tegra_grenada_links[10].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,i2s-controller-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		tegra_grenada_links[11].codec_of_node =
					tegra_grenada_links[10].cpu_of_node;
		tegra_grenada_links[11].cpu_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[12].cpu_of_node = of_parse_phandle(np,
					"nvidia,i2s-controller-2", 0);
		if (!tegra_grenada_links[12].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,i2s-controller-2' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		tegra_grenada_links[13].codec_of_node =
					tegra_grenada_links[12].cpu_of_node;
		tegra_grenada_links[13].cpu_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[14].cpu_of_node = of_parse_phandle(np,
					"nvidia,i2s-controller-3", 0);
		if (!tegra_grenada_links[14].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,i2s-controller-3' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		/* AMX dai links */
		tegra_grenada_links[15].codec_of_node =
					tegra_grenada_links[14].cpu_of_node;
		tegra_grenada_links[15].cpu_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[20].cpu_of_node = of_parse_phandle(np,
					"nvidia,amx-1", 0);
		if (!tegra_grenada_links[20].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,amx-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[20].codec_of_node = tegra_grenada_links[0].codec_of_node;

		/* ADX dai links */
		tegra_grenada_links[21].codec_of_node = of_parse_phandle(np,
					"nvidia,adx-1", 0);
		if (!tegra_grenada_links[21].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,adx-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[21].cpu_of_node = tegra_grenada_links[0].codec_of_node;


		for (i = 16; i < 20; i++) {
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[20].cpu_of_node;
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[20].codec_of_node;
		}

		for (i = 22; i < 26; i++) {
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[21].cpu_of_node;
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[21].codec_of_node;
		}

		/* MIXER dai links */
		tegra_grenada_links[36].cpu_of_node = of_parse_phandle(np,
					"nvidia,mixer-1", 0);
		if (!tegra_grenada_links[36].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,mixer-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[36].codec_of_node = tegra_grenada_links[0].codec_of_node;
		for (i = 37; i < 41; i++) {
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[36].cpu_of_node;
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[36].codec_of_node;
		}

		for (i = 26; i < 36; i++) {
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[36].cpu_of_node;
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[36].codec_of_node;
		}

		/* SFC dai links */
		tegra_grenada_links[42].cpu_of_node = of_parse_phandle(np,
					"nvidia,sfc-1", 0);
		if (!tegra_grenada_links[42].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,sfc-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[42].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[44].cpu_of_node = of_parse_phandle(np,
					"nvidia,sfc-2", 0);
		if (!tegra_grenada_links[44].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,sfc-2' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[44].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[46].cpu_of_node = of_parse_phandle(np,
					"nvidia,sfc-3", 0);
		if (!tegra_grenada_links[46].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,sfc-3' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[46].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[48].cpu_of_node = of_parse_phandle(np,
					"nvidia,sfc-4", 0);
		if (!tegra_grenada_links[48].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,sfc-4' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[48].codec_of_node = tegra_grenada_links[0].codec_of_node;

		for (i = 41; i < 48; i = i+2) {
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[i+1].cpu_of_node;
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[i+1].codec_of_node;
		}

		/* AFC dai links */
		tegra_grenada_links[50].cpu_of_node = of_parse_phandle(np,
					"nvidia,afc-1", 0);
		if (!tegra_grenada_links[50].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,afc-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[50].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[52].cpu_of_node = of_parse_phandle(np,
					"nvidia,afc-2", 0);
		if (!tegra_grenada_links[52].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,afc-2' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[52].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[54].cpu_of_node = of_parse_phandle(np,
					"nvidia,afc-3", 0);
		if (!tegra_grenada_links[54].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,afc-3' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[54].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[56].cpu_of_node = of_parse_phandle(np,
					"nvidia,afc-4", 0);
		if (!tegra_grenada_links[56].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,afc-4' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[56].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[58].cpu_of_node = of_parse_phandle(np,
					"nvidia,afc-5", 0);
		if (!tegra_grenada_links[58].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,afc-5' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[58].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[60].cpu_of_node = of_parse_phandle(np,
					"nvidia,afc-6", 0);
		if (!tegra_grenada_links[60].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,afc-6' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[60].codec_of_node = tegra_grenada_links[0].codec_of_node;

		for (i = 49; i < 60; i = i+2) {
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[i+1].cpu_of_node;
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[i+1].codec_of_node;
		}

		/* SPDIF dai links */
		tegra_grenada_links[61].codec_of_node = of_parse_phandle(np,
					"nvidia,audio-codec-s", 0);
		if (!tegra_grenada_links[61].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,audio-codec-s' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}

		tegra_grenada_links[61].cpu_of_node = of_parse_phandle(np,
					"nvidia,spdif", 0);
		if (!tegra_grenada_links[61].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,spdif' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[62].codec_of_node =
					tegra_grenada_links[61].cpu_of_node;
		tegra_grenada_links[62].cpu_of_node = tegra_grenada_links[0].codec_of_node;

		/* MVC dai links */
		tegra_grenada_links[64].cpu_of_node = of_parse_phandle(np,
					"nvidia,mvc-1", 0);
		if (!tegra_grenada_links[64].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,mvc-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[64].codec_of_node = tegra_grenada_links[0].codec_of_node;

		tegra_grenada_links[66].cpu_of_node = of_parse_phandle(np,
					"nvidia,mvc-2", 0);
		if (!tegra_grenada_links[66].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,mvc-2' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[66].codec_of_node = tegra_grenada_links[0].codec_of_node;

		for (i = 63; i < 66; i = i+2) {
			tegra_grenada_links[i].codec_of_node = tegra_grenada_links[i+1].cpu_of_node;
			tegra_grenada_links[i].cpu_of_node = tegra_grenada_links[i+1].codec_of_node;
		}

		/* ADMAIF Codec dai links */
		for (i = 67; i < 77; i++) {
			tegra_grenada_links[i].cpu_of_node =
				tegra_grenada_links[0].cpu_of_node;
			tegra_grenada_links[i].codec_of_node =
				tegra_grenada_links[0].codec_of_node;
		}

		/* ADSP-ADMAIF dai links */
		tegra_grenada_links[77].cpu_of_node = of_parse_phandle(np,
					"nvidia,adsp", 0);
		if (!tegra_grenada_links[77].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,adsp' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[77].codec_of_node =
			tegra_grenada_links[67].cpu_of_node;

		for (i = 78; i < 87; i++) {
			tegra_grenada_links[i].cpu_of_node =
				tegra_grenada_links[77].cpu_of_node;
			tegra_grenada_links[i].codec_of_node =
				tegra_grenada_links[77].codec_of_node;
		}

		/* ADSP CPU Dai links*/
		for (i = 87; i < 90; i++) {
			tegra_grenada_links[i].cpu_of_node =
				tegra_grenada_links[77].cpu_of_node;
			tegra_grenada_links[i].codec_of_node =
				tegra_grenada_links[i].cpu_of_node;
			tegra_grenada_links[i].platform_of_node =
				tegra_grenada_links[i].cpu_of_node;
		}

		/* DMIC dai links */
		tegra_grenada_links[90].codec_of_node = tegra_grenada_links[10].codec_of_node;
		tegra_grenada_links[90].cpu_of_node = of_parse_phandle(np,
					"nvidia,dmic-1", 0);
		if (!tegra_grenada_links[90].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,dmic-1' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[91].cpu_of_node = tegra_grenada_links[0].codec_of_node;
		tegra_grenada_links[91].codec_of_node = tegra_grenada_links[90].cpu_of_node;

		tegra_grenada_links[92].codec_of_node = tegra_grenada_links[12].codec_of_node;
		tegra_grenada_links[92].cpu_of_node = of_parse_phandle(np,
					"nvidia,dmic-2", 0);
		if (!tegra_grenada_links[92].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,dmic-2' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[93].cpu_of_node = tegra_grenada_links[0].codec_of_node;
		tegra_grenada_links[93].codec_of_node = tegra_grenada_links[92].cpu_of_node;

		tegra_grenada_links[94].codec_of_node = tegra_grenada_links[14].codec_of_node;
		tegra_grenada_links[94].cpu_of_node = of_parse_phandle(np,
					"nvidia,dmic-3", 0);
		if (!tegra_grenada_links[94].cpu_of_node) {
			dev_err(&pdev->dev,
				"Property 'nvidia,dmic-3' missing or invalid\n");
			ret = -EINVAL;
			goto err;
		}
		tegra_grenada_links[95].cpu_of_node = tegra_grenada_links[0].codec_of_node;
		tegra_grenada_links[95].codec_of_node = tegra_grenada_links[94].cpu_of_node;
	}

#ifndef CONFIG_MACH_GRENADA
	ret = tegra_alt_asoc_utils_init(&machine->audio_clock,
					&pdev->dev,
					card);
	if (ret)
		goto err;
#endif

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}

	return 0;

err_fini_utils:
#ifndef CONFIG_MACH_GRENADA
	tegra_alt_asoc_utils_fini(&machine->audio_clock);
#endif
err:
	return ret;
}

static int tegra_grenada_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
#ifndef CONFIG_MACH_GRENADA
	tegra_alt_asoc_utils_fini(&machine->audio_clock);
#endif

	return 0;
}

static const struct of_device_id tegra_grenada_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-grenada", },
	{},
};

static struct platform_driver tegra_grenada_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_grenada_of_match,
	},
	.probe = tegra_grenada_driver_probe,
	.remove = tegra_grenada_driver_remove,
};
module_platform_driver(tegra_grenada_driver);

MODULE_AUTHOR("Songhee Baek <sbaek@nvidia.com>");
MODULE_DESCRIPTION("Tegra+GRENADA machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_grenada_of_match);
