/*
 * tegra_vcm30t124_b00.c - Tegra VCM30 T124 B00 Machine driver
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

#include "../codecs/ad193x.h"
#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"

#define DRV_NAME "tegra-snd-vcm30t124-b00"

#define GPIO_PR0 136
#define CODEC_TO_DAP 0
#define DAP_TO_CODEC 1

static struct snd_soc_dai_link *tegra_machine_dai_links;

struct tegra_vcm30t124 {
	struct tegra_asoc_audio_clock_info audio_clock;
	int gpio_dap_direction;
	int gpio_dap_d_en;
	struct i2c_client *max9485_client;
};

#define MAX9485_MCLK_FREQ_163840 0x31
#define MAX9485_MCLK_FREQ_112896 0x22
#define MAX9485_MCLK_FREQ_122880 0x23
#define MAX9485_MCLK_FREQ_225792 0x32
#define MAX9485_MCLK_FREQ_245760 0x33

enum {
	DAI_LINK_I2S0_CIF,
	DAI_LINK_I2S0_DAP, /* ak4618 link */
	DAI_LINK_I2S4_CIF,
	DAI_LINK_I2S4_DAP, /* ad1937 link */
	NUM_CODEC_DAI_LINKS,
};

static void set_max9485_clk(struct i2c_client *i2s, int mclk)
{
	char clk;

	switch (mclk) {
	case 16384000:
		clk =  MAX9485_MCLK_FREQ_163840;
		break;
	case 11289600:
		clk = MAX9485_MCLK_FREQ_112896;
		break;
	case 12288000:
		clk = MAX9485_MCLK_FREQ_122880;
		break;
	case 22579200:
		clk = MAX9485_MCLK_FREQ_225792;
		break;
	case 24576000:
		clk = MAX9485_MCLK_FREQ_245760;
		break;
	default:
		return;
	}
	i2c_master_send(i2s, &clk, 1);
}

static int tegra_vcm30t124_ak4618_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_vcm30t124 *machine = snd_soc_card_get_drvdata(card);
	unsigned int idx = NUM_XBAR_DAI_LINKS + DAI_LINK_I2S0_DAP;
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;
	unsigned int fmt = card->rtd[idx].dai_link->dai_fmt;
	int srate, mclk, clk_out_rate;
	int err;

	srate = params_rate(params);

	if ((srate < dai_params->rate_min) || (srate > dai_params->rate_max)) {
		dev_err(card->dev, "Supported range is [%d, %d]\n",
			dai_params->rate_min, dai_params->rate_max);
		return -EINVAL;
	}

	switch (srate) {
	case 64000:
	case 96000:
		clk_out_rate = srate * 256;
		mclk = 12288000 * 2;
		break;
	case 88200:
		clk_out_rate = srate * 256;
		mclk = 11289600 * 2;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	default:
		clk_out_rate = srate * 512;
		/*
		 * MCLK is pll_a_out, it is a source clock of ahub.
		 * So it need to be faster than BCLK in slave mode.
		 */
		mclk = 12288000 * 2;
		break;
	case 44100:
		clk_out_rate = srate * 512;
		/*
		 * MCLK is pll_a_out, it is a source clock of ahub.
		 * So it need to be faster than BCLK in slave mode.
		 */
		mclk = 11289600 * 2;
		break;
	case 17640:
		clk_out_rate = srate * 128;
		mclk = 11289600 * 2;
		break;
	case 19200:
		clk_out_rate = srate * 128;
		mclk = 12288000 * 2;
		break;
	}

	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
					srate, mclk, clk_out_rate);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(card->rtd[idx].codec_dai,
			0, clk_out_rate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "x codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(card->rtd[idx].cpu_dai, 0, srate,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "x cpu_dai clock not set\n");
		return err;
	}

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_DSP_A) {
		snd_soc_dai_set_tdm_slot(card->rtd[idx].codec_dai,
					0, 0, dai_params->channels_min, 32);
	} else {
		dai_params->channels_min = params_channels(params);
		dai_params->formats = (1ULL << (params_format(params)));
		dai_params->rate_min = srate;
	}

	return 0;
}

static int tegra_vcm30t124_ad1937_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_vcm30t124 *machine = snd_soc_card_get_drvdata(card);
	unsigned int idx = NUM_XBAR_DAI_LINKS + DAI_LINK_I2S4_DAP;
	unsigned int fmt = card->rtd[idx].dai_link->dai_fmt;
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;
	int srate, mclk, clk_out_rate, val;
	int err;

	srate = params_rate(params);

	if ((srate < dai_params->rate_min) || (srate > dai_params->rate_max)) {
		dev_err(card->dev, "Supported range is [%d, %d]\n",
			dai_params->rate_min, dai_params->rate_max);
		return -EINVAL;
	}

	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		clk_out_rate = 128 * srate;
		break;
	default:
		clk_out_rate = 256 * srate;
		break;
	}

	/* update link_param to update hw_param for DAPM */
	dai_params->rate_min = srate;

	mclk = clk_out_rate * 2;

	set_max9485_clk(machine->max9485_client, mclk);

	err = snd_soc_dai_set_sysclk(card->rtd[idx].codec_dai,
			0, mclk,
			SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "y codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(card->rtd[idx].cpu_dai, 0, srate,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "y cpu_dai clock not set\n");
		return err;
	}

	/*
	 * AD193X driver enables both DAC and ADC as MASTER
	 * so both ADC and DAC drive LRCLK and BCLK and it causes
	 * noise. To solve this, we need to disable one of them.
	 */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBM_CFM) {
		val = snd_soc_read(card->rtd[idx].codec_dai->codec,
				AD193X_DAC_CTRL1);
		val &= ~AD193X_DAC_LCR_MASTER;
		val &= ~AD193X_DAC_BCLK_MASTER;
		snd_soc_write(card->rtd[idx].codec_dai->codec,
				AD193X_DAC_CTRL1, val);
	}

	return 0;
}

static int tegra_vcm30t124_ak4618_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void tegra_vcm30t124_ak4618_shutdown(struct snd_pcm_substream *substream)
{
	return;
}

static int tegra_vcm30t124_ad1937_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void tegra_vcm30t124_ad1937_shutdown(struct snd_pcm_substream *substream)
{
	return;
}

static struct snd_soc_ops tegra_vcm30t124_ak4618_ops = {
	.hw_params = tegra_vcm30t124_ak4618_hw_params,
	.startup = tegra_vcm30t124_ak4618_startup,
	.shutdown = tegra_vcm30t124_ak4618_shutdown,
};

static struct snd_soc_ops tegra_vcm30t124_ad1937_ops = {
	.hw_params = tegra_vcm30t124_ad1937_hw_params,
	.startup = tegra_vcm30t124_ad1937_startup,
	.shutdown = tegra_vcm30t124_ad1937_shutdown,
};

static const struct snd_soc_dapm_widget tegra_vcm30t124_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone-x", NULL),
	SND_SOC_DAPM_MIC("Mic-x", NULL),
	SND_SOC_DAPM_HP("Headphone-y", NULL),
	SND_SOC_DAPM_LINE("LineIn-y", NULL),
};

static const struct snd_soc_dapm_route tegra_vcm30t124_audio_map[] = {
	{"Headphone-x",	NULL,	"x DACOUT1"},
	{"Headphone-x",	NULL,	"x DACOUT2"},
	{"Headphone-x",	NULL,	"x DACOUT3"},
	{"Headphone-x",	NULL,	"x DACOUT4"},
	{"Headphone-x",	NULL,	"x DACOUT5"},
	{"Headphone-x",	NULL,	"x DACOUT6"},
	{"Mic-x", NULL, "x MICBIAS"},
	{"x IN1",	NULL,	"Mic-x"},
	{"x IN2",	NULL,	"Mic-x"},
	{"x IN3",	NULL,	"Mic-x"},
	{"x IN4",	NULL,	"Mic-x"},
	{"x IN5",	NULL,	"Mic-x"},
	{"x IN6",	NULL,	"Mic-x"},
	{"Headphone-y",	NULL,	"y DAC1OUT"},
	{"Headphone-y", NULL,	"y DAC2OUT"},
	{"Headphone-y",	NULL,	"y DAC3OUT"},
	{"Headphone-y", NULL,	"y DAC4OUT"},
	{"y ADC1IN",	NULL,	"LineIn-y"},
	{"y ADC2IN",	NULL,	"LineIn-y"},
};

static int tegra_vcm30t124_ak4618_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_vcm30t124 *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	unsigned int fmt = rtd->dai_link->dai_fmt;
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;
	unsigned int clk_out, mclk, srate;
	int err;

	/* Default sampling rate*/
	srate = dai_params->rate_min;
	clk_out = srate * 512;
	mclk = clk_out;

	tegra_alt_asoc_utils_set_parent(&machine->audio_clock, true);

	/* ak4618 needs mclk from tegra */
	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
					srate, mclk, clk_out);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, clk_out,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "ak4618 clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, srate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "i2s clock not set\n");
		return err;
	}

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_DSP_A)
		snd_soc_dai_set_tdm_slot(codec_dai, 0, 0, 8, 32);

	snd_soc_dapm_force_enable_pin(dapm, "x MICBIAS");

	return 0;
}

static int tegra_vcm30t124_ad1937_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_vcm30t124 *machine = snd_soc_card_get_drvdata(card);
	unsigned int fmt = rtd->dai_link->dai_fmt;
	unsigned int mclk, srate;
	int err;

	srate = 48000;
	mclk = srate * 512;

	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBM_CFM) {
		/* direct MCLK mode in AD1937, mclk needs to be srate * 512 */
		set_max9485_clk(machine->max9485_client, mclk);
		err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
						SND_SOC_CLOCK_IN);
		if (err < 0) {
			dev_err(card->dev, "ad1937 clock not set\n");
			return err;
		}

		snd_soc_write(codec, AD193X_PLL_CLK_CTRL1, 0x03);

		/* set SCLK, FS direction from codec to dap */
		gpio_direction_output(machine->gpio_dap_direction,
					CODEC_TO_DAP);
	} else {
		/* set PLL_SRC with LRCLK for AD1937 slave mode */
		snd_soc_write(codec, AD193X_PLL_CLK_CTRL0, 0xb9);

		/* set SCLK, FS direction from dap to codec */
		gpio_direction_output(machine->gpio_dap_direction,
					DAP_TO_CODEC);
	}

	err = snd_soc_dai_set_sysclk(cpu_dai, 0, srate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "i2s clock not set %d\n", __LINE__);
		return err;
	}

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_DSP_A)
		codec_dai->driver->ops->set_tdm_slot(codec_dai, 0, 0, 8, 0);

	return 0;
}

static int tegra_vcm30t124_amx_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *amx_dai = rtd->cpu_dai;
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

static int tegra_vcm30t124_adx_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *adx_dai = rtd->codec_dai;
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

static int tegra_vcm30t124_dam_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int in_srate, out_srate;
	int err;

	in_srate = 8000;
	out_srate = 48000;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, out_srate,
					SND_SOC_CLOCK_OUT);
	err = snd_soc_dai_set_sysclk(codec_dai, 0, in_srate,
					SND_SOC_CLOCK_IN);

	return 0;
}

static int tegra_vcm30t124_remove(struct snd_soc_card *card)
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

static struct snd_soc_dai_link tegra_vcm30t124_links[NUM_CODEC_DAI_LINKS] = {
	[DAI_LINK_I2S0_CIF] = {
		.name = "I2S0 RX",
		.stream_name = "I2S0 RX",
		.cpu_dai_name = "I2S0",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-i2s.0",
		.params = &tdm_link_params,
	},
	[DAI_LINK_I2S0_DAP] = {
		.name = "ak4618",
		.stream_name = "Playback",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "ak4618-hifi",
		.cpu_name = "tegra30-i2s.0",
		.codec_name = "ak4618.0-0010",
		.init = tegra_vcm30t124_ak4618_init,
		.params = &tdm_link_params,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
	},
	[DAI_LINK_I2S4_CIF] = {
		.name = "I2S4 RX",
		.stream_name = "I2S4 RX",
		.cpu_dai_name = "I2S4",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-i2s.4",
		.params = &tdm_link_params,
	},
	[DAI_LINK_I2S4_DAP] = {
		.name = "ad1937",
		.stream_name = "Playback",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "ad193x-hifi",
		.cpu_name = "tegra30-i2s.4",
		.codec_name = "ad193x.0-0007",
		.init = tegra_vcm30t124_ad1937_init,
		.params = &tdm_link_params,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			   SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBM_CFM,
	},
};

static struct snd_soc_codec_conf ad193x_codec_conf[] = {
	{
		.dev_name = "tegra124-amx.0",
		.name_prefix = "AMX0",
	},
	{
		.dev_name = "tegra124-amx.1",
		.name_prefix = "AMX1",
	},
	{
		.dev_name = "tegra124-adx.0",
		.name_prefix = "ADX0",
	},
	{
		.dev_name = "tegra124-adx.1",
		.name_prefix = "ADX1",
	},
	{
		.dev_name = "tegra30-dam.0",
		.name_prefix = "DAM0",
	},
	{
		.dev_name = "tegra30-dam.1",
		.name_prefix = "DAM1",
	},
	{
		.dev_name = "tegra30-dam.2",
		.name_prefix = "DAM2",
	},
	{
		.dev_name = "tegra124-afc.0",
		.name_prefix = "AFC0",
	},
	{
		.dev_name = "tegra124-afc.1",
		.name_prefix = "AFC1",
	},
	{
		.dev_name = "tegra124-afc.2",
		.name_prefix = "AFC2",
	},
	{
		.dev_name = "tegra124-afc.3",
		.name_prefix = "AFC3",
	},
	{
		.dev_name = "tegra124-afc.4",
		.name_prefix = "AFC4",
	},
	{
		.dev_name = "tegra124-afc.5",
		.name_prefix = "AFC5",
	},
	{
		.dev_name = "tegra30-i2s.0",
		.name_prefix = "I2S0",
	},
	{
		.dev_name = "ak4618.0-0010",
		.name_prefix = "x",
	},
	{
		.dev_name = "tegra30-i2s.4",
		.name_prefix = "I2S4",
	},
	{
		.dev_name = "ad193x.0-0007",
		.name_prefix = "y",
	},
};

static struct snd_soc_card snd_soc_tegra_vcm30t124 = {
	.name = "tegra-vcm30t124-b00",
	.owner = THIS_MODULE,
	.remove = tegra_vcm30t124_remove,
	.dapm_widgets = tegra_vcm30t124_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_vcm30t124_dapm_widgets),
	.codec_conf = ad193x_codec_conf,
	.num_configs = ARRAY_SIZE(ad193x_codec_conf),
	.fully_routed = true,
};

static int tegra_vcm30t124_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_vcm30t124;
	struct tegra_vcm30t124 *machine;
	int ret = 0, i;

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_vcm30t124),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_vcm30t124 struct\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	/* get the xbar dai link structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link();

	/* set APBIF dai_ops */
	for (i = DAI_LINK_APBIF0; i <= DAI_LINK_APBIF3; i++)
		tegra_machine_set_dai_ops(i, &tegra_vcm30t124_ad1937_ops);
	for (i = DAI_LINK_APBIF4; i <= DAI_LINK_APBIF7; i++)
		tegra_machine_set_dai_ops(i, &tegra_vcm30t124_ak4618_ops);
	tegra_machine_set_dai_ops(DAI_LINK_APBIF8,
		&tegra_vcm30t124_ad1937_ops);
	tegra_machine_set_dai_ops(DAI_LINK_APBIF9,
		&tegra_vcm30t124_ad1937_ops);

	/* set AMX/ADX dai_init */
	tegra_machine_set_dai_init(DAI_LINK_AMX0,
		&tegra_vcm30t124_amx_dai_init);
	tegra_machine_set_dai_init(DAI_LINK_AMX1,
		&tegra_vcm30t124_amx_dai_init);
	tegra_machine_set_dai_init(DAI_LINK_ADX0,
		&tegra_vcm30t124_adx_dai_init);
	tegra_machine_set_dai_init(DAI_LINK_ADX1,
		&tegra_vcm30t124_adx_dai_init);

	/* set DAM dai_init */
	tegra_machine_set_dai_init(DAI_LINK_DAM0_0,
		&tegra_vcm30t124_dam_init);
	tegra_machine_set_dai_init(DAI_LINK_DAM1_0,
		&tegra_vcm30t124_dam_init);
	tegra_machine_set_dai_init(DAI_LINK_DAM2_0,
		&tegra_vcm30t124_dam_init);

	/* append vcm30t124 specific dai_links */
	card->num_links = tegra_machine_append_dai_link(tegra_vcm30t124_links,
		NUM_CODEC_DAI_LINKS);
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	card->dai_link = tegra_machine_dai_links;

	machine->gpio_dap_direction = GPIO_PR0;
	card->dapm_routes = tegra_vcm30t124_audio_map;
	card->num_dapm_routes = ARRAY_SIZE(tegra_vcm30t124_audio_map);

	machine->max9485_client = i2c_new_device(i2c_get_adapter(0),
						pdev->dev.platform_data);
	if (!machine->max9485_client) {
		dev_err(&pdev->dev, "cannot get i2c device for max9485\n");
		goto err;

	}

	ret = devm_gpio_request(&pdev->dev, machine->gpio_dap_direction,
				"dap_dir_control");
	if (ret) {
		dev_err(&pdev->dev, "cannot get dap_dir_control gpio\n");
		goto err_i2c_unregister;
	}

	ret = tegra_alt_asoc_utils_init(&machine->audio_clock,
					&pdev->dev,
					card);
	if (ret)
		goto err_gpio_free;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}

	return 0;

err_fini_utils:
	tegra_alt_asoc_utils_fini(&machine->audio_clock);
err_gpio_free:
	devm_gpio_free(&pdev->dev, machine->gpio_dap_direction);
err_i2c_unregister:
	i2c_unregister_device(machine->max9485_client);
err:
	return ret;
}

static int tegra_vcm30t124_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_vcm30t124 *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);
	tegra_machine_remove_dai_link();

	tegra_alt_asoc_utils_fini(&machine->audio_clock);
	devm_gpio_free(&pdev->dev, machine->gpio_dap_direction);
	i2c_unregister_device(machine->max9485_client);

	return 0;
}

static const struct of_device_id tegra_vcm30t124_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-vcm30t124-b00", },
	{},
};

static struct platform_driver tegra_vcm30t124_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_vcm30t124_of_match,
	},
	.probe = tegra_vcm30t124_driver_probe,
	.remove = tegra_vcm30t124_driver_remove,
};
module_platform_driver(tegra_vcm30t124_driver);

MODULE_AUTHOR("Songhee Baek <sbaek@nvidia.com>");
MODULE_DESCRIPTION("Tegra+VCM30T124+B00 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_vcm30t124_of_match);
