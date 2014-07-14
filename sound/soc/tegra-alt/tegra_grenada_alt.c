/*
 * tegra_grenada.c - Tegra grenada Machine driver
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
#include "tegra_asoc_machine_alt.h"
#include "ahub_unit_fpga_clock.h"

#define DRV_NAME "tegra-snd-grenada"

#define MAX_TX_SLOT_SIZE 32
#define MAX_RX_SLOT_SIZE 32

struct tegra_grenada {
	struct tegra_asoc_audio_clock_info audio_clock;
	unsigned int num_codec_links;
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
	struct snd_soc_dai *i2s_dai = rtd->cpu_dai;
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

	unsigned int srate;
	int err;
	AD1937_EXTRA_INFO ad1937_info;

	srate = dai_params->rate_min;

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
	i2s_clk_setup(I2S3, 0, 0);
	i2s_clk_setup(I2S5, 0, 3);

	i2s_pinmux_setup(I2S1, 0);
	i2s_pinmux_setup(I2S3, 0);
	i2s_pinmux_setup(I2S5, 1);

#else
	/* PLL, I2S and Codec programming */
	program_cdc_pll(2, CLK_OUT_12_2888_MHZ);
	program_io_expander();
	i2c_clk_divider(10 - 1);

	/* I2S1: 23 for 8Khz, 3 for 48Khz */
	i2s_clk_divider(I2S1, 23);
	i2s_clk_divider(I2S3, 0);
	i2s_clk_divider(I2S5, 3);
#endif
	program_max_codec();
	ad1937_info.codecId = AD1937_X_ADDRESS;
	ad1937_info.clkgenId = CLK_OUT_FROM_TEGRA;
	ad1937_info.dacMasterEn = AUDIO_DAC_SLAVE_MODE;
	ad1937_info.daisyEn = 0;
	OnAD1937CaptureAndPlayback(AUDIO_CODEC_SLAVE_MODE,
				AUDIO_INTERFACE_I2S_FORMAT,
				I2S_DATAWIDTH_16,
				64, 0, 0,
				AUDIO_SAMPLE_RATE_48_00,
				&ad1937_info);

	ad1937_info.codecId = AD1937_Y_ADDRESS;
	ad1937_info.clkgenId = CLK_OUT_FROM_TEGRA;
	ad1937_info.dacMasterEn = AUDIO_DAC_SLAVE_MODE;
	ad1937_info.daisyEn = 0;
	OnAD1937CaptureAndPlayback(AUDIO_CODEC_SLAVE_MODE,
				AUDIO_INTERFACE_TDM_FORMAT,
				I2S_DATAWIDTH_16,
				256, 0, 0,
				AUDIO_SAMPLE_RATE_48_00,
				&ad1937_info);
	return 0;
}

static int tegra_grenada_amx1_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *amx_dai = rtd->cpu_dai;
	struct device_node *np = rtd->card->dev->of_node;
	unsigned int tx_slot[MAX_TX_SLOT_SIZE], i, j;

	if (of_property_read_u32_array(np, "nvidia,amx-slot-map",
		tx_slot, MAX_TX_SLOT_SIZE))
		for (i = 0, j = 0; i < MAX_TX_SLOT_SIZE; i += 8) {
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
				MAX_TX_SLOT_SIZE, tx_slot, 0, 0);

	return 0;
}

static int tegra_grenada_amx2_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *amx_dai = rtd->cpu_dai;
	struct device_node *np = rtd->card->dev->of_node;
	unsigned int tx_slot[MAX_TX_SLOT_SIZE], i, j;

	if (of_property_read_u32_array(np, "nvidia,amx-slot-map",
		tx_slot, MAX_TX_SLOT_SIZE))
		for (i = 0, j = 0; i < MAX_TX_SLOT_SIZE; i += 8) {
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
				MAX_TX_SLOT_SIZE, tx_slot, 0, 0);

	return 0;
}


static int tegra_grenada_adx1_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *adx_dai = rtd->codec_dai;
	struct device_node *np = rtd->card->dev->of_node;
	unsigned int rx_slot[MAX_RX_SLOT_SIZE], i, j;

	if (of_property_read_u32_array(np, "nvidia,adx-slot-map",
		rx_slot, MAX_RX_SLOT_SIZE))
		for (i = 0, j = 0; i < MAX_RX_SLOT_SIZE; i += 8) {
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
				0, 0, MAX_RX_SLOT_SIZE, rx_slot);

	return 0;
}

static int tegra_grenada_adx2_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *adx_dai = rtd->codec_dai;
	struct device_node *np = rtd->card->dev->of_node;
	unsigned int rx_slot[MAX_RX_SLOT_SIZE], i, j;

	if (of_property_read_u32_array(np, "nvidia,adx-slot-map",
		rx_slot, MAX_RX_SLOT_SIZE))
		for (i = 0, j = 0; i < MAX_RX_SLOT_SIZE; i += 8) {
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
				0, 0, MAX_RX_SLOT_SIZE, rx_slot);

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

static struct snd_soc_card snd_soc_tegra_grenada = {
	.name = "tegra-grenada",
	.owner = THIS_MODULE,
	.remove = tegra_grenada_remove,
	.dapm_widgets = tegra_grenada_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_grenada_dapm_widgets),
	.fully_routed = true,
};

static int tegra_grenada_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_grenada;
	struct tegra_grenada *machine;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *tegra_grenada_codec_links = NULL;
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra_grenada_codec_conf = NULL;
	int ret = 0, i;

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

		ret = snd_soc_of_parse_audio_routing(card,
					"nvidia,audio-routing");
		if (ret)
			goto err;
	}

	/* set new codec links and conf */
	tegra_grenada_codec_links = tegra_machine_new_codec_links(pdev,
		tegra_grenada_codec_links,
		&machine->num_codec_links);
	if (!tegra_grenada_codec_links)
		goto err_alloc_dai_link;

	tegra_grenada_codec_conf = tegra_machine_new_codec_conf(pdev,
		tegra_grenada_codec_conf,
		&machine->num_codec_links);
	if (!tegra_grenada_codec_conf)
		goto err_alloc_dai_link;

	/* get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	if (!tegra_machine_dai_links)
		goto err_alloc_dai_link;
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	if (!tegra_machine_codec_conf)
		goto err_alloc_dai_link;

	tegra_grenada_codec_links[1].init = tegra_grenada_init;

	/* set ADMAIF dai_ops */
	for (i = TEGRA210_DAI_LINK_ADMAIF1;
		i <= TEGRA210_DAI_LINK_ADMAIF10; i++)
		tegra_machine_set_dai_ops(i, &tegra_grenada_ops);

	/* set AMX/ADX dai_init */
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_AMX1,
		&tegra_grenada_amx1_dai_init);
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_ADX1,
		&tegra_grenada_adx1_dai_init);
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_AMX2,
		&tegra_grenada_amx2_dai_init);
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_ADX2,
		&tegra_grenada_adx2_dai_init);

	/* set sfc dai_init */
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_SFC1_RX,
		&tegra_grenada_sfc_init);

	/* set ADSP PCM/COMPR */
	for (i = TEGRA210_DAI_LINK_ADSP_PCM;
		i <= TEGRA210_DAI_LINK_ADSP_COMPR2; i++) {
		tegra_machine_set_dai_ops(i,
			&tegra_grenada_ops);
	}

	/* append grenada specific dai_links */
	card->num_links =
		tegra_machine_append_dai_link(tegra_grenada_codec_links,
			2 * machine->num_codec_links);
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	card->dai_link = tegra_machine_dai_links;

	/* append grenada specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf(tegra_grenada_codec_conf,
			machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	card->codec_conf = tegra_machine_codec_conf;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_alloc_dai_link;
	}

	/* release new codec_links and codec_conf */
	tegra_machine_remove_new_codec_links(tegra_grenada_codec_links);
	tegra_machine_remove_new_codec_conf(tegra_grenada_codec_conf);

	return 0;

err_alloc_dai_link:
	tegra_machine_remove_codec_conf();
	tegra_machine_remove_dai_link();
	tegra_machine_remove_new_codec_links(tegra_grenada_codec_links);
	tegra_machine_remove_new_codec_conf(tegra_grenada_codec_conf);
err:
	return ret;
}

static int tegra_grenada_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_grenada *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	tegra_machine_remove_extra_mem_alloc(machine->num_codec_links);
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
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
