/*
 * tegra_t210ref_mobile.c - Tegra T210 Machine driver for mobile
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
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/sysedp.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/pm_runtime.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/gpio-tegra.h>
#include <mach/tegra_rt5640_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/rt5639.h"
#include "../codecs/wm8731.h"
#include "../codecs/ad193x.h"

#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"
#include "ahub_unit_fpga_clock.h"

#define DRV_NAME "tegra-snd-t210ref-mobile"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)

struct tegra_t210ref {
	struct tegra_asoc_platform_data *pdata;
	struct tegra_asoc_audio_clock_info audio_clock;
	unsigned int num_codec_links;
	int gpio_requested;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
	enum snd_soc_bias_level bias_level;
	int clock_enabled;
	struct regulator *codec_reg;
	struct regulator *digital_reg;
	struct regulator *analog_reg;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	struct snd_soc_card *pcard;
};

static struct snd_soc_jack tegra_t210ref_hp_jack;

static struct snd_soc_jack_gpio tegra_t210ref_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
};

#ifdef CONFIG_SWITCH
static struct switch_dev tegra_t210ref_headset_switch = {
	.name = "h2w",
};

static int tegra_t210ref_jack_notifier(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	enum headset_state state = BIT_NO_HEADSET;
	unsigned char status_jack = 0;

	if (jack == &tegra_t210ref_hp_jack) {
		if (action) {
			/* Enable ext mic; enable signal is active-low */
			if (gpio_is_valid(pdata->gpio_ext_mic_en))
				gpio_direction_output(
				pdata->gpio_ext_mic_en, 0);

			status_jack = rt5639_headset_detect(codec, 1);

			machine->jack_status &= ~SND_JACK_HEADPHONE;
			machine->jack_status &= ~SND_JACK_MICROPHONE;

			if (status_jack == RT5639_HEADPHO_DET)
				machine->jack_status |=
				SND_JACK_HEADPHONE;
			else if (status_jack == RT5639_HEADSET_DET) {
					machine->jack_status |=
							SND_JACK_HEADPHONE;
					machine->jack_status |=
							SND_JACK_MICROPHONE;
			}
		} else {
			/* Disable ext mic; enable signal is active-low */
			if (gpio_is_valid(pdata->gpio_ext_mic_en))
				gpio_direction_output(
				pdata->gpio_ext_mic_en, 1);

			rt5639_headset_detect(codec, 0);

			machine->jack_status &= ~SND_JACK_HEADPHONE;
			machine->jack_status &= ~SND_JACK_MICROPHONE;
		}
	}

	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state = BIT_HEADSET;
		break;
	case SND_JACK_MICROPHONE:
		/* mic: would not report */
	default:
		state = BIT_NO_HEADSET;
	}

	switch_set_state(&tegra_t210ref_headset_switch, state);

	return NOTIFY_OK;
}

static struct notifier_block tegra_t210ref_jack_detect_nb = {
	.notifier_call = tegra_t210ref_jack_notifier,
};
#else
static struct snd_soc_jack_pin tegra_t210ref_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif

static int tegra_t210ref_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	unsigned int idx =  tegra_machine_get_codec_dai_link_idx("rt5639");
	struct snd_soc_pcm_stream *dai_params;
	unsigned int fmt;
	int srate;
	unsigned int mclk;
	int err;


	/* check if idx has valid number */
	if (idx == -EINVAL)
		return idx;

	dai_params =
		(struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;
	fmt = card->rtd[idx].dai_link->dai_fmt;

	srate = params_rate(params);

	/* update link_param to update hw_param for DAPM */
	dai_params->rate_min = srate;
	dai_params->channels_min = params_channels(params);
	dai_params->formats = (1ULL << (params_format(params)));

	mclk = (256 * 48000);

	/* I2S1: 23 for 8Khz, 3 for 48Khz */
	if (srate == 8000)
		i2s_clk_divider(I2S1, 23);
	else
		i2s_clk_divider(I2S1, 3);

	err = snd_soc_dai_set_sysclk(card->rtd[idx].codec_dai,
				RT5639_SCLK_S_MCLK, mclk, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(card->rtd[idx].codec_dai, fmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	return 0;
}

static int tegra_t210ref_startup(struct snd_pcm_substream *substream)
{
	return 0;
}
static void tegra_t210ref_shutdown(struct snd_pcm_substream *substream)
{
	return;
}

static int tegra_t210ref_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	struct snd_soc_dai *i2s_dai = rtd->cpu_dai;
	struct snd_soc_pcm_stream *dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;
	unsigned int srate;
	int err;

	srate = dai_params->rate_min;
	err = snd_soc_dai_set_sysclk(i2s_dai, 0, srate, SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "i2s clock not set %d\n", srate);
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

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_t210ref_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		tegra_t210ref_hp_jack_gpio.invert =
			!pdata->gpio_hp_det_active_high;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_t210ref_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_t210ref_hp_jack,
					ARRAY_SIZE(tegra_t210ref_hp_jack_pins),
					tegra_t210ref_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_t210ref_hp_jack,
					&tegra_t210ref_jack_detect_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_t210ref_hp_jack,
					1,
					&tegra_t210ref_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;
	}

	/* FIXME: Calculate automatically based on DAPM routes? */
	snd_soc_dapm_nc_pin(dapm, "LOUTL");
	snd_soc_dapm_nc_pin(dapm, "LOUTR");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static int tegra_t210ref_sfc_init(struct snd_soc_pcm_runtime *rtd)
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


static int tegra_rt5639_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int err;

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			err = regulator_enable(machine->spk_reg);
		else
			regulator_disable(machine->spk_reg);
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_spkr_en,
				!!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_rt5639_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_rt5639_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;
	int ret = 0;

	if (machine->dmic_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			ret = regulator_enable(machine->dmic_reg);
		else
			regulator_disable(machine->dmic_reg);
	}

	if (!(machine->gpio_requested & GPIO_INT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_int_mic_en,
				!!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_rt5639_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static struct snd_soc_ops tegra_t210ref_ops = {
	.hw_params = tegra_t210ref_hw_params,
	.startup = tegra_t210ref_startup,
	.shutdown = tegra_t210ref_shutdown,
};

static const struct snd_soc_dapm_widget tegra_t210ref_dapm_widgets[] = {
	SND_SOC_DAPM_HP("x Headphone Jack", tegra_rt5639_event_hp),
	SND_SOC_DAPM_SPK("x Int Spk", tegra_rt5639_event_int_spk),
	SND_SOC_DAPM_HP("y Headphone", NULL),
	SND_SOC_DAPM_HP("z Headphone", NULL),
	SND_SOC_DAPM_HP("s Headphone", NULL),
	SND_SOC_DAPM_MIC("x Int Mic", tegra_rt5639_event_int_mic),
	SND_SOC_DAPM_MIC("x Mic Jack", tegra_rt5639_event_ext_mic),
	SND_SOC_DAPM_MIC("y Mic", NULL),
	SND_SOC_DAPM_MIC("z Mic", NULL),
	SND_SOC_DAPM_MIC("s Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_soc_dapm_route tegra_t210ref_audio_map[] = {
};

static const struct snd_kcontrol_new tegra_t210ref_controls[] = {
	SOC_DAPM_PIN_SWITCH("x Int Spk"),
	SOC_DAPM_PIN_SWITCH("x Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("x Mic Jack"),
	SOC_DAPM_PIN_SWITCH("x Int Mic"),
};

static int tegra_t210ref_remove(struct snd_soc_card *card)
{
	return 0;
}

static struct snd_soc_card snd_soc_tegra_t210ref = {
	.name = "tegra-t210ref",
	.owner = THIS_MODULE,
	.remove = tegra_t210ref_remove,
	.controls = tegra_t210ref_controls,
	.num_controls = ARRAY_SIZE(tegra_t210ref_controls),
	.dapm_widgets = tegra_t210ref_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_t210ref_dapm_widgets),
	.fully_routed = true,
};

static int tegra_t210ref_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_t210ref;
	struct tegra_t210ref *machine;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *tegra_t210ref_codec_links = NULL;
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra_t210ref_codec_conf = NULL;
	struct tegra_asoc_platform_data *pdata = NULL;
	int ret = 0, i;

	if (!np) {
		dev_err(&pdev->dev, "No device tree node for t210ref driver");
		return -ENODEV;
	}

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_t210ref),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_t210ref struct\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
				"nvidia,audio-routing");
	if (ret)
		goto err;

	/* set new codec links and conf */
	tegra_t210ref_codec_links = tegra_machine_new_codec_links(pdev,
		tegra_t210ref_codec_links,
		&machine->num_codec_links);
	if (!tegra_t210ref_codec_links)
		goto err_alloc_dai_link;

	tegra_t210ref_codec_conf = tegra_machine_new_codec_conf(pdev,
		tegra_t210ref_codec_conf,
		&machine->num_codec_links);
	if (!tegra_t210ref_codec_conf)
		goto err_alloc_dai_link;


	/* get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	if (!tegra_machine_dai_links)
		goto err_alloc_dai_link;
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	if (!tegra_machine_codec_conf)
		goto err_alloc_dai_link;

	tegra_t210ref_codec_links[1].init = tegra_t210ref_init;

	/* set ADMAIF dai_ops */
	for (i = TEGRA210_DAI_LINK_ADMAIF1;
		i <= TEGRA210_DAI_LINK_ADMAIF10; i++)
		tegra_machine_set_dai_ops(i, &tegra_t210ref_ops);


	/* set sfc dai_init */
	tegra_machine_set_dai_init(TEGRA210_DAI_LINK_SFC1_RX,
		&tegra_t210ref_sfc_init);

	/* set ADSP PCM/COMPR */
	for (i = TEGRA210_DAI_LINK_ADSP_PCM;
		i <= TEGRA210_DAI_LINK_ADSP_COMPR2; i++) {
		tegra_machine_set_dai_ops(i,
			&tegra_t210ref_ops);
	}

	/* append t210ref specific dai_links */
	card->num_links =
		tegra_machine_append_dai_link(tegra_t210ref_codec_links,
			2 * machine->num_codec_links);
	tegra_machine_dai_links = tegra_machine_get_dai_link();
	card->dai_link = tegra_machine_dai_links;

	/* append t210ref specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf(tegra_t210ref_codec_conf,
			machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf();
	card->codec_conf = tegra_machine_codec_conf;

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = tegra_alt_asoc_switch_register(&tegra_t210ref_headset_switch);
	if (ret < 0)
		goto err_alloc_dai_link;
#endif

	pdata = kzalloc(sizeof(struct tegra_asoc_platform_data),
			GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev,
			"Can't allocate tegra_asoc_platform_data struct\n");
		return -ENOMEM;
	}

	pdata->gpio_ldo1_en = of_get_named_gpio(np,
					"nvidia,ldo-gpios", 0);
	if (pdata->gpio_ldo1_en < 0)
		dev_warn(&pdev->dev, "Failed to get LDO_EN GPIO\n");

	pdata->gpio_hp_det = of_get_named_gpio(np,
					"nvidia,hp-det-gpios", 0);
	if (pdata->gpio_hp_det < 0)
		dev_warn(&pdev->dev, "Failed to get HP Det GPIO\n");

	pdata->gpio_codec1 = pdata->gpio_codec2 = pdata->gpio_codec3 =
	pdata->gpio_spkr_en = pdata->gpio_hp_mute =
	pdata->gpio_int_mic_en = pdata->gpio_ext_mic_en = -1;


	if (gpio_is_valid(pdata->gpio_ldo1_en)) {
		ret = gpio_request(pdata->gpio_ldo1_en, "rt5639");
		if (ret)
			dev_err(&pdev->dev, "Fail gpio_request AUDIO_LDO1\n");
		else {
			ret = gpio_direction_output(pdata->gpio_ldo1_en, 1);
			if (ret)
				dev_err(&pdev->dev, "Fail gpio_direction AUDIO_LDO1\n");
		}
		msleep(200);
	}

	/*
	*codec_reg - its a GPIO (in the form of a fixed regulator) that enables
	*the basic(I2C) power for the codec and must be ON always
	*/
	if (!gpio_is_valid(pdata->gpio_ldo1_en)) {
		machine->codec_reg = regulator_get(&pdev->dev, "ldoen");
		if (IS_ERR(machine->codec_reg))
			machine->codec_reg = 0;
		else
			ret = regulator_enable(machine->codec_reg);
	}

	/*
	*digital_reg - provided the digital power for the codec and must be
	*ON always
	*/
	machine->digital_reg = regulator_get(&pdev->dev, "dbvdd");
	if (IS_ERR(machine->digital_reg))
		machine->digital_reg = 0;
	else
		ret = regulator_enable(machine->digital_reg);

	/*
	*analog_reg - provided the analog power for the codec and must be
	*ON always
	*/
	machine->analog_reg = regulator_get(&pdev->dev, "avdd");
	if (IS_ERR(machine->analog_reg))
		machine->analog_reg = 0;
	else
		ret = regulator_enable(machine->analog_reg);

	/*
	*spk_reg - provided the speaker power and can be turned ON
	*on need basis, when required
	*/
	machine->spk_reg = regulator_get(&pdev->dev, "spkvdd");
	if (IS_ERR(machine->spk_reg))
		machine->spk_reg = 0;

	/*
	*dmic_reg - provided the DMIC power and can be turned ON
	*on need basis, when required
	*/
	machine->dmic_reg = regulator_get(&pdev->dev, "dmicvdd");
	if (IS_ERR(machine->dmic_reg))
		machine->dmic_reg = 0;

	machine->pdata = pdata;
	machine->pcard = card;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_alloc_dai_link;
	}

	/* release new codec_links and codec_conf */
	tegra_machine_remove_new_codec_links(tegra_t210ref_codec_links);
	tegra_machine_remove_new_codec_conf(tegra_t210ref_codec_conf);

	return 0;

err_alloc_dai_link:
	tegra_machine_remove_codec_conf();
	tegra_machine_remove_dai_link();
	tegra_machine_remove_new_codec_links(tegra_t210ref_codec_links);
	tegra_machine_remove_new_codec_conf(tegra_t210ref_codec_conf);
err:
	return ret;
}

static int tegra_t210ref_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t210ref *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	tegra_machine_remove_extra_mem_alloc(machine->num_codec_links);
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
#ifndef CONFIG_MACH_t210ref
	tegra_alt_asoc_utils_fini(&machine->audio_clock);
#endif

	return 0;
}

static const struct of_device_id tegra_t210ref_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-t210ref-mobile", },
	{},
};

static struct platform_driver tegra_t210ref_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_t210ref_of_match,
	},
	.probe = tegra_t210ref_driver_probe,
	.remove = tegra_t210ref_driver_remove,
};
module_platform_driver(tegra_t210ref_driver);

MODULE_AUTHOR("Dara Ramesh <dramesh@nvidia.com>");
MODULE_DESCRIPTION("Tegra+t210ref machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_t210ref_of_match);
