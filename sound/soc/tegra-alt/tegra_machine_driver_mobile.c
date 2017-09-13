/*
 * tegra_machine_driver_mobile.c - Tegra ASoC Machine driver for mobile
 *
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/of_platform.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/platform_data/tegra_asoc_pdata.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <dt-bindings/sound/tas2552.h>
#include "rt5659.h"
#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"
#include "tegra210_xbar_alt.h"

#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#define DRV_NAME "tegra-snd-card-mobile"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)

#define PARAMS(sformat, channels)		\
	{					\
		.formats = sformat,		\
		.rate_min = 48000,		\
		.rate_max = 48000,		\
		.channels_min = channels,	\
		.channels_max = channels,	\
	}

/* machine structure which holds sound card */
struct tegra_machine {
	struct tegra_asoc_platform_data *pdata;
	struct tegra_asoc_audio_clock_info audio_clock;
	unsigned int num_codec_links;
	int gpio_requested;
	struct snd_soc_card *pcard;
	int rate_via_kcontrol;
	int is_codec_dummy;
	int fmt_via_kcontrol;
	struct tegra_machine_soc_data *soc_data;
	struct regulator *digital_reg;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
};

/* used for soc specific data */
struct tegra_machine_soc_data {
	unsigned int num_xbar_dai_links,
		/* dai link indexes */
		admaif_dai_link_start,
		admaif_dai_link_end,
		adsp_pcm_dai_link_start,
		adsp_pcm_dai_link_end,
		adsp_compr_dai_link_start,
		adsp_compr_dai_link_end,
		sfc_dai_link;

	bool is_asrc_available,
		is_clk_rate_via_dt,
		write_cdev1_state,
		write_idle_bias_off_state;

	/* call back APIs */
	unsigned int (*get_bclk_ratio)(struct snd_soc_pcm_runtime *rtd);
	struct snd_soc_dai_link *(*get_dai_link)(void);
	struct snd_soc_codec_conf *(*get_codec_conf)(void);
	int (*append_dai_link)(struct snd_soc_dai_link *link,
		unsigned int link_size);
	int (*append_codec_conf)(struct snd_soc_codec_conf *conf,
		unsigned int conf_size);
};

/* function prototypes */
static int tegra_machine_driver_remove(struct platform_device *);
static int tegra_machine_driver_probe(struct platform_device *);
static void dai_link_setup(struct platform_device *);
static int tegra_machine_sfc_init(struct snd_soc_pcm_runtime *);
static int tegra_machine_ext_codec_init(struct snd_soc_pcm_runtime *);

#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
static int tegra_machine_compr_set_params(struct snd_compr_stream *);
static void tegra_machine_compr_shutdown(struct snd_compr_stream *);
static int tegra_machine_compr_startup(struct snd_compr_stream *);
#endif

static void tegra_machine_pcm_shutdown(struct snd_pcm_substream *);
static int tegra_machine_pcm_startup(struct snd_pcm_substream *);
static void tegra_machine_pcm_shutdown(struct snd_pcm_substream *);
static int tegra_machine_suspend_pre(struct snd_soc_card *);
static int tegra_machine_pcm_hw_params(struct snd_pcm_substream *,
		struct snd_pcm_hw_params *);
static int tegra_machine_dai_init(struct snd_soc_pcm_runtime *,
		int, int, u64, bool);
static int tegra_machine_set_params(struct snd_soc_card *,
		struct tegra_machine *, int, int, u64);
static int tegra_machine_jack_notifier(struct notifier_block *,
		unsigned long, void *);
static int tegra_machine_codec_get_rate(struct snd_kcontrol *,
		struct snd_ctl_elem_value *);
static int tegra_machine_codec_put_rate(struct snd_kcontrol *,
		struct snd_ctl_elem_value *);
static int tegra_machine_codec_get_format(struct snd_kcontrol *,
		struct snd_ctl_elem_value *);
static int tegra_machine_codec_put_format(struct snd_kcontrol *,
		struct snd_ctl_elem_value *);
static int tegra_machine_codec_get_jack_state(struct snd_kcontrol *,
		struct snd_ctl_elem_value *);
static int tegra_machine_codec_put_jack_state(struct snd_kcontrol *,
		struct snd_ctl_elem_value *);

/* rt565x specific APIs */
static int tegra_rt565x_event_int_spk(struct snd_soc_dapm_widget *,
		struct snd_kcontrol *, int);
static int tegra_rt565x_event_int_mic(struct snd_soc_dapm_widget *,
		struct snd_kcontrol *, int);
static int tegra_rt565x_event_ext_mic(struct snd_soc_dapm_widget *,
		struct snd_kcontrol *, int);
static int tegra_rt565x_event_hp(struct snd_soc_dapm_widget *,
		struct snd_kcontrol *, int);

/* t210 soc data */
static const struct tegra_machine_soc_data soc_data_tegra210 = {
	.num_xbar_dai_links		= TEGRA210_XBAR_DAI_LINKS,

	.admaif_dai_link_start		= TEGRA210_DAI_LINK_ADMAIF1,
	.admaif_dai_link_end		= TEGRA210_DAI_LINK_ADMAIF10,
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
	.adsp_pcm_dai_link_start	= TEGRA210_DAI_LINK_ADSP_PCM1,
	.adsp_pcm_dai_link_end		= TEGRA210_DAI_LINK_ADSP_PCM2,
	.adsp_compr_dai_link_start	= TEGRA210_DAI_LINK_ADSP_COMPR1,
	.adsp_compr_dai_link_end	= TEGRA210_DAI_LINK_ADSP_COMPR2,
#endif
	.sfc_dai_link			= TEGRA210_DAI_LINK_SFC1_RX,

	.is_asrc_available		= false,
	.is_clk_rate_via_dt		= false,
	.write_cdev1_state		= false,
	.write_idle_bias_off_state	= false,

	.get_bclk_ratio			= &tegra_machine_get_bclk_ratio,
	.get_dai_link			= &tegra_machine_get_dai_link,
	.get_codec_conf			= &tegra_machine_get_codec_conf,
	.append_dai_link		= &tegra_machine_append_dai_link,
	.append_codec_conf		= &tegra_machine_append_codec_conf,
};

/* t186 soc data */
static const struct tegra_machine_soc_data soc_data_tegra186 = {
	.num_xbar_dai_links		= TEGRA186_XBAR_DAI_LINKS,

	.admaif_dai_link_start		= TEGRA186_DAI_LINK_ADMAIF1,
	.admaif_dai_link_end		= TEGRA186_DAI_LINK_ADMAIF10,
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
	.adsp_pcm_dai_link_start	= TEGRA186_DAI_LINK_ADSP_PCM1,
	.adsp_pcm_dai_link_end		= TEGRA186_DAI_LINK_ADSP_PCM2,
	.adsp_compr_dai_link_start	= TEGRA186_DAI_LINK_ADSP_COMPR1,
	.adsp_compr_dai_link_end	= TEGRA186_DAI_LINK_ADSP_COMPR2,
#endif
	.sfc_dai_link			= TEGRA186_DAI_LINK_SFC1_RX,

	.is_asrc_available		= true,
	.is_clk_rate_via_dt		= true,
	.write_cdev1_state		= true,
	.write_idle_bias_off_state	= true,

	.get_bclk_ratio			= &tegra_machine_get_bclk_ratio_t18x,
	.get_dai_link			= &tegra_machine_get_dai_link_t18x,
	.get_codec_conf			= &tegra_machine_get_codec_conf_t18x,
	.append_dai_link		= &tegra_machine_append_dai_link_t18x,
	.append_codec_conf		= &tegra_machine_append_codec_conf_t18x,
};

/* structure to match device tree node */
static const struct of_device_id tegra_machine_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-t186ref-mobile-rt565x",
		.data = &soc_data_tegra186 },
	{ .compatible = "nvidia,tegra-audio-t210ref-mobile-rt565x",
		.data = &soc_data_tegra210 },
	{},
};

static const char * const tegra_machine_srate_text[] = {
	"None",
	"8kHz",
	"16kHz",
	"44kHz",
	"48kHz",
	"11kHz",
	"22kHz",
	"24kHz",
	"32kHz",
	"88kHz",
	"96kHz",
	"176kHz",
	"192kHz",
};

static const char * const tegra_machine_format_text[] = {
	"None",
	"16",
	"32",
};

static const struct soc_enum tegra_machine_codec_rate =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra_machine_srate_text),
		tegra_machine_srate_text);

static const struct soc_enum tegra_machine_codec_format =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra_machine_format_text),
		tegra_machine_format_text);

static const int tegra_machine_srate_values[] = {
	0,
	8000,
	16000,
	44100,
	48000,
	11025,
	22050,
	24000,
	32000,
	88200,
	96000,
	176400,
	192000,
};

static struct snd_soc_jack tegra_machine_hp_jack;

static struct snd_soc_ops tegra_machine_pcm_ops = {
	.hw_params	= tegra_machine_pcm_hw_params,
	.startup	= tegra_machine_pcm_startup,
	.shutdown	= tegra_machine_pcm_shutdown,
};

#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
static struct snd_soc_compr_ops tegra_machine_compr_ops = {
	.set_params	= tegra_machine_compr_set_params,
	.startup	= tegra_machine_compr_startup,
	.shutdown	= tegra_machine_compr_shutdown,
};
#endif

static const struct snd_soc_dapm_widget tegra_machine_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("x Int Spk", tegra_rt565x_event_int_spk),
	SND_SOC_DAPM_HP("x Headphone Jack", tegra_rt565x_event_hp),
	SND_SOC_DAPM_MIC("x Int Mic", tegra_rt565x_event_int_mic),
	SND_SOC_DAPM_MIC("x Mic Jack", tegra_rt565x_event_ext_mic),

	SND_SOC_DAPM_SPK("d1 Headphone", NULL),
	SND_SOC_DAPM_SPK("d2 Headphone", NULL),

	SND_SOC_DAPM_HP("x Headphone", NULL),
	SND_SOC_DAPM_HP("y Headphone", NULL),
	SND_SOC_DAPM_HP("z Headphone", NULL),
	SND_SOC_DAPM_HP("m Headphone", NULL),
	SND_SOC_DAPM_HP("n Headphone", NULL),
	SND_SOC_DAPM_HP("o Headphone", NULL),
	SND_SOC_DAPM_HP("e Headphone", NULL),
	SND_SOC_DAPM_HP("s Headphone", NULL),

	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_MIC("x Mic", NULL),
	SND_SOC_DAPM_MIC("y Mic", NULL),
	SND_SOC_DAPM_MIC("z Mic", NULL),
	SND_SOC_DAPM_MIC("m Mic", NULL),
	SND_SOC_DAPM_MIC("n Mic", NULL),
	SND_SOC_DAPM_MIC("o Mic", NULL),
	SND_SOC_DAPM_MIC("a Mic", NULL),
	SND_SOC_DAPM_MIC("b Mic", NULL),
	SND_SOC_DAPM_MIC("c Mic", NULL),
	SND_SOC_DAPM_MIC("d Mic", NULL),
	SND_SOC_DAPM_MIC("e Mic", NULL),
	SND_SOC_DAPM_MIC("s Mic", NULL),
};

static const struct snd_soc_dapm_route tegra_machine_audio_map[] = {
};

#ifdef CONFIG_SWITCH
static const char * const tegra_machine_jack_state_text[] = {
	"None",
	"HS",
	"HP",
};
static struct switch_dev tegra_machine_headset_switch = {
		.name = "h2w",
};
static struct notifier_block tegra_machine_jack_detect_nb = {
	.notifier_call = tegra_machine_jack_notifier,
};
static const struct soc_enum tegra_machine_jack_state =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra_machine_jack_state_text),
		tegra_machine_jack_state_text);
#else
static struct snd_soc_jack_pin tegra_machine_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};
#endif

static const struct snd_kcontrol_new tegra_machine_controls[] = {
	SOC_DAPM_PIN_SWITCH("x Int Spk"),
	SOC_DAPM_PIN_SWITCH("x Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("x Mic Jack"),
	SOC_DAPM_PIN_SWITCH("x Int Mic"),
	SOC_ENUM_EXT("codec-x rate", tegra_machine_codec_rate,
		tegra_machine_codec_get_rate, tegra_machine_codec_put_rate),
	SOC_ENUM_EXT("codec-x format", tegra_machine_codec_format,
		tegra_machine_codec_get_format, tegra_machine_codec_put_format),
#ifdef CONFIG_SWITCH
	SOC_ENUM_EXT("Jack-state", tegra_machine_jack_state,
		tegra_machine_codec_get_jack_state,
		tegra_machine_codec_put_jack_state),
#endif
};

static struct snd_soc_card snd_soc_tegra_card = {
	.name = "tegra-snd-card",
	.owner = THIS_MODULE,
	.suspend_pre = tegra_machine_suspend_pre,
	.controls = tegra_machine_controls,
	.num_controls = ARRAY_SIZE(tegra_machine_controls),
	.dapm_widgets = tegra_machine_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_machine_dapm_widgets),
	.fully_routed = true,
};

static struct snd_soc_pcm_stream tegra_machine_asrc_link_params[] = {
	PARAMS(SNDRV_PCM_FMTBIT_S32_LE, 8),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
	PARAMS(SNDRV_PCM_FMTBIT_S16_LE, 2),
};

static int tegra_rt565x_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
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

static int tegra_rt565x_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static int tegra_rt565x_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
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

static int tegra_rt565x_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	struct tegra_asoc_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}


static int tegra_machine_codec_get_rate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = machine->rate_via_kcontrol;

	return 0;
}

static int tegra_machine_codec_put_rate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	/* set the rate control flag */
	machine->rate_via_kcontrol = ucontrol->value.integer.value[0];

	return 0;
}

static int tegra_machine_codec_get_format(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = machine->fmt_via_kcontrol;

	return 0;
}

static int tegra_machine_codec_put_format(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	/* set the format control flag */
	machine->fmt_via_kcontrol = ucontrol->value.integer.value[0];

	return 0;
}

#ifdef CONFIG_SWITCH
static int tegra_machine_codec_get_jack_state(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tegra_machine_headset_switch.state;
	return 0;
}

static int tegra_machine_codec_put_jack_state(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0] == 0)
		switch_set_state(&tegra_machine_headset_switch, BIT_NO_HEADSET);
	else if (ucontrol->value.integer.value[0] == 1)
		switch_set_state(&tegra_machine_headset_switch, BIT_HEADSET);
	else if (ucontrol->value.integer.value[0] == 2)
		switch_set_state(&tegra_machine_headset_switch,
			BIT_HEADSET_NO_MIC);
	return 0;
}

static int tegra_machine_jack_notifier(struct notifier_block *self,
				  unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_card *card = jack->card;

	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;
	static bool button_pressed;
	struct snd_soc_pcm_runtime *rtd;

	if (machine->is_codec_dummy)
		return NOTIFY_OK;

	rtd = snd_soc_get_pcm_runtime(card, "rt565x-playback");
	if (!rtd) {
		/* spurious interrupt */
		dev_dbg(card->dev, "rt565x dai-link not found\n");
		return -EINVAL;
	}

	dev_dbg(card->dev, "jack status = %d", jack->status);
	if (jack->status & (SND_JACK_BTN_0 | SND_JACK_BTN_1 |
		SND_JACK_BTN_2 | SND_JACK_BTN_3)) {
		button_pressed = true;
		return NOTIFY_OK;
	} else if ((jack->status & SND_JACK_HEADSET) && button_pressed) {
		button_pressed = false;
		return NOTIFY_OK;
	}

	switch (jack->status) {
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

	dev_dbg(card->dev, "switch state to %x\n", state);
	switch_set_state(&tegra_machine_headset_switch, state);
	return NOTIFY_OK;
}
#endif

static int tegra_machine_set_params(struct snd_soc_card *card,
					struct tegra_machine *machine,
					int rate,
					int channels,
					u64 formats)
{
	unsigned int tx_mask = (1 << channels) - 1;
	unsigned int rx_mask = (1 << channels) - 1;
	int idx = 0, err = 0;
	u64 format_k;

	int num_of_dai_links = machine->soc_data->num_xbar_dai_links +
				machine->num_codec_links;
	struct snd_soc_pcm_runtime *rtd;

	format_k = (machine->fmt_via_kcontrol == 2) ?
				(1ULL << SNDRV_PCM_FORMAT_S32_LE) : formats;

	/* update dai link hw_params */
#if KERNEL_VERSION(4, 5, 0) > LINUX_VERSION_CODE
	for (idx = 0; idx < num_of_dai_links;) {
		rtd = &card->rtd[idx];

#else
	list_for_each_entry(rtd, &card->rtd_list, list) {
#endif
		if (rtd->dai_link->params) {
			struct snd_soc_pcm_stream *dai_params;

			dai_params =
			  (struct snd_soc_pcm_stream *)
			  rtd->dai_link->params;

			dai_params->rate_min = rate;
			dai_params->channels_min = channels;
			dai_params->formats = format_k;

			if ((idx >= machine->soc_data->num_xbar_dai_links)
				&& (idx < num_of_dai_links)) {
				unsigned int fmt;
				int bclk_ratio;

				err = 0;
				/* TODO: why below overrite is needed */
				dai_params->formats = formats;

				fmt = rtd->dai_link->dai_fmt;
				bclk_ratio =
					machine->soc_data->get_bclk_ratio(rtd);

				if (bclk_ratio >= 0) {
					err = snd_soc_dai_set_bclk_ratio(
							rtd->cpu_dai,
							bclk_ratio);
				}

				if (err < 0) {
					dev_err(card->dev,
					"Failed to set cpu dai bclk ratio for %s\n",
					rtd->dai_link->name);
				}

				/* set TDM slot mask */
				if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) ==
							SND_SOC_DAIFMT_DSP_A) {
					err = snd_soc_dai_set_tdm_slot(
							rtd->cpu_dai,
							tx_mask, rx_mask, 0, 0);
					if (err < 0) {
						dev_err(card->dev,
						"%s cpu DAI slot mask not set\n",
						rtd->cpu_dai->name);
					}
				}
			}
		}
		idx++;
	}
	return 0;
}
static int tegra_machine_dai_init(struct snd_soc_pcm_runtime *runtime,
					int rate,
					int channels,
					u64 formats,
					bool is_playback)
{
	struct snd_soc_card *card = runtime->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_stream *dai_params;
	unsigned int clk_out_rate = 0, mclk = 0;
	int err, codec_rate, clk_rate;
	struct snd_soc_pcm_runtime *rtd;

	codec_rate = tegra_machine_srate_values[machine->rate_via_kcontrol];
	clk_rate = (machine->rate_via_kcontrol) ? codec_rate : rate;

	if (!machine->soc_data->is_clk_rate_via_dt) {
		/* TODO remove this hardcoding */
		/* aud_mclk, 256 times the sample rate */
		clk_out_rate = clk_rate << 8;
		switch (clk_rate) {
		case 11025:
			mclk = 22579200;
			break;
		case 22050:
		case 44100:
		case 88200:
		case 176400:
			mclk = 45158400;
			break;
		case 8000:
			mclk = 24576000;
			break;
		case 16000:
		case 32000:
		case 48000:
		case 64000:
		case 96000:
		case 192000:
		default:
			mclk = 49152000;
			break;
		}

		err = tegra210_xbar_set_clock(mclk);
		if (err < 0) {
			dev_err(card->dev,
				"Can't configure xbar clock = %d Hz\n", mclk);
			return err;
		}
	}

	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock, clk_rate,
			mclk, clk_out_rate);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	if (machine->soc_data->is_clk_rate_via_dt)
		clk_out_rate = machine->audio_clock.clk_out_rate;

	pr_debug("pll_a_out0 = %d Hz, aud_mclk = %d Hz, codec rate = %d Hz\n",
		machine->audio_clock.set_mclk, clk_out_rate, clk_rate);

	/* TODO: should we pass here clk_rate ? */
	tegra_machine_set_params(card, machine, rate, channels, formats);

	rtd = snd_soc_get_pcm_runtime(card, "rt565x-playback");
	if (rtd) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		dai_params->rate_min = clk_rate;
		dai_params->formats = (machine->fmt_via_kcontrol == 2) ?
			(1ULL << SNDRV_PCM_FORMAT_S32_LE) : formats;

		if (!machine->is_codec_dummy) {
			err = snd_soc_dai_set_sysclk(rtd->codec_dai,
			RT5659_SCLK_S_MCLK, clk_out_rate, SND_SOC_CLOCK_IN);
			if (err < 0) {
				dev_err(card->dev, "codec_dai clock not set\n");
				return err;
			}
		}
	}

	/* TODO: remove below spdif links if clk_rate is passed
	 *	in tegra_machine_set_params
	 */
	rtd = snd_soc_get_pcm_runtime(card, "spdif-dit-1");
	if (rtd) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		dai_params->rate_min = clk_rate;
	}

	/* set clk rate for i2s3 dai link*/
	rtd = snd_soc_get_pcm_runtime(card, "spdif-dit-2");
	if (rtd) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		dai_params->rate_min = clk_rate;
	}

	rtd = snd_soc_get_pcm_runtime(card, "spdif-dit-3");
	if (rtd) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		dai_params->rate_min = clk_rate;
	}

	rtd = snd_soc_get_pcm_runtime(card, "dspk-playback-r");
	if (rtd) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		if (!strcmp(rtd->codec_dai->name, "tas2552-amplifier")) {
			err = snd_soc_dai_set_sysclk(rtd->codec_dai,
				TAS2552_PDM_CLK_IVCLKIN, clk_out_rate,
				SND_SOC_CLOCK_IN);
			if (err < 0) {
				dev_err(card->dev, "codec_dai clock not set\n");
				return err;
			}
		}
	}

	rtd = snd_soc_get_pcm_runtime(card, "dspk-playback-l");
	if (rtd) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		if (!strcmp(rtd->codec_dai->name, "tas2552-amplifier")) {
			err = snd_soc_dai_set_sysclk(rtd->codec_dai,
				TAS2552_PDM_CLK_IVCLKIN, clk_out_rate,
				SND_SOC_CLOCK_IN);
			if (err < 0) {
				dev_err(card->dev, "codec_dai clock not set\n");
				return err;
			}
		}
	}

	rtd = snd_soc_get_pcm_runtime(card, "spdif-playback");
	if (rtd && (clk_rate >= 32000)) {
		dai_params =
		(struct snd_soc_pcm_stream *)rtd->dai_link->params;

		if (is_playback) {
			err = snd_soc_dai_set_sysclk(rtd->cpu_dai, 0,
						clk_rate, SND_SOC_CLOCK_OUT);
			if (err < 0) {
				dev_err(card->dev, "cpu_dai out clock not set\n");
				return err;
			}
		} else {
			err = snd_soc_dai_set_sysclk(rtd->cpu_dai, 0,
						clk_rate, SND_SOC_CLOCK_IN);
			if (err < 0) {
				dev_err(card->dev, "cpu_dai in clock not set\n");
				return err;
			}
		}
	}

	return 0;
}

static int tegra_machine_pcm_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	int err;
	bool is_playback;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		is_playback = true;
	else
		is_playback = false;

	err = tegra_machine_dai_init(rtd, params_rate(params),
			params_channels(params),
			(1ULL << (params_format(params))),
			is_playback);
	if (err < 0) {
		dev_err(card->dev, "Failed dai init\n");
		return err;
	}

	return 0;
}

static int tegra_machine_pcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);

	return 0;
}

static void tegra_machine_pcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);
}

static int tegra_machine_suspend_pre(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;

	/* DAPM dai link stream work for non pcm links */
#if KERNEL_VERSION(4, 5, 0) > LINUX_VERSION_CODE
	unsigned int idx;

	for (idx = 0; idx < card->num_rtd; idx++) {
		rtd = &card->rtd[idx];
#else
	list_for_each_entry(rtd, &card->rtd_list, list) {
#endif
		if (rtd->dai_link->params)
			INIT_DELAYED_WORK(&rtd->delayed_work, NULL);
	}

	return 0;
}

static int tegra_machine_dspk_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	int err;

	err = tegra_alt_asoc_utils_set_extern_parent(&machine->audio_clock,
							"pll_a_out0");
	if (err < 0)
		dev_err(card->dev, "Failed to set extern clk parent\n");

	snd_soc_dapm_sync(dapm);
	return err;
}

#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
static int tegra_machine_compr_startup(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);

	return 0;
}

static void tegra_machine_compr_shutdown(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);
}

static int tegra_machine_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_codec codec_params;
	int err;
	bool is_playback;

	if (platform->driver->compr_ops &&
		platform->driver->compr_ops->get_params) {
		err = platform->driver->compr_ops->get_params(cstream,
			&codec_params);
		if (err < 0) {
			dev_err(card->dev, "Failed to get compr params\n");
			return err;
		}
	} else {
		dev_err(card->dev, "compr ops not set\n");
		return -EINVAL;
	}

	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		is_playback = true;
	else
		is_playback = false;

	err = tegra_machine_dai_init(rtd, codec_params.sample_rate,
			codec_params.ch_out, SNDRV_PCM_FMTBIT_S16_LE,
			is_playback);
	if (err < 0) {
		dev_err(card->dev, "Failed dai init\n");
		return err;
	}

	return 0;
}
#endif

static int tegra_machine_ext_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	int err;

	err = tegra_alt_asoc_utils_set_extern_parent(&machine->audio_clock,
							"pll_a_out0");
	if (err < 0) {
		dev_err(card->dev, "Failed to set extern clk parent\n");
		return err;
	}

	err = snd_soc_card_jack_new(card, "Headphone Jack", SND_JACK_HEADPHONE,
					&tegra_machine_hp_jack, NULL, 0);
	if (err) {
		dev_err(card->dev, "Headset Jack creation failed %d\n", err);
		return err;
	}

#ifndef CONFIG_SWITCH
	err = snd_soc_jack_add_pins(&tegra_machine_hp_jack,
					ARRAY_SIZE(tegra_machine_hp_jack_pins),
					tegra_machine_hp_jack_pins);
	if (err) {
		dev_err(card->dev, "snd_soc_jack_add_pins failed %d\n", err);
		return err;
	}
#else
	snd_soc_jack_notifier_register(&tegra_machine_hp_jack,
		&tegra_machine_jack_detect_nb);
#endif

	/* single button supporting play/pause */
	snd_jack_set_key(tegra_machine_hp_jack.jack,
		SND_JACK_BTN_0, KEY_MEDIA);

	/* multiple buttons supporting play/pause and volume up/down */
	snd_jack_set_key(tegra_machine_hp_jack.jack,
		SND_JACK_BTN_1, KEY_MEDIA);
	snd_jack_set_key(tegra_machine_hp_jack.jack,
		SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(tegra_machine_hp_jack.jack,
		SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	snd_soc_dapm_sync(&card->dapm);

	return 0;
}

static int tegra_machine_sfc_init(struct snd_soc_pcm_runtime *rtd)
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

	return err;
}

static void dai_link_setup(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra_new_codec_conf = NULL;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *tegra_machine_codec_links = NULL;
	int i;

	/* set new codec links and conf */
	tegra_machine_codec_links = tegra_machine_new_codec_links(pdev,
		tegra_machine_codec_links,
		&machine->num_codec_links);
	if (!tegra_machine_codec_links)
		goto err_alloc_dai_link;

	/* set codec init */
	for (i = 0; i < machine->num_codec_links; i++) {
		if (tegra_machine_codec_links[i].name) {
			if (strstr(tegra_machine_codec_links[i].name,
				"rt565x-playback"))
				tegra_machine_codec_links[i].init =
					tegra_machine_ext_codec_init;
			else if (strstr(tegra_machine_codec_links[i].name,
				"dspk-playback-r"))
				tegra_machine_codec_links[i].init =
					tegra_machine_dspk_init;
			else if (strstr(tegra_machine_codec_links[i].name,
				"dspk-playback-l"))
				tegra_machine_codec_links[i].init =
					tegra_machine_dspk_init;
		}
	}

	tegra_new_codec_conf = tegra_machine_new_codec_conf(pdev,
		tegra_new_codec_conf,
		&machine->num_codec_links);
	if (!tegra_new_codec_conf)
		goto err_alloc_dai_link;

	/* get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = machine->soc_data->get_dai_link();
	if (!tegra_machine_dai_links)
		goto err_alloc_dai_link;

	tegra_machine_codec_conf = machine->soc_data->get_codec_conf();
	if (!tegra_machine_codec_conf)
		goto err_alloc_dai_link;

	/* set ADMAIF dai_ops */
	for (i = machine->soc_data->admaif_dai_link_start;
		i <= machine->soc_data->admaif_dai_link_end; i++)
		tegra_machine_set_dai_ops(i, &tegra_machine_pcm_ops);

	/* set sfc dai_init */
	tegra_machine_set_dai_init(machine->soc_data->sfc_dai_link,
		&tegra_machine_sfc_init);
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
	/* set ADSP PCM/COMPR */
	for (i = machine->soc_data->adsp_pcm_dai_link_start;
		i <= machine->soc_data->adsp_pcm_dai_link_end; i++) {
		tegra_machine_set_dai_ops(i, &tegra_machine_pcm_ops);
	}

	/* set ADSP COMPR */
	for (i = machine->soc_data->adsp_compr_dai_link_start;
		i <= machine->soc_data->adsp_compr_dai_link_end; i++) {
		tegra_machine_set_dai_compr_ops(i,
			&tegra_machine_compr_ops);
	}
#endif

	if (machine->soc_data->is_asrc_available) {
	/* set ASRC params. The default is 2 channels */
		for (i = 0; i < 6; i++) {
			tegra_machine_set_dai_params(TEGRA186_DAI_LINK_ASRC1_TX1
				+ i, (struct snd_soc_pcm_stream *)
				&tegra_machine_asrc_link_params[i]);
			tegra_machine_set_dai_params(TEGRA186_DAI_LINK_ASRC1_RX1
				 + i, (struct snd_soc_pcm_stream *)
				&tegra_machine_asrc_link_params[i]);
		}
	}

	/* append machine specific dai_links */
	card->num_links = machine->soc_data->append_dai_link(
		tegra_machine_codec_links, 2 * machine->num_codec_links);
	tegra_machine_dai_links = machine->soc_data->get_dai_link();
	card->dai_link = tegra_machine_dai_links;

	/* append machine specific codec_conf */
	card->num_configs = machine->soc_data->append_codec_conf(
		tegra_new_codec_conf, machine->num_codec_links);
	tegra_machine_codec_conf = machine->soc_data->get_codec_conf();
	card->codec_conf = tegra_machine_codec_conf;

	return;

err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
}

static int tegra_machine_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_card;
	struct tegra_machine *machine;
	struct tegra_asoc_platform_data *pdata = NULL;
	struct snd_soc_codec *codec = NULL;
	int ret = 0;
	const char *codec_dai_name;
	struct snd_soc_pcm_runtime *rtd;
	const struct of_device_id *match;

	match = of_match_device(tegra_machine_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	if (!np) {
		dev_err(&pdev->dev, "No device tree node for tegra machine driver");
		return -ENODEV;
	}

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_machine),
				   GFP_KERNEL);
	if (!machine) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Can't allocate struct for tegra_machine\n");
		goto err;
	}

	machine->soc_data = (struct tegra_machine_soc_data *)match->data;

	if (!machine->soc_data->get_dai_link ||
		!machine->soc_data->get_bclk_ratio ||
		!machine->soc_data->get_codec_conf ||
		!machine->soc_data->append_dai_link ||
		!machine->soc_data->append_codec_conf) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "Error: callback APIs are missing\n");
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);
	machine->is_codec_dummy = 0;

	if (machine->soc_data->write_cdev1_state)
		machine->audio_clock.clk_cdev1_state = 0;

	if (machine->soc_data->write_idle_bias_off_state)
		card->dapm.idle_bias_off = true;

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
				"nvidia,audio-routing");
	if (ret)
		goto err;

	if (machine->soc_data->is_clk_rate_via_dt) {
		if (of_property_read_u32(np, "nvidia,num-clk",
				   &machine->audio_clock.num_clk) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,num-clk\n");
			ret = -ENODEV;
			goto err;
		}

		if (of_property_read_u32_array(np, "nvidia,clk-rates",
				(u32 *)&machine->audio_clock.clk_rates,
				machine->audio_clock.num_clk) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,clk-rates\n");
			ret = -ENODEV;
			goto err;
		}
	}

	dai_link_setup(pdev);

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = tegra_alt_asoc_switch_register(&tegra_machine_headset_switch);
	if (ret < 0)
		goto err_alloc_dai_link;
#endif

	pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct tegra_asoc_platform_data),
				GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev,
			"Can't allocate tegra_asoc_platform_data struct\n");
		return -ENOMEM;
	}

	pdata->gpio_codec1 = pdata->gpio_codec2 = pdata->gpio_codec3 =
	pdata->gpio_spkr_en = pdata->gpio_hp_mute =
	pdata->gpio_int_mic_en = pdata->gpio_ext_mic_en = -1;

	machine->pdata = pdata;
	machine->pcard = card;

	ret = tegra_alt_asoc_utils_init(&machine->audio_clock,
					&pdev->dev,
					card);
	if (ret)
		goto err_alloc_dai_link;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}

	rtd = snd_soc_get_pcm_runtime(card, "rt565x-playback");
	if (!rtd)
		dev_warn(&pdev->dev, "codec link not defined - codec not part of sound card");
	else {
		codec = rtd->codec;
		codec_dai_name = rtd->dai_link->codec_dai_name;

		dev_info(&pdev->dev,
			"codec-dai \"%s\" registered\n", codec_dai_name);
		if (!strcmp("dit-hifi", codec_dai_name)) {
			dev_info(&pdev->dev, "This is a dummy codec\n");
			machine->is_codec_dummy = 1;
		}
	}

	if (!machine->is_codec_dummy) {
		/* setup for jack detection only in non-dummy case */
		rt5659_set_jack_detect(codec, &tegra_machine_hp_jack);
	}

	return 0;

err_fini_utils:
	tegra_alt_asoc_utils_fini(&machine->audio_clock);
err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
err:
	return ret;
}

static int tegra_machine_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
	tegra_alt_asoc_utils_fini(&machine->audio_clock);

	return 0;
}

static struct platform_driver tegra_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_machine_of_match,
	},
	.probe = tegra_machine_driver_probe,
	.remove = tegra_machine_driver_remove,
};
module_platform_driver(tegra_asoc_machine_driver);

MODULE_AUTHOR("Mohan Kumar <mkumard@nvidia.com>, Sameer Pujar <spujar@nvidia.com>");
MODULE_DESCRIPTION("Tegra ASoC machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_machine_of_match);
