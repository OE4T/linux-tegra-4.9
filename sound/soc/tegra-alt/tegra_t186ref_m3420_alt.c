/*
 * tegra_t186ref_m3420_alt.c - Tegra t186ref m3402 Machine driver
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

#include <linux/clk.h>
#include <linux/module.h>
#include <sound/soc.h>

#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"
#include "tegra_asoc_machine_alt_t18x.h"
#include "tegra210_xbar_alt.h"

#define DRV_NAME "tegra-snd-t186ref-m3420"

struct tegra_t186ref_m3420 {
	struct tegra_asoc_audio_clock_info audio_clock;
	struct mutex lock;
	unsigned int num_codec_links;
	unsigned int clk_ena_count;
};

static int tegra_t186ref_m3420_clocks_init(struct snd_soc_pcm_runtime *rtd,
					   int srate)
{
	struct snd_soc_card *card = rtd->card;
	struct tegra_t186ref_m3420 *machine = snd_soc_card_get_drvdata(card);
	unsigned int mclk, clk_out_rate;
	int err = 0;

	/*
	 * Clocks only need to be configured once for any playback sessions
	 * and so if they are enabled, then they have been configured and
	 * we are done.
	 */
	if (machine->clk_ena_count)
		goto out;

	switch (srate) {
	case 32000:
	case 44100:
	case 48000:
		machine->audio_clock.mclk_scale = 256;
		break;
	case 96000:
		machine->audio_clock.mclk_scale = 128;
		break;
	default:
		return -EINVAL;
	}

	clk_out_rate = srate * machine->audio_clock.mclk_scale;
	mclk = clk_out_rate * 2;

	tegra_alt_asoc_utils_set_parent(&machine->audio_clock, true);

	err = tegra_alt_asoc_utils_set_rate(&machine->audio_clock,
					    srate, mclk, clk_out_rate);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	err = tegra210_xbar_set_clock(mclk);
	if (err < 0) {
		dev_err(card->dev, "Can't configure xbar clocks\n");
		return err;
	}

	/*
	 * Clocks are enabled here and NOT in machine start-up because for
	 * Tegra186 we cannot the pll_a's p-divider if the pll is enabled
	 * and we cannot get the exact frequency we need. So enable it now
	 * once we have set the clock rates for the playback scenario.
	 */
	err = tegra_alt_asoc_utils_clk_enable(&machine->audio_clock);
	if (err < 0) {
		dev_err(card->dev, "Can't enable clocks\n");
		return err;
	}

out:
	machine->clk_ena_count++;

	return 0;
}

static int tegra_t186ref_m3420_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params,
					 char *name)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_pcm_stream *stream;
	struct tegra_t186ref_m3420 *machine = snd_soc_card_get_drvdata(card);
	unsigned int idx;
	int err = 0;

	mutex_lock(&machine->lock);
	err = tegra_t186ref_m3420_clocks_init(rtd, params_rate(params));
	mutex_unlock(&machine->lock);
	if (err)
		return err;

	idx = tegra_machine_get_codec_dai_link_idx_t18x(name);

	stream = (struct snd_soc_pcm_stream *)card->rtd[idx].dai_link->params;
	stream->rate_min = params_rate(params);

	return snd_soc_dai_set_sysclk(card->rtd[idx].codec_dai, 0,
				      machine->audio_clock.clk_out_rate,
				      SND_SOC_CLOCK_IN);
}

static int tegra_t186ref_m3420_i2s1_hw_params(
					struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return tegra_t186ref_m3420_hw_params(substream, params,
					     "i2s-playback-1");
}

static int tegra_t186ref_m3420_i2s2_hw_params(
					struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return tegra_t186ref_m3420_hw_params(substream, params,
					     "i2s-playback-2");
}

static int tegra_t186ref_m3420_i2s3_hw_params(
					struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return tegra_t186ref_m3420_hw_params(substream, params,
					     "i2s-playback-3");
}

static int tegra_t186ref_m3420_i2s4_hw_params(
					struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	return tegra_t186ref_m3420_hw_params(substream, params,
					     "i2s-playback-4");
}

static void tegra_t186ref_m3420_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_t186ref_m3420 *machine;

	machine = snd_soc_card_get_drvdata(rtd->card);

	mutex_lock(&machine->lock);

	if (machine->clk_ena_count > 0) {
		machine->clk_ena_count--;

		if (!machine->clk_ena_count)
			tegra_alt_asoc_utils_clk_disable(&machine->audio_clock);
	}

	mutex_unlock(&machine->lock);
}

static struct snd_soc_ops tegra_t186ref_m3420_i2s1_ops = {
	.hw_params = tegra_t186ref_m3420_i2s1_hw_params,
	.shutdown = tegra_t186ref_m3420_shutdown,
};

static struct snd_soc_ops tegra_t186ref_m3420_i2s2_ops = {
	.hw_params = tegra_t186ref_m3420_i2s2_hw_params,
	.shutdown = tegra_t186ref_m3420_shutdown,
};

static struct snd_soc_ops tegra_t186ref_m3420_i2s3_ops = {
	.hw_params = tegra_t186ref_m3420_i2s3_hw_params,
	.shutdown = tegra_t186ref_m3420_shutdown,
};

static struct snd_soc_ops tegra_t186ref_m3420_i2s4_ops = {
	.hw_params = tegra_t186ref_m3420_i2s4_hw_params,
	.shutdown = tegra_t186ref_m3420_shutdown,
};

static const struct snd_soc_dapm_widget tegra_m3420_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone-1", NULL),
	SND_SOC_DAPM_HP("Headphone-2", NULL),
	SND_SOC_DAPM_HP("Headphone-3", NULL),
	SND_SOC_DAPM_HP("Headphone-4", NULL),
};

static int tegra_t186ref_m3420_suspend_pre(struct snd_soc_card *card)
{
	unsigned int idx;

	/* DAPM dai link stream work for non pcm links */
	for (idx = 0; idx < card->num_rtd; idx++) {
		if (card->rtd[idx].dai_link->params)
			INIT_DELAYED_WORK(&card->rtd[idx].delayed_work, NULL);
	}

	return 0;
}

static struct snd_soc_card snd_soc_tegra_t186ref_m3420 = {
	.name = "tegra-t186ref-m3420",
	.owner = THIS_MODULE,
	.suspend_pre = tegra_t186ref_m3420_suspend_pre,
	.dapm_widgets = tegra_m3420_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_m3420_dapm_widgets),
	.fully_routed = true,
};

static void dai_set_ops(struct snd_soc_ops *ops, unsigned int idx_start,
			unsigned int idx_end)
{
	unsigned int i;

	for (i = idx_start; i <= idx_end; i++)
		tegra_machine_set_dai_ops(i, ops);
}

static int dai_link_setup(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t186ref_m3420 *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra_t186ref_codec_conf = NULL;
	struct snd_soc_dai_link *dai_links = NULL;
	struct snd_soc_dai_link *codec_links = NULL;
	int i, err = -ENODEV;

	/* set new codec links and conf */
	codec_links = tegra_machine_new_codec_links(pdev, codec_links,
						    &machine->num_codec_links);
	if (!codec_links)
		return err;

	tegra_t186ref_codec_conf = tegra_machine_new_codec_conf(pdev,
		tegra_t186ref_codec_conf,
		&machine->num_codec_links);
	if (!tegra_t186ref_codec_conf)
		goto err_alloc_dai_link;

	/* get the xbar dai link/codec conf structure */
	dai_links = tegra_machine_get_dai_link_t18x();
	if (!dai_links)
		goto err_alloc_codec_conf;

	tegra_machine_codec_conf = tegra_machine_get_codec_conf_t18x();
	if (!tegra_machine_codec_conf)
		goto err_alloc_codec_conf;

	/* set ADMAIF dai_ops */
	dai_set_ops(&tegra_t186ref_m3420_i2s1_ops, TEGRA186_DAI_LINK_ADMAIF1,
		    TEGRA186_DAI_LINK_ADMAIF20);

	for (i = 0; i < machine->num_codec_links; i++) {
		if (!codec_links[i].name)
			continue;

		if (strstr(codec_links[i].name, "i2s-playback-1")) {
			dai_set_ops(&tegra_t186ref_m3420_i2s1_ops,
				    TEGRA186_DAI_LINK_ADMAIF1,
				    TEGRA186_DAI_LINK_ADMAIF2);
		} else if (strstr(codec_links[i].name, "i2s-playback-2")) {
			dai_set_ops(&tegra_t186ref_m3420_i2s2_ops,
				    TEGRA186_DAI_LINK_ADMAIF3,
				    TEGRA186_DAI_LINK_ADMAIF4);
		} else if (strstr(codec_links[i].name, "i2s-playback-3")) {
			dai_set_ops(&tegra_t186ref_m3420_i2s3_ops,
				    TEGRA186_DAI_LINK_ADMAIF5,
				    TEGRA186_DAI_LINK_ADMAIF6);
		} else if (strstr(codec_links[i].name, "i2s-playback-4")) {
			dai_set_ops(&tegra_t186ref_m3420_i2s4_ops,
				    TEGRA186_DAI_LINK_ADMAIF7,
				    TEGRA186_DAI_LINK_ADMAIF8);
		}
	}

	/* append t186ref specific dai_links */
	card->num_links = tegra_machine_append_dai_link_t18x(codec_links,
						2 * machine->num_codec_links);
	dai_links = tegra_machine_get_dai_link_t18x();
	card->dai_link = dai_links;

	/* append t186ref specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf_t18x(tegra_t186ref_codec_conf,
			machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf_t18x();
	card->codec_conf = tegra_machine_codec_conf;

	return 0;

err_alloc_codec_conf:
	tegra_machine_remove_codec_conf();
err_alloc_dai_link:
	tegra_machine_remove_dai_link();

	return err;
}

static int tegra_t186ref_m3420_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_t186ref_m3420;
	struct tegra_t186ref_m3420 *machine;
	struct tegra_asoc_audio_clock_info *clocks;
	int err = 0;

	if (!np) {
		dev_err(&pdev->dev, "No device tree node found!");
		return -ENODEV;
	}

	machine = devm_kzalloc(&pdev->dev, sizeof(struct tegra_t186ref_m3420),
			       GFP_KERNEL);
	if (!machine)
		return -ENOMEM;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);
	mutex_init(&machine->lock);
	clocks = &machine->audio_clock;

	err = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (err)
		return err;

	err = snd_soc_of_parse_audio_routing(card, "nvidia,audio-routing");
	if (err)
		return err;

	if (of_property_read_u32(np, "nvidia,num-clk", &clocks->num_clk) < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,num-clk\n");
		return -ENODEV;
	}

	if (of_property_read_u32_array(np, "nvidia,clk-rates",
				       clocks->clk_rates,
				       clocks->num_clk) < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,clk-rates\n");
		return -ENODEV;
	}

	err = dai_link_setup(pdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to configured DAIs!\n");
		return err;
	}

	err = tegra_alt_asoc_utils_init(clocks, &pdev->dev, card);
	if (err)
		goto err_remove_dai_link;

	err = snd_soc_register_card(card);
	if (err) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", err);
		goto err_remove_dai_link;
	}

	return 0;

err_remove_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();

	return err;
}

static int tegra_t186ref_m3420_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();

	return 0;
}

static const struct of_device_id tegra_t186ref_m3420_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-t186ref-m3420", },
	{ },
};

static struct platform_driver tegra_t186ref_m3420_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_t186ref_m3420_of_match,
	},
	.probe = tegra_t186ref_m3420_driver_probe,
	.remove = tegra_t186ref_m3420_driver_remove,
};
module_platform_driver(tegra_t186ref_m3420_driver);

MODULE_AUTHOR("Jon Hunter <jonathanh@nvidia.com>");
MODULE_DESCRIPTION("Tegra+t186ref_m3420 machine ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_t186ref_m3420_of_match);
