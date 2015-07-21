/*
 * tegra_virt_t210ref_pcm.c - Tegra T210 reference virtual PCM driver
 *
 * Copyright (c) 2015 NVIDIA CORPORATION.  All rights reserved.
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
#include <sound/soc.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>

#include "tegra210_virt_alt_admaif.h"
#include "tegra_virt_alt_ivc.h"

#define DAI_NAME(i)		"AUDIO" #i
#define STREAM_NAME		"playback"
#define CODEC_NAME		"spdif-dit.0"
#define LINK_CPU_NAME		DRV_NAME
#define CPU_DAI_NAME(i)		"ADMAIF" #i
#define CODEC_DAI_NAME		"dit-hifi"
#define MAX_APBIF_IDS		10
#define PLATFORM_NAME LINK_CPU_NAME

static struct snd_soc_pcm_stream default_params = {
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_dai_link tegra_virt_t210ref_pcm_links[] = {
	{
		/* 0 */
		.name = DAI_NAME(1),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(1),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 1 */
		.name = DAI_NAME(2),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(2),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 2 */
		.name = DAI_NAME(3),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(3),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 3 */
		.name = DAI_NAME(4),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(4),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 4 */
		.name = DAI_NAME(5),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(5),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 5 */
		.name = DAI_NAME(6),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(6),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 6 */
		.name = DAI_NAME(7),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(7),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 7 */
		.name = DAI_NAME(8),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(8),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 8 */
		.name = DAI_NAME(9),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(9),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
	{
		/* 9 */
		.name = DAI_NAME(10),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(10),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
	},
};

static const struct of_device_id tegra_virt_t210ref_pcm_of_match[] = {
	{ .compatible = "nvidia,tegra210-virt-pcm", },
	{},
};

static struct snd_soc_card snd_soc_tegra_virt_t210ref_pcm = {
	.name = "t210ref-virt-pcm",
	.owner = THIS_MODULE,
	.dai_link = tegra_virt_t210ref_pcm_links,
	.num_links = ARRAY_SIZE(tegra_virt_t210ref_pcm_links),
	.fully_routed = true,
};

static void tegra_virt_t210ref_pcm_set_dai_params(
		struct snd_soc_dai_link *dai_link,
		struct snd_soc_pcm_stream *user_params,
		unsigned int dai_id)
{
	dai_link[dai_id].params = user_params;
}

static int tegra_virt_t210ref_pcm_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_virt_t210ref_pcm;
	int i, ret = 0;
	int admaif_ch_num = 0;
	unsigned int admaif_ch_list[MAX_ADMAIF_IDS];

	card->dev = &pdev->dev;

	if (tegra210_virt_admaif_register_component(pdev)) {
		dev_err(&pdev->dev, "Failed register admaif component\n");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"admaif_ch_num", &admaif_ch_num)) {
		dev_err(&pdev->dev, "number of admaif channels is not set\n");
		return -EINVAL;
	}

	if (of_property_read_string(pdev->dev.of_node,
		"cardname", &card->name))
			dev_warn(&pdev->dev, "Using default card name %s\n",
				card->name);

	if (admaif_ch_num > 0) {

		if (of_property_read_u32_array(pdev->dev.of_node,
						"admaif_ch_list",
						admaif_ch_list,
						admaif_ch_num)) {
			dev_err(&pdev->dev, "admaif_ch_list os not populated\n");
			return -EINVAL;
		}

		for (i = 0; i < admaif_ch_num; i++) {
			tegra_virt_t210ref_pcm_set_dai_params(
						tegra_virt_t210ref_pcm_links,
						NULL,
						(admaif_ch_list[i] - 1));
		}
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		return -EINVAL;
	}

	tegra_ape_pd_add_device(&pdev->dev);
	pm_genpd_dev_need_save(&pdev->dev, true);
	pm_genpd_dev_need_restore(&pdev->dev, true);
	pm_runtime_forbid(&pdev->dev);

	return ret;
}

static int tegra_virt_t210ref_pcm_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver tegra_virt_t210ref_pcm_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table =
			of_match_ptr(tegra_virt_t210ref_pcm_of_match),
	},
	.probe = tegra_virt_t210ref_pcm_driver_probe,
	.remove = tegra_virt_t210ref_pcm_driver_remove,
};
module_platform_driver(tegra_virt_t210ref_pcm_driver);

MODULE_AUTHOR("Paresh Anandathirtha <paresha@nvidia.com>");
MODULE_DESCRIPTION("Tegra T210ref virt pcm driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, tegra_virt_t210ref_pcm_of_match);
MODULE_ALIAS("platform:" DRV_NAME);
