/*
 * tegra210_sfc_alt.c - Tegra210 SFC driver
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
#include <linux/pinctrl/consumer.h>
#include <linux/of_device.h>

#include "tegra210_xbar_alt.h"
#include "tegra210_sfc_alt.h"

#define DRV_NAME "tegra210-sfc"

static int tegra210_sfc_runtime_suspend(struct device *dev)
{
	struct tegra210_sfc *sfc = dev_get_drvdata(dev);

	regcache_cache_only(sfc->regmap, true);

	clk_disable_unprepare(sfc->clk_sfc);

	return 0;
}

static int tegra210_sfc_runtime_resume(struct device *dev)
{
	struct tegra210_sfc *sfc = dev_get_drvdata(dev);

	int ret;

	ret = clk_prepare_enable(sfc->clk_sfc);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	regcache_cache_only(sfc->regmap, false);

	return 0;
}

static int tegra210_sfc_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct tegra210_sfc *sfc = snd_soc_dai_get_drvdata(dai);
	unsigned int value;

	switch (freq) {
	case 8000:
		value = TEGRA210_SFC_FS8;
		break;
	case 11025:
		value = TEGRA210_SFC_FS11_025;
		break;
	case 16000:
		value = TEGRA210_SFC_FS16;
		break;
	case 22050:
		value = TEGRA210_SFC_FS22_05;
		break;
	case 24000:
		value = TEGRA210_SFC_FS24;
		break;
	case 32000:
		value = TEGRA210_SFC_FS32;
		break;
	case 44100:
		value = TEGRA210_SFC_FS44_1;
		break;
	case 48000:
		value = TEGRA210_SFC_FS48;
		break;
	case 64000:
		value = TEGRA210_SFC_FS64;
		break;
	case 88200:
		value = TEGRA210_SFC_FS88_2;
		break;
	case 96000:
		value = TEGRA210_SFC_FS96;
		break;
	case 176400:
		value = TEGRA210_SFC_FS176_4;
		break;
	case 192000:
		value = TEGRA210_SFC_FS192;
		break;
	default:
		value = TEGRA210_SFC_FS8;
		break;
	}

	if (dir == SND_SOC_CLOCK_OUT)
		sfc->srate_out = value;
	else if (dir == SND_SOC_CLOCK_IN)
		sfc->srate_in = value;

	return 0;
}

static int tegra210_sfc_set_audio_cif(struct tegra210_sfc *sfc,
				struct snd_pcm_hw_params *params,
				unsigned int reg)
{
	int channels, audio_bits;
	struct tegra210_xbar_cif_conf cif_conf;

	channels = params_channels(params);
	if (channels < 2)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_16;
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

	sfc->soc_data->set_audio_cif(sfc->regmap, reg, &cif_conf);

	return 0;
}

static int tegra210_sfc_in_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_sfc *sfc = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = tegra210_sfc_set_audio_cif(sfc, params,
				TEGRA210_SFC_AXBAR_RX_CIF_CTRL);
	if (ret) {
		dev_err(dev, "Can't set SFC RX CIF: %d\n", ret);
		return ret;
	}

	regmap_write(sfc->regmap, TEGRA210_SFC_AXBAR_RX_FREQ, sfc->srate_in);
	return ret;
}

static int tegra210_sfc_out_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_sfc *sfc = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = tegra210_sfc_set_audio_cif(sfc, params,
				TEGRA210_SFC_AXBAR_TX_CIF_CTRL);
	if (ret) {
		dev_err(dev, "Can't set SFC TX CIF: %d\n", ret);
		return ret;
	}

	if (sfc->srate_out < 0) {
		dev_err(dev, "SFC%d output rate not set: %d\n",
			dev->id, -EINVAL);
		return -EINVAL;
	}

	regmap_write(sfc->regmap, TEGRA210_SFC_AXBAR_TX_FREQ, sfc->srate_out);
	return ret;
}

static int tegra210_sfc_get_srate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra210_sfc *sfc = snd_soc_codec_get_drvdata(codec);

	/* get the sfc output rate */
	ucontrol->value.integer.value[0] = sfc->srate_out + 1;

	return 0;
}

static int tegra210_sfc_put_srate(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tegra210_sfc *sfc = snd_soc_codec_get_drvdata(codec);

	/* update the sfc output rate */
	sfc->srate_out = ucontrol->value.integer.value[0] - 1;

	return 0;
}

static int tegra210_sfc_codec_probe(struct snd_soc_codec *codec)
{
	struct tegra210_sfc *sfc = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = sfc->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_dai_ops tegra210_sfc_in_dai_ops = {
	.hw_params	= tegra210_sfc_in_hw_params,
	.set_sysclk	= tegra210_sfc_set_dai_sysclk,
};

static struct snd_soc_dai_ops tegra210_sfc_out_dai_ops = {
	.hw_params	= tegra210_sfc_out_hw_params,
};

static struct snd_soc_dai_driver tegra210_sfc_dais[] = {
	{
		.name = "CIF",
		.playback = {
			.stream_name = "SFC Receive",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tegra210_sfc_in_dai_ops,
	},
	{
		.name = "DAP",
		.capture = {
			.stream_name = "SFC Transmit",
			.channels_min = 1,
			.channels_max = 16,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tegra210_sfc_out_dai_ops,
	}
};

static const struct snd_soc_dapm_widget tegra210_sfc_widgets[] = {
	SND_SOC_DAPM_AIF_IN("SFC RX", NULL, 0, SND_SOC_NOPM,
				0, 0),
	SND_SOC_DAPM_AIF_OUT("SFC TX", NULL, 0, TEGRA210_SFC_ENABLE,
				TEGRA210_SFC_EN_SHIFT, 0),
};

static const struct snd_soc_dapm_route tegra210_sfc_routes[] = {
	{ "SFC RX",       NULL, "SFC Receive" },
	{ "SFC TX",       NULL, "SFC RX" },
	{ "SFC Transmit", NULL, "SFC TX" },
};

static const char * const tegra210_sfc_srate_text[] = {
	"None",
	"8kHz",
	"11kHz",
	"16kHz",
	"22kHz",
	"24kHz",
	"32kHz",
	"44kHz",
	"48kHz",
	"64kHz",
	"88kHz",
	"96kHz",
	"176kHz",
	"192kHz",
};

static const struct soc_enum tegra210_sfc_srate =
	SOC_ENUM_SINGLE_EXT(14, tegra210_sfc_srate_text);

static const struct snd_kcontrol_new tegra210_sfc_controls[] = {
	SOC_ENUM_EXT("output rate", tegra210_sfc_srate,
		tegra210_sfc_get_srate, tegra210_sfc_put_srate),
};

static struct snd_soc_codec_driver tegra210_sfc_codec = {
	.probe = tegra210_sfc_codec_probe,
	.dapm_widgets = tegra210_sfc_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra210_sfc_widgets),
	.dapm_routes = tegra210_sfc_routes,
	.num_dapm_routes = ARRAY_SIZE(tegra210_sfc_routes),
	.controls = tegra210_sfc_controls,
	.num_controls = ARRAY_SIZE(tegra210_sfc_controls),
};

static bool tegra210_sfc_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_AXBAR_RX_INT_MASK:
	case TEGRA210_SFC_AXBAR_RX_INT_SET:
	case TEGRA210_SFC_AXBAR_RX_INT_CLEAR:
	case TEGRA210_SFC_AXBAR_RX_CIF_CTRL:
	case TEGRA210_SFC_AXBAR_RX_FREQ:

	case TEGRA210_SFC_AXBAR_TX_INT_MASK:
	case TEGRA210_SFC_AXBAR_TX_INT_SET:
	case TEGRA210_SFC_AXBAR_TX_INT_CLEAR:
	case TEGRA210_SFC_AXBAR_TX_CIF_CTRL:
	case TEGRA210_SFC_AXBAR_TX_FREQ:

	case TEGRA210_SFC_ENABLE:
	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_CG:
	case TEGRA210_SFC_COEF_RAM:
	case TEGRA210_SFC_AHUBRAMCTL_SFC_CTRL:
	case TEGRA210_SFC_AHUBRAMCTL_SFC_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_sfc_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_AXBAR_RX_STATUS:
	case TEGRA210_SFC_AXBAR_RX_INT_STATUS:
	case TEGRA210_SFC_AXBAR_RX_INT_MASK:
	case TEGRA210_SFC_AXBAR_RX_INT_SET:
	case TEGRA210_SFC_AXBAR_RX_INT_CLEAR:
	case TEGRA210_SFC_AXBAR_RX_CIF_CTRL:
	case TEGRA210_SFC_AXBAR_RX_FREQ:

	case TEGRA210_SFC_AXBAR_TX_STATUS:
	case TEGRA210_SFC_AXBAR_TX_INT_STATUS:
	case TEGRA210_SFC_AXBAR_TX_INT_MASK:
	case TEGRA210_SFC_AXBAR_TX_INT_SET:
	case TEGRA210_SFC_AXBAR_TX_INT_CLEAR:
	case TEGRA210_SFC_AXBAR_TX_CIF_CTRL:
	case TEGRA210_SFC_AXBAR_TX_FREQ:

	case TEGRA210_SFC_ENABLE:
	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_CG:
	case TEGRA210_SFC_STATUS:
	case TEGRA210_SFC_INT_STATUS:
	case TEGRA210_SFC_COEF_RAM:
	case TEGRA210_SFC_AHUBRAMCTL_SFC_CTRL:
	case TEGRA210_SFC_AHUBRAMCTL_SFC_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_sfc_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_AXBAR_RX_STATUS:
	case TEGRA210_SFC_AXBAR_RX_INT_STATUS:
	case TEGRA210_SFC_AXBAR_RX_INT_SET:

	case TEGRA210_SFC_AXBAR_TX_STATUS:
	case TEGRA210_SFC_AXBAR_TX_INT_STATUS:
	case TEGRA210_SFC_AXBAR_TX_INT_SET:

	case TEGRA210_SFC_SOFT_RESET:
	case TEGRA210_SFC_STATUS:
	case TEGRA210_SFC_INT_STATUS:
	case TEGRA210_SFC_AHUBRAMCTL_SFC_CTRL:
	case TEGRA210_SFC_AHUBRAMCTL_SFC_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_sfc_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_SFC_AHUBRAMCTL_SFC_DATA:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_sfc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_SFC_AHUBRAMCTL_SFC_DATA,
	.writeable_reg = tegra210_sfc_wr_reg,
	.readable_reg = tegra210_sfc_rd_reg,
	.volatile_reg = tegra210_sfc_volatile_reg,
	.precious_reg = tegra210_sfc_precious_reg,
	.cache_type = REGCACHE_RBTREE,
};

static const struct tegra210_sfc_soc_data soc_data_tegra210 = {
	.set_audio_cif = tegra210_xbar_set_cif,
};

static const struct of_device_id tegra210_sfc_of_match[] = {
	{ .compatible = "nvidia,tegra210-sfc", .data = &soc_data_tegra210 },
	{},
};

static int tegra210_sfc_platform_probe(struct platform_device *pdev)
{
	struct tegra210_sfc *sfc;
	struct resource *mem, *memregion;
	void __iomem *regs;
	int ret = 0;
	const struct of_device_id *match;
	struct tegra210_sfc_soc_data *soc_data;

	match = of_match_device(tegra210_sfc_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		ret = -ENODEV;
		goto err;
	}
	soc_data = (struct tegra210_sfc_soc_data *)match->data;

	sfc = devm_kzalloc(&pdev->dev, sizeof(struct tegra210_sfc), GFP_KERNEL);
	if (!sfc) {
		dev_err(&pdev->dev, "Can't allocate sfc\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, sfc);

	sfc->soc_data = soc_data;

	/* initialize default output srate */
	sfc->srate_out = TEGRA210_SFC_FS48;

	sfc->clk_sfc = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(sfc->clk_sfc)) {
		dev_err(&pdev->dev, "Can't retrieve sfc clock\n");
		ret = PTR_ERR(sfc->clk_sfc);
		goto err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_clk_put;
	}

	memregion = devm_request_mem_region(&pdev->dev, mem->start,
					    resource_size(mem), pdev->name);
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

	sfc->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &tegra210_sfc_regmap_config);
	if (IS_ERR(sfc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(sfc->regmap);
		goto err_clk_put;
	}
	regcache_cache_only(sfc->regmap, true);

	if (of_property_read_u32(pdev->dev.of_node,
				"nvidia,ahub-sfc-id",
				&pdev->dev.id) < 0) {
		dev_err(&pdev->dev,
			"Missing property nvidia,ahub-sfc-id\n");
		ret = -ENODEV;
		goto err_clk_put;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra210_sfc_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = snd_soc_register_codec(&pdev->dev, &tegra210_sfc_codec,
				     tegra210_sfc_dais,
				     ARRAY_SIZE(tegra210_sfc_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_sfc_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_clk_put:
	devm_clk_put(&pdev->dev, sfc->clk_sfc);
err:
	return ret;
}

static int tegra210_sfc_platform_remove(struct platform_device *pdev)
{
	struct tegra210_sfc *sfc = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_sfc_runtime_suspend(&pdev->dev);

	devm_clk_put(&pdev->dev, sfc->clk_sfc);

	return 0;
}

static const struct dev_pm_ops tegra210_sfc_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_sfc_runtime_suspend,
			   tegra210_sfc_runtime_resume, NULL)
};

static struct platform_driver tegra210_sfc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_sfc_of_match,
		.pm = &tegra210_sfc_pm_ops,
	},
	.probe = tegra210_sfc_platform_probe,
	.remove = tegra210_sfc_platform_remove,
};
module_platform_driver(tegra210_sfc_driver)

MODULE_AUTHOR("Arun Shamanna Lakshmi <aruns@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 SFC ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra210_sfc_of_match);
