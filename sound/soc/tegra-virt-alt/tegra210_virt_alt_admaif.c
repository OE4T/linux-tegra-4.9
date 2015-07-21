/*
 * tegra210_admaif_alt.c - Tegra ADMAIF component driver
 *
 * Copyright (c) 2014-2015 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra210_virt_alt_admaif.h"
#include "tegra_virt_alt_ivc.h"
#include "tegra_pcm_alt.h"

static struct tegra210_admaif *admaif;

static int tegra210_admaif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	struct tegra210_virt_audio_cif *cif_conf = &data->cif;
	struct nvaudio_ivc_msg	msg;
	unsigned int value;
	int err;

	data->admaif_id = dai->id;
	memset(cif_conf, 0, sizeof(struct tegra210_virt_audio_cif));
	cif_conf->audio_channels = params_channels(params);
	cif_conf->client_channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}
	cif_conf->audio_bits = TEGRA210_AUDIOCIF_BITS_32;
	cif_conf->direction = substream->stream;

	value = (cif_conf->threshold <<
			TEGRA210_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
		((cif_conf->audio_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
		((cif_conf->client_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
		(cif_conf->audio_bits <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT) |
		(cif_conf->client_bits <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT) |
		(cif_conf->expand <<
			TEGRA210_AUDIOCIF_CTRL_EXPAND_SHIFT) |
		(cif_conf->stereo_conv <<
			TEGRA210_AUDIOCIF_CTRL_STEREO_CONV_SHIFT) |
		(cif_conf->replicate <<
			TEGRA210_AUDIOCIF_CTRL_REPLICATE_SHIFT) |
		(cif_conf->truncate <<
			TEGRA210_AUDIOCIF_CTRL_TRUNCATE_SHIFT) |
		(cif_conf->mono_conv <<
			TEGRA210_AUDIOCIF_CTRL_MONO_CONV_SHIFT);

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.params.dmaif_info.id        = data->admaif_id;
	msg.params.dmaif_info.value     = value;
	if (!cif_conf->direction)
		msg.cmd = NVAUDIO_DMAIF_SET_TXCIF;
	else
		msg.cmd = NVAUDIO_DMAIF_SET_RXCIF;

	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));

	if (err < 0)
		pr_err("error on ivc_send\n");

	return 0;
}

static void tegra210_admaif_start_playback(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_START_PLAYBACK;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static void tegra210_admaif_stop_playback(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_STOP_PLAYBACK;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static void tegra210_admaif_start_capture(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_START_CAPTURE;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
}

static void tegra210_admaif_stop_capture(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_STOP_CAPTURE;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("error on ivc_send\n");
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

static int tegra210_admaif_dai_probe(struct snd_soc_dai *dai)
{

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
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = "Capture " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},						\
		.ops = &tegra210_admaif_dai_ops,			\
	}

static struct snd_soc_dai_driver tegra210_admaif_dais[] = {
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
	ADMAIF_DAI(11),
	ADMAIF_DAI(12),
	ADMAIF_DAI(13),
	ADMAIF_DAI(14),
	ADMAIF_DAI(15),
	ADMAIF_DAI(16),
	ADMAIF_DAI(17),
	ADMAIF_DAI(18),
	ADMAIF_DAI(19),
	ADMAIF_DAI(20),
};

static const struct snd_soc_component_driver tegra210_admaif_dai_driver = {
	.name		= DRV_NAME,
};

int tegra210_virt_admaif_register_component(struct platform_device *pdev)
{
	int i = 0;
	int ret;

	admaif = devm_kzalloc(&pdev->dev, sizeof(*admaif), GFP_KERNEL);
	if (admaif == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	admaif->client_data.hivc_client =
			nvaudio_ivc_alloc_ctxt(&pdev->dev);

	if (!admaif->client_data.hivc_client) {
		dev_err(&pdev->dev, "Failed to allocate IVC context\n");
		ret = -ENODEV;
		goto err;
	}

	admaif->capture_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				MAX_ADMAIF_IDS,
			GFP_KERNEL);
	if (admaif->capture_dma_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	admaif->playback_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				MAX_ADMAIF_IDS,
			GFP_KERNEL);
	if (admaif->playback_dma_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < MAX_ADMAIF_IDS; i++) {
		admaif->playback_dma_data[i].addr = TEGRA210_ADMAIF_BASE +
				TEGRA210_ADMAIF_XBAR_TX_FIFO_WRITE +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		admaif->capture_dma_data[i].addr = TEGRA210_ADMAIF_BASE +
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
			goto err;
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
			goto err;
		}
	}

	ret = snd_soc_register_component(&pdev->dev,
					&tegra210_admaif_dai_driver,
					tegra210_admaif_dais,
					MAX_ADMAIF_IDS);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAIs %d: %d\n",
			i, ret);
		goto err;
	}

	ret = tegra_alt_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_dais;
	}

	return 0;
err_unregister_dais:
	snd_soc_unregister_component(&pdev->dev);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra210_virt_admaif_register_component);
