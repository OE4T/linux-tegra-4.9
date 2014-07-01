/*
 * tegra_asoc_machine_alt.c - Tegra xbar dai link for machine drivers
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

#include <linux/of.h>
#include <sound/soc.h>

#include "tegra_asoc_machine_alt.h"

static struct snd_soc_dai_link *tegra_asoc_machine_links;
static struct snd_soc_codec_conf *tegra_asoc_codec_conf;

static const struct snd_soc_pcm_stream default_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream tdm_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 8,
	.channels_max = 8,
};

static struct snd_soc_dai_link
	tegra124_xbar_dai_links[TEGRA124_XBAR_DAI_LINKS] = {
	[TEGRA124_DAI_LINK_APBIF0] = {
		.name = "APBIF0 CIF",
		.stream_name = "APBIF0 CIF",
		.cpu_dai_name = "APBIF0",
		.codec_dai_name = "APBIF0",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF1] = {
		.name = "APBIF1 CIF",
		.stream_name = "APBIF1 CIF",
		.cpu_dai_name = "APBIF1",
		.codec_dai_name = "APBIF1",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF2] = {
		.name = "APBIF2 CIF",
		.stream_name = "APBIF2 CIF",
		.cpu_dai_name = "APBIF2",
		.codec_dai_name = "APBIF2",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF3] = {
		.name = "APBIF3 CIF",
		.stream_name = "APBIF3 CIF",
		.cpu_dai_name = "APBIF3",
		.codec_dai_name = "APBIF3",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF4] = {
		.name = "APBIF4 CIF",
		.stream_name = "APBIF4 CIF",
		.cpu_dai_name = "APBIF4",
		.codec_dai_name = "APBIF4",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF5] = {
		.name = "APBIF5 CIF",
		.stream_name = "APBIF5 CIF",
		.cpu_dai_name = "APBIF5",
		.codec_dai_name = "APBIF5",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF6] = {
		.name = "APBIF6 CIF",
		.stream_name = "APBIF6 CIF",
		.cpu_dai_name = "APBIF6",
		.codec_dai_name = "APBIF6",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF7] = {
		.name = "APBIF7 CIF",
		.stream_name = "APBIF7 CIF",
		.cpu_dai_name = "APBIF7",
		.codec_dai_name = "APBIF7",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF8] = {
		.name = "APBIF8 CIF",
		.stream_name = "APBIF8 CIF",
		.cpu_dai_name = "APBIF8",
		.codec_dai_name = "APBIF8",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_APBIF9] = {
		.name = "APBIF9 CIF",
		.stream_name = "APBIF9 CIF",
		.cpu_dai_name = "APBIF9",
		.codec_dai_name = "APBIF9",
		.cpu_name = "tegra30-ahub-apbif",
		.codec_name = "tegra30-ahub-xbar",
		.platform_name = "tegra30-ahub-apbif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA124_DAI_LINK_AMX0_0] = {
		.name = "AMX0 IN0",
		.stream_name = "AMX0 IN",
		.cpu_dai_name = "AMX0-0",
		.codec_dai_name = "IN0",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX0_1] = {
		.name = "AMX0 IN1",
		.stream_name = "AMX0 IN",
		.cpu_dai_name = "AMX0-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX0_2] = {
		.name = "AMX0 IN2",
		.stream_name = "AMX0 IN",
		.cpu_dai_name = "AMX0-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX0_3] = {
		.name = "AMX0 IN3",
		.stream_name = "AMX0 IN",
		.cpu_dai_name = "AMX0-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX0] = {
		.name = "AMX0 OUT",
		.stream_name = "AMX0 OUT",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX0",
		.cpu_name = "tegra124-amx.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &tdm_link_params,
	},
	[TEGRA124_DAI_LINK_AMX1_0] = {
		.name = "AMX1 IN0",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-0",
		.codec_dai_name = "IN0",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX1_1] = {
		.name = "AMX1 IN1",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX1_2] = {
		.name = "AMX1 IN2",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX1_3] = {
		.name = "AMX1 IN3",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-amx.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AMX1] = {
		.name = "AMX1 OUT",
		.stream_name = "AMX1 OUT",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX1",
		.cpu_name = "tegra124-amx.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &tdm_link_params,
	},
	[TEGRA124_DAI_LINK_ADX0] = {
		.name = "ADX0 CIF",
		.stream_name = "ADX0 IN",
		.cpu_dai_name = "ADX0",
		.codec_dai_name = "IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-adx.0",
		.params = &tdm_link_params,
	},
	[TEGRA124_DAI_LINK_ADX0_0] = {
		.name = "ADX0 OUT0",
		.stream_name = "ADX0 OUT",
		.cpu_dai_name = "OUT0",
		.codec_dai_name = "ADX0-0",
		.cpu_name = "tegra124-adx.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX0_1] = {
		.name = "ADX0 OUT1",
		.stream_name = "ADX0 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX0-1",
		.cpu_name = "tegra124-adx.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX0_2] = {
		.name = "ADX0 OUT2",
		.stream_name = "ADX0 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX0-2",
		.cpu_name = "tegra124-adx.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX0_3] = {
		.name = "ADX0 OUT3",
		.stream_name = "ADX0 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX0-3",
		.cpu_name = "tegra124-adx.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX1] = {
		.name = "ADX1 CIF",
		.stream_name = "ADX1 IN",
		.cpu_dai_name = "ADX1",
		.codec_dai_name = "IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-adx.1",
		.params = &tdm_link_params,
	},
	[TEGRA124_DAI_LINK_ADX1_0] = {
		.name = "ADX1 OUT0",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT0",
		.codec_dai_name = "ADX1-0",
		.cpu_name = "tegra124-adx.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX1_1] = {
		.name = "ADX1 OUT1",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX1-1",
		.cpu_name = "tegra124-adx.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX1_2] = {
		.name = "ADX1 OUT2",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX1-2",
		.cpu_name = "tegra124-adx.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_ADX1_3] = {
		.name = "ADX1 OUT3",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX1-3",
		.cpu_name = "tegra124-adx.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM0_0] = {
		.name = "DAM0 IN0",
		.stream_name = "DAM0 IN0",
		.cpu_dai_name = "DAM0-0",
		.codec_dai_name = "IN0",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-dam.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM0_1] = {
		.name = "DAM0 IN1",
		.stream_name = "DAM0 IN1",
		.cpu_dai_name = "DAM0-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-dam.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM0] = {
		.name = "DAM0 OUT",
		.stream_name = "DAM0 OUT",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "DAM0",
		.cpu_name = "tegra30-dam.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM1_0] = {
		.name = "DAM1 IN0",
		.stream_name = "DAM1 IN0",
		.cpu_dai_name = "DAM1-0",
		.codec_dai_name = "IN0",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-dam.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM1_1] = {
		.name = "DAM1 IN1",
		.stream_name = "DAM1 IN1",
		.cpu_dai_name = "DAM1-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-dam.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM1] = {
		.name = "DAM1 OUT",
		.stream_name = "DAM1 OUT",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "DAM1",
		.cpu_name = "tegra30-dam.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM2_0] = {
		.name = "DAM2 IN0",
		.stream_name = "DAM2 IN0",
		.cpu_dai_name = "DAM2-0",
		.codec_dai_name = "IN0",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-dam.2",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM2_1] = {
		.name = "DAM2 IN1",
		.stream_name = "DAM2 IN1",
		.cpu_dai_name = "DAM2-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra30-dam.2",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_DAM2] = {
		.name = "DAM2 OUT",
		.stream_name = "DAM2 OUT",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "DAM2",
		.cpu_name = "tegra30-dam.2",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC0_RX] = {
		.name = "AFC0 RX",
		.stream_name = "AFC0 RX",
		.cpu_dai_name = "AFC0",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-afc.0",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC0_TX] = {
		.name = "AFC0 TX",
		.stream_name = "AFC0 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC0",
		.cpu_name = "tegra124-afc.0",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC1_RX] = {
		.name = "AFC1 RX",
		.stream_name = "AFC1 RX",
		.cpu_dai_name = "AFC1",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-afc.1",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC1_TX] = {
		.name = "AFC1 TX",
		.stream_name = "AFC1 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC1",
		.cpu_name = "tegra124-afc.1",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC2_RX] = {
		.name = "AFC2 RX",
		.stream_name = "AFC2 RX",
		.cpu_dai_name = "AFC2",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-afc.2",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC2_TX] = {
		.name = "AFC2 TX",
		.stream_name = "AFC2 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC2",
		.cpu_name = "tegra124-afc.2",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC3_RX] = {
		.name = "AFC3 RX",
		.stream_name = "AFC3 RX",
		.cpu_dai_name = "AFC3",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-afc.3",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC3_TX] = {
		.name = "AFC3 TX",
		.stream_name = "AFC3 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC3",
		.cpu_name = "tegra124-afc.3",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC4_RX] = {
		.name = "AFC4 RX",
		.stream_name = "AFC4 RX",
		.cpu_dai_name = "AFC4",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-afc.4",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC4_TX] = {
		.name = "AFC4 TX",
		.stream_name = "AFC4 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC4",
		.cpu_name = "tegra124-afc.4",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC5_RX] = {
		.name = "AFC5 RX",
		.stream_name = "AFC5 RX",
		.cpu_dai_name = "AFC5",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra30-ahub-xbar",
		.codec_name = "tegra124-afc.5",
		.params = &default_link_params,
	},
	[TEGRA124_DAI_LINK_AFC5_TX] = {
		.name = "AFC5 TX",
		.stream_name = "AFC5 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC5",
		.cpu_name = "tegra124-afc.5",
		.codec_name = "tegra30-ahub-xbar",
		.params = &default_link_params,
	},
};

static struct snd_soc_codec_conf
	tegra124_xbar_codec_conf[TEGRA124_XBAR_CODEC_CONF] = {
	[TEGRA124_CODEC_AMX0_CONF] = {
		.dev_name = "tegra124-amx.0",
		.name_prefix = "AMX0",
	},
	[TEGRA124_CODEC_AMX1_CONF] = {
		.dev_name = "tegra124-amx.1",
		.name_prefix = "AMX1",
	},
	[TEGRA124_CODEC_ADX0_CONF] = {
		.dev_name = "tegra124-adx.0",
		.name_prefix = "ADX0",
	},
	[TEGRA124_CODEC_ADX1_CONF] = {
		.dev_name = "tegra124-adx.1",
		.name_prefix = "ADX1",
	},
	[TEGRA124_CODEC_DAM0_CONF] = {
		.dev_name = "tegra30-dam.0",
		.name_prefix = "DAM0",
	},
	[TEGRA124_CODEC_DAM1_CONF] = {
		.dev_name = "tegra30-dam.1",
		.name_prefix = "DAM1",
	},
	[TEGRA124_CODEC_DAM2_CONF] = {
		.dev_name = "tegra30-dam.2",
		.name_prefix = "DAM2",
	},
	[TEGRA124_CODEC_AFC0_CONF] = {
		.dev_name = "tegra124-afc.0",
		.name_prefix = "AFC0",
	},
	[TEGRA124_CODEC_AFC1_CONF] = {
		.dev_name = "tegra124-afc.1",
		.name_prefix = "AFC1",
	},
	[TEGRA124_CODEC_AFC2_CONF] = {
		.dev_name = "tegra124-afc.2",
		.name_prefix = "AFC2",
	},
	[TEGRA124_CODEC_AFC3_CONF] = {
		.dev_name = "tegra124-afc.3",
		.name_prefix = "AFC3",
	},
	[TEGRA124_CODEC_AFC4_CONF] = {
		.dev_name = "tegra124-afc.4",
		.name_prefix = "AFC4",
	},
	[TEGRA124_CODEC_AFC5_CONF] = {
		.dev_name = "tegra124-afc.5",
		.name_prefix = "AFC5",
	},
	[TEGRA124_CODEC_I2S0_CONF] = {
		.dev_name = "tegra30-i2s.0",
		.name_prefix = "I2S0",
	},
	[TEGRA124_CODEC_I2S1_CONF] = {
		.dev_name = "tegra30-i2s.1",
		.name_prefix = "I2S1",
	},
	[TEGRA124_CODEC_I2S2_CONF] = {
		.dev_name = "tegra30-i2s.2",
		.name_prefix = "I2S2",
	},
	[TEGRA124_CODEC_I2S3_CONF] = {
		.dev_name = "tegra30-i2s.3",
		.name_prefix = "I2S3",
	},
	[TEGRA124_CODEC_I2S4_CONF] = {
		.dev_name = "tegra30-i2s.4",
		.name_prefix = "I2S4",
	},
	[TEGRA124_CODEC_SPDIF_CONF] = {
		.dev_name = "tegra30-spdif",
		.name_prefix = "SPDIF",
	},
};

struct snd_soc_dai_link *tegra_machine_get_dai_link(void)
{
	struct snd_soc_dai_link *link = tegra124_xbar_dai_links;
	unsigned int size = TEGRA124_XBAR_DAI_LINKS;

	if (tegra_asoc_machine_links)
		return tegra_asoc_machine_links;

	tegra_asoc_machine_links = kzalloc(size *
		sizeof(struct snd_soc_dai_link), GFP_KERNEL);

	memcpy(tegra_asoc_machine_links, link,
		size * sizeof(struct snd_soc_dai_link));

	return tegra_asoc_machine_links;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_dai_link);

void tegra_machine_remove_dai_link(void)
{
	kfree(tegra_asoc_machine_links);
}
EXPORT_SYMBOL_GPL(tegra_machine_remove_dai_link);

/* @link: input structure to append
 * @link_size: size of the input structure
 * Returns the total size after appending
 */
int tegra_machine_append_dai_link(struct snd_soc_dai_link *link,
		unsigned int link_size)
{
	unsigned int size1 = TEGRA124_XBAR_DAI_LINKS;
	unsigned int size2 = link_size;

	if (!tegra_asoc_machine_links) {
		if (link) {
			tegra_asoc_machine_links = link;
			return size2;
		} else {
			return 0;
		}
	} else {
		if (link) {
			tegra_asoc_machine_links =
				(struct snd_soc_dai_link *) krealloc(
				tegra_asoc_machine_links, (size1 + size2) *
				sizeof(struct snd_soc_dai_link), GFP_KERNEL);
			memcpy(&tegra_asoc_machine_links[size1], link,
				size2 * sizeof(struct snd_soc_dai_link));
			return size1+size2;
		} else {
			return size1;
		}
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_dai_link);

void tegra_machine_set_dai_ops(int link, struct snd_soc_ops *ops)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].ops = ops;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_ops);

void tegra_machine_set_dai_init(int link, void *ptr)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].init = ptr;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_init);

void tegra_machine_set_dai_params(int link,
		struct snd_soc_pcm_stream *params)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].params = params;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_params);

void tegra_machine_set_dai_fmt(int link, unsigned int fmt)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].dai_fmt = fmt;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_fmt);

struct snd_soc_codec_conf *tegra_machine_get_codec_conf(void)
{
	struct snd_soc_codec_conf *conf = tegra124_xbar_codec_conf;
	unsigned int size = TEGRA124_XBAR_CODEC_CONF;

	if (tegra_asoc_codec_conf)
		return tegra_asoc_codec_conf;

	tegra_asoc_codec_conf = kzalloc(size *
		sizeof(struct snd_soc_codec_conf), GFP_KERNEL);

	memcpy(tegra_asoc_codec_conf, conf,
		size * sizeof(struct snd_soc_codec_conf));

	return tegra_asoc_codec_conf;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_conf);

void tegra_machine_remove_codec_conf(void)
{
	kfree(tegra_asoc_codec_conf);
}
EXPORT_SYMBOL_GPL(tegra_machine_remove_codec_conf);

/* @link: input structure to append
 * @link_size: size of the input structure
 * Returns the total size after appending
 */
int tegra_machine_append_codec_conf(struct snd_soc_codec_conf *conf,
		unsigned int conf_size)
{
	unsigned int size1 = TEGRA124_XBAR_CODEC_CONF;
	unsigned int size2 = conf_size;

	if (!tegra_asoc_codec_conf) {
		if (conf) {
			tegra_asoc_codec_conf = conf;
			return size2;
		} else {
			return 0;
		}
	} else {
		if (conf) {
			tegra_asoc_codec_conf =
				(struct snd_soc_codec_conf *) krealloc(
				tegra_asoc_codec_conf, (size1 + size2) *
				sizeof(struct snd_soc_codec_conf), GFP_KERNEL);
			memcpy(&tegra_asoc_codec_conf[size1], conf,
				size2 * sizeof(struct snd_soc_codec_conf));
			return size1+size2;
		} else {
			return size1;
		}
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_codec_conf);
