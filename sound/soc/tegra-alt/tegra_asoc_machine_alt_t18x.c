/*
 * tegra_asoc_machine_alt_t18x.c - Additional features for T186
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

#include <sound/soc.h>
#include "tegra_asoc_machine_alt.h"
#include "tegra_asoc_machine_alt_t18x.h"

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
	tegra186_xbar_dai_links[TEGRA186_XBAR_DAI_LINKS] = {
	[TEGRA186_DAI_LINK_ADMAIF1] = {
		.name = "ADMAIF1 CIF",
		.stream_name = "ADMAIF1 CIF",
		.cpu_dai_name = "ADMAIF1",
		.codec_dai_name = "ADMAIF1",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF2] = {
		.name = "ADMAIF2 CIF",
		.stream_name = "ADMAIF2 CIF",
		.cpu_dai_name = "ADMAIF2",
		.codec_dai_name = "ADMAIF2",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF3] = {
		.name = "ADMAIF3 CIF",
		.stream_name = "ADMAIF3 CIF",
		.cpu_dai_name = "ADMAIF3",
		.codec_dai_name = "ADMAIF3",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF4] = {
		.name = "ADMAIF4 CIF",
		.stream_name = "ADMAIF4 CIF",
		.cpu_dai_name = "ADMAIF4",
		.codec_dai_name = "ADMAIF4",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF5] = {
		.name = "ADMAIF5 CIF",
		.stream_name = "ADMAIF5 CIF",
		.cpu_dai_name = "ADMAIF5",
		.codec_dai_name = "ADMAIF5",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF6] = {
		.name = "ADMAIF6 CIF",
		.stream_name = "ADMAIF6 CIF",
		.cpu_dai_name = "ADMAIF6",
		.codec_dai_name = "ADMAIF6",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF7] = {
		.name = "ADMAIF7 CIF",
		.stream_name = "ADMAIF7 CIF",
		.cpu_dai_name = "ADMAIF7",
		.codec_dai_name = "ADMAIF7",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF8] = {
		.name = "ADMAIF8 CIF",
		.stream_name = "ADMAIF8 CIF",
		.cpu_dai_name = "ADMAIF8",
		.codec_dai_name = "ADMAIF8",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF9] = {
		.name = "ADMAIF9 CIF",
		.stream_name = "ADMAIF9 CIF",
		.cpu_dai_name = "ADMAIF9",
		.codec_dai_name = "ADMAIF9",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF10] = {
		.name = "ADMAIF10 CIF",
		.stream_name = "ADMAIF10 CIF",
		.cpu_dai_name = "ADMAIF10",
		.codec_dai_name = "ADMAIF10",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF11] = {
		.name = "ADMAIF11 CIF",
		.stream_name = "ADMAIF11 CIF",
		.cpu_dai_name = "ADMAIF11",
		.codec_dai_name = "ADMAIF11",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF12] = {
		.name = "ADMAIF12 CIF",
		.stream_name = "ADMAIF12 CIF",
		.cpu_dai_name = "ADMAIF12",
		.codec_dai_name = "ADMAIF12",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF13] = {
		.name = "ADMAIF13 CIF",
		.stream_name = "ADMAIF13 CIF",
		.cpu_dai_name = "ADMAIF13",
		.codec_dai_name = "ADMAIF13",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF14] = {
		.name = "ADMAIF14 CIF",
		.stream_name = "ADMAIF14 CIF",
		.cpu_dai_name = "ADMAIF14",
		.codec_dai_name = "ADMAIF14",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF15] = {
		.name = "ADMAIF15 CIF",
		.stream_name = "ADMAIF15 CIF",
		.cpu_dai_name = "ADMAIF15",
		.codec_dai_name = "ADMAIF15",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF16] = {
		.name = "ADMAIF16 CIF",
		.stream_name = "ADMAIF16 CIF",
		.cpu_dai_name = "ADMAIF16",
		.codec_dai_name = "ADMAIF16",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF17] = {
		.name = "ADMAIF17 CIF",
		.stream_name = "ADMAIF17 CIF",
		.cpu_dai_name = "ADMAIF17",
		.codec_dai_name = "ADMAIF17",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF18] = {
		.name = "ADMAIF18 CIF",
		.stream_name = "ADMAIF18 CIF",
		.cpu_dai_name = "ADMAIF18",
		.codec_dai_name = "ADMAIF18",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF19] = {
		.name = "ADMAIF19 CIF",
		.stream_name = "ADMAIF19 CIF",
		.cpu_dai_name = "ADMAIF19",
		.codec_dai_name = "ADMAIF19",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF20] = {
		.name = "ADMAIF20 CIF",
		.stream_name = "ADMAIF20 CIF",
		.cpu_dai_name = "ADMAIF20",
		.codec_dai_name = "ADMAIF20",
		.cpu_name = "tegra210-admaif",
		.codec_name = "2900800.ahub",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_AMX1_1] = {
		.name = "AMX1 IN1",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-1",
		.codec_dai_name = "IN1",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AMX1_2] = {
		.name = "AMX1 IN2",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-2",
		.codec_dai_name = "IN2",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AMX1_3] = {
		.name = "AMX1 IN3",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-3",
		.codec_dai_name = "IN3",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AMX1_4] = {
		.name = "AMX1 IN4",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-4",
		.codec_dai_name = "IN4",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AMX1] = {
		.name = "AMX1 CIF",
		.stream_name = "AMX1 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX1",
		.cpu_name = "tegra210-amx.0",
		.codec_name = "2900800.ahub",
		.params = &tdm_link_params,
	},
	[TEGRA186_DAI_LINK_ADX1] = {
		.name = "ADX1 CIF",
		.stream_name = "ADX1 IN",
		.cpu_dai_name = "ADX1",
		.codec_dai_name = "IN",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-adx.0",
		.params = &tdm_link_params,
	},
	[TEGRA186_DAI_LINK_ADX1_1] = {
		.name = "ADX1 OUT1",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX1-1",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADX1_2] = {
		.name = "ADX1 OUT2",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX1-2",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADX1_3] = {
		.name = "ADX1 OUT3",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX1-3",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADX1_4] = {
		.name = "ADX1 OUT4",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX1-4",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX1] = {
		.name = "MIXER1 RX1",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-1",
		.codec_dai_name = "RX1",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX2] = {
		.name = "MIXER1 RX2",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-2",
		.codec_dai_name = "RX2",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX3] = {
		.name = "MIXER1 RX3",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-3",
		.codec_dai_name = "RX3",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX4] = {
		.name = "MIXER1 RX4",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-4",
		.codec_dai_name = "RX4",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX5] = {
		.name = "MIXER1 RX5",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-5",
		.codec_dai_name = "RX5",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX6] = {
		.name = "MIXER1 RX6",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-6",
		.codec_dai_name = "RX6",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX7] = {
		.name = "MIXER1 RX7",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-7",
		.codec_dai_name = "RX7",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX8] = {
		.name = "MIXER1 RX8",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-8",
		.codec_dai_name = "RX8",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX9] = {
		.name = "MIXER1 RX9",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-9",
		.codec_dai_name = "RX9",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX10] = {
		.name = "MIXER1 RX10",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-10",
		.codec_dai_name = "RX10",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX1] = {
		.name = "MIXER1 TX1",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX1",
		.codec_dai_name = "MIXER1-1",
		.cpu_name = "tegra210-mixer",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX2] = {
		.name = "MIXER1 TX2",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX2",
		.codec_dai_name = "MIXER1-2",
		.cpu_name = "tegra210-mixer",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX3] = {
		.name = "MIXER1 TX3",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX3",
		.codec_dai_name = "MIXER1-3",
		.cpu_name = "tegra210-mixer",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX4] = {
		.name = "MIXER1 TX4",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX4",
		.codec_dai_name = "MIXER1-4",
		.cpu_name = "tegra210-mixer",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX5] = {
		.name = "MIXER1 TX5",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX5",
		.codec_dai_name = "MIXER1-5",
		.cpu_name = "tegra210-mixer",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_SFC1_RX] = {
		.name = "SFC1 RX",
		.stream_name = "SFC1 RX",
		.cpu_dai_name = "SFC1",
		.codec_dai_name = "CIF",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-sfc.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_SFC1_TX] = {
		.name = "SFC1 TX",
		.stream_name = "SFC1 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC1",
		.cpu_name = "tegra210-sfc.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AFC1_RX] = {
		.name = "AFC1 RX",
		.stream_name = "AFC1 RX",
		.cpu_dai_name = "AFC1",
		.codec_dai_name = "AFC IN",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-afc.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AFC1_TX] = {
		.name = "AFC1 TX",
		.stream_name = "AFC1 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC1",
		.cpu_name = "tegra210-afc.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MVC1_RX] = {
		.name = "MVC1 RX",
		.stream_name = "MVC1 RX",
		.cpu_dai_name = "MVC1",
		.codec_dai_name = "MVC IN",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-mvc.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_MVC1_TX] = {
		.name = "MVC1 TX",
		.stream_name = "MVC1 TX",
		.cpu_dai_name = "MVC OUT",
		.codec_dai_name = "MVC1",
		.cpu_name = "tegra210-mvc.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_OPE1_RX] = {
		.name = "OPE1 RX",
		.stream_name = "OPE1 RX",
		.cpu_dai_name = "OPE1",
		.codec_dai_name = "OPE IN",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra210-ope.0",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_OPE1_TX] = {
		.name = "OPE1 TX",
		.stream_name = "OPE1 TX",
		.cpu_dai_name = "OPE OUT",
		.codec_dai_name = "OPE1",
		.cpu_name = "tegra210-ope.0",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX1] = {
		.name = "ASRC1 RX1",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-1",
		.codec_dai_name = "RX1",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX2] = {
		.name = "ASRC1 RX2",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-2",
		.codec_dai_name = "RX2",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX3] = {
		.name = "ASRC1 RX3",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-3",
		.codec_dai_name = "RX3",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX4] = {
		.name = "ASRC1 RX4",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-4",
		.codec_dai_name = "RX4",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX5] = {
		.name = "ASRC1 RX5",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-5",
		.codec_dai_name = "RX5",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX6] = {
		.name = "ASRC1 RX6",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-6",
		.codec_dai_name = "RX6",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX7] = {
		.name = "ASRC1 RX7",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-7",
		.codec_dai_name = "RX7",
		.cpu_name = "2900800.ahub",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX1] = {
		.name = "ASRC1 TX1",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX1",
		.codec_dai_name = "ASRC1-1",
		.cpu_name = "tegra186-asrc",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX2] = {
		.name = "ASRC1 TX2",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX2",
		.codec_dai_name = "ASRC1-2",
		.cpu_name = "tegra186-asrc",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX3] = {
		.name = "ASRC1 TX3",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX3",
		.codec_dai_name = "ASRC1-3",
		.cpu_name = "tegra186-asrc",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX4] = {
		.name = "ASRC1 TX4",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX4",
		.codec_dai_name = "ASRC1-4",
		.cpu_name = "tegra186-asrc",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX5] = {
		.name = "ASRC1 TX5",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX5",
		.codec_dai_name = "ASRC1-5",
		.cpu_name = "tegra186-asrc",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX6] = {
		.name = "ASRC1 TX6",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX6",
		.codec_dai_name = "ASRC1-6",
		.cpu_name = "tegra186-asrc",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ARAD1_TX1] = {
		.name = "ARAD1 TX",
		.stream_name = "ARAD1 TX",
		.cpu_dai_name = "ARAD OUT",
		.codec_dai_name = "ARAD1",
		.cpu_name = "tegra186-arad",
		.codec_name = "2900800.ahub",
		.params = &default_link_params,
	},
};

static struct snd_soc_codec_conf
	tegra186_xbar_codec_conf[TEGRA186_XBAR_CODEC_CONF] = {
	[TEGRA186_CODEC_AMX1_CONF] = {
		.dev_name = "tegra210-amx.0",
		.name_prefix = "AMX1",
	},
	[TEGRA186_CODEC_ADX1_CONF] = {
		.dev_name = "tegra210-adx.0",
		.name_prefix = "ADX1",
	},
	[TEGRA186_CODEC_SFC1_CONF] = {
		.dev_name = "tegra210-sfc.0",
		.name_prefix = "SFC1",
	},
	[TEGRA186_CODEC_MVC1_CONF] = {
		.dev_name = "tegra210-mvc.0",
		.name_prefix = "MVC1",
	},
	[TEGRA186_CODEC_OPE1_CONF] = {
		.dev_name = "tegra210-ope.0",
		.name_prefix = "OPE1",
	},
	[TEGRA186_CODEC_AFC1_CONF] = {
		.dev_name = "tegra210-afc.0",
		.name_prefix = "AFC1",
	},
	[TEGRA186_CODEC_I2S1_CONF] = {
		.dev_name = "tegra210-i2s.0",
		.name_prefix = "I2S1",
	},
	[TEGRA186_CODEC_I2S2_CONF] = {
		.dev_name = "tegra210-i2s.1",
		.name_prefix = "I2S2",
	},
	[TEGRA186_CODEC_I2S3_CONF] = {
		.dev_name = "tegra210-i2s.2",
		.name_prefix = "I2S3",
	},
	[TEGRA186_CODEC_I2S4_CONF] = {
		.dev_name = "tegra210-i2s.3",
		.name_prefix = "I2S4",
	},
	[TEGRA186_CODEC_I2S5_CONF] = {
		.dev_name = "tegra210-i2s.4",
		.name_prefix = "I2S5",
	},
	[TEGRA186_CODEC_I2S6_CONF] = {
		.dev_name = "tegra210-i2s.5",
		.name_prefix = "I2S6",
	},
	[TEGRA186_CODEC_DMIC1_CONF] = {
		.dev_name = "tegra210-dmic.0",
		.name_prefix = "DMIC1",
	},
	[TEGRA186_CODEC_SPDIF_CONF] = {
		.dev_name = "tegra210-spdif",
		.name_prefix = "SPDIF",
	},
	[TEGRA186_CODEC_ASRC1_CONF] = {
		.dev_name = "tegra186-asrc",
		.name_prefix = "ASRC1",
	},
};

struct snd_soc_dai_link *tegra_machine_get_dai_link_t18x(void)
{
	struct snd_soc_dai_link *link = tegra186_xbar_dai_links;
	unsigned int size = TEGRA186_XBAR_DAI_LINKS;
	struct snd_soc_dai_link *tegra_asoc_machine_links =
		tegra_machine_get_machine_links();

	if (tegra_asoc_machine_links)
		return tegra_asoc_machine_links;

	tegra_machine_set_num_dai_links(size);

	tegra_asoc_machine_links = kzalloc(size *
		sizeof(struct snd_soc_dai_link), GFP_KERNEL);

	memcpy(tegra_asoc_machine_links, link,
		size * sizeof(struct snd_soc_dai_link));

	tegra_machine_set_machine_links(tegra_asoc_machine_links);

	return tegra_asoc_machine_links;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_dai_link_t18x);

int tegra_machine_append_dai_link_t18x(struct snd_soc_dai_link *link,
		unsigned int link_size)
{
	unsigned int size1 = TEGRA186_XBAR_DAI_LINKS;
	unsigned int size2 = link_size;
	struct snd_soc_dai_link *tegra_asoc_machine_links =
		tegra_machine_get_machine_links();

	if (!tegra_asoc_machine_links) {
		if (link) {
			tegra_machine_set_machine_links(link);
			tegra_machine_set_num_dai_links(size2);
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
			tegra_machine_set_machine_links(
				tegra_asoc_machine_links);
			memcpy(&tegra_asoc_machine_links[size1], link,
				size2 * sizeof(struct snd_soc_dai_link));
			tegra_machine_set_num_dai_links(size1+size2);
			return size1+size2;
		} else {
			tegra_machine_set_num_dai_links(size1);
			return size1;
		}
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_dai_link_t18x);

struct snd_soc_codec_conf *tegra_machine_get_codec_conf_t18x(void)
{
	struct snd_soc_codec_conf *conf = tegra186_xbar_codec_conf;
	struct snd_soc_codec_conf *tegra_asoc_codec_conf =
		tegra_machine_get_machine_codec_conf();
	unsigned int size = TEGRA186_XBAR_CODEC_CONF;

	if (tegra_asoc_codec_conf)
		return tegra_asoc_codec_conf;

	tegra_asoc_codec_conf = kzalloc(size *
		sizeof(struct snd_soc_codec_conf), GFP_KERNEL);

	memcpy(tegra_asoc_codec_conf, conf,
		size * sizeof(struct snd_soc_codec_conf));

	tegra_machine_set_machine_codec_conf(tegra_asoc_codec_conf);

	return tegra_asoc_codec_conf;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_conf_t18x);

int tegra_machine_append_codec_conf_t18x(struct snd_soc_codec_conf *conf,
		unsigned int conf_size)
{
	unsigned int size1 = TEGRA186_XBAR_CODEC_CONF;
	unsigned int size2 = conf_size;
	struct snd_soc_codec_conf *tegra_asoc_codec_conf =
		tegra_machine_get_machine_codec_conf();

	if (!tegra_asoc_codec_conf) {
		if (conf) {
			tegra_machine_set_machine_codec_conf(conf);
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
			tegra_machine_set_machine_codec_conf(
				tegra_asoc_codec_conf);
			memcpy(&tegra_asoc_codec_conf[size1], conf,
				size2 * sizeof(struct snd_soc_codec_conf));
			return size1+size2;
		} else
			return size1;
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_codec_conf_t18x);

unsigned int tegra_machine_get_codec_dai_link_idx_t18x(const char *codec_name)
{
	unsigned int idx = TEGRA186_XBAR_DAI_LINKS;
	struct snd_soc_dai_link *tegra_asoc_machine_links =
		tegra_machine_get_machine_links();

	if (tegra_machine_get_num_dai_links() <= idx)
		goto err;

	while (idx < tegra_machine_get_num_dai_links()) {
		if (tegra_asoc_machine_links[idx].name)
			if (!strcmp(tegra_asoc_machine_links[idx].name,
				codec_name))
				return idx;
		idx++;
	}

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_dai_link_idx_t18x);

unsigned int tegra_machine_get_bclk_ratio_t18x(
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *codec_dai_link = rtd->dai_link;
	char *codec_name = (char *)codec_dai_link->name;
	unsigned int idx =
		tegra_machine_get_codec_dai_link_idx_t18x(codec_name);
	unsigned int *bclk_ratio =
		tegra_machine_get_bclk_ratio_array();

	if (idx == -EINVAL)
		goto err;

	if (!bclk_ratio)
		goto err;

	idx = idx - TEGRA186_XBAR_DAI_LINKS;

	return bclk_ratio[idx];

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_bclk_ratio_t18x);
