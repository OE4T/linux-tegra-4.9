/*
 * tegra_virt_t210ref_pcm.c - Tegra T210 reference virtual PCM driver
 *
 * Copyright (c) 2015-2016 NVIDIA CORPORATION.  All rights reserved.
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
#define LINK_CPU_NAME		DRV_NAME
#define CPU_DAI_NAME(i)		"ADMAIF" #i
#define CODEC_DAI_NAME		"dit-hifi"
#define MAX_APBIF_IDS		10
#define PLATFORM_NAME LINK_CPU_NAME

#define TEGRA210_XBAR_RX_STRIDE	0x4
#define MIXER_CONFIG_SHIFT_VALUE	16

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#define CODEC_NAME		NULL
#define NUM_MUX_INPUT		83
#define NUM_ASRC_SOURCES	2
#define TEGRA186_ASRC_STREAM_RATIO_INTEGER_PART_MASK		0x1F
#define TEGRA186_ASRC_STREAM_RATIO_FRAC_PART_MASK		0xFFFFFFFF
#else
#define CODEC_NAME		"spdif-dit.0"
#define NUM_MUX_INPUT		54
#endif

#define LAST_ADMAIF_DAI_LINK 11
#define LAST_ADSP_ADMAIF_DAI_LINK 31

static struct snd_soc_pcm_stream default_params = {
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_pcm_stream adsp_default_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
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
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 10 */
		.name = DAI_NAME(11),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(11),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 11 */
		.name = DAI_NAME(12),
		.stream_name = STREAM_NAME,
		.codec_name = CODEC_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(12),
		.codec_dai_name = CODEC_DAI_NAME,
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 12 */
		.name = "ADSP ADMAIF1",
		.stream_name = "ADSP AFMAIF1",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF1",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 13 */
		.name = "ADSP ADMAIF2",
		.stream_name = "ADSP AFMAIF2",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF2",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 14 */
		.name = "ADSP ADMAIF3",
		.stream_name = "ADSP AFMAIF3",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF3",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 15 */
		.name = "ADSP ADMAIF4",
		.stream_name = "ADSP AFMAIF4",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF4",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 16 */
		.name = "ADSP ADMAIF5",
		.stream_name = "ADSP AFMAIF5",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF5",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 17 */
		.name = "ADSP ADMAIF6",
		.stream_name = "ADSP AFMAIF6",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF6",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 18 */
		.name = "ADSP ADMAIF7",
		.stream_name = "ADSP AFMAIF7",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF7",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 19 */
		.name = "ADSP ADMAIF8",
		.stream_name = "ADSP AFMAIF8",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF8",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 20 */
		.name = "ADSP ADMAIF9",
		.stream_name = "ADSP AFMAIF9",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF9",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 21 */
		.name = "ADSP ADMAIF10",
		.stream_name = "ADSP AFMAIF10",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF10",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 22 */
		.name = "ADSP ADMAIF11",
		.stream_name = "ADSP AFMAIF11",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF11",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 23 */
		.name = "ADSP ADMAIF12",
		.stream_name = "ADSP AFMAIF12",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF12",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 24 */
		.name = "ADSP ADMAIF13",
		.stream_name = "ADSP AFMAIF13",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF13",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 25 */
		.name = "ADSP ADMAIF14",
		.stream_name = "ADSP AFMAIF14",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF14",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 26 */
		.name = "ADSP ADMAIF15",
		.stream_name = "ADSP AFMAIF15",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF15",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 27 */
		.name = "ADSP ADMAIF16",
		.stream_name = "ADSP AFMAIF16",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF16",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 28 */
		.name = "ADSP ADMAIF17",
		.stream_name = "ADSP AFMAIF17",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF17",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 29 */
		.name = "ADSP ADMAIF18",
		.stream_name = "ADSP AFMAIF18",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF18",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 30 */
		.name = "ADSP ADMAIF19",
		.stream_name = "ADSP AFMAIF19",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF19",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 31 */
		.name = "ADSP ADMAIF20",
		.stream_name = "ADSP AFMAIF20",
		.codec_name = CODEC_NAME,
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP-ADMAIF20",
		.codec_dai_name = CODEC_DAI_NAME,
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 32 */
		.name = "ADSP PCM1",
		.stream_name = "ADSP PCM1",
		.codec_name = "adsp_audio",
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP PCM1",
		.codec_dai_name = "ADSP-FE1",
		.platform_name = "adsp_audio",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 33 */
		.name = "ADSP PCM2",
		.stream_name = "ADSP PCM2",
		.codec_name = "adsp_audio",
		.cpu_name = "adsp_audio",
		.cpu_dai_name = "ADSP PCM2",
		.codec_dai_name = "ADSP-FE2",
		.platform_name = "adsp_audio",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
};

#define MUX_VALUE(npart, nbit) (1 + nbit + npart * 32)
static const int tegra_virt_t210ref_source_value[] = {
	/* Mux0 input,  Mux1 input, Mux2 input */
	0,
	MUX_VALUE(0, 0),
	MUX_VALUE(0, 1),
	MUX_VALUE(0, 2),
	MUX_VALUE(0, 3),
	MUX_VALUE(0, 4),
	MUX_VALUE(0, 5),
	MUX_VALUE(0, 6),
	MUX_VALUE(0, 7),
	MUX_VALUE(0, 8),
	MUX_VALUE(0, 9),
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	MUX_VALUE(0, 10),
	MUX_VALUE(0, 11),
	MUX_VALUE(0, 12),
	MUX_VALUE(0, 13),
	MUX_VALUE(0, 14),
	MUX_VALUE(0, 15),
#endif
	MUX_VALUE(0, 16),
	MUX_VALUE(0, 17),
	MUX_VALUE(0, 18),
	MUX_VALUE(0, 19),
	MUX_VALUE(0, 20),
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	MUX_VALUE(0, 21),
#endif
	MUX_VALUE(0, 24),
	MUX_VALUE(0, 25),
	MUX_VALUE(0, 26),
	MUX_VALUE(0, 27),
	/* index 0..19 above are inputs of PART0 Mux */
	MUX_VALUE(1, 0),
	MUX_VALUE(1, 1),
	MUX_VALUE(1, 2),
	MUX_VALUE(1, 3),
	MUX_VALUE(1, 4),
	MUX_VALUE(1, 8),
	MUX_VALUE(1, 9),
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	MUX_VALUE(1, 10),
	MUX_VALUE(1, 11),
	MUX_VALUE(1, 16),
#endif
	MUX_VALUE(1, 20),
	MUX_VALUE(1, 21),
	MUX_VALUE(1, 24),
	MUX_VALUE(1, 25),
	MUX_VALUE(1, 26),
	MUX_VALUE(1, 27),
	MUX_VALUE(1, 28),
	MUX_VALUE(1, 29),
	/* index 20..34 above are inputs of PART1 Mux */
	MUX_VALUE(2, 0),
	MUX_VALUE(2, 4),
	MUX_VALUE(2, 8),
	MUX_VALUE(2, 9),
	MUX_VALUE(2, 12),
	MUX_VALUE(2, 13),
	MUX_VALUE(2, 14),
	MUX_VALUE(2, 15),
	MUX_VALUE(2, 18),
	MUX_VALUE(2, 19),
	MUX_VALUE(2, 20),
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	MUX_VALUE(2, 21),
#endif
	MUX_VALUE(2, 24),
	MUX_VALUE(2, 25),
	MUX_VALUE(2, 26),
	MUX_VALUE(2, 27),
	MUX_VALUE(2, 28),
	MUX_VALUE(2, 29),
	MUX_VALUE(2, 30),
	MUX_VALUE(2, 31),
	/* index 35..53 above are inputs of PART2 Mux */
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	MUX_VALUE(3, 0),
	MUX_VALUE(3, 1),
	MUX_VALUE(3, 2),
	MUX_VALUE(3, 3),
	MUX_VALUE(3, 4),
	MUX_VALUE(3, 5),
	MUX_VALUE(3, 6),
	MUX_VALUE(3, 7),
	MUX_VALUE(3, 16),
	MUX_VALUE(3, 17),
	MUX_VALUE(3, 18),
	MUX_VALUE(3, 19),
	MUX_VALUE(3, 24),
	MUX_VALUE(3, 25),
	MUX_VALUE(3, 26),
	MUX_VALUE(3, 27),
	MUX_VALUE(3, 28),
	MUX_VALUE(3, 29),
	/* index 54..71 above are inputs of PART3 Mux */
#endif
};

static const char * const tegra_virt_t210ref_source_text[] = {
	"None",
	"ADMAIF1",
	"ADMAIF2",
	"ADMAIF3",
	"ADMAIF4",
	"ADMAIF5",
	"ADMAIF6",
	"ADMAIF7",
	"ADMAIF8",
	"ADMAIF9",
	"ADMAIF10",
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	"ADMAIF11",
	"ADMAIF12",
	"ADMAIF13",
	"ADMAIF14",
	"ADMAIF15",
	"ADMAIF16",
#endif
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	"I2S6",
#endif
	"SFC1",
	"SFC2",
	"SFC3",
	"SFC4",
	/* index 0..19 above are inputs of PART0 Mux */
	"MIXER1-1",
	"MIXER1-2",
	"MIXER1-3",
	"MIXER1-4",
	"MIXER1-5",
	"AMX1",
	"AMX2",
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	"AMX3",
	"AMX4",
	"ARAD1",
#endif
	"SPDIF1-1",
	"SPDIF1-2",
	"AFC1",
	"AFC2",
	"AFC3",
	"AFC4",
	"AFC5",
	"AFC6",
	/* index 20..34 above are inputs of PART1 Mux */
	"OPE1",
	"SPKPROT1",
	"MVC1",
	"MVC2",
	"IQC1-1",
	"IQC1-2",
	"IQC2-1",
	"IQC2-2",
	"DMIC1",
	"DMIC2",
	"DMIC3",
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	"DMIC4",
#endif
	"ADX1-1",
	"ADX1-2",
	"ADX1-3",
	"ADX1-4",
	"ADX2-1",
	"ADX2-2",
	"ADX2-3",
	"ADX2-4",
	/* index 35..53 above are inputs of PART2 Mux */
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	"ADX3-1",
	"ADX3-2",
	"ADX3-3",
	"ADX3-4",
	"ADX4-1",
	"ADX4-2",
	"ADX4-3",
	"ADX4-4",
	"ADMAIF17",
	"ADMAIF18",
	"ADMAIF19",
	"ADMAIF20",
	"ASRC1-1",
	"ASRC1-2",
	"ASRC1-3",
	"ASRC1-4",
	"ASRC1-5",
	"ASRC1-6",
#endif
/* index 54..71 above are inputs of PART3 Mux */
};
static int tegra_virt_t210ref_get_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err, i = 0;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_XBAR_GET_ROUTE;
	msg.params.xbar_info.rx_reg = (int) reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_receive\n", __func__);

	for (i = 0; i < NUM_MUX_INPUT; i++) {
		if (msg.params.xbar_info.bit_pos ==
			tegra_virt_t210ref_source_value[i])
			break;
		}

	if (i == NUM_MUX_INPUT)
		ucontrol->value.integer.value[0] = 0;

	ucontrol->value.integer.value[0] = i;

	if (err < 0)
		return err;

	return 0;
}

static int tegra_virt_t210ref_put_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_XBAR_SET_ROUTE;
	msg.params.xbar_info.rx_reg = (int) reg;
	msg.params.xbar_info.tx_value =
	tegra_virt_t210ref_source_value
		[ucontrol->value.integer.value[0]];
	msg.params.xbar_info.tx_idx =
		ucontrol->value.integer.value[0] - 1;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210mixer_get_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int tegra_virt_t210mixer_set_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_RX_GAIN;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;
	msg.params.amixer_info.gain =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210mixer_get_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_TX_ADDER_CONFIG;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.adder_idx = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx = ((int) reg) & 0xFFFF;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_receive\n", __func__);

	ucontrol->value.integer.value[0] =
		msg.params.amixer_info.adder_rx_idx_enable;

	if (err < 0)
		return err;

	return 0;
}

static int tegra_virt_t210mixer_set_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_TX_ADDER_CONFIG;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.adder_idx = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx = ((int) reg) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210mixer_get_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_ENABLE;
	msg.params.amixer_info.id = 0;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_receive\n", __func__);

	ucontrol->value.integer.value[0] = msg.params.amixer_info.enable;

	if (err < 0)
		return err;

	return 0;
}

static int tegra_virt_t210mixer_set_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_ENABLE;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210sfc_get_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_GET_IN_FREQ;
	msg.params.sfc_info.id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.in_freq;

	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210sfc_set_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_SET_IN_FREQ;
	msg.params.sfc_info.id = reg;
	msg.params.sfc_info.in_freq =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210sfc_get_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_GET_OUT_FREQ;
	msg.params.sfc_info.id = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.out_freq;

	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	return 0;
}

static int tegra_virt_t210sfc_set_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_SET_OUT_FREQ;
	msg.params.sfc_info.id = reg;
	msg.params.sfc_info.out_freq =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
static int tegra186_virt_asrc_get_int_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_INT_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.int_ratio;

	return 0;
}
static int tegra186_virt_asrc_set_int_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_INT_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.int_ratio =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra186_virt_asrc_get_frac_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_FRAC_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.frac_ratio;

	return 0;
}
static int tegra186_virt_asrc_set_frac_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_FRAC_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.frac_ratio =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra186_virt_asrc_get_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_RATIO_SOURCE;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.ratio_source;

	return 0;
}

static int tegra186_virt_asrc_set_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_RATIO_SOURCE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.ratio_source =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra186_virt_asrc_get_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_STREAM_ENABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.stream_enable;

	return 0;
}

static int tegra186_virt_asrc_set_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_STREAM_ENABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.stream_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra186_virt_asrc_get_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_HWCOMP_DISABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.hwcomp_disable;

	return 0;
}

static int tegra186_virt_asrc_set_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_HWCOMP_DISABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.hwcomp_disable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra186_virt_asrc_get_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_INPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.out_freq;

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.input_threshold;

	return 0;
}

static int tegra186_virt_asrc_set_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_INPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.input_threshold =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}

static int tegra186_virt_asrc_get_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_OUTPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	err = nvaudio_ivc_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.asrc_info.output_threshold;

	return 0;
}

static int tegra186_virt_asrc_set_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_OUTPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.output_threshold =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
#endif

static const struct soc_enum tegra_virt_t210ref_source =
	SOC_ENUM_SINGLE_EXT(NUM_MUX_INPUT, tegra_virt_t210ref_source_text);

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
static const char * const tegra186_asrc_ratio_source_text[] = {
	"ARAD",
	"SW",
};

static const struct soc_enum tegra_virt_t186_asrc_source =
	SOC_ENUM_SINGLE_EXT(NUM_ASRC_SOURCES, tegra186_asrc_ratio_source_text);
#endif

#define MUX_REG(id) (TEGRA210_XBAR_RX_STRIDE * (id))
#define SOC_ENUM_EXT_REG(xname, xcount, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum,	\
	.tlv.p = (unsigned int *) xcount,	\
}

#define MUX_ENUM_CTRL_DECL(ename, reg) \
	SOC_ENUM_EXT_REG(ename, MUX_REG(reg),	\
	tegra_virt_t210ref_source,	\
	tegra_virt_t210ref_get_route,	\
	tegra_virt_t210ref_put_route)

#define MIXER_GAIN_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 0x20000, 0,	\
	tegra_virt_t210mixer_get_gain,	\
	tegra_virt_t210mixer_set_gain)

#define REG_PACK(id1, id2) ((id1 << MIXER_CONFIG_SHIFT_VALUE) | id2)
#define MIXER_ADDER_CTRL_DECL(ename, reg1, reg2) \
	SOC_SINGLE_EXT(ename, REG_PACK(reg1, reg2),  \
	0, 1, 0,	\
	tegra_virt_t210mixer_get_adder_config,	\
	tegra_virt_t210mixer_set_adder_config)

#define MIXER_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra_virt_t210mixer_get_enable,	\
	tegra_virt_t210mixer_set_enable)

#define SFC_IN_FREQ_CTRL_DECL(ename, id) \
	SOC_SINGLE_EXT(ename, id,	\
	0, 192000, 0,	\
	tegra_virt_t210sfc_get_in_freq,	\
	tegra_virt_t210sfc_set_in_freq)

#define SFC_OUT_FREQ_CTRL_DECL(ename, id) \
	SOC_SINGLE_EXT(ename, id,	\
	0, 192000, 0,	\
	tegra_virt_t210sfc_get_out_freq,	\
	tegra_virt_t210sfc_set_out_freq)

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#define ASRC_RATIO_INT_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, TEGRA186_ASRC_STREAM_RATIO_INTEGER_PART_MASK, 0,	\
	tegra186_virt_asrc_get_int_ratio,	\
	tegra186_virt_asrc_set_int_ratio)

#define SOC_SINGLE_EXT_FRAC(xname, xregbase, xmax, xget, xput) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_xr_sx, .get = xget, \
	.put  = xput, \
	.private_value = (unsigned long)&(struct soc_mreg_control) \
		{.regbase = xregbase, .regcount = 1, .nbits = 32, \
		.invert = 0, .min = 0, .max = xmax} }

#define ASRC_RATIO_FRAC_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT_FRAC(ename, reg,	\
	TEGRA186_ASRC_STREAM_RATIO_FRAC_PART_MASK,	\
	tegra186_virt_asrc_get_frac_ratio,	\
	tegra186_virt_asrc_set_frac_ratio)

#define ASRC_STREAM_RATIO_CTRL_DECL(ename, reg) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	tegra_virt_t186_asrc_source,	\
	tegra186_virt_asrc_get_ratio_source,	\
	tegra186_virt_asrc_set_ratio_source)

#define ASRC_STREAM_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra186_virt_asrc_get_stream_enable,	\
	tegra186_virt_asrc_set_stream_enable)

#define ASRC_STREAM_HWCOMP_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra186_virt_asrc_get_hwcomp_disable,	\
	tegra186_virt_asrc_set_hwcomp_disable)

#define ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 3, 0,	\
	tegra186_virt_asrc_get_input_threshold,	\
	tegra186_virt_asrc_set_input_threshold)

#define ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 3, 0,	\
	tegra186_virt_asrc_get_output_threshold,	\
	tegra186_virt_asrc_set_output_threshold)
#endif

static const struct snd_kcontrol_new tegra_virt_t210ref_controls[] = {
MUX_ENUM_CTRL_DECL("ADMAIF1 Mux", 0x00),
MUX_ENUM_CTRL_DECL("ADMAIF2 Mux", 0x01),
MUX_ENUM_CTRL_DECL("ADMAIF3 Mux", 0x02),
MUX_ENUM_CTRL_DECL("ADMAIF4 Mux", 0x03),
MUX_ENUM_CTRL_DECL("ADMAIF5 Mux", 0x04),
MUX_ENUM_CTRL_DECL("ADMAIF6 Mux", 0x05),
MUX_ENUM_CTRL_DECL("ADMAIF7 Mux", 0x06),
MUX_ENUM_CTRL_DECL("ADMAIF8 Mux", 0x07),
MUX_ENUM_CTRL_DECL("ADMAIF9 Mux", 0x08),
MUX_ENUM_CTRL_DECL("ADMAIF10 Mux", 0x09),
MUX_ENUM_CTRL_DECL("I2S1 Mux", 0x10),
MUX_ENUM_CTRL_DECL("I2S2 Mux", 0x11),
MUX_ENUM_CTRL_DECL("I2S3 Mux", 0x12),
MUX_ENUM_CTRL_DECL("I2S4 Mux", 0x13),
MUX_ENUM_CTRL_DECL("I2S5 Mux", 0x14),
MUX_ENUM_CTRL_DECL("SFC1 Mux", 0x18),
MUX_ENUM_CTRL_DECL("SFC2 Mux", 0x19),
MUX_ENUM_CTRL_DECL("SFC3 Mux", 0x1a),
MUX_ENUM_CTRL_DECL("SFC4 Mux", 0x1b),
MUX_ENUM_CTRL_DECL("MIXER1-1 Mux", 0x20),
MUX_ENUM_CTRL_DECL("MIXER1-2 Mux", 0x21),
MUX_ENUM_CTRL_DECL("MIXER1-3 Mux", 0x22),
MUX_ENUM_CTRL_DECL("MIXER1-4 Mux", 0x23),
MUX_ENUM_CTRL_DECL("MIXER1-5 Mux", 0x24),
MUX_ENUM_CTRL_DECL("MIXER1-6 Mux", 0x25),
MUX_ENUM_CTRL_DECL("MIXER1-7 Mux", 0x26),
MUX_ENUM_CTRL_DECL("MIXER1-8 Mux", 0x27),
MUX_ENUM_CTRL_DECL("MIXER1-9 Mux", 0x28),
MUX_ENUM_CTRL_DECL("MIXER1-10 Mux", 0x29),
MUX_ENUM_CTRL_DECL("SPDIF1-1 Mux", 0x34),
MUX_ENUM_CTRL_DECL("SPDIF1-2 Mux", 0x35),
MUX_ENUM_CTRL_DECL("AFC1 Mux", 0x38),
MUX_ENUM_CTRL_DECL("AFC2 Mux", 0x39),
MUX_ENUM_CTRL_DECL("AFC3 Mux", 0x3a),
MUX_ENUM_CTRL_DECL("AFC4 Mux", 0x3b),
MUX_ENUM_CTRL_DECL("AFC5 Mux", 0x3c),
MUX_ENUM_CTRL_DECL("AFC6 Mux", 0x3d),
MUX_ENUM_CTRL_DECL("OPE1 Mux", 0x40),
MUX_ENUM_CTRL_DECL("SPKPROT1 Mux", 0x44),
MUX_ENUM_CTRL_DECL("MVC1 Mux", 0x48),
MUX_ENUM_CTRL_DECL("MVC2 Mux", 0x49),
MUX_ENUM_CTRL_DECL("AMX1-1 Mux", 0x50),
MUX_ENUM_CTRL_DECL("AMX1-2 Mux", 0x51),
MUX_ENUM_CTRL_DECL("AMX1-3 Mux", 0x52),
MUX_ENUM_CTRL_DECL("AMX1-4 Mux", 0x53),
MUX_ENUM_CTRL_DECL("AMX2-1 Mux", 0x54),
MUX_ENUM_CTRL_DECL("AMX2-2 Mux", 0x55),
MUX_ENUM_CTRL_DECL("AMX2-3 Mux", 0x56),
MUX_ENUM_CTRL_DECL("AMX2-4 Mux", 0x57),
MUX_ENUM_CTRL_DECL("ADX1 Mux", 0x60),
MUX_ENUM_CTRL_DECL("ADX2 Mux", 0x61),
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
MUX_ENUM_CTRL_DECL("ADMAIF11 Mux", 0x0a),
MUX_ENUM_CTRL_DECL("ADMAIF12 Mux", 0x0b),
MUX_ENUM_CTRL_DECL("ADMAIF13 Mux", 0x0c),
MUX_ENUM_CTRL_DECL("ADMAIF14 Mux", 0x0d),
MUX_ENUM_CTRL_DECL("ADMAIF15 Mux", 0x0e),
MUX_ENUM_CTRL_DECL("ADMAIF16 Mux", 0x0f),
MUX_ENUM_CTRL_DECL("ADMAIF17 Mux", 0x68),
MUX_ENUM_CTRL_DECL("ADMAIF18 Mux", 0x69),
MUX_ENUM_CTRL_DECL("ADMAIF19 Mux", 0x6a),
MUX_ENUM_CTRL_DECL("ADMAIF20 Mux", 0x6b),
MUX_ENUM_CTRL_DECL("I2S6 Mux", 0x15),
MUX_ENUM_CTRL_DECL("AMX3-1 Mux", 0x58),
MUX_ENUM_CTRL_DECL("AMX3-2 Mux", 0x59),
MUX_ENUM_CTRL_DECL("AMX3-3 Mux", 0x5a),
MUX_ENUM_CTRL_DECL("AMX3-4 Mux", 0x5b),
MUX_ENUM_CTRL_DECL("AMX4-1 Mux", 0x64),
MUX_ENUM_CTRL_DECL("AMX4-2 Mux", 0x65),
MUX_ENUM_CTRL_DECL("AMX4-3 Mux", 0x66),
MUX_ENUM_CTRL_DECL("AMX4-4 Mux", 0x67),
MUX_ENUM_CTRL_DECL("ADX3 Mux", 0x62),
MUX_ENUM_CTRL_DECL("ADX4 Mux", 0x63),
MUX_ENUM_CTRL_DECL("ASRC1-1 Mux", 0x6c),
MUX_ENUM_CTRL_DECL("ASRC1-2 Mux", 0x6d),
MUX_ENUM_CTRL_DECL("ASRC1-3 Mux", 0x6e),
MUX_ENUM_CTRL_DECL("ASRC1-4 Mux", 0x6f),
MUX_ENUM_CTRL_DECL("ASRC1-5 Mux", 0x70),
MUX_ENUM_CTRL_DECL("ASRC1-6 Mux", 0x71),
MUX_ENUM_CTRL_DECL("ASRC1-7 Mux", 0x72),
MUX_ENUM_CTRL_DECL("DSPK1 Mux", 0x30),
MUX_ENUM_CTRL_DECL("DSPK2 Mux", 0x31),
#endif
MIXER_GAIN_CTRL_DECL("RX1 Gain", 0x00),
MIXER_GAIN_CTRL_DECL("RX2 Gain", 0x01),
MIXER_GAIN_CTRL_DECL("RX3 Gain", 0x02),
MIXER_GAIN_CTRL_DECL("RX4 Gain", 0x03),
MIXER_GAIN_CTRL_DECL("RX5 Gain", 0x04),
MIXER_GAIN_CTRL_DECL("RX6 Gain", 0x05),
MIXER_GAIN_CTRL_DECL("RX7 Gain", 0x06),
MIXER_GAIN_CTRL_DECL("RX8 Gain", 0x07),
MIXER_GAIN_CTRL_DECL("RX9 Gain", 0x08),
MIXER_GAIN_CTRL_DECL("RX10 Gain", 0x09),

MIXER_ADDER_CTRL_DECL("Adder1 RX1", 0x00, 0x01),
MIXER_ADDER_CTRL_DECL("Adder1 RX2", 0x00, 0x02),
MIXER_ADDER_CTRL_DECL("Adder1 RX3", 0x00, 0x03),
MIXER_ADDER_CTRL_DECL("Adder1 RX4", 0x00, 0x04),
MIXER_ADDER_CTRL_DECL("Adder1 RX5", 0x00, 0x05),
MIXER_ADDER_CTRL_DECL("Adder1 RX6", 0x00, 0x06),
MIXER_ADDER_CTRL_DECL("Adder1 RX7", 0x00, 0x07),
MIXER_ADDER_CTRL_DECL("Adder1 RX8", 0x00, 0x08),
MIXER_ADDER_CTRL_DECL("Adder1 RX9", 0x00, 0x09),
MIXER_ADDER_CTRL_DECL("Adder1 RX10", 0x00, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder2 RX1", 0x01, 0x01),
MIXER_ADDER_CTRL_DECL("Adder2 RX2", 0x01, 0x02),
MIXER_ADDER_CTRL_DECL("Adder2 RX3", 0x01, 0x03),
MIXER_ADDER_CTRL_DECL("Adder2 RX4", 0x01, 0x04),
MIXER_ADDER_CTRL_DECL("Adder2 RX5", 0x01, 0x05),
MIXER_ADDER_CTRL_DECL("Adder2 RX6", 0x01, 0x06),
MIXER_ADDER_CTRL_DECL("Adder2 RX7", 0x01, 0x07),
MIXER_ADDER_CTRL_DECL("Adder2 RX8", 0x01, 0x08),
MIXER_ADDER_CTRL_DECL("Adder2 RX9", 0x01, 0x09),
MIXER_ADDER_CTRL_DECL("Adder2 RX10", 0x01, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder3 RX1", 0x02, 0x01),
MIXER_ADDER_CTRL_DECL("Adder3 RX2", 0x02, 0x02),
MIXER_ADDER_CTRL_DECL("Adder3 RX3", 0x02, 0x03),
MIXER_ADDER_CTRL_DECL("Adder3 RX4", 0x02, 0x04),
MIXER_ADDER_CTRL_DECL("Adder3 RX5", 0x02, 0x05),
MIXER_ADDER_CTRL_DECL("Adder3 RX6", 0x02, 0x06),
MIXER_ADDER_CTRL_DECL("Adder3 RX7", 0x02, 0x07),
MIXER_ADDER_CTRL_DECL("Adder3 RX8", 0x02, 0x08),
MIXER_ADDER_CTRL_DECL("Adder3 RX9", 0x02, 0x09),
MIXER_ADDER_CTRL_DECL("Adder3 RX10", 0x02, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder4 RX1", 0x03, 0x01),
MIXER_ADDER_CTRL_DECL("Adder4 RX2", 0x03, 0x02),
MIXER_ADDER_CTRL_DECL("Adder4 RX3", 0x03, 0x03),
MIXER_ADDER_CTRL_DECL("Adder4 RX4", 0x03, 0x04),
MIXER_ADDER_CTRL_DECL("Adder4 RX5", 0x03, 0x05),
MIXER_ADDER_CTRL_DECL("Adder4 RX6", 0x03, 0x06),
MIXER_ADDER_CTRL_DECL("Adder4 RX7", 0x03, 0x07),
MIXER_ADDER_CTRL_DECL("Adder4 RX8", 0x03, 0x08),
MIXER_ADDER_CTRL_DECL("Adder4 RX9", 0x03, 0x09),
MIXER_ADDER_CTRL_DECL("Adder4 RX10", 0x03, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder5 RX1", 0x04, 0x01),
MIXER_ADDER_CTRL_DECL("Adder5 RX2", 0x04, 0x02),
MIXER_ADDER_CTRL_DECL("Adder5 RX3", 0x04, 0x03),
MIXER_ADDER_CTRL_DECL("Adder5 RX4", 0x04, 0x04),
MIXER_ADDER_CTRL_DECL("Adder5 RX5", 0x04, 0x05),
MIXER_ADDER_CTRL_DECL("Adder5 RX6", 0x04, 0x06),
MIXER_ADDER_CTRL_DECL("Adder5 RX7", 0x04, 0x07),
MIXER_ADDER_CTRL_DECL("Adder5 RX8", 0x04, 0x08),
MIXER_ADDER_CTRL_DECL("Adder5 RX9", 0x04, 0x09),
MIXER_ADDER_CTRL_DECL("Adder5 RX10", 0x04, 0x0a),

MIXER_ENABLE_CTRL_DECL("Mixer Enable", 0x00),

SFC_IN_FREQ_CTRL_DECL("SFC1 in freq", 0x00),
SFC_IN_FREQ_CTRL_DECL("SFC2 in freq", 0x01),
SFC_IN_FREQ_CTRL_DECL("SFC3 in freq", 0x02),
SFC_IN_FREQ_CTRL_DECL("SFC4 in freq", 0x03),

SFC_OUT_FREQ_CTRL_DECL("SFC1 out freq", 0x00),
SFC_OUT_FREQ_CTRL_DECL("SFC2 out freq", 0x01),
SFC_OUT_FREQ_CTRL_DECL("SFC3 out freq", 0x02),
SFC_OUT_FREQ_CTRL_DECL("SFC4 out freq", 0x03),

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio1 Int", 0x01),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio2 Int", 0x02),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio3 Int", 0x03),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio4 Int", 0x04),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio5 Int", 0x05),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio6 Int", 0x06),

ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio1 Frac", 0x01),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio2 Frac", 0x02),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio3 Frac", 0x03),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio4 Frac", 0x04),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio5 Frac", 0x05),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio6 Frac", 0x06),

ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio1 SRC", 0x01),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio2 SRC", 0x02),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio3 SRC", 0x03),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio4 SRC", 0x04),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio5 SRC", 0x05),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio6 SRC", 0x06),

ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream1 Enable", 0x01),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream2 Enable", 0x02),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream3 Enable", 0x03),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream4 Enable", 0x04),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream5 Enable", 0x05),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream6 Enable", 0x06),

ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio1 Hwcomp Disable", 0x01),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio2 Hwcomp Disable", 0x02),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio3 Hwcomp Disable", 0x03),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio4 Hwcomp Disable", 0x04),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio5 Hwcomp Disable", 0x05),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio6 Hwcomp Disable", 0x06),

ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream1 Input Thresh", 0x01),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream2 Input Thresh", 0x02),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream3 Input Thresh", 0x03),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream4 Input Thresh", 0x04),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream5 Input Thresh", 0x05),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream6 Input Thresh", 0x06),

ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream1 Output Thresh", 0x01),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream2 Output Thresh", 0x02),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream3 Output Thresh", 0x03),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream4 Output Thresh", 0x04),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream5 Output Thresh", 0x05),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream6 Output Thresh", 0x06),
#endif
};

static const struct of_device_id tegra_virt_t210ref_pcm_of_match[] = {
	{ .compatible = "nvidia,tegra210-virt-pcm", },
	{},
};

static const struct snd_soc_dapm_widget tegra_virt_t210ref_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_LINE("LineIn", NULL),
};

static const struct snd_soc_dapm_route tegra_virt_t210ref_dapm_routes[] = {
	{"Headphone", NULL, "OUT"},
	{"IN", NULL, "LineIn"},
};

static struct snd_soc_card tegra_virt_t210ref_pcm = {
	.name = "t210ref-virt-pcm",
	.owner = THIS_MODULE,
	.dai_link = tegra_virt_t210ref_pcm_links,
	.num_links = ARRAY_SIZE(tegra_virt_t210ref_pcm_links),
	.dapm_widgets = tegra_virt_t210ref_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_virt_t210ref_dapm_widgets),
	.dapm_routes = tegra_virt_t210ref_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(tegra_virt_t210ref_dapm_routes),
	.controls = tegra_virt_t210ref_controls,
	.num_controls = ARRAY_SIZE(tegra_virt_t210ref_controls),
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
	struct snd_soc_card *card = &tegra_virt_t210ref_pcm;
	int i, ret = 0;
	int admaif_ch_num = 0;
	bool adsp_enabled = false;
	unsigned int admaif_ch_list[MAX_ADMAIF_IDS];

	card->dev = &pdev->dev;


	adsp_enabled = of_property_read_bool(pdev->dev.of_node,
		"adsp_enabled");

	if (!adsp_enabled)
		dev_info(&pdev->dev, "adsp config is not set\n");

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	if (!adsp_enabled)
		tegra_virt_t210ref_pcm.num_links = LAST_ADMAIF_DAI_LINK + 1;

	for (i = 0; i < card->num_links; i++) {
		if (i < (LAST_ADSP_ADMAIF_DAI_LINK+1)) {
			card->dai_link[i].codec_of_node =
			of_parse_phandle(pdev->dev.of_node, "codec", 0);
		}
		if (i < (LAST_ADMAIF_DAI_LINK+1)) {
			strcpy((char *)card->dai_link[i].cpu_name,
					DRV_NAME_T186);
		}
	}
#endif

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

	tegra_pd_add_device(&pdev->dev);
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
