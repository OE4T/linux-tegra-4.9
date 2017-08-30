/*
 * tegra_asoc_util_virt_alt.h - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __LINUX_VIRT_UTIL_H
#define __LINUX_VIRT_UTIL_H

#include <sound/soc.h>

#define MIXER_CONFIG_SHIFT_VALUE 16
#define TEGRA186_ASRC_STREAM_RATIO_INTEGER_PART_MASK		0x1F
#define TEGRA186_ASRC_STREAM_RATIO_FRAC_PART_MASK		0xFFFFFFFF
#define TEGRA186_ASRC_STREAM_RATIO_MASK				0x1FFFFFFFFF
#define NUM_ARAD_SOURCES	11
#define NUM_ARAD_LANES		6
#define NUM_ASRC_MODE		2
#define NUM_MVC_CURVETYPE	2
#define MAX_MVC_TAR_VOL         16000

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

#define MVC_CURVE_TYPE_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	src,	\
	tegra_virt_t210mvc_get_curve_type,	\
	tegra_virt_t210mvc_set_curve_type)

#define MVC_TAR_VOL_CTRL_DECL(ename, id) \
	SOC_SINGLE_EXT(ename, id,	\
	0, MAX_MVC_TAR_VOL, 0,	\
	tegra_virt_t210mvc_get_tar_vol,	\
	tegra_virt_t210mvc_set_tar_vol)

#define MVC_MUTE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra_virt_t210mvc_get_mute,	\
	tegra_virt_t210mvc_set_mute)

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

#define ASRC_RATIO_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT_FRAC(ename, reg,	\
	TEGRA186_ASRC_STREAM_RATIO_MASK,	\
	tegra186_virt_asrc_get_ratio,	\
	tegra186_virt_asrc_set_ratio)

#define ASRC_RATIO_FRAC_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT_FRAC(ename, reg,	\
	TEGRA186_ASRC_STREAM_RATIO_FRAC_PART_MASK,	\
	tegra186_virt_asrc_get_frac_ratio,	\
	tegra186_virt_asrc_set_frac_ratio)

#define ASRC_STREAM_RATIO_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	src,	\
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

#define AMX_ENABLE_CTRL_DECL(ename, reg1, reg2) \
	SOC_SINGLE_EXT(ename, REG_PACK(reg1, reg2),  \
	0, 1, 0,	\
	tegra_virt_t210_amx_get_input_stream_enable,	\
	tegra_virt_t210_amx_set_input_stream_enable)

#define ARAD_LANE_SOURCE_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	src,	\
	tegra186_virt_arad_get_lane_source,	\
	tegra186_virt_arad_set_lane_source)

#define ARAD_LANE_PRESCALAR_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 65535, 0,	\
	tegra186_virt_arad_get_lane_prescalar,	\
	tegra186_virt_arad_set_lane_prescalar)

#define ARAD_LANE_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra186_virt_arad_get_lane_enable,	\
	tegra186_virt_arad_set_lane_enable)

#define ARAD_LANE_RATIO_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 0xFFFFFFFF, 0,	\
	tegra186_virt_arad_get_lane_ratio, NULL)

#define I2S_LOOPBACK_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra_virt_i2s_get_loopback_enable,	\
	tegra_virt_i2s_set_loopback_enable)

enum {
	numerator1_enum = 0,
	numerator2_enum,
	numerator3_enum,
	numerator4_enum,
	numerator5_enum,
	numerator6_enum,
	denominator1_enum = NUM_ARAD_LANES,
	denominator2_enum,
	denominator3_enum,
	denominator4_enum,
	denominator5_enum,
	denominator6_enum,
};

extern const int tegra186_arad_mux_value[];
extern const char * const tegra186_arad_mux_text[];
extern const char * const tegra186_asrc_ratio_source_text[];
extern const char * const tegra210_mvc_curve_type_text[];

int tegra_virt_t210mixer_get_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_set_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_get_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_set_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_get_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_set_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210sfc_get_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210sfc_set_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210sfc_get_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210sfc_set_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mvc_get_curve_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mvc_set_curve_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mvc_get_tar_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mvc_set_tar_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mvc_get_mute(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mvc_set_mute(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_int_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_int_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_frac_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_frac_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_get_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_set_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_get_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_get_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_set_output_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210_amx_get_input_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210_amx_set_input_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);


int tegra186_virt_arad_get_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_arad_set_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_get_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_set_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_get_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_set_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_get_lane_ratio(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_i2s_set_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_i2s_get_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

#endif
