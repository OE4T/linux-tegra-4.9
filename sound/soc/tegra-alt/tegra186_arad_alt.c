/*
 * tegra186_arad_alt.c - Tegra186 ARAD driver
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/delay.h>
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
#include "tegra186_arad_alt.h"

#define DRV_NAME "tegra186-arad"

#define ARAD_LANE_NUMERATOR_MUX(id)	\
	(TEGRA186_ARAD_LANE1_NUMERATOR_MUX_SEL + id*TEGRA186_ARAD_LANE_STRIDE)
#define ARAD_LANE_DENOMINATOR_MUX(id)	\
	(TEGRA186_ARAD_LANE1_DENOMINATOR_MUX_SEL + id*TEGRA186_ARAD_LANE_STRIDE)

#define ARAD_LANE_REG(reg, id) (reg + (id * TEGRA186_ARAD_LANE_STRIDE))

#define ASRC_STREAM_REG_DEFAULTS(id) \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_NUMERATOR_MUX_SEL, id), 0x0}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_RATIO_INTEGER_PART, id), 0x0}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_RATIO_FRACTIONAL_PART, id), 0x0}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_NUMERATOR_PRESCALAR, id), 0x0}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_DENOMINATOR_MUX_SEL, id), 0x0}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_DENOMINATOR_PRESCALAR, id), 0x0}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_SERVO_LOOP_CONFIG, id), 0xd5e7}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_LOCK_UNLOCK_DETECTOR_CONFIG, id), 0x840500}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_ERROR_LOCK_THRESHOLD, id), 0x400000}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_ERROR_UNLOCK_THRESHOLD, id), 0xa00000}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_RATIO_CALCULATOR_CONFIG, id), 0xf000006}, \
	{ ARAD_LANE_REG(TEGRA186_ARAD_LANE1_CYA, id), 0x0}

static const struct reg_default tegra186_arad_reg_defaults[] = {
	{ TEGRA186_ARAD_LANE_ENABLE, 0x0},
	{ TEGRA186_ARAD_LANE_SOFT_RESET, 0x0},
	{ TEGRA186_ARAD_LANE_INT_MASK, 0x0},
	{ TEGRA186_ARAD_LANE_INT_SET, 0x0},
	{ TEGRA186_ARAD_LANE_INT_CLEAR, 0x0},
	{ TEGRA186_ARAD_LANE_INT_CLEAR, 0x0},
	{ TEGRA186_ARAD_CG, 0x0},
	{ TEGRA186_ARAD_CYA_GLOBAL, 0x0},

	ASRC_STREAM_REG_DEFAULTS(0),
	ASRC_STREAM_REG_DEFAULTS(1),
	ASRC_STREAM_REG_DEFAULTS(2),
	ASRC_STREAM_REG_DEFAULTS(3),
	ASRC_STREAM_REG_DEFAULTS(4),
	ASRC_STREAM_REG_DEFAULTS(5),

	{ TEGRA186_ARAD_TX_CIF_CTRL, 0x115500},
};

static int tegra186_arad_runtime_suspend(struct device *dev)
{
	struct tegra186_arad *arad = dev_get_drvdata(dev);

	regcache_cache_only(arad->regmap, true);

	pm_runtime_put_sync(dev->parent);

	return 0;
}

static int tegra186_arad_runtime_resume(struct device *dev)
{
	struct tegra186_arad *arad = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_get_sync(dev->parent);
	if (ret < 0) {
		dev_err(dev, "parent get_sync failed: %d\n", ret);
		return ret;
	}

	regcache_cache_only(arad->regmap, false);
	regcache_sync(arad->regmap);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra186_arad_suspend(struct device *dev)
{
	struct tegra186_arad *arad = dev_get_drvdata(dev);

	regcache_mark_dirty(arad->regmap);

	return 0;
}
#endif

static int tegra186_arad_codec_probe(struct snd_soc_codec *codec)
{
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);

	codec->control_data = arad->regmap;

	return 0;
}

static int tegra186_arad_get_lane_lock_status(
	struct tegra186_arad *arad, unsigned int lane_id)
{
	unsigned int val;

	regmap_read(arad->regmap,
		TEGRA186_ARAD_LANE_STATUS, &val);
	val = (val >> (16 + lane_id)) & 0x1;

	return val;
}

static struct snd_soc_dai_ops tegra186_arad_out_dai_ops = {
};

static struct snd_soc_dai_driver tegra186_arad_dais[] = {
	{
		.name = "ARAD OUT",
		.capture = {
			.stream_name = "ARAD Transmit",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S24_LE,
		},
		.ops = &tegra186_arad_out_dai_ops,
	},
};

static const int tegra186_arad_mux_value[] = {
	-1, /* None */
	0, 1, 2, 3, 4, 5,	/* I2S1~6 */
	12, 13, 14, 15,	/* DMIC1~4 */
	24, 25,	/* DSPK1~2 */
	26, 27,	/* IQC1~2 */
	28, 29, 30, 31,	/* SPDIF_RX1,2 & SPDIF_TX1,2 */
};

static const char * const tegra186_arad_mux_text[] = {
	"None",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
	"I2S6",
	"DMIC1",
	"DMIC2",
	"DMIC3",
	"DMIC4",
	"DSPK1",
	"DSPK2",
	"IQC1",
	"IQC2",
	"SPDIF1_RX1",
	"SPDIF1_RX2",
	"SPDIF1_TX1",
	"SPDIF1_TX2",
};

static int tegra186_arad_mux_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *arad_private =
		(struct soc_enum  *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);
	unsigned int val, loop = 0;

	regmap_read(arad->regmap, arad_private->reg, &val);

	if (val) {
		for (loop = 1; loop < 19; loop++)
			if (val & (1<<arad_private->values[loop])) {
				val = loop;
				break;
			}
	}

	ucontrol->value.integer.value[0] = val;

	return 0;
}

static int tegra186_arad_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *arad_private =
		(struct soc_enum  *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);
	unsigned int val = ucontrol->value.integer.value[0];

	if (!val)
		regmap_write(arad->regmap, arad_private->reg, 0);
	else
		regmap_write(arad->regmap, arad_private->reg,
			1 << arad_private->values[val]);

	return 0;
}

static int tegra186_arad_get_ratio_int(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *arad_private =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);
	unsigned int val;

	regmap_read(arad->regmap, arad_private->reg, &val);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

static int tegra186_arad_get_ratio_frac(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *arad_private =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);
	unsigned int val;

	regmap_read(arad->regmap, arad_private->reg, &val);
	ucontrol->value.integer.value[0] = val;

	return 0;
}

static int tegra186_arad_get_enable_lane(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *arad_private =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);
	unsigned int enable;

	regmap_read(arad->regmap, arad_private->reg, &enable);
	enable = (enable >> arad_private->shift) & arad_private->max;
	ucontrol->value.integer.value[0] = enable;

	return 0;
}

static int tegra186_arad_put_enable_lane(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *arad_private =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra186_arad *arad = snd_soc_codec_get_drvdata(codec);
	unsigned int enable = 0, lane_id = arad_private->shift;
	int dcnt = 10;

	enable = ucontrol->value.integer.value[0];

	regmap_update_bits(arad->regmap,
		arad_private->reg, 1<<arad_private->shift,
		enable<<arad_private->shift);

	if (enable)
		while (!tegra186_arad_get_lane_lock_status(arad, lane_id) &&
			dcnt--)
			udelay(100);
	else {
		regmap_update_bits(arad->regmap,
			TEGRA186_ARAD_LANE_SOFT_RESET, 1<<arad_private->shift,
			1<<arad_private->shift);
		while (tegra186_arad_get_lane_lock_status(arad, lane_id) &&
			dcnt--)
			udelay(100);
	}

	if (dcnt < 0) {
		if (enable)
			pr_err("ARAD Lane %d can't be locked\n", lane_id+1);
		else
			pr_err("ARAD Lane %d can't be unlocked\n", lane_id+1);
		return -ETIMEDOUT;
	} else
		return 0;
}

#define SOC_VALUE_ENUM_WIDE(xreg, shift, xmax, xtexts, xvalues) \
{	.reg = xreg, .shift_l = shift, .shift_r = shift, \
	.items = xmax, .texts = xtexts, .values = xvalues, \
	.mask = xmax ? roundup_pow_of_two(xmax) - 1 : 0}

#define SOC_VALUE_ENUM_WIDE_DECL(name, xreg, shift, \
		xtexts, xvalues) \
	struct soc_enum name = SOC_VALUE_ENUM_WIDE(xreg, shift, \
					ARRAY_SIZE(xtexts), xtexts, xvalues)

#define ARAD_MUX_ENUM_CTRL_DECL(ename, reg)		\
	SOC_VALUE_ENUM_WIDE_DECL(ename##_enum, reg, 0,	\
			tegra186_arad_mux_text, tegra186_arad_mux_value); \
	static const struct snd_kcontrol_new ename##_ctrl =		\
		SOC_DAPM_ENUM_EXT("Mux", ename##_enum,		\
			tegra186_arad_mux_get,	\
			tegra186_arad_mux_put)

static ARAD_MUX_ENUM_CTRL_DECL(numerator1,
		TEGRA186_ARAD_LANE1_NUMERATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(numerator2,
		TEGRA186_ARAD_LANE2_NUMERATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(numerator3,
		TEGRA186_ARAD_LANE3_NUMERATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(numerator4,
		TEGRA186_ARAD_LANE4_NUMERATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(numerator5,
		TEGRA186_ARAD_LANE5_NUMERATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(numerator6,
		TEGRA186_ARAD_LANE6_NUMERATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(denominator1,
		TEGRA186_ARAD_LANE1_DENOMINATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(denominator2,
		TEGRA186_ARAD_LANE2_DENOMINATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(denominator3,
		TEGRA186_ARAD_LANE3_DENOMINATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(denominator4,
		TEGRA186_ARAD_LANE4_DENOMINATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(denominator5,
		TEGRA186_ARAD_LANE5_DENOMINATOR_MUX_SEL);
static ARAD_MUX_ENUM_CTRL_DECL(denominator6,
		TEGRA186_ARAD_LANE6_DENOMINATOR_MUX_SEL);

#define ARAD_RATIO_GEN_WIDGETS(sname, ename)	\
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0),	\
	SND_SOC_DAPM_MUX(sname " MUX", SND_SOC_NOPM, 0, 0, &ename##_ctrl)

static const struct snd_soc_dapm_widget tegra186_arad_widgets[] = {
	SND_SOC_DAPM_SIGGEN("I2S1 SIG"),
	SND_SOC_DAPM_SIGGEN("I2S2 SIG"),
	SND_SOC_DAPM_SIGGEN("I2S3 SIG"),
	SND_SOC_DAPM_SIGGEN("I2S4 SIG"),
	SND_SOC_DAPM_SIGGEN("I2S5 SIG"),
	SND_SOC_DAPM_SIGGEN("I2S6 SIG"),
	SND_SOC_DAPM_SIGGEN("DMIC1 SIG"),
	SND_SOC_DAPM_SIGGEN("DMIC2 SIG"),
	SND_SOC_DAPM_SIGGEN("DMIC3 SIG"),
	SND_SOC_DAPM_SIGGEN("DMIC4 SIG"),
	SND_SOC_DAPM_SIGGEN("DSPK1 SIG"),
	SND_SOC_DAPM_SIGGEN("DSPK2 SIG"),
	SND_SOC_DAPM_SIGGEN("IQC1 SIG"),
	SND_SOC_DAPM_SIGGEN("IQC2 SIG"),
	SND_SOC_DAPM_SIGGEN("SPDIF1_RX1 SIG"),
	SND_SOC_DAPM_SIGGEN("SPDIF1_RX2 SIG"),
	SND_SOC_DAPM_SIGGEN("SPDIF1_TX1 SIG"),
	SND_SOC_DAPM_SIGGEN("SPDIF1_TX2 SIG"),

	ARAD_RATIO_GEN_WIDGETS("Numerator1", numerator1),
	ARAD_RATIO_GEN_WIDGETS("Numerator2", numerator2),
	ARAD_RATIO_GEN_WIDGETS("Numerator3", numerator3),
	ARAD_RATIO_GEN_WIDGETS("Numerator4", numerator4),
	ARAD_RATIO_GEN_WIDGETS("Numerator5", numerator5),
	ARAD_RATIO_GEN_WIDGETS("Numerator6", numerator6),
	ARAD_RATIO_GEN_WIDGETS("Denominator1", denominator1),
	ARAD_RATIO_GEN_WIDGETS("Denominator2", denominator2),
	ARAD_RATIO_GEN_WIDGETS("Denominator3", denominator3),
	ARAD_RATIO_GEN_WIDGETS("Denominator4", denominator4),
	ARAD_RATIO_GEN_WIDGETS("Denominator5", denominator5),
	ARAD_RATIO_GEN_WIDGETS("Denominator6", denominator6),

	SND_SOC_DAPM_AIF_OUT("Packetizer", NULL, 0, 0, 0, 0),
};

#define RATIO_GEN_ROUTE(sname)	\
	{ sname " MUX",		NULL,	"I2S1 SIG" },	\
	{ sname " MUX",		NULL,	"I2S2 SIG" },	\
	{ sname " MUX",		NULL,	"I2S3 SIG" },	\
	{ sname " MUX",		NULL,	"I2S4 SIG" },	\
	{ sname " MUX",		NULL,	"I2S5 SIG" },	\
	{ sname " MUX",		NULL,	"I2S6 SIG" },	\
	{ sname " MUX",		NULL,	"DMIC1 SIG" },	\
	{ sname " MUX",		NULL,	"DMIC2 SIG" },	\
	{ sname " MUX",		NULL,	"DMIC3 SIG" },	\
	{ sname " MUX",		NULL,	"DMIC4 SIG" },	\
	{ sname " MUX",		NULL,	"IQC1 SIG" },	\
	{ sname " MUX",		NULL,	"IQC2 SIG" },	\
	{ sname " MUX",		NULL,	"SPDIF1_RX1 SIG" },	\
	{ sname " MUX",		NULL,	"SPDIF1_RX2 SIG" },	\
	{ sname " MUX",		NULL,	"SPDIF1_TX1 SIG" },	\
	{ sname " MUX",		NULL,	"SPDIF1_TX2 SIG" }

#define RATIO_GEN(sname)	\
	RATIO_GEN_ROUTE(sname),	\
	{ sname " TX",		NULL,		sname " MUX" }

#define PACKETIZER_ROUTE(sname)	\
	{ sname,	"Numerator1 TX",	"Numerator1 TX" },	\
	{ sname,	"Numerator2 TX",	"Numerator2 TX" },	\
	{ sname,	"Numerator3 TX",	"Numerator3 TX" },	\
	{ sname,	"Numerator4 TX",	"Numerator4 TX" },	\
	{ sname,	"Numerator5 TX",	"Numerator5 TX" },	\
	{ sname,	"Numerator6 TX",	"Numerator6 TX" },	\
	{ sname,	"Denominator1 TX",	"Denominator1 TX" },	\
	{ sname,	"Denominator2 TX",	"Denominator2 TX" },	\
	{ sname,	"Denominator3 TX",	"Denominator3 TX" },	\
	{ sname,	"Denominator4 TX",	"Denominator4 TX" },	\
	{ sname,	"Denominator5 TX",	"Denominator5 TX" },	\
	{ sname,	"Denominator6 TX",	"Denominator6 TX" }

static const struct snd_soc_dapm_route tegra186_arad_routes[] = {
	RATIO_GEN("Numerator1"),
	RATIO_GEN("Numerator2"),
	RATIO_GEN("Numerator3"),
	RATIO_GEN("Numerator4"),
	RATIO_GEN("Numerator5"),
	RATIO_GEN("Numerator6"),
	RATIO_GEN("Denominator1"),
	RATIO_GEN("Denominator2"),
	RATIO_GEN("Denominator3"),
	RATIO_GEN("Denominator4"),
	RATIO_GEN("Denominator5"),
	RATIO_GEN("Denominator6"),

	PACKETIZER_ROUTE("Packetizer"),

	{ "ARAD Transmit", NULL, "Packetizer" },
};

static const struct snd_kcontrol_new tegra186_arad_controls[] = {
	SOC_SINGLE_EXT("Lane1 enable", TEGRA186_ARAD_LANE_ENABLE, 0, 1, 0,
		tegra186_arad_get_enable_lane, tegra186_arad_put_enable_lane),
	SOC_SINGLE_EXT("Lane2 enable", TEGRA186_ARAD_LANE_ENABLE, 1, 1, 0,
		tegra186_arad_get_enable_lane, tegra186_arad_put_enable_lane),
	SOC_SINGLE_EXT("Lane3 enable", TEGRA186_ARAD_LANE_ENABLE, 2, 1, 0,
		tegra186_arad_get_enable_lane, tegra186_arad_put_enable_lane),
	SOC_SINGLE_EXT("Lane4 enable", TEGRA186_ARAD_LANE_ENABLE, 3, 1, 0,
		tegra186_arad_get_enable_lane, tegra186_arad_put_enable_lane),
	SOC_SINGLE_EXT("Lane5 enable", TEGRA186_ARAD_LANE_ENABLE, 4, 1, 0,
		tegra186_arad_get_enable_lane, tegra186_arad_put_enable_lane),
	SOC_SINGLE_EXT("Lane6 enable", TEGRA186_ARAD_LANE_ENABLE, 5, 1, 0,
		tegra186_arad_get_enable_lane, tegra186_arad_put_enable_lane),

	SOC_SINGLE_EXT("Lane1 Ratio Int",
		TEGRA186_ARAD_LANE1_RATIO_INTEGER_PART,
		0, TEGRA186_ARAD_LANE_RATIO_INTEGER_PART_MASK, 0,
		tegra186_arad_get_ratio_int, NULL),
	SOC_SINGLE_EXT("Lane1 Ratio Frac",
		TEGRA186_ARAD_LANE1_RATIO_FRACTIONAL_PART,
		0, TEGRA186_ARAD_LANE_RATIO_FRAC_PART_MASK, 0,
		tegra186_arad_get_ratio_frac, NULL),
	SOC_SINGLE_EXT("Lane2 Ratio Int",
		TEGRA186_ARAD_LANE2_RATIO_INTEGER_PART,
		0, TEGRA186_ARAD_LANE_RATIO_INTEGER_PART_MASK, 0,
		tegra186_arad_get_ratio_int, NULL),
	SOC_SINGLE_EXT("Lane2 Ratio Frac",
		TEGRA186_ARAD_LANE2_RATIO_FRACTIONAL_PART,
		0, TEGRA186_ARAD_LANE_RATIO_FRAC_PART_MASK, 0,
		tegra186_arad_get_ratio_frac, NULL),
	SOC_SINGLE_EXT("Lane3 Ratio Int",
		TEGRA186_ARAD_LANE3_RATIO_INTEGER_PART,
		0, TEGRA186_ARAD_LANE_RATIO_INTEGER_PART_MASK, 0,
		tegra186_arad_get_ratio_int, NULL),
	SOC_SINGLE_EXT("Lane3 Ratio Frac",
		TEGRA186_ARAD_LANE3_RATIO_FRACTIONAL_PART,
		0, TEGRA186_ARAD_LANE_RATIO_FRAC_PART_MASK, 0,
		tegra186_arad_get_ratio_frac, NULL),
	SOC_SINGLE_EXT("Lane4 Ratio Int",
		TEGRA186_ARAD_LANE4_RATIO_INTEGER_PART,
		0, TEGRA186_ARAD_LANE_RATIO_INTEGER_PART_MASK, 0,
		tegra186_arad_get_ratio_int, NULL),
	SOC_SINGLE_EXT("Lane4 Ratio Frac",
		TEGRA186_ARAD_LANE4_RATIO_FRACTIONAL_PART,
		0, TEGRA186_ARAD_LANE_RATIO_FRAC_PART_MASK, 0,
		tegra186_arad_get_ratio_frac, NULL),
	SOC_SINGLE_EXT("Lane5 Ratio Int",
		TEGRA186_ARAD_LANE5_RATIO_INTEGER_PART,
		0, TEGRA186_ARAD_LANE_RATIO_INTEGER_PART_MASK, 0,
		tegra186_arad_get_ratio_int, NULL),
	SOC_SINGLE_EXT("Lane5 Ratio Frac",
		TEGRA186_ARAD_LANE5_RATIO_FRACTIONAL_PART,
		0, TEGRA186_ARAD_LANE_RATIO_FRAC_PART_MASK, 0,
		tegra186_arad_get_ratio_frac, NULL),
	SOC_SINGLE_EXT("Lane6 Ratio Int",
		TEGRA186_ARAD_LANE6_RATIO_INTEGER_PART,
		0, TEGRA186_ARAD_LANE_RATIO_INTEGER_PART_MASK, 0,
		tegra186_arad_get_ratio_int, NULL),
	SOC_SINGLE_EXT("Lane6 Ratio Frac",
		TEGRA186_ARAD_LANE6_RATIO_FRACTIONAL_PART,
		0, TEGRA186_ARAD_LANE_RATIO_FRAC_PART_MASK, 0,
		tegra186_arad_get_ratio_frac, NULL),
};

static struct snd_soc_codec_driver tegra186_arad_codec = {
	.probe = tegra186_arad_codec_probe,
	.dapm_widgets = tegra186_arad_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra186_arad_widgets),
	.dapm_routes = tegra186_arad_routes,
	.num_dapm_routes = ARRAY_SIZE(tegra186_arad_routes),
	.controls = tegra186_arad_controls,
	.num_controls = ARRAY_SIZE(tegra186_arad_controls),
	.idle_bias_off = 1,
};

static bool tegra186_arad_wr_reg(struct device *dev, unsigned int reg)
{
	if ((reg >= TEGRA186_ARAD_LANE_START) &&
		(reg <= TEGRA186_ARAD_LANE_LIMIT)) {
		reg -= TEGRA186_ARAD_LANE_START;
		reg %= TEGRA186_ARAD_LANE_STRIDE;
		reg += TEGRA186_ARAD_LANE_START;
	}

	switch (reg) {
	case TEGRA186_ARAD_LANE_ENABLE:
	case TEGRA186_ARAD_LANE_SOFT_RESET:
	case TEGRA186_ARAD_LANE_INT_MASK:
	case TEGRA186_ARAD_LANE_INT_SET:
	case TEGRA186_ARAD_LANE_INT_CLEAR:
	case TEGRA186_ARAD_GLOBAL_SOFT_RESET:
	case TEGRA186_ARAD_SEND_RATIO:
	case TEGRA186_ARAD_CG:
	case TEGRA186_ARAD_CYA_GLOBAL:

	case TEGRA186_ARAD_LANE1_NUMERATOR_MUX_SEL:
	case TEGRA186_ARAD_LANE1_NUMERATOR_PRESCALAR:
	case TEGRA186_ARAD_LANE1_DENOMINATOR_MUX_SEL:
	case TEGRA186_ARAD_LANE1_DENOMINATOR_PRESCALAR:
	case TEGRA186_ARAD_LANE1_SERVO_LOOP_CONFIG:
	case TEGRA186_ARAD_LANE1_LOCK_UNLOCK_DETECTOR_CONFIG:
	case TEGRA186_ARAD_LANE1_ERROR_LOCK_THRESHOLD:
	case TEGRA186_ARAD_LANE1_ERROR_UNLOCK_THRESHOLD:
	case TEGRA186_ARAD_LANE1_RATIO_CALCULATOR_CONFIG:
	case TEGRA186_ARAD_LANE1_CYA:
		return true;
	default:
		return false;
	};
}

static bool tegra186_arad_rd_reg(struct device *dev, unsigned int reg)
{
	if ((reg >= TEGRA186_ARAD_LANE_START) &&
		(reg <= TEGRA186_ARAD_LANE_LIMIT)) {
		reg -= TEGRA186_ARAD_LANE_START;
		reg %= TEGRA186_ARAD_LANE_STRIDE;
		reg += TEGRA186_ARAD_LANE_START;
	}

	switch (reg) {
	case TEGRA186_ARAD_LANE_ENABLE:
	case TEGRA186_ARAD_LANE_STATUS:
	case TEGRA186_ARAD_LANE_SOFT_RESET:
	case TEGRA186_ARAD_LANE_INT_STATUS:
	case TEGRA186_ARAD_LANE_INT_MASK:
	case TEGRA186_ARAD_LANE_INT_SET:
	case TEGRA186_ARAD_LANE_INT_CLEAR:
	case TEGRA186_ARAD_GLOBAL_SOFT_RESET:
	case TEGRA186_ARAD_CG:
	case TEGRA186_ARAD_STATUS:
	case TEGRA186_ARAD_CYA_GLOBAL:

	case TEGRA186_ARAD_LANE1_NUMERATOR_MUX_SEL:
	case TEGRA186_ARAD_LANE1_NUMERATOR_PRESCALAR:
	case TEGRA186_ARAD_LANE1_DENOMINATOR_MUX_SEL:
	case TEGRA186_ARAD_LANE1_DENOMINATOR_PRESCALAR:
	case TEGRA186_ARAD_LANE1_RATIO_INTEGER_PART:
	case TEGRA186_ARAD_LANE1_RATIO_FRACTIONAL_PART:
	case TEGRA186_ARAD_LANE1_PERIOD_COUNT:
	case TEGRA186_ARAD_LANE1_SERVO_LOOP_CONFIG:
	case TEGRA186_ARAD_LANE1_LOCK_UNLOCK_DETECTOR_CONFIG:
	case TEGRA186_ARAD_LANE1_ERROR_LOCK_THRESHOLD:
	case TEGRA186_ARAD_LANE1_ERROR_UNLOCK_THRESHOLD:
	case TEGRA186_ARAD_LANE1_RATIO_CALCULATOR_CONFIG:
	case TEGRA186_ARAD_LANE1_CYA:
		return true;
	default:
		return false;
	};
}

static bool tegra186_arad_volatile_reg(struct device *dev, unsigned int reg)
{
	if ((reg >= TEGRA186_ARAD_LANE_START) &&
		(reg <= TEGRA186_ARAD_LANE_LIMIT)) {
		reg -= TEGRA186_ARAD_LANE_START;
		reg %= TEGRA186_ARAD_LANE_STRIDE;
		reg += TEGRA186_ARAD_LANE_START;
	}

	switch (reg) {
	case TEGRA186_ARAD_LANE_STATUS:
	case TEGRA186_ARAD_LANE_INT_STATUS:
	case TEGRA186_ARAD_STATUS:

	case TEGRA186_ARAD_LANE1_RATIO_INTEGER_PART:
	case TEGRA186_ARAD_LANE1_RATIO_FRACTIONAL_PART:
	case TEGRA186_ARAD_LANE1_PERIOD_COUNT:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra186_arad_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA186_ARAD_TX_CIF_CTRL,
	.writeable_reg = tegra186_arad_wr_reg,
	.readable_reg = tegra186_arad_rd_reg,
	.volatile_reg = tegra186_arad_volatile_reg,
	.reg_defaults = tegra186_arad_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tegra186_arad_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static const struct tegra186_arad_soc_data soc_data_tegra186 = {
	.set_audio_cif = tegra210_xbar_set_cif,
};

static const struct of_device_id tegra186_arad_of_match[] = {
	{ .compatible = "nvidia,tegra186-arad", .data = &soc_data_tegra186 },
	{},
};

static int tegra186_arad_platform_probe(struct platform_device *pdev)
{
	struct tegra186_arad *arad;
	struct resource *mem, *memregion;
	void __iomem *regs;
	int ret = 0;
	const struct of_device_id *match;
	struct tegra186_arad_soc_data *soc_data;

	match = of_match_device(tegra186_arad_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		ret = -ENODEV;
		goto err;
	}
	soc_data = (struct tegra186_arad_soc_data *)match->data;

	arad = devm_kzalloc(&pdev->dev,
		sizeof(struct tegra186_arad), GFP_KERNEL);
	if (!arad) {
		dev_err(&pdev->dev, "Can't allocate tegra210_arad\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, arad);

	arad->soc_data = soc_data;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err;
	}

	memregion = devm_request_mem_region(&pdev->dev, mem->start,
					    resource_size(mem), DRV_NAME);
	if (!memregion) {
		dev_err(&pdev->dev, "Memory region already claimed\n");
		ret = -EBUSY;
		goto err;
	}

	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err;
	}

	arad->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &tegra186_arad_regmap_config);
	if (IS_ERR(arad->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(arad->regmap);
		goto err;
	}
	regcache_cache_only(arad->regmap, true);

	if (of_property_read_u32(pdev->dev.of_node,
				"nvidia,ahub-arad-id",
				&pdev->dev.id) < 0) {
		dev_err(&pdev->dev,
			"Missing property nvidia,ahub-arad-id\n");
		ret = -ENODEV;
		goto err;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra186_arad_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = snd_soc_register_codec(&pdev->dev, &tegra186_arad_codec,
				     tegra186_arad_dais,
				     ARRAY_SIZE(tegra186_arad_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra186_arad_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err:
	return ret;
}

static int tegra186_arad_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra186_arad_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops tegra186_arad_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra186_arad_runtime_suspend,
			   tegra186_arad_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(tegra186_arad_suspend, NULL)
};

static struct platform_driver tegra186_arad_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra186_arad_of_match,
		.pm = &tegra186_arad_pm_ops,
	},
	.probe = tegra186_arad_platform_probe,
	.remove = tegra186_arad_platform_remove,
};
module_platform_driver(tegra186_arad_driver)

MODULE_AUTHOR("Junghyun Kim <juskim@nvidia.com>");
MODULE_DESCRIPTION("Tegra186 ARAD ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra186_arad_of_match);
