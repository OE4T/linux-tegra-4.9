/*
 * tegra186_xbar_alt.c - Tegra186 XBAR driver
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
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <linux/clk/tegra.h>
#include <linux/version.h>
#include "tegra210_xbar_alt.h"
#include "tegra210_xbar_utils_alt.h"

#define DRV_NAME "2900800.ahub"

static struct snd_soc_dai_driver tegra186_xbar_dais[] = {
	DAI(ADMAIF1),
	DAI(ADMAIF2),
	DAI(ADMAIF3),
	DAI(ADMAIF4),
	DAI(ADMAIF5),
	DAI(ADMAIF6),
	DAI(ADMAIF7),
	DAI(ADMAIF8),
	DAI(ADMAIF9),
	DAI(ADMAIF10),
	DAI(I2S1),
	DAI(I2S2),
	DAI(I2S3),
	DAI(I2S4),
	DAI(I2S5),
	DAI(SFC1),
	DAI(SFC2),
	DAI(SFC3),
	DAI(SFC4),
	DAI(MIXER1-1),
	DAI(MIXER1-2),
	DAI(MIXER1-3),
	DAI(MIXER1-4),
	DAI(MIXER1-5),
	DAI(MIXER1-6),
	DAI(MIXER1-7),
	DAI(MIXER1-8),
	DAI(MIXER1-9),
	DAI(MIXER1-10),
	DAI(SPDIF1-1),
	DAI(SPDIF1-2),
	DAI(AFC1),
	DAI(AFC2),
	DAI(AFC3),
	DAI(AFC4),
	DAI(AFC5),
	DAI(AFC6),
	DAI(OPE1),
	DAI(SPKPROT1),
	DAI(MVC1),
	DAI(MVC2),
	DAI(IQC1-1),
	DAI(IQC1-2),
	DAI(IQC2-1),
	DAI(IQC2-2),
	DAI(DMIC1),
	DAI(DMIC2),
	DAI(DMIC3),
	DAI(AMX1),
	DAI(AMX1-1),
	DAI(AMX1-2),
	DAI(AMX1-3),
	DAI(AMX1-4),
	DAI(AMX2),
	DAI(AMX2-1),
	DAI(AMX2-2),
	DAI(AMX2-3),
	DAI(AMX2-4),
	DAI(ADX1-1),
	DAI(ADX1-2),
	DAI(ADX1-3),
	DAI(ADX1-4),
	DAI(ADX1),
	DAI(ADX2-1),
	DAI(ADX2-2),
	DAI(ADX2-3),
	DAI(ADX2-4),
	DAI(ADX2),
	DAI(ADMAIF11),
	DAI(ADMAIF12),
	DAI(ADMAIF13),
	DAI(ADMAIF14),
	DAI(ADMAIF15),
	DAI(ADMAIF16),
	DAI(ADMAIF17),
	DAI(ADMAIF18),
	DAI(ADMAIF19),
	DAI(ADMAIF20),
	DAI(I2S6),
	DAI(AMX3),
	DAI(AMX3-1),
	DAI(AMX3-2),
	DAI(AMX3-3),
	DAI(AMX3-4),
	DAI(AMX4),
	DAI(AMX4-1),
	DAI(AMX4-2),
	DAI(AMX4-3),
	DAI(AMX4-4),
	DAI(ADX3-1),
	DAI(ADX3-2),
	DAI(ADX3-3),
	DAI(ADX3-4),
	DAI(ADX3),
	DAI(ADX4-1),
	DAI(ADX4-2),
	DAI(ADX4-3),
	DAI(ADX4-4),
	DAI(ADX4),
	DAI(DMIC4),
	DAI(ASRC1-1),
	DAI(ASRC1-2),
	DAI(ASRC1-3),
	DAI(ASRC1-4),
	DAI(ASRC1-5),
	DAI(ASRC1-6),
	DAI(ASRC1-7),
	DAI(ARAD1),
	DAI(DSPK1),
	DAI(DSPK2),
};

static const char * const tegra186_xbar_mux_texts[] = {
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
	"ADMAIF11",
	"ADMAIF12",
	"ADMAIF13",
	"ADMAIF14",
	"ADMAIF15",
	"ADMAIF16",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
	"I2S6",
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
	"AMX3",
	"AMX4",
	"ARAD1",
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
	"DMIC4",
	"ADX1-1",
	"ADX1-2",
	"ADX1-3",
	"ADX1-4",
	"ADX2-1",
	"ADX2-2",
	"ADX2-3",
	"ADX2-4",
	/* index 35..53 above are inputs of PART2 Mux */
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
	/* index 54..71 above are inputs of PART3 Mux */
};

static const int tegra186_xbar_mux_values[] = {
	/* Mux0 input,	Mux1 input, Mux2 input */
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
	MUX_VALUE(0, 10),
	MUX_VALUE(0, 11),
	MUX_VALUE(0, 12),
	MUX_VALUE(0, 13),
	MUX_VALUE(0, 14),
	MUX_VALUE(0, 15),
	MUX_VALUE(0, 16),
	MUX_VALUE(0, 17),
	MUX_VALUE(0, 18),
	MUX_VALUE(0, 19),
	MUX_VALUE(0, 20),
	MUX_VALUE(0, 21),
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
	MUX_VALUE(1, 10),
	MUX_VALUE(1, 11),
	MUX_VALUE(1, 16),
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
	MUX_VALUE(2, 21),
	MUX_VALUE(2, 24),
	MUX_VALUE(2, 25),
	MUX_VALUE(2, 26),
	MUX_VALUE(2, 27),
	MUX_VALUE(2, 28),
	MUX_VALUE(2, 29),
	MUX_VALUE(2, 30),
	MUX_VALUE(2, 31),
	/* index 35..53 above are inputs of PART2 Mux */
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
};

MUX_ENUM_CTRL_DECL_186(admaif1_tx, 0x00);
MUX_ENUM_CTRL_DECL_186(admaif2_tx, 0x01);
MUX_ENUM_CTRL_DECL_186(admaif3_tx, 0x02);
MUX_ENUM_CTRL_DECL_186(admaif4_tx, 0x03);
MUX_ENUM_CTRL_DECL_186(admaif5_tx, 0x04);
MUX_ENUM_CTRL_DECL_186(admaif6_tx, 0x05);
MUX_ENUM_CTRL_DECL_186(admaif7_tx, 0x06);
MUX_ENUM_CTRL_DECL_186(admaif8_tx, 0x07);
MUX_ENUM_CTRL_DECL_186(admaif9_tx, 0x08);
MUX_ENUM_CTRL_DECL_186(admaif10_tx, 0x09);
MUX_ENUM_CTRL_DECL_186(i2s1_tx, 0x10);
MUX_ENUM_CTRL_DECL_186(i2s2_tx, 0x11);
MUX_ENUM_CTRL_DECL_186(i2s3_tx, 0x12);
MUX_ENUM_CTRL_DECL_186(i2s4_tx, 0x13);
MUX_ENUM_CTRL_DECL_186(i2s5_tx, 0x14);
MUX_ENUM_CTRL_DECL_186(sfc1_tx, 0x18);
MUX_ENUM_CTRL_DECL_186(sfc2_tx, 0x19);
MUX_ENUM_CTRL_DECL_186(sfc3_tx, 0x1a);
MUX_ENUM_CTRL_DECL_186(sfc4_tx, 0x1b);
MUX_ENUM_CTRL_DECL_186(mixer11_tx, 0x20);
MUX_ENUM_CTRL_DECL_186(mixer12_tx, 0x21);
MUX_ENUM_CTRL_DECL_186(mixer13_tx, 0x22);
MUX_ENUM_CTRL_DECL_186(mixer14_tx, 0x23);
MUX_ENUM_CTRL_DECL_186(mixer15_tx, 0x24);
MUX_ENUM_CTRL_DECL_186(mixer16_tx, 0x25);
MUX_ENUM_CTRL_DECL_186(mixer17_tx, 0x26);
MUX_ENUM_CTRL_DECL_186(mixer18_tx, 0x27);
MUX_ENUM_CTRL_DECL_186(mixer19_tx, 0x28);
MUX_ENUM_CTRL_DECL_186(mixer110_tx, 0x29);
MUX_ENUM_CTRL_DECL_186(spdif11_tx, 0x34);
MUX_ENUM_CTRL_DECL_186(spdif12_tx, 0x35);
MUX_ENUM_CTRL_DECL_186(afc1_tx, 0x38);
MUX_ENUM_CTRL_DECL_186(afc2_tx, 0x39);
MUX_ENUM_CTRL_DECL_186(afc3_tx, 0x3a);
MUX_ENUM_CTRL_DECL_186(afc4_tx, 0x3b);
MUX_ENUM_CTRL_DECL_186(afc5_tx, 0x3c);
MUX_ENUM_CTRL_DECL_186(afc6_tx, 0x3d);
MUX_ENUM_CTRL_DECL_186(ope1_tx, 0x40);
MUX_ENUM_CTRL_DECL_186(spkprot_tx, 0x44);
MUX_ENUM_CTRL_DECL_186(mvc1_tx, 0x48);
MUX_ENUM_CTRL_DECL_186(mvc2_tx, 0x49);
MUX_ENUM_CTRL_DECL_186(amx11_tx, 0x50);
MUX_ENUM_CTRL_DECL_186(amx12_tx, 0x51);
MUX_ENUM_CTRL_DECL_186(amx13_tx, 0x52);
MUX_ENUM_CTRL_DECL_186(amx14_tx, 0x53);
MUX_ENUM_CTRL_DECL_186(amx21_tx, 0x54);
MUX_ENUM_CTRL_DECL_186(amx22_tx, 0x55);
MUX_ENUM_CTRL_DECL_186(amx23_tx, 0x56);
MUX_ENUM_CTRL_DECL_186(amx24_tx, 0x57);
MUX_ENUM_CTRL_DECL_186(adx1_tx, 0x60);
MUX_ENUM_CTRL_DECL_186(adx2_tx, 0x61);
MUX_ENUM_CTRL_DECL_186(dspk1_tx, 0x30);
MUX_ENUM_CTRL_DECL_186(dspk2_tx, 0x31);
MUX_ENUM_CTRL_DECL_186(amx31_tx, 0x58);
MUX_ENUM_CTRL_DECL_186(amx32_tx, 0x59);
MUX_ENUM_CTRL_DECL_186(amx33_tx, 0x5a);
MUX_ENUM_CTRL_DECL_186(amx34_tx, 0x5b);
MUX_ENUM_CTRL_DECL_186(amx41_tx, 0x64);
MUX_ENUM_CTRL_DECL_186(amx42_tx, 0x65);
MUX_ENUM_CTRL_DECL_186(amx43_tx, 0x66);
MUX_ENUM_CTRL_DECL_186(amx44_tx, 0x67);
MUX_ENUM_CTRL_DECL_186(admaif11_tx, 0x0a);
MUX_ENUM_CTRL_DECL_186(admaif12_tx, 0x0b);
MUX_ENUM_CTRL_DECL_186(admaif13_tx, 0x0c);
MUX_ENUM_CTRL_DECL_186(admaif14_tx, 0x0d);
MUX_ENUM_CTRL_DECL_186(admaif15_tx, 0x0e);
MUX_ENUM_CTRL_DECL_186(admaif16_tx, 0x0f);
MUX_ENUM_CTRL_DECL_186(i2s6_tx, 0x15);
MUX_ENUM_CTRL_DECL_186(adx3_tx, 0x62);
MUX_ENUM_CTRL_DECL_186(adx4_tx, 0x63);
MUX_ENUM_CTRL_DECL_186(admaif17_tx, 0x68);
MUX_ENUM_CTRL_DECL_186(admaif18_tx, 0x69);
MUX_ENUM_CTRL_DECL_186(admaif19_tx, 0x6a);
MUX_ENUM_CTRL_DECL_186(admaif20_tx, 0x6b);
MUX_ENUM_CTRL_DECL_186(asrc11_tx, 0x6c);
MUX_ENUM_CTRL_DECL_186(asrc12_tx, 0x6d);
MUX_ENUM_CTRL_DECL_186(asrc13_tx, 0x6e);
MUX_ENUM_CTRL_DECL_186(asrc14_tx, 0x6f);
MUX_ENUM_CTRL_DECL_186(asrc15_tx, 0x70);
MUX_ENUM_CTRL_DECL_186(asrc16_tx, 0x71);
MUX_ENUM_CTRL_DECL_186(asrc17_tx, 0x72);

static const struct snd_soc_dapm_widget tegra186_xbar_widgets[] = {
	WIDGETS("ADMAIF1", admaif1_tx),
	WIDGETS("ADMAIF2", admaif2_tx),
	WIDGETS("ADMAIF3", admaif3_tx),
	WIDGETS("ADMAIF4", admaif4_tx),
	WIDGETS("ADMAIF5", admaif5_tx),
	WIDGETS("ADMAIF6", admaif6_tx),
	WIDGETS("ADMAIF7", admaif7_tx),
	WIDGETS("ADMAIF8", admaif8_tx),
	WIDGETS("ADMAIF9", admaif9_tx),
	WIDGETS("ADMAIF10", admaif10_tx),
	WIDGETS("I2S1", i2s1_tx),
	WIDGETS("I2S2", i2s2_tx),
	WIDGETS("I2S3", i2s3_tx),
	WIDGETS("I2S4", i2s4_tx),
	WIDGETS("I2S5", i2s5_tx),
	WIDGETS("SFC1", sfc1_tx),
	WIDGETS("SFC2", sfc2_tx),
	WIDGETS("SFC3", sfc3_tx),
	WIDGETS("SFC4", sfc4_tx),
	WIDGETS("MIXER1-1", mixer11_tx),
	WIDGETS("MIXER1-2", mixer12_tx),
	WIDGETS("MIXER1-3", mixer13_tx),
	WIDGETS("MIXER1-4", mixer14_tx),
	WIDGETS("MIXER1-5", mixer15_tx),
	WIDGETS("MIXER1-6", mixer16_tx),
	WIDGETS("MIXER1-7", mixer17_tx),
	WIDGETS("MIXER1-8", mixer18_tx),
	WIDGETS("MIXER1-9", mixer19_tx),
	WIDGETS("MIXER1-10", mixer110_tx),
	WIDGETS("SPDIF1-1", spdif11_tx),
	WIDGETS("SPDIF1-2", spdif12_tx),
	WIDGETS("AFC1", afc1_tx),
	WIDGETS("AFC2", afc2_tx),
	WIDGETS("AFC3", afc3_tx),
	WIDGETS("AFC4", afc4_tx),
	WIDGETS("AFC5", afc5_tx),
	WIDGETS("AFC6", afc6_tx),
	WIDGETS("OPE1", ope1_tx),
	WIDGETS("SPKPROT1", spkprot_tx),
	WIDGETS("MVC1", mvc1_tx),
	WIDGETS("MVC2", mvc2_tx),
	WIDGETS("AMX1-1", amx11_tx),
	WIDGETS("AMX1-2", amx12_tx),
	WIDGETS("AMX1-3", amx13_tx),
	WIDGETS("AMX1-4", amx14_tx),
	WIDGETS("AMX2-1", amx21_tx),
	WIDGETS("AMX2-2", amx22_tx),
	WIDGETS("AMX2-3", amx23_tx),
	WIDGETS("AMX2-4", amx24_tx),
	WIDGETS("ADX1", adx1_tx),
	WIDGETS("ADX2", adx2_tx),
	TX_WIDGETS("IQC1-1"),
	TX_WIDGETS("IQC1-2"),
	TX_WIDGETS("IQC2-1"),
	TX_WIDGETS("IQC2-2"),
	TX_WIDGETS("DMIC1"),
	TX_WIDGETS("DMIC2"),
	TX_WIDGETS("DMIC3"),
	TX_WIDGETS("AMX1"),
	TX_WIDGETS("ADX1-1"),
	TX_WIDGETS("ADX1-2"),
	TX_WIDGETS("ADX1-3"),
	TX_WIDGETS("ADX1-4"),
	TX_WIDGETS("AMX2"),
	TX_WIDGETS("ADX2-1"),
	TX_WIDGETS("ADX2-2"),
	TX_WIDGETS("ADX2-3"),
	TX_WIDGETS("ADX2-4"),
	WIDGETS("ADMAIF11", admaif11_tx),
	WIDGETS("ADMAIF12", admaif12_tx),
	WIDGETS("ADMAIF13", admaif13_tx),
	WIDGETS("ADMAIF14", admaif14_tx),
	WIDGETS("ADMAIF15", admaif15_tx),
	WIDGETS("ADMAIF16", admaif16_tx),
	WIDGETS("ADMAIF17", admaif17_tx),
	WIDGETS("ADMAIF18", admaif18_tx),
	WIDGETS("ADMAIF19", admaif19_tx),
	WIDGETS("ADMAIF20", admaif20_tx),
	WIDGETS("I2S6", i2s6_tx),
	WIDGETS("AMX3-1", amx31_tx),
	WIDGETS("AMX3-2", amx32_tx),
	WIDGETS("AMX3-3", amx33_tx),
	WIDGETS("AMX3-4", amx34_tx),
	WIDGETS("AMX4-1", amx41_tx),
	WIDGETS("AMX4-2", amx42_tx),
	WIDGETS("AMX4-3", amx43_tx),
	WIDGETS("AMX4-4", amx44_tx),
	WIDGETS("ADX3", adx3_tx),
	WIDGETS("ADX4", adx4_tx),
	WIDGETS("ASRC1-1", asrc11_tx),
	WIDGETS("ASRC1-2", asrc12_tx),
	WIDGETS("ASRC1-3", asrc13_tx),
	WIDGETS("ASRC1-4", asrc14_tx),
	WIDGETS("ASRC1-5", asrc15_tx),
	WIDGETS("ASRC1-6", asrc16_tx),
	WIDGETS("ASRC1-7", asrc17_tx),
	WIDGETS("DSPK1", dspk1_tx),
	WIDGETS("DSPK2", dspk2_tx),
	TX_WIDGETS("AMX3"),
	TX_WIDGETS("ADX3-1"),
	TX_WIDGETS("ADX3-2"),
	TX_WIDGETS("ADX3-3"),
	TX_WIDGETS("ADX3-4"),
	TX_WIDGETS("AMX4"),
	TX_WIDGETS("ADX4-1"),
	TX_WIDGETS("ADX4-2"),
	TX_WIDGETS("ADX4-3"),
	TX_WIDGETS("ADX4-4"),
	TX_WIDGETS("DMIC4"),
	TX_WIDGETS("ARAD1"),
};

#define TEGRA210_ROUTES(name)					\
	{ name " RX",       NULL,		name " Receive"},	\
	{ name " Transmit", NULL,		name " TX"},		\
	{ name " TX",       NULL,		name " Mux" },		\
	{ name " Mux",      "ADMAIF1",		"ADMAIF1 RX" },		\
	{ name " Mux",      "ADMAIF2",		"ADMAIF2 RX" },		\
	{ name " Mux",      "ADMAIF3",		"ADMAIF3 RX" },		\
	{ name " Mux",      "ADMAIF4",		"ADMAIF4 RX" },		\
	{ name " Mux",      "ADMAIF5",		"ADMAIF5 RX" },		\
	{ name " Mux",      "ADMAIF6",		"ADMAIF6 RX" },		\
	{ name " Mux",      "ADMAIF7",		"ADMAIF7 RX" },		\
	{ name " Mux",      "ADMAIF8",		"ADMAIF8 RX" },		\
	{ name " Mux",      "ADMAIF9",		"ADMAIF9 RX" },		\
	{ name " Mux",      "ADMAIF10",		"ADMAIF10 RX" },	\
	{ name " Mux",      "I2S1",		"I2S1 RX" },		\
	{ name " Mux",      "I2S2",		"I2S2 RX" },		\
	{ name " Mux",      "I2S3",		"I2S3 RX" },		\
	{ name " Mux",      "I2S4",		"I2S4 RX" },		\
	{ name " Mux",      "I2S5",		"I2S5 RX" },		\
	{ name " Mux",      "SFC1",		"SFC1 RX" },		\
	{ name " Mux",      "SFC2",		"SFC2 RX" },		\
	{ name " Mux",      "SFC3",		"SFC3 RX" },		\
	{ name " Mux",      "SFC4",		"SFC4 RX" },		\
	{ name " Mux",      "MIXER1-1",		"MIXER1-1 RX" },	\
	{ name " Mux",      "MIXER1-2",		"MIXER1-2 RX" },	\
	{ name " Mux",      "MIXER1-3",		"MIXER1-3 RX" },	\
	{ name " Mux",      "MIXER1-4",		"MIXER1-4 RX" },	\
	{ name " Mux",      "MIXER1-5",		"MIXER1-5 RX" },	\
	{ name " Mux",      "SPDIF1-1",		"SPDIF1-1 RX" },	\
	{ name " Mux",      "SPDIF1-2",		"SPDIF1-2 RX" },	\
	{ name " Mux",      "AFC1",		"AFC1 RX" },		\
	{ name " Mux",      "AFC2",		"AFC2 RX" },		\
	{ name " Mux",      "AFC3",		"AFC3 RX" },		\
	{ name " Mux",      "AFC4",		"AFC4 RX" },		\
	{ name " Mux",      "AFC5",		"AFC5 RX" },		\
	{ name " Mux",      "AFC6",		"AFC6 RX" },		\
	{ name " Mux",      "OPE1",		"OPE1 RX" },		\
	{ name " Mux",      "MVC1",		"MVC1 RX" },		\
	{ name " Mux",      "MVC2",		"MVC2 RX" },		\
	{ name " Mux",      "IQC1-1",		"IQC1-1 RX" },		\
	{ name " Mux",      "IQC1-2",		"IQC1-2 RX" },		\
	{ name " Mux",      "IQC2-1",		"IQC2-1 RX" },		\
	{ name " Mux",      "IQC2-2",		"IQC2-2 RX" },		\
	{ name " Mux",      "DMIC1",		"DMIC1 RX" },		\
	{ name " Mux",      "DMIC2",		"DMIC2 RX" },		\
	{ name " Mux",      "DMIC3",		"DMIC3 RX" },		\
	{ name " Mux",      "AMX1",		"AMX1 RX" },		\
	{ name " Mux",      "ADX1-1",		"ADX1-1 RX" },		\
	{ name " Mux",      "ADX1-2",		"ADX1-2 RX" },		\
	{ name " Mux",      "ADX1-3",		"ADX1-3 RX" },		\
	{ name " Mux",      "ADX1-4",		"ADX1-4 RX" },		\
	{ name " Mux",      "AMX2",		"AMX2 RX" },		\
	{ name " Mux",      "ADX2-1",		"ADX2-1 RX" },		\
	{ name " Mux",      "ADX2-2",		"ADX2-2 RX" },		\
	{ name " Mux",      "ADX2-3",		"ADX2-3 RX" },		\
	{ name " Mux",      "ADX2-4",		"ADX2-4 RX" },

#define TEGRA186_ROUTES(name)					\
	{ name " Mux",      "ADMAIF11",		"ADMAIF11 RX" },	\
	{ name " Mux",      "ADMAIF12",		"ADMAIF12 RX" },	\
	{ name " Mux",      "ADMAIF13",		"ADMAIF13 RX" },	\
	{ name " Mux",      "ADMAIF14",		"ADMAIF14 RX" },	\
	{ name " Mux",      "ADMAIF15",		"ADMAIF15 RX" },	\
	{ name " Mux",      "ADMAIF16",		"ADMAIF16 RX" },	\
	{ name " Mux",      "ADMAIF17",		"ADMAIF17 RX" },	\
	{ name " Mux",      "ADMAIF18",		"ADMAIF18 RX" },	\
	{ name " Mux",      "ADMAIF19",		"ADMAIF19 RX" },	\
	{ name " Mux",      "ADMAIF20",		"ADMAIF20 RX" },	\
	{ name " Mux",      "DMIC4",		"DMIC4 RX" },		\
	{ name " Mux",      "I2S6",		"I2S6 RX" },	\
	{ name " Mux",      "ASRC1-1",		"ASRC1-1 RX" },	\
	{ name " Mux",      "ASRC1-2",		"ASRC1-2 RX" },	\
	{ name " Mux",      "ASRC1-3",		"ASRC1-3 RX" },	\
	{ name " Mux",      "ASRC1-4",		"ASRC1-4 RX" },	\
	{ name " Mux",      "ASRC1-5",		"ASRC1-5 RX" },	\
	{ name " Mux",      "ASRC1-6",		"ASRC1-6 RX" },	\
	{ name " Mux",      "AMX3",		"AMX3 RX" },		\
	{ name " Mux",      "ADX3-1",		"ADX3-1 RX" },		\
	{ name " Mux",      "ADX3-2",		"ADX3-2 RX" },		\
	{ name " Mux",      "ADX3-3",		"ADX3-3 RX" },		\
	{ name " Mux",      "ADX3-4",		"ADX3-4 RX" },		\
	{ name " Mux",      "AMX4",		"AMX4 RX" },		\
	{ name " Mux",      "ADX4-1",		"ADX4-1 RX" },		\
	{ name " Mux",      "ADX4-2",		"ADX4-2 RX" },		\
	{ name " Mux",      "ADX4-3",		"ADX4-3 RX" },		\
	{ name " Mux",      "ADX4-4",		"ADX4-4 RX" },		\
	{ name " Mux",      "ARAD1",		"ARAD1 RX" },

static const struct snd_soc_dapm_route tegra186_xbar_routes[] = {
	TEGRA210_ROUTES("ADMAIF1")
	TEGRA210_ROUTES("ADMAIF2")
	TEGRA210_ROUTES("ADMAIF3")
	TEGRA210_ROUTES("ADMAIF4")
	TEGRA210_ROUTES("ADMAIF5")
	TEGRA210_ROUTES("ADMAIF6")
	TEGRA210_ROUTES("ADMAIF7")
	TEGRA210_ROUTES("ADMAIF8")
	TEGRA210_ROUTES("ADMAIF9")
	TEGRA210_ROUTES("ADMAIF10")
	TEGRA210_ROUTES("I2S1")
	TEGRA210_ROUTES("I2S2")
	TEGRA210_ROUTES("I2S3")
	TEGRA210_ROUTES("I2S4")
	TEGRA210_ROUTES("I2S5")
	TEGRA210_ROUTES("SFC1")
	TEGRA210_ROUTES("SFC2")
	TEGRA210_ROUTES("SFC3")
	TEGRA210_ROUTES("SFC4")
	TEGRA210_ROUTES("MIXER1-1")
	TEGRA210_ROUTES("MIXER1-2")
	TEGRA210_ROUTES("MIXER1-3")
	TEGRA210_ROUTES("MIXER1-4")
	TEGRA210_ROUTES("MIXER1-5")
	TEGRA210_ROUTES("MIXER1-6")
	TEGRA210_ROUTES("MIXER1-7")
	TEGRA210_ROUTES("MIXER1-8")
	TEGRA210_ROUTES("MIXER1-9")
	TEGRA210_ROUTES("MIXER1-10")
	TEGRA210_ROUTES("SPDIF1-1")
	TEGRA210_ROUTES("SPDIF1-2")
	TEGRA210_ROUTES("AFC1")
	TEGRA210_ROUTES("AFC2")
	TEGRA210_ROUTES("AFC3")
	TEGRA210_ROUTES("AFC4")
	TEGRA210_ROUTES("AFC5")
	TEGRA210_ROUTES("AFC6")
	TEGRA210_ROUTES("OPE1")
	TEGRA210_ROUTES("SPKPROT1")
	TEGRA210_ROUTES("MVC1")
	TEGRA210_ROUTES("MVC2")
	TEGRA210_ROUTES("AMX1-1")
	TEGRA210_ROUTES("AMX1-2")
	TEGRA210_ROUTES("AMX1-3")
	TEGRA210_ROUTES("AMX1-4")
	TEGRA210_ROUTES("AMX2-1")
	TEGRA210_ROUTES("AMX2-2")
	TEGRA210_ROUTES("AMX2-3")
	TEGRA210_ROUTES("AMX2-4")
	TEGRA210_ROUTES("ADX1")
	TEGRA210_ROUTES("ADX2")
	IN_OUT_ROUTES("IQC1-1")
	IN_OUT_ROUTES("IQC1-2")
	IN_OUT_ROUTES("IQC2-1")
	IN_OUT_ROUTES("IQC2-1")
	IN_OUT_ROUTES("DMIC1")
	IN_OUT_ROUTES("DMIC2")
	IN_OUT_ROUTES("DMIC3")
	IN_OUT_ROUTES("AMX1")
	IN_OUT_ROUTES("AMX2")
	IN_OUT_ROUTES("ADX1-1")
	IN_OUT_ROUTES("ADX1-2")
	IN_OUT_ROUTES("ADX1-3")
	IN_OUT_ROUTES("ADX1-4")
	IN_OUT_ROUTES("ADX2-1")
	IN_OUT_ROUTES("ADX2-2")
	IN_OUT_ROUTES("ADX2-3")
	IN_OUT_ROUTES("ADX2-4")
	TEGRA210_ROUTES("ADMAIF11")
	TEGRA210_ROUTES("ADMAIF12")
	TEGRA210_ROUTES("ADMAIF13")
	TEGRA210_ROUTES("ADMAIF14")
	TEGRA210_ROUTES("ADMAIF15")
	TEGRA210_ROUTES("ADMAIF16")
	TEGRA210_ROUTES("ADMAIF17")
	TEGRA210_ROUTES("ADMAIF18")
	TEGRA210_ROUTES("ADMAIF19")
	TEGRA210_ROUTES("ADMAIF20")
	TEGRA210_ROUTES("AMX3-1")
	TEGRA210_ROUTES("AMX3-2")
	TEGRA210_ROUTES("AMX3-3")
	TEGRA210_ROUTES("AMX3-4")
	TEGRA210_ROUTES("AMX4-1")
	TEGRA210_ROUTES("AMX4-2")
	TEGRA210_ROUTES("AMX4-3")
	TEGRA210_ROUTES("AMX4-4")
	TEGRA210_ROUTES("ADX3")
	TEGRA210_ROUTES("ADX4")
	TEGRA210_ROUTES("I2S6")
	TEGRA210_ROUTES("ASRC1-1")
	TEGRA210_ROUTES("ASRC1-2")
	TEGRA210_ROUTES("ASRC1-3")
	TEGRA210_ROUTES("ASRC1-4")
	TEGRA210_ROUTES("ASRC1-5")
	TEGRA210_ROUTES("ASRC1-6")
	TEGRA210_ROUTES("ASRC1-7")
	TEGRA210_ROUTES("DSPK1")
	TEGRA210_ROUTES("DSPK2")
	TEGRA186_ROUTES("ADMAIF1")
	TEGRA186_ROUTES("ADMAIF2")
	TEGRA186_ROUTES("ADMAIF3")
	TEGRA186_ROUTES("ADMAIF4")
	TEGRA186_ROUTES("ADMAIF5")
	TEGRA186_ROUTES("ADMAIF6")
	TEGRA186_ROUTES("ADMAIF7")
	TEGRA186_ROUTES("ADMAIF8")
	TEGRA186_ROUTES("ADMAIF9")
	TEGRA186_ROUTES("ADMAIF10")
	TEGRA186_ROUTES("I2S1")
	TEGRA186_ROUTES("I2S2")
	TEGRA186_ROUTES("I2S3")
	TEGRA186_ROUTES("I2S4")
	TEGRA186_ROUTES("I2S5")
	TEGRA186_ROUTES("SFC1")
	TEGRA186_ROUTES("SFC2")
	TEGRA186_ROUTES("SFC3")
	TEGRA186_ROUTES("SFC4")
	TEGRA186_ROUTES("MIXER1-1")
	TEGRA186_ROUTES("MIXER1-2")
	TEGRA186_ROUTES("MIXER1-3")
	TEGRA186_ROUTES("MIXER1-4")
	TEGRA186_ROUTES("MIXER1-5")
	TEGRA186_ROUTES("MIXER1-6")
	TEGRA186_ROUTES("MIXER1-7")
	TEGRA186_ROUTES("MIXER1-8")
	TEGRA186_ROUTES("MIXER1-9")
	TEGRA186_ROUTES("MIXER1-10")
	TEGRA186_ROUTES("SPDIF1-1")
	TEGRA186_ROUTES("SPDIF1-2")
	TEGRA186_ROUTES("AFC1")
	TEGRA186_ROUTES("AFC2")
	TEGRA186_ROUTES("AFC3")
	TEGRA186_ROUTES("AFC4")
	TEGRA186_ROUTES("AFC5")
	TEGRA186_ROUTES("AFC6")
	TEGRA186_ROUTES("OPE1")
	TEGRA186_ROUTES("SPKPROT1")
	TEGRA186_ROUTES("MVC1")
	TEGRA186_ROUTES("MVC2")
	TEGRA186_ROUTES("AMX1-1")
	TEGRA186_ROUTES("AMX1-2")
	TEGRA186_ROUTES("AMX1-3")
	TEGRA186_ROUTES("AMX1-4")
	TEGRA186_ROUTES("AMX2-1")
	TEGRA186_ROUTES("AMX2-2")
	TEGRA186_ROUTES("AMX2-3")
	TEGRA186_ROUTES("AMX2-4")
	TEGRA186_ROUTES("ADX1")
	TEGRA186_ROUTES("ADX2")
	TEGRA186_ROUTES("ADMAIF11")
	TEGRA186_ROUTES("ADMAIF12")
	TEGRA186_ROUTES("ADMAIF13")
	TEGRA186_ROUTES("ADMAIF14")
	TEGRA186_ROUTES("ADMAIF15")
	TEGRA186_ROUTES("ADMAIF16")
	TEGRA186_ROUTES("ADMAIF17")
	TEGRA186_ROUTES("ADMAIF18")
	TEGRA186_ROUTES("ADMAIF19")
	TEGRA186_ROUTES("ADMAIF20")
	TEGRA186_ROUTES("AMX3-1")
	TEGRA186_ROUTES("AMX3-2")
	TEGRA186_ROUTES("AMX3-3")
	TEGRA186_ROUTES("AMX3-4")
	TEGRA186_ROUTES("AMX4-1")
	TEGRA186_ROUTES("AMX4-2")
	TEGRA186_ROUTES("AMX4-3")
	TEGRA186_ROUTES("AMX4-4")
	TEGRA186_ROUTES("ADX3")
	TEGRA186_ROUTES("ADX4")
	TEGRA186_ROUTES("I2S6")
	TEGRA186_ROUTES("ASRC1-1")
	TEGRA186_ROUTES("ASRC1-2")
	TEGRA186_ROUTES("ASRC1-3")
	TEGRA186_ROUTES("ASRC1-4")
	TEGRA186_ROUTES("ASRC1-5")
	TEGRA186_ROUTES("ASRC1-6")
	TEGRA186_ROUTES("ASRC1-7")
	TEGRA186_ROUTES("DSPK1")
	TEGRA186_ROUTES("DSPK2")
	IN_OUT_ROUTES("DMIC4")
	IN_OUT_ROUTES("AMX3")
	IN_OUT_ROUTES("AMX4")
	IN_OUT_ROUTES("ADX3-1")
	IN_OUT_ROUTES("ADX3-2")
	IN_OUT_ROUTES("ADX3-3")
	IN_OUT_ROUTES("ADX3-4")
	IN_OUT_ROUTES("ADX4-1")
	IN_OUT_ROUTES("ADX4-2")
	IN_OUT_ROUTES("ADX4-3")
	IN_OUT_ROUTES("ADX4-4")
	IN_OUT_ROUTES("ARAD1")
};

static struct snd_soc_codec_driver tegra186_xbar_codec = {
	.probe = tegra_xbar_codec_probe,
	.idle_bias_off = 1,
	.component_driver = {
		.dapm_widgets = tegra186_xbar_widgets,
		.dapm_routes = tegra186_xbar_routes,
	},
};

static struct of_dev_auxdata tegra186_xbar_auxdata[] = {
	OF_DEV_AUXDATA("nvidia,tegra186-admaif", T186_ADMAIF_BASE_ADDR, "tegra186-admaif", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-i2s", T186_I2S1_BASE_ADDR, "tegra186-i2s.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-i2s", T186_I2S2_BASE_ADDR, "tegra186-i2s.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-i2s", T186_I2S3_BASE_ADDR, "tegra186-i2s.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-i2s", T186_I2S4_BASE_ADDR, "tegra186-i2s.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-i2s", T186_I2S5_BASE_ADDR, "tegra186-i2s.4", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-i2s", T186_I2S6_BASE_ADDR, "tegra186-i2s.5", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-amx", T186_AMX1_BASE_ADDR, "tegra210-amx.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-amx", T186_AMX2_BASE_ADDR, "tegra210-amx.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-amx", T186_AMX3_BASE_ADDR, "tegra210-amx.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-amx", T186_AMX4_BASE_ADDR, "tegra210-amx.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-adx", T186_ADX1_BASE_ADDR, "tegra210-adx.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-adx", T186_ADX2_BASE_ADDR, "tegra210-adx.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-adx", T186_ADX3_BASE_ADDR, "tegra210-adx.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-adx", T186_ADX4_BASE_ADDR, "tegra210-adx.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-afc", T186_AFC1_BASE_ADDR, "tegra186-afc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-afc", T186_AFC2_BASE_ADDR, "tegra186-afc.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-afc", T186_AFC3_BASE_ADDR, "tegra186-afc.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-afc", T186_AFC4_BASE_ADDR, "tegra186-afc.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-afc", T186_AFC5_BASE_ADDR, "tegra186-afc.4", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-afc", T186_AFC6_BASE_ADDR, "tegra186-afc.5", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", T186_SFC1_BASE_ADDR, "tegra210-sfc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", T186_SFC2_BASE_ADDR, "tegra210-sfc.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", T186_SFC3_BASE_ADDR, "tegra210-sfc.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sfc", T186_SFC4_BASE_ADDR, "tegra210-sfc.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-mvc", T186_MVC1_BASE_ADDR, "tegra210-mvc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-mvc", T186_MVC2_BASE_ADDR, "tegra210-mvc.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-iqc", T186_IQC1_BASE_ADDR, "tegra210-iqc.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-iqc", T186_IQC2_BASE_ADDR, "tegra210-iqc.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", T186_DMIC1_BASE_ADDR, "tegra210-dmic.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", T186_DMIC2_BASE_ADDR, "tegra210-dmic.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", T186_DMIC3_BASE_ADDR, "tegra210-dmic.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dmic", T186_DMIC4_BASE_ADDR, "tegra210-dmic.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-ope", T186_OPE1_BASE_ADDR, "tegra210-ope.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-amixer", T186_AMIXER1_BASE_ADDR, "tegra210-mixer", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spdif", T186_SPDIF1_BASE_ADDR, "tegra210-spdif", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-asrc", T186_ASRC1_BASE_ADDR, "tegra186-asrc", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-arad", T186_ARAD1_BASE_ADDR, "tegra186-arad", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-dspk", T186_DSPK1_BASE_ADDR, "tegra186-dspk.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra186-dspk", T186_DSPK2_BASE_ADDR, "tegra186-dspk.1", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 0, "spdif-dit.0", NULL),
	{}
};


int tegra186_xbar_registration(struct platform_device *pdev)
{
	int ret;
	int num_dapm_widgets, num_dapm_routes;

	num_dapm_widgets = (TEGRA186_NUM_MUX_WIDGETS * 3) +
				(TEGRA186_NUM_DAIS - TEGRA186_NUM_MUX_WIDGETS) * 2;
	num_dapm_routes =
		(TEGRA186_NUM_DAIS - TEGRA186_NUM_MUX_WIDGETS) * 2 +
		(TEGRA186_NUM_MUX_WIDGETS * TEGRA186_NUM_MUX_INPUT);

	tegra186_xbar_codec.component_driver.num_dapm_widgets =
			num_dapm_widgets;
	tegra186_xbar_codec.component_driver.num_dapm_routes =
			num_dapm_routes;

	ret = snd_soc_register_codec(&pdev->dev, &tegra186_xbar_codec,
				tegra186_xbar_dais, TEGRA186_NUM_DAIS);

	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		return -EBUSY;
	}

	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm_runtime_get failed. ret: %d\n", ret);
		return ret;
	}

	of_platform_populate(pdev->dev.of_node, NULL, tegra186_xbar_auxdata,
			     &pdev->dev);

	ret = pm_runtime_put_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm_runtime_put failed. ret: %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra186_xbar_registration);

static const struct regmap_config tegra186_xbar_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = TEGRA186_MAX_REGISTER_ADDR,
	.volatile_reg = tegra_xbar_volatile_reg,
	.cache_type = REGCACHE_FLAT,
};

static const struct tegra_xbar_soc_data soc_data_tegra186 = {
	.regmap_config = &tegra186_xbar_regmap_config,
	.mask[0] = TEGRA186_XBAR_REG_MASK_0,
	.mask[1] = TEGRA186_XBAR_REG_MASK_1,
	.mask[2] = TEGRA186_XBAR_REG_MASK_2,
	.mask[3] = TEGRA186_XBAR_REG_MASK_3,
	.reg_count = TEGRA186_XBAR_UPDATE_MAX_REG,
	.reg_offset = TEGRA210_XBAR_PART1_RX,
	.xbar_registration = tegra186_xbar_registration,
};



static const struct of_device_id tegra186_xbar_of_match[] = {
	{ .compatible = "nvidia,tegra186-axbar", .data = &soc_data_tegra186 },
	{},
};

int tegra186_xbar_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct tegra_xbar_soc_data *soc_data;
	int ret;

	match = of_match_device(tegra186_xbar_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		ret = -ENODEV;
		goto err;
	}
	soc_data = (struct tegra_xbar_soc_data *)match->data;

	ret = tegra_xbar_probe(pdev, soc_data);

err:
	return ret;
}

static const struct dev_pm_ops tegra186_xbar_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra_xbar_runtime_suspend,
			   tegra_xbar_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(tegra_xbar_suspend, NULL)
};

static struct platform_driver tegra186_xbar_driver = {
	.probe = tegra186_xbar_probe,
	.remove = tegra_xbar_remove,
	.shutdown = tegra_xbar_shutdown,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra186_xbar_of_match,
		.pm = &tegra186_xbar_pm_ops,
	},
};
module_platform_driver(tegra186_xbar_driver);

MODULE_AUTHOR("Mohan Kumar <mkumard@nvidia.com>");
MODULE_DESCRIPTION("Tegra186 XBAR driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
