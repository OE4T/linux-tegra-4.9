/*
 * drivers/video/tegra/dc/nvdisplay/nvdisp.c
 *
 * Copyright (c) 2014-2016, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/tegra_pm_domains.h>

#include <mach/dc.h>
#include <mach/fb.h>

#include "nvdisp.h"
#include "nvdisp_priv.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "dp.h"
#include "hw_nvdisp_nvdisp.h"
#include "hw_win_nvdisp.h"

DEFINE_MUTEX(tegra_nvdisp_lock);

#define NVDISP_INPUT_LUT_SIZE   257
#define NVDISP_OUTPUT_LUT_SIZE  1025

/* Global variables provided for clocks
 * common to all heads
 */
static struct clk *hubclk;
static struct clk *compclk;

static struct reset_control *nvdisp_common_rst[DC_N_WINDOWS+1];

static int tegra_nvdisp_set_color_control(struct tegra_dc *dc);

static struct of_device_id nvdisp_disa_pd[] = {
	{ .compatible = "nvidia,tegra186-disa-pd", },
	{},
};

static struct of_device_id nvdisp_disb_pd[] = {
	{ .compatible = "nvidia,tegra186-disb-pd", },
	{},
};

static struct of_device_id nvdisp_disc_pd[] = {
	{ .compatible = "nvidia,tegra186-disc-pd", },
	{},
};

static struct nvdisp_pd_info nvdisp_pg[NVDISP_PD_COUNT];
static struct nvdisp_compclk_client compclk_client[TEGRA_MAX_DC];

static unsigned int default_sRGB_OutLUT[] = {
	0x00006000,  0x000060ce,  0x0000619d,  0x0000626c,  0x0000632d,
	0x000063d4,  0x00006469,  0x000064f0,  0x0000656b,  0x000065df,
	0x0000664a,  0x000066b0,  0x00006711,  0x0000676d,  0x000067c4,
	0x00006819,  0x0000686a,  0x000068b8,  0x00006904,  0x0000694d,
	0x00006994,  0x000069d8,  0x00006a1b,  0x00006a5d,  0x00006a9c,
	0x00006ada,  0x00006b17,  0x00006b52,  0x00006b8c,  0x00006bc5,
	0x00006bfd,  0x00006c33,  0x00006c69,  0x00006c9e,  0x00006cd1,
	0x00006d04,  0x00006d36,  0x00006d67,  0x00006d98,  0x00006dc7,
	0x00006df6,  0x00006e25,  0x00006e52,  0x00006e7f,  0x00006eac,
	0x00006ed7,  0x00006f03,  0x00006f2d,  0x00006f58,  0x00006f81,
	0x00006faa,  0x00006fd3,  0x00006ffb,  0x00007023,  0x0000704b,
	0x00007071,  0x00007098,  0x000070be,  0x000070e4,  0x00007109,
	0x0000712e,  0x00007153,  0x00007177,  0x0000719b,  0x000071bf,
	0x000071e2,  0x00007205,  0x00007227,  0x0000724a,  0x0000726c,
	0x0000728e,  0x000072af,  0x000072d0,  0x000072f1,  0x00007312,
	0x00007333,  0x00007353,  0x00007373,  0x00007392,  0x000073b2,
	0x000073d1,  0x000073f0,  0x0000740f,  0x0000742d,  0x0000744c,
	0x0000746a,  0x00007488,  0x000074a6,  0x000074c3,  0x000074e0,
	0x000074fe,  0x0000751b,  0x00007537,  0x00007554,  0x00007570,
	0x0000758d,  0x000075a9,  0x000075c4,  0x000075e0,  0x000075fc,
	0x00007617,  0x00007632,  0x0000764d,  0x00007668,  0x00007683,
	0x0000769e,  0x000076b8,  0x000076d3,  0x000076ed,  0x00007707,
	0x00007721,  0x0000773b,  0x00007754,  0x0000776e,  0x00007787,
	0x000077a0,  0x000077b9,  0x000077d2,  0x000077eb,  0x00007804,
	0x0000781d,  0x00007835,  0x0000784e,  0x00007866,  0x0000787e,
	0x00007896,  0x000078ae,  0x000078c6,  0x000078dd,  0x000078f5,
	0x0000790d,  0x00007924,  0x0000793b,  0x00007952,  0x0000796a,
	0x00007981,  0x00007997,  0x000079ae,  0x000079c5,  0x000079db,
	0x000079f2,  0x00007a08,  0x00007a1f,  0x00007a35,  0x00007a4b,
	0x00007a61,  0x00007a77,  0x00007a8d,  0x00007aa3,  0x00007ab8,
	0x00007ace,  0x00007ae3,  0x00007af9,  0x00007b0e,  0x00007b24,
	0x00007b39,  0x00007b4e,  0x00007b63,  0x00007b78,  0x00007b8d,
	0x00007ba2,  0x00007bb6,  0x00007bcb,  0x00007be0,  0x00007bf4,
	0x00007c08,  0x00007c1d,  0x00007c31,  0x00007c45,  0x00007c59,
	0x00007c6e,  0x00007c82,  0x00007c96,  0x00007ca9,  0x00007cbd,
	0x00007cd1,  0x00007ce5,  0x00007cf8,  0x00007d0c,  0x00007d1f,
	0x00007d33,  0x00007d46,  0x00007d59,  0x00007d6d,  0x00007d80,
	0x00007d93,  0x00007da6,  0x00007db9,  0x00007dcc,  0x00007ddf,
	0x00007df2,  0x00007e04,  0x00007e17,  0x00007e2a,  0x00007e3c,
	0x00007e4f,  0x00007e61,  0x00007e74,  0x00007e86,  0x00007e98,
	0x00007eab,  0x00007ebd,  0x00007ecf,  0x00007ee1,  0x00007ef3,
	0x00007f05,  0x00007f17,  0x00007f29,  0x00007f3b,  0x00007f4d,
	0x00007f5e,  0x00007f70,  0x00007f82,  0x00007f93,  0x00007fa5,
	0x00007fb6,  0x00007fc8,  0x00007fd9,  0x00007feb,  0x00007ffc,
	0x0000800d,  0x0000801e,  0x00008030,  0x00008041,  0x00008052,
	0x00008063,  0x00008074,  0x00008085,  0x00008096,  0x000080a7,
	0x000080b7,  0x000080c8,  0x000080d9,  0x000080ea,  0x000080fa,
	0x0000810b,  0x0000811c,  0x0000812c,  0x0000813d,  0x0000814d,
	0x0000815d,  0x0000816e,  0x0000817e,  0x0000818e,  0x0000819f,
	0x000081af,  0x000081bf,  0x000081cf,  0x000081df,  0x000081ef,
	0x000081ff,  0x0000820f,  0x0000821f,  0x0000822f,  0x0000823f,
	0x0000824f,  0x0000825f,  0x0000826f,  0x0000827e,  0x0000828e,
	0x0000829e,  0x000082ad,  0x000082bd,  0x000082cc,  0x000082dc,
	0x000082eb,  0x000082fb,  0x0000830a,  0x0000831a,  0x00008329,
	0x00008338,  0x00008348,  0x00008357,  0x00008366,  0x00008375,
	0x00008385,  0x00008394,  0x000083a3,  0x000083b2,  0x000083c1,
	0x000083d0,  0x000083df,  0x000083ee,  0x000083fd,  0x0000840c,
	0x0000841a,  0x00008429,  0x00008438,  0x00008447,  0x00008455,
	0x00008464,  0x00008473,  0x00008481,  0x00008490,  0x0000849f,
	0x000084ad,  0x000084bc,  0x000084ca,  0x000084d9,  0x000084e7,
	0x000084f5,  0x00008504,  0x00008512,  0x00008521,  0x0000852f,
	0x0000853d,  0x0000854b,  0x0000855a,  0x00008568,  0x00008576,
	0x00008584,  0x00008592,  0x000085a0,  0x000085ae,  0x000085bc,
	0x000085ca,  0x000085d8,  0x000085e6,  0x000085f4,  0x00008602,
	0x00008610,  0x0000861e,  0x0000862c,  0x00008639,  0x00008647,
	0x00008655,  0x00008663,  0x00008670,  0x0000867e,  0x0000868c,
	0x00008699,  0x000086a7,  0x000086b5,  0x000086c2,  0x000086d0,
	0x000086dd,  0x000086eb,  0x000086f8,  0x00008705,  0x00008713,
	0x00008720,  0x0000872e,  0x0000873b,  0x00008748,  0x00008756,
	0x00008763,  0x00008770,  0x0000877d,  0x0000878b,  0x00008798,
	0x000087a5,  0x000087b2,  0x000087bf,  0x000087cc,  0x000087d9,
	0x000087e6,  0x000087f3,  0x00008801,  0x0000880e,  0x0000881a,
	0x00008827,  0x00008834,  0x00008841,  0x0000884e,  0x0000885b,
	0x00008868,  0x00008875,  0x00008882,  0x0000888e,  0x0000889b,
	0x000088a8,  0x000088b5,  0x000088c1,  0x000088ce,  0x000088db,
	0x000088e7,  0x000088f4,  0x00008900,  0x0000890d,  0x0000891a,
	0x00008926,  0x00008933,  0x0000893f,  0x0000894c,  0x00008958,
	0x00008965,  0x00008971,  0x0000897d,  0x0000898a,  0x00008996,
	0x000089a3,  0x000089af,  0x000089bb,  0x000089c8,  0x000089d4,
	0x000089e0,  0x000089ec,  0x000089f9,  0x00008a05,  0x00008a11,
	0x00008a1d,  0x00008a29,  0x00008a36,  0x00008a42,  0x00008a4e,
	0x00008a5a,  0x00008a66,  0x00008a72,  0x00008a7e,  0x00008a8a,
	0x00008a96,  0x00008aa2,  0x00008aae,  0x00008aba,  0x00008ac6,
	0x00008ad2,  0x00008ade,  0x00008aea,  0x00008af5,  0x00008b01,
	0x00008b0d,  0x00008b19,  0x00008b25,  0x00008b31,  0x00008b3c,
	0x00008b48,  0x00008b54,  0x00008b60,  0x00008b6b,  0x00008b77,
	0x00008b83,  0x00008b8e,  0x00008b9a,  0x00008ba6,  0x00008bb1,
	0x00008bbd,  0x00008bc8,  0x00008bd4,  0x00008bdf,  0x00008beb,
	0x00008bf6,  0x00008c02,  0x00008c0d,  0x00008c19,  0x00008c24,
	0x00008c30,  0x00008c3b,  0x00008c47,  0x00008c52,  0x00008c5d,
	0x00008c69,  0x00008c74,  0x00008c80,  0x00008c8b,  0x00008c96,
	0x00008ca1,  0x00008cad,  0x00008cb8,  0x00008cc3,  0x00008ccf,
	0x00008cda,  0x00008ce5,  0x00008cf0,  0x00008cfb,  0x00008d06,
	0x00008d12,  0x00008d1d,  0x00008d28,  0x00008d33,  0x00008d3e,
	0x00008d49,  0x00008d54,  0x00008d5f,  0x00008d6a,  0x00008d75,
	0x00008d80,  0x00008d8b,  0x00008d96,  0x00008da1,  0x00008dac,
	0x00008db7,  0x00008dc2,  0x00008dcd,  0x00008dd8,  0x00008de3,
	0x00008dee,  0x00008df9,  0x00008e04,  0x00008e0e,  0x00008e19,
	0x00008e24,  0x00008e2f,  0x00008e3a,  0x00008e44,  0x00008e4f,
	0x00008e5a,  0x00008e65,  0x00008e6f,  0x00008e7a,  0x00008e85,
	0x00008e90,  0x00008e9a,  0x00008ea5,  0x00008eb0,  0x00008eba,
	0x00008ec5,  0x00008ecf,  0x00008eda,  0x00008ee5,  0x00008eef,
	0x00008efa,  0x00008f04,  0x00008f0f,  0x00008f19,  0x00008f24,
	0x00008f2e,  0x00008f39,  0x00008f43,  0x00008f4e,  0x00008f58,
	0x00008f63,  0x00008f6d,  0x00008f78,  0x00008f82,  0x00008f8c,
	0x00008f97,  0x00008fa1,  0x00008fac,  0x00008fb6,  0x00008fc0,
	0x00008fcb,  0x00008fd5,  0x00008fdf,  0x00008fea,  0x00008ff4,
	0x00008ffe,  0x00009008,  0x00009013,  0x0000901d,  0x00009027,
	0x00009031,  0x0000903c,  0x00009046,  0x00009050,  0x0000905a,
	0x00009064,  0x0000906e,  0x00009079,  0x00009083,  0x0000908d,
	0x00009097,  0x000090a1,  0x000090ab,  0x000090b5,  0x000090bf,
	0x000090c9,  0x000090d3,  0x000090dd,  0x000090e7,  0x000090f1,
	0x000090fb,  0x00009105,  0x0000910f,  0x00009119,  0x00009123,
	0x0000912d,  0x00009137,  0x00009141,  0x0000914b,  0x00009155,
	0x0000915f,  0x00009169,  0x00009173,  0x0000917d,  0x00009186,
	0x00009190,  0x0000919a,  0x000091a4,  0x000091ae,  0x000091b8,
	0x000091c1,  0x000091cb,  0x000091d5,  0x000091df,  0x000091e9,
	0x000091f2,  0x000091fc,  0x00009206,  0x00009210,  0x00009219,
	0x00009223,  0x0000922d,  0x00009236,  0x00009240,  0x0000924a,
	0x00009253,  0x0000925d,  0x00009267,  0x00009270,  0x0000927a,
	0x00009283,  0x0000928d,  0x00009297,  0x000092a0,  0x000092aa,
	0x000092b3,  0x000092bd,  0x000092c6,  0x000092d0,  0x000092da,
	0x000092e3,  0x000092ed,  0x000092f6,  0x00009300,  0x00009309,
	0x00009313,  0x0000931c,  0x00009325,  0x0000932f,  0x00009338,
	0x00009342,  0x0000934b,  0x00009355,  0x0000935e,  0x00009367,
	0x00009371,  0x0000937a,  0x00009384,  0x0000938d,  0x00009396,
	0x000093a0,  0x000093a9,  0x000093b2,  0x000093bc,  0x000093c5,
	0x000093ce,  0x000093d7,  0x000093e1,  0x000093ea,  0x000093f3,
	0x000093fc,  0x00009406,  0x0000940f,  0x00009418,  0x00009421,
	0x0000942b,  0x00009434,  0x0000943d,  0x00009446,  0x0000944f,
	0x00009459,  0x00009462,  0x0000946b,  0x00009474,  0x0000947d,
	0x00009486,  0x0000948f,  0x00009499,  0x000094a2,  0x000094ab,
	0x000094b4,  0x000094bd,  0x000094c6,  0x000094cf,  0x000094d8,
	0x000094e1,  0x000094ea,  0x000094f3,  0x000094fc,  0x00009505,
	0x0000950e,  0x00009517,  0x00009520,  0x00009529,  0x00009532,
	0x0000953b,  0x00009544,  0x0000954d,  0x00009556,  0x0000955f,
	0x00009568,  0x00009571,  0x0000957a,  0x00009583,  0x0000958c,
	0x00009595,  0x0000959d,  0x000095a6,  0x000095af,  0x000095b8,
	0x000095c1,  0x000095ca,  0x000095d3,  0x000095db,  0x000095e4,
	0x000095ed,  0x000095f6,  0x000095ff,  0x00009608,  0x00009610,
	0x00009619,  0x00009622,  0x0000962b,  0x00009633,  0x0000963c,
	0x00009645,  0x0000964e,  0x00009656,  0x0000965f,  0x00009668,
	0x00009671,  0x00009679,  0x00009682,  0x0000968b,  0x00009693,
	0x0000969c,  0x000096a5,  0x000096ad,  0x000096b6,  0x000096bf,
	0x000096c7,  0x000096d0,  0x000096d9,  0x000096e1,  0x000096ea,
	0x000096f2,  0x000096fb,  0x00009704,  0x0000970c,  0x00009715,
	0x0000971d,  0x00009726,  0x0000972e,  0x00009737,  0x00009740,
	0x00009748,  0x00009751,  0x00009759,  0x00009762,  0x0000976a,
	0x00009773,  0x0000977b,  0x00009784,  0x0000978c,  0x00009795,
	0x0000979d,  0x000097a6,  0x000097ae,  0x000097b6,  0x000097bf,
	0x000097c7,  0x000097d0,  0x000097d8,  0x000097e1,  0x000097e9,
	0x000097f1,  0x000097fa,  0x00009802,  0x0000980b,  0x00009813,
	0x0000981b,  0x00009824,  0x0000982c,  0x00009834,  0x0000983d,
	0x00009845,  0x0000984d,  0x00009856,  0x0000985e,  0x00009866,
	0x0000986f,  0x00009877,  0x0000987f,  0x00009888,  0x00009890,
	0x00009898,  0x000098a0,  0x000098a9,  0x000098b1,  0x000098b9,
	0x000098c1,  0x000098ca,  0x000098d2,  0x000098da,  0x000098e2,
	0x000098eb,  0x000098f3,  0x000098fb,  0x00009903,  0x0000990b,
	0x00009914,  0x0000991c,  0x00009924,  0x0000992c,  0x00009934,
	0x0000993c,  0x00009945,  0x0000994d,  0x00009955,  0x0000995d,
	0x00009965,  0x0000996d,  0x00009975,  0x0000997d,  0x00009986,
	0x0000998e,  0x00009996,  0x0000999e,  0x000099a6,  0x000099ae,
	0x000099b6,  0x000099be,  0x000099c6,  0x000099ce,  0x000099d6,
	0x000099de,  0x000099e6,  0x000099ee,  0x000099f6,  0x000099fe,
	0x00009a06,  0x00009a0e,  0x00009a16,  0x00009a1e,  0x00009a26,
	0x00009a2e,  0x00009a36,  0x00009a3e,  0x00009a46,  0x00009a4e,
	0x00009a56,  0x00009a5e,  0x00009a66,  0x00009a6e,  0x00009a76,
	0x00009a7e,  0x00009a86,  0x00009a8e,  0x00009a96,  0x00009a9d,
	0x00009aa5,  0x00009aad,  0x00009ab5,  0x00009abd,  0x00009ac5,
	0x00009acd,  0x00009ad5,  0x00009adc,  0x00009ae4,  0x00009aec,
	0x00009af4,  0x00009afc,  0x00009b04,  0x00009b0c,  0x00009b13,
	0x00009b1b,  0x00009b23,  0x00009b2b,  0x00009b33,  0x00009b3a,
	0x00009b42,  0x00009b4a,  0x00009b52,  0x00009b59,  0x00009b61,
	0x00009b69,  0x00009b71,  0x00009b79,  0x00009b80,  0x00009b88,
	0x00009b90,  0x00009b97,  0x00009b9f,  0x00009ba7,  0x00009baf,
	0x00009bb6,  0x00009bbe,  0x00009bc6,  0x00009bcd,  0x00009bd5,
	0x00009bdd,  0x00009be5,  0x00009bec,  0x00009bf4,  0x00009bfc,
	0x00009c03,  0x00009c0b,  0x00009c12,  0x00009c1a,  0x00009c22,
	0x00009c29,  0x00009c31,  0x00009c39,  0x00009c40,  0x00009c48,
	0x00009c50,  0x00009c57,  0x00009c5f,  0x00009c66,  0x00009c6e,
	0x00009c75,  0x00009c7d,  0x00009c85,  0x00009c8c,  0x00009c94,
	0x00009c9b,  0x00009ca3,  0x00009caa,  0x00009cb2,  0x00009cba,
	0x00009cc1,  0x00009cc9,  0x00009cd0,  0x00009cd8,  0x00009cdf,
	0x00009ce7,  0x00009cee,  0x00009cf6,  0x00009cfd,  0x00009d05,
	0x00009d0c,  0x00009d14,  0x00009d1b,  0x00009d23,  0x00009d2a,
	0x00009d32,  0x00009d39,  0x00009d40,  0x00009d48,  0x00009d4f,
	0x00009d57,  0x00009d5e,  0x00009d66,  0x00009d6d,  0x00009d75,
	0x00009d7c,  0x00009d83,  0x00009d8b,  0x00009d92,  0x00009d9a,
	0x00009da1,  0x00009da8,  0x00009db0,  0x00009db7,  0x00009dbe,
	0x00009dc6,  0x00009dcd,  0x00009dd5,  0x00009ddc,  0x00009de3,
	0x00009deb,  0x00009df2,  0x00009df9,  0x00009e01,  0x00009e08,
	0x00009e0f,  0x00009e17,  0x00009e1e,  0x00009e25,  0x00009e2d,
	0x00009e34,  0x00009e3b,  0x00009e43,  0x00009e4a,  0x00009e51,
	0x00009e58,  0x00009e60,  0x00009e67,  0x00009e6e,  0x00009e75,
	0x00009e7d,  0x00009e84,  0x00009e8b,  0x00009e92,  0x00009e9a,
	0x00009ea1,  0x00009ea8,  0x00009eaf,  0x00009eb7,  0x00009ebe,
	0x00009ec5,  0x00009ecc,  0x00009ed4,  0x00009edb,  0x00009ee2,
	0x00009ee9,  0x00009ef0,  0x00009ef7,  0x00009eff,  0x00009f06,
	0x00009f0d,  0x00009f14,  0x00009f1b,  0x00009f23,  0x00009f2a,
	0x00009f31,  0x00009f38,  0x00009f3f,  0x00009f46,  0x00009f4d,
	0x00009f55,  0x00009f5c,  0x00009f63,  0x00009f6a,  0x00009f71,
	0x00009f78,  0x00009f7f,  0x00009f86,  0x00009f8d,  0x00009f95,
	0x00009f9c,  0x00009fa3,  0x00009faa,  0x00009fb1,  0x00009fb8,
	0x00009fbf,  0x00009fc6,  0x00009fcd,  0x00009fd4,  0x00009fdb,
	0x00009fe2,  0x00009fe9,  0x00009ff0,  0x00009ff7,  0x00009ffe,
};

int tegra_nvdisp_set_output_lut(struct tegra_dc *dc,
					struct tegra_dc_lut *lut)
{
	tegra_dc_writel(dc,
			tegra_dc_reg_l32(lut->phy_addr),
			nvdisp_output_lut_base_r());
	tegra_dc_writel(dc,
			tegra_dc_reg_h32(lut->phy_addr),
			nvdisp_output_lut_base_hi_r());
	tegra_dc_writel(dc, nvdisp_output_lut_ctl_size_1025_f(),
			nvdisp_output_lut_ctl_r());

	return 0;
}

void tegra_nvdisp_get_default_cmu(struct tegra_dc_cmu *default_cmu)
{
	int i;
	u64 r = 0;

	for (i = 0; i < NVDISP_OUTPUT_LUT_SIZE; i++) {
		r = default_sRGB_OutLUT[i];
		default_cmu->rgb[i] = (r << 32) |
					(r << 16) | r;
	}
}

static int nvdisp_alloc_output_lut(struct tegra_dc *dc)
{
	struct tegra_dc_lut *lut;
	int i = 0;
	u64 r = 0;

	lut = &dc->cmu;

	if (!lut)
		return -ENOMEM;
	/* Allocate the memory for LUT */
	lut->size = NVDISP_OUTPUT_LUT_SIZE * sizeof(u64);
	lut->rgb = (u64 *)dma_zalloc_coherent(&dc->ndev->dev, lut->size,
			&lut->phy_addr, GFP_KERNEL);
	if (!lut->rgb)
		return -ENOMEM;

	/* Init LUT with cmu data provided from DT file */
	if (dc->pdata->cmu && dc->pdata->cmu_enable) {
		memcpy(lut->rgb, dc->pdata->cmu, lut->size);
		return 0;
	}

	/* Init the LUT table with default sRGB values */
	for (i = 0; i < NVDISP_OUTPUT_LUT_SIZE; i++) {
		r = default_sRGB_OutLUT[i];
		/* Represent r,g,b in 64bit */
		lut->rgb[i] = (r << 32) |
				(r << 16) | r;
	}

	return 0;
}

static int nvdisp_alloc_input_lut(struct tegra_dc *dc,
					struct tegra_dc_win *win,
					bool winlut)
{
	struct tegra_dc_lut *lut;

	if (winlut)
		lut = &win->lut;
	else
		lut = &dc->fb_lut;

	if (!lut)
		return -ENOMEM;

	/* Allocate the memory for LUT */
	lut->size = NVDISP_INPUT_LUT_SIZE * sizeof(u64);
	lut->rgb = (u64 *)dma_zalloc_coherent(&dc->ndev->dev, lut->size,
			&lut->phy_addr, GFP_KERNEL);
	if (!lut->rgb)
		return -ENOMEM;

	return 0;
}

/*	Deassert all the common nvdisplay resets.
 *      Misc and all windows groups are placed in common.
 */
static int __maybe_unused
	tegra_nvdisp_common_reset_deassert(struct tegra_dc *dc)
{
	u8 i;
	int err;


	for ( i = 0; i < DC_N_WINDOWS; i++) {

		if (!nvdisp_common_rst[i]) {
			dev_err(&dc->ndev->dev, "No nvdisp resets available\n");
			return -EINVAL;
		}

		err = reset_control_deassert(nvdisp_common_rst[i]);
		if (err) {
			dev_err(&dc->ndev->dev, "Unable to reset misc\n");
			return err;
		}
	}
	return 0;
}

static int __maybe_unused tegra_nvdisp_common_reset_assert(struct tegra_dc *dc)
{
	u8 i;
	int err;


	for ( i = 0; i < DC_N_WINDOWS; i++) {
		if (!nvdisp_common_rst[i]) {
			dev_err(&dc->ndev->dev, "No Nvdisp resets available\n");
			return -EINVAL;
		}

		err = reset_control_assert(nvdisp_common_rst[i]);
		if (err) {
			dev_err(&dc->ndev->dev, "Unable to reset misc\n");
			return err;
		}
	}
	return 0;
}

static int __maybe_unused tegra_nvdisp_wgrp_reset_assert(struct tegra_dc *dc)
{
	int idx, err = 0;
	for_each_set_bit(idx, &dc->valid_windows, DC_N_WINDOWS) {
		err = reset_control_assert(nvdisp_common_rst[idx+1]);
		if (err)
			dev_err(&dc->ndev->dev, "Failed window %d rst\n", idx);
	}

	return err;
}

static int tegra_nvdisp_wgrp_reset_deassert(struct tegra_dc *dc)
{
	int idx, err = 0;

	/* Misc deassert is common for  all windows */
	err = reset_control_deassert(nvdisp_common_rst[0]);
	if (err)
		dev_err(&dc->ndev->dev, "Failed Misc deassert\n");

	for_each_set_bit(idx, &dc->valid_windows, DC_N_WINDOWS) {
		err = reset_control_deassert(nvdisp_common_rst[idx+1]);
		if (err)
			dev_err(&dc->ndev->dev, "Failed window deassert\n");
	}

	return err;
}

static int tegra_nvdisp_reset_prepare(struct tegra_dc *dc)
{
	char rst_name[6];
	int i;

	/* Use only if bpmp is enabled */
	if (!tegra_bpmp_running())
		return 0;

	nvdisp_common_rst[0] =
		devm_reset_control_get(&dc->ndev->dev, "misc");
	if (IS_ERR(nvdisp_common_rst[0])) {
		dev_err(&dc->ndev->dev, "Unable to get misc reset\n");
		return PTR_ERR(nvdisp_common_rst[0]);
	}

	for ( i = 0; i < DC_N_WINDOWS; i++) {
		snprintf(rst_name, sizeof(rst_name), "wgrp%u", i);
		nvdisp_common_rst[i+1] =
			devm_reset_control_get(&dc->ndev->dev, rst_name);
		if (IS_ERR(nvdisp_common_rst[i+1])) {
			dev_err(&dc->ndev->dev,"Unable to get %s reset\n",
					rst_name);
			return PTR_ERR(nvdisp_common_rst[i+1]);
		}
	}

	return 0;
}

int tegra_nvdisp_set_compclk(struct tegra_dc *dc)
{
	int i;
	unsigned long rate = 0;
	bool compclk_already_on = false;

	compclk_client[dc->ctrl_num].clk = dc->clk;
	compclk_client[dc->ctrl_num].rate = dc->mode.pclk;
	if (compclk_client[dc->ctrl_num].inuse)
		compclk_already_on = true;
	compclk_client[dc->ctrl_num].inuse = true;

	/* comp clk will be maximum of head0/1/2 */
	for (i = 0; i < TEGRA_MAX_DC; i++) {
		if (compclk_client[i].inuse &&
			rate <= compclk_client[i].rate) {
			rate = compclk_client[i].rate;
			pr_info(" rate get on compclk %ld\n", rate);
			/* Set parent for Display clock */
			clk_set_parent(compclk, compclk_client[i].clk);
		}
	}

	/* Enable Display comp clock */
	if (!compclk_already_on)
		tegra_disp_clk_prepare_enable(compclk);

	return 0;
}

static int _tegra_nvdisp_init_once(struct tegra_dc *dc)
{
	int ret = 0;
	int i;
	char syncpt_name[] = "disp_a";

/*	mutex_lock(&tegra_nvdisp_lock); */

	ret = tegra_nvdisp_reset_prepare(dc);
	if (ret)
		return ret;

	/* Get the nvdisplay_hub and nvdisplay_disp clock and enable
	 * it by default. Change the rates based on requirement later
	 */
	hubclk = tegra_disp_clk_get(&dc->ndev->dev, "nvdisplayhub");
	if (IS_ERR_OR_NULL(hubclk)) {
		dev_err(&dc->ndev->dev, "can't get display hub clock\n");
		ret = -ENOENT;
		goto INIT_EXIT;
	}

	compclk = tegra_disp_clk_get(&dc->ndev->dev, "nvdisplay_disp");
	if (IS_ERR_OR_NULL(compclk)) {
		dev_err(&dc->ndev->dev, "can't get display comp clock\n");
		ret = -ENOENT;
		goto INIT_CLK_ERR;
	}

	/* Init sycpt ids */
	dc->valid_windows = 0x3f; /* Assign all windows to this head */
	for (i = 0; i < DC_N_WINDOWS; ++i, ++syncpt_name[5]) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		win->syncpt.id = nvhost_get_syncpt_client_managed(dc->ndev,
								syncpt_name);

		/* allocate input LUT memory and assign to HW */
		if (nvdisp_alloc_input_lut(dc, win, true))
			goto INIT_ERR;

		/* init default CSC */
		tegra_nvdisp_init_csc_defaults(&win->csc);
	}

	dc->valid_windows = 0;

	/* Assign powergate id for each partition*/
	nvdisp_pg[NVDISP_PD_INDEX].powergate_id =
			tegra_pd_get_powergate_id(nvdisp_disa_pd);
	nvdisp_pg[NVDISPB_PD_INDEX].powergate_id =
			tegra_pd_get_powergate_id(nvdisp_disb_pd);
	nvdisp_pg[NVDISPC_PD_INDEX].powergate_id =
			tegra_pd_get_powergate_id(nvdisp_disc_pd);

	goto INIT_EXIT;

INIT_ERR:
	for (i = 0; i < DC_N_WINDOWS; ++i) {
		struct tegra_dc_lut *lut;
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		/* Allocate the memory for Input LUT & fb LUT*/
		lut = &win->lut;
		if (lut->rgb)
			dma_free_coherent(&dc->ndev->dev, lut->size,
				(void *)lut->rgb, lut->phy_addr);
	}
INIT_CLK_ERR:
	if (hubclk)
		tegra_disp_clk_put(&dc->ndev->dev, hubclk);

	if (compclk)
		tegra_disp_clk_put(&dc->ndev->dev, compclk);
INIT_EXIT:
/*	mutex_unlock(&tegra_nvdisp_lock); */
	return ret;

}

int tegra_nvdisp_program_mode(struct tegra_dc *dc, struct tegra_dc_mode
				     *mode)
{
	unsigned long v_back_porch;
	unsigned long v_front_porch;
	unsigned long v_sync_width;
	unsigned long v_active;
	u32 csc2_control;

	if (!dc->mode.pclk)
		return 0;

	v_back_porch = mode->v_back_porch;
	v_front_porch = mode->v_front_porch;
	v_sync_width = mode->v_sync_width;
	v_active = mode->v_active;

	if (mode->vmode == FB_VMODE_INTERLACED) {
		v_back_porch /= 2;
		v_front_porch /= 2;
		v_sync_width /= 2;
		v_active /= 2;
	}

	tegra_dc_get(dc);

	/* IMP related updates */
	dc->new_bw_kbps = tegra_dc_calc_min_bandwidth(dc);
	tegra_dc_program_bandwidth(dc, true);

	tegra_dc_writel(dc,
		nvdisp_sync_width_h_f(mode->h_sync_width) |
		nvdisp_sync_width_v_f(v_sync_width),
		nvdisp_sync_width_r());
	if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP) ||
		(dc->out->type == TEGRA_DC_OUT_NVSR_DP) ||
		(dc->out->type == TEGRA_DC_OUT_LVDS)) {
		tegra_dc_writel(dc,
			nvdisp_back_porch_h_f(mode->h_back_porch) |
			nvdisp_back_porch_v_f(
				(v_back_porch - mode->v_ref_to_sync)),
			nvdisp_back_porch_r());
		tegra_dc_writel(dc,
			nvdisp_front_porch_h_f(mode->h_front_porch) |
			nvdisp_front_porch_v_f(
				(v_front_porch + mode->v_ref_to_sync)),
			nvdisp_front_porch_r());
	} else {
		tegra_dc_writel(dc,
			nvdisp_back_porch_h_f(mode->h_back_porch) |
			nvdisp_back_porch_v_f(v_back_porch),
			nvdisp_back_porch_r());
		tegra_dc_writel(dc,
			nvdisp_front_porch_h_f(mode->h_front_porch) |
			nvdisp_front_porch_v_f(v_front_porch),
			nvdisp_front_porch_r());
	}
	tegra_dc_writel(dc,
			nvdisp_active_h_f(mode->h_active) |
			nvdisp_active_v_f(v_active),
			nvdisp_active_r());


#if defined(CONFIG_TEGRA_DC_INTERLACE)
	if (mode->vmode == FB_VMODE_INTERLACED)
		tegra_dc_writel(dc, INTERLACE_MODE_ENABLE |
			INTERLACE_START_FIELD_1
			| INTERLACE_STATUS_FIELD_1,
			nvdisp_interlace_ctl_r());
	else
		tegra_dc_writel(dc, INTERLACE_MODE_DISABLE,
			nvdisp_interlace_ctl_r());

	if (mode->vmode == FB_VMODE_INTERLACED) {
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_width_v_f(v_sync_width),
			nvdisp_interlace_fld2_width_r());
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_bporch_v_f(v_back_porch + 1),
			nvdisp_interlace_fld2_bporch_r());
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_active_v_f(v_active),
			nvdisp_interlace_fld2_active_r());
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_fporch_v_f(v_front_porch),
			nvdisp_interlace_fld2_fporch_r());
	}
#endif

	/* TODO: MIPI/CRT/HDMI clock cals */
	/* TODO: confirm shift clock still exists in Parker */
	if (dc->mode.pclk != mode->pclk)
		pr_info("Redo Clock pclk 0x%x != dc-pclk 0x%x\n",
				mode->pclk, dc->mode.pclk);


	/* Check whether the extended colorimetry
	 * is requested in mode set for output csc
	 */
	csc2_control = nvdisp_csc2_control_output_color_sel_rgb_f();

	if (mode->vmode & FB_VMODE_EC_ENABLE) {
		if ((mode->vmode & FB_VMODE_EC_ADOBE_YCC601) ||
			(mode->vmode & FB_VMODE_EC_SYCC601) ||
			(mode->vmode & FB_VMODE_EC_XVYCC601))
			csc2_control =
				nvdisp_csc2_control_output_color_sel_y601_f();
		else if ((mode->vmode & FB_VMODE_EC_BT2020_CYCC) ||
			(mode->vmode & FB_VMODE_EC_BT2020_YCC_RGB))
			csc2_control =
				nvdisp_csc2_control_output_color_sel_y2020_f();
		else if (mode->vmode & FB_VMODE_EC_XVYCC709)
			csc2_control =
				nvdisp_csc2_control_output_color_sel_y709_f();
	}

	tegra_dc_writel(dc, csc2_control, nvdisp_csc2_control_r());

	/* general-update */
	tegra_dc_writel(dc, nvdisp_cmd_state_ctrl_general_update_enable_f(),
			nvdisp_cmd_state_ctrl_r());
	tegra_dc_readl(dc, nvdisp_cmd_state_ctrl_r()); /* flush */

#ifdef CONFIG_SWITCH
	switch_set_state(&dc->modeset_switch,
			 (mode->h_active << 16) | mode->v_active);
#endif

	tegra_dc_writel(dc, nvdisp_cmd_state_ctrl_general_act_req_enable_f(),
			nvdisp_cmd_state_ctrl_r());
	tegra_dc_readl(dc, nvdisp_cmd_state_ctrl_r()); /* flush */

	if (dc->out_ops && dc->out_ops->modeset_notifier)
		dc->out_ops->modeset_notifier(dc);

	tegra_dc_put(dc);

	dc->mode_dirty = false;

	trace_display_mode(dc, &dc->mode);
	return 0;
}


int tegra_nvdisp_init(struct tegra_dc *dc)
{
	char rst_name[6];
	int err = 0;
	char vblank_name[32];

	/* Only need init once no matter how many dc objects */
	if (!dc->ndev->id) {
		err = _tegra_nvdisp_init_once(dc);
		if (err)
			return err;
	}

	/*Lut alloc is needed per dc */
	if (!dc->fb_lut.rgb) {
		if (nvdisp_alloc_input_lut(dc, NULL, false))
			return -ENOMEM;
	}

	/* Output LUT is needed per dc */
	if (!(dc->cmu.rgb)) {
		if (nvdisp_alloc_output_lut(dc))
			return -ENOMEM;
	}

	/* Set the valid windows as per mask */
	dc->valid_windows = dc->pdata->win_mask;

	/* Assign powergate id for each partition*/
	dc->powergate_id = nvdisp_pg[dc->ctrl_num].powergate_id;

	/* Save for powermgmt purpose */
	/* valid_windows should be updated on dynamically changing windows */
	nvdisp_pg[dc->ctrl_num].valid_windows = dc->valid_windows;

	/* Allocate a syncpoint for vblank on each head */
	snprintf(vblank_name, sizeof(vblank_name), "vblank%u", dc->ctrl_num);
	dc->vblank_syncpt = nvhost_get_syncpt_client_managed(dc->ndev,
								vblank_name);

	/* Take the controller out of reset if bpmp is loaded*/
	if (tegra_bpmp_running() && tegra_platform_is_silicon()) {
		snprintf(rst_name, sizeof(rst_name), "head%u", dc->ctrl_num);
		dc->rst = devm_reset_control_get(&dc->ndev->dev, rst_name);
		if (IS_ERR(dc->rst)) {
			dev_err(&dc->ndev->dev,"Unable to get %s reset\n",
				rst_name);
			return PTR_ERR(dc->rst);
		}
	}

	return err;
}

static int tegra_nvdisp_set_control(struct tegra_dc *dc)
{
	u32 protocol = nvdisp_sor_control_protocol_custom_f();
	u32 reg      = nvdisp_sor_control_r();

	/* Set the protocol type in DT and use from there
	 * Current setting are default ones.
	 */

	if (dc->out->type == TEGRA_DC_OUT_HDMI)	{
		protocol = nvdisp_sor1_control_protocol_tmdsa_f();
		if(!strcmp(dc_or_node_names[dc->ndev->id], "/host1x/sor1"))
			reg = nvdisp_sor1_control_r();
		else if(!strcmp(dc_or_node_names[dc->ndev->id], "/host1x/sor"))
			reg = nvdisp_sor_control_r();
	} else if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_NVSR_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP)) {
		protocol = nvdisp_sor_control_protocol_dpa_f();
		reg = nvdisp_sor_control_r();
	} else if ((dc->out->type == TEGRA_DC_OUT_DSI) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DSIA) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DSIB) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED)) {
		protocol = nvdisp_dsi_control_protocol_dsia_f();
		reg = nvdisp_dsi_control_r();
	}

	tegra_dc_writel(dc, protocol, reg);
	tegra_dc_enable_general_act(dc);
	return 0;
}

static int tegra_nvdisp_head_init(struct tegra_dc *dc)
{
	u32 int_enable;
	u32 int_mask;
	u32 i, val;

	/* Init syncpt */
	tegra_dc_writel(dc, nvdisp_incr_syncpt_cntrl_no_stall_f(1),
		nvdisp_incr_syncpt_cntrl_r());

	/* Disabled this feature as unit fpga hang on enabling this*/
	if (tegra_platform_is_silicon())
		tegra_dc_writel(dc, nvdisp_cont_syncpt_vsync_en_enable_f() |
			nvdisp_cont_syncpt_vsync_indx_f(dc->vblank_syncpt),
			nvdisp_cont_syncpt_vsync_r());

	/* Init interrupts */
	/* Setting Int type. EDGE for most, LEVEL for UF related */
	tegra_dc_writel(dc, 0x3C000000, nvdisp_int_type_r());
	/* Setting all the Int polarity to high */
	tegra_dc_writel(dc, 0x3D8010F6, nvdisp_int_polarity_r());

	/* enable interrupts for vblank, frame_end and underflows */
	int_enable = nvdisp_cmd_int_status_frame_end_f(1) |
			nvdisp_cmd_int_status_v_blank_f(1) |
			nvdisp_cmd_int_status_uf_f(1) |
			nvdisp_cmd_int_status_sd3_f(1);
	/* for panels with one-shot mode enable tearing effect interrupt */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		int_enable |= MSF_INT;
	/* Todo: also need to enable interrupts for SD3, DSC etc */

	tegra_dc_writel(dc, int_enable, nvdisp_cmd_int_enable_r());

	int_mask = nvdisp_cmd_int_status_uf_f(1);
	tegra_dc_writel(dc, int_mask, nvdisp_cmd_int_mask_r());

	tegra_dc_writel(dc, nvdisp_state_access_write_mux_assembly_f() |
		nvdisp_state_access_read_mux_active_f(),
		nvdisp_state_access_r());

	tegra_dc_writel(dc, 0x00000000, nvdisp_background_color_r());

	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		BUG_ON(!win);

		/* refuse to operate on invalid syncpts */
		if (WARN_ON(win->syncpt.id == NVSYNCPT_INVALID))
			continue;

		if (!nvhost_syncpt_read_ext_check(dc->ndev,
						win->syncpt.id, &val))
			win->syncpt.min = win->syncpt.max = val;
	}

	dc->crc_pending = false;

	/* set mode */
	tegra_nvdisp_program_mode(dc, &dc->mode);

	/*set display control */
	tegra_nvdisp_set_control(dc);

	tegra_nvdisp_set_color_control(dc);

	tegra_dc_enable_general_act(dc);

	return 0;
}

static int tegra_nvdisp_postcomp_init(struct tegra_dc *dc)
{
	/* Set the LUT address in HW register
	 * Enabled the default sRGB_LUT, replace this
	 * with LUT from Panel Characterization through
	 * DT.
	 */
	struct tegra_dc_lut *lut = &dc->cmu;

	dc->pdata->cmu_enable = dc->cmu_enabled = true;

	tegra_nvdisp_set_output_lut(dc, lut);
	tegra_nvdisp_set_color_control(dc);
	tegra_dc_enable_general_act(dc);

	return 0;
}

static int tegra_nvdisp_rg_init(struct tegra_dc *dc)
{
	return 0;
}

static int tegra_nvdisp_cursor_init(struct tegra_dc *dc)
{
	return 0;
}

int tegra_nvdisp_head_disable(struct tegra_dc *dc)
{
	int idx;

	/* Detach windows from the head */
	for_each_set_bit(idx, &dc->pdata->win_mask, DC_N_WINDOWS) {
		if (tegra_nvdisp_detach_win(dc, idx))
			dev_err(&dc->ndev->dev,
				"failed to detach window %d\n", idx);
		else
			dev_dbg(&dc->ndev->dev,
				"Window %d detached from head %d\n", idx,
				dc->ctrl_num);
	}

	/* Disable display comp clock */
	if (compclk_client[dc->ctrl_num].inuse) {
		tegra_disp_clk_disable_unprepare(compclk);
		compclk_client[dc->ctrl_num].inuse = false;
	}

	/* Disable DC clock */
	tegra_disp_clk_disable_unprepare(dc->clk);

	/* Disable display hub clock */
	tegra_disp_clk_disable_unprepare(hubclk);
	return 0;
}

int tegra_nvdisp_head_enable(struct tegra_dc *dc)
{
	int i;
	int res;
	int idx, pclk = 0, ret = 0;
	struct clk *parent_clk = NULL;
	struct clk *hubparent_clk = NULL;

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return false;

	/* Save for powermgmt purpose */
	nvdisp_pg[dc->ctrl_num].valid_windows = dc->pdata->win_mask;

	/* TODO: confirm power domains for parker */
	tegra_dc_unpowergate_locked(dc);

	/* Set HUB CLOCK PARENT as PLLP/ ENABLE */
	/* Change the HUB to HUBPLL for higher clocks */
	hubparent_clk = tegra_disp_clk_get(&dc->ndev->dev, "pllp_display");
	if (IS_ERR_OR_NULL(hubparent_clk)) {
		dev_err(&dc->ndev->dev, "hub parent clock get failed\n");
		ret = -ENOENT;
		return ret; /*TODO: Add proper cleanup later */
	}
	clk_set_parent(hubclk, hubparent_clk);
	tegra_disp_clk_prepare_enable(hubclk);
	pr_info(" rate get on hub %ld\n", clk_get_rate(hubclk));

	/* Enable OR -- need to enable the connection first */
	if (dc->out->enable)
		dc->out->enable(&dc->ndev->dev);

	/* Setting DC clocks, DC, COMPCLK
	 * Set maximum of DC clock for COMPCLK
	 */
	if (dc->out->type == TEGRA_DC_OUT_DSI) {
		parent_clk = tegra_disp_clk_get(&dc->ndev->dev,
						"pll_d_out1");
	} else	{
		parent_clk = tegra_disp_clk_get(&dc->ndev->dev,
						dc->out->parent_clk);
		pr_info("Parent Clock set for DC %s\n",
				dc->out->parent_clk);
	}

	if (IS_ERR_OR_NULL(parent_clk)) {
		dev_err(&dc->ndev->dev,
			"Failed to get DC Parent clock\n");
		ret = -ENOENT;
		return ret; /*TODO: Add proper cleanup later */
	}

	/* Set parent for DC clock */
	clk_set_parent(dc->clk, parent_clk);

	/* Set rate on DC same as pclk */
	clk_set_rate(dc->clk, dc->mode.pclk);

	if (dc->out_ops->setup_clk)
		pclk = dc->out_ops->setup_clk(dc, dc->clk);

	/* Enable DC clock */
	tegra_disp_clk_prepare_enable(dc->clk);

	pr_info(" dc clk %ld\n", clk_get_rate(dc->clk));

	tegra_nvdisp_set_compclk(dc);

	tegra_dc_get(dc);

	/* Deassert the dc reset */
	res = reset_control_deassert(dc->rst);
	if (res) {
		dev_err(&dc->ndev->dev, "Unable to deassert dc %d\n",
				dc->ctrl_num);
		return res;
	}

	tegra_nvdisp_wgrp_reset_deassert(dc);

	/* Mask interrupts during init */
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	enable_irq(dc->irq);

	res = tegra_nvdisp_head_init(dc);
	res |= tegra_nvdisp_postcomp_init(dc);
	res |= tegra_nvdisp_rg_init(dc);
	res |= tegra_nvdisp_cursor_init(dc);

	if (res) {
		dev_err(&dc->ndev->dev, "%s, failed head enable\n", __func__);
		goto failed_enable;
	}

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	/* force a full blending update */
	for (i = 0; i < DC_N_WINDOWS; i++)
		dc->blend.z[i] = -1;

	tegra_dc_ext_enable(dc->ext);
	trace_display_enable(dc);

	tegra_dc_dsc_init(dc);

	if (dc->out->postpoweron)
		dc->out->postpoweron(&dc->ndev->dev);

	if (dc->out_ops && dc->out_ops->postpoweron)
		dc->out_ops->postpoweron(dc);

	tegra_log_resume_time();
	/*
	 * We will need to reinitialize the display the next time panel
	 * is enabled.
	 */
	dc->out->flags &= ~TEGRA_DC_OUT_INITIALIZED_MODE;

	/* Assign windows to this head */
	for_each_set_bit(idx, &dc->pdata->win_mask, DC_N_WINDOWS) {
		if (tegra_nvdisp_assign_win(dc, idx))
			dev_err(&dc->ndev->dev,
				"failed to assign window %d\n", idx);
		else
			dev_dbg(&dc->ndev->dev,
				"Window %d assigned to head %d\n", idx,
				dc->ctrl_num);
	}

	/* Set the fb_index on changing from a zero win_mask to
	 * to a valid one.
	 */
	if ((dc->pdata->fb->win == -1) && dc->pdata->win_mask) {
		tegra_fb_set_win_index(dc, dc->pdata->win_mask);
		dc->pdata->fb->win = dc->pdata->win_mask;
	}

	/* Enable RG underflow logging */
	tegra_dc_writel(dc, nvdisp_rg_underflow_enable_enable_f() |
		nvdisp_rg_underflow_mode_red_f(),
		nvdisp_rg_underflow_r());

	tegra_dc_put(dc);
	return 0;

failed_enable:
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
	disable_irq_nosync(dc->irq);
	tegra_dc_clear_bandwidth(dc);
	if (dc->out && dc->out->disable)
		dc->out->disable(&dc->ndev->dev);
	tegra_dc_put(dc);

	/* TODO: disable DC clock */
	return -EINVAL;
}

u32 tegra_nvdisp_ihub_read(struct tegra_dc *dc, int win_number,
				int ihub_switch)
{
	u32 reg = 0;
	u32 ret_val = 0;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	switch (ihub_switch) {
	case 0: /* mempool size */
		reg = tegra_dc_readl(dc, nvdisp_ihub_capa_r());
		/* base entry width is 32 bytes */
		ret_val = nvdisp_ihub_capa_mempool_entries_v(reg) *
			(32 << nvdisp_ihub_capa_mempool_width_v(reg));
		break;
	default:
		break;
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return ret_val;
}

struct tegra_fb_info *tegra_nvdisp_fb_register(struct platform_device *ndev,
	struct tegra_dc *dc, struct tegra_fb_data *fb_data,
	struct resource *fb_mem)
{
	void *virt_addr = NULL;

	/* Check fb_data->win is valid before checking for valid window */
	if (fb_data->win > -1) {
		/* Assign the given window to current dc */
		if (!tegra_dc_get_window(dc, fb_data->win)) {
			dev_err(&ndev->dev, "%s, failed to get window %d for head %d\n",
				__func__, fb_data->win, dc->ctrl_num);
			return ERR_PTR(-ENOENT);
		}

	}

	/* Allocate FBMem if not already allocated */
	if (!fb_mem->start || !fb_mem->end) {
		/* lines must be 64B aligned */
		int stride = round_up(fb_data->xres *
					fb_data->bits_per_pixel / 8, 64);
		/* Add space to permit adjustment of start of buffer.
		 * start of buffer requires 256B alignment. */
		int fb_size = stride * fb_data->yres + 256;

		if (!fb_size)
			return ERR_PTR(-ENOENT);

		virt_addr = dma_alloc_writecombine(&ndev->dev, fb_size,
			&fb_mem->start, GFP_KERNEL);
		if (!virt_addr) {
			dev_err(&ndev->dev, "Failed to allocate FBMem\n");
			return ERR_PTR(-ENOENT);
		}
		fb_mem->end = fb_mem->start + fb_size - 1;
		dev_info(&ndev->dev, "Allocated %d as FBmem\n", fb_size);
	}

	return tegra_fb_register(ndev, dc, fb_data, fb_mem, virt_addr);
}

void tegra_nvdisp_enable_crc(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	tegra_dc_writel(dc, nvdisp_crc_control_enable_enable_f() |
		nvdisp_crc_control_input_data_active_data_f(),
		nvdisp_crc_control_r());

	tegra_dc_enable_general_act(dc);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	/* Register a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, true);
}

void tegra_nvdisp_disable_crc(struct tegra_dc *dc)
{
	/* Unregister a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, false);

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, 0x0, nvdisp_crc_control_r());
	tegra_dc_enable_general_act(dc);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

u32 tegra_nvdisp_read_rg_crc(struct tegra_dc *dc)
{
	int crc = 0;
	int val = 0;

	if (!dc) {
		pr_err("Failed to get dc: NULL parameter.\n");
		goto crc_error;
	}

	/* If gated quitely return */
	if (!tegra_dc_is_powered(dc))
		return 0;

#ifdef INIT_COMPLETION
	INIT_COMPLETION(dc->crc_complete);
#else
	reinit_completion(&dc->crc_complete);
#endif
	if (dc->crc_pending &&
	    wait_for_completion_interruptible(&dc->crc_complete)) {
		pr_err("CRC read interrupted.\n");
		goto crc_error;
	}

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	val = tegra_dc_readl(dc, nvdisp_rg_crca_r());

	/* tegrasim seems to need more time to set the
	 * CRCA valid bit. So adding an infinite
	 * polling loop for tegrasim
	 */
	if (tegra_platform_is_linsim()) {
		while (val <= 0) {
			val = tegra_dc_readl(dc, nvdisp_rg_crca_r());
			msleep(100);
		}
	}

	if (val & nvdisp_rg_crca_valid_true_f())
		crc = tegra_dc_readl(dc, nvdisp_rg_crcb_r());
	/* clear the error bit if set */
	if (val & nvdisp_rg_crca_error_true_f())
		tegra_dc_writel(dc, nvdisp_rg_crca_error_true_f(),
			nvdisp_rg_crca_r());
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
crc_error:
	return crc;
}


void tegra_nvdisp_underflow_handler(struct tegra_dc *dc)
{
	u32 reg = tegra_dc_readl(dc, nvdisp_rg_underflow_r());
	dc->stats.underflows++;

	if (dc->underflow_mask & NVDISP_UF_INT)
		dc->stats.underflow_frames +=
				nvdisp_rg_underflow_frames_uflowed_v(reg);

	/* Clear the sticky bit and counter */
	tegra_dc_writel(dc,
		nvdisp_rg_underflow_frames_uflowed_rst_trigger_f() |
		nvdisp_rg_underflow_uflowed_clr_f(),
		nvdisp_rg_underflow_r());

	/* Do we need to see whether the reset is done */
}

int tegra_nvdisp_powergate_partition(int pg_id)
{
	int i , ret = 0;
	int pd_index = -1;
	bool disable_disp[NVDISP_PD_COUNT] = {
			false, false, false };

	/* DISP - common for all IHUB, ORs, head0, win 0
	 * Request from DISPB or DISPC - disable DISP last
	 * Check DISPB and DISPC valid_windows
	 * in addition to the request for HEAD 1 and 2 powergate
	 */

	for (i = 0; i < NVDISP_PD_COUNT; i++) {
		if (nvdisp_pg[i].powergate_id == pg_id) {
			pd_index = i;
			break;
		}
	}

	if (pd_index < 0) {
		pr_info("Not a disp powerdomain\n");
		return 0;
	}

	if (!nvdisp_pg[pd_index].ref_cnt) {
		pr_info("Already powergated DISP id %d\n",
				 nvdisp_pg[i].powergate_id);
		return 0;
	}

	mutex_lock(&tegra_nvdisp_lock);
	/* Check any valid_windows resides in another PD
	 * the check whether those head are in use before
	 * powering off those PDs
	 */
	for_each_set_bit(i, &nvdisp_pg[pd_index].valid_windows, DC_N_WINDOWS) {
		if (i == 0 && nvdisp_pg[NVDISP_PD_INDEX].windows_inuse) {
			nvdisp_pg[NVDISP_PD_INDEX].windows_inuse -= 1;
			disable_disp[NVDISP_PD_INDEX] = true;
		} else if (((i == 1) || (i == 2)) &&
			nvdisp_pg[NVDISPB_PD_INDEX].windows_inuse) {
			nvdisp_pg[NVDISPB_PD_INDEX].windows_inuse -= 1;
			disable_disp[NVDISPB_PD_INDEX] = true;
		} else if (nvdisp_pg[NVDISPC_PD_INDEX].windows_inuse) {
			nvdisp_pg[NVDISPC_PD_INDEX].windows_inuse -= 1;
			disable_disp[NVDISPC_PD_INDEX] = true;
		}
	}

	/* Check head is in use */
	if (nvdisp_pg[pd_index].head_inuse)
		disable_disp[pd_index] = true;

	nvdisp_pg[pd_index].head_inuse = false;

	disable_disp[pd_index] = true;
	/* Request from DISPB or DISPC - disable DISP also */
	if ((pd_index == 1) || (pd_index == 2))
		disable_disp[NVDISP_PD_INDEX] = true;

	for (i = NVDISP_PD_COUNT - 1; i >= 0; i--) {
		if (disable_disp[i] && (--nvdisp_pg[i].ref_cnt == 0)) {

			if (nvdisp_pg[i].windows_inuse &&
				nvdisp_pg[i].head_inuse)
				pr_err("Error in Windows/Head ref_count\n");

			pr_info("PD DISP%d index%d DOWN\n",
					 i, nvdisp_pg[i].powergate_id);
			/* User when using pg with clk_on */
			ret = tegra_powergate_partition_with_clk_off(
						nvdisp_pg[i].powergate_id);
			/*ret = tegra_powergate_partition(
						nvdisp_pg[i].powergate_id);*/
			if (ret)
				pr_err("Fail to powergate DISP%d\n", i);
		}
	}
	mutex_unlock(&tegra_nvdisp_lock);

	return ret;
}

int tegra_nvdisp_unpowergate_partition(int pg_id)
{
	int i, ret = 0;
	int pd_index = -1;
	bool enable_disp[NVDISP_PD_COUNT] = {
			false , false, false};

	/* DISP - common for all IHUB, ORs, head0, win 0
	 * Request from DISPB or DISPC - enable DISP first
	 * Enable DISPB and DISPC based on valid_windows
	 * checking
	 */

	for (i = 0; i < NVDISP_PD_COUNT; i++) {
		if (nvdisp_pg[i].powergate_id == pg_id) {
			pd_index = i;
			break;
		}
	}

	if (pd_index < 0) {
		pr_info(" Not a disp powerdomain\n");
		return 0;
	}
	mutex_lock(&tegra_nvdisp_lock);

	nvdisp_pg[pd_index].head_inuse = true;
	enable_disp[pd_index] = true;
	/* Request from DISPB or DISPC - enable DISP first */
	if ((pd_index == 1) || (pd_index == 2))
		enable_disp[NVDISP_PD_INDEX] = true;

	/* Check the for valid_windows per head
	 * win0 is in DISP, win1&2 in DISPB and
	 * win3,win4 &win5 in DISPC domains
	 */

	for_each_set_bit(i, &nvdisp_pg[pd_index].valid_windows, DC_N_WINDOWS) {
		if (i == 0) {
			enable_disp[NVDISP_PD_INDEX] = true;
			nvdisp_pg[NVDISP_PD_INDEX].windows_inuse += 1;
		} else if ((i == 1) || (i == 2)) {
			enable_disp[NVDISPB_PD_INDEX] = true;
			nvdisp_pg[NVDISPB_PD_INDEX].windows_inuse += 1;
		} else { /* win 3/4/5 */
			enable_disp[NVDISPC_PD_INDEX] = true;
			nvdisp_pg[NVDISPC_PD_INDEX].windows_inuse += 1;
		}
	}

	for (i = 0; i < NVDISP_PD_COUNT; i++) {
		if (enable_disp[i] && (nvdisp_pg[i].ref_cnt++ == 0)) {
			pr_info("PD DISP%d index%d UP\n",
					i, nvdisp_pg[i].powergate_id);
			/* use clk_off with clk_on  */
			ret = tegra_unpowergate_partition_with_clk_on(
						nvdisp_pg[i].powergate_id);
			/*ret = tegra_unpowergate_partition(
						nvdisp_pg[i].powergate_id);*/
			if (ret) {
				pr_err("Fail to Unpowergate DISP%d\n", i);
				mutex_unlock(&tegra_nvdisp_lock);
				return ret;
			}
		}
	}

	mutex_unlock(&tegra_nvdisp_lock);
	return ret;
}
static int tegra_nvdisp_set_color_control(struct tegra_dc *dc)
{
	u32 color_control;

	switch (dc->out->depth) {

	case 36:
		color_control = nvdisp_color_ctl_base_color_size_36bits_f();
		break;
	case 30:
		color_control = nvdisp_color_ctl_base_color_size_30bits_f();
		break;
	case 24:
		color_control = nvdisp_color_ctl_base_color_size_24bits_f();
		break;
	case 18:
		color_control = nvdisp_color_ctl_base_color_size_18bits_f();
		break;
	default:
		color_control = nvdisp_color_ctl_base_color_size_24bits_f();
		break;
	}

	switch (dc->out->dither) {
	case TEGRA_DC_UNDEFINED_DITHER:
	case TEGRA_DC_DISABLE_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_disable_f();
		break;
	case TEGRA_DC_ORDERED_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_ordered_f();
		break;
	case TEGRA_DC_TEMPORAL_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_temporal_f();
		break;
	case TEGRA_DC_ERRACC_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_err_acc_f();
		break;
	default:
		dev_err(&dc->ndev->dev, "Error: Unsupported dithering mode\n");
	}

#if defined(CONFIG_TEGRA_DC_CMU_V2)
	if (dc->cmu_enabled)
		color_control |= nvdisp_color_ctl_cmu_enable_f();
#endif
	/* TO DO - dither rotation, dither offset, dither phase */

	tegra_dc_writel(dc, color_control,
			nvdisp_color_ctl_r());
	return 0;
}

#if defined(CONFIG_TEGRA_DC_CMU_V2)
void tegra_dc_cache_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *src_cmu)
{
	/* copy the data to DC lut */
	memcpy(dc->cmu.rgb, src_cmu->rgb, sizeof(*src_cmu));
	dc->cmu_dirty = true;
}

static void _tegra_nvdisp_update_cmu(struct tegra_dc *dc,
					struct tegra_dc_lut *cmu)
{
	dc->cmu_enabled = dc->pdata->cmu_enable;
	if (!dc->cmu_enabled)
		return;

	/* Not disabling the cmu here - will
	 * consider it if there is any corruption on
	 * updating cmu while it is running
	 */
	tegra_nvdisp_set_output_lut(dc, cmu);
	dc->cmu_dirty = false;
}

int tegra_nvdisp_update_cmu(struct tegra_dc *dc, struct tegra_dc_lut *cmu)
{
	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}

	tegra_dc_get(dc);

	_tegra_nvdisp_update_cmu(dc, cmu);
	tegra_nvdisp_set_color_control(dc);
	tegra_dc_writel(dc,
			nvdisp_cmd_state_ctrl_general_act_req_enable_f(),
			nvdisp_cmd_state_ctrl_r());

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_nvdisp_update_cmu);
#endif

static void tegra_nvdisp_program_imp_head_results(struct tegra_dc *dc,
			struct tegra_dc_imp_head_results *imp_head_results)
{
	struct tegra_dc_win *win = NULL;
	u32 val;
	int i;

	if (!dc)
		return;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	for (i = 0; i < imp_head_results->num_windows; i++) {
		win = tegra_dc_get_window(dc, imp_head_results->win_ids[i]);
		if (!win)
			continue;

		/* program wgrp fetch meter slots */
		val = imp_head_results->metering_slots_value_win[i];
		nvdisp_win_write(win,
			win_ihub_fetch_meter_slots_f(val),
			win_ihub_fetch_meter_r());

		/* program wgrp latency registers */
		val = imp_head_results->thresh_lwm_dvfs_win[i];
		tegra_dc_writel(dc,
			tegra_dc_readl(dc, nvdisp_ihub_misc_ctl_r()) &
			~nvdisp_ihub_misc_ctl_latency_event_enable_f(),
			nvdisp_ihub_misc_ctl_r());

		if (val) {
			nvdisp_win_write(win,
				win_ihub_latency_ctla_ctl_mode_enable_f() |
				win_ihub_latency_ctla_submode_watermark_f(),
				win_ihub_latency_ctla_r());
			nvdisp_win_write(win,
				win_ihub_latency_ctlb_watermark_f(val),
				win_ihub_latency_ctlb_r());
			tegra_dc_writel(dc,
				nvdisp_ihub_misc_ctl_latency_event_enable_f(),
				nvdisp_ihub_misc_ctl_r());
		}

		/* program wgrp pipe meter value */
		val = imp_head_results->pipe_meter_value_win[i];
		nvdisp_win_write(win,
			win_precomp_pipe_meter_val_f(val),
			win_precomp_pipe_meter_r());
	}

	/* program cursor fetch meter slots */
	val = imp_head_results->metering_slots_value_cursor;
	tegra_dc_writel(dc,
			nvdisp_ihub_cursor_fetch_meter_slots_f(val),
			nvdisp_ihub_cursor_fetch_meter_r());

	/* program cursor pipe meter value */
	val = imp_head_results->pipe_meter_value_cursor;
	tegra_dc_writel(dc,
			nvdisp_cursor_pipe_meter_val_f(val),
			nvdisp_cursor_pipe_meter_r());

	mutex_unlock(&dc->lock);
}

int tegra_nvdisp_program_imp_results(struct tegra_dc *dc,
				struct tegra_dc_imp_head_results imp_results[])
{
	u32 val = 0;
	int i;

	mutex_lock(&tegra_nvdisp_lock);

	for (i = 0; i < TEGRA_MAX_DC; i++)
		tegra_nvdisp_program_imp_head_results(tegra_dc_get_dc(i),
							&imp_results[i]);

	/* program common win and cursor fetch meter slots */
	val = (imp_results[dc->ndev->id].window_slots_value) |
		(imp_results[dc->ndev->id].cursor_slots_value << 8);
	tegra_dc_writel(dc, val, nvdisp_ihub_common_fetch_meter_r());

	/* promote the COMMON channel state */
	tegra_dc_writel(dc, nvdisp_cmd_state_ctrl_common_act_update_enable_f(),
			nvdisp_cmd_state_ctrl_r());
	tegra_dc_readl(dc, nvdisp_cmd_state_ctrl_r()); /* flush */
	tegra_dc_writel(dc, nvdisp_cmd_state_ctrl_common_act_req_enable_f(),
			nvdisp_cmd_state_ctrl_r());
	tegra_dc_readl(dc, nvdisp_cmd_state_ctrl_r()); /* flush */

	/* wait for COMMON_ACT_REQ to complete or time out */
	if (tegra_dc_poll_register(dc, DC_CMD_STATE_CONTROL,
			COMMON_ACT_REQ, 0, 1, NVDISP_TEGRA_POLL_TIMEOUT_MS))
		dev_err(&dc->ndev->dev,
			"dc timeout waiting for DC to stop\n");

	mutex_unlock(&tegra_nvdisp_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_nvdisp_program_imp_head_results);

void reg_dump(struct tegra_dc *dc, void *data,
		       void (* print)(void *data, const char *str))
{
	int i;
	char buff[256];
	const char winname[] = "ABCDEFT";
	#if 0
	/* If gated, quietly return. */
	if (!tegra_powergate_is_powered(dc->powergate_id)){
		return;
	}
	#endif

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

#define DUMP_REG(a) do {			\
	snprintf(buff, sizeof(buff), "%-32s\t%03x\t%08lx\n",  \
		 #a, a, tegra_dc_readl(dc, a));		      \
	print(data, buff);				      \
	} while (0)

#include "hw_nvdisp_nvdisp_regdump.c"

#undef DUMP_REG

#define DUMP_REG(a) do {				\
	snprintf(buff, sizeof(buff), "%-32s\t%03x\t%08x\n",  \
		 #a, a, nvdisp_win_read(win, a));	      \
	print(data, buff);				      \
	} while (0)


	for (i = 0; i < DC_N_WINDOWS; ++i) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);
		if (!win)
			continue;

		print(data, "\n");
		snprintf(buff, sizeof(buff), "WINDOW %c:\n", winname[i]);
		print(data, buff);

#include "hw_nvdisp_win_regdump.c"

#undef DUMP_REG
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

