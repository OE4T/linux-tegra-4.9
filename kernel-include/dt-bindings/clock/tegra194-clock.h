/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __ABI_MACH_T194_CLOCK_H
#define __ABI_MACH_T194_CLOCK_H

/** @brief output of mux controlled by TEGRA194_CLK_SOC_ACTMON */
#define TEGRA194_CLK_ACTMON			1
/** @brief output of gate CLK_ENB_ADSP */
#define TEGRA194_CLK_ADSP			2
/** @brief output of gate CLK_ENB_ADSPNEON */
#define TEGRA194_CLK_ADSPNEON			3
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_AHUB */
#define TEGRA194_CLK_AHUB			4
/** @brief output of gate CLK_ENB_APB2APE */
#define TEGRA194_CLK_APB2APE			5
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_APE */
#define TEGRA194_CLK_APE			6
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_AUD_MCLK */
#define TEGRA194_CLK_AUD_MCLK			7
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_AXI_CBB */
#define TEGRA194_CLK_AXI_CBB			8
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_CAN1 */
#define TEGRA194_CLK_CAN1			9
/** @brief output of gate CLK_ENB_CAN1_HOST */
#define TEGRA194_CLK_CAN1_HOST			10
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_CAN2 */
#define TEGRA194_CLK_CAN2			11
/** @brief output of gate CLK_ENB_CAN2_HOST */
#define TEGRA194_CLK_CAN2_HOST			12
/** @brief output of gate CLK_ENB_CEC */
#define TEGRA194_CLK_CEC			13
/** @brief output of divider CLK_RST_CONTROLLER_CLK_M_DIVIDE */
#define TEGRA194_CLK_CLK_M			14
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC1 */
#define TEGRA194_CLK_DMIC1			15
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC2 */
#define TEGRA194_CLK_DMIC2			16
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC3 */
#define TEGRA194_CLK_DMIC3			17
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC4 */
#define TEGRA194_CLK_DMIC4			18
/** @brief output of gate CLK_ENB_DPAUX1 */
#define TEGRA194_CLK_DPAUX			19
/** @brief output of gate CLK_ENB_DPAUX1 */
#define TEGRA194_CLK_DPAUX1			20
/**
 * @brief output of mux controlled by CLK_RST_CONTROLLER_ACLK_BURST_POLICY
 * divided by the divider controlled by ACLK_CLK_DIVISOR in
 * CLK_RST_CONTROLLER_SUPER_ACLK_DIVIDER
 */
#define TEGRA194_CLK_ACLK			21
#define TEGRA194_CLK_RSVD_22			22
/** @brief clock recovered from EAVB input */
#define TEGRA194_CLK_EQOS_RX_INPUT		23
/** @brief clock recovered from SPDIFIN input */
#define TEGRA194_CLK_SPDIFIN_SYNC_INPUT		24
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C12 */
#define TEGRA194_CLK_I2C12			25
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C13 */
#define TEGRA194_CLK_I2C13			26
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C14 */
#define TEGRA194_CLK_I2C14			27
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLA1_BASE for use by audio clocks */
#define TEGRA194_CLK_PLLA1			28
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DSPK1 */
#define TEGRA194_CLK_DSPK1			29
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DSPK2 */
#define TEGRA194_CLK_DSPK2			30
/**
 * @brief controls the EMC clock frequency.
 * @details Doing a clk_set_rate on this clock will select the
 * appropriate clock source, program the source rate and execute a
 * specific sequence to switch to the new clock source for both memory
 * controllers. This can be used to control the balance between memory
 * throughput and memory controller power.
 */
#define TEGRA194_CLK_EMC			31
/** @brief output of gate CLK_ENB_EQOS */
#define TEGRA194_CLK_EQOS_AXI			32
/**
 * @brief output of the divider
 * CLK_RST_CONTROLLER_CLK_SOURCE_EQOS_PTP_REF_CLK_0
 */
#define TEGRA194_CLK_EQOS_PTP_REF		33
/** @brief output of gate CLK_ENB_EQOS_RX */
#define TEGRA194_CLK_EQOS_RX			34
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_EQOS_TX_CLK */
#define TEGRA194_CLK_EQOS_TX			35
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH1
 */
#define TEGRA194_CLK_EXTPERIPH1			36
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH2
 */
#define TEGRA194_CLK_EXTPERIPH2			37
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH3
 */
#define TEGRA194_CLK_EXTPERIPH3			38
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH4
 */
#define TEGRA194_CLK_EXTPERIPH4			39
/** @brief output of gate CLK_ENB_FUSE */
#define TEGRA194_CLK_FUSE			40
/** @brief GPC2CLK-div-2 */
#define TEGRA194_CLK_GPCCLK			41
/** @brief TODO */
#define TEGRA194_CLK_GPU_PWR			42
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_HDA2CODEC_2X */
#define TEGRA194_CLK_HDA			43
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_HDA2CODEC_2X */
#define TEGRA194_CLK_HDA2CODEC_2X		44
/** @brief output of gate CLK_ENB_HDA2HDMICODEC */
#define TEGRA194_CLK_HDA2HDMICODEC		45
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_HOST1X */
#define TEGRA194_CLK_HOST1X			46
/** @brief output of gate CLK_ENB_HSIC_TRK */
#define TEGRA194_CLK_HSIC_TRK			47
/** @clkdesc{i2c_clks, out, mux, CLK_RST_CONTROLLER_CLK_SOURCE_I2C1} */
#define TEGRA194_CLK_I2C1			48
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C2 */
#define TEGRA194_CLK_I2C2			49
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C3 */
#define TEGRA194_CLK_I2C3			50
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C4 */
#define TEGRA194_CLK_I2C4			51
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C6 */
#define TEGRA194_CLK_I2C6			52
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C7 */
#define TEGRA194_CLK_I2C7			53
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C8 */
#define TEGRA194_CLK_I2C8			54
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C9 */
#define TEGRA194_CLK_I2C9			55
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S1 */
#define TEGRA194_CLK_I2S1			56
/** @brief clock recovered from I2S1 input */
#define TEGRA194_CLK_I2S1_SYNC_INPUT		57
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S2 */
#define TEGRA194_CLK_I2S2			58
/** @brief clock recovered from I2S2 input */
#define TEGRA194_CLK_I2S2_SYNC_INPUT		59
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S3 */
#define TEGRA194_CLK_I2S3			60
/** @brief clock recovered from I2S3 input */
#define TEGRA194_CLK_I2S3_SYNC_INPUT		61
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S4 */
#define TEGRA194_CLK_I2S4			62
/** @brief clock recovered from I2S4 input */
#define TEGRA194_CLK_I2S4_SYNC_INPUT		63
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S5 */
#define TEGRA194_CLK_I2S5			64
/** @brief clock recovered from I2S5 input */
#define TEGRA194_CLK_I2S5_SYNC_INPUT		65
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C6 */
#define TEGRA194_CLK_I2S6			66
/** @brief clock recovered from I2S6 input */
#define TEGRA194_CLK_I2S6_SYNC_INPUT		67
/** @brief output of gate CLK_ENB_IQC1 */
#define TEGRA194_CLK_IQC1			68
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_ISP */
#define TEGRA194_CLK_ISP			69
/**
 * @brief A fake clock which must be enabled during
 * KFUSE read operations to ensure adequate VDD_CORE voltage
 */
#define TEGRA194_CLK_KFUSE			70
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_MAUD */
#define TEGRA194_CLK_MAUD			71
/** @brief output of gate CLK_ENB_MIPI_CAL */
#define TEGRA194_CLK_MIPI_CAL			72
/**
 * @brief output of the divider
 * CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_CORE_PLL_FIXED
 */
#define TEGRA194_CLK_MPHY_CORE_PLL_FIXED	73
/** @brief output of gate CLK_ENB_MPHY_L0_RX_ANA */
#define TEGRA194_CLK_MPHY_L0_RX_ANA		74
/** @brief output of gate CLK_ENB_MPHY_L0_RX_LS_BIT */
#define TEGRA194_CLK_MPHY_L0_RX_LS_BIT		75
/**
 * @brief output of the divider
 * CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_L0_RX_LS_SYMB
 */
#define TEGRA194_CLK_MPHY_L0_RX_SYMB		76
/** @brief output of gate CLK_ENB_MPHY_L0_TX_LS_3XBIT */
#define TEGRA194_CLK_MPHY_L0_TX_LS_3XBIT	77
/**
 * @brief output of the divider
 * CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_L0_TX_LS_SYMB
 */
#define TEGRA194_CLK_MPHY_L0_TX_SYMB		78
/** @brief output of gate CLK_ENB_MPHY_L1_RX_ANA */
#define TEGRA194_CLK_MPHY_L1_RX_ANA		79
/**
 * @brief output of the divider
 * CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_TX_1MHZ_REF
 */
#define TEGRA194_CLK_MPHY_TX_1MHZ_REF		80
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVCSI */
#define TEGRA194_CLK_NVCSI			81
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVCSILP */
#define TEGRA194_CLK_NVCSILP			82
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVDEC */
#define TEGRA194_CLK_NVDEC			83
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAYHUB
 */
#define TEGRA194_CLK_NVDISPLAYHUB		84
/**
 * @brief output of the NVDISPLAY_DISP_CLK_SRC mux in
 * CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_DISP
 */
#define TEGRA194_CLK_NVDISPLAY_DISP		85
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P0
 */
#define TEGRA194_CLK_NVDISPLAY_P0		86
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P1
 */
#define TEGRA194_CLK_NVDISPLAY_P1		87
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P2
 */
#define TEGRA194_CLK_NVDISPLAY_P2		88
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_NVENC
 */
#define TEGRA194_CLK_NVENC			89
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVJPG */
#define TEGRA194_CLK_NVJPG			90
/** @brief input from Tegra's XTAL_IN */
#define TEGRA194_CLK_OSC			91

#define TEGRA194_CLK_RSVD_92			92

/** PLL controlled by CLK_RST_CONTROLLER_PLLA_BASE for use by audio clocks */
#define TEGRA194_CLK_PLLA			93
/**
 * @brief PLL controlled by CLK_RST_CONTROLLER_PLLAON_BASE
 * for use by IP blocks in the AON domain
 */
#define TEGRA194_CLK_PLLAON			94
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLD_BASE for use by DSI */
#define TEGRA194_CLK_PLLD			95
/**
 * @brief PLL controlled by
 * CLK_RST_CONTROLLER_PLLD2_BASE for use by HDMI or DP
 */
#define TEGRA194_CLK_PLLD2			96
/**
 * @brief PLL controlled by
 * CLK_RST_CONTROLLER_PLLD3_BASE for use by HDMI or DP
 */
#define TEGRA194_CLK_PLLD3			97
/**
 * @brief PLL controlled by CLK_RST_CONTROLLER_PLLDP_BASE
 * for use as the DP link clock
 */
#define TEGRA194_CLK_PLLDP			98
#define TEGRA194_CLK_RSVD_99			99
/** Fixed 100MHz PLL for PCIe, SATA and superspeed USB */
#define TEGRA194_CLK_PLLE			100
/** @brief output of the divider PLLP_DIVP in CLK_RST_CONTROLLER_PLLP_BASE */
#define TEGRA194_CLK_PLLP			101
/** @brief output of the divider PLLP_DIVP in CLK_RST_CONTROLLER_PLLP_BASE */
#define TEGRA194_CLK_PLLP_OUT0			102
/** Fixed frequency 960MHz PLL for USB and EAVB */
#define TEGRA194_CLK_UTMIPLL			103
/** @brief output of the divider CLK_RST_CONTROLLER_PLLA_OUT */
#define TEGRA194_CLK_PLLA_OUT0			104
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM1 */
#define TEGRA194_CLK_PWM1			105
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM2 */
#define TEGRA194_CLK_PWM2			106
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM3 */
#define TEGRA194_CLK_PWM3			107
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM4 */
#define TEGRA194_CLK_PWM4			108
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM5 */
#define TEGRA194_CLK_PWM5			109
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM6 */
#define TEGRA194_CLK_PWM6			110
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM7 */
#define TEGRA194_CLK_PWM7			111
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM8 */
#define TEGRA194_CLK_PWM8			112

#define TEGRA194_CLK_RSVD_113			113
#define TEGRA194_CLK_RSVD_114			114

/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SATA */
#define TEGRA194_CLK_SATA			115
/** @brief output of gate CLK_ENB_SATA_OOB */
#define TEGRA194_CLK_SATA_OOB			116

#define TEGRA194_CLK_RSVD_117			117

/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_SCE_CPU_NIC
 */
#define TEGRA194_CLK_SCE_CPU_NIC		118
/** @brief output of divider CLK_RST_CONTROLLER_SCE_NIC_RATE */
#define TEGRA194_CLK_SCE_NIC			119
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1 */
#define TEGRA194_CLK_SDMMC1			120

/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC2 */
#define TEGRA194_CLK_RSVD_121			121

/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC3 */
#define TEGRA194_CLK_SDMMC3			122
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4 */
#define TEGRA194_CLK_SDMMC4			123
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SE */
#define TEGRA194_CLK_SE				124
/**
 * @brief output of the SOR0_CLK_SRC mux in
 * CLK_RST_CONTROLLER_CLK_SOURCE_SOR0
 */
#define TEGRA194_CLK_SOR0			125
/**
 * @brief output of mux controlled by SOR0_CLK_SEL0 and
 * SOR0_CLK_SEL1 in CLK_RST_CONTROLLER_CLK_SOURCE_SOR0
 */
#define TEGRA194_CLK_SOR0_OUT			126
/**
 * @brief SOR0 brick output which feeds into
 * SOR0_CLK_SEL mux in CLK_RST_CONTROLLER_CLK_SOURCE_SOR0
 */
#define TEGRA194_CLK_SOR0_PAD_CLKOUT		127
/**
 * @brief output of the SOR1_CLK_SRC mux in
 * CLK_RST_CONTROLLER_CLK_SOURCE_SOR1
 */
#define TEGRA194_CLK_SOR1			128
/**
 * @brief output of mux controlled by SOR1_CLK_SEL0
 * and SOR1_CLK_SEL1 in CLK_RST_CONTROLLER_CLK_SOURCE_SOR1
 */
#define TEGRA194_CLK_SOR1_OUT			129
/**
 * @brief SOR1 brick output which feeds into SOR1_CLK_SEL
 * mux in CLK_RST_CONTROLLER_CLK_SOURCE_SOR1
 */
#define TEGRA194_CLK_SOR1_PAD_CLKOUT		130
/** @brief output of gate CLK_ENB_SOR_SAFE */
#define TEGRA194_CLK_SOR_SAFE			131
/** @brief output of gate CLK_ENB_SPDIF_DOUBLER */
#define TEGRA194_CLK_SPDIF_DOUBLER		132
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SPDF_IN */
#define TEGRA194_CLK_SPDIF_IN			133
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_SPDIF_OUT
 */
#define TEGRA194_CLK_SPDIF_OUT			134
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SPI1 */
#define TEGRA194_CLK_SPI1			135
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SPI2 */
#define TEGRA194_CLK_SPI2			136
/**  @clkdesc{spi_clks, out, mux, CLK_RST_CONTROLLER_CLK_SOURCE_SPI3} */
#define TEGRA194_CLK_SPI3			137

#define TEGRA194_CLK_RSVD_138			138

/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC1
 */
#define TEGRA194_CLK_SYNC_DMIC1			139
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC2
 */
#define TEGRA194_CLK_SYNC_DMIC2			140
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC3
 */
#define TEGRA194_CLK_SYNC_DMIC3			141
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC4
 */
#define TEGRA194_CLK_SYNC_DMIC4			142
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DSPK1
 */
#define TEGRA194_CLK_SYNC_DSPK1			143
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DSPK2
 */
#define TEGRA194_CLK_SYNC_DSPK2			144
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S1 */
#define TEGRA194_CLK_SYNC_I2S1			145
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S2 */
#define TEGRA194_CLK_SYNC_I2S2			146
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S3 */
#define TEGRA194_CLK_SYNC_I2S3			147
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S4 */
#define TEGRA194_CLK_SYNC_I2S4			148
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S5 */
#define TEGRA194_CLK_SYNC_I2S5			149
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S6 */
#define TEGRA194_CLK_SYNC_I2S6			150
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_SPDIF
 */
#define TEGRA194_CLK_SYNC_SPDIF			151
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_TACH */
#define TEGRA194_CLK_TACH			152
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_TSEC */
#define TEGRA194_CLK_TSEC			153
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_TSECB */
#define TEGRA194_CLK_TSECB			154
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTA */
#define TEGRA194_CLK_UARTA			155
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTB */
#define TEGRA194_CLK_UARTB			156
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTC */
#define TEGRA194_CLK_UARTC			157
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTD */
#define TEGRA194_CLK_UARTD			158
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTE */
#define TEGRA194_CLK_UARTE			159
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTF */
#define TEGRA194_CLK_UARTF			160
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTG */
#define TEGRA194_CLK_UARTG			161
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_UART_FST_MIPI_CAL
 */
#define TEGRA194_CLK_UART_FST_MIPI_CAL		162
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_UFSDEV_REF
 */
#define TEGRA194_CLK_UFSDEV_REF			163
/**
 * @brief output of mux controlled by
 * CLK_RST_CONTROLLER_CLK_SOURCE_UFSHC_CG_SYS
 */
#define TEGRA194_CLK_UFSHC			164
/** @brief output of gate CLK_ENB_USB2_TRK */
#define TEGRA194_CLK_USB2_TRK			165
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_VI */
#define TEGRA194_CLK_VI				166
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_VIC */
#define TEGRA194_CLK_VIC			167
/** @brief PVA0_axi_clk */
#define TEGRA194_CLK_PVA0_AXI			168
/** @brief PVA0_vps0_clk */
#define TEGRA194_CLK_PVA0_VPS0			169
/** @brief PVA0_vps1_clk */
#define TEGRA194_CLK_PVA0_VPS1			170
/** @brief PVA1_axi_clk */
#define TEGRA194_CLK_PVA1_AXI			171
/** @brief PVA1_vps0_clk */
#define TEGRA194_CLK_PVA1_VPS0			172
/** @brief PVA1_vps1_clk */
#define TEGRA194_CLK_PVA1_VPS1			173
/** @brief DLA0_falcon_clk */
#define TEGRA194_CLK_DLA0_FALCON		174
/** @brief DLA0_core_clk */
#define TEGRA194_CLK_DLA0_CORE			175
/** @brief DLA1_falcon_clk */
#define TEGRA194_CLK_DLA1_FALCON		176
/** @brief DLA1_core_clk */
#define TEGRA194_CLK_DLA1_CORE			177

#endif
