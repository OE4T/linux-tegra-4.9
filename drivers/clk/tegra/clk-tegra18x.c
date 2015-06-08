/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clk/tegra.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/tegra-soc.h>
#include <dt-bindings/clock/tegra186-clock.h>

#include "clk.h"

static __initdata struct tegra_bpmp_clk_init tegra186_clocks[] = {
	{ .clk_num = TEGRA186_CLK_FUSE, .name = "fuse" },
	{ .clk_num = TEGRA186_CLK_GPU, .name = "gpu" },
	{ .clk_num = TEGRA186_CLK_PLLG_REF, .name = "pllg_ref" },
	{ .clk_num = TEGRA186_CLK_PCIE, .name = "pcie" },
	{ .clk_num = TEGRA186_CLK_AFI, .name = "afi" },
	{ .clk_num = TEGRA186_CLK_PCIE2_IOBIST, .name = "pcie2_iobist" },
	{ .clk_num = TEGRA186_CLK_PCIERX0, .name = "pcierx0" },
	{ .clk_num = TEGRA186_CLK_PCIERX1, .name = "pcierx1" },
	{ .clk_num = TEGRA186_CLK_PCIERX2, .name = "pcierx2" },
	{ .clk_num = TEGRA186_CLK_PCIERX3, .name = "pcierx3" },
	{ .clk_num = TEGRA186_CLK_PCIERX4, .name = "pcierx4" },
	{ .clk_num = TEGRA186_CLK_PLLC_OUT_ISP, .name = "pllc_out_isp" },
	{ .clk_num = TEGRA186_CLK_PLLC_OUT_VE, .name = "pllc_out_ve" },
	{ .clk_num = TEGRA186_CLK_PLLC_OUT_AON, .name = "pllc_out_aon" },
	{ .clk_num = TEGRA186_CLK_PLLP_OUT_CPU, .name = "pllp_out_cpu" },
	{ .clk_num = TEGRA186_CLK_PLLP_PLL_REF, .name = "pllp_pll_ref" },
	{ .clk_num = TEGRA186_CLK_PLLP_XUSB, .name = "pllp_xusb" },
	{ .clk_num = TEGRA186_CLK_PLLP_VIC, .name = "pllp_vic" },
	{ .clk_num = TEGRA186_CLK_PLLP_NVENC, .name = "pllp_nvenc" },
	{ .clk_num = TEGRA186_CLK_PLLP_NVDEC, .name = "pllp_nvdec" },
	{ .clk_num = TEGRA186_CLK_PLLP_MC1, .name = "pllp_mc1" },
	{ .clk_num = TEGRA186_CLK_PLLP_MC0, .name = "pllp_mc0" },
	{ .clk_num = TEGRA186_CLK_PLLP_ISP, .name = "pllp_isp" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTTB, .name = "pllp_grttb" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTTA, .name = "pllp_grtta" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTLB, .name = "pllp_grtlb" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTLA, .name = "pllp_grtla" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTCBB, .name = "pllp_grtcbb" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTCBA, .name = "pllp_grtcba" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTCAB, .name = "pllp_grtcab" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTCAA, .name = "pllp_grtcaa" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTBB, .name = "pllp_grtbb" },
	{ .clk_num = TEGRA186_CLK_PLLP_GRTBA, .name = "pllp_grtba" },
	{ .clk_num = TEGRA186_CLK_PLLP_DISPLAY, .name = "pllp_display" },
	{ .clk_num = TEGRA186_CLK_PLLP_BOOT, .name = "pllp_boot" },
	{ .clk_num = TEGRA186_CLK_PLLP_BB, .name = "pllp_bb" },
	{ .clk_num = TEGRA186_CLK_PLLP_AUDIO, .name = "pllp_audio" },
	{ .clk_num = TEGRA186_CLK_PLLP_AON, .name = "pllp_aon" },
	{ .clk_num = TEGRA186_CLK_PLLP_UPHY, .name = "pllp_uphy" },
	{ .clk_num = TEGRA186_CLK_SOR_SAFE, .name = "sor_safe" },
	{ .clk_num = TEGRA186_CLK_PLLP_CAR, .name = "pllp_car" },
	{ .clk_num = TEGRA186_CLK_PLLD_OUT_CPU, .name = "plld_out_cpu" },
	{ .clk_num = TEGRA186_CLK_I2S2, .name = "i2s2" },
	{ .clk_num = TEGRA186_CLK_I2S3, .name = "i2s3" },
	{ .clk_num = TEGRA186_CLK_SPDIF_IN, .name = "spdif_in" },
	{ .clk_num = TEGRA186_CLK_SPDIF_DOUBLER, .name = "spdif_doubler" },
	{ .clk_num = TEGRA186_CLK_SPI3, .name = "spi3" },
	{ .clk_num = TEGRA186_CLK_I2C1, .name = "i2c1" },
	{ .clk_num = TEGRA186_CLK_I2C5, .name = "i2c5" },
	{ .clk_num = TEGRA186_CLK_SPI1, .name = "spi1" },
	{ .clk_num = TEGRA186_CLK_ISP, .name = "isp" },
	{ .clk_num = TEGRA186_CLK_VI, .name = "vi" },
	{ .clk_num = TEGRA186_CLK_SDMMC1, .name = "sdmmc1" },
	{ .clk_num = TEGRA186_CLK_SDMMC2, .name = "sdmmc2" },
	{ .clk_num = TEGRA186_CLK_SDMMC4, .name = "sdmmc4" },
	{ .clk_num = TEGRA186_CLK_UARTA, .name = "uarta" },
	{ .clk_num = TEGRA186_CLK_UARTB, .name = "uartb" },
	{ .clk_num = TEGRA186_CLK_HOST1X, .name = "host1x" },
	{ .clk_num = TEGRA186_CLK_EMC, .name = "emc" },
	{ .clk_num = TEGRA186_CLK_MEM, .name = "mem" },
	{ .clk_num = TEGRA186_CLK_EMC_DLL, .name = "emc_dll" },
	{ .clk_num = TEGRA186_CLK_MC_CAPA, .name = "mc_capa" },
	{ .clk_num = TEGRA186_CLK_MC_CBPA, .name = "mc_cbpa" },
	{ .clk_num = TEGRA186_CLK_MC_CPU, .name = "mc_cpu" },
	{ .clk_num = TEGRA186_CLK_MC_BBC, .name = "mc_bbc" },
	{ .clk_num = TEGRA186_CLK_EMC_LATENCY, .name = "emc_latency" },
	{ .clk_num = TEGRA186_CLK_EMC_IOBIST, .name = "emc_iobist" },
	{ .clk_num = TEGRA186_CLK_MC1, .name = "mc1" },
	{ .clk_num = TEGRA186_CLK_MCHUBSA, .name = "mchubsa" },
	{ .clk_num = TEGRA186_CLK_MC_CCPA, .name = "mc_ccpa" },
	{ .clk_num = TEGRA186_CLK_MC_CDPA, .name = "mc_cdpa" },
	{ .clk_num = TEGRA186_CLK_MC_CEPA, .name = "mc_cepa" },
	{ .clk_num = TEGRA186_CLK_MC_CFPA, .name = "mc_cfpa" },
	{ .clk_num = TEGRA186_CLK_EXTPERIPH4, .name = "extperiph4" },
	{ .clk_num = TEGRA186_CLK_SPI4, .name = "spi4" },
	{ .clk_num = TEGRA186_CLK_I2C3, .name = "i2c3" },
	{ .clk_num = TEGRA186_CLK_SDMMC3, .name = "sdmmc3" },
	{ .clk_num = TEGRA186_CLK_UARTD, .name = "uartd" },
	{ .clk_num = TEGRA186_CLK_CSITE, .name = "csite" },
	{ .clk_num = TEGRA186_CLK_I2S1, .name = "i2s1" },
	{ .clk_num = TEGRA186_CLK_DTV, .name = "dtv" },
	{ .clk_num = TEGRA186_CLK_TSEC, .name = "tsec" },
	{ .clk_num = TEGRA186_CLK_DP2, .name = "dp2" },
	{ .clk_num = TEGRA186_CLK_LA, .name = "la" },
	{ .clk_num = TEGRA186_CLK_I2S4, .name = "i2s4" },
	{ .clk_num = TEGRA186_CLK_I2S5, .name = "i2s5" },
	{ .clk_num = TEGRA186_CLK_I2C4, .name = "i2c4" },
	{ .clk_num = TEGRA186_CLK_AHUB, .name = "ahub" },
	{ .clk_num = TEGRA186_CLK_HDA2CODEC_2X, .name = "hda2codec_2x" },
	{ .clk_num = TEGRA186_CLK_EXTPERIPH1, .name = "extperiph1" },
	{ .clk_num = TEGRA186_CLK_EXTPERIPH2, .name = "extperiph2" },
	{ .clk_num = TEGRA186_CLK_EXTPERIPH3, .name = "extperiph3" },
	{ .clk_num = TEGRA186_CLK_I2C_SLOW, .name = "i2c_slow" },
	{ .clk_num = TEGRA186_CLK_SOR1, .name = "sor1" },
	{ .clk_num = TEGRA186_CLK_CEC, .name = "cec" },
	{ .clk_num = TEGRA186_CLK_DPAUX1, .name = "dpaux1" },
	{ .clk_num = TEGRA186_CLK_DPAUX, .name = "dpaux" },
	{ .clk_num = TEGRA186_CLK_SOR0, .name = "sor0" },
	{ .clk_num = TEGRA186_CLK_HDA2HDMICODEC, .name = "hda2hdmicodec" },
	{ .clk_num = TEGRA186_CLK_SATA, .name = "sata" },
	{ .clk_num = TEGRA186_CLK_SATA_OOB, .name = "sata_oob" },
	{ .clk_num = TEGRA186_CLK_SATA_IOBIST, .name = "sata_iobist" },
	{ .clk_num = TEGRA186_CLK_HDA, .name = "hda" },
	{ .clk_num = TEGRA186_CLK_SE, .name = "se" },
	{ .clk_num = TEGRA186_CLK_APB2APE, .name = "apb2ape" },
	{ .clk_num = TEGRA186_CLK_APE, .name = "ape" },
	{ .clk_num = TEGRA186_CLK_IQC1, .name = "iqc1" },
	{ .clk_num = TEGRA186_CLK_IQC2, .name = "iqc2" },
	{ .clk_num = TEGRA186_CLK_PLLREFE_OUT, .name = "pllrefe_out" },
	{ .clk_num = TEGRA186_CLK_PLLREFE_PLL_REF, .name = "pllrefe_pll_ref" },
	{ .clk_num = TEGRA186_CLK_PLLC4_OUT, .name = "pllc4_out" },
	{ .clk_num = TEGRA186_CLK_XUSB, .name = "xusb" },
	{ .clk_num = TEGRA186_CLK_XUSB_DEV, .name = "xusb_dev" },
	{ .clk_num = TEGRA186_CLK_XUSB_HOST, .name = "xusb_host" },
	{ .clk_num = TEGRA186_CLK_XUSB_SS, .name = "xusb_ss" },
	{ .clk_num = TEGRA186_CLK_DSI, .name = "dsi" },
	{ .clk_num = TEGRA186_CLK_MIPI_CAL, .name = "mipi_cal" },
	{ .clk_num = TEGRA186_CLK_DSIA_LP, .name = "dsia_lp" },
	{ .clk_num = TEGRA186_CLK_DSIB, .name = "dsib" },
	{ .clk_num = TEGRA186_CLK_DSIB_LP, .name = "dsib_lp" },
	{ .clk_num = TEGRA186_CLK_ENTROPY, .name = "entropy" },
	{ .clk_num = TEGRA186_CLK_DVFS, .name = "dvfs" },
	{ .clk_num = TEGRA186_CLK_DMIC1, .name = "dmic1" },
	{ .clk_num = TEGRA186_CLK_DMIC2, .name = "dmic2" },
	{ .clk_num = TEGRA186_CLK_AUD_MCLK, .name = "aud_mclk" },
	{ .clk_num = TEGRA186_CLK_I2C6, .name = "i2c6" },
	{ .clk_num = TEGRA186_CLK_UART_FST_MIPI_CAL, .name = "uart_fst_mipi_cal" },
	{ .clk_num = TEGRA186_CLK_VIC, .name = "vic" },
	{ .clk_num = TEGRA186_CLK_SDMMC_LEGACY_TM, .name = "sdmmc_legacy_tm" },
	{ .clk_num = TEGRA186_CLK_NVDEC, .name = "nvdec" },
	{ .clk_num = TEGRA186_CLK_NVJPG, .name = "nvjpg" },
	{ .clk_num = TEGRA186_CLK_NVENC, .name = "nvenc" },
	{ .clk_num = TEGRA186_CLK_QSPI, .name = "qspi" },
	{ .clk_num = TEGRA186_CLK_VI_I2C, .name = "vi_i2c" },
	{ .clk_num = TEGRA186_CLK_HSIC_TRK, .name = "hsic_trk" },
	{ .clk_num = TEGRA186_CLK_USB2_TRK, .name = "usb2_trk" },
	{ .clk_num = TEGRA186_CLK_MAUD, .name = "maud" },
	{ .clk_num = TEGRA186_CLK_TSECB, .name = "tsecb" },
	{ .clk_num = TEGRA186_CLK_ADSP, .name = "adsp" },
	{ .clk_num = TEGRA186_CLK_ADSPNEON, .name = "adspneon" },
	{ .clk_num = TEGRA186_CLK_MPHY_L0_RX_SYMB, .name = "mphy_l0_rx_symb" },
	{ .clk_num = TEGRA186_CLK_MPHY_L0_RX_LS_BIT, .name = "mphy_l0_rx_ls_bit" },
	{ .clk_num = TEGRA186_CLK_MPHY_L0_TX_SYMB, .name = "mphy_l0_tx_symb" },
	{ .clk_num = TEGRA186_CLK_MPHY_L0_TX_LS_3XBIT, .name = "mphy_l0_tx_ls_3xbit" },
	{ .clk_num = TEGRA186_CLK_MPHY_L0_RX_ANA, .name = "mphy_l0_rx_ana" },
	{ .clk_num = TEGRA186_CLK_MPHY_L1_RX_ANA, .name = "mphy_l1_rx_ana" },
	{ .clk_num = TEGRA186_CLK_MPHY_IOBIST, .name = "mphy_iobist" },
	{ .clk_num = TEGRA186_CLK_MPHY_TX_1MHZ_REF, .name = "mphy_tx_1mhz_ref" },
	{ .clk_num = TEGRA186_CLK_MPHY_CORE_PLL_FIXED, .name = "mphy_core_pll_fixed" },
	{ .clk_num = TEGRA186_CLK_AXI_CBB, .name = "axi_cbb" },
	{ .clk_num = TEGRA186_CLK_DMIC3, .name = "dmic3" },
	{ .clk_num = TEGRA186_CLK_DMIC4, .name = "dmic4" },
	{ .clk_num = TEGRA186_CLK_DSPK1, .name = "dspk1" },
	{ .clk_num = TEGRA186_CLK_DSPK2, .name = "dspk2" },
	{ .clk_num = TEGRA186_CLK_I2S6, .name = "i2s6" },
	{ .clk_num = TEGRA186_CLK_NVDISPLAY_P0, .name = "nvdisplay_p0" },
	{ .clk_num = TEGRA186_CLK_NVDISPLAY_DISP, .name = "nvdisplay_disp" },
	{ .clk_num = TEGRA186_CLK_NVDISPLAY_DSC, .name = "nvdisplay_dsc" },
	{ .clk_num = TEGRA186_CLK_NVDISPLAYHUB, .name = "nvdisplayhub" },
	{ .clk_num = TEGRA186_CLK_NVDISPLAY_P1, .name = "nvdisplay_p1" },
	{ .clk_num = TEGRA186_CLK_NVDISPLAY_P2, .name = "nvdisplay_p2" },
	{ .clk_num = TEGRA186_CLK_GPIO_CTL0, .name = "gpio_ctl0" },
	{ .clk_num = TEGRA186_CLK_GPIO_CTL1, .name = "gpio_ctl1" },
	{ .clk_num = TEGRA186_CLK_GPIO_CTL2, .name = "gpio_ctl2" },
	{ .clk_num = TEGRA186_CLK_GPIO_CTL3, .name = "gpio_ctl3" },
	{ .clk_num = TEGRA186_CLK_GPIO_CTL4, .name = "gpio_ctl4" },
	{ .clk_num = TEGRA186_CLK_TACH, .name = "tach" },
	{ .clk_num = TEGRA186_CLK_EQOS_AXI, .name = "eqos_axi" },
	{ .clk_num = TEGRA186_CLK_EQOS_RX, .name = "eqos_rx" },
	{ .clk_num = TEGRA186_CLK_EMCSB, .name = "emcsb" },
	{ .clk_num = TEGRA186_CLK_MEMB, .name = "memb" },
	{ .clk_num = TEGRA186_CLK_EMCSB_DLL, .name = "emcsb_dll" },
	{ .clk_num = TEGRA186_CLK_MCB_CPU, .name = "mcb_cpu" },
	{ .clk_num = TEGRA186_CLK_MCB_BBC, .name = "mcb_bbc" },
	{ .clk_num = TEGRA186_CLK_EMCSB_LATENCY, .name = "emcsb_latency" },
	{ .clk_num = TEGRA186_CLK_EMCSB_IOBIST, .name = "emcsb_iobist" },
	{ .clk_num = TEGRA186_CLK_MC3, .name = "mc3" },
	{ .clk_num = TEGRA186_CLK_MCHUBSB, .name = "mchubsb" },
	{ .clk_num = TEGRA186_CLK_UFSHC, .name = "ufshc" },
	{ .clk_num = TEGRA186_CLK_UFSDEV_REF, .name = "ufsdev_ref" },
	{ .clk_num = TEGRA186_CLK_NVCSI, .name = "nvcsi" },
	{ .clk_num = TEGRA186_CLK_NVCSILP, .name = "nvcsilp" },
	{ .clk_num = TEGRA186_CLK_I2C7, .name = "i2c7" },
	{ .clk_num = TEGRA186_CLK_I2C9, .name = "i2c9" },
	{ .clk_num = TEGRA186_CLK_I2C12, .name = "i2c12" },
	{ .clk_num = TEGRA186_CLK_I2C13, .name = "i2c13" },
	{ .clk_num = TEGRA186_CLK_I2C14, .name = "i2c14" },
	{ .clk_num = TEGRA186_CLK_PWM1, .name = "pwm1" },
	{ .clk_num = TEGRA186_CLK_PWM2, .name = "pwm2" },
	{ .clk_num = TEGRA186_CLK_PWM3, .name = "pwm3" },
	{ .clk_num = TEGRA186_CLK_PWM5, .name = "pwm5" },
	{ .clk_num = TEGRA186_CLK_PWM6, .name = "pwm6" },
	{ .clk_num = TEGRA186_CLK_PWM7, .name = "pwm7" },
	{ .clk_num = TEGRA186_CLK_PWM8, .name = "pwm8" },
	{ .clk_num = TEGRA186_CLK_UARTE, .name = "uarte" },
	{ .clk_num = TEGRA186_CLK_UARTF, .name = "uartf" },
	{ .clk_num = TEGRA186_CLK_DBGAPB, .name = "dbgapb" },
	{ .clk_num = TEGRA186_CLK_BPMP_CPU_NIC, .name = "bpmp_cpu_nic" },
	{ .clk_num = TEGRA186_CLK_BPMP_CPU, .name = "bpmp_cpu" },
	{ .clk_num = TEGRA186_CLK_BPMP_APB, .name = "bpmp_apb" },
	{ .clk_num = TEGRA186_CLK_SOC_THERM, .name = "soc_therm" },
	{ .clk_num = TEGRA186_CLK_ACTMON, .name = "actmon" },
	{ .clk_num = TEGRA186_CLK_TSENSOR, .name = "tsensor" },
	{ .clk_num = TEGRA186_CLK_SIMON, .name = "simon" },
	{ .clk_num = TEGRA186_CLK_SIMON_SOC, .name = "simon_soc" },
	{ .clk_num = TEGRA186_CLK_SIMON_LCPU, .name = "simon_lcpu" },
	{ .clk_num = TEGRA186_CLK_SIMON_BCPU, .name = "simon_bcpu" },
	{ .clk_num = TEGRA186_CLK_SIMON_GPU, .name = "simon_gpu" },
	{ .clk_num = TEGRA186_CLK_AON_CPU_NIC, .name = "aon_cpu_nic" },
	{ .clk_num = TEGRA186_CLK_AON_CPU, .name = "aon_cpu" },
	{ .clk_num = TEGRA186_CLK_CAN1, .name = "can1" },
	{ .clk_num = TEGRA186_CLK_CAN1_HOST, .name = "can1_host" },
	{ .clk_num = TEGRA186_CLK_CAN2, .name = "can2" },
	{ .clk_num = TEGRA186_CLK_CAN2_HOST, .name = "can2_host" },
	{ .clk_num = TEGRA186_CLK_AON_APB, .name = "aon_apb" },
	{ .clk_num = TEGRA186_CLK_UARTC, .name = "uartc" },
	{ .clk_num = TEGRA186_CLK_UARTG, .name = "uartg" },
	{ .clk_num = TEGRA186_CLK_AON_UART_FST_MIPI_CAL, .name = "aon_uart_fst_mipi_cal" },
	{ .clk_num = TEGRA186_CLK_I2C2, .name = "i2c2" },
	{ .clk_num = TEGRA186_CLK_I2C8, .name = "i2c8" },
	{ .clk_num = TEGRA186_CLK_I2C10, .name = "i2c10" },
	{ .clk_num = TEGRA186_CLK_AON_I2C_SLOW, .name = "aon_i2c_slow" },
	{ .clk_num = TEGRA186_CLK_SPI2, .name = "spi2" },
	{ .clk_num = TEGRA186_CLK_DMIC5, .name = "dmic5" },
	{ .clk_num = TEGRA186_CLK_AON_TOUCH, .name = "aon_touch" },
	{ .clk_num = TEGRA186_CLK_PWM4, .name = "pwm4" },
	{ .clk_num = TEGRA186_CLK_TSC, .name = "tsc" },
	{ .clk_num = TEGRA186_CLK_MSS_ENCRYPT, .name = "mss_encrypt" },
	{ .clk_num = TEGRA186_CLK_SCE_CPU_NIC, .name = "sce_cpu_nic" },
	{ .clk_num = TEGRA186_CLK_SCE_CPU, .name = "sce_cpu" },
	{ .clk_num = TEGRA186_CLK_SCE_APB, .name = "sce_apb" },
	{ .clk_num = TEGRA186_CLK_DSIC, .name = "dsic" },
	{ .clk_num = TEGRA186_CLK_DSIC_LP, .name = "dsic_lp" },
	{ .clk_num = TEGRA186_CLK_DSID, .name = "dsid" },
	{ .clk_num = TEGRA186_CLK_DSID_LP, .name = "dsid_lp" },
	{ .clk_num = TEGRA186_CLK_GPIO_CTL5, .name = "gpio_ctl5" },
	{ .clk_num = TEGRA186_CLK_PEX_SATA_USB_RX_BYP, .name = "pex_sata_usb_rx_byp" },
	{ .clk_num = TEGRA186_CLK_IPFS, .name = "ipfs" },
	{ .clk_num = TEGRA186_CLK_SPDIF_OUT, .name = "spdif_out" },
	{ .clk_num = TEGRA186_CLK_EQOS_PTP_REF, .name = "eqos_ptp_ref" },
	{ .clk_num = TEGRA186_CLK_EQOS_TX, .name = "eqos_tx" },
	{ .clk_num = TEGRA186_CLK_USB2_HSIC_TRK, .name = "usb2_hsic_trk" },
	{ .clk_num = TEGRA186_CLK_XUSB_CORE_SS, .name = "xusb_core_ss" },
	{ .clk_num = TEGRA186_CLK_XUSB_CORE_DEV, .name = "xusb_core_dev" },
	{ .clk_num = TEGRA186_CLK_XUSB_FALCON, .name = "xusb_falcon" },
	{ .clk_num = TEGRA186_CLK_XUSB_FS, .name = "xusb_fs" },
	{ .clk_num = TEGRA186_CLK_PLL_A_OUT0, .name = "pll_a_out0" },
	{ .clk_num = TEGRA186_CLK_PLL_A_OUT1, .name = "pll_a_out1" },
	{ .clk_num = TEGRA186_CLK_SYNC_I2S1, .name = "sync_i2s1" },
	{ .clk_num = TEGRA186_CLK_SYNC_I2S2, .name = "sync_i2s2" },
	{ .clk_num = TEGRA186_CLK_SYNC_I2S3, .name = "sync_i2s3" },
	{ .clk_num = TEGRA186_CLK_SYNC_I2S4, .name = "sync_i2s4" },
	{ .clk_num = TEGRA186_CLK_SYNC_I2S5, .name = "sync_i2s5" },
	{ .clk_num = TEGRA186_CLK_SYNC_I2S6, .name = "sync_i2s6" },
	{ .clk_num = TEGRA186_CLK_SYNC_DSPK1, .name = "sync_dspk1" },
	{ .clk_num = TEGRA186_CLK_SYNC_DSPK2, .name = "sync_dspk2" },
	{ .clk_num = TEGRA186_CLK_SYNC_DMIC1, .name = "sync_dmic1" },
	{ .clk_num = TEGRA186_CLK_SYNC_DMIC2, .name = "sync_dmic2" },
	{ .clk_num = TEGRA186_CLK_SYNC_DMIC3, .name = "sync_dmic3" },
	{ .clk_num = TEGRA186_CLK_SYNC_DMIC4, .name = "sync_dmic4" },
	{ .clk_num = TEGRA186_CLK_SYNC_SPDIF, .name = "sync_spdif" },
	{ .clk_num = TEGRA186_CLK_PLLREFE_OUT_GATED, .name = "pllrefe_out_gated" },
	{ .clk_num = TEGRA186_CLK_PLLREFE_OUT1, .name = "pllrefe_out1" },
	{ .clk_num = TEGRA186_CLK_PLLM_UD, .name = "pllm_ud" },
	{ .clk_num = TEGRA186_CLK_PLLMB_UD, .name = "pllmb_ud" },
	{ .clk_num = TEGRA186_CLK_PLLP_UD, .name = "pllp_ud" },
	{ .clk_num = TEGRA186_CLK_PLLC_UD, .name = "pllc_ud" },
	{ .clk_num = TEGRA186_CLK_PLLD_OUT1, .name = "plld_out1" },
	{ .clk_num = TEGRA186_CLK_PLLD_OUT2, .name = "plld_out2" },
	{ .clk_num = TEGRA186_CLK_PLLP_OUT0, .name = "pllp_out0" },
	{ .clk_num = TEGRA186_CLK_PLLP_OUT5, .name = "pllp_out5" },
	{ .clk_num = TEGRA186_CLK_PLLA, .name = "plla" },
	{ .clk_num = TEGRA186_CLK_JTAG_TCK_IB, .name = "jtag_tck_ib" },
	{ .clk_num = TEGRA186_CLK_ACLK, .name = "aclk" },
	{ .clk_num = TEGRA186_CLK_PLL_U_48M, .name = "pll_u_48m" },
	{ .clk_num = TEGRA186_CLK_PLL_U_480M, .name = "pll_u_480m" },
	{ .clk_num = TEGRA186_CLK_PLLC4_OUT0, .name = "pllc4_out0" },
	{ .clk_num = TEGRA186_CLK_PLLC4_OUT1, .name = "pllc4_out1" },
	{ .clk_num = TEGRA186_CLK_PLLC4_OUT2, .name = "pllc4_out2" },
	{ .clk_num = TEGRA186_CLK_PLLC4_OUT_MUX, .name = "pllc4_out_mux" },
	{ .clk_num = TEGRA186_CLK_EMC_SAFE, .name = "emc_safe" },
	{ .clk_num = TEGRA186_CLK_EMCSB_SAFE, .name = "emcsb_safe" },
	{ .clk_num = TEGRA186_CLK_MC, .name = "mc" },
	{ .clk_num = TEGRA186_CLK_MCSB, .name = "mcsb" },
	{ .clk_num = TEGRA186_CLK_PLLE, .name = "plle" },
	{ .clk_num = TEGRA186_CLK_PLLC, .name = "pllc" },
	{ .clk_num = TEGRA186_CLK_PLLM, .name = "pllm" },
	{ .clk_num = TEGRA186_CLK_PLLMB, .name = "pllmb" },
	{ .clk_num = TEGRA186_CLK_PLLP, .name = "pllp" },
	{ .clk_num = TEGRA186_CLK_PLLA_VCO, .name = "plla_vco" },
	{ .clk_num = TEGRA186_CLK_PLLD, .name = "plld" },
	{ .clk_num = TEGRA186_CLK_PLLD2, .name = "plld2" },
	{ .clk_num = TEGRA186_CLK_PLLREFE_VCO, .name = "pllrefe_vco" },
	{ .clk_num = TEGRA186_CLK_PLLC2, .name = "pllc2" },
	{ .clk_num = TEGRA186_CLK_PLLC3, .name = "pllc3" },
	{ .clk_num = TEGRA186_CLK_PLLDP, .name = "plldp" },
	{ .clk_num = TEGRA186_CLK_PLLC4_VCO, .name = "pllc4_vco" },
	{ .clk_num = TEGRA186_CLK_PLLA1, .name = "plla1" },
	{ .clk_num = TEGRA186_CLK_PLLNVCSI, .name = "pllnvcsi" },
	{ .clk_num = TEGRA186_CLK_PLLDISPHUB, .name = "plldisphub" },
	{ .clk_num = TEGRA186_CLK_PLLD3, .name = "plld3" },
	{ .clk_num = TEGRA186_CLK_PLLMSB, .name = "pllmsb" },
	{ .clk_num = TEGRA186_CLK_PLLMSBB, .name = "pllmsbb" },
	{ .clk_num = TEGRA186_CLK_PLLBPMPCAM, .name = "pllbpmpcam" },
	{ .clk_num = TEGRA186_CLK_PLLAON, .name = "pllaon" },
	{ .clk_num = TEGRA186_CLK_PLLU, .name = "pllu" },
	{ .clk_num = TEGRA186_CLK_NAFLL_AXI_CBB, .name = "nafll_axi_cbb" },
	{ .clk_num = TEGRA186_CLK_NAFLL_BPMP, .name = "nafll_bpmp" },
	{ .clk_num = TEGRA186_CLK_NAFLL_ISP, .name = "nafll_isp" },
	{ .clk_num = TEGRA186_CLK_NAFLL_NVDEC, .name = "nafll_nvdec" },
	{ .clk_num = TEGRA186_CLK_NAFLL_NVENC, .name = "nafll_nvenc" },
	{ .clk_num = TEGRA186_CLK_NAFLL_NVJPG, .name = "nafll_nvjpg" },
	{ .clk_num = TEGRA186_CLK_NAFLL_SCE, .name = "nafll_sce" },
	{ .clk_num = TEGRA186_CLK_NAFLL_SE, .name = "nafll_se" },
	{ .clk_num = TEGRA186_CLK_NAFLL_TSEC, .name = "nafll_tsec" },
	{ .clk_num = TEGRA186_CLK_NAFLL_TSECB, .name = "nafll_tsecb" },
	{ .clk_num = TEGRA186_CLK_NAFLL_VI, .name = "nafll_vi" },
	{ .clk_num = TEGRA186_CLK_NAFLL_VIC, .name = "nafll_vic" },
	{ .clk_num = TEGRA186_CLK_NAFLL_DISP, .name = "nafll_disp" },
	{ .clk_num = TEGRA186_CLK_CLK_32K, .name = "clk_32k" },
	{ .clk_num = TEGRA186_CLK_CLK_M, .name = "clk_m" },
	{ .clk_num = TEGRA186_CLK_PLL_REF, .name = "pll_ref" },
	{ .clk_num = TEGRA186_CLK_EXT_VIMCLK, .name = "ext_vimclk" },
	{ .clk_num = TEGRA186_CLK_DFLLDISP_DIV, .name = "dflldisp_div" },
	{ .clk_num = TEGRA186_CLK_PLLDISPHUB_DIV, .name = "plldisphub_div" },
	{ .clk_num = TEGRA186_CLK_PLLP_DIV8, .name = "pllp_div8" },
};

/* Needed for a nvdisp linsim clock hack */
#define CLK_RST_CONTROLLER_RST_DEV_NVDISPLAY0_CLR_0 0x800008
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_NVDISPLAY0_SET_0 0x80100

void enable_gpio_clk(void)
{
	void __iomem *clk_base = ioremap(0x5840000, 0x21000);

	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x1000);
	udelay(2);

	__raw_writel(0x0, clk_base + 0x10000);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x11000);

	udelay(2);
	iounmap(clk_base);

	clk_base = ioremap(0x5860000, 0x21000);
	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 1000);
	udelay(2);

	__raw_writel(0x0, clk_base + 0x10000);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x11000);
	udelay(2);
	iounmap(clk_base);

	clk_base = ioremap(0x5880000, 0x21000);
	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 0x1000);
	udelay(2);
	iounmap(clk_base);

	clk_base = ioremap(0x5fc0000, 0x21000);

	__raw_writel(0x0, clk_base);
	udelay(2);
	__raw_writel(0x1, clk_base + 1000);
	udelay(2);
	iounmap(clk_base);
}

/* Needed for GPCDMA controller reset */
#define CLK_RST_CONTROLLER_RST_DEV_AXI_CBB_0 0x56A0000
void reset_gpcdma_controller(void)
{
	void __iomem *gpcdma_rst;

	gpcdma_rst = ioremap(CLK_RST_CONTROLLER_RST_DEV_AXI_CBB_0, 0x4);
	writel(0x6, gpcdma_rst);
	udelay(2);
	writel(0x4, gpcdma_rst);
}

static void __init tegra186_clock_init(struct device_node *np)
{
	int err;

	printk(KERN_INFO "Registering Tegra186 clocks (this may take a while)...");
	err = PTR_RET(tegra_bpmp_clk_init(tegra186_clocks,
				ARRAY_SIZE(tegra186_clocks), np));
	printk("done\n");

	if (err)
		pr_err("Failed to initialize Tegra186 clocks. err: %d\n", err);

	/* Nvdisp linsim clock hack */
	if (tegra_platform_is_linsim()) {
		void __iomem *base;

		base = of_iomap(np, 0);
		if (!base) {
			pr_err("ioremap Tegra186 CAR failed\n");
			return;
		}

		writel(0x3ff, base + CLK_RST_CONTROLLER_RST_DEV_NVDISPLAY0_CLR_0);
		writel(0xf, base + CLK_RST_CONTROLLER_CLK_OUT_ENB_NVDISPLAY0_SET_0);
	}
	enable_gpio_clk();
	reset_gpcdma_controller();
}

static const struct of_device_id tegra186_clock_ids[] __initconst = {
	{ .compatible = "nvidia,tegra18x-car",  .data = tegra186_clock_init},
	{},
};

static int __init tegra186_of_clk_init(void)
{
	of_clk_init(tegra186_clock_ids);

	return 0;
}

arch_initcall(tegra186_of_clk_init);
