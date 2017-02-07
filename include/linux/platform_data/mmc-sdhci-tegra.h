/*
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __PLATFORM_DATA_TEGRA_SDHCI_H
#define __PLATFORM_DATA_TEGRA_SDHCI_H

#include <linux/mmc/host.h>

/*
 * MMC_OCR_1V8_MASK will be used in board sdhci file
 * Example for cardhu it will be used in board-cardhu-sdhci.c
 * for built_in = 0 devices enabling ocr_mask to MMC_OCR_1V8_MASK
 * sets the voltage to 1.8V
 */
#define MMC_OCR_1V8_MASK    0x00000008
#define MMC_OCR_2V8_MASK    0x00010000
#define MMC_OCR_3V2_MASK    0x00100000
#define MMC_OCR_3V3_MASK    0x00200000


/* uhs mask can be used to mask any of the UHS modes support */
#define MMC_UHS_MASK_SDR12	0x1
#define MMC_UHS_MASK_SDR25	0x2
#define MMC_UHS_MASK_SDR50	0x4
#define MMC_UHS_MASK_DDR50	0x8
#define MMC_UHS_MASK_SDR104	0x10
#define MMC_MASK_HS200		0x20
#define MMC_MASK_HS400		0x40

struct tegra_sdhci_platform_data {
	unsigned int instance;
	unsigned int max_clk_limit;
	unsigned int uhs_mask;
	unsigned int ddr_tap_delay;
	unsigned int ddr_trim_delay;
	bool en_strobe;
	unsigned int ocr_mask;
	bool pwrdet_support;
	bool disable_rtpm;
	bool disable_auto_cal;
	unsigned int auto_cal_step;
	bool enable_autocal_slew_override;
	unsigned int dqs_trim_delay;
	bool is_ddr_trim_delay;
	unsigned int tap_delay;
	unsigned int trim_delay;
	bool en_periodic_cflush; /* Enable periodic cache flush for eMMC */
	int cd_gpio;
	bool cd_wakeup_capable;
	bool en_periodic_calib;
};

#endif
