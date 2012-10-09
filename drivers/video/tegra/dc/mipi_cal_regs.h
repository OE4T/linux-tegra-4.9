/*
 * drivers/video/tegra/dc/mipi_cal_regs.h
 *
 * Copyright (c) 2012, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_MIPI_CAL_REG_H__
#define __DRIVERS_VIDEO_TEGRA_DC_MIPI_CAL_REG_H__

#define MIPI_CAL_MIPI_CAL_CTRL_0	0x0
#define MIPI_CAL_NOISE_FLT(x)		(((x) & 0xf) << 26)
#define MIPI_CAL_PRESCALE(x)		(((x) & 0x3) << 24)
#define MIPI_CAL_CLKEN_OVR(x)		(((x) & 0x1) << 4)
#define MIPI_CAL_AUTOCAL_EN(x)		(((x) & 0x1) << 1)
#define MIPI_CAL_STARTCAL(x)		(((x) & 0x1) << 0)

#define MIPI_CAL_MIPI_BIAS_PAD_CFG0_0	0x58
#define MIPI_BIAS_PAD_PDVCLAMP(x)	(((x) & 0x1) << 1)
#define MIPI_BIAS_PAD_E_VCLAMP_REF(x)	(((x) & 0x1) << 0)

#define MIPI_CAL_MIPI_BIAS_PAD_CFG1_0	0x5c
#define PAD_TEST_SEL(x)			(((x) & 0x7) << 24)
#define PAD_DRIV_DN_REF(x)		(((x) & 0x7) << 16)
#define PAD_DRIV_UP_REF(x)		(((x) & 0x7) << 8)
#define PAD_TERM_REF(x)			(((x) & 0x7) << 0)

#define MIPI_CAL_MIPI_BIAS_PAD_CFG2_0	0x60
#define PAD_VCLAMP_LEVEL(x)		(((x) & 0x7) << 16)
#define PAD_SPARE(x)			(((x) & 0xff) << 8)
#define PAD_VAUXP_LEVEL(x)		(((x) & 0x7) << 4)
#define PAD_PDVREG(x)			(((x) & 0x1) << 1)
#define PAD_VBYPASS(x)			(((x) & 0x1) << 0)

#endif
