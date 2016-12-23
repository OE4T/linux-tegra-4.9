/*
 * nvlink-hw.h:
 * This header contains register definitions and HW related macros for the
 * NVLINK driver stack.
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

#ifndef NVLINK_REGS_H
#define NVLINK_REGS_H

/* NVLINK APERTURES - START */


/* TIOCTRL MACROS */
/* TODO: Confirm this delay is correct for Tegra */
#define NVLW_POST_RESET_DELAY_US			8

/* TIOCTRL Registers */
#define NVLW_RESET					0x140
#define NVLW_RESET_LINKRESET				8

#define NVLW_DEBUG_RESET				0x144
#define NVLW_DEBUG_RESET_LINK				0
#define NVLW_DEBUG_RESET_COMMON				31


/* NVL Registers */
#define NVL_LINK_STATE					0x0
#define NVL_LINK_STATE_STATE_MASK			0xff
#define NVL_LINK_STATE_STATE_SWCFG			2

#define NVL_LINK_CONFIG					0x18
#define NVL_LINK_CONFIG_LINK_EN				31

#define NVL_LINK_CHANGE					0x40

#define NVL_SUBLINK_CHANGE				0x44

#define NVL_SL0_SAFE_CTRL2_TX				0x2008

#define NVL_SL0_TRAIN0_TX				0x2018

#define NVL_SL0_TRAIN1_TX				0x201c

#define NVL_SL1_CONFIG_RX				0x3000

#define NVL_SL1_SLSM_STATUS_RX				0x3014
#define NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_MASK	0xf0
#define NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SHIFT	4
#define NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SAFE	6

#define NVL_SL1_RXSLSM_TIMEOUT_2			0x3034


/* NVLTLC Registers */
#define NVLTLC_TX_CTRL_BUFFER_SZ_VC0			0x200

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC1			0x204

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC2			0x208

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC3			0x20c

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC4			0x210

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC5			0x214

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC6			0x218

#define NVLTLC_TX_CTRL_BUFFER_SZ_VC7			0x21c

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC0		0x300

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC1		0x304

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC2		0x308

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC3		0x30c

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC4		0x310

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC5		0x314

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC6		0x318

#define NVLTLC_TX_CTRL_BUFFER_CREDITS_VC7		0x31c

#define NVLTLC_TX_CTRL_BUFFER_READY			0x400

#define NVLTLC_TX_ERR_LOG_EN_0				0x704

#define NVLTLC_TX_ERR_REPORT_EN_0			0x708

#define NVLTLC_TX_ERR_CONTAIN_EN_0			0x70c

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC0			0xa00

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC1			0xa04

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC2			0xa08

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC3			0xa0c

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC4			0xa10

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC5			0xa14

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC6			0xa18

#define NVLTLC_RX_CTRL_BUFFER_SZ_VC7			0xa1c

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC0		0xb00

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC1		0xb04

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC2		0xb08

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC3		0xb0c

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC4		0xb10

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC5		0xb14

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC6		0xb18

#define NVLTLC_RX_CTRL_BUFFER_CREDITS_VC7		0xb1c

#define NVLTLC_RX_CTRL_BUFFER_READY			0xc00

#define NVLTLC_RX_ERR_LOG_EN_0				0xf04

#define NVLTLC_RX_ERR_REPORT_EN_0			0xf08

#define NVLTLC_RX_ERR_CONTAIN_EN_0			0xf0c

#define NVLTLC_RX_ERR_LOG_EN_1				0xf1c

#define NVLTLC_RX_ERR_REPORT_EN_1			0xf20

#define NVLTLC_RX_ERR_CONTAIN_EN_1			0xf24


/* MSSNVLINK Registers */
#define MSSNVLINK_MASTER_CREDIT_TRANSINFO		0x10

#define MSSNVLINK_MASTER_CREDIT_INGR_DATA		0x14

#define MSSNVLINK_SLAVE_CREDIT_TRANSINFO		0x20

#define MSSNVLINK_SLAVE_CREDIT_INGR_DATA		0x24


/* NVLINK APERTURES - END */


/* NON-NVLINK APERTURES - START */
/* TODO: Remove all non-NVLINK register accesses from the driver */


/* NVHS_SOC_CAR Registers */
#define NVHS_SOC_CAR_BASE				0x202f0000

#define CAR_RST_DEV_NVHS_RAIL_CLR			(NVHS_SOC_CAR_BASE + \
							0x8)


/* NVHS_UPHY_CAR Registers */
#define NVHS_UPHY_CAR_BASE				0x211b0000

#define CAR_CLK_SOURCE_NVHS_PLL0_MGMT			(NVHS_UPHY_CAR_BASE + \
							0x1c)

#define CAR_NVHS_UPHY_PLL0_CFG0				(NVHS_UPHY_CAR_BASE + \
							0x20)


/* PLLNVHS_CAR Registers */
#define PLLNVHS_CAR_BASE				0x211f0000

#define CAR_PLLNVHS_SS_CNTL				(PLLNVHS_CAR_BASE + 0x0)

#define CAR_PLLNVHS_MISC1				(PLLNVHS_CAR_BASE + 0x4)

#define CAR_PLLNVHS_BASE				(PLLNVHS_CAR_BASE + 0x8)

#define CAR_PLLNVHS_BASE1				(PLLNVHS_CAR_BASE + 0xc)

#define CAR_PLLNVHS_MISC				(PLLNVHS_CAR_BASE + \
							0x10)

#define CAR_PLLNVHS_SS_CNTL1				(PLLNVHS_CAR_BASE + \
							0x14)

#define CAR_PLLNVHS_SS_CNTL2				(PLLNVHS_CAR_BASE + \
							0x18)


/* NVLINK_CORE_CAR Registers */
#define NVLINK_CORE_CAR_BASE				0x21260000

#define CAR_RST_DEV_NVLINK				(NVLINK_CORE_CAR_BASE \
							+ 0x0)

#define CAR_NVLINK_CLK_CTRL				(NVLINK_CORE_CAR_BASE \
							+ 0x2000)
#define CAR_NVLINK_CLK_CTRL_NVLINK_TXCLK_STS		4

#define CAR_CLK_OUT_ENB_NVLINK_SYSCLK			(NVLINK_CORE_CAR_BASE \
							+ 0x3000)

#define CAR_CLOCK_SOURCE_NVLINK_SYSCLK			(NVLINK_CORE_CAR_BASE \
							+ 0x4000)


/* NON-NVLINK APERTURES - END */

#endif /* NVLINK_REGS_H */
