/*
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
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-soc.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/usb.h>
#include <soc/tegra/xusb.h>
#include <linux/tegra_prod.h>
#include <dt-bindings/pinctrl/pinctrl-tegra-padctl-uphy.h>

/* #define VERBOSE_DEBUG */
#ifdef TRACE
#undef TRACE
#endif
#ifdef VERBOSE_DEBUG
#define TRACE(dev, fmt, args...)					\
	dev_dbg(dev, "%s(%d) " fmt "\n", __func__, __LINE__, ## args)
#else
#define TRACE(dev, fmt, args...)					\
	do {								\
		if (0)							\
			dev_dbg(dev, "%s(%d) " fmt "\n",		\
				__func__, __LINE__, ## args);		\
	} while (0)
#endif

#include "core.h"
#include "pinctrl-utils.h"

#define TEGRA_PCIE_PHYS		(3)
#define TEGRA_SATA_PHYS		(1)
#define TEGRA_UFS_PHYS		(1)
#define TEGRA_USB3_PHYS		(3)
#define TEGRA_UTMI_PHYS		(3)
#define TEGRA_HSIC_PHYS		(1)

#define T186_UPHY_PLLS				(2)
#define T186_UPHY_LANES				(6)

/* UPHY PLL registers */
#define UPHY_PLL_CTL_1				(0x0)
#define   PLL_IDDQ				(1 << 0)
#define   PLL_SLEEP(x)				(((x) & 0x3) << 1)
#define   PLL_ENABLE				(1 << 3)
#define   PWR_OVRD				(1 << 4)
#define   RATE_ID(x)				(((x) & 0x3) << 8)
#define   RATE_ID_OVRD				(1 << 11)
#define   LOCKDET_STATUS			(1 << 15)

#define UPHY_PLL_CTL_2				(0x4)
#define   CAL_EN				(1 << 0)
#define   CAL_DONE				(1 << 1)
#define   CAL_OVRD				(1 << 2)
#define   CAL_RESET				(1 << 3)
#define   RCAL_EN				(1 << 12)
#define   RCAL_CLK_EN				(1 << 13)
#define   RCAL_OVRD				(1 << 15)
#define   RCAL_DONE				(1 << 31)

#define UPHY_PLL_CTL_4				(0xc)

/* UPHY Lane registers */
#define UPHY_LANE_AUX_CTL_1			(0x0)
#define   AUX_RX_IDLE_TH(x)			(((x) & 0x3) << 24)

#define UPHY_LANE_DIRECT_CTL_1			(0x10)
#define   MISC_CTRL(x)				(((x) & 0xff) << 0)
#define   MISC_OUT(x)				(((x) & 0xff) << 16)

#define UPHY_LANE_DIRECT_CTL_2			(0x14)
#define   CFG_WDATA(x)				(((x) & 0xffff) << 0)
#define   CFG_ADDR(x)				(((x) & 0xff) << 16)
#define   CFG_WDS(x)                            (((x) & 0x1) << 24)
#define   CFG_RDS(x)                            (((x) & 0x1) << 25)
#define   CFG_RESET(x)                          (((x) & 0x1) << 27)

#define UPHY_LANE_MUX				(0x284)
#define   SEL(x)				(((x) & 0x7) << 0)
#define     SEL_XUSB				SEL(0)
#define     SEL_PCIE				SEL(1)
#define     SEL_SATA				SEL(2)
#define     SEL_MPHY				SEL(3)
#define   CLAMP_EN_EARLY			(1 << 8)
#define   FORCE_IDDQ_DISABLE			(1 << 9)

/* UPHY APB Dynamic DYN_CTL registers */
#define UPHY_LANE_DYN_CTL_1			(0x80)
#define   TX_DRV_AMP_SEL0(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_AMP_SEL1(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_AMP_SEL2(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_AMP_SEL3(x)			(((x) & 0x3f) << 24)

#define UPHY_LANE_DYN_CTL_2			(0x84)
#define   TX_DRV_AMP_SEL4(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_AMP_SEL5(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_AMP_SEL6(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_AMP_SEL7(x)			(((x) & 0x3f) << 24)

#define UPHY_LANE_DYN_CTL_3			(0x88)
#define   TX_DRV_AMP_SEL8(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_AMP_SEL9(x)			(((x) & 0x3f) << 8)

#define UPHY_LANE_DYN_CTL_4			(0x8c)
#define   TX_DRV_POST_SEL0(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_PRE_SEL0(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_POST_SEL1(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_PRE_SEL1(x)			(((x) & 0x3f) << 24)

#define UPHY_LANE_DYN_CTL_5			(0x90)
#define   TX_DRV_POST_SEL2(x)			(((x) & 0x3f) << 0)
#define   TX_DRV_PRE_SEL2(x)			(((x) & 0x3f) << 8)
#define   TX_DRV_POST_SEL3(x)			(((x) & 0x3f) << 16)
#define   TX_DRV_PRE_SEL3(x)			(((x) & 0x3f) << 24)

/* FUSE USB_CALIB registers */
/* FUSE_USB_CALIB_0 */
#define HS_CURR_LEVEL_PADX_SHIFT(x)		((x) ? (11 + (x - 1) * 6) : 0)
#define HS_CURR_LEVEL_PAD_MASK			(0x3f)
/* TODO: HS_TERM_RANGE_ADJ has bits overlap, check with hardware team */
#define HS_TERM_RANGE_ADJ_SHIFT			(7)
#define HS_TERM_RANGE_ADJ_MASK			(0xf)
#define HS_SQUELCH_SHIFT			(29)
#define HS_SQUELCH_MASK				(0x7)

/* FUSE_USB_CALIB_EXT_0 */
#define RPD_CTRL_SHIFT				(0)
#define RPD_CTRL_MASK				(0xf)

/* FUSE SATA MPHY registers */
#define FUSE_SATA_MPHY_ODM_CALIB_0	(0x224)
#define    SATA_MPHY_ODM_CALIB_0_1(x)		(((x) & 0x3) << 0)

#define FUSE_SATA_NV_CALIB_0		(0x49c)
#define    SATA_NV_CALIB_0_1(x)			(((x) & (0x3 << 0)) >> 0)
#define    SATA_NV_CALIB_2_3(x)			(((x) & (0x3 << 2)) >> 2)

#define FUSE_MPHY_NV_CALIB_0		(0x4a0)
#define    MPHY_NV_CALIB_0_1(x)			(((x) & (0x3 << 0)) >> 0)
#define    MPHY_NV_CALIB_2_3(x)			(((x) & (0x3 << 2)) >> 2)
#define    MPHY_NV_CALIB_4_5(x)			(((x) & (0x3 << 4)) >> 4)

/* XUSB PADCTL registers */
#define XUSB_PADCTL_USB2_PAD_MUX		(0x4)
#define   PORT_HSIC				(0)
#define   PORT_XUSB				(1)

#define XUSB_PADCTL_USB2_PORT_CAP		(0x8)
#define XUSB_PADCTL_SS_PORT_CAP			(0xc)
#define   PORTX_CAP_SHIFT(x)			((x) * 4)
#define   PORT_CAP_MASK				(0x3)
#define     PORT_CAP_DISABLED			(0x0)
#define     PORT_CAP_HOST			(0x1)
#define     PORT_CAP_DEVICE			(0x2)
#define     PORT_CAP_OTG			(0x3)

#define XUSB_PADCTL_ELPG_PROGRAM		(0x20)

#define XUSB_PADCTL_ELPG_PROGRAM_1		(0x24)
#define   SSPX_ELPG_CLAMP_EN(x)			(1 << (0 + (x) * 3))
#define   SSPX_ELPG_CLAMP_EN_EARLY(x)		(1 << (1 + (x) * 3))
#define   SSPX_ELPG_VCORE_DOWN(x)		(1 << (2 + (x) * 3))

#define USB2_BATTERY_CHRG_OTGPADX_CTL1(x)	(0x84 + (x) * 0x40)
#define   VREG_LEV(x)				(((x) & 0x3) << 7)
#define   VREG_FIX18				(1 << 6)

#define XUSB_PADCTL_USB2_OTG_PADX_CTL0(x)	(0x88 + (x) * 0x40)
#define   HS_CURR_LEVEL(x)			((x) & 0x3f)
#define   USB2_OTG_PD				(1 << 26)
#define   USB2_OTG_PD2				(1 << 27)
#define   USB2_OTG_PD2_OVRD_EN			(1 << 28)
#define   USB2_OTG_PD_ZI			(1 << 29)

#define XUSB_PADCTL_USB2_OTG_PADX_CTL1(x)	(0x8c + (x) * 0x40)
#define   USB2_OTG_PD_DR			(1 << 2)
#define   TERM_RANGE_ADJ(x)			(((x) & 0xf) << 3)
#define   RPD_CTRL(x)				(((x) & 0x1f) << 26)

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL0		(0x284)
#define   BIAS_PAD_PD				(1 << 11)
#define   HS_SQUELCH_LEVEL(x)			(((x) & 0x7) << 0)

#define XUSB_PADCTL_USB2_BIAS_PAD_CTL1		(0x288)
#define   USB2_TRK_START_TIMER(x)		(((x) & 0x7f) << 12)
#define   USB2_TRK_DONE_RESET_TIMER(x)		(((x) & 0x7f) << 19)
#define   USB2_PD_TRK				(1 << 26)

#define XUSB_PADCTL_HSIC_PADX_CTL0(x)		(0x300 + (x) * 0x20)
#define   HSIC_PD_TX_DATA0			(1 << 1)
#define   HSIC_PD_TX_STROBE			(1 << 3)
#define   HSIC_PD_RX_DATA0			(1 << 4)
#define   HSIC_PD_RX_STROBE			(1 << 6)
#define   HSIC_PD_ZI_DATA0			(1 << 7)
#define   HSIC_PD_ZI_STROBE			(1 << 9)
#define   HSIC_RPD_DATA0			(1 << 13)
#define   HSIC_RPD_STROBE			(1 << 15)
#define   HSIC_RPU_DATA0			(1 << 16)
#define   HSIC_RPU_STROBE			(1 << 18)

#define XUSB_PADCTL_HSIC_PAD_TRK_CTL0		(0x340)
#define   HSIC_TRK_START_TIMER(x)		(((x) & 0x7f) << 5)
#define   HSIC_TRK_DONE_RESET_TIMER(x)		(((x) & 0x7f) << 12)
#define   HSIC_PD_TRK				(1 << 19)

#define USB2_VBUS_ID				(0x360)
#define   VBUS_OVERRIDE				(1 << 14)

/* XUSB AO registers */
#define XUSB_AO_USB_DEBOUNCE_DEL		(0x4)
#define   UHSIC_LINE_DEB_CNT(x)			(((x) & 0xf) << 4)
#define   UTMIP_LINE_DEB_CNT(x)			((x) & 0xf)

#define XUSB_AO_UTMIP_TRIGGERS(x)		(0x40 + (x) * 4)
#define   CLR_WALK_PTR				(1 << 0)
#define   CAP_CFG				(1 << 1)
#define   CLR_WAKE_ALARM			(1 << 3)

#define XUSB_AO_UHSIC_TRIGGERS(x)		(0x60 + (x) * 4)
#define   HSIC_CLR_WALK_PTR			(1 << 0)
#define   HSIC_CLR_WAKE_ALARM			(1 << 3)
#define   HSIC_CAP_CFG				(1 << 4)

#define XUSB_AO_UTMIP_SAVED_STATE(x)		(0x70 + (x) * 4)
#define   SPEED(x)				((x) & 0x3)
#define     UTMI_HS				SPEED(0)
#define     UTMI_FS				SPEED(1)
#define     UTMI_LS				SPEED(2)
#define     UTMI_RST				SPEED(3)

#define XUSB_AO_UHSIC_SAVED_STATE(x)		(0x90 + (x) * 4)
#define   MODE(x)				((x) & 0x1)
#define   MODE_HS				MODE(1)
#define   MODE_RST				MODE(0)

#define XUSB_AO_UTMIP_SLEEPWALK_CFG(x)		(0xd0 + (x) * 4)
#define XUSB_AO_UHSIC_SLEEPWALK_CFG(x)		(0xf0 + (x) * 4)
#define   FAKE_USBOP_VAL			(1 << 0)
#define   FAKE_USBON_VAL			(1 << 1)
#define   FAKE_USBOP_EN				(1 << 2)
#define   FAKE_USBON_EN				(1 << 3)
#define   FAKE_STROBE_VAL			(1 << 0)
#define   FAKE_DATA_VAL				(1 << 1)
#define   FAKE_STROBE_EN			(1 << 2)
#define   FAKE_DATA_EN				(1 << 3)
#define   WAKE_WALK_EN				(1 << 14)
#define   MASTER_ENABLE				(1 << 15)
#define   LINEVAL_WALK_EN			(1 << 16)
#define   WAKE_VAL(x)				(((x) & 0xf) << 17)
#define     WAKE_VAL_NONE			WAKE_VAL(12)
#define     WAKE_VAL_ANY			WAKE_VAL(15)
#define     WAKE_VAL_DS10			WAKE_VAL(2)
#define   LINE_WAKEUP_EN			(1 << 21)
#define   MASTER_CFG_SEL			(1 << 22)

#define XUSB_AO_UTMIP_SLEEPWALK(x)		(0x100 + (x) * 4)
/* phase A */
#define   USBOP_RPD_A				(1 << 0)
#define   USBON_RPD_A				(1 << 1)
#define   AP_A					(1 << 4)
#define   AN_A					(1 << 5)
#define   HIGHZ_A				(1 << 6)
/* phase B */
#define   USBOP_RPD_B				(1 << 8)
#define   USBON_RPD_B				(1 << 9)
#define   AP_B					(1 << 12)
#define   AN_B					(1 << 13)
#define   HIGHZ_B				(1 << 14)
/* phase C */
#define   USBOP_RPD_C				(1 << 16)
#define   USBON_RPD_C				(1 << 17)
#define   AP_C					(1 << 20)
#define   AN_C					(1 << 21)
#define   HIGHZ_C				(1 << 22)
/* phase D */
#define   USBOP_RPD_D				(1 << 24)
#define   USBON_RPD_D				(1 << 25)
#define   AP_D					(1 << 28)
#define   AN_D					(1 << 29)
#define   HIGHZ_D				(1 << 30)

#define XUSB_AO_UHSIC_SLEEPWALK(x)		(0x120 + (x) * 4)
/* phase A */
#define   RPD_STROBE_A				(1 << 0)
#define   RPD_DATA0_A				(1 << 1)
#define   RPU_STROBE_A				(1 << 2)
#define   RPU_DATA0_A				(1 << 3)
/* phase B */
#define   RPD_STROBE_B				(1 << 8)
#define   RPD_DATA0_B				(1 << 9)
#define   RPU_STROBE_B				(1 << 10)
#define   RPU_DATA0_B				(1 << 11)
/* phase C */
#define   RPD_STROBE_C				(1 << 16)
#define   RPD_DATA0_C				(1 << 17)
#define   RPU_STROBE_C				(1 << 18)
#define   RPU_DATA0_C				(1 << 19)
/* phase D */
#define   RPD_STROBE_D				(1 << 24)
#define   RPD_DATA0_D				(1 << 25)
#define   RPU_STROBE_D				(1 << 26)
#define   RPU_DATA0_D				(1 << 27)

#define XUSB_AO_UTMIP_PAD_CFG(x)		(0x130 + (x) * 4)
#define   FSLS_USE_XUSB_AO			(1 << 3)
#define   TRK_CTRL_USE_XUSB_AO			(1 << 4)
#define   RPD_CTRL_USE_XUSB_AO			(1 << 5)
#define   RPU_USE_XUSB_AO			(1 << 6)
#define   VREG_USE_XUSB_AO			(1 << 7)
#define   USBOP_VAL_PD				(1 << 8)
#define   USBON_VAL_PD				(1 << 9)

#define XUSB_AO_UHSIC_PAD_CFG(x)		(0x150 + (x) * 4)
#define   STROBE_VAL_PD				(1 << 0)
#define   DATA0_VAL_PD				(1 << 1)
#define   USE_XUSB_AO				(1 << 4)

/* UPHY PLL config space registers */
#define MGMT_FREQ_CTRL_ID0			(0)
#define MGMT_FREQ_CTRL_ID1			(1)
#define   CFG_FREQ_NDIV(x)			(((x) & 0xff) << 0)
#define   CFG_FREQ_MDIV(x)			(((x) & 0x3) << 8)
#define   CFG_FREQ_PSDIV(x)			(((x) & 0x3) << 10)
#define   CFG_MODE(x)				(((x) & 0x3) << 12)

#define MGMT_REFCLK_CTRL			(2)

#define MGMT_CORECLK_CTRL_ID0			(3)
#define MGMT_CORECLK_CTRL_ID1			(4)
#define   CFG_TXCLKREF_EN(x)			(((x) & 0x1) << 0)
#define   CFG_TXCLKREF_SEL(x)			(((x) & 0x3) << 4)
#define     DIVIDE_TX_BY_10			(0)
#define     DIVIDE_TX_BY_8			(1)
#define     DIVIDE_TX_BY_5			(2)
#define     DIVIDE_TX_BY_4			(3)
#define   CFG_XDIGCLK_EN(x)			(((x) & 0x1) << 8)
#define   CFG_XDIGCLK_SEL(x)			(((x) & 0x3) << 12)

#define PLLC_CRSWRD_OVRD_ID0			(5)

#define PLLC_CRSWRD_OVRD_ID1			(6)

#define MGMT_CYA_CTRL				(29)
#define   CFG_MGMT_CLK_SEL(x)			(((x) & 0x1) << 4)

/* UPHY Lane config space registers */
#define MGMT_TX_RATE_CTRL_ID0			(0)
#define MGMT_TX_RATE_CTRL_ID1			(1)
#define MGMT_TX_RATE_CTRL_ID2			(2)
#define MGMT_TX_RATE_CTRL_ID3			(3)
#define   TX_RATE_SDIV(x)			(((x) & 0x3) << 0)
#define     SDIV4				(0)
#define     SDIV2				(1)
#define     SDIV1				(2)
#define     SDIVX				(3)

#define MGMT_RX_RATE_CTRL_ID0			(4)
#define MGMT_RX_RATE_CTRL_ID1			(5)
#define MGMT_RX_RATE_CTRL_ID2			(6)
#define MGMT_RX_RATE_CTRL_ID3			(7)
#define   RX_RATE_SDIV(x)			(((x) & 0x3) << 0)
#define   RX_RATE_CDIV(x)			(((x) & 0x3) << 4)
#define     CDIV1				(0)
#define     CDIV2				(1)
#define     CDIV4				(2)
#define     CDIV8				(3)

#define MGMT_TX_CTRL				(8)
#define   TX_TERM_MODE(x)			(((x) & 0x1) << 0)
#define   SYNC_DLY(x)				(((x) & 0xf) << 4)

#define AE_CTLE_CTRL_ID0			(10)
#define AE_CTLE_CTRL_ID1			(11)
#define AE_CTLE_CTRL_ID2			(12)
#define AE_CTLE_CTRL_ID3			(13)
#define   LF_UGRAY(x)				(((x) & 0xf) << 0)
#define   HF_UBIN(x)				(((x) & 0xf) << 4)

#define AE_DFE0_CTRL_ID0			(14)
#define AE_DFE0_CTRL_ID1			(15)
#define AE_DFE0_CTRL_ID2			(16)
#define AE_DFE1_CTRL_ID2			(20)
#define   H1_SBIN(x)				(((x) & 0x1f) << 0)
#define   H2_SBIN(x)				(((x) & 0xf) << 8)
#define   H3_SBIN(x)				(((x) & 0x7) << 12)

#define AE_DFE1_CTRL_ID0			(18)
#define   H4_SBIN(x)				(((x) & 0x7) << 0)
#define   H0_SBIN(x)				(((x) & 0x3f) << 8)

#define AE_CDR_CTRL_ID0				(22)
#define AE_CDR_CTRL_ID1				(23)
#define AE_CDR_CTRL_ID2				(24)
#define AE_CDR_CTRL_ID3				(25)
#define   PHGAIN(x)				(((x) & 0xf) << 0)
#define   FRGAIN(x)				(((x) & 0xf) << 4)
#define   FRLOOP_EN(x)				(((x) & 0x1) << 8)

#define AE_EQ0_CTRL_ID0				(26)
#define AE_EQ0_CTRL_ID1				(27)
#define AE_EQ0_CTRL_ID2				(28)
#define   EQ0_SEQ_MODE(x)			(((x) & 0x3) << 0)
#define     SEQ_MODE_HF_Z_H0_HN_Z		(0)
#define     SEQ_MODE_H0_HN_Z_HF_Z		(1)
#define     SEQ_MODE_HF_H0_HN_Z			(2)
#define     SEQ_MODE_HF_H0_HN_Z_HF		(3)
#define   H0INIT_ITERS(x)			(((x) & 0xf) << 4)
#define   H4_GRAD_INV(x)			(((x) & 0x1) << 8)
#define   H3_GRAD_INV(x)			(((x) & 0x1) << 9)
#define   H2_GRAD_INV(x)			(((x) & 0x1) << 10)
#define   H1_GRAD_INV(x)			(((x) & 0x1) << 11)
#define   H0_GRAD_INV(x)			(((x) & 0x1) << 12)
#define   CTLE_HF_GRAD_INV(x)			(((x) & 0x1) << 13)
#define   SAMP_VOS_GRAD_INV(x)			(((x) & 0x1) << 14)
#define   CTLE_VOS_GRAD_INV(x)			(((x) & 0x1) << 15)

#define AE_EQ1_CTRL_ID0				(30)
#define AE_EQ1_CTRL_ID1				(31)
#define AE_EQ1_CTRL_ID2				(32)
#define AE_EQ1_CTRL_ID3				(33)
#define   CTLE_HF_MODE(x)			(((x) & 0x3) << 0)
#define     VAR_MODE_AUTO			(0)
#define     VAR_MODE_OFF			(1)
#define     VAR_MODE_HOLD			(2)
#define     VAR_MODE_LOAD			(3)
#define   CTLE_HF_GRAD(x)			(((x) & 0x3) << 2)
#define      CTLE_HF_GRAD_1P5			(0)
#define      CTLE_HF_GRAD_2P5			(1)
#define      CTLE_HF_GRAD_3P5			(2)
#define      CTLE_HF_GRAD_1TP			(3)
#define   H1_MODE(x)				(((x) & 0x3) << 8)
#define   H2_MODE(x)				(((x) & 0x3) << 10)
#define   H3_MODE(x)				(((x) & 0x3) << 12)
#define   H4_MODE(x)				(((x) & 0x3) << 14)

#define AE_EQ2_CTRL_ID0				(34)
#define AE_EQ2_CTRL_ID1				(35)
#define AE_EQ2_CTRL_ID2				(36)
#define AE_EQ2_CTRL_ID3				(37)
#define   H0_MODE(x)				(((x) & 0x3) << 0)
#define   H0_DAC_TIME(x)			(((x) & 0x3) << 2)
#define   H0_TIME(x)				(((x) & 0xf) << 4)
#define   H0_ITERS(x)				(((x) & 0xf) << 8)
#define   HN_ITERS(x)				(((x) & 0xf) << 12)

#define AE_EQ3_CTRL_ID0				(38)
#define AE_EQ3_CTRL_ID2				(40)
#define   H0_GAIN(x)				(((x) & 0xf) << 0)
#define   HN_GAIN(x)				(((x) & 0xf) << 4)
#define   CTLE_HF_TIME(x)			(((x) & 0xf) << 8)
#define   CTLE_HF_GAIN(x)			(((x) & 0xf) << 12)

#define MGMT_RX_PI_CTRL_ID2			(48)
#define   TX_SLEW(x)				(((x) & 0x3) << 0)
#define   RX_SLEW(x)				(((x) & 0x3) << 2)

enum tegra186_function {
	TEGRA186_FUNC_HSIC,
	TEGRA186_FUNC_XUSB,
	TEGRA186_FUNC_PCIE,
	TEGRA186_FUNC_USB3,
	TEGRA186_FUNC_SATA,
	TEGRA186_FUNC_MPHY,
};

struct tegra_padctl_uphy_function {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct tegra_padctl_uphy_group {
	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_padctl_uphy_soc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;

	const struct tegra_padctl_uphy_function *functions;
	unsigned int num_functions;

	const struct tegra_padctl_uphy_lane *lanes;
	unsigned int num_lanes;

	unsigned int hsic_port_offset;
};

struct tegra_padctl_uphy_lane {
	const char *name;

	unsigned int offset;
	unsigned int shift;
	unsigned int mask;
	unsigned int iddq;

	const unsigned int *funcs;
	unsigned int num_funcs;
};

struct tegra_xusb_fuse_calibration {
	u32 hs_curr_level[TEGRA_UTMI_PHYS];
	u32 hs_squelch;
	u32 hs_term_range_adj;
	u32 rpd_ctrl;
};

struct tegra_fuse_calibration {
	u32 sata_mphy_odm;
	u32 sata_nv;
	u32 mphy_nv;
};

enum xusb_port_cap {
	CAP_DISABLED = TEGRA_PADCTL_PORT_DISABLED,
	HOST_ONLY = TEGRA_PADCTL_PORT_HOST_ONLY,
	DEVICE_ONLY = TEGRA_PADCTL_PORT_DEVICE_ONLY,
	OTG = TEGRA_PADCTL_PORT_OTG_CAP,
};

struct tegra_xusb_usb3_port {
	enum xusb_port_cap port_cap;
	unsigned int uphy_lane;
};

struct tegra_pcie_controller {
	unsigned int uphy_lane_bitmap;
};

struct tegra_xusb_utmi_port {
	enum xusb_port_cap port_cap;
};

enum uphy_pll_state {
	PLL_POWER_DOWN,
	PLL_POWER_UP_SW_CTRL,
	PLL_POWER_UP_HW_SEQ,
};

struct tegra_padctl_uphy {
	struct device *dev;
	void __iomem *padctl_regs;
	void __iomem *ao_regs;
	void __iomem *uphy_regs;
	void __iomem *uphy_pll_regs[2];
	void __iomem *uphy_lane_regs[6];

	struct reset_control *uphy_rst;

	struct clk *usb2_trk_clk; /* utmi tracking circuit clock */
	struct clk *hsic_trk_clk; /* hsic tracking circuit clock */

	struct mutex lock;

	const struct tegra_padctl_uphy_soc *soc;
	struct tegra_xusb_fuse_calibration calib;
	struct tegra_fuse_calibration fuse_calib;
	struct tegra_prod_list *prod_list;
	struct pinctrl_dev *pinctrl;
	struct pinctrl_desc desc;

	struct phy_provider *provider;
	struct phy *usb3_phys[TEGRA_USB3_PHYS];
	struct phy *utmi_phys[TEGRA_UTMI_PHYS];
	struct phy *hsic_phys[TEGRA_HSIC_PHYS];
	struct phy *pcie_phys[TEGRA_PCIE_PHYS];
	struct phy *sata_phys[TEGRA_SATA_PHYS];
	struct phy *ufs_phys[TEGRA_UFS_PHYS];
	struct tegra_xusb_utmi_port utmi_ports[TEGRA_UTMI_PHYS];
	struct tegra_xusb_usb3_port usb3_ports[TEGRA_USB3_PHYS];
	unsigned long usb3_lanes;
	struct tegra_pcie_controller pcie_controllers[TEGRA_PCIE_PHYS];
	unsigned long pcie_lanes;
	unsigned long sata_lanes;
	unsigned long ufs_lanes;

	unsigned int enable_counts;
	bool sata_bypass_fuse;

	struct work_struct mbox_req_work;
	struct tegra_xusb_mbox_msg mbox_req;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;

	unsigned int utmi_enable;
	unsigned int hs_curr_level_offset[TEGRA_UTMI_PHYS];
	/* TODO: should move to host controller driver? */
	struct regulator *vbus[TEGRA_UTMI_PHYS];
	struct regulator *vddio_hsic;

	int uphy_pll_clients[T186_UPHY_PLLS];
	enum uphy_pll_state uphy_pll_state[T186_UPHY_PLLS];
	struct reset_control *uphy_pll_rst[T186_UPHY_PLLS];
	struct reset_control *uphy_lane_rst[T186_UPHY_LANES];
	struct reset_control *uphy_master_rst;

};

#ifdef VERBOSE_DEBUG
#define ao_writel(_padctl, _value, _offset)				\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_debug("%s ao_writel %s(@0x%lx) with 0x%lx\n", __func__,	\
		#_offset, o, v);					\
	writel(v, _padctl->ao_regs + o);				\
}

#define ao_readl(_padctl, _offset)					\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_padctl->ao_regs + o);				\
	pr_debug("%s ao_readl %s(@0x%lx) = 0x%lx\n", __func__,		\
		#_offset, o, v);					\
	v;								\
})
#else
static inline void ao_writel(struct tegra_padctl_uphy *padctl, u32 value,
				 unsigned long offset)
{
	writel(value, padctl->ao_regs + offset);
}

static inline u32 ao_readl(struct tegra_padctl_uphy *padctl,
			       unsigned long offset)
{
	return readl(padctl->ao_regs + offset);
}
#endif

#ifdef VERBOSE_DEBUG
#define padctl_writel(_padctl, _value, _offset)				\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_debug("%s padctl_write %s(@0x%lx) with 0x%lx\n", __func__,	\
		#_offset, o, v);					\
	writel(v, _padctl->padctl_regs + o);				\
}

#define padctl_readl(_padctl, _offset)					\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_padctl->padctl_regs + o);				\
	pr_debug("%s padctl_read %s(@0x%lx) = 0x%lx\n", __func__,	\
		#_offset, o, v);					\
	v;								\
})
#else
static inline void padctl_writel(struct tegra_padctl_uphy *padctl, u32 value,
				 unsigned long offset)
{
	writel(value, padctl->padctl_regs + offset);
}

static inline u32 padctl_readl(struct tegra_padctl_uphy *padctl,
			       unsigned long offset)
{
	return readl(padctl->padctl_regs + offset);
}
#endif

#ifdef VERBOSE_DEBUG
#define uphy_pll_writel(_uphy, _pll, _value, _offset)			\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_debug("%s uphy_pll_writel pll %d %s(@0x%lx) with 0x%lx\n",	\
		__func__, _pll, #_offset, o, v);			\
	writel(v, _uphy->uphy_pll_regs[_pll] + o);			\
}

#define uphy_pll_readl(_uphy, _pll, _offset)				\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_uphy->uphy_pll_regs[_pll] + o);			\
	pr_debug("%s uphy_pll_readl pll %d %s(@0x%lx) = 0x%lx\n",	\
		__func__, _pll, #_offset, o, v);			\
	v;								\
})
#else
static inline void uphy_pll_writel(struct tegra_padctl_uphy *uphy, int pll,
				   u32 value, unsigned long offset)
{
	writel(value, uphy->uphy_pll_regs[pll] + offset);
}

static inline u32 uphy_pll_readl(struct tegra_padctl_uphy *uphy, int pll,
				 unsigned long offset)
{
	return readl(uphy->uphy_pll_regs[pll] + offset);
}
#endif

#ifdef VERBOSE_DEBUG
#define uphy_lane_writel(_uphy, _lane, _value, _offset)			\
{									\
	unsigned long v = _value, o = _offset;				\
	pr_debug("%s uphy_lane_writel lane %d %s(@0x%lx) with 0x%lx\n",	\
		__func__, _lane, #_offset, o, v);			\
	writel(v, _uphy->uphy_lane_regs[_lane] + o);			\
}

#define uphy_lane_readl(_uphy, _lane, _offset)				\
({									\
	unsigned long v, o = _offset;					\
	v = readl(_uphy->uphy_lane_regs[_lane] + o);			\
	pr_debug("%s uphy_lane_readl lane %d %s(@0x%lx) = 0x%lx\n",	\
		__func__, _lane, #_offset, o, v);			\
	v;								\
})
#else
static inline void uphy_lane_writel(struct tegra_padctl_uphy *uphy, int lane,
				   u32 value, unsigned long offset)
{
	writel(value, uphy->uphy_lane_regs[lane] + offset);
}

static inline u32 uphy_lane_readl(struct tegra_padctl_uphy *uphy, int lane,
				 unsigned long offset)
{
	return readl(uphy->uphy_lane_regs[lane] + offset);
}
#endif

struct tegra_mphy_sata_calib {
	u8 aux_rx_idle_th;
	u8 tx_drv_amp_sel0;
	u8 tx_drv_amp_sel1;
	u8 tx_drv_amp_sel2;
	u8 tx_drv_amp_sel3;
	u8 tx_drv_amp_sel4;
	u8 tx_drv_amp_sel5;
	u8 tx_drv_amp_sel6;
	u8 tx_drv_amp_sel7;
	u8 tx_drv_amp_sel8;
	u8 tx_drv_amp_sel9;
	u8 tx_drv_post_sel0;
	u8 tx_drv_post_sel1;
	u8 tx_drv_post_sel2;
	u8 tx_drv_post_sel3;
	u8 tx_drv_pre_sel3;
	u8 ae_ctle_ctrl_id0;
	u8 ae_ctle_ctrl_id1;
};

static struct tegra_mphy_sata_calib mphy_data[] = {
	{
		.aux_rx_idle_th = 0x0,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
	{
		.aux_rx_idle_th = 0x1,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
	{
		.aux_rx_idle_th = 0x2,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
	{
		.aux_rx_idle_th = 0x3,
		.tx_drv_amp_sel0 = 0x8,
		.tx_drv_amp_sel1 = 0x11,
		.tx_drv_amp_sel2 = 0x8,
		.tx_drv_amp_sel3 = 0x11,
		.tx_drv_amp_sel4 = 0x8,
		.tx_drv_amp_sel5 = 0x11,
		.tx_drv_amp_sel6 = 0x8,
		.tx_drv_amp_sel7 = 0x11,
		.tx_drv_amp_sel8 = 0x8,
		.tx_drv_amp_sel9 = 0x11,
		.tx_drv_post_sel0 = 0x0,
		.tx_drv_post_sel1 = 0xa,
		.tx_drv_post_sel2 = 0xf,
		.tx_drv_post_sel3 = 0x8,
		.tx_drv_pre_sel3 = 0x8,
	},
};

static struct tegra_mphy_sata_calib sata_data[] = {
	{
		.aux_rx_idle_th = 0x0,
		.tx_drv_amp_sel0 = 0x1b,
		.tx_drv_amp_sel1 = 0x1f,
		.tx_drv_post_sel0 = 0x7,
		.tx_drv_post_sel1 = 0xa,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0x8f,
	},
	{
		.aux_rx_idle_th = 0x1,
		.tx_drv_amp_sel0 = 0x17,
		.tx_drv_amp_sel1 = 0x1b,
		.tx_drv_post_sel0 = 0x5,
		.tx_drv_post_sel1 = 0xa,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0x4f,
	},
	{
		.aux_rx_idle_th = 0x2,
		.tx_drv_amp_sel0 = 0x13,
		.tx_drv_amp_sel1 = 0x17,
		.tx_drv_post_sel0 = 0x4,
		.tx_drv_post_sel1 = 0xa,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0xf,
	},
	{
		.aux_rx_idle_th = 0x3,
		.tx_drv_amp_sel0 = 0x1f,
		.tx_drv_amp_sel1 = 0x23,
		.tx_drv_post_sel0 = 0xa,
		.tx_drv_post_sel1 = 0xe,
		.ae_ctle_ctrl_id0 = 0xf,
		.ae_ctle_ctrl_id1 = 0xcd,
	},
};

struct init_data {
	u8 cfg_addr;
	u16 cfg_wdata;
	bool cfg_wds;
	bool cfg_rds;
	bool cfg_rst;
};

static struct init_data usb3_pll_g1_init_data[] = {
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_5),
	},
};

static void pcie_usb3_pll_defaults(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(usb3_pll_g1_init_data); i++) {
		reg = CFG_ADDR(usb3_pll_g1_init_data[i].cfg_addr);
		reg |= CFG_WDATA(usb3_pll_g1_init_data[i].cfg_wdata);
		uphy_pll_writel(uphy, 0, reg, UPHY_PLL_CTL_4);
	}
}

#define pcie_pll_init pcie_usb3_pll_defaults

static struct init_data sata_pll_g1_g2_g3_init_data[] = {
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID0,
		.cfg_wdata = CFG_FREQ_NDIV(30),
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_10),
	},
};

static void sata_pll_defaults(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_pll_g1_g2_g3_init_data); i++) {
		reg = CFG_ADDR(sata_pll_g1_g2_g3_init_data[i].cfg_addr);
		reg |= CFG_WDATA(sata_pll_g1_g2_g3_init_data[i].cfg_wdata);
		uphy_pll_writel(uphy, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_pll_g1_g2_g3_A_B_init_data[] = {
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID0,
		.cfg_wdata = CFG_FREQ_NDIV(24) | CFG_FREQ_MDIV(1),
	},
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID1,
		.cfg_wdata = CFG_FREQ_NDIV(28) | CFG_FREQ_MDIV(1),
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_10),
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID1,
		.cfg_wdata = CFG_TXCLKREF_EN(1) |
			     CFG_TXCLKREF_SEL(DIVIDE_TX_BY_10),
	},
	{
		.cfg_addr = PLLC_CRSWRD_OVRD_ID1,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = MGMT_CYA_CTRL,
		.cfg_wdata = CFG_MGMT_CLK_SEL(1),
	},
};

static void ufs_pll_defaults(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_pll_g1_g2_g3_A_B_init_data); i++) {
		reg = CFG_ADDR(ufs_pll_g1_g2_g3_A_B_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_pll_g1_g2_g3_A_B_init_data[i].cfg_wdata);
		uphy_pll_writel(uphy, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_pll_rateid_init_data[] = {
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID0,
		.cfg_wdata = CFG_FREQ_NDIV(0x18) | CFG_FREQ_MDIV(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_FREQ_CTRL_ID1,
		.cfg_wdata = CFG_FREQ_NDIV(0x1c) | CFG_FREQ_MDIV(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_REFCLK_CTRL,
		.cfg_wdata = 0x0,
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_CORECLK_CTRL_ID0,
		.cfg_wdata = CFG_XDIGCLK_SEL(0x7) | CFG_XDIGCLK_EN(0x1)
			| CFG_TXCLKREF_SEL(0x2) | CFG_TXCLKREF_EN(1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = CFG_XDIGCLK_SEL(0x7) | CFG_XDIGCLK_EN(0x1)
			| CFG_TXCLKREF_SEL(0x2) | CFG_TXCLKREF_EN(1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = PLLC_CRSWRD_OVRD_ID0,
		.cfg_wdata = 0x3e,
		.cfg_wds = true,
		.cfg_rst = true,
	}
};

static void ufs_pll_rateid_init(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_pll_rateid_init_data); i++) {
		reg = CFG_ADDR(ufs_pll_rateid_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_pll_rateid_init_data[i].cfg_wdata);
		reg |= CFG_WDS(ufs_pll_rateid_init_data[i].cfg_wds);
		reg |= CFG_RDS(ufs_pll_rateid_init_data[i].cfg_rds);
		reg |= CFG_RESET(ufs_pll_rateid_init_data[i].cfg_rst);
		uphy_pll_writel(uphy, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_lane_rateid_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV4),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = TX_RATE_SDIV(SDIV2),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID2,
		.cfg_wdata = TX_RATE_SDIV(SDIV1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID3,
		.cfg_wdata = TX_RATE_SDIV(SDIV1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(CDIV1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID1,
		.cfg_wdata = RX_RATE_SDIV(CDIV2),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID2,
		.cfg_wdata = RX_RATE_SDIV(CDIV4),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(CDIV4),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0x3) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID1,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0x6) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID2,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0xc) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID3,
		.cfg_wdata = FRLOOP_EN(0x1) | FRGAIN(0xc) | PHGAIN(0x7),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = HF_UBIN(0x0) | LF_UGRAY(0xf),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID1,
		.cfg_wdata = HF_UBIN(0x0) | LF_UGRAY(0xf),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID2,
		.cfg_wdata = HF_UBIN(0x8) | LF_UGRAY(0xf),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID3,
		.cfg_wdata = HF_UBIN(0xf) | LF_UGRAY(0xd),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID0,
		.cfg_wdata = CTLE_HF_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID1,
		.cfg_wdata = CTLE_HF_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID2,
		.cfg_wdata = CTLE_HF_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID3,
		.cfg_wdata = CTLE_HF_MODE(0x0),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x1),
		.cfg_wds = true,
		.cfg_rst = true,
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H0_MODE(0x0),
		.cfg_wds = true,
		.cfg_rst = true,
	},
};

static void ufs_lane_rateid_init(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_lane_rateid_init_data); i++) {
		reg = CFG_ADDR(ufs_lane_rateid_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_lane_rateid_init_data[i].cfg_wdata);
		reg |= CFG_WDS(ufs_lane_rateid_init_data[i].cfg_wds);
		reg |= CFG_RDS(ufs_lane_rateid_init_data[i].cfg_rds);
		reg |= CFG_RESET(ufs_lane_rateid_init_data[i].cfg_rst);
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}
}


static struct init_data pcie_lane_g1_g2_init_data[] = {
};

static void pcie_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(pcie_lane_g1_g2_init_data); i++) {
		reg = CFG_ADDR(pcie_lane_g1_g2_init_data[i].cfg_addr);
		reg |= CFG_WDATA(pcie_lane_g1_g2_init_data[i].cfg_wdata);
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}
}

static struct init_data usb3_lane_g1_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV1),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(SDIV1) | RX_RATE_CDIV(CDIV4),
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(0),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0xd) | HF_UBIN(0xf),
	},
	{
		.cfg_addr = AE_DFE0_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0xd) | HF_UBIN(0xf),
	},
	{
		.cfg_addr = AE_DFE1_CTRL_ID0,
		.cfg_wdata = H0_SBIN(48) | H4_SBIN(-1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0xc) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_EQ0_CTRL_ID0,
		.cfg_wdata = EQ0_SEQ_MODE(SEQ_MODE_HF_Z_H0_HN_Z) |
			     H0INIT_ITERS(0x3),
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID0,
		.cfg_wdata = CTLE_HF_MODE(VAR_MODE_AUTO) |
			     CTLE_HF_GRAD(CTLE_HF_GRAD_1P5) |
			     H1_MODE(VAR_MODE_AUTO) |
			     H2_MODE(VAR_MODE_AUTO) |
			     H3_MODE(VAR_MODE_AUTO) |
			     H4_MODE(VAR_MODE_AUTO),
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID0,
		.cfg_wdata = H1_MODE(VAR_MODE_AUTO) | H0_DAC_TIME(0x2) |
			     H0_TIME(0x6) | H0_ITERS(0x3) | HN_ITERS(0x1),
	},
	{
		.cfg_addr = AE_EQ3_CTRL_ID0,
		.cfg_wdata = HN_GAIN(0Xf) | CTLE_HF_TIME(0xc) |
			     CTLE_HF_GAIN(0xf),
	},
};

static void usb3_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(usb3_lane_g1_init_data); i++) {
		reg = CFG_ADDR(usb3_lane_g1_init_data[i].cfg_addr);
		reg |= CFG_WDATA(usb3_lane_g1_init_data[i].cfg_wdata);
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static struct init_data sata_lane_g1_g2_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV4),
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = TX_RATE_SDIV(SDIV2),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(SDIV4) | RX_RATE_CDIV(CDIV1),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID1,
		.cfg_wdata = RX_RATE_SDIV(SDIV2) | RX_RATE_CDIV(CDIV2),
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(0),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0x8),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x3) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID1,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x6) | FRLOOP_EN(1),
	},
};

static void sata_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_lane_g1_g2_init_data); i++) {
		reg = CFG_ADDR(sata_lane_g1_g2_init_data[i].cfg_addr);
		reg |= CFG_WDATA(sata_lane_g1_g2_init_data[i].cfg_wdata);
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static struct init_data ufs_lane_g1_g2_g3_init_data[] = {
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID0,
		.cfg_wdata = TX_RATE_SDIV(SDIV4),
	},
	{
		.cfg_addr = MGMT_TX_RATE_CTRL_ID1,
		.cfg_wdata = TX_RATE_SDIV(SDIV2),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID0,
		.cfg_wdata = RX_RATE_SDIV(SDIV4) | RX_RATE_CDIV(CDIV1),
	},
	{
		.cfg_addr = MGMT_RX_RATE_CTRL_ID1,
		.cfg_wdata = RX_RATE_SDIV(SDIV2) | RX_RATE_CDIV(CDIV2),
	},
	{
		.cfg_addr = MGMT_TX_CTRL,
		.cfg_wdata = TX_TERM_MODE(1),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID0,
		.cfg_wdata = LF_UGRAY(0xf),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID1,
		.cfg_wdata = LF_UGRAY(0xf),
	},
	{
		.cfg_addr = AE_CTLE_CTRL_ID2,
		.cfg_wdata = LF_UGRAY(0xf) | HF_UBIN(8),
	},

	{
		.cfg_addr = AE_DFE0_CTRL_ID2,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = AE_DFE1_CTRL_ID2,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID0,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x3) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID1,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0x6) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_CDR_CTRL_ID2,
		.cfg_wdata = PHGAIN(0x7) | FRGAIN(0xc) | FRLOOP_EN(1),
	},
	{
		.cfg_addr = AE_EQ0_CTRL_ID2,
		.cfg_wdata = EQ0_SEQ_MODE(SEQ_MODE_HF_Z_H0_HN_Z),
	},
	{
		.cfg_addr = AE_EQ1_CTRL_ID2,
		.cfg_wdata = CTLE_HF_MODE(VAR_MODE_OFF) |
			     CTLE_HF_GRAD(CTLE_HF_GRAD_1P5) |
			     H1_MODE(VAR_MODE_OFF) |
			     H2_MODE(VAR_MODE_OFF) |
			     H3_MODE(VAR_MODE_OFF) |
			     H4_MODE(VAR_MODE_OFF),
	},
	{
		.cfg_addr = AE_EQ2_CTRL_ID2,
		.cfg_wdata = H1_MODE(VAR_MODE_OFF),
	},
	{
		.cfg_addr = AE_EQ3_CTRL_ID2,
		.cfg_wdata = 0,
	},
	{
		.cfg_addr = MGMT_RX_PI_CTRL_ID2,
		.cfg_wdata = 0,
	},
};

static void ufs_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_lane_g1_g2_g3_init_data); i++) {
		reg = CFG_ADDR(ufs_lane_g1_g2_g3_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_lane_g1_g2_g3_init_data[i].cfg_wdata);
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static int __uphy_pll_init(struct tegra_padctl_uphy *uphy, int pll,
			enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	u32 reg;
	int i;

	/* pll defaults */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg |= PWR_OVRD;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg |= (CAL_OVRD | RCAL_OVRD | CAL_RESET);
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	/* power up PLL */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_IDDQ;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_SLEEP(~0);
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	ndelay(100);

	/* perform PLL calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg |= CAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	for (i = 0; i < 20; i++) {
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		if (reg & CAL_DONE)
			break;
		usleep_range(10, 15);
	}
	if (!(reg & CAL_DONE)) {
		dev_err(dev, "start PLL %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* stop PLL calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~CAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	for (i = 0; i < 20; i++) {
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		if (!(reg & CAL_DONE))
			break;
		usleep_range(10, 15);
	}
	if (reg & CAL_DONE) {
		dev_err(dev, "stop PLL %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	if (pll == 1 && func == TEGRA186_FUNC_MPHY) {
		/* perform PLL rate change for UPHY_PLL_1, used by UFS */
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
		reg &= ~RATE_ID(~0);
		reg |= (RATE_ID(1) | RATE_ID_OVRD);
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

		ndelay(100);

		/* perform PLL calibration for rate B */
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		reg |= CAL_EN;
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
		for (i = 0; i < 20; i++) {
			reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
			if (reg & CAL_DONE)
				break;
			usleep_range(10, 15);
		}
		if (!(reg & CAL_DONE)) {
			dev_err(dev, "start PLL %d calibration timeout\n", pll);
			return -ETIMEDOUT;
		}

		/* stop PLL calibration */
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		reg &= ~CAL_EN;
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
		for (i = 0; i < 20; i++) {
			reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
			if (!(reg & CAL_DONE))
				break;
			usleep_range(10, 15);
		}
		if (reg & CAL_DONE) {
			dev_err(dev, "stop PLL %d calibration timeout\n", pll);
			return -ETIMEDOUT;
		}

		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
		reg &= ~RATE_ID(~0);
		reg |= RATE_ID_OVRD;
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);
		ndelay(100);
	}

	/* enable PLL and wait for lock */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg |= PLL_ENABLE;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);
	usleep_range(20, 25);
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	if (!(reg & LOCKDET_STATUS)) {
		dev_err(dev, "enable PLL %d timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* perform resistor calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg |= RCAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg |= RCAL_CLK_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	usleep_range(5, 10);
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	if (!(reg & RCAL_DONE)) {
		dev_err(dev, "start resistor %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* stop resistor calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~RCAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	usleep_range(5, 10);
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	if (reg & RCAL_DONE) {
		dev_err(dev, "stop resistor %d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~RCAL_CLK_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	/* remove SW overrides to allow HW sequencer to run */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PWR_OVRD;
	if (pll == 1 && func == TEGRA186_FUNC_MPHY)
		reg &= ~RATE_ID_OVRD;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~(CAL_OVRD | RCAL_OVRD);
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	uphy->uphy_pll_state[pll] = PLL_POWER_UP_HW_SEQ;
	return 0;
}

static int uphy_pll_init(struct tegra_padctl_uphy *uphy, int pll,
		enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc = 0;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	mutex_lock(&uphy->lock);

	TRACE(dev, "pll %d state %d", pll, uphy->uphy_pll_state[pll]);
	if (uphy->uphy_pll_state[pll] != PLL_POWER_DOWN)
		goto done;

	rc = __uphy_pll_init(uphy, pll, func);

done:
	mutex_unlock(&uphy->lock);
	return rc;
}

static int uphy_pll_deinit(struct tegra_padctl_uphy *uphy, int pll)
{
	struct device *dev = uphy->dev;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	mutex_lock(&uphy->lock);
	TRACE(dev, "pll %d state %d", pll, uphy->uphy_pll_state[pll]);
	if (uphy->uphy_pll_state[pll] == PLL_POWER_DOWN)
		goto done;

	/* PLL power down sequence */
	dev_info(dev, "%s FIXME: implement!\n", __func__); /* TODO */

done:
	mutex_lock(&uphy->lock);
	return 0;
}

static int uphy_pll_reset_deassert(struct tegra_padctl_uphy *uphy, int pll)
{
	struct device *dev = uphy->dev;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	mutex_lock(&uphy->lock);

	TRACE(dev, "pll %d clients %d", pll, uphy->uphy_pll_clients[pll]);
	if (uphy->uphy_pll_clients[pll]++ > 0)
		goto out;

	reset_control_deassert(uphy->uphy_pll_rst[pll]);

out:
	mutex_unlock(&uphy->lock);
	return 0;
}

static int uphy_pll_reset_assert(struct tegra_padctl_uphy *uphy, int pll)
{
	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	mutex_lock(&uphy->lock);
	TRACE(uphy->dev, "pll %d clients %d", pll, uphy->uphy_pll_clients[pll]);

	if (WARN_ON(uphy->uphy_pll_clients[pll] == 0))
		goto out;

	if (--uphy->uphy_pll_clients[pll] > 0)
		goto out;

	reset_control_assert(uphy->uphy_pll_rst[pll]);

out:
	mutex_lock(&uphy->lock);
	return 0;
}

static int uphy_lane_reset_deassert(struct tegra_padctl_uphy *uphy, int lane)
{

	if (lane < 0 || lane >= T186_UPHY_LANES)
		return -EINVAL;

	reset_control_deassert(uphy->uphy_lane_rst[lane]);

	return 0;
}

static int uphy_lane_reset_assert(struct tegra_padctl_uphy *uphy, int lane)
{
	if (lane < 0 || lane >= T186_UPHY_LANES)
		return -EINVAL;

	reset_control_assert(uphy->uphy_lane_rst[lane]);

	return 0;
}

static inline
struct tegra_padctl_uphy *mbox_work_to_uphy(struct work_struct *work)
{
	return container_of(work, struct tegra_padctl_uphy, mbox_req_work);
}

#define PIN_OTG_0	0
#define PIN_OTG_1	1
#define PIN_OTG_2	2
#define PIN_HSIC_0	3
#define PIN_UPHY_0	4
#define PIN_UPHY_1	5
#define PIN_UPHY_2	6
#define PIN_UPHY_3	7
#define PIN_UPHY_4	8
#define PIN_UPHY_5	9

static inline bool lane_is_otg(unsigned int lane)
{
	return lane >= PIN_OTG_0 && lane <= PIN_OTG_2;
}

static inline bool lane_is_hsic(unsigned int lane)
{
	return lane == PIN_HSIC_0;
}

static inline bool lane_is_uphy(unsigned int lane)
{
	return lane >= PIN_UPHY_0 && lane <= PIN_UPHY_5;
}

static int lane_to_usb3_port(struct tegra_padctl_uphy *uphy,
			     unsigned int uphy_lane)
{
	unsigned int i;

	for (i = 0; i < TEGRA_USB3_PHYS; i++) {
		if (uphy->usb3_ports[i].uphy_lane == uphy_lane)
			return i;
	}

	return -EINVAL;
}

static int lane_to_pcie_controller(struct tegra_padctl_uphy *uphy,
			     unsigned int uphy_lane)
{
	unsigned int i;

	for (i = 0; i < TEGRA_PCIE_PHYS; i++) {
		if (uphy->pcie_controllers[i].uphy_lane_bitmap | BIT(uphy_lane))
			return i;
	}

	return -EINVAL;
}

static int tegra_padctl_uphy_get_groups_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);

	TRACE(uphy->dev, "num_pins %u", uphy->soc->num_pins);
	return uphy->soc->num_pins;
}

static const char *tegra_padctl_uphy_get_group_name(struct pinctrl_dev *pinctrl,
						    unsigned int group)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);

	TRACE(uphy->dev, "group %u name %s", group,
						uphy->soc->pins[group].name);
	return uphy->soc->pins[group].name;
}

static int tegra_padctl_uphy_get_group_pins(struct pinctrl_dev *pinctrl,
				 unsigned group,
				 const unsigned **pins,
				 unsigned *num_pins)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);

	*pins = &uphy->soc->pins[group].number;
	*num_pins = 1; /* one pin per group */

	TRACE(uphy->dev, "group %u num_pins %u pins[0] %u",
						group, *num_pins, *pins[0]);

	return 0;
}

enum tegra_xusb_padctl_param {
	TEGRA_PADCTL_UPHY_USB3_PORT,
	TEGRA_PADCTL_UPHY_PORT_CAP,
	TEGRA_PADCTL_UPHY_PCIE_CONTROLLER_NUM,
};

static const struct tegra_padctl_uphy_property {
	const char *name;
	enum tegra_xusb_padctl_param param;
} properties[] = {
	{"nvidia,usb3-port", TEGRA_PADCTL_UPHY_USB3_PORT},
	{"nvidia,port-cap", TEGRA_PADCTL_UPHY_PORT_CAP},
	{"nvidia,pcie-controller", TEGRA_PADCTL_UPHY_PCIE_CONTROLLER_NUM},
};

#define TEGRA_XUSB_PADCTL_PACK(param, value) ((param) << 16 | (value))
#define TEGRA_XUSB_PADCTL_UNPACK_PARAM(config) ((config) >> 16)
#define TEGRA_XUSB_PADCTL_UNPACK_VALUE(config) ((config) & 0xffff)

static int tegra186_padctl_uphy_parse_subnode(struct tegra_padctl_uphy *uphy,
					   struct device_node *np,
					   struct pinctrl_map **maps,
					   unsigned int *reserved_maps,
					   unsigned int *num_maps)
{
	unsigned int i, reserve = 0, num_configs = 0;
	unsigned long config, *configs = NULL;
	const char *function, *group;
	struct property *prop;
	int err = 0;
	u32 value;

	err = of_property_read_string(np, "nvidia,function", &function);
	if (err < 0) {
		if (err != -EINVAL)
			return err;

		function = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		err = of_property_read_u32(np, properties[i].name, &value);
		if (err < 0) {
			if (err == -EINVAL)
				continue;

			return err;
		}

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, value);

		err = pinctrl_utils_add_config(uphy->pinctrl, &configs,
					       &num_configs, config);
		if (err < 0)
			return err;
	}

	if (function)
		reserve++;

	if (num_configs)
		reserve++;

	err = of_property_count_strings(np, "nvidia,lanes");
	if (err < 0)
		return err;

	reserve *= err;

	err = pinctrl_utils_reserve_map(uphy->pinctrl, maps, reserved_maps,
					num_maps, reserve);
	if (err < 0)
		return err;

	of_property_for_each_string(np, "nvidia,lanes", prop, group) {
		if (function) {
			err = pinctrl_utils_add_map_mux(uphy->pinctrl, maps,
					reserved_maps, num_maps, group,
					function);
			if (err < 0)
				return err;
		}

		if (num_configs) {
			err = pinctrl_utils_add_map_configs(uphy->pinctrl,
					maps, reserved_maps, num_maps, group,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_GROUP);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int tegra_padctl_uphy_dt_node_to_map(struct pinctrl_dev *pinctrl,
					    struct device_node *parent,
					    struct pinctrl_map **maps,
					    unsigned int *num_maps)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);
	unsigned int reserved_maps = 0;
	struct device_node *np;
	int err;

	*num_maps = 0;
	*maps = NULL;

	for_each_child_of_node(parent, np) {
		err = tegra186_padctl_uphy_parse_subnode(uphy, np, maps,
						      &reserved_maps,
						      num_maps);
		if (err < 0) {
			pr_info("%s %d err %d\n", __func__, __LINE__, err);
			return err;
		}
	}

	return 0;
}

static const struct pinctrl_ops tegra_xusb_padctl_pinctrl_ops = {
	.get_groups_count = tegra_padctl_uphy_get_groups_count,
	.get_group_name = tegra_padctl_uphy_get_group_name,
	.get_group_pins = tegra_padctl_uphy_get_group_pins,
	.dt_node_to_map = tegra_padctl_uphy_dt_node_to_map,
	.dt_free_map = pinctrl_utils_dt_free_map,
};

static int tegra186_padctl_uphy_get_functions_count(struct pinctrl_dev *pinctrl)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);

	TRACE(uphy->dev, "num_functions %u", uphy->soc->num_functions);
	return uphy->soc->num_functions;
}

static const char *
tegra186_padctl_uphy_get_function_name(struct pinctrl_dev *pinctrl,
				    unsigned int function)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);

	TRACE(uphy->dev, "function %u name %s", function,
					uphy->soc->functions[function].name);

	return uphy->soc->functions[function].name;
}

static int tegra186_padctl_uphy_get_function_groups(struct pinctrl_dev *pinctrl,
						 unsigned int function,
						 const char * const **groups,
						 unsigned * const num_groups)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);

	*num_groups = uphy->soc->functions[function].num_groups;
	*groups = uphy->soc->functions[function].groups;

	TRACE(uphy->dev, "function %u *num_groups %u groups %s",
				function, *num_groups, *groups[0]);
	return 0;
}

static int tegra186_padctl_uphy_pinmux_set(struct pinctrl_dev *pinctrl,
					   unsigned int function,
					   unsigned int group)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);
	const struct tegra_padctl_uphy_lane *lane;
	unsigned int i;
	u32 value;

	lane = &uphy->soc->lanes[group];

	TRACE(uphy->dev, "group %u (%s) function %u num_funcs %d",
			group, lane->name, function, lane->num_funcs);

	for (i = 0; i < lane->num_funcs; i++) {
		if (lane->funcs[i] == function)
			break;
	}

	if (i >= lane->num_funcs)
		return -EINVAL;

	TRACE(uphy->dev, "group %s set to function %s",
			lane->name, uphy->soc->functions[function].name);

	if (lane_is_otg(group)) {
		value = padctl_readl(uphy, lane->offset);
		value &= ~(lane->mask << lane->shift);
		value |= (PORT_XUSB << lane->shift);
		padctl_writel(uphy, value, lane->offset);
	} else if (lane_is_hsic(group)) {
		value = padctl_readl(uphy, lane->offset);
		value &= ~(lane->mask << lane->shift);
		value |= (PORT_HSIC << lane->shift);
		padctl_writel(uphy, value, lane->offset);
	} else if (lane_is_uphy(group)) {
		int uphy_lane = group - PIN_UPHY_0;

		if (function == TEGRA186_FUNC_USB3)
			set_bit(uphy_lane, &uphy->usb3_lanes);
		else if (function == TEGRA186_FUNC_PCIE)
			set_bit(uphy_lane, &uphy->pcie_lanes);
		else if (function == TEGRA186_FUNC_SATA)
			set_bit(uphy_lane, &uphy->sata_lanes);
		else if (function == TEGRA186_FUNC_MPHY)
			set_bit(uphy_lane, &uphy->ufs_lanes);
	} else
		return -EINVAL;

	return 0;
}

static const struct pinmux_ops tegra186_padctl_uphy_pinmux_ops = {
	.get_functions_count = tegra186_padctl_uphy_get_functions_count,
	.get_function_name = tegra186_padctl_uphy_get_function_name,
	.get_function_groups = tegra186_padctl_uphy_get_function_groups,
	.set_mux = tegra186_padctl_uphy_pinmux_set,
};

static int tegra_padctl_uphy_pinconf_group_get(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *config)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);
	struct device *dev = uphy->dev;
	enum tegra_xusb_padctl_param param;
	unsigned uphy_lane;
	int value = 0;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(*config);

	TRACE(uphy->dev, "group %u param 0x%x\n", group, param);

	switch (param) {
	case TEGRA_PADCTL_UPHY_USB3_PORT:
		uphy_lane = group - PIN_UPHY_0;
		value = lane_to_usb3_port(uphy, uphy_lane);
		if (value < 0) {
			dev_err(dev, "Pin %d not mapped to USB3 port\n", group);
			return -EINVAL;
		}
		break;

	case TEGRA_PADCTL_UPHY_PCIE_CONTROLLER_NUM:
		uphy_lane = group - PIN_UPHY_0;
		value = lane_to_pcie_controller(uphy, uphy_lane);
		if (value < 0) {
			dev_err(dev, "Pin %d not mapped to PCIE controller\n",
				group);
			return -EINVAL;
		}

		break;
	default:
		dev_err(uphy->dev, "invalid configuration parameter: %04x\n",
			param);
		return -ENOTSUPP;
	}

	*config = TEGRA_XUSB_PADCTL_PACK(param, value);
	return 0;
}

static int tegra_padctl_uphy_pinconf_group_set(struct pinctrl_dev *pinctrl,
					       unsigned int group,
					       unsigned long *configs,
					       unsigned num_configs)
{
	struct tegra_padctl_uphy *uphy = pinctrl_dev_get_drvdata(pinctrl);
	struct device *dev = uphy->dev;
	enum tegra_xusb_padctl_param param;
	unsigned long value;
	unsigned uphy_lane;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(configs[i]);
		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(configs[i]);

		TRACE(dev, "group %u config 0x%lx param 0x%x value 0x%lx",
			group, configs[i], param, value);

		switch (param) {
		case TEGRA_PADCTL_UPHY_USB3_PORT:
			if (value >= TEGRA_USB3_PHYS) {
				dev_err(dev, "Invalid USB3 port: %lu\n", value);
				return -EINVAL;
			}
			if (!lane_is_uphy(group)) {
				dev_err(dev, "USB3 port not applicable for pin %d\n",
					group);
				return -EINVAL;
			}

			/* TODO: make sure lane configuration is valid */
			uphy_lane = group - PIN_UPHY_0;
			TRACE(dev, "USB3 port %lu uses uphy-lane-%u",
			      value, uphy_lane);
			uphy->usb3_ports[value].uphy_lane = uphy_lane;
			break;

		case TEGRA_PADCTL_UPHY_PORT_CAP:
			if (value > TEGRA_PADCTL_PORT_OTG_CAP) {
				dev_err(dev, "Invalid port-cap: %lu\n", value);
				return -EINVAL;
			}
			if (lane_is_uphy(group)) {
				int port;

				uphy_lane = group - PIN_UPHY_0;
				port = lane_to_usb3_port(uphy, uphy_lane);
				if (port < 0) {
					dev_err(dev, "Pin %d not mapped to USB3 port\n",
						group);
					return -EINVAL;
				}
				uphy->usb3_ports[port].port_cap = value;
				TRACE(dev, "USB3 port %d cap %lu",
				      port, value);
			} else if (lane_is_otg(group)) {
				int port = group - PIN_OTG_0;

				uphy->utmi_ports[port].port_cap = value;
				TRACE(dev, "UTMI port %d cap %lu",
				      port, value);
			} else {
				dev_err(dev, "port-cap not applicable for pin %d\n",
					group);
				return -EINVAL;
			}

			break;
		case TEGRA_PADCTL_UPHY_PCIE_CONTROLLER_NUM:
			if (value >= TEGRA_PCIE_PHYS) {
				dev_err(dev, "Invalid PCIE controller: %lu\n",
					value);
				return -EINVAL;
			}
			if (!lane_is_uphy(group)) {
				dev_err(dev,
					"PCIE controller not applicable for pin %d\n",
					group);
				return -EINVAL;
			}

			/* TODO: make sure lane configuration is valid */
			uphy_lane = group - PIN_UPHY_0;
			TRACE(dev, "PCIE controller %lu uses uphy-lane-%u",
			      value, uphy_lane);
			uphy->pcie_controllers[value].uphy_lane_bitmap |=
								BIT(uphy_lane);

			break;

		default:
			dev_err(dev, "invalid configuration parameter: %04x\n",
				param);
			return -ENOTSUPP;
		}
	}
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static const char *strip_prefix(const char *s)
{
	const char *comma = strchr(s, ',');

	if (!comma)
		return s;

	return comma + 1;
}

static void
tegra_padctl_uphy_pinconf_group_dbg_show(struct pinctrl_dev *pinctrl,
					 struct seq_file *s,
					 unsigned int group)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		unsigned long config, value;
		int err;

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, 0);

		err = tegra_padctl_uphy_pinconf_group_get(pinctrl, group,
							  &config);
		if (err < 0)
			continue;

		value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

		seq_printf(s, "\n\t%s=%lu\n", strip_prefix(properties[i].name),
			   value);
	}
}

static void
tegra_padctl_uphy_pinconf_config_dbg_show(struct pinctrl_dev *pinctrl,
					  struct seq_file *s,
					  unsigned long config)
{
	enum tegra_xusb_padctl_param param;
	const char *name = "unknown";
	unsigned long value;
	unsigned int i;

	param = TEGRA_XUSB_PADCTL_UNPACK_PARAM(config);
	value = TEGRA_XUSB_PADCTL_UNPACK_VALUE(config);

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		if (properties[i].param == param) {
			name = properties[i].name;
			break;
		}
	}

	seq_printf(s, "%s=%lu", strip_prefix(name), value);
}
#endif

static const struct pinconf_ops tegra_padctl_uphy_pinconf_ops = {
	.pin_config_group_get = tegra_padctl_uphy_pinconf_group_get,
	.pin_config_group_set = tegra_padctl_uphy_pinconf_group_set,
#ifdef CONFIG_DEBUG_FS
	.pin_config_group_dbg_show = tegra_padctl_uphy_pinconf_group_dbg_show,
	.pin_config_config_dbg_show = tegra_padctl_uphy_pinconf_config_dbg_show,
#endif
};

static int tegra_padctl_uphy_enable(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	mutex_lock(&uphy->lock);
	TRACE(uphy->dev, "enable_counts %d", uphy->enable_counts);

	if (uphy->enable_counts++ > 0)
		goto out;

	dev_info(uphy->dev, "%s FIXME: implement!\n", __func__); /* TODO */
out:
	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra_padctl_uphy_disable(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	mutex_lock(&uphy->lock);
	TRACE(uphy->dev, "enable_counts %d", uphy->enable_counts);

	if (WARN_ON(uphy->enable_counts == 0))
		goto out;

	if (--uphy->enable_counts > 0)
		goto out;

	dev_info(uphy->dev, "%s FIXME: implement!\n", __func__); /* TODO */
out:
	mutex_unlock(&uphy->lock);
	return 0;
}

static int pcie_phy_to_controller(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_PCIE_PHYS; i++) {
		if (phy == uphy->pcie_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_pcie_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;
	unsigned int uphy_lane;
	u32 reg;

	if (controller < 0)
		return controller;

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "power on PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	/* Remove iddq and clamp */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg |= FORCE_IDDQ_DISABLE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	udelay(100);

	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~CLAMP_EN_EARLY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	return 0;
}

static int tegra186_pcie_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;
	unsigned int uphy_lane;
	u32 reg;

	if (controller < 0)
		return controller;

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "power off PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	/* Enable clamp */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg |= CLAMP_EN_EARLY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Enable force IDDQ on lanes */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~FORCE_IDDQ_DISABLE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	return 0;
}

static int tegra186_pcie_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;
	unsigned int uphy_lane;
	u32 reg;

	if (controller < 0)
		return controller;
	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "phy init PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	/* Enable pllp(102M) and plle(100M) */

	/* Program lane ownership by selecting mux to PCIE */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= SEL(~0);
		reg |= SEL_PCIE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Reset release of lanes and PLL1 */
	uphy_pll_reset_deassert(uphy, 0);
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		uphy_lane_reset_deassert(uphy, uphy_lane);
	}

	/* Program pll defaults */
	pcie_usb3_pll_defaults(uphy);

	/* Program lane defaults */
	pcie_lane_defaults(uphy, uphy_lane);

	uphy_pll_init(uphy, 0, TEGRA186_FUNC_PCIE);

	return tegra_padctl_uphy_enable(phy);
}

static int tegra186_pcie_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;
	unsigned int uphy_lane;

	if (controller < 0)
		return controller;

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "phy exit PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	uphy_pll_deinit(uphy, 0);

	/* Assert reset on lanes and pll */
	uphy_pll_reset_assert(uphy, 0);
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		uphy_lane_reset_assert(uphy, uphy_lane);
	}

	/* Disable plle and pllp */

	return tegra_padctl_uphy_disable(phy);
}

static const struct phy_ops pcie_phy_ops = {
	.init = tegra186_pcie_phy_init,
	.exit = tegra186_pcie_phy_exit,
	.power_on = tegra186_pcie_phy_power_on,
	.power_off = tegra186_pcie_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra186_sata_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;
	u32 reg;

	dev_dbg(dev, "power on SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	/* Remove iddq and clamp */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg |= FORCE_IDDQ_DISABLE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	udelay(100);

	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~CLAMP_EN_EARLY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}
	return 0;
}

static int tegra186_sata_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;
	u32 reg;

	dev_dbg(dev, "power off SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	/* Enable clamp */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg |= CLAMP_EN_EARLY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Enable force IDDQ on lanes */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~FORCE_IDDQ_DISABLE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	return 0;
}

static int tegra186_sata_fuse_calibration(struct tegra_padctl_uphy *uphy,
						int lane)
{
	void __iomem *base;
	u32 reg;
	int idx, err;

	if (uphy->sata_bypass_fuse && uphy->prod_list) {
		base = uphy->uphy_lane_regs[lane];
		err = tegra_prod_set_by_name(&base,
					"prod_c_sata", uphy->prod_list);
		if (!err)
			return 0;

		/* In case of err update setting based on fuse */
		dev_warn(uphy->dev,
			"Failed to set sata prod settings, err %d", err);
	}

	/* Update based on fuse_sata_nv_calib[1:0] value */
	idx = SATA_NV_CALIB_0_1(uphy->fuse_calib.sata_nv);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_AUX_CTL_1);
	reg |= AUX_RX_IDLE_TH(sata_data[idx].aux_rx_idle_th);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_AUX_CTL_1);

	/* Update based on fuse_sata_nv_calib[3:2] value TBD */
	idx = SATA_NV_CALIB_2_3(uphy->fuse_calib.sata_nv);

	/* Update based on fuse_sata_mphy_odm_calib[1:0] value */
	idx = SATA_MPHY_ODM_CALIB_0_1(uphy->fuse_calib.sata_mphy_odm);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_1);
	reg |= TX_DRV_AMP_SEL0(sata_data[idx].tx_drv_amp_sel0);
	reg |= TX_DRV_AMP_SEL1(sata_data[idx].tx_drv_amp_sel1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_1);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_4);
	reg |= TX_DRV_POST_SEL0(sata_data[idx].tx_drv_post_sel0);
	reg |= TX_DRV_POST_SEL1(sata_data[idx].tx_drv_post_sel1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_4);

	reg = CFG_ADDR(AE_CTLE_CTRL_ID0);
	reg |= CFG_WDATA(sata_data[idx].ae_ctle_ctrl_id0);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);

	reg = CFG_ADDR(AE_CTLE_CTRL_ID1);
	reg |= CFG_WDATA(sata_data[idx].ae_ctle_ctrl_id1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);

	return 0;
}

static int tegra186_sata_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	unsigned uphy_lane;
	u32 reg;

	dev_dbg(dev, "phy init SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);
	/* Enable pllp(102M) and plle(100M) */

	/* Program lane ownership by selecting mux to SATA */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= SEL(~0);
		reg |= SEL_SATA;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Reset release of lanes and PLL0/PLL1 */
	uphy_pll_reset_deassert(uphy, 0);
	uphy_pll_reset_deassert(uphy, 1);
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		uphy_lane_reset_deassert(uphy, uphy_lane);
	}

	/* Program pll defaults */
	sata_pll_defaults(uphy);

	/* Program lane defaults */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		sata_lane_defaults(uphy, uphy_lane);
	}

	tegra186_sata_fuse_calibration(uphy, uphy->sata_lanes);
	uphy_pll_init(uphy, 1, TEGRA186_FUNC_SATA);

	return tegra_padctl_uphy_enable(phy);
}

static int tegra186_sata_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;

	dev_dbg(dev, "phy exit SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	/* Assert reset on lanes and pll */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		uphy_lane_reset_assert(uphy, uphy_lane);
	}

	uphy_pll_reset_assert(uphy, 1);

	/* Disable plle and pllp */

	uphy_pll_deinit(uphy, 1);

	return tegra_padctl_uphy_disable(phy);
}

static const struct phy_ops sata_phy_ops = {
	.init = tegra186_sata_phy_init,
	.exit = tegra186_sata_phy_exit,
	.power_on = tegra186_sata_phy_power_on,
	.power_off = tegra186_sata_phy_power_off,
	.owner = THIS_MODULE,
};

static int tegra186_ufs_fuse_calibration(struct tegra_padctl_uphy *uphy,
						int lane)
{
	u32 reg;
	int idx;

	/* Update based on fuse_mphy_nv_calib[1:0] value */
	idx = MPHY_NV_CALIB_0_1(uphy->fuse_calib.mphy_nv);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_AUX_CTL_1);
	reg |= AUX_RX_IDLE_TH(mphy_data[idx].aux_rx_idle_th);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_AUX_CTL_1);

	/* Update based on fuse_mphy_nv_calib[3:2] value TBD */
	idx = MPHY_NV_CALIB_2_3(uphy->fuse_calib.mphy_nv);

	/* Update based on fuse_mphy_nv_calib[5:4] value TBD */
	idx = MPHY_NV_CALIB_4_5(uphy->fuse_calib.mphy_nv);

	/* Update based on fuse_sata_mphy_odm_calib[1:0] value */
	idx = SATA_MPHY_ODM_CALIB_0_1(uphy->fuse_calib.sata_mphy_odm);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_1);
	reg |= TX_DRV_AMP_SEL0(mphy_data[idx].tx_drv_amp_sel0);
	reg |= TX_DRV_AMP_SEL1(mphy_data[idx].tx_drv_amp_sel1);
	reg |= TX_DRV_AMP_SEL2(mphy_data[idx].tx_drv_amp_sel2);
	reg |= TX_DRV_AMP_SEL3(mphy_data[idx].tx_drv_amp_sel3);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_1);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_2);
	reg |= TX_DRV_AMP_SEL4(mphy_data[idx].tx_drv_amp_sel4);
	reg |= TX_DRV_AMP_SEL5(mphy_data[idx].tx_drv_amp_sel5);
	reg |= TX_DRV_AMP_SEL6(mphy_data[idx].tx_drv_amp_sel6);
	reg |= TX_DRV_AMP_SEL7(mphy_data[idx].tx_drv_amp_sel7);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_2);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_3);
	reg |= TX_DRV_AMP_SEL8(mphy_data[idx].tx_drv_amp_sel8);
	reg |= TX_DRV_AMP_SEL9(mphy_data[idx].tx_drv_amp_sel9);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_3);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_4);
	reg |= TX_DRV_POST_SEL0(mphy_data[idx].tx_drv_post_sel0);
	reg |= TX_DRV_POST_SEL1(mphy_data[idx].tx_drv_post_sel1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_4);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_5);
	reg |= TX_DRV_POST_SEL2(mphy_data[idx].tx_drv_post_sel2);
	reg |= TX_DRV_POST_SEL3(mphy_data[idx].tx_drv_post_sel3);
	reg |= TX_DRV_PRE_SEL3(mphy_data[idx].tx_drv_pre_sel3);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_5);

	return 0;
}

static int tegra186_ufs_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	dev_dbg(uphy->dev, "power on UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);

	/* step 1.5: Enable pllp(102M) and pllrefe(208M)*/

	/* step 2.1: De-assert UPHY LANE PAD Macro, already done in .probe() */

	/* step 2.2: Program lane ownership by selecting mux to MPHY */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= SEL(~0);
		reg |= SEL_MPHY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* step 2.3: Bring refPLLE to under HW control (optional). */

	/* step 5.3: Reset release of lanes and PLL1. */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		uphy_lane_reset_deassert(uphy, uphy_lane);
	}
	uphy_pll_reset_deassert(uphy, 0);
	uphy_pll_reset_deassert(uphy, 1);

	/* step 6.1: Program pll defaults */
	ufs_pll_defaults(uphy);

	/* step 6.2: Program lane defaults */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		TRACE(uphy->dev, "uphy_lane %u", uphy_lane);
		ufs_lane_defaults(uphy, uphy_lane);
	}

	/* step 6.3: Electrical parameters programming based on fuses */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES)
		tegra186_ufs_fuse_calibration(uphy, uphy_lane);

	/* step 7: Rate id programming */
	ufs_pll_rateid_init(uphy);
	ufs_lane_rateid_init(uphy, uphy->ufs_lanes);

	/* step 8: Uphy pll1 calibration */
	uphy_pll_init(uphy, 1, TEGRA186_FUNC_MPHY);

	/* step 9: Remove iddq and clamp */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg |= FORCE_IDDQ_DISABLE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	udelay(100);

	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~CLAMP_EN_EARLY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	return 0;
}

static int tegra186_ufs_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	unsigned int uphy_lane;
	u32 reg;

	dev_dbg(uphy->dev, "power off UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);

	/* Enable clamp */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg |= CLAMP_EN_EARLY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Enable force IDDQ on lanes */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~FORCE_IDDQ_DISABLE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Assert reset on lanes and pll */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		uphy_lane_reset_assert(uphy, uphy_lane);
	}
	uphy_pll_reset_assert(uphy, 0);
	uphy_pll_reset_assert(uphy, 1);

	/* Disable pllrefe and pllp */

	return 0;
}

static int tegra186_ufs_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	dev_dbg(uphy->dev, "phy init UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);
	return tegra_padctl_uphy_enable(phy);
}

static int tegra186_ufs_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	dev_dbg(uphy->dev, "phy exit UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);
	return tegra_padctl_uphy_disable(phy);
}

static const struct phy_ops ufs_phy_ops = {
	.init = tegra186_ufs_phy_init,
	.exit = tegra186_ufs_phy_exit,
	.power_on = tegra186_ufs_phy_power_on,
	.power_off = tegra186_ufs_phy_power_off,
	.owner = THIS_MODULE,
};

static int usb3_phy_to_port(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_USB3_PHYS; i++) {
		if (phy == uphy->usb3_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_usb3_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	char prod_name[] = "prod_c_ssX";
	unsigned int uphy_lane;
	u32 reg;

	if (port < 0)
		return port;

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(uphy->dev, "power on USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	sprintf(prod_name, "prod_c_ssX%d", port);
	tegra_prod_set_by_name(&uphy->padctl_regs, prod_name, uphy->prod_list);

	reg = padctl_readl(uphy, XUSB_PADCTL_SS_PORT_CAP);
	reg &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(port));
	if (uphy->usb3_ports[port].port_cap == CAP_DISABLED)
		reg |= (PORT_CAP_DISABLED << PORTX_CAP_SHIFT(port));
	else if (uphy->usb3_ports[port].port_cap == DEVICE_ONLY)
		reg |= (PORT_CAP_DEVICE << PORTX_CAP_SHIFT(port));
	else if (uphy->usb3_ports[port].port_cap == HOST_ONLY)
		reg |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(port));
	else if (uphy->usb3_ports[port].port_cap == OTG)
		reg |= (PORT_CAP_OTG << PORTX_CAP_SHIFT(port));
	padctl_writel(uphy, reg, XUSB_PADCTL_SS_PORT_CAP);

	reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
	reg &= SEL(~0);
	reg |= SEL_XUSB;
	reg |= FORCE_IDDQ_DISABLE;
	reg &= ~CLAMP_EN_EARLY;
	uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	return 0;
}

static int tegra186_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	unsigned int uphy_lane;
	u32 reg;

	if (port < 0)
		return port;

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(uphy->dev, "power off USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
	reg &= ~FORCE_IDDQ_DISABLE;
	reg |= CLAMP_EN_EARLY;
	uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(250, 350);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	return 0;
}

static int tegra186_usb3_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = usb3_phy_to_port(phy);
	unsigned uphy_lane;

	if (port < 0)
		return port;

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(uphy->dev, "phy init USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	/* TODO: error handling */
	uphy_pll_reset_deassert(uphy, 0);
	uphy_lane_reset_deassert(uphy, uphy_lane);
	usb3_lane_defaults(uphy, uphy_lane);
	pcie_usb3_pll_defaults(uphy);
	uphy_pll_init(uphy, 0, TEGRA186_FUNC_USB3);

	return tegra_padctl_uphy_enable(phy);
}

static int tegra186_usb3_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	int port = usb3_phy_to_port(phy);
	unsigned uphy_lane;

	if (port < 0)
		return port;

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(dev, "phy exit USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	uphy_pll_reset_assert(uphy, 0);
	uphy_lane_reset_assert(uphy, uphy_lane);
	uphy_pll_deinit(uphy, 0);

	return tegra_padctl_uphy_disable(phy);
}

static const struct phy_ops usb3_phy_ops = {
	.init = tegra186_usb3_phy_init,
	.exit = tegra186_usb3_phy_exit,
	.power_on = tegra186_usb3_phy_power_on,
	.power_off = tegra186_usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static int utmi_phy_to_port(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_UTMI_PHYS; i++) {
		if (phy == uphy->utmi_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_utmi_phy_enable_sleepwalk(struct tegra_padctl_uphy *uphy,
				       int port, enum usb_device_speed speed)
{
	u32 reg;

	/* ensure sleepwalk logic is disabled */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~MASTER_ENABLE;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* ensure sleepwalk logics are in low power mode */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg |= MASTER_CFG_SEL;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* set debounce time */
	reg = ao_readl(uphy, XUSB_AO_USB_DEBOUNCE_DEL);
	reg &= ~UTMIP_LINE_DEB_CNT(~0);
	reg |= UTMIP_LINE_DEB_CNT(1);
	ao_writel(uphy, reg, XUSB_AO_USB_DEBOUNCE_DEL);

	/* ensure fake events of sleepwalk logic are desiabled */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~(FAKE_USBOP_VAL | FAKE_USBON_VAL |
		FAKE_USBOP_EN | FAKE_USBON_EN);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* ensure wake events of sleepwalk logic are not latched */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~LINE_WAKEUP_EN;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* disable wake event triggers of sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_NONE;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* power down the line state detectors of the pad */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_PAD_CFG(port));
	reg |= (USBOP_VAL_PD | USBON_VAL_PD);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_PAD_CFG(port));

	/* save state per speed */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SAVED_STATE(port));
	reg &= ~SPEED(~0);
	if (speed == USB_SPEED_HIGH)
		reg |= UTMI_HS;
	else if (speed == USB_SPEED_FULL)
		reg |= UTMI_FS;
	else if (speed == USB_SPEED_LOW)
		reg |= UTMI_LS;
	else
		reg |= UTMI_RST;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SAVED_STATE(port));

	/* enable the trigger of the sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg |= (WAKE_WALK_EN | LINEVAL_WALK_EN);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* reset the walk pointer and clear the alarm of the sleepwalk logic,
	 * as well as capture the configuration of the USB2.0 pad
	 */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_TRIGGERS(port));
	reg |= (CLR_WALK_PTR | CLR_WAKE_ALARM | CAP_CFG);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_TRIGGERS(port));

	/* setup the pull-ups and pull-downs of the signals during the four
	 * stages of sleepwalk.
	 * if device is connected, program sleepwalk logic to maintain a J and
	 * keep driving K upon seeing remote wake.
	 */
	reg = (USBOP_RPD_A | USBOP_RPD_B | USBOP_RPD_C | USBOP_RPD_D);
	reg |= (USBON_RPD_A | USBON_RPD_B | USBON_RPD_C | USBON_RPD_D);
	if (speed == USB_SPEED_UNKNOWN) {
		reg |= (HIGHZ_A | HIGHZ_B | HIGHZ_C | HIGHZ_D);
	} else if ((speed == USB_SPEED_HIGH) || (speed == USB_SPEED_FULL)) {
		/* J state: D+/D- = high/low, K state: D+/D- = low/high */
		reg |= HIGHZ_A;
		reg |= (AP_A);
		reg |= (AN_B | AN_C | AN_D);
	} else if (speed == USB_SPEED_LOW) {
		/* J state: D+/D- = low/high, K state: D+/D- = high/low */
		reg |= HIGHZ_A;
		reg |= AN_A;
		reg |= (AP_B | AP_C | AP_D);
	}
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK(port));

	/* power up the line state detectors of the pad */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_PAD_CFG(port));
	reg &= ~(USBOP_VAL_PD | USBON_VAL_PD);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_PAD_CFG(port));

	udelay(1);

	/* switch the electric control of the USB2.0 pad to XUSB_AO */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_PAD_CFG(port));
	reg |= (FSLS_USE_XUSB_AO | TRK_CTRL_USE_XUSB_AO |
		RPD_CTRL_USE_XUSB_AO | RPU_USE_XUSB_AO | VREG_USE_XUSB_AO);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_PAD_CFG(port));

	/* set the wake signaling trigger events */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_ANY;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* enable the wake detection */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg |= (MASTER_ENABLE | LINE_WAKEUP_EN);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	return 0;
}

static int tegra186_utmi_phy_disable_sleepwalk(struct tegra_padctl_uphy *uphy,
					       int port)
{
	u32 reg;

	/* disable the wake detection */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~(MASTER_ENABLE | LINE_WAKEUP_EN);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* switch the electric control of the USB2.0 pad to XUSB vcore logic */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_PAD_CFG(port));
	reg &= ~(FSLS_USE_XUSB_AO | TRK_CTRL_USE_XUSB_AO |
		RPD_CTRL_USE_XUSB_AO | RPU_USE_XUSB_AO | VREG_USE_XUSB_AO);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_PAD_CFG(port));

	/* disable wake event triggers of sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_NONE;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(port));

	/* power down the line state detectors of the port */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_PAD_CFG(port));
	reg |= (USBOP_VAL_PD | USBON_VAL_PD);
	ao_writel(uphy, reg, XUSB_AO_UTMIP_PAD_CFG(port));

	/* clear alarm of the sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UTMIP_TRIGGERS(port));
	reg |= CLR_WAKE_ALARM;
	ao_writel(uphy, reg, XUSB_AO_UTMIP_TRIGGERS(port));

	return 0;
}

static int tegra186_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	char prod_name[] = "prod_c_utmiX";
	int err;
	u32 reg;

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "power on UTMI port %d\n",  port);

	sprintf(prod_name, "prod_c_utmi%d", port);
	tegra_prod_set_by_name(&uphy->padctl_regs, prod_name, uphy->prod_list);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_PORT_CAP);
	reg &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(port));
	if (uphy->utmi_ports[port].port_cap == CAP_DISABLED)
		reg |= (PORT_CAP_DISABLED << PORTX_CAP_SHIFT(port));
	else if (uphy->utmi_ports[port].port_cap == DEVICE_ONLY)
		reg |= (PORT_CAP_DEVICE << PORTX_CAP_SHIFT(port));
	else if (uphy->utmi_ports[port].port_cap == HOST_ONLY)
		reg |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(port));
	else if (uphy->utmi_ports[port].port_cap == OTG)
		reg |= (PORT_CAP_OTG << PORTX_CAP_SHIFT(port));
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_PORT_CAP);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	reg &= ~(USB2_OTG_PD | USB2_OTG_PD_ZI);
	reg &= ~HS_CURR_LEVEL(~0);
	reg |= HS_CURR_LEVEL(uphy->calib.hs_curr_level[port]);
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	reg &= ~USB2_OTG_PD_DR;
	reg &= ~TERM_RANGE_ADJ(~0);
	reg |= TERM_RANGE_ADJ(uphy->calib.hs_term_range_adj);
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));

	err = regulator_enable(uphy->vbus[port]);
	if (err) {
		dev_err(uphy->dev, "enable port %d vbus failed %d\n",
			port, err);
		return err;
	}

	mutex_lock(&uphy->lock);

	if (uphy->utmi_enable++ > 0)
		goto out;

	/* BIAS PAD */
	if (uphy->usb2_trk_clk)
		clk_enable(uphy->usb2_trk_clk);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	reg &= ~USB2_TRK_START_TIMER(~0);
	reg |= USB2_TRK_START_TIMER(0x1e);
	reg &= ~USB2_TRK_DONE_RESET_TIMER(~0);
	reg |= USB2_TRK_DONE_RESET_TIMER(0xa);
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	reg &= ~BIAS_PAD_PD;
	reg &= ~HS_SQUELCH_LEVEL(~0);
	reg |= HS_SQUELCH_LEVEL(uphy->calib.hs_squelch);
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	udelay(1);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	reg &= ~USB2_PD_TRK;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	usleep_range(50, 60);

	if (uphy->usb2_trk_clk)
		clk_disable(uphy->usb2_trk_clk);
out:
	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_utmi_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	int rc;
	u32 reg;

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "power off UTMI port %d\n", port);

	rc = regulator_disable(uphy->vbus[port]);
	if (rc) {
		dev_err(uphy->dev, "disable port %d vbus failed %d\n",
			port, rc);
	}
	mutex_lock(&uphy->lock);

	if (WARN_ON(uphy->utmi_enable == 0))
		goto out;

	if (--uphy->utmi_enable > 0)
		goto out;

	/* BIAS pad */
	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	reg |= BIAS_PAD_PD;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	reg |= USB2_PD_TRK;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

out:
	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_utmi_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "phy init UTMI port %d\n",  port);
	return tegra_padctl_uphy_enable(phy);
}

static int tegra186_utmi_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "phy exit UTMI port %d\n",  port);
	return tegra_padctl_uphy_disable(phy);
}

static const struct phy_ops utmi_phy_ops = {
	.init = tegra186_utmi_phy_init,
	.exit = tegra186_utmi_phy_exit,
	.power_on = tegra186_utmi_phy_power_on,
	.power_off = tegra186_utmi_phy_power_off,
	.owner = THIS_MODULE,
};

static inline bool is_utmi_phy(struct phy *phy)
{
	return (phy->ops == &utmi_phy_ops);
}

static int hsic_phy_to_port(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	unsigned int i;

	for (i = 0; i < TEGRA_HSIC_PHYS; i++) {
		if (phy == uphy->hsic_phys[i])
			return i;
	}
	WARN_ON(1);

	return -EINVAL;
}

static int tegra186_hsic_phy_enable_sleepwalk(struct tegra_padctl_uphy *uphy,
					      int port)
{
	u32 reg;

	/* ensure sleepwalk logic is disabled */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~MASTER_ENABLE;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* ensure sleepwalk logics are in low power mode */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg |= MASTER_CFG_SEL;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* set debounce time */
	reg = ao_readl(uphy, XUSB_AO_USB_DEBOUNCE_DEL);
	reg &= ~UHSIC_LINE_DEB_CNT(~0);
	reg |= UHSIC_LINE_DEB_CNT(1);
	ao_writel(uphy, reg, XUSB_AO_USB_DEBOUNCE_DEL);

	/* ensure fake events of sleepwalk logic are desiabled */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~(FAKE_STROBE_VAL | FAKE_DATA_VAL |
		FAKE_STROBE_EN | FAKE_DATA_EN);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* ensure wake events of sleepwalk logic are not latched */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~LINE_WAKEUP_EN;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* disable wake event triggers of sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_NONE;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* power down the line state detectors of the port */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_PAD_CFG(port));
	reg |= (STROBE_VAL_PD | DATA0_VAL_PD);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_PAD_CFG(port));

	/* save state, HSIC always comes up as HS */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SAVED_STATE(port));
	reg &= ~MODE(~0);
	reg |= MODE_HS;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SAVED_STATE(port));

	/* enable the trigger of the sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg |= (WAKE_WALK_EN | LINEVAL_WALK_EN);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* reset the walk pointer and clear the alarm of the sleepwalk logic,
	 * as well as capture the configuration of the USB2.0 port
	 */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_TRIGGERS(port));
	reg |= (HSIC_CLR_WALK_PTR | HSIC_CLR_WAKE_ALARM | HSIC_CAP_CFG);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_TRIGGERS(port));

	/* setup the pull-ups and pull-downs of the signals during the four
	 * stages of sleepwalk.
	 * maintain a HSIC IDLE and keep driving HSIC RESUME upon remote wake
	 */
	reg = (RPD_DATA0_A | RPU_DATA0_B | RPU_DATA0_C | RPU_DATA0_D);
	reg |= (RPU_STROBE_A | RPD_STROBE_B | RPD_STROBE_C | RPD_STROBE_D);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK(port));

	/* power up the line state detectors of the port */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_PAD_CFG(port));
	reg &= ~(DATA0_VAL_PD | STROBE_VAL_PD);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_PAD_CFG(port));

	udelay(1);

	/* switch the electric control of the USB2.0 pad to XUSB_AO */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_PAD_CFG(port));
	reg |= USE_XUSB_AO;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_PAD_CFG(port));

	/* set the wake signaling trigger events */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_DS10;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* enable the wake detection */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg |= (MASTER_ENABLE | LINE_WAKEUP_EN);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	return 0;
}

static int tegra186_hsic_phy_disable_sleepwalk(struct tegra_padctl_uphy *uphy,
					       int port)
{
	u32 reg;

	/* disable the wake detection */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~(MASTER_ENABLE | LINE_WAKEUP_EN);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* switch the electric control of the USB2.0 pad to XUSB vcore logic */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_PAD_CFG(port));
	reg &= ~USE_XUSB_AO;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_PAD_CFG(port));

	/* disable wake event triggers of sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_NONE;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_SLEEPWALK_CFG(port));

	/* power down the line state detectors of the port */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_PAD_CFG(port));
	reg |= (STROBE_VAL_PD | DATA0_VAL_PD);
	ao_writel(uphy, reg, XUSB_AO_UHSIC_PAD_CFG(port));

	/* clear alarm of the sleepwalk logic */
	reg = ao_readl(uphy, XUSB_AO_UHSIC_TRIGGERS(port));
	reg |= HSIC_CLR_WAKE_ALARM;
	ao_writel(uphy, reg, XUSB_AO_UHSIC_TRIGGERS(port));

	return 0;
}

static int tegra186_hsic_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);
	char prod_name[] = "prod_c_hsicX";
	int rc;
	u32 reg;

	dev_dbg(uphy->dev, "power on HSIC port %d\n", port);
	if (port < 0)
		return port;

	sprintf(prod_name, "prod_c_hsic%d", port);
	tegra_prod_set_by_name(&uphy->padctl_regs, prod_name, uphy->prod_list);

	rc = regulator_enable(uphy->vddio_hsic);
	if (rc) {
		dev_err(uphy->dev, "enable hsic %d power failed %d\n",
			port, rc);
		return rc;
	}

	if (uphy->hsic_trk_clk)
		clk_enable(uphy->hsic_trk_clk);

	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PAD_TRK_CTL0);
	reg &= ~HSIC_TRK_START_TIMER(~0);
	reg |= HSIC_TRK_START_TIMER(0x1e);
	reg &= ~HSIC_TRK_DONE_RESET_TIMER(~0);
	reg |= HSIC_TRK_DONE_RESET_TIMER(0xa);
	padctl_writel(uphy, reg, XUSB_PADCTL_HSIC_PAD_TRK_CTL0);

	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PADX_CTL0(port));
	reg &= ~(HSIC_PD_TX_DATA0 | HSIC_PD_TX_STROBE |
		HSIC_PD_RX_DATA0 | HSIC_PD_RX_STROBE |
		HSIC_PD_ZI_DATA0 | HSIC_PD_ZI_STROBE);
	padctl_writel(uphy, reg, XUSB_PADCTL_HSIC_PADX_CTL0(port));

	udelay(1);

	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PAD_TRK_CTL0);
	reg &= ~HSIC_PD_TRK;
	padctl_writel(uphy, reg, XUSB_PADCTL_HSIC_PAD_TRK_CTL0);

	usleep_range(50, 60);

	if (uphy->hsic_trk_clk)
		clk_disable(uphy->hsic_trk_clk);

	return 0;
}

static int tegra186_hsic_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);
	int rc;
	u32 reg;

	dev_dbg(uphy->dev, "power off HSIC port %d\n", port);
	if (port < 0)
		return port;

	rc = regulator_disable(uphy->vddio_hsic);
	if (rc) {
		dev_err(uphy->dev, "disable hsic %d power failed %d\n",
			port, rc);
	}
	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PADX_CTL0(port));
	reg |= (HSIC_PD_TX_DATA0 | HSIC_PD_TX_STROBE |
		HSIC_PD_RX_DATA0 | HSIC_PD_RX_STROBE |
		HSIC_PD_ZI_DATA0 | HSIC_PD_ZI_STROBE);
	padctl_writel(uphy, reg, XUSB_PADCTL_HSIC_PADX_CTL0(port));

	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PAD_TRK_CTL0);
	reg |= HSIC_PD_TRK;
	padctl_writel(uphy, reg, XUSB_PADCTL_HSIC_PAD_TRK_CTL0);

	return 0;
}

static void hsic_phy_set_idle(struct tegra_padctl_uphy *uphy,
			      unsigned int port, bool idle)
{
	dev_dbg(uphy->dev, "set idle HSIC port %d\n", port);
	dev_info(uphy->dev, "%s FIXME: implement!\n", __func__); /* TODO */
}

static int tegra186_hsic_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);

	dev_dbg(uphy->dev, "phy init HSIC port %d\n", port);
	dev_info(uphy->dev, "%s FIXME: implement!\n", __func__); /* TODO */
	return tegra_padctl_uphy_enable(phy);
}

static int tegra186_hsic_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);

	dev_dbg(uphy->dev, "phy init HSIC port %d\n", port);
	dev_info(uphy->dev, "%s FIXME: implement!\n", __func__); /* TODO */
	return tegra_padctl_uphy_disable(phy);
}

static const struct phy_ops hsic_phy_ops = {
	.init = tegra186_hsic_phy_init,
	.exit = tegra186_hsic_phy_exit,
	.power_on = tegra186_hsic_phy_power_on,
	.power_off = tegra186_hsic_phy_power_off,
	.owner = THIS_MODULE,
};

static inline bool is_hsic_phy(struct phy *phy)
{
	return (phy->ops == &hsic_phy_ops);
}

static void tegra_xusb_phy_mbox_work(struct work_struct *work)
{
	struct tegra_padctl_uphy *uphy = mbox_work_to_uphy(work);
	struct tegra_xusb_mbox_msg *msg = &uphy->mbox_req;
	struct tegra_xusb_mbox_msg resp;
	unsigned int i;
	u32 ports;

	dev_dbg(uphy->dev, "mailbox command %d\n", msg->cmd);
	resp.cmd = 0;
	switch (msg->cmd) {
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		ports = msg->data >> (uphy->soc->hsic_port_offset + 1);
		resp.data = msg->data;
		resp.cmd = MBOX_CMD_ACK;
		for (i = 0; i < TEGRA_HSIC_PHYS; i++) {
			if (!(ports & BIT(i)))
				continue;
			if (msg->cmd == MBOX_CMD_START_HSIC_IDLE)
				hsic_phy_set_idle(uphy, i, true);
			else
				hsic_phy_set_idle(uphy, i, false);
		}
		break;
	default:
		break;
	}

	if (resp.cmd)
		mbox_send_message(uphy->mbox_chan, &resp);
}

static bool is_phy_mbox_message(u32 cmd)
{
	switch (cmd) {
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		return true;
	default:
		return false;
	}
}

static void tegra_xusb_phy_mbox_rx(struct mbox_client *cl, void *data)
{
	struct tegra_padctl_uphy *uphy = dev_get_drvdata(cl->dev);
	struct tegra_xusb_mbox_msg *msg = data;

	if (is_phy_mbox_message(msg->cmd)) {
		uphy->mbox_req = *msg;
		schedule_work(&uphy->mbox_req_work);
	}
}

static struct phy *tegra186_padctl_uphy_xlate(struct device *dev,
					   struct of_phandle_args *args)
{
	struct tegra_padctl_uphy *uphy = dev_get_drvdata(dev);
	unsigned int index = args->args[0];
	unsigned int phy_index;
	struct phy *phy = NULL;

	if (args->args_count <= 0)
		return ERR_PTR(-EINVAL);

	dev_dbg(dev, "%s index %d\n", __func__, index);

	if ((index >= TEGRA_PADCTL_UPHY_USB3_BASE) &&
		(index < TEGRA_PADCTL_UPHY_USB3_BASE + 16)) {

		phy_index = index - TEGRA_PADCTL_UPHY_USB3_BASE;
		if (phy_index < TEGRA_USB3_PHYS)
			phy = uphy->usb3_phys[phy_index];

	} else if ((index >= TEGRA_PADCTL_UPHY_UTMI_BASE) &&
		(index < TEGRA_PADCTL_UPHY_UTMI_BASE + 16)) {

		phy_index = index - TEGRA_PADCTL_UPHY_UTMI_BASE;
		if (phy_index < TEGRA_UTMI_PHYS)
			phy = uphy->utmi_phys[phy_index];

	} else if ((index >= TEGRA_PADCTL_UPHY_HSIC_BASE) &&
		(index < TEGRA_PADCTL_UPHY_HSIC_BASE + 16)) {

		phy_index = index - TEGRA_PADCTL_UPHY_HSIC_BASE;
		if (phy_index < TEGRA_HSIC_PHYS)
			phy = uphy->hsic_phys[phy_index];

	} else if ((index >= TEGRA_PADCTL_UPHY_PCIE_BASE) &&
		(index < TEGRA_PADCTL_UPHY_PCIE_BASE + 16)) {

		phy_index = index - TEGRA_PADCTL_UPHY_PCIE_BASE;
		if (phy_index < TEGRA_PCIE_PHYS)
			phy = uphy->pcie_phys[phy_index];

	} else if ((index >= TEGRA_PADCTL_UPHY_SATA_BASE) &&
		(index < TEGRA_PADCTL_UPHY_SATA_BASE + 16)) {

		phy_index = index - TEGRA_PADCTL_UPHY_SATA_BASE;
		if (phy_index < TEGRA_SATA_PHYS)
			phy = uphy->sata_phys[phy_index];

	} else if ((index >= TEGRA_PADCTL_UPHY_UFS_BASE) &&
		(index < TEGRA_PADCTL_UPHY_UFS_BASE + 16)) {

		phy_index = index - TEGRA_PADCTL_UPHY_UFS_BASE;
		if (phy_index < TEGRA_UFS_PHYS)
			phy = uphy->ufs_phys[phy_index];

	}

	return (phy) ? phy : ERR_PTR(-EINVAL);
}

static const struct pinctrl_pin_desc tegra186_pins[] = {
	PINCTRL_PIN(PIN_OTG_0,  "otg-0"),
	PINCTRL_PIN(PIN_OTG_1,  "otg-1"),
	PINCTRL_PIN(PIN_OTG_2,  "otg-2"),
	PINCTRL_PIN(PIN_HSIC_0, "hsic-0"),
	PINCTRL_PIN(PIN_UPHY_0, "uphy-lane-0"),
	PINCTRL_PIN(PIN_UPHY_1, "uphy-lane-1"),
	PINCTRL_PIN(PIN_UPHY_2, "uphy-lane-2"),
	PINCTRL_PIN(PIN_UPHY_3, "uphy-lane-3"),
	PINCTRL_PIN(PIN_UPHY_4, "uphy-lane-4"),
	PINCTRL_PIN(PIN_UPHY_5, "uphy-lane-5"),
};


static const char * const tegra186_hsic_groups[] = {
	"hsic-0",
};

static const char * const tegra186_hsic_plus_groups[] = {
	"hsic-0",
};

static const char * const tegra186_xusb_groups[] = {
	"otg-0",
	"otg-1",
	"otg-2",
};

static const char * const tegra186_pcie_groups[] = {
	"uphy-lane-0",
	"uphy-lane-1",
	"uphy-lane-2",
	"uphy-lane-3",
	"uphy-lane-4",
};

static const char * const tegra186_usb3_groups[] = {
	"uphy-lane-0",
	"uphy-lane-1",
	"uphy-lane-2",
};

static const char * const tegra186_sata_groups[] = {
	"uphy-lane-5",
};

static const char * const tegra186_mphy_groups[] = {
	"uphy-lane-4",
	"uphy-lane-5",
};

#define TEGRA186_FUNCTION(_name)					\
	{								\
		.name = #_name,						\
		.num_groups = ARRAY_SIZE(tegra186_##_name##_groups),	\
		.groups = tegra186_##_name##_groups,			\
	}

static struct tegra_padctl_uphy_function tegra186_functions[] = {
	TEGRA186_FUNCTION(hsic),
	TEGRA186_FUNCTION(xusb),
	TEGRA186_FUNCTION(pcie),
	TEGRA186_FUNCTION(usb3),
	TEGRA186_FUNCTION(sata),
	TEGRA186_FUNCTION(mphy),
};

static const unsigned int tegra186_otg_functions[] = {
	TEGRA186_FUNC_XUSB,
};

static const unsigned int tegra186_hsic_functions[] = {
	TEGRA186_FUNC_HSIC,
};

static const unsigned int tegra186_uphy_functions[] = {
	TEGRA186_FUNC_USB3,
	TEGRA186_FUNC_PCIE,
	TEGRA186_FUNC_SATA,
	TEGRA186_FUNC_MPHY,
};

#define TEGRA186_LANE(_name, _offset, _shift, _mask, _funcs)	\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.num_funcs = ARRAY_SIZE(tegra186_##_funcs##_functions),	\
		.funcs = tegra186_##_funcs##_functions,			\
	}

static const struct tegra_padctl_uphy_lane tegra186_lanes[] = {
	TEGRA186_LANE("otg-0",  0x004,  0, 0x3, otg),
	TEGRA186_LANE("otg-1",  0x004,  2, 0x3, otg),
	TEGRA186_LANE("otg-2",  0x004,  4, 0x3, otg),
	TEGRA186_LANE("hsic-0", 0x004, 20, 0x1, hsic),
	TEGRA186_LANE("uphy-lane-0", 0x284, 0, 0x3, uphy),
	TEGRA186_LANE("uphy-lane-1", 0x284, 0, 0x3, uphy),
	TEGRA186_LANE("uphy-lane-2", 0x284, 0, 0x3, uphy),
	TEGRA186_LANE("uphy-lane-3", 0x284, 0, 0x3, uphy),
	TEGRA186_LANE("uphy-lane-4", 0x284, 0, 0x3, uphy),
	TEGRA186_LANE("uphy-lane-5", 0x284, 0, 0x3, uphy),
};

static const struct tegra_padctl_uphy_soc tegra186_soc = {
	.num_pins = ARRAY_SIZE(tegra186_pins),
	.pins = tegra186_pins,
	.num_functions = ARRAY_SIZE(tegra186_functions),
	.functions = tegra186_functions,
	.num_lanes = ARRAY_SIZE(tegra186_lanes),
	.lanes = tegra186_lanes,
	.hsic_port_offset = 6,
};

static const struct of_device_id tegra_padctl_uphy_of_match[] = {
	{.compatible = "nvidia,tegra186-padctl-uphy", .data = &tegra186_soc},
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_padctl_uphy_of_match);

static int tegra_xusb_read_fuse_calibration(struct tegra_padctl_uphy *uphy)
{
	unsigned int i;
	u32 reg;

	reg = tegra_fuse_readl(FUSE_SKU_USB_CALIB_0);
	dev_info(uphy->dev, "FUSE_SKU_USB_CALIB_0 0x%x\n", reg);
	for (i = 0; i < TEGRA_UTMI_PHYS; i++) {
		uphy->calib.hs_curr_level[i] =
			(reg >> HS_CURR_LEVEL_PADX_SHIFT(i)) &
			HS_CURR_LEVEL_PAD_MASK;
	}
	uphy->calib.hs_squelch = (reg >> HS_SQUELCH_SHIFT) & HS_SQUELCH_MASK;
	uphy->calib.hs_term_range_adj = (reg >> HS_TERM_RANGE_ADJ_SHIFT) &
					HS_TERM_RANGE_ADJ_MASK;


	reg = tegra_fuse_readl(FUSE_USB_CALIB_EXT_0);
	dev_info(uphy->dev, "FUSE_USB_CALIB_EXT_0 0x%x\n", reg);
	uphy->calib.rpd_ctrl = (reg >> RPD_CTRL_SHIFT) & RPD_CTRL_MASK;

	return 0;
}

static int tegra_mphy_sata_fuse_calibration(struct tegra_padctl_uphy *padctl)
{
	u32 value;

	value = tegra_fuse_readl(FUSE_SATA_MPHY_ODM_CALIB_0);
	padctl->fuse_calib.sata_mphy_odm = value;

	value = tegra_fuse_readl(FUSE_SATA_NV_CALIB_0);
	padctl->fuse_calib.sata_nv = value;

	value = tegra_fuse_readl(FUSE_MPHY_NV_CALIB_0);
	padctl->fuse_calib.mphy_nv = value;

	return 0;
}

static int tegra_xusb_setup_usb(struct tegra_padctl_uphy *uphy)
{
	struct phy *phy;
	unsigned int i;

	for (i = 0; i < TEGRA_USB3_PHYS; i++) {
		phy = devm_phy_create(uphy->dev, NULL, &usb3_phy_ops, NULL);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		uphy->usb3_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	for (i = 0; i < TEGRA_UTMI_PHYS; i++) {
		char reg_name[sizeof("vbus-N")];

		sprintf(reg_name, "vbus-%d", i);
		uphy->vbus[i] = devm_regulator_get(uphy->dev, reg_name);
		if (IS_ERR(uphy->vbus[i]))
			return PTR_ERR(uphy->vbus[i]);

		phy = devm_phy_create(uphy->dev, NULL, &utmi_phy_ops, NULL);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		uphy->utmi_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	uphy->vddio_hsic = devm_regulator_get(uphy->dev, "vddio_hsic");
	if (IS_ERR(uphy->vddio_hsic))
		return PTR_ERR(uphy->vddio_hsic);

	for (i = 0; i < TEGRA_HSIC_PHYS; i++) {
		phy = devm_phy_create(uphy->dev, NULL, &hsic_phy_ops, NULL);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		uphy->hsic_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	return 0;
}

#ifdef DEBUG
#define reg_dump(_dev, _base, _reg) \
	dev_dbg(_dev, "%s @%x = 0x%x\n", #_reg, _reg, ioread32(_base + _reg))
#else
#define reg_dump(_dev, _base, _reg)	do {} while (0)
#endif

static int tegra186_padctl_uphy_probe(struct platform_device *pdev)
{
	struct tegra_padctl_uphy *uphy;
	const struct of_device_id *match;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct phy *phy;
	int i;
	int err;

	uphy = devm_kzalloc(dev, sizeof(*uphy), GFP_KERNEL);
	if (!uphy)
		return -ENOMEM;

	platform_set_drvdata(pdev, uphy);
	mutex_init(&uphy->lock);
	uphy->dev = dev;

	match = of_match_node(tegra_padctl_uphy_of_match, pdev->dev.of_node);
	if (!match)
		return -ENODEV;
	uphy->soc = match->data;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "padctl");
	uphy->padctl_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(uphy->padctl_regs))
		return PTR_ERR(uphy->padctl_regs);
	dev_info(dev, "padctl mmio start %pa end %pa\n",
		 &res->start, &res->end);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ao");
	uphy->ao_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(uphy->ao_regs))
		return PTR_ERR(uphy->ao_regs);
	dev_info(dev, "ao mmio start %pa end %pa\n", &res->start, &res->end);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "uphy");
	uphy->uphy_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(uphy->uphy_regs))
		return PTR_ERR(uphy->uphy_regs);
	dev_info(dev, "uphy mmio start %pa end %pa\n", &res->start, &res->end);

	uphy->uphy_pll_regs[0] = uphy->uphy_regs;
	uphy->uphy_lane_regs[0] = uphy->uphy_regs + 0x20000;
	uphy->uphy_lane_regs[1] = uphy->uphy_regs + 0x30000;
	uphy->uphy_lane_regs[2] = uphy->uphy_regs + 0x40000;
	uphy->uphy_lane_regs[3] = uphy->uphy_regs + 0x50000;
	uphy->uphy_lane_regs[4] = uphy->uphy_regs + 0x60000;
	uphy->uphy_lane_regs[5] = uphy->uphy_regs + 0x70000;
	uphy->uphy_pll_regs[1] = uphy->uphy_regs + 0x80000;

	if (tegra_platform_is_silicon()) {
		err = tegra_xusb_read_fuse_calibration(uphy);
		if (err < 0)
			return err;

		err = tegra_mphy_sata_fuse_calibration(uphy);
		if (err < 0)
			return err;
	}

	for (i = 0; i < T186_UPHY_PLLS; i++) {
		char rst_name[] = "uphy_pllX_rst";

		snprintf(rst_name, sizeof(rst_name), "uphy_pll%d_rst", i);
		uphy->uphy_pll_rst[i] = devm_reset_control_get(dev, rst_name);
		if (IS_ERR(uphy->uphy_pll_rst[i])) {
			dev_err(uphy->dev, "fail get uphy_pll%d reset\n", i);
			return PTR_ERR(uphy->uphy_pll_rst[i]);
		}
	}

	for (i = 0; i < T186_UPHY_LANES; i++) {
		char rst_name[] = "uphy_laneX_rst";

		snprintf(rst_name, sizeof(rst_name), "uphy_lane%d_rst", i);
		uphy->uphy_lane_rst[i] = devm_reset_control_get(dev, rst_name);
		if (IS_ERR(uphy->uphy_lane_rst[i])) {
			dev_err(uphy->dev, "fail get uphy_lanel%d reset\n", i);
			return PTR_ERR(uphy->uphy_lane_rst[i]);
		}
	}

	uphy->uphy_master_rst = devm_reset_control_get(dev, "uphy_master_rst");
	if (IS_ERR(uphy->uphy_master_rst)) {
		dev_err(uphy->dev, "failed to get uphy master reset\n");
		return PTR_ERR(uphy->uphy_master_rst);
	}

	uphy->uphy_rst = devm_reset_control_get(dev, "uphy_rst");
	if (IS_ERR(uphy->uphy_rst)) {
		dev_err(uphy->dev, "failed to get uphy pad reset\n");
		return PTR_ERR(uphy->uphy_rst);
	}

	uphy->usb2_trk_clk = devm_clk_get(dev, "usb2_trk");
	if (IS_ERR(uphy->usb2_trk_clk)) {
		dev_err(dev, "failed to get usb2_trk clock\n");
		return PTR_ERR(uphy->usb2_trk_clk);
	}

	uphy->hsic_trk_clk = devm_clk_get(dev, "hsic_trk");
	if (IS_ERR(uphy->hsic_trk_clk)) {
		dev_err(dev, "failed to get hsic_trk clock\n");
		return PTR_ERR(uphy->hsic_trk_clk);
	}

	reset_control_deassert(uphy->uphy_master_rst);
	reset_control_deassert(uphy->uphy_rst);

	memset(&uphy->desc, 0, sizeof(uphy->desc));
	uphy->desc.name = dev_name(dev);
	uphy->desc.pins = uphy->soc->pins;
	uphy->desc.npins = uphy->soc->num_pins;
	uphy->desc.pctlops = &tegra_xusb_padctl_pinctrl_ops;
	uphy->desc.pmxops = &tegra186_padctl_uphy_pinmux_ops;
	uphy->desc.confops = &tegra_padctl_uphy_pinconf_ops;
	uphy->desc.owner = THIS_MODULE;

	uphy->pinctrl = pinctrl_register(&uphy->desc, &pdev->dev, uphy);
	if (!uphy->pinctrl) {
		dev_err(&pdev->dev, "failed to register pinctrl\n");
		err = -ENODEV;
		goto assert_uphy_reset;
	}

	for (i = 0; i < TEGRA_PCIE_PHYS; i++) {
		phy = devm_phy_create(dev, NULL, &pcie_phy_ops, NULL);
		if (IS_ERR(phy)) {
			err = PTR_ERR(phy);
			goto unregister;
		}
		uphy->pcie_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	phy = devm_phy_create(dev, NULL, &sata_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}
	uphy->sata_phys[0] = phy;
	phy_set_drvdata(phy, uphy);

	phy = devm_phy_create(dev, NULL, &ufs_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto unregister;
	}
	uphy->ufs_phys[0] = phy;
	phy_set_drvdata(phy, uphy);

	INIT_WORK(&uphy->mbox_req_work, tegra_xusb_phy_mbox_work);
	uphy->mbox_client.dev = dev;
	uphy->mbox_client.tx_block = true;
	uphy->mbox_client.tx_tout = 0;
	uphy->mbox_client.rx_callback = tegra_xusb_phy_mbox_rx;
	uphy->mbox_chan = mbox_request_channel(&uphy->mbox_client, 0);
	if (IS_ERR(uphy->mbox_chan)) {
		err = PTR_ERR(uphy->mbox_chan);
		if (err == -EPROBE_DEFER) {
			dev_info(&pdev->dev, "mailbox is not ready yet");
			goto unregister;
		} else {
			dev_warn(&pdev->dev,
				 "failed to get mailbox, USB support disabled");
		}
	} else {
		err = tegra_xusb_setup_usb(uphy);
		if (err)
			goto unregister;
	}

	uphy->provider = devm_of_phy_provider_register(dev,
					tegra186_padctl_uphy_xlate);
	if (IS_ERR(uphy->provider)) {
		err = PTR_ERR(uphy->provider);
		dev_err(&pdev->dev, "failed to register PHYs: %d\n", err);
		goto unregister;
	}

	uphy->prod_list = tegra_prod_init(pdev->dev.of_node);
	if (IS_ERR(uphy->prod_list)) {
		dev_warn(&pdev->dev, "Prod-settings not available\n");
		uphy->prod_list = NULL;
	}

	uphy->sata_bypass_fuse =
		of_property_read_bool(np, "nvidia,sata-use-prods");

	return 0;

unregister:
	pinctrl_unregister(uphy->pinctrl);
assert_uphy_reset:
	reset_control_assert(uphy->uphy_master_rst);
	reset_control_assert(uphy->uphy_rst);
	return err;
}

static int tegra186_padctl_uphy_remove(struct platform_device *pdev)
{
	struct tegra_padctl_uphy *uphy = platform_get_drvdata(pdev);

	if (!IS_ERR(uphy->mbox_chan)) {
		cancel_work_sync(&uphy->mbox_req_work);
		mbox_free_channel(uphy->mbox_chan);
	}

	pinctrl_unregister(uphy->pinctrl);
	reset_control_assert(uphy->uphy_master_rst);
	reset_control_assert(uphy->uphy_rst);
	if (uphy->prod_list)
		tegra_prod_release(&uphy->prod_list);
	return 0;
}

static struct platform_driver tegra186_padctl_uphy_driver = {
	.driver = {
		.name = "tegra186-padctl-uphy",
		.of_match_table = tegra_padctl_uphy_of_match,
	},
	.probe = tegra186_padctl_uphy_probe,
	.remove = tegra186_padctl_uphy_remove,
};
module_platform_driver(tegra186_padctl_uphy_driver);

/* Tegra Generic PHY Extensions */
int tegra_phy_xusb_enable_sleepwalk(struct phy *phy,
				    enum usb_device_speed speed)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port;

	if (is_utmi_phy(phy)) {
		port = utmi_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_utmi_phy_enable_sleepwalk(uphy, port, speed);
	} else if (is_hsic_phy(phy)) {
		port = hsic_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_hsic_phy_enable_sleepwalk(uphy, port);
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_enable_sleepwalk);

int tegra_phy_xusb_disable_sleepwalk(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port;

	if (is_utmi_phy(phy)) {
		port = utmi_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_utmi_phy_disable_sleepwalk(uphy, port);
	} else if (is_hsic_phy(phy)) {
		port = hsic_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_hsic_phy_disable_sleepwalk(uphy, port);
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_disable_sleepwalk);

MODULE_AUTHOR("JC Kuo <jckuo@nvidia.com>");
MODULE_DESCRIPTION("Tegra 186 XUSB PADCTL and UPHY PLL/Lane driver");
MODULE_LICENSE("GPL v2");
