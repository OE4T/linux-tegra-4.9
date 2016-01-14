/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/slab.h>
#include <linux/usb.h>
#include <soc/tegra/xusb.h>
#include <linux/tegra_prod.h>
#include <dt-bindings/pinctrl/pinctrl-tegra-padctl-uphy.h>

#define VERBOSE_DEBUG
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
#define   AUX_TX_RDET_STATUS		(0x1 << 7)

#define UPHY_LANE_DIRECT_CTL_1			(0x10)
#define   MISC_CTRL(x)				(((x) & 0xff) << 0)
#define   MISC_OUT(x)				(((x) & 0xff) << 16)

#define UPHY_LANE_DIRECT_CTL_2			(0x14)
#define   CFG_WDATA(x)				(((x) & 0xffff) << 0)
#define   CFG_WDATA_0(x)			(((x) & 0x1) << 0)
#define   CFG_WDATA_1_2(x)			(((x) & 0x2) << 1)
#define   CFG_WDATA_3_4(x)			(((x) & 0x2) << 3)
#define   CFG_ADDR(x)				(((x) & 0xff) << 16)
#define   CFG_WDS                            (1 << 24)
#define   CFG_RDS                            (1 << 25)
#define   CFG_RESET                          (1 << 27)

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

#define UPHY_LANE_MPHY_CTL_1	0x240
#define   TX_BYP_MODE(x)	(((x) & 0x3) << 4)
#define   TX_RATE_PDIV(x)	(((x) & 0x3) << 1)

#define UPHY_LANE_MPHY_CTL_2	0x244
#define   RX_BYP_MODE(x)	(((x) & 0x3) << 4)
#define   RX_RATE_PDIV(x)	(((x) & 0x3) << 1)

#define UPHY_LANE_MISC_CTL_2	0x8
#define   RX_BYP_REFCLK_EN	(1 << 11)

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
#define RPD_CTRL_MASK				(0x1f)

/* FUSE SATA MPHY registers */
#define FUSE_SATA_MPHY_ODM_CALIB_0	(0x224)
#define    SATA_MPHY_ODM_CALIB_0_1(x)		(((x) & 0x3) << 0)

#define FUSE_SATA_NV_CALIB_0		(0x49c)
#define    SATA_NV_CALIB_0_1(x)			(((x) & (0x3 << 0)) >> 0)
#define    SATA_NV_CALIB_2_3(x)			(((x) & (0x3 << 2)) >> 2)

#define FUSE_MPHY_NV_CALIB_0		(0x4a0)
#define    MPHY_NV_CALIB_0_1(x)			(((x) & (0x3 << 0)) >> 0)
#define    MPHY_NV_CALIB_2_3(x)			(((x) & (0x3 << 2)) >> 2)
#define    MPHY_NV_CALIB_4(x)			(((x) & (0x1 << 4)) >> 4)

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
#define   USB2_PORT_WAKE_INTERRUPT_ENABLE(x)	(1 << (x))
#define   USB2_PORT_WAKEUP_EVENT(x)		(1 << ((x) + 7))
#define   SS_PORT_WAKE_INTERRUPT_ENABLE(x)	(1 << ((x) + 14))
#define   SS_PORT_WAKEUP_EVENT(x)		(1 << ((x) + 21))
#define   USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(x)	(1 << ((x) + 28))
#define   USB2_HSIC_PORT_WAKEUP_EVENT(x)	(1 << ((x) + 30))
#define   ALL_WAKE_EVENTS						\
	(USB2_PORT_WAKEUP_EVENT(0) | USB2_PORT_WAKEUP_EVENT(1) |	\
	USB2_PORT_WAKEUP_EVENT(2) | SS_PORT_WAKEUP_EVENT(0) |		\
	SS_PORT_WAKEUP_EVENT(1) | SS_PORT_WAKEUP_EVENT(2) |		\
	USB2_HSIC_PORT_WAKEUP_EVENT(0))

#define XUSB_PADCTL_ELPG_PROGRAM_1		(0x24)
#define   SSPX_ELPG_CLAMP_EN(x)			(1 << (0 + (x) * 3))
#define   SSPX_ELPG_CLAMP_EN_EARLY(x)		(1 << (1 + (x) * 3))
#define   SSPX_ELPG_VCORE_DOWN(x)		(1 << (2 + (x) * 3))

#define USB2_BATTERY_CHRG_OTGPADX_CTL0(x)	(0x80 + (x) * 0x40)
#define   PD_CHG				(1 << 0)

#define USB2_BATTERY_CHRG_OTGPADX_CTL1(x)	(0x84 + (x) * 0x40)
#define   VREG_LEV(x)				(((x) & 0x3) << 7)
#define   VREG_FIX18				(1 << 6)

#define XUSB_PADCTL_USB2_OTG_PADX_CTL0(x)	(0x88 + (x) * 0x40)
#define   HS_CURR_LEVEL(x)			((x) & 0x3f)
#define   TERM_SEL				(1 << 25)
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
#define   ID_OVERRIDE(x)			(((x) & 0xf) << 18)
#define   ID_OVERRIDE_FLOATING			ID_OVERRIDE(8)
#define   ID_OVERRIDE_GROUNDED			ID_OVERRIDE(0)

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

	const char * const *supply_names;
	unsigned int num_supplies;
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
	int hs_curr_level_offset; /* deal with platform design deviation */
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

struct tegra_xusb_hsic_port {
	bool pretend_connected;
};

enum uphy_pll_state {
	UPHY_PLL_POWER_DOWN = 0,
	UPHY_PLL_POWER_UP_PARTIAL,
	UPHY_PLL_POWER_UP_FULL,
	UPHY_PLL_POWER_UP_HW_SEQ,
};

static const char * const uphy_pll_states[] = {
	"UPHY_PLL_POWER_DOWN",
	"UPHY_PLL_POWER_UP_PARTIAL",
	"UPHY_PLL_POWER_UP_FULL",
	"UPHY_PLL_POWER_UP_HW_SEQ"
};

enum source_pll_state {
	PLL_POWER_DOWN = 0,
	PLL_POWER_UP_SW_CTL,
	PLL_POWER_UP_HW_SEQ, /* only valid for plle */
};

static const char * const source_pll_states[] = {
	"PLL_POWER_DOWN",
	"PLL_POWER_UP_SW_CTL",
	"PLL_POWER_UP_HW_SEQ",
};

struct padctl_context {
	u32 vbus_id;
};

struct tegra_padctl_uphy {
	struct device *dev;
	void __iomem *padctl_regs;
	void __iomem *ao_regs;
	void __iomem *uphy_regs;
	void __iomem *uphy_pll_regs[T186_UPHY_PLLS];
	void __iomem *uphy_lane_regs[T186_UPHY_LANES];

	struct reset_control *padctl_rst;
	struct reset_control *uphy_rst;

	struct clk *xusb_clk; /* xusb main clock */
	struct clk *pllp; /* pllp, uphy management clock */
	struct clk *plle; /* plle in software control state */
	struct clk *plle_pwrseq; /* plle in hardware power sequencer control */
	struct clk *pllrefe_pex;
	struct clk *plle_passthrough;
	struct clk *pllrefe; /* alternate pll1 parent */
	struct clk *utmipll; /* utmi pads */
	struct clk *usb2_trk_clk; /* utmi tracking circuit clock */
	struct clk *hsic_trk_clk; /* hsic tracking circuit clock */
	struct clk *rx_byp_clk; /* rx bypass clock */
	struct clk *uphy_pll_mgmt[T186_UPHY_PLLS];
	struct clk *uphy_pll_pwrseq[T186_UPHY_PLLS];

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
	struct tegra_xusb_hsic_port hsic_ports[TEGRA_HSIC_PHYS];
	struct tegra_xusb_utmi_port utmi_ports[TEGRA_UTMI_PHYS];
	int utmi_otg_port_base_1; /* one based utmi port number */
	struct tegra_xusb_usb3_port usb3_ports[TEGRA_USB3_PHYS];
	int usb3_otg_port_base_1; /* one based usb3 port number */
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

	bool host_mode_phy_disabled; /* set true if mailbox is not available */
	unsigned int utmi_enable;
	/* TODO: should move to host controller driver? */
	struct regulator *vbus[TEGRA_UTMI_PHYS];
	struct regulator *vddio_hsic;

	unsigned long uphy_pll_users[T186_UPHY_PLLS];
	enum uphy_pll_state uphy_pll_state[T186_UPHY_PLLS];
	enum source_pll_state plle_state;
	enum source_pll_state pllrefe_state;
	enum source_pll_state pll_mgmt_state[T186_UPHY_PLLS];

	int uphy_pll_clients[T186_UPHY_PLLS];
	struct reset_control *uphy_pll_rst[T186_UPHY_PLLS];
	struct reset_control *uphy_lane_rst[T186_UPHY_LANES];
	struct reset_control *uphy_master_rst;

	/* vbus/id based OTG */
	struct work_struct otg_vbus_work;
	bool otg_vbus_on;
	bool otg_vbus_alwayson;

	struct regulator_bulk_data *supplies;
	struct padctl_context padctl_context;
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
	},
	{
		.aux_rx_idle_th = 0x1,
		.tx_drv_amp_sel0 = 0x17,
		.tx_drv_amp_sel1 = 0x1b,
		.tx_drv_post_sel0 = 0x5,
		.tx_drv_post_sel1 = 0xa,
	},
	{
		.aux_rx_idle_th = 0x2,
		.tx_drv_amp_sel0 = 0x13,
		.tx_drv_amp_sel1 = 0x17,
		.tx_drv_post_sel0 = 0x4,
		.tx_drv_post_sel1 = 0xa,
	},
	{
		.aux_rx_idle_th = 0x3,
		.tx_drv_amp_sel0 = 0x1f,
		.tx_drv_amp_sel1 = 0x23,
		.tx_drv_post_sel0 = 0xa,
		.tx_drv_post_sel1 = 0xe,
	},
};

struct init_data {
	u8 cfg_addr;
	u16 cfg_wdata;
};

static struct init_data usb3_pll_g1_init_data[] = {
	{.cfg_addr = 0x2,  .cfg_wdata = 0x0000},
	{.cfg_addr = 0x3,  .cfg_wdata = 0x7051},
};

static void pcie_usb3_pll_defaults(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(usb3_pll_g1_init_data); i++) {
		reg = CFG_ADDR(usb3_pll_g1_init_data[i].cfg_addr);
		reg |= CFG_WDATA(usb3_pll_g1_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_pll_writel(uphy, 0, reg, UPHY_PLL_CTL_4);
	}
}

#define pcie_pll_init pcie_usb3_pll_defaults

static struct init_data sata_pll_g1_g2_g3_init_data[] = {
	{.cfg_addr = 0x0,  .cfg_wdata = 0x001E},
	{.cfg_addr = 0x2,  .cfg_wdata = 0x0000},
	{.cfg_addr = 0x3,  .cfg_wdata = 0x7001},
};

static void sata_pll_defaults(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_pll_g1_g2_g3_init_data); i++) {
		reg = CFG_ADDR(sata_pll_g1_g2_g3_init_data[i].cfg_addr);
		reg |= CFG_WDATA(sata_pll_g1_g2_g3_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_pll_writel(uphy, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_pll_g1_g2_g3_A_B_init_data[] = {
	{.cfg_addr = 0x0,  .cfg_wdata = 0x0041},
	{.cfg_addr = 0x1,  .cfg_wdata = 0x004C},
	{.cfg_addr = 0x2,  .cfg_wdata = 0x0001},
	{.cfg_addr = 0x3,  .cfg_wdata = 0x7001},
	{.cfg_addr = 0x4,  .cfg_wdata = 0x3001},
	{.cfg_addr = 0x13, .cfg_wdata = 0x0002},
	{.cfg_addr = 0x16, .cfg_wdata = 0x162A},
	{.cfg_addr = 0x17, .cfg_wdata = 0x162A},
	{.cfg_addr = 0x19, .cfg_wdata = 0x001F},
	{.cfg_addr = 0x1C, .cfg_wdata = 0x160E},
	{.cfg_addr = 0x1D, .cfg_wdata = 0x160E},
};

static void ufs_pll_defaults(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_pll_g1_g2_g3_A_B_init_data); i++) {
		reg = CFG_ADDR(ufs_pll_g1_g2_g3_A_B_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_pll_g1_g2_g3_A_B_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_pll_writel(uphy, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_pll_rateid_init_data[] = {
	{.cfg_addr = 0x00, .cfg_wdata = 0x0041},
	{.cfg_addr = 0x01, .cfg_wdata = 0x004c},
	{.cfg_addr = 0x02, .cfg_wdata = 0x0001},
	{.cfg_addr = 0x03, .cfg_wdata = 0x7001},
	{.cfg_addr = 0x04, .cfg_wdata = 0x3001},
	{.cfg_addr = 0x05, .cfg_wdata = 0x6454},
	{.cfg_addr = 0x06, .cfg_wdata = 0x6454},
	{.cfg_addr = 0x0a, .cfg_wdata = 0x0487},
	{.cfg_addr = 0x13, .cfg_wdata = 0x0002},
	{.cfg_addr = 0x14, .cfg_wdata = 0x0002},
	{.cfg_addr = 0x15, .cfg_wdata = 0x0070},
	{.cfg_addr = 0x16, .cfg_wdata = 0x162a},
	{.cfg_addr = 0x17, .cfg_wdata = 0x162a},
	{.cfg_addr = 0x18, .cfg_wdata = 0x001f},
	{.cfg_addr = 0x19, .cfg_wdata = 0x001f},
	{.cfg_addr = 0x1c, .cfg_wdata = 0x160e},
	{.cfg_addr = 0x1d, .cfg_wdata = 0x160e},
	{.cfg_addr = 0x1e, .cfg_wdata = 0x0037},
	{.cfg_addr = 0x1f, .cfg_wdata = 0x0037},
	{.cfg_addr = 0x25, .cfg_wdata = 0x0730},
	{.cfg_addr = 0x26, .cfg_wdata = 0x0730},
};

static void ufs_pll_rateid_init(struct tegra_padctl_uphy *uphy)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_pll_rateid_init_data); i++) {
		reg = CFG_ADDR(ufs_pll_rateid_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_pll_rateid_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_pll_writel(uphy, 1, reg, UPHY_PLL_CTL_4);
	}
}

static struct init_data ufs_lane_rateid_init_data[] = {
	{.cfg_addr = 0x01, .cfg_wdata = 0x0004},
	{.cfg_addr = 0x03, .cfg_wdata = 0x0012},
	{.cfg_addr = 0x04, .cfg_wdata = 0x0030},
	{.cfg_addr = 0x05, .cfg_wdata = 0x0031},
	{.cfg_addr = 0x06, .cfg_wdata = 0x0032},
	{.cfg_addr = 0x07, .cfg_wdata = 0x0800},
	{.cfg_addr = 0x08, .cfg_wdata = 0x0811},
	{.cfg_addr = 0x09, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x0a, .cfg_wdata = 0x0227},
	{.cfg_addr = 0x0b, .cfg_wdata = 0x0009},
	{.cfg_addr = 0x1e, .cfg_wdata = 0x0002},
	{.cfg_addr = 0x28, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x29, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x2a, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x2e, .cfg_wdata = 0x0070},
	{.cfg_addr = 0x2f, .cfg_wdata = 0x0070},
	{.cfg_addr = 0x30, .cfg_wdata = 0x0070},
	{.cfg_addr = 0x35, .cfg_wdata = 0x070f},
	{.cfg_addr = 0x36, .cfg_wdata = 0x070f},
	{.cfg_addr = 0x37, .cfg_wdata = 0x070f},
	{.cfg_addr = 0x38, .cfg_wdata = 0x0734},
	{.cfg_addr = 0x39, .cfg_wdata = 0x0052},
	{.cfg_addr = 0x3a, .cfg_wdata = 0x8000},
	{.cfg_addr = 0x3b, .cfg_wdata = 0x8000},
	{.cfg_addr = 0x3c, .cfg_wdata = 0x8000},
	{.cfg_addr = 0x3d, .cfg_wdata = 0x2213},
	{.cfg_addr = 0x3e, .cfg_wdata = 0x2213},
	{.cfg_addr = 0x3f, .cfg_wdata = 0x2213},
	{.cfg_addr = 0x49, .cfg_wdata = 0x0f37},
	{.cfg_addr = 0x4a, .cfg_wdata = 0x0f67},
	{.cfg_addr = 0x4b, .cfg_wdata = 0x0fc7},
	{.cfg_addr = 0x4f, .cfg_wdata = 0x0c33},
	{.cfg_addr = 0x50, .cfg_wdata = 0x1767},
	{.cfg_addr = 0x53, .cfg_wdata = 0x0c00},
	{.cfg_addr = 0x54, .cfg_wdata = 0x0c00},
	{.cfg_addr = 0x55, .cfg_wdata = 0x0c00},
	{.cfg_addr = 0x56, .cfg_wdata = 0x0c00},
	{.cfg_addr = 0x57, .cfg_wdata = 0x0c00},
	{.cfg_addr = 0x58, .cfg_wdata = 0x0c00},
	{.cfg_addr = 0x59, .cfg_wdata = 0x0733},
	{.cfg_addr = 0x5a, .cfg_wdata = 0x0703},
	{.cfg_addr = 0x5b, .cfg_wdata = 0x1c90},
	{.cfg_addr = 0x5c, .cfg_wdata = 0x00ac},
	{.cfg_addr = 0x5d, .cfg_wdata = 0xff00},
	{.cfg_addr = 0x5e, .cfg_wdata = 0x0813},
	{.cfg_addr = 0x5f, .cfg_wdata = 0xc004},
	{.cfg_addr = 0x60, .cfg_wdata = 0x4456},
	{.cfg_addr = 0x61, .cfg_wdata = 0xa869},
	{.cfg_addr = 0x62, .cfg_wdata = 0x0082},
	{.cfg_addr = 0x63, .cfg_wdata = 0x010c},
	{.cfg_addr = 0x64, .cfg_wdata = 0x003c},
	{.cfg_addr = 0x67, .cfg_wdata = 0x0534},
	{.cfg_addr = 0x68, .cfg_wdata = 0x0200},
	{.cfg_addr = 0x69, .cfg_wdata = 0x0534},
	{.cfg_addr = 0x6a, .cfg_wdata = 0x0200},
	{.cfg_addr = 0x6b, .cfg_wdata = 0x0534},
	{.cfg_addr = 0x96, .cfg_wdata = 0x0001},
};

static void ufs_lane_rateid_init(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_lane_rateid_init_data); i++) {
		reg = CFG_ADDR(ufs_lane_rateid_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_lane_rateid_init_data[i].cfg_wdata);
		reg |= CFG_WDS;
		reg |= CFG_RESET;
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}
}

static struct init_data pcie_lane_g1_g2_init_data[] = {
	{.cfg_addr = 0x97, .cfg_wdata = 0x0080},
};

static void pcie_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(pcie_lane_g1_g2_init_data); i++) {
		reg = CFG_ADDR(pcie_lane_g1_g2_init_data[i].cfg_addr);
		reg |= CFG_WDATA(pcie_lane_g1_g2_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}
}

static struct init_data usb3_lane_g1_init_data[] = {
	{.cfg_addr = 0x1,  .cfg_wdata = 0x0002},
	{.cfg_addr = 0x4,  .cfg_wdata = 0x0032},
	{.cfg_addr = 0x7,  .cfg_wdata = 0x0022},
	{.cfg_addr = 0x35, .cfg_wdata = 0x2587},
	{.cfg_addr = 0x49, .cfg_wdata = 0x0FC7},
	{.cfg_addr = 0x52, .cfg_wdata = 0x0001},
	{.cfg_addr = 0x53, .cfg_wdata = 0x3C0F},
	{.cfg_addr = 0x56, .cfg_wdata = 0xC00F},
	{.cfg_addr = 0x5D, .cfg_wdata = 0xFF07},
	{.cfg_addr = 0x5E, .cfg_wdata = 0x141A},
};

static void usb3_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(usb3_lane_g1_init_data); i++) {
		reg = CFG_ADDR(usb3_lane_g1_init_data[i].cfg_addr);
		reg |= CFG_WDATA(usb3_lane_g1_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static struct init_data sata_lane_g1_g2_init_data[] = {
	{.cfg_addr = 0x1,  .cfg_wdata = 0x0003},
	{.cfg_addr = 0x3,  .cfg_wdata = 0x0010},
	{.cfg_addr = 0x4,  .cfg_wdata = 0x0030},
	{.cfg_addr = 0x5,  .cfg_wdata = 0x0031},
	{.cfg_addr = 0x7,  .cfg_wdata = 0x0800},
	{.cfg_addr = 0x8,  .cfg_wdata = 0x0811},
	{.cfg_addr = 0x28, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x29, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x2E, .cfg_wdata = 0x0050},
	{.cfg_addr = 0x2F, .cfg_wdata = 0x0050},
	{.cfg_addr = 0x49, .cfg_wdata = 0x0F37},
	{.cfg_addr = 0x4A, .cfg_wdata = 0x0F67},
};

static void sata_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(sata_lane_g1_g2_init_data); i++) {
		reg = CFG_ADDR(sata_lane_g1_g2_init_data[i].cfg_addr);
		reg |= CFG_WDATA(sata_lane_g1_g2_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static struct init_data ufs_lane_g1_g2_g3_init_data[] = {
	{.cfg_addr = 0x0,  .cfg_wdata = 0x0004},
	{.cfg_addr = 0x3,  .cfg_wdata = 0x0012},
	{.cfg_addr = 0x4,  .cfg_wdata = 0x0030},
	{.cfg_addr = 0x5,  .cfg_wdata = 0x0031},
	{.cfg_addr = 0x6,  .cfg_wdata = 0x0032},
	{.cfg_addr = 0x7,  .cfg_wdata = 0x0800},
	{.cfg_addr = 0x8,  .cfg_wdata = 0x0811},
	{.cfg_addr = 0xA,  .cfg_wdata = 0x0227},
	{.cfg_addr = 0xB,  .cfg_wdata = 0x0009},
	{.cfg_addr = 0x1E, .cfg_wdata = 0x0002},
	{.cfg_addr = 0x28, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x29, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x2A, .cfg_wdata = 0x0022},
	{.cfg_addr = 0x30, .cfg_wdata = 0x0070},
	{.cfg_addr = 0x37, .cfg_wdata = 0x070F},
	{.cfg_addr = 0x3c, .cfg_wdata = 0x8000},
	{.cfg_addr = 0x49, .cfg_wdata = 0x0F37},
	{.cfg_addr = 0x4A, .cfg_wdata = 0x0F67},
	{.cfg_addr = 0x4B, .cfg_wdata = 0x0FC7},
	{.cfg_addr = 0x55, .cfg_wdata = 0x0C00},
	{.cfg_addr = 0x58, .cfg_wdata = 0xC000},
	{.cfg_addr = 0x96, .cfg_wdata = 0x0001},
};

static void ufs_lane_defaults(struct tegra_padctl_uphy *uphy, int lane)
{
	u32 reg;
	int i;

	for (i = 0; i < ARRAY_SIZE(ufs_lane_g1_g2_g3_init_data); i++) {
		reg = CFG_ADDR(ufs_lane_g1_g2_g3_init_data[i].cfg_addr);
		reg |= CFG_WDATA(ufs_lane_g1_g2_g3_init_data[i].cfg_wdata);
		reg |= CFG_RESET;
		reg |= CFG_WDS;
		uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);
	}

}

static int tegra186_padctl_uphy_regulators_init(struct tegra_padctl_uphy *uphy)
{
	struct device *dev = uphy->dev;
	size_t size;
	int err;
	int i;

	size = uphy->soc->num_supplies * sizeof(struct regulator_bulk_data);
	uphy->supplies = devm_kzalloc(dev, size, GFP_ATOMIC);
	if (!uphy->supplies) {
		dev_err(dev, "failed to alloc memory for regulators\n");
		return -ENOMEM;
	}

	for (i = 0; i < uphy->soc->num_supplies; i++)
		uphy->supplies[i].supply = uphy->soc->supply_names[i];

	err = devm_regulator_bulk_get(dev, uphy->soc->num_supplies,
					uphy->supplies);
	if (err) {
		dev_err(dev, "failed to request regulators %d\n", err);
		return err;
	}

	return 0;
}

/* caller must hold uphy->lock */
static void uphy_pll_sw_overrides(struct tegra_padctl_uphy *uphy, int pll,
				enum tegra186_function func, bool set)
{
	struct device *dev = uphy->dev;
	u32 reg;

	dev_dbg(dev, "%s PLL%d overrides\n", set ? "set" : "clear", pll);

	if (set) {
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
		reg |= PWR_OVRD;
		if (pll == 1 && func == TEGRA186_FUNC_MPHY)
			reg |= RATE_ID_OVRD;
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		reg |= (CAL_OVRD | RCAL_OVRD);
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	} else {
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
		reg &= ~PWR_OVRD;
		if (pll == 1 && func == TEGRA186_FUNC_MPHY)
			reg &= ~RATE_ID_OVRD;
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		reg &= ~(CAL_OVRD | RCAL_OVRD);
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);
	}
}

#define uphy_pll_clear_sw_overrides(u, p, f)		\
	uphy_pll_sw_overrides(u, p, f, false)

#define uphy_pll_set_sw_overrides(u, p, f)		\
	uphy_pll_sw_overrides(u, p, f, true)

/* caller must hold uphy->lock */
static int uphy_pll_source_clk_state_check(struct tegra_padctl_uphy *uphy,
					   int pll)
{
	struct device *dev = uphy->dev;

	if ((uphy->pll_mgmt_state[pll] < PLL_POWER_DOWN) ||
		(uphy->pll_mgmt_state[pll] > PLL_POWER_UP_SW_CTL)) {
		dev_err(dev, "invalid PLL%d MGMT state %d\n", pll,
			uphy->pll_mgmt_state[pll]);
		return -EINVAL;
	}
	dev_dbg(dev, "PLL%d MGMT state %s\n", pll,
		source_pll_states[uphy->pll_mgmt_state[pll]]);

	if ((uphy->plle_state < PLL_POWER_DOWN) ||
		(uphy->plle_state > PLL_POWER_UP_HW_SEQ)) {
		dev_err(dev, "invalid PLLE state %d\n", uphy->plle_state);
		return -EINVAL;
	}
	dev_dbg(dev, "PLLE state %s\n", source_pll_states[uphy->plle_state]);


	if ((uphy->pllrefe_state < PLL_POWER_DOWN) ||
		(uphy->pllrefe_state > PLL_POWER_UP_SW_CTL)) {
		dev_err(dev, "invalid PLLREFE state %d\n", uphy->pllrefe_state);
		return -EINVAL;
	}
	dev_dbg(dev, "PLLREFE state %s\n",
		source_pll_states[uphy->pllrefe_state]);

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_source_clk_enable(struct tegra_padctl_uphy *uphy, int pll,
				      enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc = 0;

	rc = uphy_pll_source_clk_state_check(uphy, pll);
	if (rc)
		return rc;

	/* power up PLL management clock if it has not been enabled */
	if (uphy->pll_mgmt_state[pll] == PLL_POWER_DOWN) {
		dev_dbg(dev, "enable PLL%d mgmt clock\n", pll);
		rc = clk_prepare_enable(uphy->uphy_pll_mgmt[pll]);
		if (rc) {
			dev_err(dev, "failed to enable PLL%d MGMT clock %d\n",
				pll, rc);
			return rc;
		}
		uphy->pll_mgmt_state[pll] = PLL_POWER_UP_SW_CTL;
	}

	if ((pll == 0) ||
		((pll == 1) && func == TEGRA186_FUNC_SATA)) {
		/* power up PLLE if it has not been enabled */
		if (uphy->plle_state == PLL_POWER_DOWN) {
			dev_dbg(dev, "enable PLLE\n");
			rc = clk_prepare_enable(uphy->plle);
			if (rc) {
				dev_err(dev, "failed to enable PLLE clock %d\n",
					rc);
				return rc;
			}
			uphy->plle_state = PLL_POWER_UP_SW_CTL;
		}

		if ((pll == 1) && (func == TEGRA186_FUNC_SATA)) {
			dev_dbg(dev, "enable PLLE passthrough\n");
			rc = clk_prepare_enable(uphy->pllrefe_pex);
			if (rc) {
				dev_err(dev, "failed to enable pllrefe_pex %d\n",
					rc);
				return rc;
			}

			rc = clk_set_parent(uphy->pllrefe_pex,
					    uphy->plle_passthrough);
			if (rc) {
				dev_err(dev, "failed to set plle_passthrough as parent %d\n",
					rc);
				return rc;
			}
		}
		return 0; /* in this case, needs plle only */
	}

#if 0	/* by default, MPHY use XTAL */
	/* for pll1, power up PLLREFE if it has not been enabled */
	if (uphy->pllrefe_state == PLL_POWER_DOWN) {
		dev_dbg(dev, "enable PLLREFE\n");
		rc = clk_prepare_enable(uphy->pllrefe);
		if (rc) {
			dev_err(dev, "failed to enable PLLREFE clock %d\n", rc);
			return rc;
		}
		uphy->pllrefe_state = PLL_POWER_UP_SW_CTL;
	}
#endif
	return rc;
}

/* caller must hold uphy->lock */
static int uphy_pll_source_clk_disable(struct tegra_padctl_uphy *uphy, int pll,
				       enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int i;
	int rc = 0;

	rc = uphy_pll_source_clk_state_check(uphy, pll);
	if (rc)
		return rc;

	/* power down PLL management */
	if (uphy->pll_mgmt_state[pll] == PLL_POWER_UP_SW_CTL) {
		dev_dbg(dev, "disable PLL%d mgmt clock\n", pll);
		clk_disable_unprepare(uphy->uphy_pll_mgmt[pll]);
		uphy->pll_mgmt_state[pll] = PLL_POWER_DOWN;
	}

	if ((pll == 1) && (func == TEGRA186_FUNC_SATA)) {
		dev_dbg(dev, "disable PLLE passthrough\n");
		clk_disable_unprepare(uphy->pllrefe_pex);
	}

	for (i = 0; i < T186_UPHY_PLLS; i++) {
		if (uphy->pll_mgmt_state[pll] != PLL_POWER_DOWN)
			return 0;
	}

	/* both UPHY PLLs are powered down, power down PLLE */
	if (uphy->plle_state == PLL_POWER_UP_SW_CTL) {
		dev_dbg(dev, "disable PLLE\n");
		clk_disable_unprepare(uphy->plle);
		uphy->plle_state = PLL_POWER_DOWN;
	}

	/* for pll1, power down PLLREFE */
	if (uphy->pllrefe_state == PLL_POWER_UP_SW_CTL) {
		dev_dbg(dev, "disable PLLREFE\n");
		clk_disable_unprepare(uphy->pllrefe);
		uphy->pllrefe_state = PLL_POWER_DOWN;
	}

	return rc;
}

/* caller must hold uphy->lock */
static int uphy_pll_resistor_calibration(struct tegra_padctl_uphy *uphy,
					 int pll)
{
	struct device *dev = uphy->dev;
	u32 reg;

	dev_dbg(dev, "PLL%d resistor calibration\n", pll);

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
		dev_err(dev, "PLL%d start resistor calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* stop resistor calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~RCAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	usleep_range(5, 10);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	if (reg & RCAL_DONE) {
		dev_err(dev, "PLL%d stop resistor calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~RCAL_CLK_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_init_partial(struct tegra_padctl_uphy *uphy, int pll,
				 enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc = 0;
	u32 reg;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	if ((uphy->uphy_pll_state[pll] < UPHY_PLL_POWER_DOWN) ||
		(uphy->uphy_pll_state[pll] > UPHY_PLL_POWER_UP_HW_SEQ)) {
		dev_err(dev, "invalid PLL%d state %d\n", pll,
					uphy->uphy_pll_state[pll]);
		return -EINVAL;
	}

	dev_dbg(dev, "PLL%d state %s\n", pll,
			uphy_pll_states[uphy->uphy_pll_state[pll]]);

	if (uphy->uphy_pll_state[pll] >= UPHY_PLL_POWER_UP_PARTIAL)
		return 0; /* already done */

	dev_dbg(dev, "PLL%d partital init by function %d\n", pll, func);

	uphy_pll_set_sw_overrides(uphy, pll, func);

	/* power up PLL */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_IDDQ;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_SLEEP(~0);
	reg |= PLL_SLEEP(2);
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	ndelay(100);

	rc = uphy_pll_resistor_calibration(uphy, pll);
	if (rc)
		return rc;

	uphy->uphy_pll_state[pll] = UPHY_PLL_POWER_UP_PARTIAL;

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_hw_sequencer_enable(struct tegra_padctl_uphy *uphy, int pll,
					enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc;

	dev_dbg(dev, "enable PLL%d HW power sequencer by function %d\n",
						pll, func);
	rc = clk_prepare_enable(uphy->uphy_pll_pwrseq[pll]);
	if (rc) {
		dev_err(dev, "failed to enable PLL%d Power sequencer %d\n",
			pll, rc);
		return rc;
	}
	uphy->uphy_pll_state[pll] = UPHY_PLL_POWER_UP_HW_SEQ;

	/* remove SW overrides to allow HW sequencer to run */
	uphy_pll_clear_sw_overrides(uphy, pll, func);

	if ((uphy->uphy_pll_state[0] == UPHY_PLL_POWER_UP_HW_SEQ) &&
		(uphy->uphy_pll_state[1] == UPHY_PLL_POWER_UP_HW_SEQ)) {
		dev_dbg(dev, "enable PLLE Power sequencer\n");
		rc = clk_prepare_enable(uphy->plle_pwrseq);
		if (rc) {
			dev_err(dev, "failed to enable PLLE Power sequencer %d\n",
				rc);
			return rc;
		}
		uphy->plle_state = PLL_POWER_UP_HW_SEQ;
	}
	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_hw_sequencer_disable(struct tegra_padctl_uphy *uphy, int pll
					, enum tegra186_function func)
{
	struct device *dev = uphy->dev;

	if (uphy->plle_state == PLL_POWER_UP_HW_SEQ) {
		dev_dbg(dev, "disable PLLE hardware power sequencer\n");
		clk_disable_unprepare(uphy->plle_pwrseq);
		uphy->plle_state = PLL_POWER_UP_SW_CTL;
	}

	/* TODO check PLL physical state */

	uphy_pll_set_sw_overrides(uphy, pll, func);

	/* get back to software control */
	dev_dbg(dev, "disable PLL%d hardware power sequencer\n", pll);
	clk_disable_unprepare(uphy->uphy_pll_pwrseq[pll]);
	uphy->uphy_pll_state[pll] = UPHY_PLL_POWER_UP_FULL;

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_power_down(struct tegra_padctl_uphy *uphy, int pll
					, enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	u32 reg;

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_ENABLE;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	usleep_range(20, 25);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	if (reg & LOCKDET_STATUS) {
		dev_err(dev, "disable PLL%d timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* enter sleep state = 3 */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_SLEEP(~0);
	reg |= PLL_SLEEP(3);
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	/* apply IDDQ */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg |= PLL_IDDQ;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);
	uphy->uphy_pll_state[pll] = UPHY_PLL_POWER_DOWN;

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_calibration(struct tegra_padctl_uphy *uphy, int pll)
{
	struct device *dev = uphy->dev;
	u32 reg;
	int i;

	dev_dbg(dev, "PLL%d calibration\n", pll);

	/* perform PLL calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg |= CAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	for (i = 0; i < 50; i++) {
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		if (reg & CAL_DONE)
			break;
		usleep_range(10, 15);
	}
	if (!(reg & CAL_DONE)) {
		dev_err(dev, "start PLL%d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	/* stop PLL calibration */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
	reg &= ~CAL_EN;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_2);

	for (i = 0; i < 50; i++) {
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_2);
		if (!(reg & CAL_DONE))
			break;
		usleep_range(10, 15);
	}
	if (reg & CAL_DONE) {
		dev_err(dev, "stop PLL%d calibration timeout\n", pll);
		return -ETIMEDOUT;
	}

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_init_full(struct tegra_padctl_uphy *uphy, int pll,
			      enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc = 0;
	u32 reg;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	if ((uphy->uphy_pll_state[pll] < UPHY_PLL_POWER_DOWN) ||
		(uphy->uphy_pll_state[pll] > UPHY_PLL_POWER_UP_HW_SEQ)) {
		dev_err(dev, "invalid PLL%d state %d\n", pll,
					uphy->uphy_pll_state[pll]);
		return -EINVAL;
	}

	dev_dbg(dev, "PLL%d state %s\n", pll,
			uphy_pll_states[uphy->uphy_pll_state[pll]]);

	if (uphy->uphy_pll_state[pll] >= UPHY_PLL_POWER_UP_FULL)
		return 0; /* already done */

	dev_dbg(dev, "PLL%d full init by function %d\n", pll, func);

	if (uphy->uphy_pll_state[pll] == UPHY_PLL_POWER_DOWN)
		uphy_pll_set_sw_overrides(uphy, pll, func);

	/* power up PLL */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_IDDQ;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg &= ~PLL_SLEEP(~0);
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	ndelay(100);

	rc = uphy_pll_calibration(uphy, pll);
	if (rc)
		return rc;

	if (pll == 1 && func == TEGRA186_FUNC_MPHY) {
		/* perform PLL rate change for PLL_1, used by HighSpeed UFS */
		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
		reg &= ~RATE_ID(~0);
		reg |= (RATE_ID(1) | RATE_ID_OVRD);
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

		ndelay(100);

		rc = uphy_pll_calibration(uphy, pll);
		if (rc)
			return rc;

		reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
		reg &= ~RATE_ID(~0);
		reg |= (RATE_ID(0) | RATE_ID_OVRD);
		uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);
	}

	/* enable PLL and wait for lock */
	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	reg |= PLL_ENABLE;
	uphy_pll_writel(uphy, pll, reg, UPHY_PLL_CTL_1);

	usleep_range(65, 70);

	reg = uphy_pll_readl(uphy, pll, UPHY_PLL_CTL_1);
	if (!(reg & LOCKDET_STATUS)) {
		dev_err(dev, "enable PLL%d timeout\n", pll);
		return -ETIMEDOUT;
	}

	if (uphy->uphy_pll_state[pll] == UPHY_PLL_POWER_DOWN) {
		rc = uphy_pll_resistor_calibration(uphy, pll);
		if (rc)
			return rc;
	}

	uphy->uphy_pll_state[pll] = UPHY_PLL_POWER_UP_FULL;

	return uphy_pll_hw_sequencer_enable(uphy, pll, func);
}

/* caller must hold uphy->lock */
static int uphy_pll_init(struct tegra_padctl_uphy *uphy,
			 enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc = 0;
	int i;

	dev_dbg(dev, "PLL init by function %d\n", func);

	if (func == TEGRA186_FUNC_USB3 || func == TEGRA186_FUNC_PCIE) {
		rc = uphy_pll_source_clk_enable(uphy, 0, func);
	} else if (func == TEGRA186_FUNC_SATA || func == TEGRA186_FUNC_MPHY) {
		rc = uphy_pll_source_clk_enable(uphy, 0, func);
		if (rc)
			return rc;
		rc = uphy_pll_source_clk_enable(uphy, 1, func);
	}
	if (rc)
		return rc;

	switch (func) {
	case TEGRA186_FUNC_PCIE:
	case TEGRA186_FUNC_USB3:
		rc = uphy_pll_init_full(uphy, 0, func);
		break;
	case TEGRA186_FUNC_SATA:
	case TEGRA186_FUNC_MPHY:
		rc = uphy_pll_init_partial(uphy, 0, func);
		if (rc)
			return rc;
		rc = uphy_pll_init_full(uphy, 1, func);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	if (rc == 0) {
		if (func == TEGRA186_FUNC_PCIE || func == TEGRA186_FUNC_USB3) {
			uphy->uphy_pll_users[0] |= BIT(func);
		} else {
			/* SATA/UFS */
			uphy->uphy_pll_users[0] |= BIT(func);
			uphy->uphy_pll_users[1] |= BIT(func);
		}
		for (i = 0; i < T186_UPHY_PLLS; i++) {
			dev_dbg(dev, "PLL%d users 0x%lx\n", i,
				uphy->uphy_pll_users[i]);
		}
	}

	return rc;
}

/* caller must hold uphy->lock */
int uphy_pll_deinit(struct tegra_padctl_uphy *uphy,
			   enum tegra186_function func)
{
	struct device *dev = uphy->dev;
	int rc;
	int i;

	if (func == TEGRA186_FUNC_PCIE || func == TEGRA186_FUNC_USB3) {
		uphy->uphy_pll_users[0] &= ~BIT(func);
	} else if (func == TEGRA186_FUNC_SATA || func == TEGRA186_FUNC_MPHY) {
		/* SATA/UFS */
		uphy->uphy_pll_users[0] &= ~BIT(func);
		uphy->uphy_pll_users[1] &= ~BIT(func);
	} else
		return -EINVAL;

	for (i = T186_UPHY_PLLS - 1; i >= 0; i--) {
		dev_dbg(dev, "PLL%d users 0x%lx\n", i, uphy->uphy_pll_users[i]);
		if (uphy->uphy_pll_users[i] == 0) {
			/* TODO error handling */
			rc = uphy_pll_hw_sequencer_disable(uphy, i, func);
			if (rc)
				return rc;
			rc = uphy_pll_power_down(uphy, i, func);
			if (rc)
				return rc;
			rc = uphy_pll_source_clk_disable(uphy, i, func);
			if (rc)
				return rc;
		}
	}

	return 0;
}

/* caller must hold uphy->lock */
static int uphy_pll_reset_deassert(struct tegra_padctl_uphy *uphy, int pll)
{
	struct device *dev = uphy->dev;
	int rc = 0;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	dev_dbg(dev, "PLL%d clients %d\n", pll, uphy->uphy_pll_clients[pll]);

	if (WARN_ON(uphy->uphy_pll_clients[pll] < 0))
		return -EINVAL;

	if (uphy->uphy_pll_clients[pll]++ > 0)
		goto out;

	dev_dbg(dev, "deassert reset to PLL%d\n", pll);
	rc = reset_control_deassert(uphy->uphy_pll_rst[pll]);
	if (rc) {
		uphy->uphy_pll_clients[pll]--;
		dev_err(dev, "failed to deassert reset PLL%d %d\n", pll, rc);
	}

out:
	return rc;
}

/* caller must hold uphy->lock */
static int uphy_pll_reset_assert(struct tegra_padctl_uphy *uphy, int pll)
{
	struct device *dev = uphy->dev;
	int rc = 0;

	if (pll < 0 || pll >= T186_UPHY_PLLS)
		return -EINVAL;

	dev_dbg(dev, "PLL%d clients %d\n", pll, uphy->uphy_pll_clients[pll]);

	if (WARN_ON(uphy->uphy_pll_clients[pll] == 0))
		return -EINVAL;

	if (--uphy->uphy_pll_clients[pll] > 0)
		goto out;

	dev_dbg(dev, "assert reset to PLL%d\n", pll);
	rc = reset_control_assert(uphy->uphy_pll_rst[pll]);
	if (rc) {
		uphy->uphy_pll_clients[pll]++;
		dev_err(dev, "failed to assert reset for PLL%d %d\n", pll, rc);
	}
out:
	return rc;
}

/* caller must hold uphy->lock */
static int uphy_lanes_reset_deassert(struct tegra_padctl_uphy *uphy,
				     unsigned long uphy_lane_bitmap)
{
	struct device *dev = uphy->dev;
	int last_done_lane = 0;
	int lane;
	int rc;

	for_each_set_bit(lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		dev_dbg(dev, "deassert reset to Lane %d\n", lane);
		rc = reset_control_deassert(uphy->uphy_lane_rst[lane]);
		if (rc) {
			dev_err(dev, "failed to deassert reset Lane %d %d\n",
				lane, rc);
			goto assert;
		}
		last_done_lane = lane;
	}

	return 0;

assert:
	for_each_set_bit(lane, &uphy_lane_bitmap, last_done_lane)
		reset_control_assert(uphy->uphy_lane_rst[lane]);

	return rc;
}

/* caller must hold uphy->lock */
static int uphy_lanes_reset_assert(struct tegra_padctl_uphy *uphy,
				   unsigned long uphy_lane_bitmap)
{
	struct device *dev = uphy->dev;
	int last_done_lane = 0;
	int lane;
	int rc;

	for_each_set_bit(lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		dev_dbg(dev, "assert reset to Lane %d\n", lane);
		rc = reset_control_assert(uphy->uphy_lane_rst[lane]);
		if (rc) {
			dev_err(dev, "failed to assert Lane %d %d\n", lane, rc);
			goto deassert;
		}
		last_done_lane = lane;
	}

	return 0;

deassert:
	for_each_set_bit(lane, &uphy_lane_bitmap, last_done_lane)
		reset_control_deassert(uphy->uphy_lane_rst[lane]);

	return rc;
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
	TEGRA_PADCTL_UPHY_HSIC_PRETEND_CONNECTED,
};

static const struct tegra_padctl_uphy_property {
	const char *name;
	enum tegra_xusb_padctl_param param;
} properties[] = {
	{"nvidia,usb3-port", TEGRA_PADCTL_UPHY_USB3_PORT},
	{"nvidia,port-cap", TEGRA_PADCTL_UPHY_PORT_CAP},
	{"nvidia,pcie-controller", TEGRA_PADCTL_UPHY_PCIE_CONTROLLER_NUM},
	{"nvidia,pretend-connected", TEGRA_PADCTL_UPHY_HSIC_PRETEND_CONNECTED},
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
			goto out;

		function = NULL;
	}

	for (i = 0; i < ARRAY_SIZE(properties); i++) {
		err = of_property_read_u32(np, properties[i].name, &value);
		if (err < 0) {
			if (err == -EINVAL)
				continue;

			goto out;
		}

		config = TEGRA_XUSB_PADCTL_PACK(properties[i].param, value);

		err = pinctrl_utils_add_config(uphy->pinctrl, &configs,
					       &num_configs, config);
		if (err < 0)
			goto out;
	}

	if (function)
		reserve++;

	if (num_configs)
		reserve++;

	err = of_property_count_strings(np, "nvidia,lanes");
	if (err < 0)
		goto out;

	reserve *= err;

	err = pinctrl_utils_reserve_map(uphy->pinctrl, maps, reserved_maps,
					num_maps, reserve);
	if (err < 0)
		goto out;

	of_property_for_each_string(np, "nvidia,lanes", prop, group) {
		if (function) {
			err = pinctrl_utils_add_map_mux(uphy->pinctrl, maps,
					reserved_maps, num_maps, group,
					function);
			if (err < 0)
				goto out;
		}

		if (num_configs) {
			err = pinctrl_utils_add_map_configs(uphy->pinctrl,
					maps, reserved_maps, num_maps, group,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_GROUP);
			if (err < 0)
				goto out;
		}
	}

	err = 0;

out:
	kfree(configs);
	return err;
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
		/* If node status is disabled then ignore the node */
		if (!of_device_is_available(np))
			continue;

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
				if (value == OTG) {
					if (uphy->usb3_otg_port_base_1)
						dev_warn(dev, "enabling OTG on multiple USB3 ports\n");


					dev_info(dev, "using USB3 port %d for otg\n",
						 port);
					uphy->usb3_otg_port_base_1 =
								port + 1;
				}
			} else if (lane_is_otg(group)) {
				int port = group - PIN_OTG_0;

				uphy->utmi_ports[port].port_cap = value;
				TRACE(dev, "UTMI port %d cap %lu",
				      port, value);
				if (value == OTG) {
					if (uphy->utmi_otg_port_base_1)
						dev_warn(dev, "enabling OTG on multiple UTMI ports\n");

					dev_info(dev, "using UTMI port %d for otg\n",
						 port);

					uphy->utmi_otg_port_base_1 =
								port + 1;
				}
			} else {
				dev_err(dev, "port-cap not applicable for pin %d\n",
					group);
				return -EINVAL;
			}

			break;

		case TEGRA_PADCTL_UPHY_HSIC_PRETEND_CONNECTED:
			if (lane_is_hsic(group)) {
				int port = group - PIN_HSIC_0;

				uphy->hsic_ports[port].pretend_connected =
									  value;
				TRACE(dev, "HSIC port %d pretend-connected %ld",
				      port, value);
			} else {
				dev_err(dev, "pretend-connected is not applicable for pin %d\n",
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

/* caller must hold uphy->lock */
static int tegra_padctl_uphy_enable(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	TRACE(uphy->dev, "enable_counts %d", uphy->enable_counts);

	if (uphy->enable_counts++ > 0)
		goto out;

	/* TODO: anything we need to initialize */
out:
	return 0;
}

/* caller must hold uphy->lock */
static int tegra_padctl_uphy_disable(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	TRACE(uphy->dev, "enable_counts %d", uphy->enable_counts);

	if (WARN_ON(uphy->enable_counts == 0))
		goto out;

	if (--uphy->enable_counts > 0)
		goto out;

	/* TODO: anything we need to de-initialize */
out:
	return 0;
}

/* caller must hold uphy->lock */
static inline void uphy_lanes_clamp(struct tegra_padctl_uphy *uphy,
				    unsigned long uphy_lane_bitmap,
				    bool enable)
{
	unsigned int lane;
	u32 reg;

	if (enable) {
		/* Enable clamp */
		for_each_set_bit(lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
			reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MUX);
			reg |= CLAMP_EN_EARLY;
			uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MUX);
		}

		/* Enable force IDDQ on lanes */
		for_each_set_bit(lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
			reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MUX);
			reg &= ~FORCE_IDDQ_DISABLE;
			uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MUX);
		}
	} else {
		/* Remove IDDQ and clamp */
		for_each_set_bit(lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
			reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MUX);
			reg |= FORCE_IDDQ_DISABLE;
			uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MUX);
		}

		usleep_range(100, 200);

		for_each_set_bit(lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
			reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MUX);
			reg &= ~CLAMP_EN_EARLY;
			uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MUX);
		}
	}
}

#define uphy_lanes_clamp_enable(u, m) uphy_lanes_clamp(u, m, true)
#define uphy_lanes_clamp_disable(u, m) uphy_lanes_clamp(u, m, false)

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

	if (controller < 0)
		return controller;

	mutex_lock(&uphy->lock);

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "power on PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	uphy_lanes_clamp_disable(uphy, uphy_lane_bitmap);

	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_pcie_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;

	if (controller < 0)
		return controller;

	mutex_lock(&uphy->lock);

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(dev, "power off PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	uphy_lanes_clamp_enable(uphy, uphy_lane_bitmap);

	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_pcie_uphy_pll_init(struct tegra_padctl_uphy *uphy)
{
	unsigned long uphy_lane_bitmap;
	unsigned int uphy_lane;
	u32 reg;
	int rc;

	mutex_lock(&uphy->lock);

	uphy_lane_bitmap = uphy->pcie_lanes;
	dev_dbg(uphy->dev, "%s PCIE controller uphy-lanes 0x%lx\n",
		__func__, uphy_lane_bitmap);

	/* Program lane ownership by selecting mux to PCIE */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~SEL(~0);
		reg |= SEL_PCIE;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Reset release of lanes and PLL0 */
	rc = uphy_pll_reset_deassert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_lanes_reset_deassert(uphy, uphy_lane_bitmap);
	if (rc)
		goto assert_pll0_reset;

	/* Program pll defaults */
	pcie_usb3_pll_defaults(uphy);

	/* Program lane defaults */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES)
		pcie_lane_defaults(uphy, uphy_lane);

	rc = uphy_pll_init(uphy, TEGRA186_FUNC_PCIE);
	if (rc)
		goto assert_lanes_reset;

	mutex_unlock(&uphy->lock);
	return 0;

assert_lanes_reset:
	uphy_lanes_reset_assert(uphy, uphy_lane_bitmap);
assert_pll0_reset:
	uphy_pll_reset_assert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);
	return rc;
}

static int tegra186_pcie_uphy_pll_deinit(struct tegra_padctl_uphy *uphy)
{
	int rc;

	dev_dbg(uphy->dev, "%s PCIE controller uphy-lanes 0x%lx\n",
		__func__, uphy->pcie_lanes);

	mutex_lock(&uphy->lock);

	rc = uphy_pll_reset_assert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_lanes_reset_assert(uphy, uphy->pcie_lanes);
	if (rc)
		goto deassert_pll0_reset;

	rc = uphy_pll_deinit(uphy, TEGRA186_FUNC_PCIE);
	if (rc)
		goto deassert_lanes_reset;

	mutex_unlock(&uphy->lock);

	return 0;

deassert_lanes_reset:
	uphy_lanes_reset_deassert(uphy, uphy->pcie_lanes);
deassert_pll0_reset:
	uphy_pll_reset_deassert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);

	return rc;
}

static int tegra186_pcie_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;

	if (controller < 0)
		return controller;

	mutex_lock(&uphy->lock);

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "phy init PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	tegra_padctl_uphy_enable(phy);

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_pcie_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int controller = pcie_phy_to_controller(phy);
	unsigned long uphy_lane_bitmap;

	if (controller < 0)
		return controller;

	mutex_lock(&uphy->lock);

	uphy_lane_bitmap = uphy->pcie_controllers[controller].uphy_lane_bitmap;
	dev_dbg(uphy->dev, "phy exit PCIE controller %d uphy-lanes 0x%lx\n",
		controller, uphy_lane_bitmap);

	tegra_padctl_uphy_disable(phy);
	mutex_unlock(&uphy->lock);

	return 0;
}

static const struct phy_ops pcie_phy_ops = {
	.init = tegra186_pcie_phy_init,
	.exit = tegra186_pcie_phy_exit,
	.power_on = tegra186_pcie_phy_power_on,
	.power_off = tegra186_pcie_phy_power_off,
	.owner = THIS_MODULE,
};

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

	return 0;
}

static int tegra186_sata_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;

	mutex_lock(&uphy->lock);

	dev_dbg(dev, "power on SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	uphy_lanes_clamp_disable(uphy, uphy->sata_lanes);

	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_sata_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;

	mutex_lock(&uphy->lock);

	dev_dbg(dev, "power off SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	uphy_lanes_clamp_enable(uphy, uphy->sata_lanes);

	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_sata_uphy_pll_init(struct tegra_padctl_uphy *uphy)
{
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;
	u32 reg;
	int rc;

	dev_dbg(dev, "%s SATA uphy-lanes 0x%lx\n", __func__, uphy->sata_lanes);

	mutex_lock(&uphy->lock);

	/* Program lane ownership by selecting mux to SATA */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~SEL(~0);
		reg |= SEL_SATA;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Reset release of lanes and PLL0/PLL1 */
	rc = uphy_pll_reset_deassert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_pll_reset_deassert(uphy, 1);
	if (rc)
		goto assert_pll0_reset;

	rc = uphy_lanes_reset_deassert(uphy, uphy->sata_lanes);
	if (rc)
		goto assert_pll1_reset;

	/* Program pll defaults */
	sata_pll_defaults(uphy);

	/* Program lane defaults */
	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES)
		sata_lane_defaults(uphy, uphy_lane);

	for_each_set_bit(uphy_lane, &uphy->sata_lanes, T186_UPHY_LANES)
		tegra186_sata_fuse_calibration(uphy, uphy_lane);

	rc = uphy_pll_init(uphy, TEGRA186_FUNC_SATA);
	if (rc)
		goto assert_lanes_reset;

	mutex_unlock(&uphy->lock);

	return 0;

assert_lanes_reset:
	uphy_lanes_reset_assert(uphy, uphy->sata_lanes);
assert_pll1_reset:
	uphy_pll_reset_assert(uphy, 1);
assert_pll0_reset:
	uphy_pll_reset_assert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);

	return rc;
}

static int tegra186_sata_uphy_pll_deinit(struct tegra_padctl_uphy *uphy)
{
	struct device *dev = uphy->dev;
	int rc;

	dev_dbg(dev, "%s SATA uphy-lanes 0x%lx\n", __func__, uphy->sata_lanes);

	mutex_lock(&uphy->lock);

	rc = uphy_pll_reset_assert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_pll_reset_assert(uphy, 1);
	if (rc)
		goto deassert_pll0_reset;

	rc = uphy_lanes_reset_assert(uphy, uphy->sata_lanes);
	if (rc)
		goto deassert_pll1_reset;

	rc = uphy_pll_deinit(uphy, TEGRA186_FUNC_SATA);
	if (rc)
		goto deassert_lanes_reset;

	mutex_unlock(&uphy->lock);

	return 0;

deassert_lanes_reset:
	uphy_lanes_reset_deassert(uphy, uphy->sata_lanes);
deassert_pll1_reset:
	uphy_pll_reset_deassert(uphy, 1);
deassert_pll0_reset:
	uphy_pll_reset_deassert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);

	return rc;
}

static int tegra186_sata_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	int rc;

	dev_dbg(dev, "phy init SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	mutex_lock(&uphy->lock);

	rc = tegra_padctl_uphy_enable(phy);

	mutex_unlock(&uphy->lock);

	return rc;
}

static int tegra186_sata_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;

	mutex_lock(&uphy->lock);

	dev_dbg(dev, "phy exit SATA uphy-lanes 0x%lx\n", uphy->sata_lanes);

	tegra_padctl_uphy_disable(phy);

	mutex_unlock(&uphy->lock);

	return 0;
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

	/* Update based on fuse_mphy_nv_calib[4:2] value */
	reg = CFG_ADDR(0x35);
	reg |= CFG_WDATA_0(MPHY_NV_CALIB_4(uphy->fuse_calib.mphy_nv));
	reg |= CFG_WDATA_1_2(MPHY_NV_CALIB_2_3(uphy->fuse_calib.mphy_nv));
	reg |= CFG_WDATA_3_4(MPHY_NV_CALIB_2_3(uphy->fuse_calib.mphy_nv));
	reg |= CFG_RESET;
	reg |= CFG_WDS;
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DIRECT_CTL_2);

	/* Update based on fuse_sata_mphy_odm_calib[1:0] value */
	idx = SATA_MPHY_ODM_CALIB_0_1(uphy->fuse_calib.sata_mphy_odm);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_1);
	reg &= ~TX_DRV_AMP_SEL0(~0);
	reg &= ~TX_DRV_AMP_SEL1(~0);
	reg &= ~TX_DRV_AMP_SEL2(~0);
	reg &= ~TX_DRV_AMP_SEL3(~0);
	reg |= TX_DRV_AMP_SEL0(mphy_data[idx].tx_drv_amp_sel0);
	reg |= TX_DRV_AMP_SEL1(mphy_data[idx].tx_drv_amp_sel1);
	reg |= TX_DRV_AMP_SEL2(mphy_data[idx].tx_drv_amp_sel2);
	reg |= TX_DRV_AMP_SEL3(mphy_data[idx].tx_drv_amp_sel3);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_1);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_2);
	reg &= ~TX_DRV_AMP_SEL4(~0);
	reg &= ~TX_DRV_AMP_SEL5(~0);
	reg &= ~TX_DRV_AMP_SEL6(~0);
	reg &= ~TX_DRV_AMP_SEL7(~0);
	reg |= TX_DRV_AMP_SEL4(mphy_data[idx].tx_drv_amp_sel4);
	reg |= TX_DRV_AMP_SEL5(mphy_data[idx].tx_drv_amp_sel5);
	reg |= TX_DRV_AMP_SEL6(mphy_data[idx].tx_drv_amp_sel6);
	reg |= TX_DRV_AMP_SEL7(mphy_data[idx].tx_drv_amp_sel7);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_2);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_3);
	reg &= ~TX_DRV_AMP_SEL8(~0);
	reg &= ~TX_DRV_AMP_SEL9(~0);
	reg |= TX_DRV_AMP_SEL8(mphy_data[idx].tx_drv_amp_sel8);
	reg |= TX_DRV_AMP_SEL9(mphy_data[idx].tx_drv_amp_sel9);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_3);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_4);
	reg &= ~TX_DRV_POST_SEL0(~0);
	reg &= ~TX_DRV_POST_SEL1(~0);
	reg |= TX_DRV_POST_SEL0(mphy_data[idx].tx_drv_post_sel0);
	reg |= TX_DRV_POST_SEL1(mphy_data[idx].tx_drv_post_sel1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_4);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_DYN_CTL_5);
	reg &= ~TX_DRV_POST_SEL2(~0);
	reg &= ~TX_DRV_POST_SEL3(~0);
	reg &= ~TX_DRV_PRE_SEL3(~0);
	reg |= TX_DRV_POST_SEL2(mphy_data[idx].tx_drv_post_sel2);
	reg |= TX_DRV_POST_SEL3(mphy_data[idx].tx_drv_post_sel3);
	reg |= TX_DRV_PRE_SEL3(mphy_data[idx].tx_drv_pre_sel3);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_DYN_CTL_5);

	return 0;
}

static void ufs_lane_pad_macro_configuration(struct tegra_padctl_uphy *uphy,
						int lane)
{
	u32 reg;

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MPHY_CTL_2);
	reg &= ~RX_BYP_MODE(~0);
	reg |= RX_BYP_MODE(0x2);
	reg &= ~RX_RATE_PDIV(~0);
	reg |= RX_RATE_PDIV(0x1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MPHY_CTL_2);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MPHY_CTL_1);
	reg &= ~TX_RATE_PDIV(~0);
	reg |= TX_RATE_PDIV(0x1);
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MPHY_CTL_1);

	reg = uphy_lane_readl(uphy, lane, UPHY_LANE_MISC_CTL_2);
	reg |= RX_BYP_REFCLK_EN;
	uphy_lane_writel(uphy, lane, reg, UPHY_LANE_MISC_CTL_2);
}

static int tegra186_ufs_uphy_pll_init(struct tegra_padctl_uphy *uphy)
{
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;
	u32 reg;
	int rc;

	mutex_lock(&uphy->lock);

	dev_dbg(dev, "%s UFS uphy-lanes 0x%lx\n", __func__, uphy->ufs_lanes);

	/* step 2.2: Program lane ownership by selecting mux to MPHY */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~SEL(~0);
		reg |= SEL_MPHY;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* step 5.2: Bias PWM detector logic */
	rc = clk_prepare_enable(uphy->rx_byp_clk);
	if (rc) {
		dev_err(dev, "failed to enable rx_byp reference clock %d\n",
			rc);
	}

	/* step 5.3: Reset release of lanes and PLL1. */
	rc = uphy_pll_reset_deassert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_pll_reset_deassert(uphy, 1);
	if (rc)
		goto assert_pll0_reset;

	rc = uphy_lanes_reset_deassert(uphy, uphy->ufs_lanes);
	if (rc)
		goto assert_pll1_reset;

	/* step 6.1: Program pll defaults */
	ufs_pll_defaults(uphy);

	/* step 6.2: Program lane defaults */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES) {
		TRACE(dev, "uphy_lane %u", uphy_lane);
		ufs_lane_defaults(uphy, uphy_lane);
	}

	/* step 6.3: Electrical parameters programming based on fuses */
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES)
		tegra186_ufs_fuse_calibration(uphy, uphy_lane);

	/* step 7: Rate id programming */
	ufs_pll_rateid_init(uphy);
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES)
		ufs_lane_rateid_init(uphy, uphy_lane);
	for_each_set_bit(uphy_lane, &uphy->ufs_lanes, T186_UPHY_LANES)
		ufs_lane_pad_macro_configuration(uphy, uphy_lane);

	/* step 8: Uphy pll1 calibration */
	rc = uphy_pll_init(uphy, TEGRA186_FUNC_MPHY);
	if (rc)
		goto assert_lanes_reset;

	mutex_unlock(&uphy->lock);

	return 0;

assert_lanes_reset:
	uphy_lanes_reset_assert(uphy, uphy->ufs_lanes);
assert_pll1_reset:
	uphy_pll_reset_assert(uphy, 1);
assert_pll0_reset:
	uphy_pll_reset_assert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);
	return rc;
}

static int tegra186_ufs_uphy_pll_deinit(struct tegra_padctl_uphy *uphy)
{
	struct device *dev = uphy->dev;
	int rc;

	dev_dbg(dev, "%s UFS uphy-lanes 0x%lx\n", __func__, uphy->ufs_lanes);

	mutex_lock(&uphy->lock);

	rc = uphy_pll_reset_assert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_pll_reset_assert(uphy, 1);
	if (rc)
		goto deassert_pll0_reset;

	rc = uphy_lanes_reset_assert(uphy, uphy->ufs_lanes);
	if (rc)
		goto deassert_pll1_reset;

	rc = uphy_pll_deinit(uphy, TEGRA186_FUNC_MPHY);
	if (rc)
		goto deassert_lanes_reset;

	mutex_unlock(&uphy->lock);

	return 0;

deassert_lanes_reset:
	uphy_lanes_reset_deassert(uphy, uphy->ufs_lanes);
deassert_pll1_reset:
	uphy_pll_reset_deassert(uphy, 1);
deassert_pll0_reset:
	uphy_pll_reset_deassert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);

	return rc;
}

static int tegra186_ufs_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	mutex_lock(&uphy->lock);

	dev_dbg(uphy->dev, "power on UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);

	uphy_lanes_clamp_disable(uphy, uphy->ufs_lanes);

	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_ufs_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;

	dev_dbg(dev, "power off UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);

	mutex_lock(&uphy->lock);

	uphy_lanes_clamp_enable(uphy, uphy->ufs_lanes);

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_ufs_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	mutex_lock(&uphy->lock);

	dev_dbg(uphy->dev, "phy init UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);

	tegra_padctl_uphy_enable(phy);

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_ufs_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);

	mutex_lock(&uphy->lock);

	dev_dbg(uphy->dev, "phy exit UFS uphy-lanes 0x%lx\n", uphy->ufs_lanes);

	tegra_padctl_uphy_disable(phy);

	mutex_unlock(&uphy->lock);
	return 0;
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
	struct device *dev = uphy->dev;
	int port = usb3_phy_to_port(phy);
	char prod_name[] = "prod_c_ssX";
	unsigned int uphy_lane;
	u32 reg;
	int err;

	if (port < 0)
		return port;

	mutex_lock(&uphy->lock);

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(uphy->dev, "power on USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	sprintf(prod_name, "prod_c_ss%d", port);
	err = tegra_prod_set_by_name(uphy->uphy_lane_regs, prod_name,
				     uphy->prod_list);
	if (err) {
		dev_info(dev, "failed to apply prod for ss pad%d (%d)\n",
			port, err);
	}

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

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	uphy_lanes_clamp_disable(uphy, BIT(uphy_lane));

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	int port = usb3_phy_to_port(phy);
	unsigned int uphy_lane;
	u32 reg;

	if (port < 0)
		return port;

	mutex_lock(&uphy->lock);

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(dev, "power off USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	uphy_lanes_clamp_enable(uphy, BIT(uphy_lane));

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(250, 350);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_VCORE_DOWN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_usb3_phy_enable_wakelogic(struct tegra_padctl_uphy *uphy,
					      int port)
{
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;
	u32 reg;

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(dev, "enable wakelogic USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(250, 350);

	return 0;
}

static int tegra186_usb3_phy_disable_wakelogic(struct tegra_padctl_uphy *uphy,
					      int port)
{
	struct device *dev = uphy->dev;
	unsigned int uphy_lane;
	u32 reg;

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(dev, "disable wakelogic USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN(port);
	padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	return 0;
}

static int tegra186_usb3_uphy_pll_init(struct tegra_padctl_uphy *uphy)
{
	unsigned long uphy_lane_bitmap;
	unsigned uphy_lane;
	u32 reg;
	int rc;

	uphy_lane_bitmap = uphy->usb3_lanes;
	dev_dbg(uphy->dev, "%s XUSB controller uphy-lanes 0x%lx\n",
		__func__, uphy_lane_bitmap);

	mutex_lock(&uphy->lock);

	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES) {
		reg = uphy_lane_readl(uphy, uphy_lane, UPHY_LANE_MUX);
		reg &= ~SEL(~0);
		reg |= SEL_XUSB;
		uphy_lane_writel(uphy, uphy_lane, reg, UPHY_LANE_MUX);
	}

	/* Reset release of lanes and PLL0 */
	rc = uphy_pll_reset_deassert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_lanes_reset_deassert(uphy, uphy_lane_bitmap);
	if (rc)
		goto assert_pll0_reset;

	/* Program pll defaults */
	pcie_usb3_pll_defaults(uphy);

	/* Program lane defaults */
	for_each_set_bit(uphy_lane, &uphy_lane_bitmap, T186_UPHY_LANES)
		usb3_lane_defaults(uphy, uphy_lane);

	rc = uphy_pll_init(uphy, TEGRA186_FUNC_USB3);
	if (rc)
		goto assert_lanes_reset;

	mutex_unlock(&uphy->lock);
	return 0;

assert_lanes_reset:
	uphy_lanes_reset_assert(uphy, uphy_lane_bitmap);
assert_pll0_reset:
	uphy_pll_reset_assert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);
	return rc;
}

static int tegra186_usb3_uphy_pll_deinit(struct tegra_padctl_uphy *uphy)
{
	unsigned long uphy_lane_bitmap;
	int rc;

	uphy_lane_bitmap = uphy->usb3_lanes;
	dev_dbg(uphy->dev, "%s XUSB controller uphy-lanes 0x%lx\n",
		__func__, uphy_lane_bitmap);

	mutex_lock(&uphy->lock);

	rc = uphy_pll_reset_assert(uphy, 0);
	if (rc)
		goto unlock_out;

	rc = uphy_lanes_reset_assert(uphy, uphy_lane_bitmap);
	if (rc)
		goto deassert_pll0_reset;

	rc = uphy_pll_deinit(uphy, TEGRA186_FUNC_USB3);
	if (rc)
		goto deassert_lanes_reset;

	mutex_unlock(&uphy->lock);

	return 0;

deassert_lanes_reset:
	uphy_lanes_reset_deassert(uphy, uphy_lane_bitmap);
deassert_pll0_reset:
	uphy_pll_reset_deassert(uphy, 0);
unlock_out:
	mutex_unlock(&uphy->lock);

	return rc;
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

	mutex_lock(&uphy->lock);

	tegra_padctl_uphy_enable(phy);

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_usb3_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	int port = usb3_phy_to_port(phy);
	unsigned uphy_lane;

	if (port < 0)
		return port;

	mutex_lock(&uphy->lock);

	uphy_lane = uphy->usb3_ports[port].uphy_lane;
	dev_dbg(dev, "phy exit USB3 port %d uphy-lane-%u\n",
		port, uphy_lane);

	tegra_padctl_uphy_disable(phy);
	mutex_unlock(&uphy->lock);
	return 0;
}

static const struct phy_ops usb3_phy_ops = {
	.init = tegra186_usb3_phy_init,
	.exit = tegra186_usb3_phy_exit,
	.power_on = tegra186_usb3_phy_power_on,
	.power_off = tegra186_usb3_phy_power_off,
	.owner = THIS_MODULE,
};
static inline bool is_usb3_phy(struct phy *phy)
{
	return phy->ops == &usb3_phy_ops;
}

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

	dev_dbg(uphy->dev, "enable sleepwalk UTMI port %d speed %d\n",
		port, speed);

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
	reg |= LINEVAL_WALK_EN;
	reg &= ~WAKE_WALK_EN;
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

	usleep_range(50, 100);

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

	dev_dbg(uphy->dev, "disable sleepwalk UTMI port %d\n",  port);

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

void tegra_phy_xusb_utmi_pad_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;
	u32 reg;

	if (!phy)
		return;

	uphy = phy_get_drvdata(phy);
	port = utmi_phy_to_port(phy);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	reg &= ~USB2_OTG_PD;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	reg &= ~USB2_OTG_PD_DR;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_utmi_pad_power_on);

void tegra_phy_xusb_utmi_pad_power_down(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;
	u32 reg;

	if (!phy)
		return;

	uphy = phy_get_drvdata(phy);
	port = utmi_phy_to_port(phy);

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));
	reg |= USB2_OTG_PD;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	reg |= USB2_OTG_PD_DR;
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_utmi_pad_power_down);

void tegra_phy_xusb_utmi_pad_chg_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;
	u32 reg;

	if (!phy)
		return;

	uphy = phy_get_drvdata(phy);
	port = utmi_phy_to_port(phy);

	reg = padctl_readl(uphy, USB2_BATTERY_CHRG_OTGPADX_CTL0(port));
	reg &= ~PD_CHG;
	padctl_writel(uphy, reg, USB2_BATTERY_CHRG_OTGPADX_CTL0(port));
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_utmi_pad_chg_power_on);

void tegra_phy_xusb_utmi_pad_chg_power_down(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;
	u32 reg;

	if (!phy)
		return;

	uphy = phy_get_drvdata(phy);
	port = utmi_phy_to_port(phy);

	reg = padctl_readl(uphy, USB2_BATTERY_CHRG_OTGPADX_CTL0(port));
	reg |= PD_CHG;
	padctl_writel(uphy, reg, USB2_BATTERY_CHRG_OTGPADX_CTL0(port));
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_utmi_pad_chg_power_down);

static int tegra186_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	struct device *dev = uphy->dev;
	int port = utmi_phy_to_port(phy);
	char prod_name[] = "prod_c_utmiX";
	int err;
	u32 reg;

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "power on UTMI port %d\n",  port);

	sprintf(prod_name, "prod_c_utmi%d", port);
	err = tegra_prod_set_by_name(&uphy->padctl_regs, prod_name,
		uphy->prod_list);
	if (err) {
		dev_info(dev, "failed to apply prod for utmi pad%d (%d)\n",
			port, err);
	}

	sprintf(prod_name, "prod_c_bias");
	err = tegra_prod_set_by_name(&uphy->padctl_regs, prod_name,
		uphy->prod_list);
	if (err)
		dev_info(dev, "failed to apply prod for bias pad (%d)\n", err);

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
	reg &= ~USB2_OTG_PD_ZI;
	reg |= TERM_SEL;
	reg &= ~HS_CURR_LEVEL(~0);
	if (uphy->calib.hs_curr_level_offset) {
		int hs_current_level;

		dev_dbg(uphy->dev, "UTMI port %d apply hs_curr_level_offset %d\n",
			port, uphy->calib.hs_curr_level_offset);

		hs_current_level = (int) uphy->calib.hs_curr_level[port] +
			uphy->calib.hs_curr_level_offset;

		if (hs_current_level < 0)
			hs_current_level = 0;
		if (hs_current_level > 0x3f)
			hs_current_level = 0x3f;

		reg |= HS_CURR_LEVEL(hs_current_level);
	} else
		reg |= HS_CURR_LEVEL(uphy->calib.hs_curr_level[port]);
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(port));

	reg = padctl_readl(uphy, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));
	reg &= ~TERM_RANGE_ADJ(~0);
	reg |= TERM_RANGE_ADJ(uphy->calib.hs_term_range_adj);
	reg &= ~RPD_CTRL(~0);
	reg |= RPD_CTRL(uphy->calib.rpd_ctrl);
	padctl_writel(uphy, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(port));

	reg = padctl_readl(uphy, USB2_BATTERY_CHRG_OTGPADX_CTL1(port));
	reg |= VREG_FIX18;
	padctl_writel(uphy, reg, USB2_BATTERY_CHRG_OTGPADX_CTL1(port));

	mutex_lock(&uphy->lock);

	if (uphy->utmi_enable++ > 0)
		goto out;

	err = clk_prepare_enable(uphy->utmipll);
	if (err)
		dev_err(uphy->dev, "failed to enable UTMIPLL %d\n", err);

	/* BIAS PAD */
	err = clk_prepare_enable(uphy->usb2_trk_clk);
	if (err) {
		dev_err(uphy->dev, "failed to enable USB2 tracking clock %d\n",
			err);
	}

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

	clk_disable_unprepare(uphy->usb2_trk_clk);
out:
	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_utmi_phy_power_off(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	u32 reg;

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "power off UTMI port %d\n", port);

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

	clk_disable_unprepare(uphy->utmipll);
out:
	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_utmi_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	int rc;

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "phy init UTMI port %d\n",  port);

	mutex_lock(&uphy->lock);

	if (uphy->vbus[port] && uphy->utmi_ports[port].port_cap == HOST_ONLY) {
		rc = regulator_enable(uphy->vbus[port]);
		if (rc) {
			dev_err(uphy->dev, "enable port %d vbus failed %d\n",
				port, rc);
			mutex_unlock(&uphy->lock);
			return rc;
		}
	}

	tegra_padctl_uphy_enable(phy);

	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_utmi_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = utmi_phy_to_port(phy);
	int rc;

	if (port < 0)
		return port;

	dev_dbg(uphy->dev, "phy exit UTMI port %d\n",  port);

	mutex_lock(&uphy->lock);

	if (uphy->vbus[port] && uphy->utmi_ports[port].port_cap == HOST_ONLY) {
		rc = regulator_disable(uphy->vbus[port]);
		if (rc) {
			dev_err(uphy->dev, "disable port %d vbus failed %d\n",
				port, rc);
			mutex_unlock(&uphy->lock);
			return rc;
		}
	}

	tegra_padctl_uphy_disable(phy);

	mutex_unlock(&uphy->lock);

	return 0;
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
	return phy->ops == &utmi_phy_ops;
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

enum hsic_pad_pupd {
	PUPD_DISABLE = 0,
	PUPD_IDLE,
	PUPD_RESET
};

static int tegra186_hsic_phy_pupd_set(struct tegra_padctl_uphy *uphy, int pad,
				      enum hsic_pad_pupd pupd)
{
	struct device *dev = uphy->dev;
	u32 reg;

	if (pad >= 1) {
		dev_err(dev, "%s invalid HSIC pad number %u\n", __func__, pad);
		return -EINVAL;
	}

	dev_dbg(dev, "%s pad %u pupd %d\n", __func__, pad, pupd);

	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PADX_CTL0(pad));
	reg &= ~(HSIC_RPD_DATA0 | HSIC_RPU_DATA0);
	reg &= ~(HSIC_RPU_STROBE | HSIC_RPD_STROBE);
	if (pupd == PUPD_IDLE) {
		reg |= (HSIC_RPD_DATA0 | HSIC_RPU_STROBE);
	} else if (pupd == PUPD_RESET) {
		reg |= (HSIC_RPD_DATA0 | HSIC_RPD_STROBE);
	} else if (pupd != PUPD_DISABLE) {
		dev_err(dev, "%s invalid pupd %d\n", __func__, pupd);
		return -EINVAL;
	}
	padctl_writel(uphy, reg, XUSB_PADCTL_HSIC_PADX_CTL0(pad));

	return 0;
}

static ssize_t hsic_power_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_padctl_uphy *uphy = platform_get_drvdata(pdev);
	int pad = 0;
	u32 reg;
	int on;

	reg = padctl_readl(uphy, XUSB_PADCTL_HSIC_PADX_CTL0(pad));

	if (reg & (HSIC_RPD_DATA0 | HSIC_RPD_STROBE))
		on = 0; /* bus in reset */
	else
		on = 1;

	return sprintf(buf, "%d\n", on);
}

static ssize_t hsic_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_padctl_uphy *uphy = platform_get_drvdata(pdev);
	struct tegra_xusb_mbox_msg msg;
	unsigned int on;
	int port;
	int rc;

	if (kstrtouint(buf, 10, &on))
		return -EINVAL;

	if (uphy->host_mode_phy_disabled) {
		dev_err(dev, "doesn't support HSIC PHY because mailbox is not available\n");
		return -EINVAL;
	}

	if (on)
		msg.cmd = MBOX_CMD_AIRPLANE_MODE_DISABLED;
	else
		msg.cmd = MBOX_CMD_AIRPLANE_MODE_ENABLED;

	port = uphy->soc->hsic_port_offset;
	msg.data = BIT(port + 1);
	rc = mbox_send_message(uphy->mbox_chan, &msg);
	if (rc < 0)
		dev_err(dev, "failed to send message to firmware %d\n", rc);

	if (on)
		rc = tegra186_hsic_phy_pupd_set(uphy, 0, PUPD_IDLE);
	else
		rc = tegra186_hsic_phy_pupd_set(uphy, 0, PUPD_RESET);

	return n;
}
static DEVICE_ATTR(hsic_power, S_IRUGO | S_IWUSR,
		   hsic_power_show, hsic_power_store);

static ssize_t otg_vbus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_padctl_uphy *uphy = platform_get_drvdata(pdev);
	int port = uphy->utmi_otg_port_base_1 - 1;

	if (!uphy->utmi_otg_port_base_1)
		return sprintf(buf, "No UTMI OTG port\n");

	return sprintf(buf, "OTG port %d vbus always-on: %s\n",
			port, uphy->otg_vbus_alwayson ? "yes" : "no");
}

static ssize_t otg_vbus_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_padctl_uphy *uphy = platform_get_drvdata(pdev);
	int port = uphy->utmi_otg_port_base_1 - 1;
	unsigned int on;
	int err = 0;

	if (kstrtouint(buf, 10, &on))
		return -EINVAL;

	if (!uphy->utmi_otg_port_base_1) {
		dev_err(dev, "No UTMI OTG port\n");
		return -EINVAL;
	}

	if (!uphy->vbus[port]) {
		dev_err(dev, "UTMI OTG port %d has no vbus regulator\n", port);
		return -EINVAL;
	}

	if (on && !uphy->otg_vbus_alwayson) {
		err = regulator_enable(uphy->vbus[port]);
		if (!err)
			uphy->otg_vbus_alwayson = true;
	} else if (!on && uphy->otg_vbus_alwayson) {
		err = regulator_disable(uphy->vbus[port]);
		if (!err)
			uphy->otg_vbus_alwayson = false;
	}

	if (err)
		dev_err(dev, "failed to %s OTG port %d vbus always-on: %d\n",
				on ? "enable" : "disable", port, err);

	return n;
}

static DEVICE_ATTR(otg_vbus, S_IRUGO | S_IWUSR, otg_vbus_show, otg_vbus_store);

static struct attribute *padctl_uphy_attrs[] = {
	&dev_attr_hsic_power.attr,
	&dev_attr_otg_vbus.attr,
	NULL,
};
static struct attribute_group padctl_uphy_attr_group = {
	.attrs = padctl_uphy_attrs,
};

static int tegra186_hsic_phy_pretend_connected(struct tegra_padctl_uphy *uphy
					, int port)
{
	struct device *dev = uphy->dev;
	struct tegra_xusb_mbox_msg msg;
	int rc;

	if (!uphy->hsic_ports[port].pretend_connected)
		return 0; /* pretend-connected is not enabled */

	msg.cmd = MBOX_CMD_HSIC_PRETEND_CONNECT;
	msg.data = BIT(uphy->soc->hsic_port_offset + port + 1);
	rc = mbox_send_message(uphy->mbox_chan, &msg);
	if (rc < 0)
		dev_err(dev, "failed to send message to firmware %d\n", rc);

	return rc;
}

static int tegra186_hsic_phy_enable_sleepwalk(struct tegra_padctl_uphy *uphy,
					      int port)
{
	u32 reg;

	dev_dbg(uphy->dev, "enable sleepwalk HSIC port %d\n",  port);

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

	usleep_range(50, 100);

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

	dev_dbg(uphy->dev, "disable sleepwalk HSIC port %d\n",  port);

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
	rc = tegra_prod_set_by_name(&uphy->padctl_regs, prod_name,
				    uphy->prod_list);
	if (rc) {
		dev_info(uphy->dev, "failed to apply prod for hsic pad%d (%d)\n",
			port, rc);
	}

	rc = regulator_enable(uphy->vddio_hsic);
	if (rc) {
		dev_err(uphy->dev, "enable hsic %d power failed %d\n",
			port, rc);
		return rc;
	}

	rc = clk_prepare_enable(uphy->utmipll);
	if (rc)
		dev_err(uphy->dev, "failed to enable UTMIPLL %d\n", rc);

	rc = clk_prepare_enable(uphy->hsic_trk_clk);
	if (rc) {
		dev_err(uphy->dev, "failed to enable HSIC tracking clock %d\n",
			rc);
	}

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

	clk_disable_unprepare(uphy->hsic_trk_clk);

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

	clk_disable_unprepare(uphy->utmipll);

	return 0;
}

static int tegra186_hsic_phy_init(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);

	mutex_lock(&uphy->lock);

	dev_dbg(uphy->dev, "phy init HSIC port %d\n", port);
	tegra_padctl_uphy_enable(phy);

	mutex_unlock(&uphy->lock);
	return 0;
}

static int tegra186_hsic_phy_exit(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	int port = hsic_phy_to_port(phy);

	mutex_lock(&uphy->lock);

	dev_dbg(uphy->dev, "phy exit HSIC port %d\n", port);
	tegra_padctl_uphy_disable(phy);

	mutex_unlock(&uphy->lock);
	return 0;
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
	return phy->ops == &hsic_phy_ops;
}

static void tegra_xusb_phy_mbox_work(struct work_struct *work)
{
	struct tegra_padctl_uphy *uphy = mbox_work_to_uphy(work);
	struct tegra_xusb_mbox_msg *msg = &uphy->mbox_req;
	struct tegra_xusb_mbox_msg resp;
	u32 ports;

	dev_dbg(uphy->dev, "mailbox command %d\n", msg->cmd);
	resp.cmd = 0;
	switch (msg->cmd) {
	case MBOX_CMD_START_HSIC_IDLE:
	case MBOX_CMD_STOP_HSIC_IDLE:
		ports = msg->data >> (uphy->soc->hsic_port_offset + 1);
		resp.data = msg->data;
		resp.cmd = MBOX_CMD_ACK;
		if (msg->cmd == MBOX_CMD_START_HSIC_IDLE)
			tegra186_hsic_phy_pupd_set(uphy, 0, PUPD_IDLE);
		else
			tegra186_hsic_phy_pupd_set(uphy, 0, PUPD_DISABLE);
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

static const char * const tegra186_supply_names[] = {
	"hvdd_pex_pll",		/* 1.8V, pex_pll_hvdd */
	"dvdd_pex",		/* 1.05V, pex_pll_dvdd, pex_l*_dvdd */
	"hvdd_pex",		/* 1.8V, pex_l*_hvdd */
	"avdd_usb",		/* 3.3V, vddp_usb */
	"vclamp_usb",		/* 1.8V, vclamp_usb_init */
	"avdd_pll_erefeut",	/* 1.8V, pll_utmip_avdd */
};

static const struct tegra_padctl_uphy_soc tegra186_soc = {
	.num_pins = ARRAY_SIZE(tegra186_pins),
	.pins = tegra186_pins,
	.num_functions = ARRAY_SIZE(tegra186_functions),
	.functions = tegra186_functions,
	.num_lanes = ARRAY_SIZE(tegra186_lanes),
	.lanes = tegra186_lanes,
	.hsic_port_offset = 6,
	.supply_names = tegra186_supply_names,
	.num_supplies = ARRAY_SIZE(tegra186_supply_names),
};

static const struct of_device_id tegra_padctl_uphy_of_match[] = {
	{.compatible = "nvidia,tegra186-padctl-uphy", .data = &tegra186_soc},
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_padctl_uphy_of_match);

static int tegra_xusb_read_fuse_calibration(struct tegra_padctl_uphy *uphy)
{
	struct platform_device *pdev = to_platform_device(uphy->dev);
	struct device_node *np = pdev->dev.of_node;
	unsigned int i;
	u32 reg;
	s32 v;

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

	if (of_property_read_s32(np, "nvidia,hs_curr_level_offset", &v) == 0) {
		dev_dbg(uphy->dev, "HS current level offset %d\n", v);
		uphy->calib.hs_curr_level_offset = v;
	}

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

static void tegra_xusb_otg_vbus_work(struct work_struct *work)
{
	struct tegra_padctl_uphy *uphy =
		container_of(work, struct tegra_padctl_uphy, otg_vbus_work);
	struct device *dev = uphy->dev;
	int port = uphy->utmi_otg_port_base_1 - 1;
	u32 reg;
	int rc;

	if (!uphy->utmi_otg_port_base_1)
		return; /* nothing to do if there is no UTMI otg port */

	reg = padctl_readl(uphy, USB2_VBUS_ID);
	dev_dbg(dev, "USB2_VBUS_ID 0x%x otg_vbus_on was %d\n", reg,
		uphy->otg_vbus_on);
	if ((reg & ID_OVERRIDE(~0)) == ID_OVERRIDE_GROUNDED) {
		/* entering host mode role */
		if (uphy->vbus[port] && !uphy->otg_vbus_on) {
			rc = regulator_enable(uphy->vbus[port]);
			if (rc) {
				dev_err(dev, "failed to enable otg port vbus %d\n"
					, rc);
			}
			uphy->otg_vbus_on = true;
		}
	} else if ((reg & ID_OVERRIDE(~0)) == ID_OVERRIDE_FLOATING) {
		/* leaving host mode role */
		if (uphy->vbus[port] && uphy->otg_vbus_on) {
			rc = regulator_disable(uphy->vbus[port]);
			if (rc) {
				dev_err(dev, "failed to disable otg port vbus %d\n"
					, rc);
			}
			uphy->otg_vbus_on = false;
		}
	}
}

static int tegra_xusb_setup_usb(struct tegra_padctl_uphy *uphy)
{
	struct phy *phy;
	unsigned int i;

	for (i = 0; i < TEGRA_USB3_PHYS; i++) {
		if (uphy->usb3_ports[i].port_cap == CAP_DISABLED)
			continue;
		if (uphy->host_mode_phy_disabled &&
			(uphy->usb3_ports[i].port_cap == HOST_ONLY))
			continue; /* no mailbox support */

		phy = devm_phy_create(uphy->dev, NULL, &usb3_phy_ops, NULL);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		uphy->usb3_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	for (i = 0; i < TEGRA_UTMI_PHYS; i++) {
		char reg_name[sizeof("vbus-N")];

		if (uphy->utmi_ports[i].port_cap == CAP_DISABLED)
			continue;
		if (uphy->host_mode_phy_disabled &&
			(uphy->utmi_ports[i].port_cap == HOST_ONLY))
			continue; /* no mailbox support */

		sprintf(reg_name, "vbus-%d", i);
		uphy->vbus[i] = devm_regulator_get_optional(uphy->dev,
							    reg_name);
		if (IS_ERR(uphy->vbus[i])) {
			if (PTR_ERR(uphy->vbus[i]) == -EPROBE_DEFER)
				return -EPROBE_DEFER;

			uphy->vbus[i] = NULL;
		}

		phy = devm_phy_create(uphy->dev, NULL, &utmi_phy_ops, NULL);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		uphy->utmi_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	if (uphy->host_mode_phy_disabled)
		goto skip_hsic; /* no mailbox support */

	uphy->vddio_hsic = devm_regulator_get(uphy->dev, "vddio-hsic");
	if (IS_ERR(uphy->vddio_hsic))
		return PTR_ERR(uphy->vddio_hsic);

	for (i = 0; i < TEGRA_HSIC_PHYS; i++) {
		phy = devm_phy_create(uphy->dev, NULL, &hsic_phy_ops, NULL);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		uphy->hsic_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

skip_hsic:
	return 0;
}

#ifdef DEBUG
#define reg_dump(_dev, _base, _reg) \
	dev_dbg(_dev, "%s @%x = 0x%x\n", #_reg, _reg, ioread32(_base + _reg))
#else
#define reg_dump(_dev, _base, _reg)	do {} while (0)
#endif

static int tegra186_uphy_pll_init(struct tegra_padctl_uphy *uphy)
{
	int rc;

	if (uphy->pcie_lanes) {
		rc = tegra186_pcie_uphy_pll_init(uphy);
		if (rc)
			return rc;
	}
	if (uphy->usb3_lanes) {
		rc = tegra186_usb3_uphy_pll_init(uphy);
		if (rc)
			return rc;
	}

	if (uphy->sata_lanes) {
		rc = tegra186_sata_uphy_pll_init(uphy);
		if (rc)
			return rc;
	}

	if (uphy->ufs_lanes) {
		rc = tegra186_ufs_uphy_pll_init(uphy);
		if (rc)
			return rc;
	}

	return 0;
}

static int tegra186_uphy_pll_deinit(struct tegra_padctl_uphy *uphy)
{
	int rc;

	if (uphy->pcie_lanes) {
		rc = tegra186_pcie_uphy_pll_deinit(uphy);
		if (rc)
			return rc;
	}

	if (uphy->usb3_lanes) {
		rc = tegra186_usb3_uphy_pll_deinit(uphy);
		if (rc)
			return rc;
	}

	if (uphy->sata_lanes) {
		rc = tegra186_sata_uphy_pll_deinit(uphy);
		if (rc)
			return rc;
	}

	if (uphy->ufs_lanes) {
		rc = tegra186_ufs_uphy_pll_deinit(uphy);
		if (rc)
			return rc;
	}

	return 0;
}

static void tegra186_padctl_save(struct tegra_padctl_uphy *uphy)
{
	uphy->padctl_context.vbus_id = padctl_readl(uphy, USB2_VBUS_ID);
}

static void tegra186_padctl_restore(struct tegra_padctl_uphy *uphy)
{
	padctl_writel(uphy, uphy->padctl_context.vbus_id, USB2_VBUS_ID);
}

static int tegra186_padctl_uphy_suspend(struct device *dev)
{
	struct tegra_padctl_uphy *uphy = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	tegra186_padctl_save(uphy);

	return tegra186_uphy_pll_deinit(uphy);
}

static int tegra186_padctl_uphy_resume(struct device *dev)
{
	struct tegra_padctl_uphy *uphy = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	tegra186_padctl_restore(uphy);

	return tegra186_uphy_pll_init(uphy);
}

static const struct dev_pm_ops tegra186_padctl_uphy_pm_ops = {
	.suspend_noirq = tegra186_padctl_uphy_suspend,
	.resume_noirq = tegra186_padctl_uphy_resume,
};

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
		char mgmt_clk[] = "uphy_pllX_mgmt";
		char hwseq_clk[] = "uphy_pllX_pwrseq";

		snprintf(rst_name, sizeof(rst_name), "uphy_pll%d_rst", i);
		uphy->uphy_pll_rst[i] = devm_reset_control_get(dev, rst_name);
		if (IS_ERR(uphy->uphy_pll_rst[i])) {
			dev_err(uphy->dev, "fail get uphy_pll%d reset\n", i);
			return PTR_ERR(uphy->uphy_pll_rst[i]);
		}

		snprintf(mgmt_clk, sizeof(mgmt_clk), "uphy_pll%d_mgmt", i);
		uphy->uphy_pll_mgmt[i] = devm_clk_get(dev, mgmt_clk);
		if (IS_ERR(uphy->uphy_pll_mgmt[i])) {
			dev_err(dev, "failed to get pll%d mgmt clock\n", i);
			return PTR_ERR(uphy->uphy_pll_mgmt[i]);
		}

		snprintf(hwseq_clk, sizeof(hwseq_clk), "uphy_pll%d_pwrseq", i);
		uphy->uphy_pll_pwrseq[i] = devm_clk_get(dev, hwseq_clk);
		if (IS_ERR(uphy->uphy_pll_pwrseq[i])) {
			dev_err(dev, "failed to get pll%d pwrseq clock\n", i);
			return PTR_ERR(uphy->uphy_pll_pwrseq[i]);
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

	uphy->padctl_rst = devm_reset_control_get(dev, "padctl_rst");
	if (IS_ERR(uphy->padctl_rst)) {
		dev_err(uphy->dev, "failed to get padctl reset\n");
		return PTR_ERR(uphy->padctl_rst);
	}

	uphy->xusb_clk = devm_clk_get(dev, "xusb_clk");
	if (IS_ERR(uphy->xusb_clk)) {
		dev_err(dev, "failed to get xusb_clk clock\n");
		return PTR_ERR(uphy->xusb_clk);
	}

	uphy->pllp = devm_clk_get(dev, "pllp");
	if (IS_ERR(uphy->pllp)) {
		dev_err(dev, "failed to get pllp clock\n");
		return PTR_ERR(uphy->pllp);
	}

	uphy->plle = devm_clk_get(dev, "plle");
	if (IS_ERR(uphy->plle)) {
		dev_err(dev, "failed to get plle clock\n");
		return PTR_ERR(uphy->plle);
	}

	uphy->plle_pwrseq = devm_clk_get(dev, "plle_pwrseq");
	if (IS_ERR(uphy->plle_pwrseq)) {
		dev_err(dev, "failed to get plle_pwrseq clock\n");
		return PTR_ERR(uphy->plle_pwrseq);
	}

	uphy->pllrefe = devm_clk_get(dev, "pllrefe");
	if (IS_ERR(uphy->pllrefe)) {
		dev_err(dev, "failed to get pllrefe clock\n");
		return PTR_ERR(uphy->pllrefe);
	}

	uphy->utmipll = devm_clk_get(dev, "utmipll");
	if (IS_ERR(uphy->utmipll)) {
		dev_err(dev, "failed to get utmipll clock\n");
		return PTR_ERR(uphy->utmipll);
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

	uphy->rx_byp_clk = devm_clk_get(dev, "rx_byp");
	if (IS_ERR(uphy->rx_byp_clk)) {
		dev_err(dev, "failed to get rx_byp clock\n");
		return PTR_ERR(uphy->rx_byp_clk);
	}

	uphy->pllrefe_pex = devm_clk_get(dev, "pllrefe_pex");
	if (IS_ERR(uphy->pllrefe_pex)) {
		dev_err(dev, "failed to get pllrefe_pex clock\n");
		return PTR_ERR(uphy->pllrefe_pex);
	}

	uphy->plle_passthrough = devm_clk_get(dev, "plle_passthrough");
	if (IS_ERR(uphy->plle_passthrough)) {
		dev_err(dev, "failed to get plle_passthrough clock\n");
		return PTR_ERR(uphy->plle_passthrough);
	}

	err = tegra186_padctl_uphy_regulators_init(uphy);
	if (err < 0)
		return err;

	err = regulator_bulk_enable(uphy->soc->num_supplies, uphy->supplies);
	if (err) {
		dev_err(dev, "failed to enable regulators %d\n", err);
		return err;
	}

	/* enable uphy management clock - pllp */
	err = clk_prepare_enable(uphy->pllp);
	if (err) {
		dev_err(dev, "failed to enable pllp %d\n", err);
		goto disable_regulators;
	}

	err = clk_prepare_enable(uphy->xusb_clk);
	if (err) {
		dev_err(dev, "failed to enable xusb_clk %d\n", err);
		goto disable_regulators;
	}

	err = reset_control_deassert(uphy->uphy_master_rst);
	if (err) {
		dev_err(dev, "failed to deassert uphy_master_rst %d\n", err);
		goto disable_pllp;
	}

	err = reset_control_deassert(uphy->uphy_rst);
	if (err) {
		dev_err(dev, "failed to deassert uphy_rst %d\n", err);
		goto assert_uphy_master_rst;
	}

	err = reset_control_deassert(uphy->padctl_rst);
	if (err) {
		dev_err(dev, "failed to deassert padctl_rst %d\n", err);
		goto assert_uphy_master_rst;
	}

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

	err = tegra186_uphy_pll_init(uphy);
	if (err) {
		dev_err(dev, "failed to initialize UPHY PLLs %d\n", err);
		goto unregister;
	}

	for (i = 0; i < TEGRA_PCIE_PHYS; i++) {
		phy = devm_phy_create(dev, NULL, &pcie_phy_ops, NULL);
		if (IS_ERR(phy)) {
			err = PTR_ERR(phy);
			goto uphy_pll_deinit;
		}
		uphy->pcie_phys[i] = phy;
		phy_set_drvdata(phy, uphy);
	}

	phy = devm_phy_create(dev, NULL, &sata_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto uphy_pll_deinit;
	}
	uphy->sata_phys[0] = phy;
	phy_set_drvdata(phy, uphy);

	phy = devm_phy_create(dev, NULL, &ufs_phy_ops, NULL);
	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto uphy_pll_deinit;
	}
	uphy->ufs_phys[0] = phy;
	phy_set_drvdata(phy, uphy);

	INIT_WORK(&uphy->otg_vbus_work, tegra_xusb_otg_vbus_work);
	INIT_WORK(&uphy->mbox_req_work, tegra_xusb_phy_mbox_work);
	uphy->mbox_client.dev = dev;
	uphy->mbox_client.tx_block = true;
	uphy->mbox_client.tx_tout = 0;
	uphy->mbox_client.rx_callback = tegra_xusb_phy_mbox_rx;
	uphy->mbox_chan = mbox_request_channel(&uphy->mbox_client, 0);
	if (IS_ERR(uphy->mbox_chan)) {
		err = PTR_ERR(uphy->mbox_chan);
		if (err == -EPROBE_DEFER) {
			dev_info(&pdev->dev, "mailbox is not ready yet\n");
			goto uphy_pll_deinit;
		} else {
			dev_warn(&pdev->dev,
				 "failed to get mailbox, USB Host PHY support disabled\n");
			uphy->host_mode_phy_disabled = true;
		}
	}

	err = tegra_xusb_setup_usb(uphy);
	if (err)
		goto free_mailbox;


	uphy->provider = devm_of_phy_provider_register(dev,
					tegra186_padctl_uphy_xlate);
	if (IS_ERR(uphy->provider)) {
		err = PTR_ERR(uphy->provider);
		dev_err(&pdev->dev, "failed to register PHYs: %d\n", err);
		goto free_mailbox;
	}

	err = sysfs_create_group(&pdev->dev.kobj, &padctl_uphy_attr_group);
	if (err) {
		dev_err(&pdev->dev, "cannot create sysfs group: %d\n", err);
		goto free_mailbox;
	}

	uphy->prod_list = tegra_prod_init(pdev->dev.of_node);
	if (IS_ERR(uphy->prod_list)) {
		dev_warn(&pdev->dev, "Prod-settings not available\n");
		uphy->prod_list = NULL;
	}

	uphy->sata_bypass_fuse =
		of_property_read_bool(np, "nvidia,sata-use-prods");

	return 0;

free_mailbox:
	if (!IS_ERR(uphy->mbox_chan)) {
		cancel_work_sync(&uphy->mbox_req_work);
		mbox_free_channel(uphy->mbox_chan);
	}
uphy_pll_deinit:
	tegra186_uphy_pll_deinit(uphy);
unregister:
	pinctrl_unregister(uphy->pinctrl);
assert_uphy_reset:
	reset_control_assert(uphy->uphy_rst);
assert_uphy_master_rst:
	reset_control_assert(uphy->uphy_master_rst);
disable_pllp:
	clk_disable_unprepare(uphy->pllp);
disable_regulators:
	regulator_bulk_disable(uphy->soc->num_supplies, uphy->supplies);
	return err;
}

static int tegra186_padctl_uphy_remove(struct platform_device *pdev)
{
	struct tegra_padctl_uphy *uphy = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &padctl_uphy_attr_group);

	if (!IS_ERR(uphy->mbox_chan)) {
		cancel_work_sync(&uphy->mbox_req_work);
		mbox_free_channel(uphy->mbox_chan);
	}

	pinctrl_unregister(uphy->pinctrl);
	reset_control_assert(uphy->uphy_master_rst);
	reset_control_assert(uphy->uphy_rst);
	if (uphy->prod_list)
		tegra_prod_release(&uphy->prod_list);

	clk_disable_unprepare(uphy->pllp);
	regulator_bulk_disable(uphy->soc->num_supplies, uphy->supplies);
	return 0;
}

static struct platform_driver tegra186_padctl_uphy_driver = {
	.driver = {
		.name = "tegra186-padctl-uphy",
		.of_match_table = tegra_padctl_uphy_of_match,
		.pm = &tegra186_padctl_uphy_pm_ops,
	},
	.probe = tegra186_padctl_uphy_probe,
	.remove = tegra186_padctl_uphy_remove,
};
module_platform_driver(tegra186_padctl_uphy_driver);

/* Tegra Generic PHY Extensions */
bool tegra_phy_get_lane_rdet(struct phy *phy, u8 lane_num)
{
	struct tegra_padctl_uphy *uphy = phy_get_drvdata(phy);
	u32 data;

	data = uphy_lane_readl(uphy, lane_num, UPHY_LANE_AUX_CTL_1);
	data = data & AUX_TX_RDET_STATUS;
	return !(!data);
}
EXPORT_SYMBOL_GPL(tegra_phy_get_lane_rdet);

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
	} else if (is_usb3_phy(phy)) {
		port = usb3_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_usb3_phy_enable_wakelogic(uphy, port);
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
	} else if (is_usb3_phy(phy)) {
		port = usb3_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_usb3_phy_disable_wakelogic(uphy, port);
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_disable_sleepwalk);

static int tegra186_padctl_vbus_override(struct tegra_padctl_uphy *uphy,
					 bool on)
{
	u32 reg;

	reg = padctl_readl(uphy, USB2_VBUS_ID);
	if (on) {
		reg |= VBUS_OVERRIDE;
		reg &= ~ID_OVERRIDE(~0);
		reg |= ID_OVERRIDE_FLOATING;
	} else
		reg &= ~VBUS_OVERRIDE;
	padctl_writel(uphy, reg, USB2_VBUS_ID);

	schedule_work(&uphy->otg_vbus_work);

	return 0;
}

int tegra_phy_xusb_set_vbus_override(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	return tegra186_padctl_vbus_override(uphy, true);
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_set_vbus_override);

int tegra_phy_xusb_clear_vbus_override(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	return tegra186_padctl_vbus_override(uphy, false);
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_clear_vbus_override);

static int tegra186_padctl_id_override(struct tegra_padctl_uphy *uphy,
					 bool grounded)
{
	u32 reg;

	reg = padctl_readl(uphy, USB2_VBUS_ID);
	if (grounded) {
		if (reg & VBUS_OVERRIDE) {
			reg &= ~VBUS_OVERRIDE;
			padctl_writel(uphy, reg, USB2_VBUS_ID);
			usleep_range(1000, 2000);

			reg = padctl_readl(uphy, USB2_VBUS_ID);
		}

		reg &= ~ID_OVERRIDE(~0);
		reg |= ID_OVERRIDE_GROUNDED;
	} else {
		reg &= ~ID_OVERRIDE(~0);
		reg |= ID_OVERRIDE_FLOATING;
	}
	padctl_writel(uphy, reg, USB2_VBUS_ID);

	schedule_work(&uphy->otg_vbus_work);

	return 0;
}

int tegra_phy_xusb_set_id_override(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	return tegra186_padctl_id_override(uphy, true);
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_set_id_override);

int tegra_phy_xusb_clear_id_override(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	return tegra186_padctl_id_override(uphy, false);
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_clear_id_override);

bool tegra_phy_xusb_has_otg_cap(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;

	if (!phy)
		return false;

	uphy = phy_get_drvdata(phy);
	if (is_utmi_phy(phy)) {
		if ((uphy->utmi_otg_port_base_1) &&
			uphy->utmi_phys[uphy->utmi_otg_port_base_1 - 1] == phy)
			return true;
	} else if (is_usb3_phy(phy)) {
		if ((uphy->usb3_otg_port_base_1) &&
			uphy->usb3_phys[uphy->usb3_otg_port_base_1 - 1] == phy)
			return true;
	}

	return false;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_has_otg_cap);

static int tegra186_usb3_phy_set_wake(struct tegra_padctl_uphy *uphy,
					 int port, bool enable)
{
	u32 reg;

	mutex_lock(&uphy->lock);
	if (enable) {
		dev_dbg(uphy->dev, "enable USB3 port %d wake\n", port);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= SS_PORT_WAKEUP_EVENT(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= SS_PORT_WAKE_INTERRUPT_ENABLE(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
		reg |= SSPX_ELPG_VCORE_DOWN(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);
	} else {
		dev_dbg(uphy->dev, "disable USB3 port %d wake\n", port);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg &= ~SS_PORT_WAKE_INTERRUPT_ENABLE(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= SS_PORT_WAKEUP_EVENT(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM_1);
		reg &= ~SSPX_ELPG_VCORE_DOWN(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM_1);
	}
	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_utmi_phy_set_wake(struct tegra_padctl_uphy *uphy,
					 int port, bool enable)
{
	u32 reg;

	mutex_lock(&uphy->lock);
	if (enable) {
		dev_dbg(uphy->dev, "enable UTMI port %d wake\n", port);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= USB2_PORT_WAKEUP_EVENT(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= USB2_PORT_WAKE_INTERRUPT_ENABLE(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

	} else {
		dev_dbg(uphy->dev, "disable UTMI port %d wake\n", port);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg &= ~USB2_PORT_WAKE_INTERRUPT_ENABLE(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= USB2_PORT_WAKEUP_EVENT(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);
	}
	mutex_unlock(&uphy->lock);

	return 0;
}

static int tegra186_hsic_phy_set_wake(struct tegra_padctl_uphy *uphy,
					 int port, bool enable)
{
	u32 reg;

	mutex_lock(&uphy->lock);
	if (enable) {
		dev_dbg(uphy->dev, "enable HSIC port %d wake\n", port);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= USB2_HSIC_PORT_WAKEUP_EVENT(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg |= USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

	} else {
		dev_dbg(uphy->dev, "disable HSIC port %d wake\n", port);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg &= ~USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);

		usleep_range(10, 20);

		reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
		reg &= ~ALL_WAKE_EVENTS;
		reg |= USB2_HSIC_PORT_WAKEUP_EVENT(port);
		padctl_writel(uphy, reg, XUSB_PADCTL_ELPG_PROGRAM);
	}
	mutex_unlock(&uphy->lock);

	return 0;
}

int tegra_phy_xusb_enable_wake(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;


	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	if (is_utmi_phy(phy)) {
		port = utmi_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_utmi_phy_set_wake(uphy, port, true);
	} else if (is_hsic_phy(phy)) {
		port = hsic_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_hsic_phy_set_wake(uphy, port, true);
	} else if (is_usb3_phy(phy)) {
		port = usb3_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_usb3_phy_set_wake(uphy, port, true);
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_enable_wake);

int tegra_phy_xusb_disable_wake(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	if (is_utmi_phy(phy)) {
		port = utmi_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_utmi_phy_set_wake(uphy, port, false);
	} else if (is_hsic_phy(phy)) {
		port = hsic_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_hsic_phy_set_wake(uphy, port, false);
	} else if (is_usb3_phy(phy)) {
		port = usb3_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_usb3_phy_set_wake(uphy, port, false);
	}
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_disable_wake);


static int tegra186_usb3_phy_remote_wake_detected(struct tegra_padctl_uphy *uphy
					, int port)
{
	u32 reg;

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
	if ((reg & SS_PORT_WAKE_INTERRUPT_ENABLE(port)) &&
			(reg & SS_PORT_WAKEUP_EVENT(port)))
		return true;
	else
		return false;
}

static int tegra186_utmi_phy_remote_wake_detected(struct tegra_padctl_uphy *uphy
					, int port)
{
	u32 reg;

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
	if ((reg & USB2_PORT_WAKE_INTERRUPT_ENABLE(port)) &&
			(reg & USB2_PORT_WAKEUP_EVENT(port)))
		return true;
	else
		return false;
}

static int tegra186_hsic_phy_remote_wake_detected(struct tegra_padctl_uphy *uphy
					, int port)
{
	u32 reg;

	reg = padctl_readl(uphy, XUSB_PADCTL_ELPG_PROGRAM);
	if ((reg & USB2_HSIC_PORT_WAKE_INTERRUPT_ENABLE(port)) &&
			(reg & USB2_HSIC_PORT_WAKEUP_EVENT(port)))
		return true;
	else
		return false;
}

int tegra_phy_xusb_remote_wake_detected(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	if (is_utmi_phy(phy)) {
		port = utmi_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_utmi_phy_remote_wake_detected(uphy, port);
	} else if (is_hsic_phy(phy)) {
		port = hsic_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_hsic_phy_remote_wake_detected(uphy, port);
	} else if (is_usb3_phy(phy)) {
		port = usb3_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_usb3_phy_remote_wake_detected(uphy, port);
	}
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_remote_wake_detected);


int tegra_phy_xusb_pretend_connected(struct phy *phy)
{
	struct tegra_padctl_uphy *uphy;
	int port;

	if (!phy)
		return 0;

	uphy = phy_get_drvdata(phy);

	/* applicable to HSIC only */
	if (is_hsic_phy(phy)) {
		port = hsic_phy_to_port(phy);
		if (port < 0)
			return -EINVAL;
		return tegra186_hsic_phy_pretend_connected(uphy, port);
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_phy_xusb_pretend_connected);

MODULE_AUTHOR("JC Kuo <jckuo@nvidia.com>");
MODULE_DESCRIPTION("Tegra 186 XUSB PADCTL and UPHY PLL/Lane driver");
MODULE_LICENSE("GPL v2");
