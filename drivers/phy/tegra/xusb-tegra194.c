/*
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
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/tegra-soc.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/tegra_prod.h>

#include <soc/tegra/fuse.h>

#include "xusb.h"

#define TEGRA194_USB3_PHYS	(4)
#define TEGRA194_UTMI_PHYS	(4)

/* FUSE USB_CALIB registers */
#define   HS_CURR_LEVEL_PADX_SHIFT(x)		((x) ? (11 + (x - 1) * 6) : 0)
#define   HS_CURR_LEVEL_PAD_MASK		(0x3f)
#define   HS_TERM_RANGE_ADJ_SHIFT		(7)
#define   HS_TERM_RANGE_ADJ_MASK		(0xf)
#define   HS_SQUELCH_SHIFT			(29)
#define   HS_SQUELCH_MASK			(0x7)

#define   RPD_CTRL_SHIFT			(0)
#define   RPD_CTRL_MASK				(0x1f)

/* XUSB PADCTL registers */
#define XUSB_PADCTL_USB2_PAD_MUX		(0x4)
#define   USB2_PORT_SHIFT(x)			((x) * 2)
#define   USB2_PORT_MASK			(0x3)
#define    PORT_XUSB				(1)

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
#define   ALL_WAKE_EVENTS						\
	(USB2_PORT_WAKEUP_EVENT(0) | USB2_PORT_WAKEUP_EVENT(1) |	\
	 USB2_PORT_WAKEUP_EVENT(2) | USB2_PORT_WAKEUP_EVENT(3) |	\
	 SS_PORT_WAKEUP_EVENT(0) | SS_PORT_WAKEUP_EVENT(1) |	\
	 SS_PORT_WAKEUP_EVENT(2) | SS_PORT_WAKEUP_EVENT(3))

#define XUSB_PADCTL_ELPG_PROGRAM_1		(0x24)
#define   SSPX_ELPG_CLAMP_EN(x)			(1 << (0 + (x) * 3))
#define   SSPX_ELPG_CLAMP_EN_EARLY(x)		(1 << (1 + (x) * 3))
#define   SSPX_ELPG_VCORE_DOWN(x)		(1 << (2 + (x) * 3))

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

#define USB2_VBUS_ID				(0x360)
#define   VBUS_OVERRIDE				(1 << 14)
#define   ID_OVERRIDE(x)			(((x) & 0xf) << 18)
#define   ID_OVERRIDE_FLOATING			ID_OVERRIDE(8)
#define   ID_OVERRIDE_GROUNDED			ID_OVERRIDE(0)

/* XUSB AO registers */
#define XUSB_AO_USB_DEBOUNCE_DEL		(0x4)
#define   UTMIP_LINE_DEB_CNT(x)			((x) & 0xf)

#define XUSB_AO_UTMIP_TRIGGERS(x)		(0x40 + (x) * 4)
#define   CLR_WALK_PTR				(1 << 0)
#define   CAP_CFG				(1 << 1)
#define   CLR_WAKE_ALARM			(1 << 3)

#define XUSB_AO_UTMIP_SAVED_STATE(x)		(0x70 + (x) * 4)
#define   SPEED(x)				((x) & 0x3)
#define     UTMI_HS				SPEED(0)
#define     UTMI_FS				SPEED(1)
#define     UTMI_LS				SPEED(2)
#define     UTMI_RST				SPEED(3)

#define XUSB_AO_UTMIP_SLEEPWALK_CFG(x)		(0xd0 + (x) * 4)
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

#define XUSB_AO_UTMIP_PAD_CFG(x)		(0x130 + (x) * 4)
#define   FSLS_USE_XUSB_AO			(1 << 3)
#define   TRK_CTRL_USE_XUSB_AO			(1 << 4)
#define   RPD_CTRL_USE_XUSB_AO			(1 << 5)
#define   RPU_USE_XUSB_AO			(1 << 6)
#define   VREG_USE_XUSB_AO			(1 << 7)
#define   USBOP_VAL_PD				(1 << 8)
#define   USBON_VAL_PD				(1 << 9)
#define   E_DPD_OVRD_EN				(1 << 10)
#define   E_DPD_OVRD_VAL			(1 << 11)

#define TEGRA194_LANE(_name, _offset, _shift, _mask, _type)		\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.num_funcs = ARRAY_SIZE(tegra194_##_type##_functions),	\
		.funcs = tegra194_##_type##_functions,			\
	}

struct tegra_xusb_fuse_calibration {
	u32 hs_curr_level[TEGRA194_UTMI_PHYS];
	u32 hs_squelch;
	u32 hs_term_range_adj;
	u32 rpd_ctrl;
};

struct tegra194_xusb_padctl {
	struct tegra_xusb_padctl base;

	void __iomem *ao_regs;

	/* prod settings */
	struct tegra_prod *prod_list;
	struct tegra_xusb_fuse_calibration calib;

	/* utmi bias and tracking */
	struct clk *usb2_trk_clk;
	unsigned int bias_pad_enable;
};

static inline void ao_writel(struct tegra194_xusb_padctl *priv, u32 value,
			     unsigned long offset)
{
	dev_dbg(priv->base.dev, "ao %08lx < %08x\n", offset, value);
	writel(value, priv->ao_regs + offset);
}

static inline u32 ao_readl(struct tegra194_xusb_padctl *priv,
			   unsigned long offset)
{
	u32 value = readl(priv->ao_regs + offset);

	dev_dbg(priv->base.dev, "ao %08lx > %08x\n", offset, value);
	return value;
}

static inline struct tegra194_xusb_padctl *
to_tegra194_xusb_padctl(struct tegra_xusb_padctl *padctl)
{
	return container_of(padctl, struct tegra194_xusb_padctl, base);
}

/* USB 2.0 UTMI PHY support */
static struct tegra_xusb_lane *
tegra194_usb2_lane_probe(struct tegra_xusb_pad *pad, struct device_node *np,
			 unsigned int index)
{
	struct tegra_xusb_usb2_lane *usb2;
	int err;

	usb2 = kzalloc(sizeof(*usb2), GFP_KERNEL);
	if (!usb2)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&usb2->base.list);
	usb2->base.soc = &pad->soc->lanes[index];
	usb2->base.index = index;
	usb2->base.pad = pad;
	usb2->base.np = np;

	err = tegra_xusb_lane_parse_dt(&usb2->base, np);
	if (err < 0) {
		kfree(usb2);
		return ERR_PTR(err);
	}

	return &usb2->base;
}

static void tegra194_usb2_lane_remove(struct tegra_xusb_lane *lane)
{
	struct tegra_xusb_usb2_lane *usb2 = to_usb2_lane(lane);

	kfree(usb2);
}

static const struct tegra_xusb_lane_ops tegra194_usb2_lane_ops = {
	.probe = tegra194_usb2_lane_probe,
	.remove = tegra194_usb2_lane_remove,
};

static int tegra194_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_usb2_lane *usb2 = to_usb2_lane(lane);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	struct tegra194_xusb_padctl *priv = to_tegra194_xusb_padctl(padctl);
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	struct tegra_xusb_usb2_port *port;
	u32 reg;

	dev_dbg(dev, "phy power on UTMI %d\n", index);

	port = tegra_xusb_find_usb2_port(padctl, index);
	if (!port) {
		dev_err(dev, "no port found for USB2 lane %u\n", index);
		return -ENODEV;
	}

	if (priv->prod_list) {
		char prod_name[] = "prod_c_utmiX";
		int err;

		sprintf(prod_name, "prod_c_utmi%d", index);
		err = tegra_prod_set_by_name(&padctl->regs, prod_name,
					     priv->prod_list);
		if (err) {
			dev_dbg(dev, "failed to apply prod for utmi pad%d\n",
				index);
		}

		err = tegra_prod_set_by_name(&padctl->regs, "prod_c_bias",
					     priv->prod_list);
		if (err)
			dev_dbg(dev, "failed to apply prod for bias pad\n");
	}

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_PAD_MUX);
	reg &= ~(USB2_PORT_MASK << USB2_PORT_SHIFT(index));
	reg |= (PORT_XUSB << USB2_PORT_SHIFT(index));
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_PAD_MUX);

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_PORT_CAP);
	reg &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(index));
	if (port->port_cap == USB_PORT_DISABLED)
		reg |= (PORT_CAP_DISABLED << PORTX_CAP_SHIFT(index));
	else if (port->port_cap == USB_DEVICE_CAP)
		reg |= (PORT_CAP_DEVICE << PORTX_CAP_SHIFT(index));
	else if (port->port_cap == USB_HOST_CAP)
		reg |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(index));
	else if (port->port_cap == USB_OTG_CAP)
		reg |= (PORT_CAP_OTG << PORTX_CAP_SHIFT(index));
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_PORT_CAP);

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(index));
	reg &= ~USB2_OTG_PD_ZI;
	reg |= TERM_SEL;
	reg &= ~HS_CURR_LEVEL(~0);
	/* TODO hs_curr_level_offset support */
	if (usb2->hs_curr_level_offset) {
		int hs_current_level;

		dev_dbg(dev, "UTMI port %d apply hs_curr_level_offset %d\n",
			index, usb2->hs_curr_level_offset);

		hs_current_level = (int) priv->calib.hs_curr_level[index] +
			usb2->hs_curr_level_offset;

		if (hs_current_level < 0)
			hs_current_level = 0;
		if (hs_current_level > 0x3f)
			hs_current_level = 0x3f;

		reg |= HS_CURR_LEVEL(hs_current_level);
	} else
		reg |= HS_CURR_LEVEL(priv->calib.hs_curr_level[index]);
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(index));

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));
	reg &= ~TERM_RANGE_ADJ(~0);
	reg |= TERM_RANGE_ADJ(priv->calib.hs_term_range_adj);
	reg &= ~RPD_CTRL(~0);
	reg |= RPD_CTRL(priv->calib.rpd_ctrl);
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));

	return 0;
}

static int tegra194_utmi_phy_power_off(struct phy *phy)
{
	tegra_phy_xusb_utmi_pad_power_down(phy);

	return 0;
}

static int tegra194_utmi_phy_enable_sleepwalk(struct phy *phy,
					      enum usb_device_speed speed)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	struct tegra194_xusb_padctl *priv = to_tegra194_xusb_padctl(padctl);
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy enable sleepwalk UTMI %d speed %d\n", index, speed);

	mutex_lock(&padctl->lock);

	/* ensure sleepwalk logic is disabled */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~MASTER_ENABLE;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* ensure sleepwalk logics are in low power mode */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg |= MASTER_CFG_SEL;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* set debounce time */
	reg = ao_readl(priv, XUSB_AO_USB_DEBOUNCE_DEL);
	reg &= ~UTMIP_LINE_DEB_CNT(~0);
	reg |= UTMIP_LINE_DEB_CNT(1);
	ao_writel(priv, reg, XUSB_AO_USB_DEBOUNCE_DEL);

	/* ensure fake events of sleepwalk logic are desiabled */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~(FAKE_USBOP_VAL | FAKE_USBON_VAL |
		FAKE_USBOP_EN | FAKE_USBON_EN);
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* ensure wake events of sleepwalk logic are not latched */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~LINE_WAKEUP_EN;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* disable wake event triggers of sleepwalk logic */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_NONE;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* power down the line state detectors of the pad */
	reg = ao_readl(priv, XUSB_AO_UTMIP_PAD_CFG(index));
	reg |= (USBOP_VAL_PD | USBON_VAL_PD);
	ao_writel(priv, reg, XUSB_AO_UTMIP_PAD_CFG(index));

	/* save state per speed */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SAVED_STATE(index));
	reg &= ~SPEED(~0);
	if (speed == USB_SPEED_HIGH)
		reg |= UTMI_HS;
	else if (speed == USB_SPEED_FULL)
		reg |= UTMI_FS;
	else if (speed == USB_SPEED_LOW)
		reg |= UTMI_LS;
	else
		reg |= UTMI_RST;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SAVED_STATE(index));

	/* enable the trigger of the sleepwalk logic */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg |= LINEVAL_WALK_EN;
	reg &= ~WAKE_WALK_EN;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* reset the walk pointer and clear the alarm of the sleepwalk logic,
	 * as well as capture the configuration of the USB2.0 pad
	 */
	reg = ao_readl(priv, XUSB_AO_UTMIP_TRIGGERS(index));
	reg |= (CLR_WALK_PTR | CLR_WAKE_ALARM | CAP_CFG);
	ao_writel(priv, reg, XUSB_AO_UTMIP_TRIGGERS(index));

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
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK(index));

	/* power up the line state detectors of the pad */
	reg = ao_readl(priv, XUSB_AO_UTMIP_PAD_CFG(index));
	reg &= ~(USBOP_VAL_PD | USBON_VAL_PD);
	ao_writel(priv, reg, XUSB_AO_UTMIP_PAD_CFG(index));

	usleep_range(150, 200);

	/* switch the electric control of the USB2.0 pad to XUSB_AO */
	reg = ao_readl(priv, XUSB_AO_UTMIP_PAD_CFG(index));
	reg |= (FSLS_USE_XUSB_AO | TRK_CTRL_USE_XUSB_AO |
		RPD_CTRL_USE_XUSB_AO | RPU_USE_XUSB_AO | VREG_USE_XUSB_AO);
	ao_writel(priv, reg, XUSB_AO_UTMIP_PAD_CFG(index));

	/* set the wake signaling trigger events */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_ANY;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* enable the wake detection */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg |= (MASTER_ENABLE | LINE_WAKEUP_EN);
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_utmi_phy_disable_sleepwalk(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	struct tegra194_xusb_padctl *priv = to_tegra194_xusb_padctl(padctl);
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy disable sleepwalk UTMI %d\n", index);

	mutex_lock(&padctl->lock);

	/* disable the wake detection */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~(MASTER_ENABLE | LINE_WAKEUP_EN);
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* switch the electric control of the USB2.0 pad to XUSB vcore logic */
	reg = ao_readl(priv, XUSB_AO_UTMIP_PAD_CFG(index));
	reg &= ~(FSLS_USE_XUSB_AO | TRK_CTRL_USE_XUSB_AO |
		RPD_CTRL_USE_XUSB_AO | RPU_USE_XUSB_AO | VREG_USE_XUSB_AO);
	ao_writel(priv, reg, XUSB_AO_UTMIP_PAD_CFG(index));

	/* disable wake event triggers of sleepwalk logic */
	reg = ao_readl(priv, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));
	reg &= ~WAKE_VAL(~0);
	reg |= WAKE_VAL_NONE;
	ao_writel(priv, reg, XUSB_AO_UTMIP_SLEEPWALK_CFG(index));

	/* power down the line state detectors of the port */
	reg = ao_readl(priv, XUSB_AO_UTMIP_PAD_CFG(index));
	reg |= (USBOP_VAL_PD | USBON_VAL_PD);
	ao_writel(priv, reg, XUSB_AO_UTMIP_PAD_CFG(index));

	/* clear alarm of the sleepwalk logic */
	reg = ao_readl(priv, XUSB_AO_UTMIP_TRIGGERS(index));
	reg |= CLR_WAKE_ALARM;
	ao_writel(priv, reg, XUSB_AO_UTMIP_TRIGGERS(index));

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_utmi_phy_enable_wake(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy enable wake UTMI %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg |= USB2_PORT_WAKEUP_EVENT(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(10, 20);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg |= USB2_PORT_WAKE_INTERRUPT_ENABLE(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_utmi_phy_disable_wake(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy disable wake UTMI %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg &= ~USB2_PORT_WAKE_INTERRUPT_ENABLE(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(10, 20);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg |= USB2_PORT_WAKEUP_EVENT(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_utmi_phy_init(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct tegra_xusb_usb2_port *port;
	int rc = 0;

	dev_dbg(padctl->dev, "phy init UTMI %d\n", index);

	port = tegra_xusb_find_usb2_port(padctl, index);
	if (!port) {
		dev_err(padctl->dev, "no port found for USB2 lane %u\n", index);
		return -ENODEV;
	}

	mutex_lock(&padctl->lock);

	if (port->supply && port->port_cap == USB_HOST_CAP) {
		rc = regulator_enable(port->supply);
		if (rc) {
			dev_err(padctl->dev, "enable port %d vbus failed %d\n",
				index, rc);
		}
	}

	mutex_unlock(&padctl->lock);

	return rc;
}

static int tegra194_utmi_phy_exit(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct tegra_xusb_usb2_port *port;
	int rc = 0;

	dev_dbg(padctl->dev, "phy exit UTMI %d\n", index);

	port = tegra_xusb_find_usb2_port(padctl, index);
	if (!port) {
		dev_err(padctl->dev, "no port found for USB2 lane %u\n", index);
		return -ENODEV;
	}

	mutex_lock(&padctl->lock);

	if (port->supply && port->port_cap == USB_HOST_CAP) {
		rc = regulator_disable(port->supply);
		if (rc) {
			dev_err(padctl->dev, "disable port %d vbus failed %d\n",
				index, rc);
		}
	}

	mutex_unlock(&padctl->lock);

	return rc;
}

static const struct phy_ops utmi_phy_ops = {
	.init = tegra194_utmi_phy_init,
	.exit = tegra194_utmi_phy_exit,
	.power_on = tegra194_utmi_phy_power_on,
	.power_off = tegra194_utmi_phy_power_off,
	.owner = THIS_MODULE,
};

static inline bool is_utmi_phy(struct phy *phy)
{
	return phy->ops == &utmi_phy_ops;
}

static struct tegra_xusb_pad *
tegra194_usb2_pad_probe(struct tegra_xusb_padctl *padctl,
			const struct tegra_xusb_pad_soc *soc,
			struct device_node *np)
{
	struct tegra194_xusb_padctl *priv = to_tegra194_xusb_padctl(padctl);
	struct tegra_xusb_usb2_pad *usb2;
	struct tegra_xusb_pad *pad;
	int err;

	usb2 = kzalloc(sizeof(*usb2), GFP_KERNEL);
	if (!usb2)
		return ERR_PTR(-ENOMEM);

	pad = &usb2->base;
	pad->ops = &tegra194_usb2_lane_ops;
	pad->soc = soc;

	err = tegra_xusb_pad_init(pad, padctl, np);
	if (err < 0) {
		kfree(usb2);
		goto out;
	}

	if (tegra_platform_is_silicon()) {
		priv->usb2_trk_clk = devm_clk_get(&pad->dev, "trk");
		if (IS_ERR(usb2->clk)) {
			err = PTR_ERR(usb2->clk);
			dev_dbg(&pad->dev,
				"failed to get usb2 trk clock: %d\n", err);
			goto unregister;
		}
	}

	err = tegra_xusb_pad_register(pad, &utmi_phy_ops);
	if (err < 0)
		goto unregister;

	dev_set_drvdata(&pad->dev, pad);

	return pad;

unregister:
	device_unregister(&pad->dev);
out:
	return ERR_PTR(err);
}

static void tegra194_usb2_pad_remove(struct tegra_xusb_pad *pad)
{
	struct tegra_xusb_usb2_pad *usb2 = to_usb2_pad(pad);

	kfree(usb2);
}

static const struct tegra_xusb_pad_ops tegra194_usb2_pad_ops = {
	.probe = tegra194_usb2_pad_probe,
	.remove = tegra194_usb2_pad_remove,
};

static const char * const tegra194_usb2_functions[] = {
	"xusb",
};

static const struct tegra_xusb_lane_soc tegra194_usb2_lanes[] = {
	TEGRA194_LANE("usb2-0", 0,  0, 0, usb2),
	TEGRA194_LANE("usb2-1", 0,  0, 0, usb2),
	TEGRA194_LANE("usb2-2", 0,  0, 0, usb2),
};

static const struct tegra_xusb_pad_soc tegra194_usb2_pad = {
	.name = "usb2",
	.num_lanes = ARRAY_SIZE(tegra194_usb2_lanes),
	.lanes = tegra194_usb2_lanes,
	.ops = &tegra194_usb2_pad_ops,
};

static int tegra194_usb2_port_enable(struct tegra_xusb_port *port)
{
	return 0;
}

static void tegra194_usb2_port_disable(struct tegra_xusb_port *port)
{
}

static struct tegra_xusb_lane *
tegra194_usb2_port_map(struct tegra_xusb_port *port)
{
	return tegra_xusb_find_lane(port->padctl, "usb2", port->index);
}

static const struct tegra_xusb_port_ops tegra194_usb2_port_ops = {
	.enable = tegra194_usb2_port_enable,
	.disable = tegra194_usb2_port_disable,
	.map = tegra194_usb2_port_map,
};

/* SuperSpeed PHY support */
static struct tegra_xusb_lane *
tegra194_usb3_lane_probe(struct tegra_xusb_pad *pad, struct device_node *np,
			 unsigned int index)
{
	struct tegra_xusb_usb3_lane *usb3;
	int err;

	usb3 = kzalloc(sizeof(*usb3), GFP_KERNEL);
	if (!usb3)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&usb3->base.list);
	usb3->base.soc = &pad->soc->lanes[index];
	usb3->base.index = index;
	usb3->base.pad = pad;
	usb3->base.np = np;

	err = tegra_xusb_lane_parse_dt(&usb3->base, np);
	if (err < 0) {
		kfree(usb3);
		return ERR_PTR(err);
	}

	return &usb3->base;
}

static void tegra194_usb3_lane_remove(struct tegra_xusb_lane *lane)
{
	struct tegra_xusb_usb3_lane *usb3 = to_usb3_lane(lane);

	kfree(usb3);
}

static const struct tegra_xusb_lane_ops tegra194_usb3_lane_ops = {
	.probe = tegra194_usb3_lane_probe,
	.remove = tegra194_usb3_lane_remove,
};
static int tegra194_usb3_port_enable(struct tegra_xusb_port *port)
{
	return 0;
}

static void tegra194_usb3_port_disable(struct tegra_xusb_port *port)
{
}

static struct tegra_xusb_lane *
tegra194_usb3_port_map(struct tegra_xusb_port *port)
{
	return tegra_xusb_find_lane(port->padctl, "usb3", port->index);
}

static const struct tegra_xusb_port_ops tegra194_usb3_port_ops = {
	.enable = tegra194_usb3_port_enable,
	.disable = tegra194_usb3_port_disable,
	.map = tegra194_usb3_port_map,
};

static int tegra194_usb3_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	struct tegra_xusb_usb3_port *port;
	u32 reg;

	dev_dbg(dev, "phy power on USB3 %d\n", index);

	port = tegra_xusb_find_usb3_port(padctl, index);
	if (!port) {
		dev_err(dev, "no port found for USB3 lane %u\n", index);
		return -ENODEV;
	}

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_CAP);
	reg &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(index));
	if (port->port_cap == USB_PORT_DISABLED)
		reg |= (PORT_CAP_DISABLED << PORTX_CAP_SHIFT(index));
	else if (port->port_cap == USB_DEVICE_CAP)
		reg |= (PORT_CAP_DEVICE << PORTX_CAP_SHIFT(index));
	else if (port->port_cap == USB_HOST_CAP)
		reg |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(index));
	else if (port->port_cap == USB_OTG_CAP)
		reg |= (PORT_CAP_OTG << PORTX_CAP_SHIFT(index));
	padctl_writel(padctl, reg, XUSB_PADCTL_SS_PORT_CAP);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_VCORE_DOWN(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy power off USB3 %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(250, 350);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_VCORE_DOWN(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_usb3_phy_enable_sleepwalk(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy enable sleepwalk USB3 %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN_EARLY(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg |= SSPX_ELPG_CLAMP_EN(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(250, 350);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_usb3_phy_disable_sleepwalk(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy disable sleepwalk USB3 %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN_EARLY(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	usleep_range(100, 200);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM_1);
	reg &= ~SSPX_ELPG_CLAMP_EN(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM_1);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_usb3_phy_enable_wake(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy enable wake USB3 %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg |= SS_PORT_WAKEUP_EVENT(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(10, 20);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg |= SS_PORT_WAKE_INTERRUPT_ENABLE(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_usb3_phy_disable_wake(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	u32 reg;

	dev_dbg(dev, "phy disable wake USB3 %d\n", index);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg &= ~SS_PORT_WAKE_INTERRUPT_ENABLE(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	usleep_range(10, 20);

	reg = padctl_readl(padctl, XUSB_PADCTL_ELPG_PROGRAM);
	reg &= ~ALL_WAKE_EVENTS;
	reg |= SS_PORT_WAKEUP_EVENT(index);
	padctl_writel(padctl, reg, XUSB_PADCTL_ELPG_PROGRAM);

	mutex_unlock(&padctl->lock);

	return 0;
}

static int tegra194_usb3_phy_init(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	struct device *dev = padctl->dev;
	unsigned int index = lane->index;
	struct tegra_xusb_usb3_port *port;
	struct tegra_xusb_usb2_port *companion_usb2_port;
	int rc = 0;

	dev_dbg(dev, "phy init USB3 %d\n", index);

	port = tegra_xusb_find_usb3_port(padctl, index);
	if (!port) {
		dev_err(dev, "no port found for USB3 lane %u\n", index);
		return -ENODEV;
	}

	companion_usb2_port = tegra_xusb_find_usb2_port(padctl, port->port);
	if (!companion_usb2_port) {
		dev_err(dev, "no companion port found for USB3 lane %u\n",
			index);
		return -ENODEV;
	}
	dev_dbg(dev, "USB3 port %d companion USB2 port %d mode %d\n", index,
		port->port, companion_usb2_port->port_cap);

	mutex_lock(&padctl->lock);

	port->port_cap = companion_usb2_port->port_cap;

	mutex_unlock(&padctl->lock);

	return rc;
}

static int tegra194_usb3_phy_exit(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	struct tegra_xusb_usb3_port *port;
	int rc = 0;

	dev_dbg(dev, "phy exit USB3 %d\n", index);

	port = tegra_xusb_find_usb3_port(padctl, index);
	if (!port) {
		dev_err(dev, "no port found for USB3 lane %u\n", index);
		return -ENODEV;
	}

	mutex_lock(&padctl->lock);

	mutex_unlock(&padctl->lock);

	return rc;
}

static const struct phy_ops usb3_phy_ops = {
	.init = tegra194_usb3_phy_init,
	.exit = tegra194_usb3_phy_exit,
	.power_on = tegra194_usb3_phy_power_on,
	.power_off = tegra194_usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static inline bool is_usb3_phy(struct phy *phy)
{
	return phy->ops == &usb3_phy_ops;
}

static struct tegra_xusb_pad *
tegra194_usb3_pad_probe(struct tegra_xusb_padctl *padctl,
			const struct tegra_xusb_pad_soc *soc,
			struct device_node *np)
{
	struct tegra_xusb_usb3_pad *usb3;
	struct tegra_xusb_pad *pad;
	int err;

	usb3 = kzalloc(sizeof(*usb3), GFP_KERNEL);
	if (!usb3)
		return ERR_PTR(-ENOMEM);

	pad = &usb3->base;
	pad->ops = &tegra194_usb3_lane_ops;
	pad->soc = soc;

	err = tegra_xusb_pad_init(pad, padctl, np);
	if (err < 0) {
		kfree(usb3);
		goto out;
	}

	err = tegra_xusb_pad_register(pad, &usb3_phy_ops);
	if (err < 0)
		goto unregister;

	dev_set_drvdata(&pad->dev, pad);

	return pad;

unregister:
	device_unregister(&pad->dev);
out:
	return ERR_PTR(err);
}

static void tegra194_usb3_pad_remove(struct tegra_xusb_pad *pad)
{
	struct tegra_xusb_usb2_pad *usb2 = to_usb2_pad(pad);

	kfree(usb2);
}

static const struct tegra_xusb_pad_ops tegra194_usb3_pad_ops = {
	.probe = tegra194_usb3_pad_probe,
	.remove = tegra194_usb3_pad_remove,
};

static const char * const tegra194_usb3_functions[] = {
	"xusb",
};

static const struct tegra_xusb_lane_soc tegra194_usb3_lanes[] = {
	TEGRA194_LANE("usb3-0", 0,  0, 0, usb3),
	TEGRA194_LANE("usb3-1", 0,  0, 0, usb3),
	TEGRA194_LANE("usb3-2", 0,  0, 0, usb3),
};

static const struct tegra_xusb_pad_soc tegra194_usb3_pad = {
	.name = "usb3",
	.num_lanes = ARRAY_SIZE(tegra194_usb3_lanes),
	.lanes = tegra194_usb3_lanes,
	.ops = &tegra194_usb3_pad_ops,
};

static const struct tegra_xusb_pad_soc * const tegra194_pads[] = {
	&tegra194_usb2_pad,
	&tegra194_usb3_pad,
};

static int
tegra194_xusb_read_fuse_calibration(struct tegra194_xusb_padctl *padctl)
{
	unsigned int i;
	int rc;
	u32 reg;

	rc = tegra_fuse_readl(TEGRA_FUSE_SKU_CALIB_0, &reg);
	if (rc) {
		dev_err(padctl->base.dev, "read calib fuse failed %d\n", rc);
		return rc;
	}

	dev_dbg(padctl->base.dev, "FUSE_USB_CALIB_0 0x%x\n", reg);

	for (i = 0; i < TEGRA194_UTMI_PHYS; i++) {
		padctl->calib.hs_curr_level[i] =
			(reg >> HS_CURR_LEVEL_PADX_SHIFT(i)) &
			HS_CURR_LEVEL_PAD_MASK;
	}
	padctl->calib.hs_squelch = (reg >> HS_SQUELCH_SHIFT) & HS_SQUELCH_MASK;
	padctl->calib.hs_term_range_adj = (reg >> HS_TERM_RANGE_ADJ_SHIFT) &
					HS_TERM_RANGE_ADJ_MASK;

	rc = tegra_fuse_readl(TEGRA_FUSE_USB_CALIB_EXT_0, &reg);
	if (rc) {
		dev_err(padctl->base.dev, "read calib fuse failed %d\n", rc);
		return rc;
	}

	dev_dbg(padctl->base.dev, "FUSE_USB_CALIB_EXT_0 0x%x\n", reg);

	padctl->calib.rpd_ctrl = (reg >> RPD_CTRL_SHIFT) & RPD_CTRL_MASK;

	return 0;
}

static struct tegra_xusb_padctl *
tegra194_xusb_padctl_probe(struct device *dev,
			   const struct tegra_xusb_padctl_soc *soc)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra194_xusb_padctl *priv;
	struct resource *res;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->base.dev = dev;
	priv->base.soc = soc;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ao");
	priv->ao_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->ao_regs))
		return priv->ao_regs;

	if (tegra_platform_is_silicon()) {
		err = tegra194_xusb_read_fuse_calibration(priv);
		if (err < 0)
			return ERR_PTR(err);
	}

	priv->prod_list = devm_tegra_prod_get(dev);
	if (IS_ERR(priv->prod_list)) {
		dev_warn(dev, "Prod-settings is not available\n");
		priv->prod_list = NULL;
	}

	return &priv->base;
}

static void tegra194_xusb_padctl_remove(struct tegra_xusb_padctl *padctl)
{
}

static int tegra194_xusb_padctl_vbus_override(struct tegra_xusb_padctl *padctl,
					      bool set)
{
	u32 reg;

	reg = padctl_readl(padctl, USB2_VBUS_ID);
	if (set) {
		reg |= VBUS_OVERRIDE;
		reg &= ~ID_OVERRIDE(~0);
		reg |= ID_OVERRIDE_FLOATING;
	} else
		reg &= ~VBUS_OVERRIDE;
	padctl_writel(padctl, reg, USB2_VBUS_ID);

#if 0 /* TODO OTG support */
	schedule_work(&padctl->otg_vbus_work);
#endif
	return 0;
}

static int tegra194_xusb_padctl_phy_sleepwalk(struct tegra_xusb_padctl *padctl,
					      struct phy *phy, bool enable,
					      enum usb_device_speed speed)
{
	if (!phy)
		return 0;

	if (is_usb3_phy(phy)) {
		if (enable)
			return tegra194_usb3_phy_enable_sleepwalk(phy);
		else
			return tegra194_usb3_phy_disable_sleepwalk(phy);
	} else if (is_utmi_phy(phy)) {
		if (enable)
			return tegra194_utmi_phy_enable_sleepwalk(phy, speed);
		else
			return tegra194_utmi_phy_disable_sleepwalk(phy);
	} else
		return -EINVAL;

	return 0;
}

static int tegra194_xusb_padctl_phy_wake(struct tegra_xusb_padctl *padctl,
					 struct phy *phy, bool enable)
{
	if (!phy)
		return 0;

	if (is_usb3_phy(phy)) {
		if (enable)
			return tegra194_usb3_phy_enable_wake(phy);
		else
			return tegra194_usb3_phy_disable_wake(phy);
	} else if (is_utmi_phy(phy)) {
		if (enable)
			return tegra194_utmi_phy_enable_wake(phy);
		else
			return tegra194_utmi_phy_disable_wake(phy);
	} else
		return -EINVAL;

	return 0;
}

static const struct tegra_xusb_padctl_ops tegra194_xusb_padctl_ops = {
	.probe = tegra194_xusb_padctl_probe,
	.remove = tegra194_xusb_padctl_remove,
	.vbus_override = tegra194_xusb_padctl_vbus_override,
	.phy_sleepwalk = tegra194_xusb_padctl_phy_sleepwalk,
	.phy_wake = tegra194_xusb_padctl_phy_wake,
};

const struct tegra_xusb_padctl_soc tegra194_xusb_padctl_soc = {
	.num_pads = ARRAY_SIZE(tegra194_pads),
	.pads = tegra194_pads,
	.ports = {
		.usb2 = {
			.ops = &tegra194_usb2_port_ops,
			.count = TEGRA194_UTMI_PHYS,
		},
		.usb3 = {
			.ops = &tegra194_usb3_port_ops,
			.count = TEGRA194_USB3_PHYS,
		},
	},
	.ops = &tegra194_xusb_padctl_ops,
};
EXPORT_SYMBOL_GPL(tegra194_xusb_padctl_soc);

MODULE_AUTHOR("JC Kuo <jckuo@nvidia.com>");
MODULE_DESCRIPTION("Tegra194 (Xavier) XUSB PADCTL driver");
MODULE_LICENSE("GPL v2");
