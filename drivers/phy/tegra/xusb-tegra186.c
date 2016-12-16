/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#define TEGRA186_USB3_PHYS	(3)
#define TEGRA186_UTMI_PHYS	(3)
#define TEGRA186_HSIC_PHYS	(1)

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
#define   HSIC_PORT_SHIFT(x)			((x) + 20)
#define   HSIC_PORT_MASK			(0x1)
#define    PORT_HSIC				(0)

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

#define TEGRA186_LANE(_name, _offset, _shift, _mask, _type)		\
	{								\
		.name = _name,						\
		.offset = _offset,					\
		.shift = _shift,					\
		.mask = _mask,						\
		.num_funcs = ARRAY_SIZE(tegra186_##_type##_functions),	\
		.funcs = tegra186_##_type##_functions,			\
	}

struct tegra_xusb_fuse_calibration {
	u32 hs_curr_level[TEGRA186_UTMI_PHYS];
	u32 hs_squelch;
	u32 hs_term_range_adj;
	u32 rpd_ctrl;
};

struct tegra186_xusb_padctl {
	struct tegra_xusb_padctl base;

	/* prod settings */
	struct tegra_prod *prod_list;
	struct tegra_xusb_fuse_calibration calib;

	/* utmi bias and tracking */
	struct clk *usb2_trk_clk;
	unsigned int bias_pad_enable;
};

static inline struct tegra186_xusb_padctl *
to_tegra186_xusb_padctl(struct tegra_xusb_padctl *padctl)
{
	return container_of(padctl, struct tegra186_xusb_padctl, base);
}

/* USB 2.0 UTMI PHY support */
static struct tegra_xusb_lane *
tegra186_usb2_lane_probe(struct tegra_xusb_pad *pad, struct device_node *np,
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

static void tegra186_usb2_lane_remove(struct tegra_xusb_lane *lane)
{
	struct tegra_xusb_usb2_lane *usb2 = to_usb2_lane(lane);

	kfree(usb2);
}

static const struct tegra_xusb_lane_ops tegra186_usb2_lane_ops = {
	.probe = tegra186_usb2_lane_probe,
	.remove = tegra186_usb2_lane_remove,
};

static void tegra186_utmi_bias_pad_power_on(struct tegra_xusb_padctl *padctl)
{
	struct tegra186_xusb_padctl *priv = to_tegra186_xusb_padctl(padctl);
	u32 reg;

	mutex_lock(&padctl->lock);
	if (priv->bias_pad_enable++ > 0) {
		mutex_unlock(&padctl->lock);
		return;
	}

	if (clk_prepare_enable(priv->usb2_trk_clk))
		dev_warn(padctl->dev, "failed to enable USB2 trk clock\n");

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	reg &= ~USB2_TRK_START_TIMER(~0);
	reg |= USB2_TRK_START_TIMER(0x1e);
	reg &= ~USB2_TRK_DONE_RESET_TIMER(~0);
	reg |= USB2_TRK_DONE_RESET_TIMER(0xa);
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);
	reg &= ~BIAS_PAD_PD;
	reg &= ~HS_SQUELCH_LEVEL(~0);
	reg |= HS_SQUELCH_LEVEL(priv->calib.hs_squelch);
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL0);

	udelay(1);

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	reg &= ~USB2_PD_TRK;
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	mutex_unlock(&padctl->lock);
}

static void tegra186_utmi_bias_pad_power_off(struct tegra_xusb_padctl *padctl)
{
	struct tegra186_xusb_padctl *priv = to_tegra186_xusb_padctl(padctl);
	u32 reg;

	mutex_lock(&padctl->lock);

	if (WARN_ON(priv->bias_pad_enable == 0)) {
		mutex_unlock(&padctl->lock);
		return;
	}

	if (--priv->bias_pad_enable > 0) {
		mutex_unlock(&padctl->lock);
		return;
	}

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);
	reg |= USB2_PD_TRK;
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_BIAS_PAD_CTL1);

	clk_disable_unprepare(priv->usb2_trk_clk);

	mutex_unlock(&padctl->lock);
}

void tegra_phy_xusb_utmi_pad_power_on(struct phy *phy)
{
	struct tegra_xusb_lane *lane;
	struct tegra_xusb_usb2_lane *usb2;
	struct tegra_xusb_padctl *padctl;
	unsigned int index;
	struct device *dev;
	struct tegra_xusb_usb2_port *port;
	u32 reg;

	if (!phy)
		return;

	lane = phy_get_drvdata(phy);
	usb2 = to_usb2_lane(lane);
	padctl = lane->pad->padctl;
	index = lane->index;
	dev = padctl->dev;

	dev_dbg(dev, "power on UTMI pads %d\n", index);

	port = tegra_xusb_find_usb2_port(padctl, index);
	if (!port) {
		dev_err(dev, "no port found for USB2 lane %u\n", index);
		return;
	}

	if (usb2->powered_on)
		return;

	tegra186_utmi_bias_pad_power_on(padctl);

	udelay(2);

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(index));
	reg &= ~USB2_OTG_PD;
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(index));

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));
	reg &= ~USB2_OTG_PD_DR;
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));

	usb2->powered_on = true;
}
#if 0
/* TODO: pad power saving */
EXPORT_SYMBOL_GPL(tegra_phy_xusb_utmi_pad_power_on);
#endif

void tegra_phy_xusb_utmi_pad_power_down(struct phy *phy)
{
	struct tegra_xusb_lane *lane;
	struct tegra_xusb_usb2_lane *usb2;
	struct tegra_xusb_padctl *padctl;
	unsigned int index;
	struct device *dev;
	u32 reg;

	if (!phy)
		return;

	lane = phy_get_drvdata(phy);
	usb2 = to_usb2_lane(lane);
	padctl = lane->pad->padctl;
	index = lane->index;
	dev = padctl->dev;

	dev_dbg(dev, "power down UTMI pad %d\n",  index);

	if (!usb2->powered_on)
		return;

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL0(index));
	reg |= USB2_OTG_PD;
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL0(index));

	reg = padctl_readl(padctl, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));
	reg |= USB2_OTG_PD_DR;
	padctl_writel(padctl, reg, XUSB_PADCTL_USB2_OTG_PADX_CTL1(index));

	udelay(2);

	tegra186_utmi_bias_pad_power_off(padctl);
	usb2->powered_on = false;
}
#if 0
/* TODO: pad power saving */
EXPORT_SYMBOL_GPL(tegra_phy_xusb_utmi_pad_power_down);
#endif

static int tegra186_utmi_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_usb2_lane *usb2 = to_usb2_lane(lane);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	struct tegra186_xusb_padctl *priv = to_tegra186_xusb_padctl(padctl);
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	struct tegra_xusb_usb2_port *port;
	u32 reg;

	dev_dbg(dev, "phy power on UTMI %d\n",  index);

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

	/* TODO: pad power saving */
	tegra_phy_xusb_utmi_pad_power_on(phy);
	return 0;
}

static int tegra186_utmi_phy_power_off(struct phy *phy)
{
	/* TODO: pad power saving */
	tegra_phy_xusb_utmi_pad_power_down(phy);

	return 0;
}

static int tegra186_utmi_phy_init(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct tegra_xusb_usb2_port *port;
	int rc;

	dev_dbg(padctl->dev, "phy init UTMI %d\n",  index);

	port = tegra_xusb_find_usb2_port(padctl, index);
	if (!port) {
		dev_err(padctl->dev, "no port found for USB2 lane %u\n", index);
		return -ENODEV;
	}

	if (port->supply && port->port_cap == USB_HOST_CAP) {
		rc = regulator_enable(port->supply);
		if (rc) {
			dev_err(padctl->dev, "enable port %d vbus failed %d\n",
				index, rc);
			return rc;
		}
	}

	return 0;
}

static int tegra186_utmi_phy_exit(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct tegra_xusb_usb2_port *port;
	int rc;

	dev_dbg(padctl->dev, "phy exit UTMI %d\n",  index);

	port = tegra_xusb_find_usb2_port(padctl, index);
	if (!port) {
		dev_err(padctl->dev, "no port found for USB2 lane %u\n", index);
		return -ENODEV;
	}

	if (port->supply && port->port_cap == USB_HOST_CAP) {
		rc = regulator_disable(port->supply);
		if (rc) {
			dev_err(padctl->dev, "disable port %d vbus failed %d\n",
				index, rc);
			return rc;
		}
	}

	return 0;
}

static const struct phy_ops utmi_phy_ops = {
	.init = tegra186_utmi_phy_init,
	.exit = tegra186_utmi_phy_exit,
	.power_on = tegra186_utmi_phy_power_on,
	.power_off = tegra186_utmi_phy_power_off,
	.owner = THIS_MODULE,
};

static struct tegra_xusb_pad *
tegra186_usb2_pad_probe(struct tegra_xusb_padctl *padctl,
			const struct tegra_xusb_pad_soc *soc,
			struct device_node *np)
{
	struct tegra186_xusb_padctl *priv = to_tegra186_xusb_padctl(padctl);
	struct tegra_xusb_usb2_pad *usb2;
	struct tegra_xusb_pad *pad;
	int err;

	usb2 = kzalloc(sizeof(*usb2), GFP_KERNEL);
	if (!usb2)
		return ERR_PTR(-ENOMEM);

	pad = &usb2->base;
	pad->ops = &tegra186_usb2_lane_ops;
	pad->soc = soc;

	err = tegra_xusb_pad_init(pad, padctl, np);
	if (err < 0) {
		kfree(usb2);
		goto out;
	}

	priv->usb2_trk_clk = devm_clk_get(&pad->dev, "trk");
	if (IS_ERR(usb2->clk)) {
		err = PTR_ERR(usb2->clk);
		dev_dbg(&pad->dev, "failed to get usb2 trk clock: %d\n", err);
		goto unregister;
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

static void tegra186_usb2_pad_remove(struct tegra_xusb_pad *pad)
{
	struct tegra_xusb_usb2_pad *usb2 = to_usb2_pad(pad);

	kfree(usb2);
}

static const struct tegra_xusb_pad_ops tegra186_usb2_pad_ops = {
	.probe = tegra186_usb2_pad_probe,
	.remove = tegra186_usb2_pad_remove,
};

static const char * const tegra186_usb2_functions[] = {
	"xusb",
};

static const struct tegra_xusb_lane_soc tegra186_usb2_lanes[] = {
	TEGRA186_LANE("usb2-0", 0,  0, 0, usb2),
	TEGRA186_LANE("usb2-1", 0,  0, 0, usb2),
	TEGRA186_LANE("usb2-2", 0,  0, 0, usb2),
};

static const struct tegra_xusb_pad_soc tegra186_usb2_pad = {
	.name = "usb2",
	.num_lanes = ARRAY_SIZE(tegra186_usb2_lanes),
	.lanes = tegra186_usb2_lanes,
	.ops = &tegra186_usb2_pad_ops,
};

static int tegra186_usb2_port_enable(struct tegra_xusb_port *port)
{
	return 0;
}

static void tegra186_usb2_port_disable(struct tegra_xusb_port *port)
{
}

static struct tegra_xusb_lane *
tegra186_usb2_port_map(struct tegra_xusb_port *port)
{
	return tegra_xusb_find_lane(port->padctl, "usb2", port->index);
}

static const struct tegra_xusb_port_ops tegra186_usb2_port_ops = {
	.enable = tegra186_usb2_port_enable,
	.disable = tegra186_usb2_port_disable,
	.map = tegra186_usb2_port_map,
};

/* SuperSpeed PHY support */
static struct tegra_xusb_lane *
tegra186_usb3_lane_probe(struct tegra_xusb_pad *pad, struct device_node *np,
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

static void tegra186_usb3_lane_remove(struct tegra_xusb_lane *lane)
{
	struct tegra_xusb_usb3_lane *usb3 = to_usb3_lane(lane);

	kfree(usb3);
}

static const struct tegra_xusb_lane_ops tegra186_usb3_lane_ops = {
	.probe = tegra186_usb3_lane_probe,
	.remove = tegra186_usb3_lane_remove,
};
static int tegra186_usb3_port_enable(struct tegra_xusb_port *port)
{
	return 0;
}

static void tegra186_usb3_port_disable(struct tegra_xusb_port *port)
{
}

static struct tegra_xusb_lane *
tegra186_usb3_port_map(struct tegra_xusb_port *port)
{
	return tegra_xusb_find_lane(port->padctl, "usb3", port->index);
}

static const struct tegra_xusb_port_ops tegra186_usb3_port_ops = {
	.enable = tegra186_usb3_port_enable,
	.disable = tegra186_usb3_port_disable,
	.map = tegra186_usb3_port_map,
};

static int tegra186_usb3_phy_power_on(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	struct tegra_xusb_usb3_port *port;
	struct tegra_xusb_usb2_port *companion_usb2_port;
	u32 reg;

	dev_dbg(dev, "phy power on USB3 %d\n",  index);

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
	dev_dbg(padctl->dev, "USB3 port %d with usb2 port %d mode %d\n", index,
		port->port, companion_usb2_port->port_cap);

	mutex_lock(&padctl->lock);

	reg = padctl_readl(padctl, XUSB_PADCTL_SS_PORT_CAP);
	reg &= ~(PORT_CAP_MASK << PORTX_CAP_SHIFT(index));
	if (companion_usb2_port->port_cap == USB_PORT_DISABLED)
		reg |= (PORT_CAP_DISABLED << PORTX_CAP_SHIFT(index));
	else if (companion_usb2_port->port_cap == USB_DEVICE_CAP)
		reg |= (PORT_CAP_DEVICE << PORTX_CAP_SHIFT(index));
	else if (companion_usb2_port->port_cap == USB_HOST_CAP)
		reg |= (PORT_CAP_HOST << PORTX_CAP_SHIFT(index));
	else if (companion_usb2_port->port_cap == USB_OTG_CAP)
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

static int tegra186_usb3_phy_power_off(struct phy *phy)
{
	struct tegra_xusb_lane *lane = phy_get_drvdata(phy);
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	unsigned int index = lane->index;
	struct device *dev = padctl->dev;
	struct tegra_xusb_usb3_port *port;
	u32 reg;

	dev_dbg(dev, "phy power off USB3 %d\n",  index);

	port = tegra_xusb_find_usb3_port(padctl, index);
	if (!port) {
		dev_err(dev, "no port found for USB3 lane %u\n", index);
		return -ENODEV;
	}

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

static int tegra186_usb3_phy_init(struct phy *phy)
{
	return 0;
}

static int tegra186_usb3_phy_exit(struct phy *phy)
{
	return 0;
}

static const struct phy_ops usb3_phy_ops = {
	.init = tegra186_usb3_phy_init,
	.exit = tegra186_usb3_phy_exit,
	.power_on = tegra186_usb3_phy_power_on,
	.power_off = tegra186_usb3_phy_power_off,
	.owner = THIS_MODULE,
};

static struct tegra_xusb_pad *
tegra186_usb3_pad_probe(struct tegra_xusb_padctl *padctl,
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
	pad->ops = &tegra186_usb3_lane_ops;
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

static void tegra186_usb3_pad_remove(struct tegra_xusb_pad *pad)
{
	struct tegra_xusb_usb2_pad *usb2 = to_usb2_pad(pad);

	kfree(usb2);
}

static const struct tegra_xusb_pad_ops tegra186_usb3_pad_ops = {
	.probe = tegra186_usb3_pad_probe,
	.remove = tegra186_usb3_pad_remove,
};

static const char * const tegra186_usb3_functions[] = {
	"xusb",
};

static const struct tegra_xusb_lane_soc tegra186_usb3_lanes[] = {
	TEGRA186_LANE("usb3-0", 0,  0, 0, usb3),
	TEGRA186_LANE("usb3-1", 0,  0, 0, usb3),
	TEGRA186_LANE("usb3-2", 0,  0, 0, usb3),
};

static const struct tegra_xusb_pad_soc tegra186_usb3_pad = {
	.name = "usb3",
	.num_lanes = ARRAY_SIZE(tegra186_usb3_lanes),
	.lanes = tegra186_usb3_lanes,
	.ops = &tegra186_usb3_pad_ops,
};

static const struct tegra_xusb_pad_soc * const tegra186_pads[] = {
	&tegra186_usb2_pad,
	&tegra186_usb3_pad,
#if 0 /* TODO implement */
	&tegra186_hsic_pad,
#endif
};

static int
tegra186_xusb_read_fuse_calibration(struct tegra186_xusb_padctl *padctl)
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

	for (i = 0; i < TEGRA186_UTMI_PHYS; i++) {
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
tegra186_xusb_padctl_probe(struct device *dev,
			   const struct tegra_xusb_padctl_soc *soc)
{
	struct tegra186_xusb_padctl *priv;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->base.dev = dev;
	priv->base.soc = soc;

	err = tegra186_xusb_read_fuse_calibration(priv);
	if (err < 0)
		return ERR_PTR(err);

	priv->prod_list = devm_tegra_prod_get(dev);
	if (IS_ERR(priv->prod_list)) {
		dev_warn(dev, "Prod-settings is not available\n");
		priv->prod_list = NULL;
	}

	return &priv->base;
}

static void tegra186_xusb_padctl_remove(struct tegra_xusb_padctl *padctl)
{
}

static int tegra186_xusb_padctl_vbus_override(struct tegra_xusb_padctl *padctl,
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

static const struct tegra_xusb_padctl_ops tegra186_xusb_padctl_ops = {
	.probe = tegra186_xusb_padctl_probe,
	.remove = tegra186_xusb_padctl_remove,
	.vbus_override = tegra186_xusb_padctl_vbus_override,
};

const struct tegra_xusb_padctl_soc tegra186_xusb_padctl_soc = {
	.num_pads = ARRAY_SIZE(tegra186_pads),
	.pads = tegra186_pads,
	.ports = {
		.usb2 = {
			.ops = &tegra186_usb2_port_ops,
			.count = TEGRA186_UTMI_PHYS,
		},
		.usb3 = {
			.ops = &tegra186_usb3_port_ops,
			.count = TEGRA186_USB3_PHYS,
		},
#if 0 /* TODO implement */
		.hsic = {
			.ops = &tegra186_hsic_port_ops,
			.count = TEGRA186_HSIC_PHYS,
		},

#endif
	},
	.ops = &tegra186_xusb_padctl_ops,
};
EXPORT_SYMBOL_GPL(tegra186_xusb_padctl_soc);

MODULE_AUTHOR("JC Kuo <jckuo@nvidia.com>");
MODULE_DESCRIPTION("Tegra186 (Parker) XUSB PADCTL driver");
MODULE_LICENSE("GPL v2");
