/*
 * GPIO driver for NVIDIA Tegra186
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Suresh Mangipudi <smangipudi@nvidia.com>
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio/gpio-tegra.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm.h>
#include <linux/irqchip/tegra.h>
#include <linux/version.h>
#include <dt-bindings/gpio/tegra186-gpio.h>

#define GPIO_ENB_CONFIG_REG	0x00
#define GPIO_ENB_BIT		BIT(0)
#define GPIO_INOUT_BIT		BIT(1)
#define GPIO_TRG_TYPE_BIT(x)	(0x3 & (x))
#define GPIO_TRG_TYPE_BIT_OFFSET 0x2
#define GPIO_TRG_LVL_BIT       BIT(4)
#define GPIO_DEB_FUNC_BIT	BIT(5)
#define GPIO_INT_FUNC_BIT	BIT(6)
#define GPIO_TIMESTMP_FUNC_BIT	0x7

#define GPIO_DBC_THRES_REG	0x04
#define GPIO_DBC_THRES_BIT(val)	((val) & 0xFF)
#define GPIO_INPUT_REG		0x08
#define GPIO_OUT_CTRL_REG	0x0c
#define GPIO_OUT_VAL_REG	0x10
#define GPIO_INT_CLEAR_REG	0x14

#define GPIO_REG_DIFF		0x20

#define GPIO_SCR_REG		0x04
#define GPIO_SCR_DIFF		0x08
#define GPIO_SCR_BASE_DIFF	0x40

#define GPIO_CONTROLLERS_DIFF	0x1000
#define GPIO_SCR_SEC_WEN	BIT(28)
#define GPIO_SCR_SEC_REN	BIT(27)
#define GPIO_SCR_SEC_G1R	BIT(1)
#define GPIO_SCR_SEC_G1W	BIT(9)

#define GPIO_INT_LVL_NO_TRIGGER			0x0
#define GPIO_INT_LVL_LEVEL_TRIGGER		0x1
#define GPIO_INT_LVL_SINGLE_EDGE_TRIGGER	0x2
#define GPIO_INT_LVL_BOTH_EDGE_TRIGGER		0x3

#define TRIGGER_LEVEL_LOW		0x0
#define TRIGGER_LEVEL_HIGH		0x1

#define GPIO_INT_STATUS_OFFSET		0x100
#define GPIO_STATUS_G1			0x04

#define GPIO_FULL_ACCESS	(GPIO_SCR_SEC_WEN | GPIO_SCR_SEC_REN |	\
					GPIO_SCR_SEC_G1R | GPIO_SCR_SEC_G1W)

#define MAX_GPIO_CONTROLLERS		7
#define MAX_GPIO_PORTS			8
#define MAX_GPIO_CAR_CTRL		6

#define MAX_PORTS			32
#define MAX_PINS_PER_PORT		8

#define GPIO_PORT(g)			((g) >> 3)
#define GPIO_PIN(g)			((g) & 0x7)

static const int tegra186_gpio_wakes[] = {
	TEGRA_GPIO(A, 6),		/* wake0 */
	TEGRA_GPIO(A, 2),		/* wake1 */
	TEGRA_GPIO(A, 5),		/* wake2 */
	TEGRA_GPIO(D, 3),		/* wake3 */
	TEGRA_GPIO(E, 3),		/* wake4 */
	TEGRA_GPIO(G, 3),		/* wake5 */
	-EINVAL,			/* wake6 */
	TEGRA_GPIO(B, 3),		/* wake7 */
	TEGRA_GPIO(B, 5),		/* wake8 */
	TEGRA_GPIO(C, 0),		/* wake9 */
	-EINVAL,			/* wake10 */
	TEGRA_GPIO(H, 2),		/* wake11 */
	TEGRA_GPIO(J, 5),		/* wake12 */
	TEGRA_GPIO(J, 6),		/* wake13 */
	TEGRA_GPIO(J, 7),		/* wake14 */
	TEGRA_GPIO(K, 0),		/* wake15 */
	TEGRA_GPIO(Q, 1),		/* wake16 */
	TEGRA_GPIO(F, 4),		/* wake17 */
	TEGRA_GPIO(M, 5),		/* wake18 */
	TEGRA_GPIO(P, 0),		/* wake19 */
	TEGRA_GPIO(P, 2),		/* wake20 */
	TEGRA_GPIO(P, 1),		/* wake21 */
	TEGRA_GPIO(O, 3),		/* wake22 */
	TEGRA_GPIO(R, 5),		/* wake23 */
	-EINVAL,			/* wake24 */
	-EINVAL,			/* wake25 */
	-EINVAL,			/* wake26 */
	-EINVAL,			/* wake27 */
	TEGRA_GPIO(F, 2),		/* wake28 */
	-EINVAL,			/* wake29 */
	-EINVAL,			/* wake30 */
	TEGRA_GPIO(C, 6),		/* wake31 */
	-EINVAL,			/* wake32 */
	-EINVAL,			/* wake33 */
	-EINVAL,			/* wake34 */
	-EINVAL,			/* wake35 */
	-EINVAL,			/* wake36 */
	-EINVAL,			/* wake37 */
	-EINVAL,			/* wake38 */
	-EINVAL,			/* wake39 */
	-EINVAL,			/* wake40 */
	-EINVAL,			/* wake41 */
	-EINVAL,			/* wake42 */
	-EINVAL,			/* wake43 */
	-EINVAL,			/* wake44 */
	-EINVAL,			/* wake45 */
	-EINVAL,			/* wake46 */
	-EINVAL,			/* wake47 */
	-EINVAL,			/* wake48 */
	-EINVAL,			/* wake49 */
	-EINVAL,			/* wake50 */
	-EINVAL,			/* wake51 */
	TEGRA_GPIO(X, 3),		/* wake52 */
	TEGRA_GPIO(X, 7),		/* wake53 */
	TEGRA_GPIO(Y, 0),		/* wake54 */
	TEGRA_GPIO(Y, 1),		/* wake55 */
	TEGRA_GPIO(Y, 2),		/* wake56 */
	TEGRA_GPIO(Y, 5),		/* wake57 */
	TEGRA_GPIO(Y, 6),		/* wake58 */
	TEGRA_GPIO(L, 1),		/* wake59 */
	TEGRA_GPIO(L, 3),		/* wake60 */
	TEGRA_GPIO(L, 4),		/* wake61 */
	TEGRA_GPIO(L, 5),		/* wake62 */
	TEGRA_GPIO(I, 4),		/* wake63 */
	TEGRA_GPIO(I, 6),		/* wake64 */
	-EINVAL,			/* wake65 */
	-EINVAL,			/* wake66 */
	-EINVAL,			/* wake67 */
	-EINVAL,			/* wake68 */
	-EINVAL,		/* wake69 */
	TEGRA_GPIO(H, 3),	/* wake70 */
	TEGRA_GPIO(P, 5),	/* wake71 */
	-EINVAL,		/* wake72 */
	-EINVAL,		/* wake73 */
	-EINVAL,		/* wake74 */
	-EINVAL,		/* wake75 */
	-EINVAL,		/* wake76 */
	-EINVAL,		/* wake77 */
	-EINVAL,		/* wake78 */
	-EINVAL,		/* wake79 */
	-EINVAL,		/* wake80 */
	-EINVAL,		/* wake81 */
	-EINVAL,		/* wake82 */
	-EINVAL,		/* wake83 */
	-EINVAL,		/* wake84 */
	-EINVAL,		/* wake85 */
	-EINVAL,		/* wake86 */
	-EINVAL,		/* wake87 */
	-EINVAL,		/* wake88 */
	-EINVAL,		/* wake89 */
	-EINVAL,		/* wake90 */
	-EINVAL,		/* wake91 */
	-EINVAL,		/* wake92 */
	-EINVAL,		/* wake93 */
	-EINVAL,		/* wake94 */
	-EINVAL,		/* wake95 */
};

static const int tegra186_aon_gpio_wakes[] = {
	-EINVAL,			/* wake0 */
	-EINVAL,			/* wake1 */
	-EINVAL,			/* wake2 */
	-EINVAL,			/* wake3 */
	-EINVAL,			/* wake4 */
	-EINVAL,			/* wake5 */
	-EINVAL,			/* wake6 */
	-EINVAL,			/* wake7 */
	-EINVAL,			/* wake8 */
	-EINVAL,			/* wake9 */
	TEGRA_GPIO_AON(S, 2),		/* wake10 */
	-EINVAL,			/* wake11 */
	-EINVAL,			/* wake12 */
	-EINVAL,			/* wake13 */
	-EINVAL,			/* wake14 */
	-EINVAL,			/* wake15 */
	-EINVAL,			/* wake16 */
	-EINVAL,			/* wake17 */
	-EINVAL,			/* wake18 */
	-EINVAL,			/* wake19 */
	-EINVAL,			/* wake20 */
	-EINVAL,			/* wake21 */
	-EINVAL,			/* wake22 */
	-EINVAL,			/* wake23 */
	-EINVAL,			/* wake24 */
	TEGRA_GPIO_AON(S, 3),		/* wake25 */
	TEGRA_GPIO_AON(S, 4),		/* wake26 */
	TEGRA_GPIO_AON(S, 1),		/* wake27 */
	-EINVAL,			/* wake28 */
	TEGRA_GPIO_AON(FF, 0),		/* wake29 */
	TEGRA_GPIO_AON(FF, 4),		/* wake30 */
	-EINVAL,			/* wake31 */
	TEGRA_GPIO_AON(W, 2),		/* wake32 */
	TEGRA_GPIO_AON(W, 5),		/* wake33 */
	TEGRA_GPIO_AON(W, 1),		/* wake34 */
	TEGRA_GPIO_AON(V, 0),		/* wake35 */
	TEGRA_GPIO_AON(V, 1),		/* wake36 */
	TEGRA_GPIO_AON(V, 2),		/* wake37 */
	TEGRA_GPIO_AON(V, 3),		/* wake38 */
	TEGRA_GPIO_AON(V, 4),		/* wake39 */
	TEGRA_GPIO_AON(V, 5),		/* wake40 */
	TEGRA_GPIO_AON(EE, 0),		/* wake41 */
	TEGRA_GPIO_AON(Z, 1),		/* wake42 */
	TEGRA_GPIO_AON(Z, 3),		/* wake43 */
	TEGRA_GPIO_AON(AA, 0),		/* wake44 */
	TEGRA_GPIO_AON(AA, 1),		/* wake45 */
	TEGRA_GPIO_AON(AA, 2),		/* wake46 */
	TEGRA_GPIO_AON(AA, 3),		/* wake47 */
	TEGRA_GPIO_AON(AA, 4),		/* wake48 */
	TEGRA_GPIO_AON(AA, 5),		/* wake49 */
	TEGRA_GPIO_AON(AA, 6),		/* wake50 */
	TEGRA_GPIO_AON(AA, 7),		/* wake51 */
	-EINVAL,			/* wake52 */
	-EINVAL,			/* wake53 */
	-EINVAL,			/* wake54 */
	-EINVAL,			/* wake55 */
	-EINVAL,			/* wake56 */
	-EINVAL,			/* wake57 */
	-EINVAL,			/* wake58 */
	-EINVAL,			/* wake59 */
	-EINVAL,			/* wake60 */
	-EINVAL,			/* wake61 */
	-EINVAL,			/* wake62 */
	-EINVAL,			/* wake63 */
	-EINVAL,			/* wake64 */
	TEGRA_GPIO_AON(Z, 0),		/* wake65 */
	TEGRA_GPIO_AON(Z, 2),		/* wake66 */
	TEGRA_GPIO_AON(FF, 1),		/* wake67 */
	TEGRA_GPIO_AON(FF, 2),		/* wake68 */
	TEGRA_GPIO_AON(FF, 3),		/* wake69 */
	-EINVAL,			/* wake70 */
	-EINVAL,			/* wake71 */
	-EINVAL,		/* wake72 */
	-EINVAL,		/* wake73 */
	-EINVAL,		/* wake74 */
	-EINVAL,		/* wake75 */
	-EINVAL,		/* wake76 */
	-EINVAL,		/* wake77 */
	-EINVAL,		/* wake78 */
	-EINVAL,		/* wake79 */
	-EINVAL,		/* wake80 */
	-EINVAL,		/* wake81 */
	-EINVAL,		/* wake82 */
	-EINVAL,		/* wake83 */
	-EINVAL,		/* wake84 */
	-EINVAL,		/* wake85 */
	-EINVAL,		/* wake86 */
	-EINVAL,		/* wake87 */
	-EINVAL,		/* wake88 */
	-EINVAL,		/* wake89 */
	-EINVAL,		/* wake90 */
	-EINVAL,		/* wake91 */
	-EINVAL,		/* wake92 */
	-EINVAL,		/* wake93 */
	-EINVAL,		/* wake94 */
	-EINVAL,		/* wake95 */
};


struct tegra_gpio_port_soc_info {
	int cont_id;
	int cont_index;
	int valid_pins;
	int reg_index;
	int scr_offset;
	u32 reg_offset;
};

#define TEGRA_GPIO_PORT_INFO(port, cid, cind, npins)		\
[TEGRA_GPIO_BANK_ID_##port] = {					\
		.cont_id = cid,					\
		.cont_index = cind,				\
		.valid_pins = npins,				\
		.reg_index = 0,					\
		.scr_offset = cid * 0x1000 + cind * 0x40,	\
		.reg_offset = cid * 0x1000 + cind * 0x200,	\
}

#define TEGRA_AON_GPIO_PORT_INFO(port, cid, cind, npins)	\
[TEGRA_AON_GPIO_BANK_ID_##port] = {				\
		.cont_id = cid,					\
		.cont_index = cind,				\
		.valid_pins = npins,				\
		.reg_index = 1,					\
		.scr_offset = cind * 0x40,			\
		.reg_offset = cind * 0x200,			\
}

static struct tegra_gpio_port_soc_info tegra_gpio_cinfo[] = {
	TEGRA_GPIO_PORT_INFO(A, 2, 0, 7),
	TEGRA_GPIO_PORT_INFO(B, 3, 0, 7),
	TEGRA_GPIO_PORT_INFO(C, 3, 1, 7),
	TEGRA_GPIO_PORT_INFO(D, 3, 2, 6),
	TEGRA_GPIO_PORT_INFO(E, 2, 1, 8),
	TEGRA_GPIO_PORT_INFO(F, 2, 2, 6),
	TEGRA_GPIO_PORT_INFO(G, 4, 1, 6),
	TEGRA_GPIO_PORT_INFO(H, 1, 0, 7),
	TEGRA_GPIO_PORT_INFO(I, 0, 4, 8),
	TEGRA_GPIO_PORT_INFO(J, 5, 0, 8),
	TEGRA_GPIO_PORT_INFO(K, 5, 1, 1),
	TEGRA_GPIO_PORT_INFO(L, 1, 1, 8),
	TEGRA_GPIO_PORT_INFO(M, 5, 3, 6),
	TEGRA_GPIO_PORT_INFO(N, 0, 0, 7),
	TEGRA_GPIO_PORT_INFO(O, 0, 1, 4),
	TEGRA_GPIO_PORT_INFO(P, 4, 0, 7),
	TEGRA_GPIO_PORT_INFO(Q, 0, 2, 6),
	TEGRA_GPIO_PORT_INFO(R, 0, 5, 6),
	TEGRA_GPIO_PORT_INFO(T, 0, 3, 4),
	TEGRA_GPIO_PORT_INFO(X, 1, 2, 8),
	TEGRA_GPIO_PORT_INFO(Y, 1, 3, 7),
	TEGRA_GPIO_PORT_INFO(BB, 2, 3, 2),
	TEGRA_GPIO_PORT_INFO(CC, 5, 2, 4),
	TEGRA_GPIO_PORT_INFO(DD, -1, -1, 0),
};

static struct tegra_gpio_port_soc_info tegra_aon_gpio_cinfo[] = {
	TEGRA_AON_GPIO_PORT_INFO(S, 0, 1, 5),
	TEGRA_AON_GPIO_PORT_INFO(U, 0, 2, 6),
	TEGRA_AON_GPIO_PORT_INFO(V, 0, 4, 8),
	TEGRA_AON_GPIO_PORT_INFO(W, 0, 5, 8),
	TEGRA_AON_GPIO_PORT_INFO(Z, 0, 7, 4),
	TEGRA_AON_GPIO_PORT_INFO(AA, 0, 6, 8),
	TEGRA_AON_GPIO_PORT_INFO(EE, 0, 3, 3),
	TEGRA_AON_GPIO_PORT_INFO(FF, 0, 0, 5),
};

struct tegra_gpio_info;

struct tegra_gpio_soc_info {
	const char *name;
	const char *debug_fs_name;
	const struct tegra_gpio_port_soc_info *port;
	int nports;
	const int *wake_table;
	int nwakes;
};

struct tegra_gpio_controller {
	int controller;
	int irq;
	struct tegra_gpio_info *tgi;
};

struct tegra_gpio_info {
	struct device *dev;

	int nbanks;
	void __iomem *gpio_regs;
	void __iomem *scr_regs;
	struct irq_domain *irq_domain;
	const struct tegra_gpio_soc_info *soc;
	struct tegra_gpio_controller tg_contrlr[MAX_GPIO_CONTROLLERS];
	struct gpio_chip gc;
	struct irq_chip ic;
	struct lock_class_key lock_class;
};

static int tegra186_gpio_to_wake(struct tegra_gpio_info *tgi, int gpio)
{
	int i;

	for (i = 0; i < tgi->soc->nwakes; i++) {
		if (tgi->soc->wake_table[i] == gpio) {
			pr_info("gpio %s wake%d for gpio=%d\n",
				tgi->soc->name, i, gpio);
			return i;
		}
	}

	return -EINVAL;
}

static inline u32 tegra_gpio_readl(struct tegra_gpio_info *tgi, u32 gpio,
				   u32 reg_offset)
{
	int port = GPIO_PORT(gpio);
	int pin = GPIO_PIN(gpio);
	u32 addr = tgi->soc->port[port].reg_offset;

	addr += (GPIO_REG_DIFF * pin) + reg_offset;
	return __raw_readl(tgi->gpio_regs + addr);
}

static inline void tegra_gpio_writel(struct tegra_gpio_info *tgi, u32 val,
				     u32 gpio, u32 reg_offset)
{
	int port = GPIO_PORT(gpio);
	int pin = GPIO_PIN(gpio);
	u32 addr = tgi->soc->port[port].reg_offset;

	addr += (GPIO_REG_DIFF * pin) + reg_offset;
	__raw_writel(val, tgi->gpio_regs + addr);
}

static inline void tegra_gpio_update(struct tegra_gpio_info *tgi, u32 gpio,
				     u32 reg_offset,	u32 mask, u32 val)
{
	int port = GPIO_PORT(gpio);
	int pin = GPIO_PIN(gpio);
	u32 addr = tgi->soc->port[port].reg_offset;
	u32 rval;

	addr += (GPIO_REG_DIFF * pin) + reg_offset;
	rval = __raw_readl(tgi->gpio_regs + addr);
	rval = (rval & ~mask) | (val & mask);
	__raw_writel(rval, tgi->gpio_regs + addr);
}

int tegra_gpio_get_bank_int_nr(int gpio)
{
	return gpio_to_irq(gpio);
}
EXPORT_SYMBOL(tegra_gpio_get_bank_int_nr);

/*
 * This function will return if the GPIO is accessible by CPU
 */
static inline bool is_gpio_accessible(struct tegra_gpio_info *tgi, u32 offset)
{
	int port = GPIO_PORT(offset);
	int pin = GPIO_PIN(offset);
	u32 val;
	int cont_id;
	u32 scr_offset = tgi->soc->port[port].scr_offset;

	if (pin >= tgi->soc->port[port].valid_pins)
		return false;

	cont_id = tgi->soc->port[port].cont_id;
	if (cont_id  < 0)
		return false;

	val = __raw_readl(tgi->scr_regs + scr_offset +
			(pin * GPIO_SCR_DIFF) + GPIO_SCR_REG);

	if ((val & GPIO_FULL_ACCESS) == GPIO_FULL_ACCESS)
		return true;

	return false;
}

static void tegra_gpio_enable(struct tegra_gpio_info *tgi, int gpio)
{
	tegra_gpio_update(tgi, gpio, GPIO_ENB_CONFIG_REG, 0x1, 0x1);
}

static void tegra_gpio_disable(struct tegra_gpio_info *tgi, int gpio)
{
	tegra_gpio_update(tgi, gpio, GPIO_ENB_CONFIG_REG, 0x1, 0x0);
}

static int tegra_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	bool accessible;

	accessible = is_gpio_accessible(tgi, offset);
	if (accessible)
		return pinctrl_request_gpio(chip->base + offset);
	return -EBUSY;
}

static void tegra_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);

	pinctrl_free_gpio(chip->base + offset);
	tegra_gpio_disable(tgi, offset);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	u32 val = (value) ? 0x1 : 0x0;

	tegra_gpio_writel(tgi, val, offset, GPIO_OUT_VAL_REG);
	tegra_gpio_writel(tgi, 0, offset, GPIO_OUT_CTRL_REG);
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	u32 val;

	val = tegra_gpio_readl(tgi, offset, GPIO_ENB_CONFIG_REG);
	if (val & GPIO_INOUT_BIT)
		return tegra_gpio_readl(tgi, offset, GPIO_OUT_VAL_REG) & 0x1;

	return tegra_gpio_readl(tgi, offset, GPIO_INPUT_REG) & 0x1;
}

static void set_gpio_direction_mode(struct gpio_chip *chip, u32 offset,
				    bool mode)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	u32 val;

	val = tegra_gpio_readl(tgi, offset, GPIO_ENB_CONFIG_REG);
	if (mode)
		val |= GPIO_INOUT_BIT;
	else
		val &= ~GPIO_INOUT_BIT;
	tegra_gpio_writel(tgi, val, offset, GPIO_ENB_CONFIG_REG);
}

static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	int ret;

	set_gpio_direction_mode(chip, offset, 0);
	tegra_gpio_enable(tgi, offset);
	ret = pinctrl_gpio_direction_input(chip->base + offset);
	if (ret < 0)
		dev_err(chip->parent,
			"Tegra gpio input: pinctrl input failed: %d\n", ret);
	return 0;
}

static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				       int value)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	int ret;

	tegra_gpio_set(chip, offset, value);
	set_gpio_direction_mode(chip, offset, 1);
	tegra_gpio_enable(tgi, offset);
	ret = pinctrl_gpio_direction_output(chip->base + offset);
	if (ret < 0)
		dev_err(chip->parent,
			"Tegra gpio output: pinctrl output failed: %d\n", ret);
	return 0;
}

static int tegra_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				   unsigned debounce)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	unsigned dbc_ms = DIV_ROUND_UP(debounce, 1000);

	tegra_gpio_update(tgi, offset, GPIO_ENB_CONFIG_REG, 0x1, 0x1);
	tegra_gpio_update(tgi, offset, GPIO_DEB_FUNC_BIT, 0x5, 0x1);
	/* Update debounce threshold */
	tegra_gpio_writel(tgi, dbc_ms, offset, GPIO_DBC_THRES_REG);
	return 0;
}

static int tegra_gpio_is_enabled(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	u32 val;

	if (!is_gpio_accessible(tgi, offset))
		return 0;

	val = tegra_gpio_readl(tgi, offset, GPIO_ENB_CONFIG_REG);

	return !!(val & 0x1);
}

static int tegra_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);
	u32 val;

	if (!is_gpio_accessible(tgi, offset))
		return 0;

	val = tegra_gpio_readl(tgi, offset, GPIO_OUT_CTRL_REG);

	return (val & 0x1);
}

static int tegra_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct tegra_gpio_info *tgi = gpiochip_get_data(chip);

	return irq_find_mapping(tgi->irq_domain, offset);
}

static void tegra_gpio_irq_ack(struct irq_data *d)
{
	struct tegra_gpio_controller *ctrlr = irq_data_get_irq_chip_data(d);
	int gpio = d->hwirq;

	tegra_gpio_writel(ctrlr->tgi, 1, gpio, GPIO_INT_CLEAR_REG);
}

static void tegra_gpio_irq_mask(struct irq_data *d)
{
	struct tegra_gpio_controller *c = irq_data_get_irq_chip_data(d);
	int gpio = d->hwirq;

	tegra_gpio_update(c->tgi, gpio, GPIO_ENB_CONFIG_REG, GPIO_INT_FUNC_BIT,
			  0);
}

static void tegra_gpio_irq_unmask(struct irq_data *d)
{
	struct tegra_gpio_controller *c = irq_data_get_irq_chip_data(d);
	int gpio = d->hwirq;

	tegra_gpio_update(c->tgi, gpio, GPIO_ENB_CONFIG_REG, GPIO_INT_FUNC_BIT,
			  GPIO_INT_FUNC_BIT);
}

static int tegra_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct tegra_gpio_controller *ctrlr = irq_data_get_irq_chip_data(d);
	int gpio = d->hwirq;
	u32 lvl_type = 0;
	u32 trg_type = 0;
	u32 val;
	int wake = tegra186_gpio_to_wake(ctrlr->tgi, d->hwirq);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		trg_type = TRIGGER_LEVEL_HIGH;
		lvl_type = GPIO_INT_LVL_SINGLE_EDGE_TRIGGER;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		trg_type = TRIGGER_LEVEL_LOW;
		lvl_type = GPIO_INT_LVL_SINGLE_EDGE_TRIGGER;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_INT_LVL_BOTH_EDGE_TRIGGER;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		trg_type = TRIGGER_LEVEL_HIGH;
		lvl_type = GPIO_INT_LVL_LEVEL_TRIGGER;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		trg_type = TRIGGER_LEVEL_LOW;
		lvl_type = GPIO_INT_LVL_LEVEL_TRIGGER;
		break;

	default:
		return -EINVAL;
	}

	trg_type = trg_type << 0x4;
	lvl_type = lvl_type << 0x2;

	/* Clear and Program the values */
	val = tegra_gpio_readl(ctrlr->tgi, gpio, GPIO_ENB_CONFIG_REG);
	val &= ~((0x3 << GPIO_TRG_TYPE_BIT_OFFSET) | (GPIO_TRG_LVL_BIT));
	val |= trg_type | lvl_type;
	tegra_gpio_writel(ctrlr->tgi, val, gpio, GPIO_ENB_CONFIG_REG);

	tegra_gpio_enable(ctrlr->tgi, gpio);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		irq_set_handler_locked(d, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		irq_set_handler_locked(d, handle_edge_irq);

	if (wake >= 0)
		tegra_pm_irq_set_wake_type(wake, type);

	return 0;
}

static int tegra_gpio_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	struct tegra_gpio_controller *ctrlr = irq_data_get_irq_chip_data(d);
	int wake = tegra186_gpio_to_wake(ctrlr->tgi, d->hwirq);
	int ret;

	if (wake < 0)
		return wake;

	ret = tegra_pm_irq_set_wake(wake, enable);
	if (ret)
		pr_err("Failed gpio lp0 %s for irq=%d, error=%d\n",
		       (enable ? "enable" : "disable"), d->irq, ret);
	return ret;
}

static void tegra_gpio_irq_handler_desc(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct tegra_gpio_controller *tg_cont = irq_desc_get_handler_data(desc);
	struct tegra_gpio_info *tgi = tg_cont->tgi;
	int pin;
	int port;
	u32 i;
	unsigned long val;
	u32 gpio;
	u32 addr;
	int port_map[MAX_GPIO_PORTS];

	for (i = 0; i < MAX_GPIO_PORTS; ++i)
		port_map[i] = -1;

	for (i = 0; i < tgi->soc->nports; ++i) {
		if (tgi->soc->port[i].cont_id == tg_cont->controller)
			port_map[tgi->soc->port[i].cont_index] = i;
	}

	chained_irq_enter(chip, desc);
	for (i = 0; i < MAX_GPIO_PORTS; i++) {
		port = port_map[i];
		if (port == -1)
			continue;

		addr = tgi->soc->port[port].reg_offset;
		val = __raw_readl(tg_cont->tgi->gpio_regs + addr +
				GPIO_INT_STATUS_OFFSET + GPIO_STATUS_G1);
		gpio = tgi->gc.base + (port * 8);
		for_each_set_bit(pin, &val, 8)
			generic_handle_irq(gpio_to_irq(gpio + pin));
	}

	chained_irq_exit(chip, desc);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
static void tegra_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	tegra_gpio_irq_handler_desc(desc);
}
#else
static void tegra_gpio_irq_handler(struct irq_desc *desc)
{
	tegra_gpio_irq_handler_desc(desc);
}
#endif

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	struct tegra_gpio_info *tgi = s->private;
	int i;
	bool accessible;
	char x, y;
	int count = 0;

	x = ' ';
	y = 'A';

	seq_puts(s, "Port:Pin:ENB DBC IN OUT_CTRL OUT_VAL INT_CLR\n");

	for (i = 0; i < tgi->gc.ngpio; i++) {
		accessible = is_gpio_accessible(tgi, i);
		if (count == 8)
			count = 0;

		if ((count == 0) && (i / 8)) {
			if (x != ' ')
				x++;
			if (y == 'Z') {
				y = 'A';
				x = 'A';
			} else {
				y++;
			}
		}
		count++;
		if (accessible) {
			seq_printf(s, "%c%c:%d 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				   x, y, i % 8,
				tegra_gpio_readl(tgi, i, GPIO_ENB_CONFIG_REG),
				tegra_gpio_readl(tgi, i, GPIO_DBC_THRES_REG),
				tegra_gpio_readl(tgi, i, GPIO_INPUT_REG),
				tegra_gpio_readl(tgi, i, GPIO_OUT_CTRL_REG),
				tegra_gpio_readl(tgi, i, GPIO_OUT_VAL_REG),
				tegra_gpio_readl(tgi, i, GPIO_INT_CLEAR_REG));
		}
	}

	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_gpio_debuginit(struct tegra_gpio_info *tgi)
{
	(void)debugfs_create_file(tgi->soc->debug_fs_name, S_IRUGO,
				   NULL, tgi, &debug_fops);

	return 0;
}
#else
static inline int tegra_gpio_debuginit(struct tegra_gpio_info *tgi)
{
	return 0;
}
#endif

static int tegra_gpio_probe(struct platform_device *pdev)
{
	struct tegra_gpio_info *tgi;
	struct resource *res;
	int bank;
	int gpio;
	int ret;

	for (bank = 0;; bank++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, bank);
		if (!res)
			break;
	}
	if (!bank) {
		dev_err(&pdev->dev, "No GPIO Controller found\n");
		return -ENODEV;
	}

	tgi = devm_kzalloc(&pdev->dev, sizeof(*tgi), GFP_KERNEL);
	if (!tgi)
		return -ENOMEM;
	tgi->dev = &pdev->dev;
	tgi->nbanks = bank;
	tgi->soc = of_device_get_match_data(&pdev->dev);

	tgi->gc.label			= tgi->soc->name;
	tgi->gc.request			= tegra_gpio_request;
	tgi->gc.free			= tegra_gpio_free;
	tgi->gc.direction_input		= tegra_gpio_direction_input;
	tgi->gc.get			= tegra_gpio_get;
	tgi->gc.direction_output	= tegra_gpio_direction_output;
	tgi->gc.set			= tegra_gpio_set;
	tgi->gc.get_direction		= tegra_gpio_get_direction;
	tgi->gc.is_enabled			= tegra_gpio_is_enabled;
	tgi->gc.to_irq			= tegra_gpio_to_irq;
	tgi->gc.set_debounce		= tegra_gpio_set_debounce;
	tgi->gc.base			= -1;
	tgi->gc.ngpio			= tgi->soc->nports * 8;
	tgi->gc.parent			= &pdev->dev;
	tgi->gc.of_node			= pdev->dev.of_node;

	tgi->ic.name		= tgi->soc->name;
	tgi->ic.irq_ack		= tegra_gpio_irq_ack;
	tgi->ic.irq_mask	= tegra_gpio_irq_mask;
	tgi->ic.irq_unmask	= tegra_gpio_irq_unmask;
	tgi->ic.irq_set_type	= tegra_gpio_irq_set_type;
	tgi->ic.irq_shutdown	= tegra_gpio_irq_mask;
	tgi->ic.irq_set_wake	= tegra_gpio_irq_set_wake;
	tgi->ic.irq_disable	= tegra_gpio_irq_mask;

	platform_set_drvdata(pdev, tgi);

	for (bank = 0; bank < tgi->nbanks; bank++) {
		res = platform_get_resource(pdev, IORESOURCE_IRQ, bank);
		tgi->tg_contrlr[bank].controller = bank;
		tgi->tg_contrlr[bank].irq = res->start;
		tgi->tg_contrlr[bank].tgi = tgi;
	}

	tgi->irq_domain = irq_domain_add_linear(pdev->dev.of_node,
						tgi->gc.ngpio,
						&irq_domain_simple_ops, NULL);
	if (!tgi->irq_domain)
		return -ENODEV;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "security");
	if (!res) {
		dev_err(&pdev->dev, "Missing security MEM resource\n");
		return -ENODEV;
	}
	tgi->scr_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(tgi->scr_regs)) {
		ret = PTR_ERR(tgi->scr_regs);
		dev_err(&pdev->dev, "Failed to iomap for security: %d\n", ret);
		return ret;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gpio");
	if (!res) {
		dev_err(&pdev->dev, "Missing gpio MEM resource\n");
		return -ENODEV;
	}
	tgi->gpio_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(tgi->gpio_regs)) {
		ret = PTR_ERR(tgi->gpio_regs);
		dev_err(&pdev->dev, "Failed to iomap for gpio: %d\n", ret);
		return ret;
	}

	ret = gpiochip_add_data(&tgi->gc, tgi);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	for (gpio = 0; gpio < tgi->gc.ngpio; gpio++) {
		int irq = irq_create_mapping(tgi->irq_domain, gpio);
		int cont_id = tgi->soc->port[GPIO_PORT(gpio)].cont_id;

		if (is_gpio_accessible(tgi, gpio))
			/* mask interrupts for this GPIO */
			tegra_gpio_update(tgi, gpio, GPIO_ENB_CONFIG_REG,
					  GPIO_INT_FUNC_BIT, 0);

		irq_set_lockdep_class(irq, &tgi->lock_class);
		irq_set_chip_data(irq, &tgi->tg_contrlr[cont_id]);
		irq_set_chip_and_handler(irq, &tgi->ic, handle_simple_irq);
	}

	for (bank = 0; bank < tgi->nbanks; bank++)
		irq_set_chained_handler_and_data(tgi->tg_contrlr[bank].irq,
						 tegra_gpio_irq_handler,
						 &tgi->tg_contrlr[bank]);

	tegra_pm_update_gpio_wakeup_table(tgi->gc.base,
					  (int *)tgi->soc->wake_table,
					  tgi->soc->nwakes);

	tegra_gpio_debuginit(tgi);

	return 0;
}

static const struct tegra_gpio_soc_info t186_gpio_soc = {
	.name = "tegra-gpio",
	.debug_fs_name = "tegra_gpio",
	.port = tegra_gpio_cinfo,
	.nports = ARRAY_SIZE(tegra_gpio_cinfo),
	.wake_table = tegra186_gpio_wakes,
	.nwakes = ARRAY_SIZE(tegra186_gpio_wakes),
};

static const struct tegra_gpio_soc_info t186_aon_gpio_soc = {
	.name = "tegra-gpio-aon",
	.debug_fs_name = "tegra-gpio-on",
	.port = tegra_aon_gpio_cinfo,
	.nports = ARRAY_SIZE(tegra_aon_gpio_cinfo),
	.wake_table = tegra186_aon_gpio_wakes,
	.nwakes = ARRAY_SIZE(tegra186_aon_gpio_wakes),
};

static struct of_device_id tegra_gpio_of_match[] = {
	{ .compatible = "nvidia,tegra186-gpio", .data = &t186_gpio_soc},
	{ .compatible = "nvidia,tegra186-gpio-aon", .data = &t186_aon_gpio_soc},
	{ },
};

static struct platform_driver tegra_gpio_driver = {
	.driver		= {
		.name	= "tegra-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = tegra_gpio_of_match,
	},
	.probe		= tegra_gpio_probe,
};

static int __init tegra_gpio_init(void)
{
	return platform_driver_register(&tegra_gpio_driver);
}
postcore_initcall(tegra_gpio_init);

MODULE_AUTHOR("Suresh Mangipudi <smangipudi@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra186 GPIO driver");
MODULE_LICENSE("GPL v2");
