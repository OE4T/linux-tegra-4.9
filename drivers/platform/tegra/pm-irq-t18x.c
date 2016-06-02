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

#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/syscore_ops.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-pmc.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-fuse.h>

#include "tegra186-aowake.h"

/* Per wake registers */
#define WAKE_AOWAKE_CNTRL_0		0x0	/* ~0x17f */
#define WAKE_AOWAKE_MASK_W_0		0x180	/* ~0x2ff */
#define WAKE_AOWAKE_STATUS_W_0		0x30c	/* ~0x48b */

/* Aggregated wake registers */
#define WAKE_AOWAKE_MASK_R_31_0_0	0x300
#define WAKE_AOWAKE_MASK_R_63_32_0	0x304
#define WAKE_AOWAKE_MASK_R_95_64_0	0x308

#define WAKE_AOWAKE_STATUS_R_31_0_0	0x48c
#define WAKE_AOWAKE_SW_STATUS_31_0_0	0x4a0
#define WAKE_AOWAKE_TIER2_ROUTING_31_0_0	0x4cc

/* Regular registers */
#define WAKE_LATCH_SW		0x498

#define WAKE_NR_EVENTS	96
#define WAKE_NR_VECTORS	(WAKE_NR_EVENTS / 32)

#define wk_set_bit(nr, p) (__set_bit(nr, (ulong *)p))
#define wk_clr_bit(nr, p) (__clear_bit(nr, (ulong *)p))

/* wake level/polarity constants */
enum {
	WAKE_LEVEL_LO = 0,
	WAKE_LEVEL_HI,
	WAKE_LEVEL_ANY
};

static u32 wke_wake_enb[WAKE_NR_VECTORS];
static u32 wke_wake_level[WAKE_NR_VECTORS];
static u32 wke_wake_level_any[WAKE_NR_VECTORS];

static u32 wke_wake_irq_count[WAKE_NR_EVENTS];

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
static void wke_32kwritel(u32 val, u32 reg)
{
	tegra_aowake_write(val, reg);
	udelay(130);
}

static void print_vals(char *name, u32 *vals)
{
	int i;
	for (i = 0; i < WAKE_NR_VECTORS; i++)
		pr_info("Wake[%d-%d]  %s=%#x\n",
			(i + 1) * 32, i * 32, name, vals[i]);
}

static void wke_write_wake_masks(u32 *enb)
{
	u32 reg = WAKE_AOWAKE_MASK_W_0;
	u32 val;
	int i;
	for (i = 0; i < WAKE_NR_EVENTS; i++, reg += 4) {
		val = test_bit(i, (ulong *)enb);
		tegra_aowake_write(val, reg);
	}
	print_vals("enable", enb);
}

static void wke_write_tier2_routing(u32 *enb)
{
	int i;
	u32 reg = WAKE_AOWAKE_TIER2_ROUTING_31_0_0;

	for (i = 0; i < WAKE_NR_VECTORS; i++, reg += 4)
		tegra_aowake_write(enb[i], reg);
	print_vals("route", enb);
}

static void wke_read_wake_levels(u32 *lvl)
{
	int i, j;
	u32 val;
	u32 reg = WAKE_AOWAKE_CNTRL_0;

	for (i = 0; i < WAKE_NR_VECTORS; i++, reg += 4) {
		lvl[i] = 0;
		for (j = 0; j < 32; j++) {
			val = tegra_aowake_read(reg);
			lvl[i] |= (((val >> 3) & 0x1) << j);
		}
	}
}

static void wke_write_wake_levels(u32 *lvl)
{
	int i;
	u32 val;
	u32 reg = WAKE_AOWAKE_CNTRL_0;

	for (i = 0; i < WAKE_NR_EVENTS; i++, reg += 4) {
		val = tegra_aowake_read(reg);
		val |= test_bit(i, (ulong *)lvl) << 3;
		tegra_aowake_write(val, reg);
	}
	print_vals("level", lvl);
}

int tegra_read_wake_status(u32 *status)
{
	int i;
	u32 reg = WAKE_AOWAKE_STATUS_R_31_0_0;
	u32 mask = WAKE_AOWAKE_TIER2_ROUTING_31_0_0;

	for (i = 0; i < WAKE_NR_VECTORS; i++, reg += 4, mask += 4) {
		status[i] = tegra_aowake_read(reg);
		status[i] = status[i] & tegra_aowake_read(mask);
	}

	return WAKE_NR_VECTORS;
}

static void wke_read_sw_wake_status(u32 *status)
{
	int i;
	u32 reg = WAKE_AOWAKE_SW_STATUS_31_0_0;
	u32 mask = WAKE_AOWAKE_TIER2_ROUTING_31_0_0;

	for (i = 0; i < WAKE_NR_VECTORS; i++, reg += 4, mask += 4) {
		status[i] = tegra_aowake_read(reg);
		status[i] = status[i] & tegra_aowake_read(mask);
	}
}

static void wke_clear_sw_wake_status(void)
{
	int i;
	u32 reg = WAKE_AOWAKE_STATUS_W_0;

	for (i = 0; i < WAKE_NR_EVENTS; i++, reg += 4)
		wke_32kwritel(0, reg);
}

static void wke_clear_wake_status(void)
{
	u32 regw;
	u32 status;
	int i, wake;
	u32 reg = WAKE_AOWAKE_STATUS_R_31_0_0;
	u32 mask = WAKE_AOWAKE_TIER2_ROUTING_31_0_0;

	for (i = 0; i < WAKE_NR_VECTORS; i++, reg += 4, mask += 4) {
		status = tegra_aowake_read(reg);
		status = status & tegra_aowake_read(reg);
		regw = WAKE_AOWAKE_STATUS_W_0 + i * 32 * 4;
		for_each_set_bit(wake, (ulong *)&status, 32)
			wke_32kwritel(1, regw + wake * 4);
	}
}

static int wke_irq_set_wake(int wake, int enable)
{
	if (wake < 0)
		return -EINVAL;

	if (enable) {
		wk_set_bit(wake, wke_wake_enb);
		pr_info("Enabling wake%d\n", wake);
	} else {
		wk_clr_bit(wake, wke_wake_enb);
		pr_info("Disabling wake%d\n", wake);
	}

	return 0;
}

static int wke_irq_set_wake_level(int wake, int flow_type)
{
	if (wake < 0)
		return -EINVAL;

	switch (flow_type) {
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		wk_clr_bit(wake, wke_wake_level);
		wk_clr_bit(wake, wke_wake_level_any);
		break;
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		wk_set_bit(wake, wke_wake_level);
		wk_set_bit(wake, wke_wake_level_any);
		break;
	case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
		wk_set_bit(wake, wke_wake_level_any);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* translate sc7 wake sources back into irqs to catch edge triggered wakeups */
static void process_wake_event(int index, u32 status)
{
	int irq;
	int wake;
	struct irq_desc *desc;

	pr_info("Wake[%d:%d]  status=0x%x\n",
		(index + 1) * 32, index * 32, status);

	for_each_set_bit(wake, (ulong *)&status, 32) {
		irq = tegra_wake_to_irq(wake + 32 * index);
		if (irq == -EINVAL) {
			pr_info("Resume caused by WAKE%d\n",
				(wake + 32 * index));
			continue;
		}

		desc = irq_to_desc(irq);
		if (!desc || !desc->action || !desc->action->name) {
			pr_info("Resume caused by WAKE%d, irq %d\n",
				(wake + 32 * index), irq);
			continue;
		}

		pr_info("Resume caused by WAKE%d, %s\n", (wake + 32 * index),
			desc->action->name);

		wke_wake_irq_count[wake + 32 * index]++;

		generic_handle_irq(irq);
	}
}

static void tegra_pm_irq_resume(void)
{
	int i;
	u32 status;
	u32 reg = WAKE_AOWAKE_STATUS_R_31_0_0;
	u32 mask = WAKE_AOWAKE_TIER2_ROUTING_31_0_0;

	for (i = 0; i < WAKE_NR_VECTORS; i++, reg += 4, mask += 4) {
		status = tegra_aowake_read(reg);
		status = status & tegra_aowake_read(mask);
		process_wake_event(i, status);
	}
}

/* set up sc7 wake sources */
static int tegra_pm_irq_suspend(void)
{
	u32 status[WAKE_NR_VECTORS];
	u32 lvl[WAKE_NR_VECTORS];
	u32 wake_level[WAKE_NR_VECTORS];
	u32 wake_enb[WAKE_NR_VECTORS];
	enum tegra_revision revision;
	int i;

	wke_clear_sw_wake_status();

	wke_32kwritel(1, WAKE_LATCH_SW);
	wke_32kwritel(0, WAKE_LATCH_SW);

	wke_read_sw_wake_status(status);

	wke_read_wake_levels(lvl);

	/* flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups */
	for (i = 0; i < WAKE_NR_VECTORS; i++) {
		lvl[i] ^= status[i];
		lvl[i] &= wke_wake_level_any[i];
		wake_level[i] = lvl[i] | wke_wake_level[i];
		wake_enb[i] = wke_wake_enb[i];
	}

	/* Clear PMC Wake Status registers while going to suspend */
	wke_clear_wake_status();
	revision = tegra_chip_get_revision();
	if (revision < TEGRA_REVISION_A02p)
		wake_enb[2] &= ~(7 << 12);

	wke_write_wake_levels(wake_level);
	wke_write_wake_masks(wake_enb);
	wke_write_tier2_routing(wake_enb);

	return 0;
}

static struct syscore_ops pm_irq_ops = {
	.suspend = tegra_pm_irq_suspend,
	.resume = tegra_pm_irq_resume,
	.save = tegra_pm_irq_suspend,
	.restore = tegra_pm_irq_resume,
};

static int tegra_pm_irq_init(void)
{
	register_syscore_ops(&pm_irq_ops);
	return 0;
}
subsys_initcall(tegra_pm_irq_init);

#ifndef CONFIG_IRQ_DOMAIN_HIERARCHY
static int pm_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	int i;
	int ret;
	int wake_size;
	int wake_list[WAKE_NR_EVENTS];
	int err = 0;

	tegra_irq_to_wake(d->irq, wake_list, &wake_size);

	for (i = 0; i < wake_size; i++) {
		ret = wke_irq_set_wake_level(wake_list[i], flow_type);
		if (ret < 0) {
			pr_err("Set lp0 wake type=%d fail for irq=%d, wake%d ret=%d\n",
				flow_type, d->irq, wake_list[i], ret);
			if (!err)
				err = ret;
		}
	}

	return err;
}

static int pm_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	int i;
	int ret;
	int wake_size;
	int wake_list[WAKE_NR_EVENTS];
	int err = 0;

	tegra_irq_to_wake(d->irq, wake_list, &wake_size);

	for (i = 0; i < wake_size; i++) {
		/* pmc lp0 wake enable for non-gpio wake sources */
		ret = wke_irq_set_wake(wake_list[i], enable);
		if (ret < 0) {
			pr_err("Failed lp0 wake %s for irq=%d, wake%d ret=%d\n",
				(enable ? "enable" : "disable"), d->irq,
				wake_list[i], ret);
			if (!err)
				err = ret;
		}
	}

	return err;
}
#endif /* CONFIG_IRQ_DOMAIN_HIERARCHY  */

int tegra_pm_irq_set_wake_type(int wake, int flow_type)
{
	return wke_irq_set_wake_level(wake, flow_type);
}

int tegra_pm_irq_set_wake(int wake, int enable)
{
	return wke_irq_set_wake(wake, enable);
}

int __init pm_irq_init(void)
{
	tegra_wakeup_table_init();
	/* Hook into GIC ops */
#ifndef CONFIG_IRQ_DOMAIN_HIERARCHY
	gic_arch_extn.irq_set_type = pm_irq_set_type;
	gic_arch_extn.irq_set_wake = pm_irq_set_wake;
#endif
	return 0;
}
