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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/tegra-timer.h>
#include <linux/tegra-pm.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-mce.h>
#include <trace/events/power.h>

#include <asm/suspend.h>

#define TEGRA186_CPUIDLE_C6		0
#define TEGRA186_CPUIDLE_C7		1
#define TEGRA186_CPUIDLE_SC7		2
#define TEGRA186_CPUIDLE_SC7_IDLE	3

static u64 resume_time;
static u64 resume_entry_time;
static u64 suspend_time;
static u64 suspend_entry_time;

void tegra_log_suspend_time(void)
{
	suspend_entry_time = arch_timer_read_counter();
}

void tegra_log_resume_time(void)
{
	resume_time = arch_timer_read_counter() - resume_entry_time;
}

static void tegra_get_suspend_time(void)
{
	suspend_time = arch_timer_read_counter() - suspend_entry_time;
}

static int tegra186_suspend_enter(suspend_state_t state)
{
	trace_cpu_suspend(CPU_SUSPEND_START, arch_timer_read_counter());
	tegra_get_suspend_time();

	system_suspend();

	resume_entry_time = arch_timer_read_counter();
	trace_cpu_suspend(CPU_SUSPEND_DONE, resume_entry_time);

	return 0;
}

static int tegra186_suspend_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

static const struct platform_suspend_ops tegra186_suspend_ops = {
	.enter = tegra186_suspend_enter,
	.valid = tegra186_suspend_valid,
};

static __init int tegra186_suspend_init(void)
{
	suspend_set_ops(&tegra186_suspend_ops);
	return 0;
}
core_initcall(tegra186_suspend_init);
