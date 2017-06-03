/*
 * drivers/cpuidle/cpuidle-tegra19x.c
 *
 * Copyright (C) 2017, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <soc/tegra/chip-id.h>
#include <linux/t19x_mce.h>
#include <linux/t194_nvg.h>
#include <linux/suspend.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/tick.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include "../../kernel/irq/internals.h"
#include <linux/pm_qos.h>
#include <linux/cpu_pm.h>
#include <linux/psci.h>
#include <linux/version.h>

#include <asm/cpuidle.h>
#include <asm/suspend.h>
#include <asm/cputype.h> /* cpuid */
#include <asm/cpu.h>
#include <asm/arch_timer.h>
#include "../../drivers/cpuidle/dt_idle_states.h"
#include "../../kernel/time/tick-internal.h"

#define PSCI_STATE_ID_WKTIM_MASK	(~0xf000000f)
#define PSCI_STATE_ID_WKTIM_SHIFT	4
#define CORE_WAKE_MASK			0x180C
#define T19x_CPUIDLE_C7_STATE		2
#define T19x_CPUIDLE_C6_STATE		1

static u32 read_cluster_info(struct device_node *of_states);
static u32 deepest_cc_state;
static u32 deepest_cg_state;
static u64 forced_idle_state;
static u64 forced_cluster_idle_state;
static u32 testmode;
static struct cpuidle_driver t19x_cpu_idle_driver;
static int crossover_init(void);
static void program_cc_state(void *data);
static void program_cg_state(void *data);
static u32 tsc_per_sec, nsec_per_tsc_tick;
static u32 tsc_per_usec;

#ifdef CPUIDLE_FLAG_TIME_VALID
#define DRIVER_FLAGS		CPUIDLE_FLAG_TIME_VALID
#else
#define DRIVER_FLAGS		0
#endif

static bool check_mce_version(void)
{
	u32 mce_version_major, mce_version_minor;
	int err;

	err = t19x_mce_read_versions(&mce_version_major, &mce_version_minor);
	if (!err && (mce_version_major >= TEGRA_NVG_VERSION_MAJOR))
		return true;
	else
		return false;
}

static void t19x_cpu_enter_c6(u32 wake_time)
{
	cpu_pm_enter();
	t19x_mce_update_cstate_info(0, 0, 0, 0, CORE_WAKE_MASK, 1);
	t19x_mce_enter_cstate(TEGRA_NVG_CORE_C6, wake_time);
	asm volatile("wfi\n");
	cpu_pm_exit();
}

static void t19x_cpu_enter_c7(u32 wake_time)
{
	cpu_pm_enter(); /* power down notifier */
	arm_cpuidle_suspend(T19x_CPUIDLE_C7_STATE);
	cpu_pm_exit();
}

static int t19x_cpu_enter_state(
		struct cpuidle_device *dev,
		struct cpuidle_driver *drv,
		int index)
{
	u32 wake_time;
	struct timespec t;

	if (tegra_platform_is_vdk()) {
		asm volatile("wfi\n");
		return index;
	}

	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	wake_time = t.tv_sec * tsc_per_sec + t.tv_nsec / nsec_per_tsc_tick;

	if (testmode) {
		t19x_mce_update_cstate_info(forced_cluster_idle_state,
				0, 0, 0, 0, 0);
		if (forced_idle_state >= t19x_cpu_idle_driver.state_count) {
			pr_err("%s: Requested invalid forced idle state\n",
				__func__);
			index = t19x_cpu_idle_driver.state_count;
		} else
			index = forced_idle_state;

		wake_time = 0xFFFFEEEE;
	}

	if (index == T19x_CPUIDLE_C7_STATE)
		t19x_cpu_enter_c7(wake_time);
	else if (index == T19x_CPUIDLE_C6_STATE)
		t19x_cpu_enter_c6(wake_time);
	else
		asm volatile("wfi\n");

	return index;
}

static u32 t19x_make_power_state(u32 state)
{
	u32 wake_time;
	struct timespec t;

	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	wake_time = t.tv_sec * tsc_per_sec + t.tv_nsec / nsec_per_tsc_tick;

	if (testmode)
		wake_time = 0xFFFFEEEE;

	/* The 8-LSB bits of wake time is lost and only 24 MSB bits
	of wake time can fit into the additional state id bits */
	state = state | ((wake_time >> PSCI_STATE_ID_WKTIM_SHIFT)
				& PSCI_STATE_ID_WKTIM_MASK);

	return state;
}

static struct cpuidle_driver t19x_cpu_idle_driver = {
	.name = "tegra19x_cpuidle_driver",
	.owner = THIS_MODULE,
	/*
	 * State at index 0 is standby wfi and considered standard
	 * on all ARM platforms. If in some platforms simple wfi
	 * can't be used as "state 0", DT bindings must be implemented
	 * to work around this issue and allow installing a special
	 * handler for idle state index 0.
	 */
	.states[0] = {
		.enter			= t19x_cpu_enter_state,
		.exit_latency		= 1,
		.target_residency	= 1,
		.power_usage		= UINT_MAX,
		.flags			= DRIVER_FLAGS,
		.name			= "C1",
		.desc			= "c1-cpu-clockgated",
	}
};

static bool is_timer_irq(struct irq_desc *desc)
{
	return desc && desc->action && (desc->action->flags & IRQF_TIMER);
}

static void suspend_all_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		/* Don't disable the 'wakeup' interrupt */
		if (is_timer_irq(desc))
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__disable_irq(desc);
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}

	for_each_irq_desc(irq, desc) {
		if (is_timer_irq(desc))
			continue;
		synchronize_irq(irq);
	}
}

static void resume_all_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		if (is_timer_irq(desc))
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__enable_irq(desc);
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

static struct dentry *cpuidle_debugfs_node;

static int forced_idle_write(void *data, u64 val)
{
	unsigned long timer_interval_us = (ulong)val;
	ktime_t time, interval, sleep;
	u32 pmstate;
	u32 wake_time;

	val = (val * 1000) / nsec_per_tsc_tick;
	if (val > 0xffffffff)
		val = 0xffffffff;
	wake_time = val;

	if (forced_idle_state >= t19x_cpu_idle_driver.state_count) {
		pr_err("%s: Requested invalid forced idle state\n", __func__);
		return -EINVAL;
	}

	suspend_all_device_irqs();
	preempt_disable();
	tick_nohz_idle_enter();
	stop_critical_timings();
	local_fiq_disable();
	local_irq_disable();

	interval = ktime_set(0, (NSEC_PER_USEC * timer_interval_us));

	time = ktime_get();
	sleep = ktime_add(time, interval);
	tick_program_event(sleep, true);

	pmstate = forced_idle_state;
	t19x_mce_update_cstate_info(forced_cluster_idle_state, 0, 0, 0, 0, 0);

	if (pmstate == T19x_CPUIDLE_C7_STATE)
		t19x_cpu_enter_c7(wake_time);
	else if (pmstate == T19x_CPUIDLE_C6_STATE)
		t19x_cpu_enter_c6(wake_time);
	else
		asm volatile("wfi\n");

	sleep = ktime_sub(ktime_get(), time);
	time = ktime_sub(sleep, interval);
	trace_printk("idle: %lld, exit latency: %lld\n", sleep.tv64, time.tv64);

	local_irq_enable();
	local_fiq_enable();
	start_critical_timings();
	tick_nohz_idle_exit();
	preempt_enable_no_resched();
	resume_all_device_irqs();

	return 0;
}

struct xover_smp_call_data {
	int index;
	int value;
};

static void program_single_crossover(void *data)
{
	struct xover_smp_call_data *xover_data =
		(struct xover_smp_call_data *)data;
	t19x_mce_update_crossover_time(xover_data->index,
					xover_data->value * tsc_per_usec);
}

static int setup_crossover(int index, int value)
{
	struct xover_smp_call_data xover_data;

	xover_data.index = index;
	xover_data.value = value;

	on_each_cpu_mask(cpu_online_mask, program_single_crossover,
			&xover_data, 1);
	return 0;
}

static int c6_xover_write(void *data, u64 val)
{
	return setup_crossover(TEGRA_NVG_CHANNEL_CROSSOVER_C6_LOWER_BOUND,
		(u32) val);
}

static int cc6_xover_write(void *data, u64 val)
{
	return setup_crossover(TEGRA_NVG_CHANNEL_CROSSOVER_CC6_LOWER_BOUND,
		(u32) val);
}

static int cg7_xover_write(void *data, u64 val)
{
	return setup_crossover(TEGRA_NVG_CHANNEL_CROSSOVER_CG7_LOWER_BOUND,
		(u32) val);
}

static int set_testmode(void *data, u64 val)
{
	testmode = (u32)val;
	if (testmode) {
		setup_crossover(TEGRA_NVG_CHANNEL_CROSSOVER_C6_LOWER_BOUND, 0);
		setup_crossover(TEGRA_NVG_CHANNEL_CROSSOVER_CC6_LOWER_BOUND, 0);
		setup_crossover(TEGRA_NVG_CHANNEL_CROSSOVER_CG7_LOWER_BOUND, 0);
	} else {
		/* Restore the cluster state */
		on_each_cpu_mask(cpu_online_mask,
			program_cc_state, &deepest_cc_state, 1);
		/* Restore the cluster group state */
		on_each_cpu_mask(cpu_online_mask,
			program_cg_state, &deepest_cg_state, 1);
		/* Restore the crossover values */
		crossover_init();
	}
	return 0;
}

static int cc_state_set(void *data, u64 val)
{
	deepest_cc_state = (u32)val;
	on_each_cpu_mask(cpu_online_mask, program_cc_state,
			&deepest_cc_state, 1);
	return 0;
}

static int cc_state_get(void *data, u64 *val)
{
	*val = (u64) deepest_cc_state;
	return 0;
}

static int cg_state_set(void *data, u64 val)
{
	deepest_cg_state = (u32)val;
	on_each_cpu_mask(cpu_online_mask, program_cg_state,
			&deepest_cg_state, 1);
	return 0;
}

static int cg_state_get(void *data, u64 *val)
{
	*val = (u64) deepest_cg_state;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(duration_us_fops, NULL, forced_idle_write, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(xover_c6_fops, NULL, c6_xover_write, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(xover_cc6_fops, NULL,
						cc6_xover_write, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(xover_cg7_fops, NULL,
						cg7_xover_write, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(cc_state_fops, cc_state_get,
						cc_state_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(cg_state_fops, cg_state_get,
						cg_state_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(testmode_fops, NULL, set_testmode, "%llu\n");

static int cpuidle_debugfs_init(void)
{
	struct dentry *dfs_file;

	cpuidle_debugfs_node = debugfs_create_dir("tegra_cpuidle", NULL);
	if (!cpuidle_debugfs_node)
		goto err_out;

	dfs_file = debugfs_create_u64("forced_idle_state", 0644,
		cpuidle_debugfs_node, &forced_idle_state);

	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_u64("forced_cluster_idle_state", 0644,
		cpuidle_debugfs_node, &forced_cluster_idle_state);

	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("forced_idle_duration_us", 0644,
		cpuidle_debugfs_node, NULL, &duration_us_fops);

	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("testmode", 0644,
		cpuidle_debugfs_node, NULL, &testmode_fops);
	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("crossover_c1_c6", 0644,
		cpuidle_debugfs_node, NULL, &xover_c6_fops);
	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("crossover_cc1_cc6", 0644,
		cpuidle_debugfs_node, NULL, &xover_cc6_fops);
	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("crossover_cc1_cg7", 0644,
		cpuidle_debugfs_node, NULL, &xover_cg7_fops);
	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("deepest_cc_state", 0644,
		cpuidle_debugfs_node, NULL, &cc_state_fops);
	if (!dfs_file)
		goto err_out;

	dfs_file = debugfs_create_file("deepest_cg_state", 0644,
		cpuidle_debugfs_node, NULL, &cg_state_fops);
	if (!dfs_file)
		goto err_out;

	return 0;

err_out:
	pr_err("%s: Couldn't create debugfs node for cpuidle\n", __func__);
	debugfs_remove_recursive(cpuidle_debugfs_node);
	return -ENOMEM;
}

static const struct of_device_id t19x_idle_of_match[] = {
	{ .compatible = "nvidia,tegra194-cpuidle-core",
	  .data = t19x_cpu_enter_state },
	{ },
};

static u32 read_cluster_info(struct device_node *of_states)
{
	u32 power = UINT_MAX;
	u32 value, pmstate, deepest_pmstate = 0;
	struct device_node *child;
	int err;

	for_each_child_of_node(of_states, child) {
		if (of_property_match_string(child, "status", "okay"))
			continue;
		err = of_property_read_u32(child, "power", &value);
		if (err) {
			pr_warn(" %s missing power property\n",
				child->full_name);
			continue;
		}
		err = of_property_read_u32(child, "pmstate", &pmstate);
		if (err) {
			pr_warn(" %s missing pmstate property\n",
				child->full_name);
			continue;
		}
		/* Enable the deepest power state */
		if (value > power)
			continue;
		power = value;
		deepest_pmstate = pmstate;
	}
	return deepest_pmstate;
}

struct xover_table {
	char *name;
	int index;
};

static void send_crossover(void *data)
{
	struct device_node *child;
	struct device_node *of_states = (struct device_node *)data;
	u32 value;
	int i;

	struct xover_table table1[] = {
		{"crossover_c1_c6", TEGRA_NVG_CHANNEL_CROSSOVER_C6_LOWER_BOUND},
		{"crossover_cc1_cc6",
			TEGRA_NVG_CHANNEL_CROSSOVER_CC6_LOWER_BOUND},
		{"crossover_cc1_cg7",
			TEGRA_NVG_CHANNEL_CROSSOVER_CG7_LOWER_BOUND},
	};

	for_each_child_of_node(of_states, child)
		for (i = 0; i < sizeof(table1)/sizeof(table1[0]); i++) {
			if (of_property_read_u32(child,
				table1[i].name, &value) == 0)
				t19x_mce_update_crossover_time
					(table1[i].index, value * tsc_per_usec);
	}
}

static int crossover_init(void)
{
	struct device_node *cpu_xover;

	cpu_xover = of_find_node_by_name(NULL,
			"cpu_crossover_thresholds");

	pr_debug("cpuidle: Init Power Crossover thresholds.\n");

	if (!cpu_xover) {
		pr_err("WARNING: cpuidle: %s: DT entry ", __func__);
		pr_err("missing for Crossover thresholds\n");
	} else
		on_each_cpu_mask(cpu_online_mask, send_crossover,
			cpu_xover, 1);

	return 0;
}

static void program_cc_state(void *data)
{
	u32 *cc_state = (u32 *)data;

	t19x_mce_update_cstate_info(*cc_state, 0, 0, 0, 0, 0);
}

static void program_cg_state(void *data)
{
	u32 *cg_state = (u32 *)data;

	t19x_mce_update_cstate_info(0, *cg_state, 0, 0, 0, 0);
}

static int tegra_suspend_notify_callback(struct notifier_block *nb,
	unsigned long action, void *pcpu)
{
	switch (action) {
	case PM_POST_SUSPEND:
	/*
	 * Re-program deepest allowed cluster and cluster group power state
	 * after system resumes from SC7
	 */
		on_each_cpu_mask(cpu_online_mask, program_cc_state,
			&deepest_cc_state, 1);
		on_each_cpu_mask(cpu_online_mask, program_cg_state,
			&deepest_cg_state, 1);
		break;
	}
	return NOTIFY_OK;
}

static int tegra_cpu_notify_callback(struct notifier_block *nb,
	unsigned long action, void *pcpu)
{
	int cpu = (long) pcpu;

	switch (action) {
	case CPU_ONLINE:
	/*
	 * Re-program deepest allowed cluster and cluster group power state
	 * after a core in that cluster is onlined.
	 */
		smp_call_function_single(cpu, program_cc_state,
			&deepest_cc_state, 1);
		smp_call_function_single(cpu, program_cg_state,
			&deepest_cg_state, 1);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block cpu_on_notifier = {
	.notifier_call = tegra_cpu_notify_callback,
};

static struct notifier_block suspend_notifier = {
	.notifier_call = tegra_suspend_notify_callback,
};

static int __init tegra19x_cpuidle_probe(struct platform_device *pdev)
{
	int cpu_number;
	struct device_node *cpu_cc_states, *cpu_cg_states;
	int err;
	struct cpumask *cpumask;

	if (!check_mce_version()) {
		pr_err("cpuidle: Incompatible MCE version. Not registering\n");
		return -ENODEV;
	}

	tsc_per_sec = arch_timer_get_cntfrq();
	nsec_per_tsc_tick = 1000000000/tsc_per_sec;
	tsc_per_usec = tsc_per_sec / 1000000;

	cpumask = kmalloc(sizeof(struct cpumask), GFP_KERNEL);
	cpumask_clear(cpumask);

	for_each_online_cpu(cpu_number) {
		cpumask_set_cpu(cpu_number, cpumask);
		err = arm_cpuidle_init(cpu_number);
		if (err) {
			pr_err("cpuidle: failed to init ops for cpu %d \n",
				cpu_number);
			goto probe_exit;
		}
	}

	crossover_init();

	cpu_cc_states =
		of_find_node_by_name(NULL, "cpu_cluster_power_states");
	cpu_cg_states =
		of_find_node_by_name(NULL, "cpu_cluster_group_power_states");

	pr_info("cpuidle: Initializing cpuidle driver\n");
	extended_ops.make_power_state = t19x_make_power_state;

	/* read cluster state info from DT */
	deepest_cc_state = read_cluster_info(cpu_cc_states);
	on_each_cpu_mask(cpu_online_mask, program_cc_state,
			&deepest_cc_state, 1);
	/* read cluster group info from DT */
	deepest_cg_state = read_cluster_info(cpu_cg_states);
	on_each_cpu_mask(cpu_online_mask, program_cg_state,
			&deepest_cg_state, 1);

	t19x_cpu_idle_driver.cpumask = cpumask;
	err = dt_init_idle_driver(&t19x_cpu_idle_driver,
		t19x_idle_of_match, 1);
	if (err <= 0) {
		pr_err("cpuidle: failed to init idle driver states \n");
		err = -ENODEV;
		goto probe_exit;
	}
	err = cpuidle_register(&t19x_cpu_idle_driver, NULL);

	if (err) {
		pr_err("cpuidle: failed to register cpuidle driver \n");
		goto probe_exit;
	}

	cpuidle_debugfs_init();
	register_cpu_notifier(&cpu_on_notifier);
	register_pm_notifier(&suspend_notifier);
	return 0;

probe_exit:
	kfree(cpumask);
	pr_err("cpuidle: failed to register cpuidle driver\n");
	return err;
}

static int tegra19x_cpuidle_remove(struct platform_device *pdev)
{
	cpuidle_unregister(&t19x_cpu_idle_driver);
	kfree(t19x_cpu_idle_driver.cpumask);
	unregister_cpu_notifier(&cpu_on_notifier);
	unregister_pm_notifier(&suspend_notifier);
	return 0;
}

static const struct of_device_id tegra19x_cpuidle_of[] = {
	{ .compatible = "nvidia,tegra19x-cpuidle" },
	{}
};

static struct platform_driver tegra19x_cpuidle_driver __refdata = {
	.probe	= tegra19x_cpuidle_probe,
	.remove	= tegra19x_cpuidle_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cpuidle-tegra19x",
		.of_match_table = of_match_ptr(tegra19x_cpuidle_of)
	}
};

module_platform_driver(tegra19x_cpuidle_driver);
