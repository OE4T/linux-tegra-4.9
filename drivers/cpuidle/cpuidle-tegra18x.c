/*
 * drivers/cpuidle/cpuidle-tegra18x.c
 *
 * Copyright (C) 2015-2016 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-mce.h>
#include <linux/tegra186-pm.h>
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

#include <asm/cpuidle.h>
#include <asm/suspend.h>
#include <asm/cputype.h> /* cpuid */
#include <asm/cpu.h>
#include "../../drivers/cpuidle/dt_idle_states.h"

#define PSCI_STATE_ID_STATE_MASK        (0xf)
#define PSCI_STATE_ID_WKTIM_MASK        (~0xf)
#define PSCI_STATE_TYPE_SHIFT           3
#define A57_CORE_WAKE_MASK		0x180C
#define DENVER_CORE_WAKE_MASK		0x180C
#define TSC_PER_SEC			32768
#define NSEC_PER_TSC_TICK		30518
#define USEC_PER_TSC_TICK		30
#define TEGRA186_DENVER_CPUIDLE_C7	2
#define TEGRA186_DENVER_CPUIDLE_C6	1
#define TEGRA186_A57_CPUIDLE_C7		1

static bool check_mce_version(void)
{
	u32 mce_version_major, mce_version_minor;

	tegra_mce_read_versions(&mce_version_major, &mce_version_minor);
	if (mce_version_major >= 2)
		return true;
	else
		return false;
}
static void tegra186_denver_enter_c6(u32 wake_time)
{
	tegra_mce_update_cstate_info(0, 0, 0, 0, DENVER_CORE_WAKE_MASK, 1);
	tegra_mce_enter_cstate(TEGRA186_CPUIDLE_C6, wake_time);
	asm volatile("wfi\n");
}

static void tegra186_denver_enter_c7(u32 wake_time)
{
	tegra_mce_update_cstate_info(0, 0, 0, 0, DENVER_CORE_WAKE_MASK, 1);
	/* Block all interrupts in the cpu core */
	local_irq_disable();
	local_fiq_disable();
	cpu_pm_enter();  /* power down notifier */
	arm_cpuidle_suspend(TEGRA186_DENVER_CPUIDLE_C7);
	cpu_pm_exit();
	local_fiq_enable();
	local_irq_enable();
}

static void tegra186_a57_enter_c7(u32 wake_time)
{
	/* Set wake mask */
	tegra_mce_update_cstate_info(0, 0, 0, 0, A57_CORE_WAKE_MASK, 1);
	cpu_pm_enter();  /* power down notifier */
	arm_cpuidle_suspend(TEGRA186_A57_CPUIDLE_C7);
	cpu_pm_exit();
}

static int t18x_denver_enter_state(
		struct cpuidle_device *dev,
		struct cpuidle_driver *drv,
		int index)
{
	u32 wake_time;
	struct timespec t;
	/* Todo: Based on future need, we might need the var latency_req. */
	/* int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);*/

	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	wake_time = t.tv_sec * TSC_PER_SEC + t.tv_nsec / NSEC_PER_TSC_TICK;

	/* Todo: Based on the Latency number reprogram deepest */
	/*       CC state allowed if needed*/

	/* Todo: inform MCE of cstate only if wake time */
	/*       is higher than CCP3 crossover threshold.    */

	if (tegra_cpu_is_asim()) {
		asm volatile("wfi\n");
		return index;
	}

	if (index == TEGRA186_DENVER_CPUIDLE_C7)
		tegra186_denver_enter_c7(wake_time);
	else if (index == TEGRA186_DENVER_CPUIDLE_C6)
		tegra186_denver_enter_c6(wake_time);
	else
		asm volatile("wfi\n");

	/* Todo: can we query the last entered state from mce */
	/*       to update the return value?                  */
	return index;
}

static int t18x_a57_enter_state(
		struct cpuidle_device *dev,
		struct cpuidle_driver *drv,
		int index)
{
	u32 wake_time;
	struct timespec t;
	/* Todo: Based on future need, we might need the var latency_req. */
	/* int latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);*/

	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	wake_time = t.tv_sec * TSC_PER_SEC + t.tv_nsec / NSEC_PER_TSC_TICK;

	/* Todo: Based on the Latency number reprogram deepest */
	/*       CC state allowed if needed*/

	/* Todo: inform MCE of cstate only if wake time */
	/*       is higher than CCP3 crossover threshold.    */

	if (tegra_cpu_is_asim()) {
		asm volatile("wfi\n");
		return index;
	}

	if (index == TEGRA186_A57_CPUIDLE_C7) {
		tegra186_a57_enter_c7(wake_time);
	} else
		asm volatile("wfi\n");

	return index;
}

static u32 t18x_make_power_state(u32 state)
{
	u32 wake_time;
	struct timespec t;

	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	wake_time = t.tv_sec * TSC_PER_SEC + t.tv_nsec / NSEC_PER_TSC_TICK;
	state = state | (wake_time << 4);

	return state;
}

static struct cpuidle_driver t18x_denver_idle_driver = {
	.name = "tegra18x_idle_denver",
	.owner = THIS_MODULE,
	/*
	 * State at index 0 is standby wfi and considered standard
	 * on all ARM platforms. If in some platforms simple wfi
	 * can't be used as "state 0", DT bindings must be implemented
	 * to work around this issue and allow installing a special
	 * handler for idle state index 0.
	 */
	.states[0] = {
		.enter			= t18x_denver_enter_state,
		.exit_latency		= 1,
		.target_residency	= 1,
		.power_usage		= UINT_MAX,
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C1",
		.desc			= "c1-cpu-clockgated",
	}
};

static struct cpuidle_driver t18x_a57_idle_driver = {
	.name = "tegra18x_idle_a57",
	.owner = THIS_MODULE,
	.states[0] = {
                .enter                  = t18x_a57_enter_state,
                .exit_latency           = 1,
                .target_residency       = 1,
                .power_usage            = UINT_MAX,
                .flags                  = CPUIDLE_FLAG_TIME_VALID,
                .name                   = "C1",
                .desc                   = "c1-cpu-clockgated",
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
                __disable_irq(desc, irq);
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
                __enable_irq(desc, irq);
                raw_spin_unlock_irqrestore(&desc->lock, flags);
        }
}

static struct dentry *cpuidle_debugfs_denver;
static struct dentry *cpuidle_debugfs_a57;
static u64 denver_idle_state;
static u64 a57_idle_state;
static u64 denver_cluster_idle_state;
static u64 a57_cluster_idle_state;

static int denver_idle_write(void *data, u64 val)
{
        unsigned long timer_interval_us = (ulong)val;
        ktime_t time, interval, sleep;
	u32 pmstate;
	u32 wake_time;

	wake_time = (u32) (val / USEC_PER_TSC_TICK);

        if (denver_idle_state >= t18x_denver_idle_driver.state_count) {
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

	pmstate = denver_idle_state;
	tegra_mce_update_cstate_info(denver_cluster_idle_state,0,0,0,0,0);

	if (pmstate == TEGRA186_DENVER_CPUIDLE_C7)
		tegra186_denver_enter_c7(wake_time);
	else if (pmstate == TEGRA186_DENVER_CPUIDLE_C6)
		tegra186_denver_enter_c6(wake_time);
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

static int a57_idle_write(void *data, u64 val)
{
        unsigned long timer_interval_us = (ulong)val;
        ktime_t time, interval, sleep;
	u32 pmstate;
	u32 wake_time;

	wake_time = (u32) (val / USEC_PER_TSC_TICK);

        if (a57_idle_state >= t18x_a57_idle_driver.state_count) {
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

	pmstate = a57_idle_state;
	tegra_mce_update_cstate_info(a57_cluster_idle_state,0,0,0,0,0);

	if (pmstate == TEGRA186_A57_CPUIDLE_C7)
		tegra186_a57_enter_c7(wake_time);
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

DEFINE_SIMPLE_ATTRIBUTE(duration_us_denver_fops, NULL,
						denver_idle_write, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(duration_us_a57_fops, NULL, a57_idle_write, "%llu\n");

static int cpuidle_debugfs_init(void)
{
	struct dentry *dfs_file;

	cpuidle_debugfs_denver = debugfs_create_dir("cpuidle_denver", NULL);
	if (!cpuidle_debugfs_denver)
		goto err_out;

        dfs_file = debugfs_create_u64("forced_idle_state", 0644,
                        cpuidle_debugfs_denver, &denver_idle_state);

        if (!dfs_file)
                goto err_out;

        dfs_file = debugfs_create_u64("forced_cluster_idle_state", 0644,
                        cpuidle_debugfs_denver, &denver_cluster_idle_state);

        if (!dfs_file)
                goto err_out;

        dfs_file = debugfs_create_file("forced_idle_duration_us", 0644,
                        cpuidle_debugfs_denver, NULL, &duration_us_denver_fops);

        if (!dfs_file)
                goto err_out;


	cpuidle_debugfs_a57 = debugfs_create_dir("cpuidle_a57", NULL);
	if (!cpuidle_debugfs_a57)
		goto err_out;

        dfs_file = debugfs_create_u64("forced_idle_state", 0644,
                        cpuidle_debugfs_a57, &a57_idle_state);

        if (!dfs_file)
                goto err_out;

        dfs_file = debugfs_create_u64("forced_cluster_idle_state", 0644,
                        cpuidle_debugfs_a57, &a57_cluster_idle_state);

        if (!dfs_file)
                goto err_out;

        dfs_file = debugfs_create_file("forced_idle_duration_us", 0644,
                        cpuidle_debugfs_a57, NULL, &duration_us_a57_fops);

        if (!dfs_file)
                goto err_out;

	return 0;

err_out:
	pr_err("%s: Couldn't create debugfs node for cpuidle\n", __func__);
	debugfs_remove_recursive(cpuidle_debugfs_denver);
	debugfs_remove_recursive(cpuidle_debugfs_a57);
	return -ENOMEM;
}

static const struct of_device_id tegra18x_a57_idle_state_match[] = {
	{ .compatible = "nvidia,tegra186-cpuidle-a57",
	  .data = t18x_a57_enter_state },
	{ },
};

static const struct of_device_id tegra18x_denver_idle_state_match[] = {
	{ .compatible = "nvidia,tegra186-cpuidle-denver",
	  .data = t18x_denver_enter_state },
	{ },
};

static void cluster_state_init(void *data)
{
	u32 power = UINT_MAX;
	u32 value, pmstate;
	struct device_node *of_states = (struct device_node *)data;
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
		tegra_mce_update_cstate_info(pmstate, 0, 0, 0, 0, 0);
	}
}

static int tegra18x_cpuidle_probe(struct platform_device *pdev)
{
	int cpu_number;
	struct cpumask denver_cpumask;
	struct cpumask a57_cpumask;
	struct device_node *a57_cluster_states;
	struct device_node *denver_cluster_states;
	int err;

	if (!check_mce_version()) {
		pr_err("cpuidle: failed to register."
			" Incompatible MCE version.\n");
		return -ENODEV;
	}

	cpumask_clear(&denver_cpumask);
	cpumask_clear(&a57_cpumask);

	for_each_online_cpu(cpu_number) {
		struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu_number);
		u32 midr = cpuinfo->reg_midr;
		if (MIDR_IMPLEMENTOR(midr) == ARM_CPU_IMP_ARM)
			cpumask_set_cpu(cpu_number, &a57_cpumask);
		else
			cpumask_set_cpu(cpu_number, &denver_cpumask);

		err = arm_cpuidle_init(cpu_number);
		if (err) {
			pr_err("cpuidle: failed to register cpuidle driver\n");
			return err;
		}
	}

	a57_cluster_states =
		of_find_node_by_name(NULL, "a57_cluster_power_states");
	denver_cluster_states =
		of_find_node_by_name(NULL, "denver_cluster_power_states");

	if (!cpumask_empty(&denver_cpumask)) {
		/* Denver cluster cpuidle init */
		pr_info("cpuidle: Initializing cpuidle driver init for "
				"Denver cluster\n");

		smp_call_function_any(&denver_cpumask, cluster_state_init,
			denver_cluster_states, 1);

		t18x_denver_idle_driver.cpumask = &denver_cpumask;
		err = dt_init_idle_driver(&t18x_denver_idle_driver,
			tegra18x_denver_idle_state_match, 1);
		if (err <=0) {
			pr_err("cpuidle: failed to init idle states for denver\n");
			return err? : -ENODEV;
		}
		err = cpuidle_register(&t18x_denver_idle_driver, NULL);

		if (err) {
			pr_err("%s: failed to init denver cpuidle power"
				" states.\n", __func__);
			return err;
		}
	}

	if (!cpumask_empty(&a57_cpumask)) {
		/* A57 cluster cpuidle init */
		pr_info("cpuidle: Initializing cpuidle driver init for "
				"A57 cluster\n");
		extended_ops.make_power_state = t18x_make_power_state;

		smp_call_function_any(&a57_cpumask, cluster_state_init,
			a57_cluster_states, 1);

		t18x_a57_idle_driver.cpumask = &a57_cpumask;
		err = dt_init_idle_driver(&t18x_a57_idle_driver,
			tegra18x_a57_idle_state_match, 1);
		if (err <=0) {
			pr_err("cpuidle: failed to init idle states for A57\n");
			return err? : -ENODEV;
		}
		err = cpuidle_register(&t18x_a57_idle_driver, NULL);

		if (err) {
			pr_err("%s: failed to init a57 cpuidle power"
				" states.\n", __func__);
			return err;
		}
	}

	cpuidle_debugfs_init();
	return 0;
}

static const struct of_device_id tegra18x_cpuidle_of[] = {
        { .compatible = "nvidia,tegra18x-cpuidle" },
        {}
};

static struct platform_driver tegra18x_cpuidle_driver = {
        .probe = tegra18x_cpuidle_probe,
        .driver = {
                .owner = THIS_MODULE,
                .name = "cpuidle-tegra18x",
                .of_match_table = of_match_ptr(tegra18x_cpuidle_of)
        }
};

module_platform_driver(tegra18x_cpuidle_driver);

struct xover_table {
	char *name;
	int index;
};

static void crossover_init(struct device_node *of_states) {

	struct device_node *child;
	u32 value;
	int i;

	struct xover_table table1[] = {
		{"crossover_c1_c6", TEGRA_MCE_XOVER_C1_C6},
		{"crossover_cc1_cc6", TEGRA_MCE_XOVER_CC1_CC6},
		{"crossover_cc1_cc7", TEGRA_MCE_XOVER_CC1_CC7},
		{"crossover_ccp1_ccp3", TEGRA_MCE_XOVER_CCP1_CCP3},
		{"crossover_ccp3_sc2", TEGRA_MCE_XOVER_CCP3_SC2},
		{"crossover_ccp3_sc3", TEGRA_MCE_XOVER_CCP3_SC3},
		{"crossover_ccp3_sc4", TEGRA_MCE_XOVER_CCP3_SC4},
		{"crossover_ccp3_sc7", TEGRA_MCE_XOVER_CCP3_SC7},
	};

	for_each_child_of_node(of_states, child)
		for (i=0; i< TEGRA_MCE_XOVER_MAX; i++) {
			if (of_property_read_u32(child,
				table1[i].name, &value) == 0)
				tegra_mce_update_crossover_time
					(table1[i].index, value);
	}
}

static int tegra_mce_cpu_notify(struct notifier_block *nb,
        unsigned long action, void *pcpu)
{
	struct device_node *denver_xover;
	struct device_node *a57_xover;
        int cpu = (long)pcpu;

	denver_xover = of_find_node_by_name(NULL, "denver_crossover_thresholds");
	a57_xover = of_find_node_by_name(NULL, "a57_crossover_thresholds");

        switch (action) {
        case CPU_STARTING:
		pr_debug("cpuidle: Init Power Crossover thresholds for core %d\n"
				, cpu);
                if (read_cpuid_implementor() == ARM_CPU_IMP_ARM) {
			if (!a57_xover) {
				pr_err("%s: failed to init xover for core %d\n",
					__func__, cpu);
				break;
			}
			crossover_init(a57_xover);
                } else {
			if (!denver_xover) {
				pr_err("%s: failed to init xover for core %d\n",
					__func__, cpu);
				break;
			}
			crossover_init(denver_xover);
		}
                break;
        }

        return NOTIFY_OK;
}

static struct notifier_block mce_cpu_notifier = {
        .notifier_call = tegra_mce_cpu_notify,
};

static int __init tegra_mce_early_init(void)
{

	if (!check_mce_version()) {
		pr_err("cpuidle: skipping crossover programming."
			" Incompatible MCE version.\n");
		return -ENODEV;
	}

        /* Initialize thresholds for boot cpu now */
        tegra_mce_cpu_notify(NULL, CPU_STARTING, (void *)0);
	/* Initialize thresholds for rest of the cpu when they start */
        register_cpu_notifier(&mce_cpu_notifier);
        return 0;
}
early_initcall(tegra_mce_early_init);
