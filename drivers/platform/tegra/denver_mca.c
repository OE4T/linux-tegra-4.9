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

#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/traps.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/module.h>
#include <linux/platform/tegra/denver_mca.h>

static int mca_trip(void *data, u64 val)
{
	unsigned long flags, mca_enable;

	flags = arch_local_save_flags();

	/* Print some debug information */
	pr_crit("%s: DAIF = 0x%lx\n", __func__, flags);
	if (flags & 0x4) {
		pr_crit("%s: \"A\" not set", __func__);
		return 0;
	}
	asm volatile("mrs %0, s3_0_c15_c3_2" : "=r" (mca_enable) : : );
	pr_crit("%s: s3_0_c15_c3_2 = 0x%lx\n", __func__, mca_enable);
	if (!(mca_enable & 1)) {
		pr_crit("%s: s3_0_c15_c3_2,not set", __func__);
		return 0;
	}
	asm volatile("mrs %0, s3_0_c15_c4_6" : "=r" (mca_enable) : : );
	pr_crit("%s: s3_0_c15_c4_6 = 0x%lx\n", __func__, mca_enable);

	/* Do the actual MCA trip */
	pr_crit("msr s3_0_c15_c4_7, 0x%llx\n", val);
	asm volatile("msr s3_0_c15_c4_7, %0" : : "r" (val));
	return 0;
}

/* This will return the special MCA value to be writen back to the node to trip
   an MCA error for debug purposes */
static int mca_trip_null_get(void *data, u64 *val)
{
	*val = 0xb400000000000404UL;
	return 0;
}

static int mca_trip_open(struct inode *inode, struct file *file)
{
	return simple_attr_open(inode, file, mca_trip_null_get, mca_trip,
				"0x%08lx");
}

static const struct file_operations fops_mca_trip = {
	.read =		simple_attr_read,
	.write =	simple_attr_write,
	.open =		mca_trip_open,
	.llseek =	noop_llseek,
};

/* MCA bank handling functions */

#define SERRi_STATUS_VAL	(1ULL << 63)
#define SERRi_STATUS_OVF	(1ULL << 62)
#define SERRi_STATUS_UC		(1ULL << 61)
#define SERRi_STATUS_EN		(1ULL << 60)
#define SERRi_STATUS_MV		(1ULL << 59)
#define SERRi_STATUS_AV		(1ULL << 58)
#define SERRi_STATUS_ERROR_CODE	0xffffULL

static void print_bank(struct denver_mca_bank *mca_bank, u64 status)
{
	struct denver_mca_error *errors;
	u64 msc1, msc2, addr;
	u16 error;
	u64 i;
	int found = 0;

	pr_crit("**************************************");
	pr_crit("Machine check error in %s:\n", mca_bank->name);
	pr_crit("\tStatus = 0x%llx\n", status);

	/* Find the name of known errors */
	error = status & SERRi_STATUS_ERROR_CODE;
	errors = mca_bank->errors;
	if (errors) {
		for (i = 0; errors[i].name; i++) {
			if (errors[i].error_code  == error) {
				pr_crit("\t%s: 0x%x\n", errors[i].name, error);
				found = 1;
				break;
			}
		}
		if (!found)
			pr_crit("\tUnknown error: 0x%x\n", error);
	} else {
		pr_crit("\tBank does not have any known errors\n");
	}

	if (status & SERRi_STATUS_OVF)
		pr_crit("\tOverflow (there may be more errors)\n");
	if (status & SERRi_STATUS_UC)
		pr_crit("\tUncorrected (this is fatal)\n");
	else
		pr_crit("\tCorrectable (but, not corrected)\n");
	if (status & SERRi_STATUS_EN)
		pr_crit("\tError reporting enabled when error arrived\n");
	else
		pr_crit("\tError reporting not enabled when error arrived\n");
	if (status & SERRi_STATUS_MV) {
		msc1 = mca_bank->msc1();
		msc2 = mca_bank->msc2();
		pr_crit("\tMSC1 = 0x%llx\n", msc1);
		pr_crit("\tMSC2 = 0x%llx\n", msc2);
	}
	if (status & SERRi_STATUS_AV) {
		addr = mca_bank->addr();
		pr_crit("\tADDR = 0x%llx\n", addr);
	}
	pr_crit("**************************************");
}

static LIST_HEAD(denver_mca_list);
static DEFINE_RAW_SPINLOCK(denver_mca_lock);

void register_denver_mca_bank(struct denver_mca_bank *bank)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&denver_mca_lock, flags);
	list_add(&bank->node, &denver_mca_list);
	raw_spin_unlock_irqrestore(&denver_mca_lock, flags);
}
EXPORT_SYMBOL(register_denver_mca_bank);

void unregister_denver_mca_bank(struct denver_mca_bank *bank)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&denver_mca_lock, flags);
	list_del(&bank->node);
	raw_spin_unlock_irqrestore(&denver_mca_lock, flags);
}
EXPORT_SYMBOL(unregister_denver_mca_bank);

/* MCA assert register dump */
static int denver_mca_handler(void)
{
	u64 status;
	u64 bank_count;
	struct denver_mca_bank *bank;
	unsigned long flags;

	/* Ask the hardware how many banks exist */
	asm volatile("mrs %0, s3_0_c15_c3_0" : "=r" (bank_count) : );
	bank_count &= 0xff;

	/* Iterate through the banks looking for one with an error */
	raw_spin_lock_irqsave(&denver_mca_lock, flags);
	list_for_each_entry(bank, &denver_mca_list, node) {
		if (bank->bank <= bank_count) {
			status = bank->stat();
			if (status & SERRi_STATUS_VAL)
				print_bank(bank, status);
		}
	}
	raw_spin_unlock_irqrestore(&denver_mca_lock, flags);
	return 0;	/* Not handled */
}

/* Handle SError for Denver cores */
static int denver_serr_hook(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	u64 serr_status;
	int ret;

	/* Check that this is a Denver CPU */
	if (read_cpuid_implementor() != ARM_CPU_IMP_NVIDIA)
		return 0;

	asm volatile("mrs %0, s3_0_c15_c3_1" : "=r" (serr_status));
	if (serr_status & 4) {
		ret = denver_mca_handler();
		serr_status = 0;
		asm volatile("msr s3_0_c15_c3_1, %0" : : "r" (serr_status));
		return ret;
	}
	return 0;	/* Not handled */
}

static struct serr_hook hook = {
	.fn = denver_serr_hook
};

/* Hotplug callback to enable Denver MCA every time the core comes online */
static void denver_setup_mca(void *info)
{
	unsigned long serr_ctl_enable = 1;

	asm volatile("msr s3_0_c15_c3_2, %0" : : "r" (serr_ctl_enable));
}

static int denver_mca_setup_callback(struct notifier_block *nfb,
				     unsigned long action, void *hcpu)
{
	int cpu = (int)(uintptr_t) hcpu;
	struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu);

	if ((action == CPU_ONLINE || action == CPU_ONLINE_FROZEN) &&
		(MIDR_IMPLEMENTOR(cpuinfo->reg_midr) == ARM_CPU_IMP_NVIDIA))
	{
		smp_call_function_single(cpu, denver_setup_mca, NULL, 1);
	}
	return 0;
}

static struct notifier_block denver_mca_notifier = {
	.notifier_call = denver_mca_setup_callback
};

static struct dentry *debugfs_dir;
static struct dentry *debugfs_node;

static int __init denver_serr_init(void)
{
	int cpu;
	struct cpuinfo_arm64 *cpuinfo;

	/* Register the SError hook so that this driver is called on SError */
	register_serr_hook(&hook);

	/* Ensure that any CPU brough online sets up MCA */
	register_hotcpu_notifier(&denver_mca_notifier);

	/* Enable MCA on all online CPUs */
	for_each_online_cpu(cpu) {
		/* Skip non-Denver CPUs */
		cpuinfo = &per_cpu(cpu_data, cpu);
		if (MIDR_IMPLEMENTOR(cpuinfo->reg_midr) != ARM_CPU_IMP_NVIDIA)
			continue;
		smp_call_function_single(cpu, denver_setup_mca, NULL, 1);
	}

	/* Install debugfs nodes to test the MCA behavior */
	debugfs_dir = debugfs_create_dir("denver_mca", NULL);
	if (!debugfs_dir) {
		pr_err("Error creating tegra_mca debugfs dir.\n");
		return -ENODEV;
	}
	debugfs_node = debugfs_create_file("mca_trip", 0600, debugfs_dir, NULL,
					   &fops_mca_trip);
	if (!debugfs_node) {
		pr_err("Error creating mca_trip debugfs node.\n");
		return -ENODEV;
	}

	return 0;
}

module_init(denver_serr_init);

static void __exit denver_serr_exit(void)
{
	debugfs_remove_recursive(debugfs_dir);
	unregister_serr_hook(&hook);
	unregister_hotcpu_notifier(&denver_mca_notifier);
}
module_exit(denver_serr_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Denver Machine Check / SError handler");
