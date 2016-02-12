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

#include <asm/traps.h>
#include <linux/debugfs.h>
#include <linux/debugfs.h>
#include <linux/cpu_pm.h>
#include <linux/cpu.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/cpu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/tegra-mce.h>
#include <linux/platform/tegra/ari_mca.h>
#include <linux/platform/tegra/tegra18_a57_mca.h>
#include <linux/tegra-soc.h>

static u64 read_cpumerrsr(void)
{
	u64 reg;

	asm volatile("mrs %0, s3_1_c15_c2_2" : "=r" (reg));
	return reg;
}

static void write_cpumerrsr(u64 value)
{
	asm volatile("msr s3_1_c15_c2_2, %0" : "=r" (value));
}

static u64 read_l2ctlr(void)
{
	u64 reg;

	asm volatile("mrs %0, s3_1_c11_c0_2" : "=r" (reg));
	return reg;
}

static int report_serr(int cpu, int uncorrected, int corrected)
{
	u32 error;
	int e;
	u64 *data = NULL;
	mca_cmd_t mca_cmd = {.cmd = MCA_ARI_CMD_REPORT_SERR,
			     .idx = 0x1,
			     .subidx = cpu};

	mca_cmd.inst = uncorrected ? 0x1 : 0;
	mca_cmd.inst |= corrected ? 0x2 : 0;

	e = tegra_mce_read_uncore_mca(mca_cmd, data, &error);
	if (e != 0) {
		pr_err("%s: ARI call failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static struct a57_ramid ramids[] = {
	{.name = "Instruction L1 Tag RAM", .id = 0x00},
	{.name = "Instruction L1 Data RAM", .id = 0x01},
	{.name = "Data L1 Tag RAM", .id = 0x08},
	{.name = "Data L1 Data RAM", .id = 0x09},
	{.name = "L2 TLB RAM", .id = 0x18},
	{}
};

static void print_ramid(u64 ramid)
{
	u64 i;
	int found = 0;

	for (i = 0; ramids[i].name; i++) {
		if (ramids[i].id == ramid) {
			pr_crit("\t%s: 0x%llx\n", ramids[i].name, ramid);
			found = 1;
			break;
		}
	}

	if (!found)
		pr_crit("\tUnknown RAMID: 0x%llx\n", ramid);
}

static void print_a57_cpumerrsr(int cpu, int cpuid, u64 syndrome)
{
	pr_crit("**************************************\n");
	pr_crit("A57 CPU Memory Error on CPU %d (A57 Core %d)\n",
		cpu, cpuid);
	if (syndrome & A57_CPUMERRSR_FATAL)
		pr_crit("\tFATAL Error\n");
	print_ramid(get_a57_cpumerrsr_ramid(syndrome));
	pr_crit("\tBank/Way of Error: 0x%llx\n",
		get_a57_cpumerrsr_bank(syndrome));
	pr_crit("\tIndex of Error: 0x%llx\n",
		get_a57_cpumerrsr_index(syndrome));
	pr_crit("\tRepeat Error Count: %lld\n",
		get_a57_cpumerrsr_repeat(syndrome));
	pr_crit("\tOther Error Count: %lld\n",
		get_a57_cpumerrsr_other(syndrome));
	pr_crit("**************************************\n");
}

static DEFINE_RAW_SPINLOCK(a57_mca_lock);

static int a57_serr_hook(struct pt_regs *regs, int reason,
			 unsigned int esr, void *priv)
{
	u64 syndrome;
	int cpu;
	int cpuid;
	u64 mpidr;
	int corrected;
	unsigned long flags;
	struct cpuinfo_arm64 *cpuinfo;

	raw_spin_lock_irqsave(&a57_mca_lock, flags);
	cpu = smp_processor_id();
	cpuinfo = &per_cpu(cpu_data, cpu);

	if (MIDR_PARTNUM(cpuinfo->reg_midr) == ARM_CPU_PART_CORTEX_A57) {
		syndrome = read_cpumerrsr();
		if (syndrome & A57_CPUMERRSR_VALID) {
			mpidr = read_cpuid_mpidr();
			cpuid = (int)MCA_ARI_EXTRACT(mpidr, 1, 0);
			print_a57_cpumerrsr(cpu, cpuid, syndrome);
			corrected = (get_a57_cpumerrsr_other(syndrome) != 0) ||
				    (get_a57_cpumerrsr_repeat(syndrome) != 0);
			report_serr(cpuid,
				    (syndrome & A57_CPUMERRSR_FATAL) == 0ULL,
				    corrected);
			write_cpumerrsr(0x0ULL);
		}
	}

	raw_spin_unlock_irqrestore(&a57_mca_lock, flags);
	return 0;	/* Not handled */
}

static struct serr_hook hook = {
	.fn = a57_serr_hook
};

static int __init tegra18_a57_serr_init(void)
{
	int cpu;
	u32 ecc_settings;
	unsigned long flags;
	struct cpuinfo_arm64 *cpuinfo;
	char *core_type	= "Denver";

	/*
	 * No point in registering an ECC error handler on the
	 * simulator.
	 */
	if (tegra_cpu_is_asim())
		return 0;

	raw_spin_lock_irqsave(&a57_mca_lock, flags);
	cpu = smp_processor_id();
	cpuinfo = &per_cpu(cpu_data, cpu);

	if (MIDR_PARTNUM(cpuinfo->reg_midr) == ARM_CPU_PART_CORTEX_A57) {
		ecc_settings = read_l2ctlr();
		pr_info("**** A57 ECC: %s\n",
			(ecc_settings & A57_L2CTLR_ECC_EN) ? "Enabled" :
			"Disabled");
		core_type = "A57";
	}

	pr_info("%s: on CPU %d a %s Core\n", __func__, cpu, core_type);

	register_serr_hook(&hook);
	raw_spin_unlock_irqrestore(&a57_mca_lock, flags);
	return 0;
}
module_init(tegra18_a57_serr_init);

static void __exit tegra18_a57_serr_exit(void)
{
	unregister_serr_hook(&hook);
}
module_exit(tegra18_a57_serr_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Tegra A57 SError handler");
