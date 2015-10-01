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
#include <linux/tegra-mce.h>
#include <linux/platform/tegra/ari_mca.h>

/* MCA bank handling functions */

#define SERRi_STATUS_VAL	(1ULL << 63)
#define SERRi_STATUS_OVF	(1ULL << 62)
#define SERRi_STATUS_UC		(1ULL << 61)
#define SERRi_STATUS_EN		(1ULL << 60)
#define SERRi_STATUS_MV		(1ULL << 59)
#define SERRi_STATUS_AV		(1ULL << 58)
#define SERRi_STATUS_ERROR_CODE	0xffffULL

enum {
	MCA_ARI_RW_SUBIDX_CTRL = 0,
	MCA_ARI_RW_SUBIDX_STAT = 1,
	MCA_ARI_RW_SUBIDX_ADDR = 2,
	MCA_ARI_RW_SUBIDX_MSC1 = 3,
	MCA_ARI_RW_SUBIDX_MSC2 = 4,
};

static int read_bank_status(struct ari_mca_bank *mca_bank, u64 *data)
{
	u32 error;
	int e;
	mca_cmd_t mca_cmd = {.cmd = 1, .idx = mca_bank->bank,
			     .subidx = MCA_ARI_RW_SUBIDX_STAT};

	e = tegra_mce_read_uncore_mca(mca_cmd, data, &error);
	if (e != 0) {
		pr_err("%s: ARI call failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int read_bank_address(struct ari_mca_bank *mca_bank, u64 *data)
{
	u32 error;
	int e;
	mca_cmd_t mca_cmd = {.cmd = 1, .idx = mca_bank->bank,
			     .subidx = MCA_ARI_RW_SUBIDX_ADDR};

	e = tegra_mce_read_uncore_mca(mca_cmd, data, &error);
	if (e != 0) {
		pr_err("%s: ARI call failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int read_bank_misc1(struct ari_mca_bank *mca_bank, u64 *data)
{
	u32 error;
	int e;
	mca_cmd_t mca_cmd = {.cmd = 1, .idx = mca_bank->bank,
			     .subidx = MCA_ARI_RW_SUBIDX_MSC1};

	e = tegra_mce_read_uncore_mca(mca_cmd, data, &error);
	if (e != 0) {
		pr_err("%s: ARI call failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int read_bank_misc2(struct ari_mca_bank *mca_bank, u64 *data)
{
	u32 error;
	mca_cmd_t mca_cmd = {.cmd = 1, .idx = mca_bank->bank,
			     .subidx = MCA_ARI_RW_SUBIDX_MSC2};

	if (tegra_mce_read_uncore_mca(mca_cmd, data, &error) != 0) {
		pr_err("%s: ARI call failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void print_bank(struct ari_mca_bank *mca_bank, u64 status)
{
	struct ari_mca_error *errors;
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
		if (read_bank_misc1(mca_bank, &msc1) != 0)
			return;
		if (read_bank_misc2(mca_bank, &msc2) != 0)
			return;
		pr_crit("\tMSC1 = 0x%llx\n", msc1);
		pr_crit("\tMSC2 = 0x%llx\n", msc2);
	}
	if (status & SERRi_STATUS_AV) {
		if (read_bank_address(mca_bank, &addr) != 0)
			return;
		pr_crit("\tADDR = 0x%llx\n", addr);
	}
	pr_crit("**************************************");
}

static LIST_HEAD(ari_mca_list);
static DEFINE_RAW_SPINLOCK(ari_mca_lock);

void register_ari_mca_bank(struct ari_mca_bank *bank)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ari_mca_lock, flags);
	list_add(&bank->node, &ari_mca_list);
	raw_spin_unlock_irqrestore(&ari_mca_lock, flags);
}
EXPORT_SYMBOL(register_ari_mca_bank);

void unregister_ari_mca_bank(struct ari_mca_bank *bank)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ari_mca_lock, flags);
	list_del(&bank->node);
	raw_spin_unlock_irqrestore(&ari_mca_lock, flags);
}
EXPORT_SYMBOL(unregister_ari_mca_bank);

/* MCA assert register dump */
static int ari_serr_hook(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	u64 status;
	struct ari_mca_bank *bank;
	unsigned long flags;

	/* Iterate through the banks looking for one with an error */
	raw_spin_lock_irqsave(&ari_mca_lock, flags);
	list_for_each_entry(bank, &ari_mca_list, node) {
		if (read_bank_status(bank, &status) != 0)
			continue;
		if (status & SERRi_STATUS_VAL)
			print_bank(bank, status);
	}
	raw_spin_unlock_irqrestore(&ari_mca_lock, flags);
	return 0;	/* Not handled */
}

static struct serr_hook hook = {
	.fn = ari_serr_hook
};

static int __init ari_serr_init(void)
{
	/* Register the SError hook so that this driver is called on SError */
	register_serr_hook(&hook);

	return 0;
}

module_init(ari_serr_init);

static void __exit ari_serr_exit(void)
{
	unregister_serr_hook(&hook);
}
module_exit(ari_serr_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARI Machine Check / SError handler");
