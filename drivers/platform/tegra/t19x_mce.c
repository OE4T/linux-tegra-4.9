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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <linux/t19x_mce.h>
#include <soc/tegra/chip-id.h>
#include <linux/t194_nvg.h>
#include <linux/module.h>
#include <linux/platform/tegra/tegra-cpu.h>

#include <asm/smp_plat.h>

#define SMC_SIP_INVOKE_MCE	0xC2FFFF00

#define NR_SMC_REGS		6

/* MCE command enums for SMC calls */
enum {
	MCE_SMC_ENTER_CSTATE = 0,
	MCE_SMC_UPDATE_CSTATE_INFO = 1,
	MCE_SMC_UPDATE_XOVER_TIME = 2,
	MCE_SMC_READ_CSTATE_STATS = 3,
	MCE_SMC_WRITE_CSTATE_STATS = 4,
	MCE_SMC_IS_SC7_ALLOWED = 5,
	MCE_SMC_ONLINE_CORE = 6,
	MCE_SMC_CC3_CTRL = 7,
	MCE_SMC_ECHO_DATA = 8,
	MCE_SMC_READ_VERSIONS = 9,
	MCE_SMC_ENUM_FEATURES = 10,
	MCE_SMC_ROC_FLUSH_CACHE = 11,
	MCE_SMC_ENUM_READ_MCA = 12,
	MCE_SMC_ENUM_WRITE_MCA = 13,
	MCE_SMC_ROC_FLUSH_CACHE_ONLY = 14,
	MCE_SMC_ROC_CLEAN_CACHE_ONLY = 15,
	MCE_SMC_ENABLE_LATIC = 16,
	MCE_SMC_UNCORE_PERFMON_REQ = 17,
	MCE_SMC_MISC_CCPLEX = 18,
	MCE_SMC_ENUM_MAX = 0xFF,	/* enums cannot exceed this value */
};

struct mce_regs {
	u64 args[NR_SMC_REGS];
};

static noinline notrace int __send_smc(u8 func, struct mce_regs *regs)
{
	u32 ret = SMC_SIP_INVOKE_MCE | (func & MCE_SMC_ENUM_MAX);

	asm volatile (
	"	mov	x0, %0\n"
	"	ldp	x1, x2, [%1, #16 * 0]\n"
	"	ldp	x3, x4, [%1, #16 * 1]\n"
	"	ldp	x5, x6, [%1, #16 * 2]\n"
	"	isb\n"
	"	smc	#0\n"
	"	mov	%0, x0\n"
	"	stp	x0, x1, [%1, #16 * 0]\n"
	"	stp	x2, x3, [%1, #16 * 1]\n"
	: "+r" (ret)
	: "r" (regs)
	: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8",
	"x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	return ret;
}

#define send_smc(func, regs) \
({ \
	int __ret = __send_smc(func, regs); \
	if (__ret) { \
		pr_err("%s: failed (ret=%d)\n", __func__, __ret); \
		return __ret; \
	} \
	__ret; \
})

/**
 * Specify power state and wake time for entering upon STANDBYWFI
 *
 * @state:		requested core power state
 * @wake_time:	wake time in TSC ticks
 *
 * Returns 0 if success.
 */
int t19x_mce_enter_cstate(u32 state, u32 wake_time)
{
	struct mce_regs regs;

	regs.args[0] = state;
	regs.args[1] = wake_time;
	return send_smc(MCE_SMC_ENTER_CSTATE, &regs);
}
EXPORT_SYMBOL(t19x_mce_enter_cstate);

/**
 * Specify deepest cluster/ccplex/system states allowed.
 *
 * @cluster:	deepest cluster-wide state
 * @ccplex:		deepest ccplex-wide state
 * @system:		deepest system-wide state
 * @force:		forced system state
 * @wake_mask:	wake mask to be updated
 * @valid:		is wake_mask applicable?
 *
 * Returns 0 if success.
 */
int t19x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
	u8 force, u32 wake_mask, bool valid)
{
	struct mce_regs regs;

	regs.args[0] = cluster;
	regs.args[1] = ccplex;
	regs.args[2] = system;
	regs.args[3] = force;
	regs.args[4] = wake_mask;
	regs.args[5] = valid;
	return send_smc(MCE_SMC_UPDATE_CSTATE_INFO, &regs);
}
EXPORT_SYMBOL(t19x_mce_update_cstate_info);

/**
 * Update threshold for one specific c-state crossover
 *
 * @type: type of state crossover.
 * @time: idle time threshold.
 *
 * Returns 0 if success.
 */
int t19x_mce_update_crossover_time(u32 type, u32 time)
{
	struct mce_regs regs;

	regs.args[0] = type;
	regs.args[1] = time;
	return send_smc(MCE_SMC_UPDATE_XOVER_TIME, &regs);
}
EXPORT_SYMBOL(t19x_mce_update_crossover_time);

/**
 * Query the runtime stats of a specific cstate
 *
 * @state: c-state of the stats.
 * @stats: output integer to hold the stats.
 *
 * Returns 0 if success.
 */
int t19x_mce_read_cstate_stats(u32 state, u32 *stats)
{
	struct mce_regs regs;

	regs.args[0] = state;
	send_smc(MCE_SMC_WRITE_CSTATE_STATS, &regs);
	send_smc(MCE_SMC_READ_CSTATE_STATS, &regs);
	*stats = (u32)regs.args[2];
	return 0;
}
EXPORT_SYMBOL(t19x_mce_read_cstate_stats);

/**
 * Bring another offlined core back online to C0 state.
 *
 * @cpu:		logical cpuid from smp_processor_id()
 *
 * Returns 0 if success.
 */
int t19x_mce_online_core(int cpu)
{
	struct mce_regs regs;

	regs.args[0] = cpu_logical_map(cpu);
	return send_smc(MCE_SMC_ONLINE_CORE, &regs);
}
EXPORT_SYMBOL(t19x_mce_online_core);

/**
 * Program Auto-CC3 feature.
 *
 * @ndiv:		ndiv of IDLE freq register
 * @enable:		enable bit for Auto-CC3
 *
 * Returns 0 if success.
 */
int t19x_mce_cc3_ctrl(u32 ndiv, u8 enable)
{
	struct mce_regs regs;

	regs.args[0] = ndiv;
	regs.args[1] = enable;
	return send_smc(MCE_SMC_CC3_CTRL, &regs);
}
EXPORT_SYMBOL(t19x_mce_cc3_ctrl);

/**
 * Read out MCE API major/minor versions
 *
 * @major: output for major number.
 * @minor: output for minor number.
 *
 * Returns 0 if success.
 */
int t19x_mce_read_versions(u32 *major, u32 *minor)
{
	struct mce_regs regs;

	send_smc(MCE_SMC_READ_VERSIONS, &regs);
	*major = (u32)regs.args[1];
	*minor = (u32)regs.args[2];
	return 0;
}
EXPORT_SYMBOL(t19x_mce_read_versions);

/**
 * Enumerate MCE API features
 *
 * @features: output feature vector (4bits each)
 *
 * Returns 0 if success.
 */
int t19x_mce_enum_features(u64 *features)
{
	struct mce_regs regs;

	send_smc(MCE_SMC_ENUM_FEATURES, &regs);
	*features = (u32)regs.args[1];
	return 0;
}
EXPORT_SYMBOL(t19x_mce_enum_features);

#ifdef CONFIG_DEBUG_FS

#define CSTAT_ENTRY(stat) NVG_STAT_QUERY_##stat

static struct cstats_info cstats_table[] = {
	{ "SC7_ENTRIES", CSTAT_ENTRY(SC7_ENTRIES), 1},
	{ "SC7_RESIDENCY_SUM", CSTAT_ENTRY(SC7_RESIDENCY_SUM), 1},
	{ "CG7_ENTRIES", CSTAT_ENTRY(CG7_ENTRIES), 2},
	{ "CG7_RESIDENCY_SUM", CSTAT_ENTRY(CG7_RESIDENCY_SUM), 2},
	{ "CC6_ENTRIES", CSTAT_ENTRY(CC6_ENTRIES), 4},
	{ "CC6_RESIDENCY_SUM", CSTAT_ENTRY(CC6_RESIDENCY_SUM), 4},
	{ "C7_ENTRIES", CSTAT_ENTRY(C7_ENTRIES), 8},
	{ "C7_RESIDENCY_SUM", CSTAT_ENTRY(C7_RESIDENCY_SUM), 8},
	{ "C6_ENTRIES", CSTAT_ENTRY(C6_ENTRIES), 8},
	{ "C6_RESIDENCY_SUM", CSTAT_ENTRY(C6_RESIDENCY_SUM), 8},
};

static int mce_versions_get(void *data, u64 *val)
{
	u32 major, minor;
	int ret = t19x_mce_read_versions(&major, &minor);

	if (!ret)
		*val = ((u64)major << 32) | minor;
	return ret;
}

static int mce_features_get(void *data, u64 *val)
{
	return t19x_mce_enum_features(val);
}

static int mce_dbg_cstats_show(struct seq_file *s, void *data)
{
	int st, unit;
	u32 val, mce_index;

	seq_printf(s, "%-25s%-15s%-10s\n", "name", "unit-id", "count/time");
	seq_printf(s, "---------------------------------------------------\n");
	for (st = 0; st < NVG_STAT_MAX_ENTRIES; st++) {
		for (unit = 0; unit < cstats_table[st].units; unit++) {
			mce_index = ((u32)cstats_table[st].id <<
					MCE_STAT_ID_SHIFT) + (u32)unit;
			if (t19x_mce_read_cstate_stats(mce_index, &val))
				pr_err("mce: failed to read cstat: %s, %x\n",
					cstats_table[st].name, mce_index);
			else
				seq_printf(s, "%-25s%-15d%-10d\n",
					cstats_table[st].name, unit, val);
		}
	}
	return 0;
}

static int mce_dbg_cstats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mce_dbg_cstats_show, inode->i_private);
}

static const struct file_operations mce_cstats_fops = {
	.open = mce_dbg_cstats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

DEFINE_SIMPLE_ATTRIBUTE(mce_versions_fops, mce_versions_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mce_features_fops, mce_features_get, NULL, "%llu\n");

static struct dentry *mce_debugfs_root;

struct debugfs_entry {
	const char *name;
	const struct file_operations *fops;
	mode_t mode;
};

static struct debugfs_entry mce_dbg_attrs[] = {
	{ "versions", &mce_versions_fops, S_IRUGO },
	{ "features", &mce_features_fops, S_IRUGO },
	{ "cstats", &mce_cstats_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static __init int t19x_mce_debugfs_init(void)
{
	struct dentry *dent;
	struct debugfs_entry *fent;

	if (!tegra_is_cpu_carmel(smp_processor_id()))
		return -ENODEV;

	mce_debugfs_root = debugfs_create_dir("tegra_mce", NULL);
	if (!mce_debugfs_root)
		return -ENOMEM;

	fent = mce_dbg_attrs;
	while (fent->name) {
		dent = debugfs_create_file(fent->name, fent->mode,
			mce_debugfs_root, NULL, fent->fops);
		if (IS_ERR_OR_NULL(dent))
			goto abort;
		fent++;
	}

	return 0;

abort:
	debugfs_remove_recursive(mce_debugfs_root);
	return -EFAULT;
}

fs_initcall(t19x_mce_debugfs_init);

#endif
