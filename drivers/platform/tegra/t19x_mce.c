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

/* Issue a NVG request with data */
static noinline notrace uint64_t nvg_send_req_data(uint64_t req, uint64_t data)
{
	uint64_t ret;

	asm volatile ("msr s3_0_c15_c1_2, %0" :: "r" (req));
	asm volatile ("msr s3_0_c15_c1_3, %0" :: "r" (data));
	asm volatile ("mov %0, x0\n" : "=r" (ret));

	return ret;
}

/* Issue a NVG request to read the command response */
static noinline notrace uint64_t nvg_get_response(void)
{
	uint64_t ret;

	asm volatile ("mrs %0, s3_0_c15_c1_3" : "=r" (ret));

	return ret;
}

/**
 * Specify power state and wake time for entering upon STANDBYWFI
 *
 * @state: requested core power state
 * @wake_time: wake time in TSC ticks
 *
 * Returns 0 if success.
 */
int t19x_mce_enter_cstate(u32 state, u32 wake_time)
{
	/* use PSCI interface instead */
	return 0;
}

/**
 * Specify deepest cluster/ccplex/system states allowed.
 *
 * @cluster: deepest cluster-wide state
 * @ccplex: deepest ccplex-wide state
 * @system: deepest system-wide state
 * @force: forced system state
 * @wake_mask: wake mask to be updated
 * @valid: is wake_mask applicable?
 *
 * Returns 0 if success.
 */
int t19x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
	u8 force, u32 wake_mask, bool valid)
{
	nvg_cstate_info_channel_t cstate_info = { 0 };
	uint64_t ret;

	/* disable preemption */
	preempt_disable();

	/* update CLUSTER_CSTATE? */
	if (cluster) {
		cstate_info.bits.cluster_state = cluster;
		cstate_info.bits.update_cluster = 1;
	}

	/* update CCPLEX_CSTATE? */
	if (ccplex) {
		cstate_info.bits.cg_cstate = ccplex;
		cstate_info.bits.update_cg = 1;
	}

	/* update SYSTEM_CSTATE? */
	if (system) {
		cstate_info.bits.system_cstate = system;
		cstate_info.bits.update_system = 1;
	}

	/* update wake mask value? */
	if (valid)
		cstate_info.bits.update_wake_mask = 1;

	/* set the wake mask */
	cstate_info.bits.wake_mask = wake_mask;

	/* set the updated cstate info */
	ret = nvg_send_req_data(TEGRA_NVG_CHANNEL_CSTATE_INFO, cstate_info.flat);

	/* enable preemption */
	preempt_enable();

	if (ret) {
		pr_err("%s failed with error (%lld)\n", __func__, ret);
		return -ENOMSG;
	}

	return 0;
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
	uint64_t ret;

	if ((type != TEGRA_NVG_CHANNEL_CROSSOVER_C6_LOWER_BOUND) &&
	    (type != TEGRA_NVG_CHANNEL_CROSSOVER_CC6_LOWER_BOUND) &&
	    (type != TEGRA_NVG_CHANNEL_CROSSOVER_CG7_LOWER_BOUND)) {
		pr_err("%s: unknown crossover type (%d)\n", __func__, type);
		return -EINVAL;
	}

	/* disable pre-emption*/
	preempt_disable();

	ret = nvg_send_req_data(type, (uint64_t)time);

	/* enable pre-emption */
	preempt_enable();

	if (ret) {
		pr_err("%s failed with error (%lld)\n", __func__, ret);
		return -ENOMSG;
	}

	return 0;
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
	uint64_t ret;

	if (!stats)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	ret = nvg_send_req_data(TEGRA_NVG_CHANNEL_CSTATE_STAT_QUERY_REQUEST,
				(uint64_t)state);
	if (ret) {
		pr_err("%s failed with error (%lld)\n", __func__, ret);
		preempt_enable();
		return -ENOMSG;
	}

	ret = nvg_send_req_data(TEGRA_NVG_CHANNEL_CSTATE_STAT_QUERY_VALUE, 0);
	if (ret) {
		pr_err("%s failed with error (%lld)\n", __func__, ret);
		preempt_enable();
		return -ENOMSG;
	}

	*stats = (u32)nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}
EXPORT_SYMBOL(t19x_mce_read_cstate_stats);

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
	nvg_cc3_control_channel_t cc3_ctrl;
	uint64_t ret;

	/* disable preemption */
	preempt_disable();

	/*
	 * If the enable bit is cleared, Auto-CC3 will be disabled by setting
	 * the SW visible frequency request registers for all non
	 * floorswept cores valid independent of StandbyWFI and disabling
	 * the IDLE frequency request register. If set, Auto-CC3
	 * will be enabled by setting the ARM SW visible frequency
	 * request registers for all non floorswept cores to be enabled by
	 * StandbyWFI or the equivalent signal, and always keeping the IDLE
	 * frequency request register enabled.
	 */
	cc3_ctrl.bits.freq_req = ndiv;
	cc3_ctrl.bits.enable = !!enable;

	ret = nvg_send_req_data(TEGRA_NVG_CHANNEL_CC3_CTRL, cc3_ctrl.flat);
	if (ret) {
		pr_err("%s failed with error (%lld)\n", __func__, ret);
		ret = -ENOMSG;
	}

	/* enable preemption */
	preempt_enable();

	return ret;
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
	uint64_t version, ret;

	if (!major || !minor)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	ret = nvg_send_req_data(TEGRA_NVG_CHANNEL_VERSION, 0);
	if (ret != 0) {
		pr_err("%s failed with error (%lld)\n", __func__, ret);
		preempt_enable();
		return -ENOMSG;
	}

	version = nvg_get_response();
	*major = (u32)version;
	*minor = (u32)(version >> 32);

	/* enable preemption */
	preempt_enable();

	return 0;
}
EXPORT_SYMBOL(t19x_mce_read_versions);

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

static struct dentry *mce_debugfs_root;

struct debugfs_entry {
	const char *name;
	const struct file_operations *fops;
	mode_t mode;
};

static struct debugfs_entry mce_dbg_attrs[] = {
	{ "versions", &mce_versions_fops, S_IRUGO },
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
