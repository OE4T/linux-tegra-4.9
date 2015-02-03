/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <linux/tegra-mce.h>

#include <asm/cputype.h> /* cpuid */

struct mce_ops {
	int (*enter_cstate)(u32 state, u32 wake);
	int (*update_cstate_info)(u32 cluster, u32 ccplex,
		u32 system, u8 force, u32 wake);
	int (*update_crossover_time)(u32 type, u32 time);
	int (*read_cstate_stats)(u32 state, u32 *stats);
	int (*write_cstate_stats)(u32 state, u32 stats);
	int (*call_misc)(u32 *low, u32 *high);
	int (*is_sc7_allowed)(u32 state, u32 wake, u32 *allowed);
	int (*online_core)(u32 core);
};

#define ENTRY(type, op) .op = type ## _ ## op

#define DEFINE_MCE_OPS(type) \
struct mce_ops type ## _ops = { \
	ENTRY(type, enter_cstate), \
	ENTRY(type, update_cstate_info), \
	ENTRY(type, update_crossover_time), \
	ENTRY(type, read_cstate_stats), \
	ENTRY(type, write_cstate_stats), \
	ENTRY(type, call_misc), \
	ENTRY(type, is_sc7_allowed), \
	ENTRY(type, online_core), \
};

#define mce_request(opfunc, ...) \
({ \
	int __ret; \
	struct mce_ops *__ops = get_cpu_var(cpu_mce_ops); \
	BUG_ON(!__ops); \
	__ret = __ops->opfunc(__VA_ARGS__); \
	put_cpu_var(cpu_mce_ops); \
	__ret; \
})

enum { MCE_MISC_CMD_ECHO, MCE_MISC_CMD_VERS, MCE_MISC_CMD_ENUM };

/*
 * ARI support for ARM cores
 */
#define ARI_APERTURE_LEN		0x20

#define ARI_REQUEST				0x0
#define ARI_REQUEST_EVENT_MASK	0x4
#define ARI_STATUS				0x8
#define ARI_REQUEST_DATA_LO		0x10
#define ARI_REQUEST_DATA_HI		0x14
#define ARI_RESPONSE_DATA_LO	0x18
#define ARI_RESPONSE_DATA_HI	0x1c

#define ARI_REQUEST_VALID		8

#define ARI_EVT_MASK_NULL		(0)
#define ARI_EVT_MASK_STANDBYWFI	(1 << 7)

enum {
	ARI_STATUS_PENDING = 1,
	ARI_STATUS_ONGOING = 3,
	ARI_STATUS_NONE,
};

enum {
	ARI_REQ_ENTER_CSTATE = 0x1,
	ARI_REQ_UPDATE_CLUSTER_CSTATE,
	ARI_REQ_UPDATE_CROSSOVER_TIME,
	ARI_REQ_READ_CSTATE_STATS,
	ARI_REQ_WRITE_CSTATE_STATS,
	ARI_REQ_IS_SC7_ALLOWED,
	ARI_REQ_CALL_MISC,
	ARI_REQ_ONLINE_CORE,
};

static void __iomem	*ari_bases[NR_CPUS];

static inline u32 ari_readl(u32 cpu, u32 reg)
{
	return readl(ari_bases[cpu] + reg);
}

static inline void ari_writel(u32 cpu, u32 val, u32 reg)
{
	writel(val, ari_bases[cpu] + reg);
}

static void ari_request_nowait(u32 events, u32 request, u32 low, u32 high)
{
	u32 reg;
	int cpu = raw_smp_processor_id();

	ari_writel(cpu, low, ARI_REQUEST_DATA_LO);
	ari_writel(cpu, high, ARI_REQUEST_DATA_HI);

	ari_writel(cpu, ARI_REQUEST_EVENT_MASK, events);

	barrier();

	reg = request | (1 << ARI_REQUEST_VALID);
	ari_writel(cpu, reg, ARI_REQUEST);
}

static inline u32 ari_get_response_low(void)
{
	return ari_readl(raw_smp_processor_id(), ARI_RESPONSE_DATA_LO);
}

static inline u32 ari_get_response_high(void)
{
	return ari_readl(raw_smp_processor_id(), ARI_RESPONSE_DATA_HI);
}

static inline void ari_clobber_response(void)
{
	int cpu = raw_smp_processor_id();
	ari_writel(cpu, 0, ARI_RESPONSE_DATA_LO);
	ari_writel(cpu, 0, ARI_RESPONSE_DATA_HI);
}

static int ari_request(u32 events, u32 request, u32 low, u32 high)
{
	int status;
	int busy_mask;
	int cpu = raw_smp_processor_id();

	ari_request_nowait(events, request, low, high);

	status = ARI_STATUS_NONE;
	busy_mask = ARI_STATUS_ONGOING | ARI_STATUS_PENDING;

	/* NOTE: add timeout check if needed */
	while (status & busy_mask)
		status = ari_readl(cpu, ARI_STATUS);

	return 0;
}

static int ari_enter_cstate(u32 state, u32 wake)
{
	return ari_request(ARI_EVT_MASK_STANDBYWFI,
		ARI_REQ_ENTER_CSTATE, state & 0x3, wake);
}

static u32 gen_cstate_info_low(u32 cluster, u32 ccplex, u32 system, u8 force)
{
	u32 low = 0;
	if (cluster)
		low |= (cluster & 0x7) | (1<<7);
	if (ccplex)
		low |= (ccplex & 0x3) << 8 | (1 << 15);
	if (system)
		low |= (system & 0x7) << 16 | (force << 22) | 1 << 23;
	return low;
}

static int
ari_update_cstate_info(u32 cluster, u32 ccplex, u32 system, u8 force, u32 wake)
{
	u32 low = gen_cstate_info_low(cluster, ccplex, system, force);
	return ari_request(0, ARI_REQ_UPDATE_CLUSTER_CSTATE, low, wake);
}

static int ari_update_crossover_time(u32 type, u32 time)
{
	if (type == TEGRA_MCE_XOVER_C1_C6)
		return -EINVAL;
	return ari_request(0, ARI_REQ_UPDATE_CROSSOVER_TIME, type & 0x3, time);
}

static int ari_read_cstate_stats(u32 state, u32 *stats)
{
	int ret = ari_request(0, ARI_REQ_READ_CSTATE_STATS, state, 0);
	if (!ret)
		*stats = ari_get_response_low();
	return ret;
}

static int ari_write_cstate_stats(u32 state, u32 stats)
{
	return ari_request(0, ARI_REQ_WRITE_CSTATE_STATS, state, stats);
}

static int ari_call_misc(u32 *low, u32 *high)
{
	int ret;
	ari_clobber_response();
	ret = ari_request(0, ARI_REQ_CALL_MISC, *low, *high);
	if (!ret) {
		*low = ari_get_response_low();
		*high = ari_get_response_high();
	}
	return ret;
}

static int ari_is_sc7_allowed(u32 state, u32 wake, u32 *allowed)
{
	int ret = ari_request(0, ARI_REQ_IS_SC7_ALLOWED, state & 0x3, wake);
	if (!ret)
		*allowed = ari_get_response_low() & 0x1;
	return ret;
}

static int ari_online_core(u32 core)
{
	return ari_request(0, ARI_REQ_ONLINE_CORE, core & 0x7, 0);
}

/*
 * NV generic (NVG) MTS-SW interfaces for Denver cores
 */

#define	NVG_ECHO_SUPPORTED	0 /* No ECHO support in NVG */

enum {
	NVG_REQ_WAKE_TIME = 0x3,
	NVG_REQ_CSTATE_INFO,

	/* crossovers */
	NVG_REQ_XOVER_C1_C6,
	NVG_REQ_XOVER_CC1_CC6,
	NVG_REQ_XOVER_CC1_CC7,
	NVG_REQ_XOVER_CCP1_CCP3,
	NVG_REQ_XOVER_CCP3_SC2,
	NVG_REQ_XOVER_CCP3_SC3,
	NVG_REQ_XOVER_CCP3_SC4,
	NVG_REQ_XOVER_CCP3_SC7,

	/* cstate stats */
	NVG_REQ_CSTATS_CLEAR,
	NVG_REQ_CSTATS_ENTRIES_SC7,
	NVG_REQ_CSTATS_ENTRIES_SC4,
	NVG_REQ_CSTATS_ENTRIES_SC3,
	NVG_REQ_CSTATS_ENTRIES_SC2,
	NVG_REQ_CSTATS_ENTRIES_CCP3,
	NVG_REQ_CSTATS_ENTRIES_A57_CC6,
	NVG_REQ_CSTATS_ENTRIES_A57_CC7,
	NVG_REQ_CSTATS_ENTRIES_D15_CC6,
	NVG_REQ_CSTATS_ENTRIES_D15_CC7,
	NVG_REQ_CSTATS_ENTRIES_D15_CORE0_C6,
	NVG_REQ_CSTATS_ENTRIES_D15_CORE1_C6,
	/* RESV: 25-26 */
	NVG_REQ_CSTATS_ENTRIES_D15_CORE0_C7,
	NVG_REQ_CSTATS_ENTRIES_D15_CORE1_C7,
	/* RESV: 28-29 */
	NVG_REQ_CSTATS_ENTRIES_A57_CORE0_C7 = 18,
	NVG_REQ_CSTATS_ENTRIES_A57_CORE1_C7,
	NVG_REQ_CSTATS_ENTRIES_A57_CORE2_C7,
	NVG_REQ_CSTATS_ENTRIES_A57_CORE3_C7,
	NVG_REQ_CSTATS_LAST_ENTRY_D15_CORE0,
	NVG_REQ_CSTATS_LAST_ENTRY_D15_CORE1,
	/* RESV: 36-37 */
	NVG_REQ_CSTATS_LAST_ENTRY_A57_CORE0,
	NVG_REQ_CSTATS_LAST_ENTRY_A57_CORE1,
	NVG_REQ_CSTATS_LAST_ENTRY_A57_CORE2,
	NVG_REQ_CSTATS_LAST_ENTRY_A57_CORE3,

	NVG_REQ_IS_SC7_ALLOWED = 42,
	NVG_REQ_ONLINE_CORE,
};

static inline void nvg_write_request(u32 index)
{
	asm volatile("msr s3_0_c15_c1_2, %0" : : "r" (index));
}

static inline void nvg_write_input(u64 data)
{
	asm volatile("msr s3_0_c15_c1_3, %0" : : "r" (data));
}

static inline void nvg_read_output(u64 *data)
{
	asm volatile("mrs %0, s3_0_c15_c1_3" : "=r" (*data));
}

static inline void nvg_clr_output(void)
{
	asm volatile("msr s3_0_c15_c1_3, %0" : : "r"(0));
}

static int nvg_enter_cstate(u32 state, u32 wake)
{
	/* Program wake time */
	nvg_write_input(wake);
	nvg_write_request(NVG_REQ_WAKE_TIME);
	/* Enter c-state */
	asm volatile("msr actlr_el1, %0" : : "r" (state));
	asm volatile("wfi");
	return 0;
}

static int nvg_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
	u8 force, u32 wake)
{
	u32 low = gen_cstate_info_low(cluster, ccplex, system, force);
	u64 high = ((u64)wake << 32);
	nvg_write_input(high | low);
	nvg_write_request(NVG_REQ_CSTATE_INFO);
	return 0;
}

static int nvg_update_crossover_time(u32 type, u32 time)
{
	nvg_write_input((u64)time << 32);
	nvg_write_request(type + NVG_REQ_XOVER_C1_C6);
	return 0;
}

static int nvg_read_cstate_stats(u32 state, u32 *stats)
{
	u64 data;
	nvg_write_input(state);
	nvg_write_request(state + NVG_REQ_CSTATS_CLEAR);
	nvg_read_output(&data);
	*stats = (u32)data;
	return 0;
}

static int nvg_write_cstate_stats(u32 state, u32 stats)
{
	nvg_write_input((u64)stats << 32 | state);
	nvg_write_request(state + NVG_REQ_CSTATS_CLEAR);
	return 0;
}

static int nvg_call_misc(u32 *low, u32 *high)
{
	/* Reroute MISC to ARI  */
	return ari_call_misc(low, high);
}

static int nvg_is_sc7_allowed(u32 state, u32 wake, u32 *allowed)
{
	u64 data = ((u64)wake << 32) | (state & 0x7);
	nvg_write_input(data);
	nvg_write_request(NVG_REQ_IS_SC7_ALLOWED);
	nvg_read_output(&data);
	*allowed = (u32)(data & 0x1);
	return 0;
}

static int nvg_online_core(u32 core)
{
	nvg_write_input(core & 0x7);
	nvg_write_request(NVG_REQ_ONLINE_CORE);
	return 0;
}

/* Instantiate MCE operations */
DEFINE_MCE_OPS(ari);
DEFINE_MCE_OPS(nvg);

/* Store MCE ops in a per-CPU variable */
DEFINE_PER_CPU(struct mce_ops *, cpu_mce_ops);

/**
 * Prepare MCE for a c-state.
 *
 * @state: power state to enter.
 * @wake: idle expectancy.
 *
 * Returns 0 if success.
 */
int tegra_mce_enter_cstate(u32 state, u32 wake)
{
	return mce_request(enter_cstate, state, wake);
}
EXPORT_SYMBOL(tegra_mce_enter_cstate);

/**
 * Specify deepest cluster/ccplex/system states allowed.
 *
 * @cluster:	deepest cluster-wide state
 * @ccplex:		deepest ccplex-wide state
 * @system:		deepest system-wide state
 * @force:		forced system state
 * @wake:		wake mask to be updated
 *
 * Returns 0 if success.
 */
int tegra_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
	u8 force, u32 wake_mask)
{
	return mce_request(update_cstate_info, cluster,
		ccplex, system, force, wake_mask);
}
EXPORT_SYMBOL(tegra_mce_update_cstate_info);

/**
 * Update threshold for one specific c-state crossover
 *
 * @type: type of state crossover.
 * @time: idle time threshold.
 *
 * Returns 0 if success.
 */
int tegra_mce_update_crossover_time(u32 type, u32 time)
{
	if (type > TEGRA_MCE_XOVER_MAX)
		return -EINVAL;
	return mce_request(update_crossover_time, type, time);
}
EXPORT_SYMBOL(tegra_mce_update_crossover_time);

/**
 * Query the runtime stats of a specific cstate
 *
 * @state: c-state of the stats.
 * @stats: output integer to hold the stats.
 *
 * Returns 0 if success.
 */
int tegra_mce_read_cstate_stats(u32 state, u32 *stats)
{
	if (!stats ||
		state == TEGRA_MCE_CSTATS_CLEAR ||
		state > TEGRA_MCE_CSTATS_MAX)
		return -EINVAL;
	return mce_request(read_cstate_stats, state, stats);
}
EXPORT_SYMBOL(tegra_mce_read_cstate_stats);

/**
 * Overwrite the runtime stats of a specific c-state
 *
 * @state: c-state of the stats.
 * @stats: integer represents the new stats.
 *
 * Returns 0 if success.
 */
int tegra_mce_write_cstate_stats(u32 state, u32 stats)
{
	if (state > TEGRA_MCE_CSTATS_MAX)
		return -EINVAL;
	return mce_request(write_cstate_stats, state, stats);
}
EXPORT_SYMBOL(tegra_mce_write_cstate_stats);

/**
 * Query MCE to determine if SC7 is allowed
 * given a target core's C-state and wake time
 *
 * @state: c-state of the stats.
 * @stats: integer represents the new stats.
 * @allowed: pointer to result
 *
 * Returns 0 if success.
 */
int tegra_mce_is_sc7_allowed(u32 state, u32 wake, u32 *allowed)
{
	if (!allowed)
		return -EINVAL;
	return mce_request(is_sc7_allowed, state, wake, allowed);
}
EXPORT_SYMBOL(tegra_mce_is_sc7_allowed);

/**
 * Send data to MCE which echoes it back.
 *
 * @data: data to be sent to MCE.
 * @out: output data to hold the response.
 * @matched: pointer to matching result
 *
 * Returns 0 if success.
 */
int tegra_mce_echo_data(u32 data, int *matched)
{
	u32 high = data;
	u32 low = MCE_MISC_CMD_ECHO;
	int ret;
	if (!matched)
		return -EINVAL;
	ret = mce_request(call_misc, &low, &high);
	if (!ret)
		*matched = (low == data && high == data);
	return ret;
}
EXPORT_SYMBOL(tegra_mce_echo_data);

/**
 * Read out MCE API major/minor versions
 *
 * @major: output for major number.
 * @minor: output for minor number.
 *
 * Returns 0 if success.
 */
int tegra_mce_read_versions(u32 *major, u32 *minor)
{
	u32 high = 0;
	u32 low = MCE_MISC_CMD_VERS;
	int ret;
	if (!major || !minor)
		return -EINVAL;
	ret = mce_request(call_misc, &low, &high);
	if (!ret) {
		*major = low;
		*minor = high;
	}
	return ret;
}
EXPORT_SYMBOL(tegra_mce_read_versions);

/**
 * Enumerate MCE API features
 *
 * @features: output feature vector (4bits each)
 *
 * Returns 0 if success.
 */
int tegra_mce_enum_features(u64 *features)
{
	u32 high = 0;
	u32 low = MCE_MISC_CMD_ENUM;
	int ret;
	if (!features)
		return -EINVAL;
	ret = mce_request(call_misc, &low, &high);
	if (!ret)
		*features = ((u64)high << 32) | low;
	return ret;
}
EXPORT_SYMBOL(tegra_mce_enum_features);

/**
 * Bring an offlined core back online to C0 state.
 *
 * @core: core to be onlined.
 *
 * Returns 0 if success.
 */
int tegra_mce_online_core(u32 core)
{
	if (core > TEGRA_MCE_ENUM_MAX)
		return -EINVAL;
	return mce_request(online_core, core);
}
EXPORT_SYMBOL(tegra_mce_online_core);

#ifdef CONFIG_DEBUG_FS

static int mce_echo_set(void *data, u64 val)
{
	u32 matched;
	int ret = tegra_mce_echo_data((u32)val, &matched);
	return ret ? ret : matched != 1;
}

static int mce_versions_get(void *data, u64 *val)
{
	u32 major, minor;
	int ret = tegra_mce_read_versions(&major, &minor);
	if (!ret)
		*val = ((u64)major << 32) | minor;
	return ret;
}

static int mce_features_get(void *data, u64 *val)
{
	return tegra_mce_enum_features(val);
}

DEFINE_SIMPLE_ATTRIBUTE(mce_echo_fops, NULL, mce_echo_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mce_versions_fops, mce_versions_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mce_features_fops, mce_features_get, NULL, "%llu\n");

static struct dentry *mce_debugfs_root;

struct debugfs_entry {
	const char *name;
	const struct file_operations *fops;
	mode_t mode;
};

static struct debugfs_entry mce_dbg_attrs[] = {
	{ "echo", &mce_echo_fops, S_IWUSR },
	{ "versions", &mce_versions_fops, S_IRUGO },
	{ "features", &mce_features_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static __init int mce_debugfs_init(struct device *dev)
{
	struct dentry *dent;
	struct debugfs_entry *fent;

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
#else
static __init int mce_debugfs_init(struct device *dev) { return 0; }
#endif

static int tegra_mce_cpu_notify(struct notifier_block *nb,
	unsigned long action, void *pcpu)
{
	int impl;
	int cpu = (long)pcpu;
	switch (action) {
	case CPU_STARTING:
		impl = read_cpuid_implementor();
		if (impl == ARM_CPU_IMP_ARM)
			per_cpu(cpu_mce_ops, cpu) = &ari_ops;
		else
			per_cpu(cpu_mce_ops, cpu) = &nvg_ops;
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block mce_cpu_notifier = {
	.notifier_call = tegra_mce_cpu_notify,
};

static int __init tegra_mce_early_init(void)
{
	/* Initialize boot CPU now */
	tegra_mce_cpu_notify(NULL, CPU_STARTING, (void *)0);
	register_cpu_notifier(&mce_cpu_notifier);
	return 0;
}
early_initcall(tegra_mce_early_init);

static __init int tegra_mce_probe(struct platform_device *pdev)
{
	int cpu;
	void __iomem *base;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (!np) {
		dev_err(dev, "DT data required.\n");
		return -EINVAL;
	}

	base = of_iomap(np, 0);
	if (!base) {
		dev_err(dev, "failed to map aperture.\n");
		return -1;
	}

	/*
	 * Configure bases for all cpus even though only
	 * ARM cores need to use ARI to be future-proof.
	 */
	for_each_possible_cpu(cpu)
		ari_bases[cpu] = base + cpu * ARI_APERTURE_LEN;

	dev_info(dev, "initialized.\n");

	return mce_debugfs_init(dev);
}

static const struct of_device_id mce_of_match[] __initconst = {
	{ .compatible = "nvidia,tegra186-mce", },
	{},
};

static __initdata struct platform_driver tegra_mce_driver = {
	.probe	= tegra_mce_probe,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_mce",
		.of_match_table = of_match_ptr(mce_of_match),
	},
};

static int __init tegra_mce_init(void)
{
	return platform_driver_register(&tegra_mce_driver);
}
core_initcall(tegra_mce_init);
