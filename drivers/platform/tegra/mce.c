/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/tegra-mce.h>

#define ARI_CORE_INST_LEN		0x20

#define ARI_REQUEST				0x0
#define ARI_REQUEST_EVENT_MASK	0x4
#define ARI_STATUS				0x8
#define ARI_REQUEST_DATA_LO		0x10
#define ARI_REQUEST_DATA_HI		0x14
#define ARI_RESPONSE_DATA_LO	0x18
#define ARI_RESPONSE_DATA_HI	0x1c

#define ARI_STATUS_REQ_PENDING	0x1
#define ARI_STATUS_REQ_ONGOING	0x3

#define ARI_STATUS_ONGOING_ID	8
#define ARI_STATUS_PENDING_ID	16
#define ARI_STATUS_ID_LEN		7

#define ARI_REQUEST_ID			0
#define ARI_REQUEST_VALID		8
#define ARI_REQUEST_KILL		9

#define ARI_REQ_ENTER_CSTATE			0x1
#define ARI_REQ_UPDATE_CLUSTER_CSTATE	0x2
#define ARI_REQ_UPDATE_CROSSOVER_TIME	0x3
#define ARI_REQ_CSTATE_STATS			0x4
#define ARI_REQ_ECHO					0x5

static void __iomem	*ari_bases[NR_CPUS];

static inline u32 ari_readl(u32 cpu, u32 reg)
{
	return readl(ari_bases[cpu] + reg);
}

static inline void ari_writel(u32 cpu, u32 val, u32 reg)
{
	writel(val, ari_bases[cpu] + reg);
}

static inline u32 ari_status_id(u32 reg, u32 pos)
{
	return (reg & (((1 << ARI_STATUS_ID_LEN) - 1) << pos)) >> pos;
}

static void ari_request_nowait(u32 events, u32 request, u64 data)
{
	u32 reg;
	int cpu = raw_smp_processor_id();

	ari_writel(cpu, (u32)data, ARI_REQUEST_DATA_LO);
	data >>= 32;
	ari_writel(cpu, (u32)data, ARI_REQUEST_DATA_HI);

	ari_writel(cpu, ARI_REQUEST_EVENT_MASK, events);

	barrier();

	reg = request | (1 << ARI_REQUEST_VALID);
	ari_writel(cpu, reg, ARI_REQUEST);
}

/**
 * Retrieve the status of current ongoing request.
 *
 * @request: output data to hold the current request ID.
 *
 * Returns the request status (pending/ongoing).
 */
int ari_get_status(u32 *request)
{
	u32 reg;
	int cpu;
	int status;

	cpu = raw_smp_processor_id();
	reg = ari_readl(cpu, ARI_STATUS);
	status = reg & (ARI_STATUS_PENDING | ARI_STATUS_ONGOING_ID);
	if (request) {
		if (status == ARI_STATUS_PENDING)
			*request = ari_status_id(reg, ARI_STATUS_PENDING_ID);
		else
			*request = ari_status_id(reg, ARI_STATUS_ONGOING_ID);
	}
	return status;
}
EXPORT_SYMBOL(ari_get_status);

/**
 * Retrieve the reponses from last ARI call.
 *
 * @data: output data to hold the reponse.
 *
 * Returns 0 if success.
 */
int ari_get_response(u64 *data)
{
	u32 lo, hi;
	int cpu = raw_smp_processor_id();
	BUG_ON(!data);
	lo = ari_readl(cpu, ARI_RESPONSE_DATA_LO);
	hi = ari_readl(cpu, ARI_RESPONSE_DATA_HI);
	*data = lo | ((u64)hi) << 32;
	return 0;
}
EXPORT_SYMBOL(ari_get_response);

/**
 * Issue an ARI request.
 *
 * @events: event masks to trigger the MCE fsm.
 * @request: ARI request ID.
 * @data: input data argument.
 *
 * Returns 0 if success.
 */
int ari_request(u32 events, u32 request, u64 data)
{
	int status;
	int busy_mask;
	int cpu = get_cpu();

	ari_request_nowait(events, request, data);

	status = ARI_STATUS_NONE;
	busy_mask = ARI_STATUS_ONGOING | ARI_STATUS_PENDING;

	/* NOTE: add timeout check if needed */
	while (status & busy_mask)
		status = ari_readl(cpu, ARI_STATUS);

	put_cpu();

	return 0;
}
EXPORT_SYMBOL(ari_request);

/**
 * Cancel a pending ARI request.
 *
 * @request: ID of a pending request to be killed.
 *
 * Returns 0 if success.
 */
int ari_cancel(u32 request)
{
	u32 reg;
	int cpu = raw_smp_processor_id();

	reg = request | (1 << ARI_REQUEST_KILL);
	ari_writel(cpu, reg, ARI_REQUEST);

	return 0;
}
EXPORT_SYMBOL(ari_cancel);

/**
 * Prepare MCE for a c-state.
 *
 * @state: power state to enter.
 * @wake: idle expectancy.
 *
 * Returns 0 if success.
 */
int tegra_ari_enter_cstate(u32 state, u32 wake)
{
	u64 data = state | ((u64)wake << 32);
	return ari_request(0, ARI_REQ_ENTER_CSTATE, data);
}
EXPORT_SYMBOL(tegra_ari_enter_cstate);

/**
 * Specify the lowest c-state current cluster/ccplex allows.
 *
 * @cluster_state: state for the current cluster.
 * @ccplex_state: state for the whole ccplex.
 *
 * Returns 0 if success.
 */
int tegra_ari_update_cluster_cstate(u32 cluster_state, u32 ccplex_state)
{
	u32 lo = !cluster_state ? 0 : cluster_state | (1<<31);
	u32 hi = !ccplex_state ? 0 : ccplex_state | (1<<31);
	u64 data = lo | ((u64)hi << 32);
	return ari_request(0, ARI_REQ_ENTER_CSTATE, data);
}
EXPORT_SYMBOL(tegra_ari_update_cluster_cstate);

/**
 * Update threshold for one specific c-state crossover
 *
 * @type: type of state crossover.
 * @time: idle time threshold.
 *
 * Returns 0 if success.
 */
int tegra_ari_update_crossover_time(u32 type, u32 time)
{
	u64 data = type | ((u64)time << 32);
	return ari_request(0, ARI_REQ_UPDATE_CROSSOVER_TIME, data);
}
EXPORT_SYMBOL(tegra_ari_update_crossover_time);

/**
 * Retrieve the runtime stats of a specific c-state
 *
 * @state: c-state of the stats.
 * @stats: output integer to hold the stats.
 *
 * Returns 0 if success.
 */
int tegra_ari_cstate_stats(int state, u64 *stats)
{
	int ret;
	u64 data = state;
	BUG_ON(!stats);
	ret = ari_request(0, ARI_REQ_CSTATE_STATS, data);
	if (!ret)
		ari_get_response(stats);
	return ret;
}
EXPORT_SYMBOL(tegra_ari_cstate_stats);

#ifdef CONFIG_DEBUG_FS
static struct dentry *mce_debugfs_root;
static bool ari_echo_success;

static int ari_echo_get(void *data, u64 *val)
{
	*val = ari_echo_success;
	return 0;
}

static int ari_echo_set(void *data, u64 val)
{
	u64 result;
	int ret = 0;
	int cpu = get_cpu();
	struct device *dev = (struct device *)data;

	/* clobber response registers */
	ari_writel(cpu, 0, ARI_RESPONSE_DATA_LO);
	ari_writel(cpu, 0, ARI_RESPONSE_DATA_HI);

	ret = ari_request(0, ARI_REQ_ECHO, val);
	if (ret) {
		dev_err(dev, "failed to issue ECHO request.\n");
		goto exit;
	}

	ari_get_response(&result);

	/* Store the result */
	ari_echo_success = (val == result);

exit:
	put_cpu();
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(ari_echo_fops, ari_echo_get, ari_echo_set, "%llu\n");

static __init int ari_debugfs_init(struct device *dev)
{
	mce_debugfs_root = debugfs_create_dir(
			"tegra_mce", NULL);

	if (!mce_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file(
		"ari_echo", S_IRUGO | S_IWUGO, mce_debugfs_root,
		dev, &ari_echo_fops)) {
		dev_err(dev, "failed to add ari_echo debugfs");
		return -ENOMEM;
	}

	return 0;
}
#else
static __init int ari_debugfs_init(void)
{
	return 0;
}
#endif

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

	for_each_possible_cpu(cpu)
		ari_bases[cpu] = base + cpu * ARI_CORE_INST_LEN;

	dev_info(dev, "initialized.\n");

	return ari_debugfs_init(dev);
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
