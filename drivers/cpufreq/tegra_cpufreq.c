/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/cpu.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <soc/tegra/tegra_bpmp.h>
#include <linux/delay.h>

#define MAX_NDIV		512 /* No of NDIV */
#define MAX_VINDEX		80 /*No of voltage index */
#define NUM_TEMP_RANGE		4
#define NUM_TEMP_MIN		NUM_TEMP_RANGE
#define NUM_TEMP_MAX		NUM_TEMP_RANGE
#define NUM_RESERVE		((MAX_VINDEX * NUM_TEMP_RANGE) \
				+ NUM_TEMP_MIN + NUM_TEMP_MAX)
 /* cpufreq transisition latency */
#define TEGRA_CPUFREQ_TRANSITION_LATENCY	(300 * 1000)

#define KHZ_TO_HZ		1000
#define REF_CLK_KHZ		408000 /* 408 MHz */
#define US_DELAY		20
#define CPUFREQ_TBL_STEP_SIZE	4

#define CLUSTER_STR(cl)	(cl == B_CLUSTER ? \
				"B_CLUSTER" : "M_CLUSTER")
#define LOOP_FOR_EACH_CLUSTER(cl)	for (cl = M_CLUSTER; \
					cl < MAX_CLUSTERS; cl++)

/* EDVD register details */
#define EDVD_CL_NDIV_VHINT_OFFSET	0x20
#define EDVD_COREX_NDIV_VAL_SHIFT	(0)
#define EDVD_COREX_NDIV_MASK		(0x1ff << 0)
#define EDVD_COREX_VINDEX_VAL_SHIFT	(26)
#define EDVD_COREX_VINDEX_MASK		(0xff << 26)

/* ACTMON counter register details */
#define CORECLK_OFFSET			(0x0)
#define REFCLK_OFFSET			(0x4)
#define REG_OFFSET			(0x4)
#define REF_CLOCK_MASK			(0xfffffff)
#define coreclk_base(base, cpu)	(base + CORECLK_OFFSET \
					+ (REG_OFFSET * cpu))
#define refclk_base(base, cpu)		(base + REFCLK_OFFSET \
					+ (REG_OFFSET * cpu))
#define tcpufreq_readl(base, cpu)	readl((void __iomem *) \
					base + \
					(REG_OFFSET * cpu))
#define tcpufreq_writel(val, base, cpu)	writel(val, base + \
					(REG_OFFSET * cpu))
#define logical_to_phys_map(cpu)	(MPIDR_AFFINITY_LEVEL \
					(cpu_logical_map(cpu), 0))

enum cluster {
	M_CLUSTER, /* DENEVER cluster */
	B_CLUSTER, /* ARM cluster */
	MAX_CLUSTERS,
};

/* cluster freq params */
struct cl_lut {
	uint32_t ref_clk_hz; /* reference clock */
	uint16_t pdiv; /* post divider */
	uint16_t mdiv; /* input divider */
	uint16_t ndiv_max; /* fMAX expressed with max NDIV */
	uint16_t ndiv[MAX_VINDEX];
	uint16_t reserved[NUM_RESERVE]; /* space reserved for future use */
};

/**
 * Cpu side dvfs table
 * This table needs to be constructed at boot up time
 * BPMP will provide NDIV and Vidx tuple.
 * BPMP will also provide per custer Pdiv, Mdiv, ref_clk.
 * freq = (ndiv * refclk) / (pdiv * mdiv)
*/
struct cpu_vhint_table {
	struct cl_lut *lut; /* virtual address of NDIV[VINDEX] */
	dma_addr_t phys;
	uint32_t ref_clk_hz;
	uint16_t pdiv; /* post divider */
	uint16_t mdiv; /* input divider */
	uint8_t *vindx;
};

struct per_cluster_data {
	struct cpufreq_frequency_table *clft;
	void __iomem *edvd_pub;
	struct cpu_vhint_table dvfs_tbl;
};

struct tegra_cpufreq_data {
	struct per_cluster_data pcluster[MAX_CLUSTERS];
	struct mutex mlock; /* lock protecting below params */
	uint32_t freq_compute_delay; /* delay in reading clock counters */
};

struct msg_data {
	uint32_t addr;
	uint8_t cl; /* cluster ID */
} __packed;

static struct tegra_cpufreq_data tfreq_data;
static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};
static DEFINE_PER_CPU(spinlock_t, pcpu_slock);
static DEFINE_PER_CPU(struct mutex, pcpu_mlock);

static enum cluster get_cpu_cluster(uint8_t cpu)
{
	struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu);
	u32 midr = cpuinfo->reg_midr;

	return (MIDR_IMPLEMENTOR(midr) == ARM_CPU_IMP_ARM ? B_CLUSTER
		: M_CLUSTER);
}

static uint32_t get_coreclk_count(uint8_t cpu)
{
	enum cluster cur_cluster = get_cpu_cluster(cpu);
	void __iomem *reg_base;
	uint32_t phy_cpu;

	phy_cpu = logical_to_phys_map(cpu);

	reg_base = coreclk_base(tfreq_data.pcluster[cur_cluster].edvd_pub,
				phy_cpu);
	return tcpufreq_readl(reg_base, phy_cpu);
}

static uint32_t get_refclk_count(uint8_t cpu)
{
	enum cluster cur_cl = get_cpu_cluster(cpu);
	void __iomem *reg_base;
	uint32_t phy_cpu;

	phy_cpu = logical_to_phys_map(cpu);

	reg_base = refclk_base(tfreq_data.pcluster[cur_cl].edvd_pub, phy_cpu);
	return tcpufreq_readl(reg_base, phy_cpu) & REF_CLOCK_MASK;
}

/**
 * Return instantaneous cpu speed
 * Instantaneous freq is calculated as -
 * -Takes sample on every query of getting the freq.
 *        - Read core and ref clock counters;
 *        - Delay for X us
 *       -  Read above cycle counters again
 *       - Calculates freq by subtracting current and previous counters
 *          divided by the delay time or eqv. of ref_clk_counter in delta time
 *       - Return Kcycles/second, freq in KHz
 *
 * - delta time period = x sec
 *          = delta ref_clk_counter / (408 * 10^6) sec
 * freq in Hz = cycles/sec
 *                 = (delta cycles / x sec
 *                 = (delta cycles * 408 * 10^6) / delta ref_clk_counter
 *     in KHz = (delta cycles * 408 * 10^3) / delta ref_clk_counter
 *
 * @cpu - logical cpu whose freq to be updated
 * Returns freq in KHz on success, 0 if cpu is offline
 */
static unsigned int tegra_get_speed(uint32_t cpu)
{
	uint32_t coreclk_cnt, last_coreclk_cnt, delta_ccnt;
	uint32_t refclk_cnt, last_refclk_cnt, delta_refcnt;
	unsigned int rate_khz = 0;
	unsigned long flags;
	spinlock_t *slock;

	if (cpu_online(cpu)) {
		slock = &per_cpu(pcpu_slock, cpu);
		spin_lock_irqsave(slock, flags);

		last_coreclk_cnt = get_coreclk_count(cpu);
		last_refclk_cnt = get_refclk_count(cpu);

		udelay(tfreq_data.freq_compute_delay);

		coreclk_cnt = get_coreclk_count(cpu);
		refclk_cnt = get_refclk_count(cpu);

		delta_ccnt = (coreclk_cnt > last_coreclk_cnt ?
			(coreclk_cnt - last_coreclk_cnt) :
			(last_coreclk_cnt - coreclk_cnt));

		delta_refcnt = (refclk_cnt > last_refclk_cnt ?
			(refclk_cnt - last_refclk_cnt) :
			(last_refclk_cnt - refclk_cnt));

		rate_khz = (delta_ccnt * REF_CLK_KHZ) / delta_refcnt;

		spin_unlock_irqrestore(slock, flags);
	}
	/* Do we have to align rate as nearest freq step ? */
	return rate_khz; /* in KHz */
}


/* TBDL: Set emc clock by looking up the cpu_to_emc look up table */
static void tegra_set_cpufreq_to_emcfreq(unsigned int cpu_freq)
{
	return;
}

static struct cpufreq_frequency_table *get_freqtable(uint8_t cpu)
{
	enum cluster cur_cl = get_cpu_cluster(cpu);

	return tfreq_data.pcluster[cur_cl].clft;
}

static int tegra_verify_speed(struct cpufreq_policy *policy)
{
	return 0;
}

/**
 * tegra_update_cpu_speed - update cpu freq
 * @rate - in kHz
 * @cpu - cpu whose freq to be updated
 * Returns 0 on success, -ve on failure
 */
static void tegra_update_cpu_speed(uint32_t rate, uint8_t cpu)
{
	struct cpu_vhint_table *vhtbl;
	uint32_t val = 0, phy_cpu;
	enum cluster cur_cl;
	unsigned long flags;
	spinlock_t *slock;
	uint16_t ndiv;
	int8_t vindx;

	slock = &per_cpu(pcpu_slock, cpu);
	spin_lock_irqsave(slock, flags);

	cur_cl = get_cpu_cluster(cpu);
	vhtbl = &tfreq_data.pcluster[cur_cl].dvfs_tbl;
	rate *= vhtbl->pdiv * vhtbl->mdiv;
	ndiv = (rate * KHZ_TO_HZ) / vhtbl->ref_clk_hz;
	if ((rate * KHZ_TO_HZ) % vhtbl->ref_clk_hz)
		ndiv++;

	val |= (ndiv << EDVD_COREX_NDIV_VAL_SHIFT);
	vindx = vhtbl->vindx[ndiv];
	if (vindx == -1) {
		pr_err("unable to find Vhint for Ndiv %u setting Vhint : 0\n",
			ndiv);
		vindx = 0;
	}
	val |= (vindx << EDVD_COREX_VINDEX_VAL_SHIFT);
	phy_cpu = logical_to_phys_map(cpu);

	tcpufreq_writel(val, tfreq_data.pcluster[cur_cl].edvd_pub +
		EDVD_CL_NDIV_VHINT_OFFSET, phy_cpu);
	spin_unlock_irqrestore(slock, flags);
}

/**
 * tegra_setspeed - Request freq to be set for policy->cpu
 * @policy - cpufreq policy per cpu
 * @index - freq table index
 * Returns 0 on success, -ve on failure
 */
static int tegra_setspeed(struct cpufreq_policy *policy, unsigned int index)
{
	struct cpufreq_frequency_table *ftbl;
	struct cpufreq_freqs freqs;
	struct mutex *mlock;
	uint32_t tgt_freq;

	int ret = 0;

	if (!policy || (!cpu_online(policy->cpu)))
		return -EINVAL;

	mlock = &per_cpu(pcpu_mlock, policy->cpu);
	mutex_lock(mlock);

	ftbl = get_freqtable(policy->cpu);
	tgt_freq = ftbl[index].frequency;
	freqs.old = tegra_get_speed(policy->cpu);

	if (freqs.old == tgt_freq)
		goto out;

	freqs.new = tgt_freq;

	cpufreq_freq_transition_begin(policy, &freqs);
	tegra_update_cpu_speed(tgt_freq, policy->cpu);

	policy->cur = tegra_get_speed(policy->cpu);
	freqs.new = policy->cur;
	/* TBDLater set emc freq for this cpu freq */
	tegra_set_cpufreq_to_emcfreq(policy->cur);
	cpufreq_freq_transition_end(policy, &freqs, ret);
out:
	pr_debug("cpu: %d, oldfreq(kHz): %d, req freq(kHz): %d final freq(kHz): %d tgt_index %u\n",
		policy->cpu, freqs.old, tgt_freq, policy->cur, index);
	mutex_unlock(mlock);
	return ret;
}

#ifdef CONFIG_DEBUG_FS
#define RW_MODE			(S_IWUSR | S_IRUGO)
#define RO_MODE			(S_IRUGO)

static int get_delay(void *data, u64 *val)
{
	mutex_lock(&tfreq_data.mlock);

	*val = tfreq_data.freq_compute_delay;

	mutex_unlock(&tfreq_data.mlock);
	return 0;
}

static int set_delay(void *data, u64 val)
{
	uint32_t udelay = val;

	mutex_lock(&tfreq_data.mlock);

	if (udelay)
		tfreq_data.freq_compute_delay = udelay;

	mutex_unlock(&tfreq_data.mlock);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(freq_compute_fops, get_delay, set_delay,
	"%llu\n");

static void dump_lut(struct seq_file *s, struct cpu_vhint_table *vht)
{
	uint16_t i, j;

	seq_printf(s, "reference clk(hz): %u\n", vht->ref_clk_hz);
	seq_printf(s, "pdiv: %u\n", vht->pdiv);
	seq_printf(s, "mdiv: %u\n", vht->mdiv);
	for (i = 0, j = 0; i < MAX_NDIV; i++) {
		if (vht->vindx[i] == 0)
			continue;
		else
			seq_printf(s, "ndiv[%u]: %u\n", j++, vht->vindx[i]);
	}
	seq_puts(s, "\n");
}

static int show_bpmp_to_cpu_lut(struct seq_file *s, void *data)
{
	struct cpu_vhint_table *vht;
	enum cluster cl;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vht = &tfreq_data.pcluster[cl].dvfs_tbl;
		seq_printf(s, "\n%s:\n", CLUSTER_STR(cl));
		dump_lut(s, vht);
	}

	return 0;
}

static int stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_bpmp_to_cpu_lut, inode->i_private);
}

static const struct file_operations lut_fops = {
	.open = stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *tegra_cpufreq_debugfs_root;
static int __init tegra_cpufreq_debug_init(void)
{
	tegra_cpufreq_debugfs_root = debugfs_create_dir("tegra_cpufreq", NULL);
	if (!tegra_cpufreq_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file("bpmp_cpu_vhint_table", RO_MODE,
				 tegra_cpufreq_debugfs_root,
					NULL,
					&lut_fops))
		goto err_out;

	if (!debugfs_create_file("freq_compute_delay", RW_MODE,
				 tegra_cpufreq_debugfs_root,
					NULL,
					&freq_compute_fops))
		goto err_out;

	return 0;
err_out:
	debugfs_remove_recursive(tegra_cpufreq_debugfs_root);
	return -ENOMEM;
}

static void __exit tegra_cpufreq_debug_exit(void)
{
	debugfs_remove_recursive(tegra_cpufreq_debugfs_root);
}
#endif

static int tegra_cpu_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *ftbl;
	uint32_t freq;
	int ret = 0;
	int idx;

	if (policy->cpu >= CONFIG_NR_CPUS)
		return -EINVAL;

	freq = tegra_get_speed(policy->cpu); /* boot freq */

	ftbl = get_freqtable(policy->cpu);

	cpufreq_table_validate_and_show(policy, ftbl);

	/* clip boot frequency to table entry */
	ret = cpufreq_frequency_table_target(policy, ftbl, freq,
		CPUFREQ_RELATION_H, &idx);
	if (!ret && (freq != ftbl[idx].frequency)) {
		freq = ftbl[idx].frequency;
		tegra_update_cpu_speed(freq, policy->cpu);
	}
	policy->cur = tegra_get_speed(policy->cpu);

	/* TBDLater set emc freq for this cpu freq */
	tegra_set_cpufreq_to_emcfreq(policy->cur);

	policy->cpuinfo.transition_latency =
	TEGRA_CPUFREQ_TRANSITION_LATENCY;
	cpumask_copy(policy->cpus, cpu_possible_mask);
	return ret;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *ftbl;

	ftbl = get_freqtable(policy->cpu);
	cpufreq_frequency_table_cpuinfo(policy, ftbl);
	return 0;
}

static struct cpufreq_driver tegra_cpufreq_driver = {
	.name		= "tegra_cpufreq",
	.flags		= CPUFREQ_ASYNC_NOTIFICATION | CPUFREQ_STICKY |
				CPUFREQ_CONST_LOOPS,
	.verify		= tegra_verify_speed,
	.target_index	= tegra_setspeed,
	.get		= tegra_get_speed,
	.init		= tegra_cpu_init,
	.exit		= tegra_cpu_exit,
	.attr		= tegra_cpufreq_attr,
};

/* Free lut space shared beteen CPU and BPMP */
static void __init free_shared_lut(void)
{
	uint16_t size = sizeof(struct cl_lut);
	struct cpu_vhint_table *vhtbl;
	enum cluster cl;

	LOOP_FOR_EACH_CLUSTER(cl) {
		/* Free lut space shared by BPMP */
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;
		if (vhtbl->lut && vhtbl->phys)
			tegra_bpmp_free_coherent(size, vhtbl->lut,
				vhtbl->phys);
	}
}
static void free_resources(void)
{
	enum cluster cl;

	LOOP_FOR_EACH_CLUSTER(cl) {
		/* unmap iova space */
		if (tfreq_data.pcluster[cl].edvd_pub)
			iounmap(tfreq_data.pcluster[cl].edvd_pub);

		/* free ndiv_to_vindex mem */
		kfree(tfreq_data.pcluster[cl].dvfs_tbl.vindx);

		/* free table*/
		kfree(tfreq_data.pcluster[cl].clft);
	}
}

static void __init free_allocated_res_init(void)
{
	free_resources();
}

static void __exit free_allocated_res_exit(void)
{
	free_resources();
}

static bool __init adjust_remainder(uint16_t ndiv_max, int16_t *steps)
{
	bool ret;

	*steps = (ndiv_max / CPUFREQ_TBL_STEP_SIZE);
	ret = ((ndiv_max % CPUFREQ_TBL_STEP_SIZE) != 0);
	if (ret)
		*steps += 1;
	return ret;
}

static int __init init_freqtbls(void)
{
	struct cpufreq_frequency_table *ftbl;
	struct cpu_vhint_table *vhtbl;
	uint16_t ndiv, max_freq_steps;
	enum cluster cl;
	bool rem;
	int ret = 0;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;
		rem  = adjust_remainder(vhtbl->lut->ndiv_max,
					&max_freq_steps);

		ftbl = kzalloc(sizeof(struct cpufreq_frequency_table) *
			max_freq_steps, GFP_KERNEL);
		if (!ftbl) {
			ret = -ENOMEM;
			while (cl--)
				kfree(tfreq_data.pcluster[cl].clft);
			goto err_out;
		}

		/* generate freq table from ndiv table */
		for (ndiv = 0; ndiv < (max_freq_steps - rem); ndiv++)
			ftbl[ndiv].frequency = ((ndiv + 1) *
				CPUFREQ_TBL_STEP_SIZE * vhtbl->ref_clk_hz) /
				(vhtbl->pdiv * vhtbl->mdiv * 1000);
		if (rem)
			ftbl[ndiv++].frequency = (vhtbl->lut->ndiv_max *
			vhtbl->ref_clk_hz) /
			(vhtbl->pdiv * vhtbl->mdiv * 1000);

		ftbl[ndiv].frequency = CPUFREQ_TABLE_END;

		tfreq_data.pcluster[cl].clft = ftbl;
	}

err_out:
	return ret;
}

static int __init create_ndiv_to_vindex_table(void)
{
	struct cpu_vhint_table *vhtbl;
	uint16_t mid_ndiv, i;
	struct cl_lut *lut;
	enum cluster cl;
	uint8_t vindx;
	int ret = 0;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;

		lut = vhtbl->lut;
		vhtbl->vindx = kzalloc(sizeof(uint8_t) * MAX_NDIV,
				GFP_KERNEL);
		if (!vhtbl->vindx) {
			ret = -ENOMEM;
			while (cl--)
				kfree(vhtbl->vindx);
			goto err_out;
		}

		i = 0;
		for (vindx = 0; vindx < MAX_VINDEX; vindx++) {
			mid_ndiv = lut->ndiv[vindx];
			for (; ((mid_ndiv < MAX_NDIV) && (i < mid_ndiv)); i++)
				vhtbl->vindx[i] = vindx;
		}
		/* Fill remaining vindex table by last vindex value */
		for (; i < MAX_NDIV; i++)
			vhtbl->vindx[i] = vindx - 1;

		vhtbl->ref_clk_hz =  lut->ref_clk_hz;
		vhtbl->pdiv = lut->pdiv;
		vhtbl->mdiv = lut->mdiv;
	}
err_out:
	return ret;
}

static int __init get_lut_from_bpmp(void)
{
	const size_t size = sizeof(struct cl_lut);
	struct cpu_vhint_table *vhtbl;
	struct msg_data md;
	struct cl_lut *virt;
	dma_addr_t phys;
	enum cluster cl;
	int ret = 0;

	LOOP_FOR_EACH_CLUSTER(cl) {
		vhtbl = &tfreq_data.pcluster[cl].dvfs_tbl;
		virt = (struct cl_lut *)tegra_bpmp_alloc_coherent(size, &phys,
				 GFP_KERNEL);
		if (!virt) {
			ret = -ENOMEM;
			while (cl--) {
				tegra_bpmp_free_coherent(size, vhtbl->lut,
					vhtbl->phys);
			}

			goto err_out;
		}
		vhtbl->lut = virt;
		vhtbl->phys = phys;

		md.addr = cpu_to_le32(phys);
		md.cl = cpu_to_le32(cl);

		ret = tegra_bpmp_send_receive(MRQ_CPU_VHINT, &md,
				sizeof(struct msg_data), NULL, 0);
		if (ret) {
			pr_err("CPU_to_BPMP send receive failure %d\n",
				ret);
			goto err_out;
		}
	}
err_out:
	return ret;
}

static int __init mem_map_device(void)
{
	void __iomem *base = NULL;
	struct device_node *dn;
	enum cluster cl;
	int ret = 0;

	dn = of_find_compatible_node(NULL, NULL, "nvidia,tegra18x-edvd");
	if (dn == NULL) {
		pr_err("tegra18x-edvd: dt node not found\n");
		ret = -EINVAL;
		goto err_out;
	}

	LOOP_FOR_EACH_CLUSTER(cl) {
		base = of_iomap(dn, cl);
		if (IS_ERR_OR_NULL(base)) {
			pr_warn("Failed to iomap memory for %s\n",
			CLUSTER_STR(cl));
			ret = -EINVAL;
			while (cl--)
				iounmap(tfreq_data.pcluster[cl].edvd_pub);
			goto err_out;
		}

		tfreq_data.pcluster[cl].edvd_pub = base;
	}
err_out:
	return ret;
}

static int __init tegra_cpufreq_init(void)
{
	uint32_t cpu;
	int ret = 0;

	ret = mem_map_device();
	if (ret)
		return ret;

	ret = get_lut_from_bpmp();
	if (ret)
		goto err_free_res;

	ret = create_ndiv_to_vindex_table();
	if (ret)
		goto err_free_res;

	ret = init_freqtbls();
	if (ret)
		goto err_free_res;

	mutex_init(&tfreq_data.mlock);
	tfreq_data.freq_compute_delay = US_DELAY;

	for_each_possible_cpu(cpu) {
		spin_lock_init(&per_cpu(pcpu_slock, cpu));
		mutex_init(&per_cpu(pcpu_mlock, cpu));
	}

#ifdef CONFIG_DEBUG_FS
	tegra_cpufreq_debug_init();
#endif

	ret = cpufreq_register_driver(&tegra_cpufreq_driver);
	if (ret)
		goto err_free_res;

	goto exit_out;
err_free_res:
	free_allocated_res_init();
exit_out:
	free_shared_lut();
	pr_info("cpufreq: platform driver Initialization: %s\n",
		(ret ? "fail" : "pass"));
	return ret;
}

static void __exit tegra_cpufreq_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	tegra_cpufreq_debug_exit();
#endif
	cpufreq_unregister_driver(&tegra_cpufreq_driver);
	free_allocated_res_exit();
}

MODULE_AUTHOR("Puneet Saxena <puneets@nvidia.com>");
MODULE_DESCRIPTION("cpufreq platform driver for Nvidia Tegra18x");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
