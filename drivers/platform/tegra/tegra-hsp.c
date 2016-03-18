/*
 * Copyright (c) 2014-2016 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>

#include <linux/tegra-hsp.h>

#define HSP_DB_REG_TRIGGER		0x0
#define HSP_DB_REG_ENABLE		0x4
#define HSP_DB_REG_RAW			0x8
#define HSP_DB_REG_PENDING		0xc

enum tegra_hsp_init_status {
	HSP_INIT_PENDING,
	HSP_INIT_FAILED,
	HSP_INIT_OKAY,
};

struct hsp_top {
	int nr_sm;
	int nr_as;
	int nr_ss;
	int nr_db;
	int nr_si;
	int status;
	void __iomem *base;
	spinlock_t lock;
};

struct db_handler_info {
	db_handler_t	handler;
	void			*data;
};

static struct hsp_top hsp_top = { .status = HSP_INIT_PENDING };
static void __iomem *db_bases[HSP_NR_DBS];

static DEFINE_MUTEX(db_handlers_lock);
static int db_irq;
static struct db_handler_info db_handlers[HSP_LAST_MASTER + 1];

static const char * const master_names[] = {
	[HSP_MASTER_SECURE_CCPLEX] = "SECURE_CCPLEX",
	[HSP_MASTER_SECURE_DPMU] = "SECURE_DPMU",
	[HSP_MASTER_SECURE_BPMP] = "SECURE_BPMP",
	[HSP_MASTER_SECURE_SPE] = "SECURE_SPE",
	[HSP_MASTER_SECURE_SCE] = "SECURE_SCE",
	[HSP_MASTER_CCPLEX] = "CCPLEX",
	[HSP_MASTER_DPMU] = "DPMU",
	[HSP_MASTER_BPMP] = "BPMP",
	[HSP_MASTER_SPE] = "SPE",
	[HSP_MASTER_SCE] = "SCE",
	[HSP_MASTER_APE] = "APE",
};

static const char * const db_names[] = {
	[HSP_DB_DPMU] = "DPMU",
	[HSP_DB_CCPLEX] = "CCPLEX",
	[HSP_DB_CCPLEX_TZ] = "CCPLEX_TZ",
	[HSP_DB_BPMP] = "BPMP",
	[HSP_DB_SPE] = "SPE",
	[HSP_DB_SCE] = "SCE",
	[HSP_DB_APE] = "APE",
};

static inline int is_master_valid(int master)
{
	return master_names[master] != NULL;
}

static inline int next_valid_master(int m)
{
	for (m++; m <= HSP_LAST_MASTER && !is_master_valid(m); m++)
		;
	return m;
}

#define for_each_valid_master(m) \
	for (m = HSP_FIRST_MASTER; \
		 m <= HSP_LAST_MASTER; \
		 m = next_valid_master(m))

#define hsp_db_offset(i, d) \
	(d.base + ((1 + (d.nr_sm>>1) + d.nr_ss + d.nr_as)<<16) + (i) * 0x100)

#define hsp_ready() (hsp_top.status == HSP_INIT_OKAY)

static inline u32 hsp_readl(void __iomem *base, int reg)
{
	return readl(base + reg);
}

static inline void hsp_writel(void __iomem *base, int reg, u32 val)
{
	writel(val, base + reg);
	(void)readl(base + reg);
}

static irqreturn_t dbell_irq(int irq, void *data)
{
	ulong reg;
	int master;
	struct db_handler_info *info;

	reg = (ulong)hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING);
	hsp_writel(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_PENDING, reg);

	for_each_set_bit(master, &reg, HSP_LAST_MASTER + 1) {
		info = &db_handlers[master];
		if (unlikely(!is_master_valid(master))) {
			pr_warn("invalid master from HW.\n");
			continue;
		}
		if (info->handler)
			info->handler(info->data);
	}

	return IRQ_HANDLED;
}

/**
 * tegra_hsp_db_get_enabled_masters: get masters that can ring CCPLEX
 * @master:	 HSP master
 *
 * Returns the mask of enabled masters of CCPLEX.
 */
int tegra_hsp_db_get_enabled_masters(void)
{
	if (!hsp_ready())
		return -EINVAL;
	return hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE);
}
EXPORT_SYMBOL(tegra_hsp_db_get_enabled_masters);

/**
 * tegra_hsp_db_enable_master: allow <master> to ring CCPLEX
 * @master:	 HSP master
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_enable_master(enum tegra_hsp_master master)
{
	u32 reg;
	unsigned long flags;
	if (!hsp_ready() || !is_master_valid(master))
		return -EINVAL;
	spin_lock_irqsave(&hsp_top.lock, flags);
	reg = hsp_readl(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE);
	reg |= BIT(master);
	hsp_writel(db_bases[HSP_DB_CCPLEX], HSP_DB_REG_ENABLE, reg);
	spin_unlock_irqrestore(&hsp_top.lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_enable_master);

/**
 * tegra_hsp_db_ring: ring the <dbell>
 * @dbell:	 HSP dbell to be rung
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_ring(enum tegra_hsp_doorbell dbell)
{
	if (!hsp_ready() || dbell >= HSP_NR_DBS)
		return -EINVAL;
	hsp_writel(db_bases[dbell], HSP_DB_REG_TRIGGER, 1);
	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_ring);

/**
 * tegra_hsp_db_can_ring: check if CCPLEX can ring the <dbell>
 * @dbell:	 HSP dbell to be checked
 *
 * Returns 1 if CCPLEX can ring <dbell> otherwise 0.
 */
int tegra_hsp_db_can_ring(enum tegra_hsp_doorbell dbell)
{
	int reg;
	if (!hsp_ready() || dbell >= HSP_NR_DBS)
		return 0;
	reg = hsp_readl(db_bases[dbell], HSP_DB_REG_ENABLE);
	return !!(reg & BIT(HSP_MASTER_CCPLEX));
}
EXPORT_SYMBOL(tegra_hsp_db_can_ring);

static int tegra_hsp_db_get_signals(u32 reg)
{
	if (!hsp_ready())
		return -EINVAL;
	return hsp_readl(db_bases[HSP_DB_CCPLEX], reg);
}

static int tegra_hsp_db_clr_signals(u32 reg, u32 mask)
{
	if (!hsp_ready())
		return -EINVAL;
	hsp_writel(db_bases[HSP_DB_CCPLEX], reg, mask);
	return 0;
}

/**
 * tegra_hsp_db_get_pending: get pending rings to CCPLEX
 *
 * Returns a mask of pending signals.
 */
int tegra_hsp_db_get_pending(void)
{
	return tegra_hsp_db_get_signals(HSP_DB_REG_PENDING);
}
EXPORT_SYMBOL(tegra_hsp_db_get_pending);

/**
 * tegra_hsp_db_clr_pending: clear rings of ccplex based on <mask>
 * @mask:	mask of masters to be cleared
 */
int tegra_hsp_db_clr_pending(u32 mask)
{
	return tegra_hsp_db_clr_signals(HSP_DB_REG_PENDING, mask);
}
EXPORT_SYMBOL(tegra_hsp_db_clr_pending);

/**
 * tegra_hsp_db_get_raw: get unmasked rings to CCPLEX
 *
 * Returns a mask of unmasked pending signals.
 */
int tegra_hsp_db_get_raw(void)
{
	return tegra_hsp_db_get_signals(HSP_DB_REG_RAW);
}
EXPORT_SYMBOL(tegra_hsp_db_get_raw);

/**
 * tegra_hsp_db_clr_raw: clear unmasked rings of ccplex based on <mask>
 * @mask:	mask of masters to be cleared
 */
int tegra_hsp_db_clr_raw(u32 mask)
{
	return tegra_hsp_db_clr_signals(HSP_DB_REG_RAW, mask);
}
EXPORT_SYMBOL(tegra_hsp_db_clr_raw);

/**
 * tegra_hsp_db_add_handler: register an CCPLEX doorbell IRQ handler
 * @ master:	master id
 * @ handler:	IRQ handler
 * @ data:		custom data
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_add_handler(int master, db_handler_t handler, void *data)
{
	if (!handler || !is_master_valid(master))
		return -EINVAL;

	if (unlikely(db_irq <= 0))
		return -ENODEV;

	mutex_lock(&db_handlers_lock);
	if (likely(db_handlers[master].handler != NULL)) {
		mutex_unlock(&db_handlers_lock);
		return -EBUSY;
	}

	disable_irq(db_irq);
	db_handlers[master].handler = handler;
	db_handlers[master].data = data;
	enable_irq(db_irq);
	mutex_unlock(&db_handlers_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_add_handler);

/**
 * tegra_hsp_db_del_handler: unregister an CCPLEX doorbell IRQ handler
 * @handler:	IRQ handler
 *
 * Returns 0 if successful.
 */
int tegra_hsp_db_del_handler(int master)
{
	if (!is_master_valid(master))
		return -EINVAL;

	if (unlikely(db_irq <= 0))
		return -ENODEV;

	mutex_lock(&db_handlers_lock);
	WARN_ON(db_handlers[master].handler == NULL);
	disable_irq(db_irq);
	db_handlers[master].handler = NULL;
	db_handlers[master].data = NULL;
	enable_irq(db_irq);
	mutex_unlock(&db_handlers_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_hsp_db_del_handler);

#ifdef CONFIG_DEBUG_FS

static int hsp_dbg_enable_master_show(void *data, u64 *val)
{
	*val = tegra_hsp_db_get_enabled_masters();
	return 0;
}

static int hsp_dbg_enable_master_store(void *data, u64 val)
{
	if (!is_master_valid((u32)val))
		return -EINVAL;
	return tegra_hsp_db_enable_master((enum tegra_hsp_master)val);
}

static int hsp_dbg_ring_store(void *data, u64 val)
{
	if (val >= HSP_NR_DBS)
		return -EINVAL;
	return tegra_hsp_db_ring((enum tegra_hsp_doorbell)val);
}

static int hsp_dbg_can_ring_show(void *data, u64 *val)
{
	enum tegra_hsp_doorbell db;
	*val = 0ull;
	for (db = HSP_FIRST_DB; db <= HSP_LAST_DB; db++)
		if (tegra_hsp_db_can_ring(db))
			*val |= BIT(db);
	return 0;
}

/* By convention, CPU shouldn't touch other processors' DBs.
 * So this interface is created for debugging purpose.
 */
static int hsp_dbg_can_ring_store(void *data, u64 val)
{
	int reg;
	enum tegra_hsp_doorbell dbell = (int)val;
	unsigned long flags;
	if (!hsp_ready() || dbell >= HSP_NR_DBS)
		return -EINVAL;
	spin_lock_irqsave(&hsp_top.lock, flags);
	reg = hsp_readl(db_bases[dbell], HSP_DB_REG_ENABLE);
	reg |= BIT(HSP_MASTER_CCPLEX);
	hsp_writel(db_bases[dbell], HSP_DB_REG_ENABLE, reg);
	spin_unlock_irqrestore(&hsp_top.lock, flags);
	return 0;
}

static int hsp_dbg_pending_show(void *data, u64 *val)
{
	*val = tegra_hsp_db_get_pending();
	return 0;
}

static int hsp_dbg_pending_store(void *data, u64 val)
{
	tegra_hsp_db_clr_pending((u32)val);
	return 0;
}

static int hsp_dbg_raw_show(void *data, u64 *val)
{
	*val = tegra_hsp_db_get_raw();
	return 0;
}

static int hsp_dbg_raw_store(void *data, u64 val)
{
	tegra_hsp_db_clr_raw((u32)val);
	return 0;
}

static u32 ccplex_intr_count;

static void hsp_dbg_db_handler(void *data)
{
	ccplex_intr_count++;
}

static int hsp_dbg_intr_count_show(void *data, u64 *val)
{
	*val = ccplex_intr_count;
	return 0;
}

static int hsp_dbg_intr_count_store(void *data, u64 val)
{
	ccplex_intr_count = val;
	return 0;
}

static int hsp_dbg_doorbells_show(struct seq_file *s, void *data)
{
	int db;
	seq_printf(s, "%-20s%-10s%-10s\n", "name", "id", "offset");
	seq_printf(s, "--------------------------------------------------\n");
	for (db = HSP_FIRST_DB; db <= HSP_LAST_DB; db++)
		seq_printf(s, "%-20s%-10d%-10lx\n", db_names[db], db,
			(uintptr_t)(db_bases[db] - hsp_top.base));
	return 0;
}

static int hsp_dbg_masters_show(struct seq_file *s, void *data)
{
	int m;
	seq_printf(s, "%-20s%-10s\n", "name", "id");
	seq_printf(s, "----------------------------------------\n");
	for_each_valid_master(m)
		seq_printf(s, "%-20s%-10d\n", master_names[m], m);
	return 0;
}

static int hsp_dbg_handlers_show(struct seq_file *s, void *data)
{
	int m;
	seq_printf(s, "%-20s%-30s\n", "master", "handler");
	seq_printf(s, "--------------------------------------------------\n");
	for_each_valid_master(m)
		seq_printf(s, "%-20s%-30pS\n", master_names[m],
			db_handlers[m].handler);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(enable_master_fops,
	hsp_dbg_enable_master_show, hsp_dbg_enable_master_store, "%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(ring_fops,
	NULL, hsp_dbg_ring_store, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(can_ring_fops,
	hsp_dbg_can_ring_show, hsp_dbg_can_ring_store, "%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(pending_fops,
	hsp_dbg_pending_show, hsp_dbg_pending_store, "%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(raw_fops,
	hsp_dbg_raw_show, hsp_dbg_raw_store, "%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(intr_count_fops,
	hsp_dbg_intr_count_show, hsp_dbg_intr_count_store, "%lld\n");

#define DEFINE_DBG_OPEN(name) \
static int hsp_dbg_##name##_open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, hsp_dbg_##name##_show, inode->i_private); \
} \
static const struct file_operations name##_fops = { \
	.open = hsp_dbg_##name##_open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release, \
};

DEFINE_DBG_OPEN(doorbells);
DEFINE_DBG_OPEN(masters);
DEFINE_DBG_OPEN(handlers);

struct debugfs_entry {
	const char *name;
	const struct file_operations *fops;
	mode_t mode;
};

static struct debugfs_entry hsp_dbg_attrs[] = {
	{ "enable_master", &enable_master_fops, S_IRUGO | S_IWUSR },
	{ "ring", &ring_fops, S_IWUSR },
	{ "can_ring", &can_ring_fops, S_IRUGO | S_IWUSR },
	{ "pending", &pending_fops, S_IRUGO | S_IWUSR },
	{ "raw", &raw_fops, S_IRUGO | S_IWUSR },
	{ "doorbells", &doorbells_fops, S_IRUGO },
	{ "masters", &masters_fops, S_IRUGO },
	{ "handlers", &handlers_fops, S_IRUGO },
	{ "intr_count", &intr_count_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static struct dentry *hsp_debugfs_root;

static int debugfs_init(void)
{
	struct dentry *dent;
	struct debugfs_entry *fent;

	if (!hsp_ready())
		return 0;

	hsp_debugfs_root = debugfs_create_dir("tegra_hsp", NULL);
	if (IS_ERR_OR_NULL(hsp_debugfs_root))
		return -EFAULT;

	fent = hsp_dbg_attrs;
	while (fent->name) {
		dent = debugfs_create_file(fent->name, fent->mode,
			hsp_debugfs_root, NULL, fent->fops);
		if (IS_ERR_OR_NULL(dent))
			goto abort;
		fent++;
	}

	tegra_hsp_db_add_handler(HSP_MASTER_CCPLEX, hsp_dbg_db_handler, NULL);

	return 0;

abort:
	debugfs_remove_recursive(hsp_debugfs_root);
	return -EFAULT;
}
late_initcall(debugfs_init);
#endif

#define NV(prop) "nvidia," prop

static const struct of_device_id tegra_hsp_of_match[] = {
	{ .compatible = "nvidia,tegra186-hsp", },
	{},
};

int tegra_hsp_init(void)
{
	int i;
	int irq;
	struct device_node *np;
	int ret = 0;

	if (hsp_ready())
		return 0;

	np = of_find_compatible_node(NULL, NULL,
		tegra_hsp_of_match[0].compatible);
	if (!np) {
		WARN_ON(1);
		pr_err("tegra-hsp: NV data required.\n");
		return -EINVAL;
	}

	ret |= of_property_read_u32(np, NV("num-SM"), &hsp_top.nr_sm);
	ret |= of_property_read_u32(np, NV("num-AS"), &hsp_top.nr_as);
	ret |= of_property_read_u32(np, NV("num-SS"), &hsp_top.nr_ss);
	ret |= of_property_read_u32(np, NV("num-DB"), &hsp_top.nr_db);
	ret |= of_property_read_u32(np, NV("num-SI"), &hsp_top.nr_si);

	if (ret) {
		pr_err("tegra-hsp: failed to parse HSP config.\n");
		return -EINVAL;
	}

	hsp_top.base = of_iomap(np, 0);
	if (!hsp_top.base) {
		pr_err("tegra-hsp: failed to map HSP IO space.\n");
		return -EINVAL;
	}

	for (i = HSP_FIRST_DB; i <= HSP_LAST_DB; i++) {
		db_bases[i] = hsp_db_offset(i, hsp_top);
		pr_debug("tegra-hsp: db[%d]: %p\n", i, db_bases[i]);
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("tegra-hsp: failed to parse doorbell irq\n");
		return -EINVAL;
	}

	ret = request_irq(irq, dbell_irq, IRQF_NO_SUSPEND, "hsp", NULL);
	if (ret) {
		pr_err("tegra-hsp: request_irq() failed (%d)\n", ret);
		return -EINVAL;
	}

	of_node_put(np);

	db_irq = irq;
	hsp_top.status = HSP_INIT_OKAY;
	return 0;
}

static int tegra_hsp_probe(struct platform_device *pdev)
{
	return tegra_hsp_init();
}

static struct platform_driver tegra_hsp_driver = {
	.probe	= tegra_hsp_probe,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_hsp",
		.of_match_table = of_match_ptr(tegra_hsp_of_match),
	},
};

static int __init tegra_hsp_initcall(void)
{
	return platform_driver_register(&tegra_hsp_driver);
}
core_initcall(tegra_hsp_initcall);
