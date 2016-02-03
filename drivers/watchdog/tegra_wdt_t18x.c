/*
 * drivers/watchdog/tegra_wdt_t18x.c
 *
 * watchdog driver for NVIDIA tegra internal watchdog
 *
 * Copyright (c) 2012-2016, NVIDIA CORPORATION. All rights reserved.
 *
 * based on drivers/watchdog/softdog.c and drivers/watchdog/omap_wdt.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-pmc.h>
#include <linux/tegra_wdt_t18x.h>

/* minimum and maximum watchdog trigger periods, in seconds */
#define MIN_WDT_PERIOD	5
#define MAX_WDT_PERIOD	1000

struct tegra_wdt_t18x {
	struct platform_device *pdev;
	struct watchdog_device	wdt;
	unsigned long		users;
	void __iomem		*wdt_source;
	void __iomem		*wdt_timer;
	void __iomem		*wdt_tke;
	u32			config;
	int			irq;
	unsigned long		status;
	int			cluster_id;
	bool			enable_on_init;
	int			expiry_count;
	int			heartbeat;
/* Bit numbers for status flags */
#define WDT_ENABLED		0
#define WDT_ENABLED_ON_INIT	1
#define WDT_ENABLED_USERSPACE	2
};

/* Cluster ids */
#define WDT_CLUSTER_ID_DENVER	0
#define WDT_CLUSTER_ID_ARM	1
#define WDT_CLUSTER_ID_COUNT	2

/*
 * Global variable to store wdt pointer for all clusters
 * required by nvdumper and pmic to change wdt state
 */
static struct tegra_wdt_t18x *t18x_wdt_array[WDT_CLUSTER_ID_COUNT];

/*
 * The total expiry count of Tegra WDTs is limited to HW design and depends
 * on skip configuration if supported. To be safe, we set the default expiry
 * count to 1. It should be updated later with value specified in device tree.
 */
#define EXPIRY_COUNT	1

/*
 * To detect lockup condition, the heartbeat should be expiry_count*lockup.
 * It may be taken over later by timeout value requested by application.
 * Must be greater than expiry_count*MIN_WDT_PERIOD and lower than
 * expiry_count*MAX_WDT_PERIOD.
 */
#define HEARTBEAT	120

static inline struct tegra_wdt_t18x *to_tegra_wdt_t18x(
					struct watchdog_device *wdt)
{
	return container_of(wdt, struct tegra_wdt_t18x, wdt);
}

#define TOP_TKE_TKEIE_BASE		0x100
#define TOP_TKE_TKEIE(i)		(0x100 + 4 * (i))
#define TOP_TKE_TKEIE_WDT_MASK(i)	(1 << (16 + 4 * (i)))
#define TOP_TKE_TMR_PTV			0
#define TOP_TKE_TMR_EN			(1 << 31)
#define TOP_TKE_TMR_PERIODIC		(1 << 30)
#define TOP_TKE_TMR_PCR			0x4
#define TOP_TKE_TMR_PCR_INTR		(1 << 30)
#define WDT_CFG				(0)
#define WDT_CFG_PERIOD			(1 << 4)
#define WDT_CFG_INT_EN			(1 << 12)
#define WDT_CFG_FINT_EN			(1 << 13)
#define WDT_CFG_REMOTE_INT_EN		(1 << 14)
#define WDT_CFG_DBG_RST_EN		(1 << 15)
#define WDT_CFG_SYS_PORST_EN		(1 << 16)
#define WDT_CFG_ERR_THRESHOLD		(7 << 20)
#define WDT_STATUS			(0x4)
#define WDT_INTR_STAT			(1 << 1)
#define WDT_CMD				(0x8)
#define WDT_CMD_START_COUNTER		(1 << 0)
#define WDT_CMD_DISABLE_COUNTER		(1 << 1)
#define WDT_UNLOCK			(0xC)
#define WDT_UNLOCK_PATTERN		(0xC45A << 0)
#define WDT_SKIP			(0x10)
#define WDT_SKIP_VAL(i, val)		(((val) & 0xf) << (4 * (i)))
#define MAX_NR_CLUSTER_WDT		0x2

static int __tegra_wdt_t18x_ping(struct tegra_wdt_t18x *tegra_wdt_t18x)
{
	u32 val;

	/*
	 * Disable timer, load the timeout value and restart.
	 */
	writel(WDT_UNLOCK_PATTERN, tegra_wdt_t18x->wdt_source + WDT_UNLOCK);
	writel(WDT_CMD_DISABLE_COUNTER, tegra_wdt_t18x->wdt_source + WDT_CMD);

	writel(TOP_TKE_TMR_PCR_INTR, tegra_wdt_t18x->wdt_timer +
							TOP_TKE_TMR_PCR);
	val = (tegra_wdt_t18x->wdt.timeout * USEC_PER_SEC) /
				tegra_wdt_t18x->expiry_count;
	val |= (TOP_TKE_TMR_EN | TOP_TKE_TMR_PERIODIC);
	writel(val, tegra_wdt_t18x->wdt_timer + TOP_TKE_TMR_PTV);

	writel(WDT_CMD_START_COUNTER, tegra_wdt_t18x->wdt_source + WDT_CMD);

	dev_dbg(tegra_wdt_t18x->wdt.dev, "wdt cleared\n");
	return 0;
}

static irqreturn_t tegra_wdt_t18x_isr(int irq, void *data)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = data;

	__tegra_wdt_t18x_ping(tegra_wdt_t18x);

	return IRQ_HANDLED;
}

static void tegra_wdt_t18x_ref(struct watchdog_device *wdt)
{
	struct tegra_wdt_t18x *tegra_wdt = to_tegra_wdt_t18x(wdt);

	if (tegra_wdt->irq <= 0)
		return;

	/*
	 * Remove the interrupt handler if userspace is taking over WDT.
	 */
	if (!test_and_set_bit(WDT_ENABLED_USERSPACE, &tegra_wdt->status) &&
			test_bit(WDT_ENABLED_ON_INIT, &tegra_wdt->status))
		devm_free_irq(&tegra_wdt->pdev->dev, tegra_wdt->irq,
				tegra_wdt);
}

static inline void tegra_wdt_t18x_skip(struct tegra_wdt_t18x *tegra_wdt_t18x)
{
	u32 val = 0;

	/* Skip the 2nd expiry of WDT for ARM cluster */
	if (tegra_wdt_t18x->cluster_id == WDT_CLUSTER_ID_ARM)
		val = WDT_SKIP_VAL(1, 1);

	/* Skip the 4th expiry if debug reset is disabled */
	if (!(tegra_wdt_t18x->config & WDT_CFG_DBG_RST_EN))
		val |= WDT_SKIP_VAL(3, 1);

	if (val)
		writel(val, tegra_wdt_t18x->wdt_source + WDT_SKIP);
}

static int __tegra_wdt_t18x_enable(struct tegra_wdt_t18x *tegra_wdt_t18x)
{
	u32 val;

	writel(TOP_TKE_TMR_PCR_INTR, tegra_wdt_t18x->wdt_timer +
							TOP_TKE_TMR_PCR);
	/*
	 * The timeout needs to be divided by expiry_count here so as to
	 * keep the ultimate watchdog reset timeout the same as the program
	 * timeout requested by application. The program timeout should make
	 * sure WDT FIQ will never be asserted in a valid use case.
	 */
	val = (tegra_wdt_t18x->wdt.timeout * USEC_PER_SEC) /
			tegra_wdt_t18x->expiry_count;
	val |= (TOP_TKE_TMR_EN | TOP_TKE_TMR_PERIODIC);
	writel(val, tegra_wdt_t18x->wdt_timer + TOP_TKE_TMR_PTV);

	tegra_wdt_t18x_skip(tegra_wdt_t18x);

	writel(tegra_wdt_t18x->config, tegra_wdt_t18x->wdt_source + WDT_CFG);
	writel(WDT_CMD_START_COUNTER, tegra_wdt_t18x->wdt_source + WDT_CMD);
	set_bit(WDT_ENABLED, &tegra_wdt_t18x->status);

	return 0;
}

static int __tegra_wdt_t18x_disable(struct tegra_wdt_t18x *tegra_wdt_t18x)
{
	writel(WDT_UNLOCK_PATTERN, tegra_wdt_t18x->wdt_source + WDT_UNLOCK);
	writel(WDT_CMD_DISABLE_COUNTER, tegra_wdt_t18x->wdt_source + WDT_CMD);

	writel(0, tegra_wdt_t18x->wdt_timer + TOP_TKE_TMR_PTV);

	clear_bit(WDT_ENABLED, &tegra_wdt_t18x->status);

	return 0;
}

static int tegra_wdt_t18x_enable(struct watchdog_device *wdt)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = to_tegra_wdt_t18x(wdt);
	return __tegra_wdt_t18x_enable(tegra_wdt_t18x);
}

static int tegra_wdt_t18x_disable(struct watchdog_device *wdt)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = to_tegra_wdt_t18x(wdt);
	return __tegra_wdt_t18x_disable(tegra_wdt_t18x);
}

static int tegra_wdt_t18x_ping(struct watchdog_device *wdt)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = to_tegra_wdt_t18x(wdt);
	return __tegra_wdt_t18x_ping(tegra_wdt_t18x);
}


static int tegra_wdt_t18x_set_timeout(struct watchdog_device *wdt,
	unsigned int timeout)
{
	tegra_wdt_t18x_disable(wdt);
	wdt->timeout = timeout;
	tegra_wdt_t18x_enable(wdt);

	dev_info(wdt->dev, "wdt timeout set to %u seconds\n", timeout);
	return 0;
}

static const struct watchdog_info tegra_wdt_t18x_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "Tegra WDT",
	.firmware_version = 1,
};

static const struct watchdog_ops tegra_wdt_t18x_ops = {
	.owner = THIS_MODULE,
	.start = tegra_wdt_t18x_enable,
	.stop  = tegra_wdt_t18x_disable,
	.ping  = tegra_wdt_t18x_ping,
	.set_timeout = tegra_wdt_t18x_set_timeout,
	.ref   = tegra_wdt_t18x_ref,
};

static inline int tegra_wdt_t18x_update_config_bit(struct tegra_wdt_t18x
	*tegra_wdt_t18x, u32 bitmask, bool set)
{
	if (set)
		tegra_wdt_t18x->config |= bitmask;
	else
		tegra_wdt_t18x->config &= ~bitmask;

	/* Apply the config only if WDT is enabled */
	if (test_bit(WDT_ENABLED, &tegra_wdt_t18x->status)) {
		__tegra_wdt_t18x_disable(tegra_wdt_t18x);
		__tegra_wdt_t18x_enable(tegra_wdt_t18x);
	}
	return 0;
}

void tegra_wdt_t18x_debug_reset(bool state)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x;
	int i;

	for (i = 0; i < WDT_CLUSTER_ID_COUNT; i++) {
		tegra_wdt_t18x = t18x_wdt_array[i];
		if (!tegra_wdt_t18x)
			continue;
		tegra_wdt_t18x_update_config_bit(tegra_wdt_t18x,
						WDT_CFG_DBG_RST_EN, state);
	}
}
EXPORT_SYMBOL(tegra_wdt_t18x_debug_reset);

void tegra_wdt_t18x_por_reset(bool state)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x;
	int i;

	for (i = 0; i < WDT_CLUSTER_ID_COUNT; i++) {
		tegra_wdt_t18x = t18x_wdt_array[i];
		if (!tegra_wdt_t18x)
			continue;
		tegra_wdt_t18x_update_config_bit(tegra_wdt_t18x,
						WDT_CFG_SYS_PORST_EN, state);
	}
}
EXPORT_SYMBOL(tegra_wdt_t18x_por_reset);

void tegra_wdt_t18x_disable_all(void)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x;
	int i;

	for (i = 0; i < WDT_CLUSTER_ID_COUNT; i++) {
		tegra_wdt_t18x = t18x_wdt_array[i];
		if (!tegra_wdt_t18x)
			continue;
		__tegra_wdt_t18x_disable(tegra_wdt_t18x);
	}
}
EXPORT_SYMBOL(tegra_wdt_t18x_disable_all);

#ifdef CONFIG_DEBUG_FS

static int dump_registers_show(void *data, u64 *val)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = data;

	pr_info("timer config register \t%x\n",
			readl(tegra_wdt_t18x->wdt_timer + TOP_TKE_TMR_PTV));
	pr_info("timer status register \t%x\n",
			readl(tegra_wdt_t18x->wdt_timer + TOP_TKE_TMR_PCR));
	pr_info("watchdog config register \t%x\n",
			readl(tegra_wdt_t18x->wdt_source + WDT_CFG));
	pr_info("watchdog status register \t%x\n",
			readl(tegra_wdt_t18x->wdt_source + WDT_STATUS));
	pr_info("watchdog command register \t%x\n",
			readl(tegra_wdt_t18x->wdt_source + WDT_CMD));
	pr_info("watchdog skip register \t%x\n",
			readl(tegra_wdt_t18x->wdt_source + WDT_SKIP));

	*val = 0;

	return 0;
}

static int disable_dbg_reset_show(void *data, u64 *val)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = data;

	*val = tegra_wdt_t18x->config & WDT_CFG_DBG_RST_EN ? 0 : 1;
	return 0;
}

static int disable_dbg_reset_store(void *data, u64 val)
{
	return tegra_wdt_t18x_update_config_bit((struct tegra_wdt_t18x*)data,
			WDT_CFG_DBG_RST_EN, !val);
}

static int disable_por_reset_show(void *data, u64 *val)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = data;

	*val = tegra_wdt_t18x->config &	WDT_CFG_SYS_PORST_EN ? 0 : 1;
	return 0;
}

static int disable_por_reset_store(void *data, u64 val)
{
	return tegra_wdt_t18x_update_config_bit((struct tegra_wdt_t18x*)data,
			WDT_CFG_SYS_PORST_EN, !val);
}

DEFINE_SIMPLE_ATTRIBUTE(dump_regs_fops, dump_registers_show,
	NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(disable_dbg_reset_fops, disable_dbg_reset_show,
	disable_dbg_reset_store, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(disable_por_reset_fops, disable_por_reset_show,
	disable_por_reset_store, "%lld\n");

static void tegra_wdt_t18x_debugfs_init(struct tegra_wdt_t18x *tegra_wdt_t18x)
{
	struct dentry *root;
	struct dentry *retval;
	struct platform_device *pdev = tegra_wdt_t18x->pdev;

	root = debugfs_create_dir(pdev->name, NULL);
	if (IS_ERR_OR_NULL(root))
		goto clean;

	retval = debugfs_create_file("dump_regs", S_IRUGO | S_IWUSR,
			root, (void *)tegra_wdt_t18x, &dump_regs_fops);
	if (IS_ERR_OR_NULL(retval))
		goto clean;

	retval = debugfs_create_file("disable_dbg_reset", S_IRUGO | S_IWUSR,
			root, (void *)tegra_wdt_t18x, &disable_dbg_reset_fops);
	if (IS_ERR_OR_NULL(retval))
		goto clean;

	retval = debugfs_create_file("disable_por_reset", S_IRUGO | S_IWUSR,
			root, (void *)tegra_wdt_t18x, &disable_por_reset_fops);
	if (IS_ERR_OR_NULL(retval))
		goto clean;

	return;
clean:
	pr_warn("tegra_wdt_t18x: Failed to create debugfs!\n");
	debugfs_remove_recursive(root);
}

#else /* !CONFIG_DEBUG_FS */
static inline void tegra_wdt_t18x_debugfs_init(
			struct tegra_wdt_t18x *tegra_wdt_t18x) { };
#endif /* CONFIG_DEBUG_FS */

static int tegra_wdt_t18x_setup_pet(struct tegra_wdt_t18x *tegra_wdt_t18x,
					int index)
{
	struct platform_device *pdev = tegra_wdt_t18x->pdev;
	int ret = 0;

	if (!tegra_wdt_t18x->enable_on_init)
		return 0;

	if (tegra_wdt_t18x->irq <= 0) {
		dev_err(&pdev->dev, "failed to get WDT IRQ\n");
		return -ENXIO;
	}

	if (!tegra_platform_is_linsim())
		writel(TOP_TKE_TKEIE_WDT_MASK(index), tegra_wdt_t18x->wdt_tke +
				TOP_TKE_TKEIE(tegra_wdt_t18x->irq - 32));

	ret = devm_request_threaded_irq(&pdev->dev,
			tegra_wdt_t18x->irq,	NULL, tegra_wdt_t18x_isr,
			IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
			dev_name(&pdev->dev), tegra_wdt_t18x);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register irq %d err %d\n",
				tegra_wdt_t18x->irq, ret);
		return ret;
	}

	tegra_wdt_t18x_enable(&tegra_wdt_t18x->wdt);
	set_bit(WDOG_ACTIVE, &tegra_wdt_t18x->wdt.status);
	set_bit(WDT_ENABLED_ON_INIT, &tegra_wdt_t18x->status);
	pr_info("Tegra WDT setup for cluster %d. Timeout = %u seconds.\n",
			tegra_wdt_t18x->cluster_id,
			tegra_wdt_t18x->wdt.timeout);

	return 0;
}

static int tegra_wdt_t18x_probe(struct platform_device *pdev)
{
	struct resource *res_src, *res_wdt, *res_tke;
	struct tegra_wdt_t18x *tegra_wdt_t18x;
	struct device_node *np = pdev->dev.of_node;
	u32 pval = 0;
	int ret = 0, index;

	/*
	 * If HALT_IN_FIQ is set, skip driver probe to allow
	 * external debugger to poke.
	 */
	if (tegra_pmc_is_halt_in_fiq()) {
			pr_warn("Skipping WDT driver probe\n");
			return -EACCES;
	}

	if (!np) {
		dev_err(&pdev->dev, "Support registration from DT only");
		return -EPERM;
	}

	tegra_wdt_t18x = devm_kzalloc(&pdev->dev, sizeof(*tegra_wdt_t18x),
								GFP_KERNEL);
	if (!tegra_wdt_t18x) {
		dev_err(&pdev->dev, "out of memory\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(np, "nvidia,heartbeat-init", &pval);
	if (!ret)
		tegra_wdt_t18x->heartbeat = pval;
	else
		tegra_wdt_t18x->heartbeat = HEARTBEAT;

	ret = of_property_read_u32(np, "nvidia,expiry-count", &pval);
	if (!ret)
		tegra_wdt_t18x->expiry_count = pval;
	else
		tegra_wdt_t18x->expiry_count = EXPIRY_COUNT;

	tegra_wdt_t18x->pdev = pdev;
	tegra_wdt_t18x->wdt.info = &tegra_wdt_t18x_info;
	tegra_wdt_t18x->wdt.ops = &tegra_wdt_t18x_ops;
	tegra_wdt_t18x->wdt.min_timeout = MIN_WDT_PERIOD *
					tegra_wdt_t18x->expiry_count;
	tegra_wdt_t18x->wdt.max_timeout = MAX_WDT_PERIOD *
					tegra_wdt_t18x->expiry_count;
	tegra_wdt_t18x->wdt.timeout = tegra_wdt_t18x->heartbeat;
	tegra_wdt_t18x->irq = irq_of_parse_and_map(np, 0);
	tegra_wdt_t18x->enable_on_init =
			of_property_read_bool(np, "nvidia,enable-on-init");

	ret = of_property_read_u32(np, "nvidia,wdt-cluster-id", &pval);
	if (ret) {
		dev_err(&pdev->dev, "no WDT cluster ID provided!\n");
		return -EINVAL;
	}
	tegra_wdt_t18x->cluster_id = pval;
	if (pval < WDT_CLUSTER_ID_COUNT)
		t18x_wdt_array[pval] = tegra_wdt_t18x;
	else
		dev_warn(&pdev->dev, "Cluster ID out of bound\n");

	res_src = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res_wdt = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	res_tke = platform_get_resource(pdev, IORESOURCE_MEM, 2);

	if (!res_src || !res_wdt || !res_tke) {
		dev_err(&pdev->dev, "incorrect resources\n");
		return -ENOENT;
	}

	tegra_wdt_t18x->wdt_source = devm_ioremap_resource(&pdev->dev,
								res_src);
	if (IS_ERR(tegra_wdt_t18x->wdt_source)) {
		dev_err(&pdev->dev,
			"Cannot request memregion/iomap res_src\n");
		return PTR_ERR(tegra_wdt_t18x->wdt_source);
	}

	tegra_wdt_t18x->wdt_timer = devm_ioremap_resource(&pdev->dev, res_wdt);
	if (IS_ERR(tegra_wdt_t18x->wdt_timer)) {
		dev_err(&pdev->dev,
			"Cannot request memregion/iomap res_wdt\n");
		return PTR_ERR(tegra_wdt_t18x->wdt_timer);
	}

	tegra_wdt_t18x->wdt_tke = devm_ioremap(&pdev->dev,
			res_tke->start, resource_size(res_tke));
	if (IS_ERR(tegra_wdt_t18x->wdt_tke)) {
		dev_err(&pdev->dev,
			"Cannot request memregion/iomap res_tke\n");
		return PTR_ERR(tegra_wdt_t18x->wdt_tke);
	}

	/* Watchdog index in list of wdts under top_tke */
	index = ((res_src->start >> 16) & 0xF) - 0xc;

	/* Configure timer source and period */
	tegra_wdt_t18x->config = (((res_wdt->start >> 16) & (0xf)) - 2 ) |
								WDT_CFG_PERIOD;
	/* Enable local interrupt for WDT petting */
	tegra_wdt_t18x->config |= WDT_CFG_INT_EN;

	/* Enable local FIQ and remote interrupt for debug dump */
	if (tegra_wdt_t18x->cluster_id == WDT_CLUSTER_ID_DENVER)
		tegra_wdt_t18x->config |= WDT_CFG_FINT_EN;
	tegra_wdt_t18x->config |= WDT_CFG_REMOTE_INT_EN;

	/* Enable debug and POR reset if not explicitly disabled */
	if(!of_property_read_bool(np, "nvidia,disable-debug-reset"))
		tegra_wdt_t18x->config |= WDT_CFG_DBG_RST_EN;
	if(!of_property_read_bool(np, "nvidia,disable-por-reset"))
		tegra_wdt_t18x->config |= WDT_CFG_SYS_PORST_EN;

	tegra_wdt_t18x_disable(&tegra_wdt_t18x->wdt);
	writel(TOP_TKE_TMR_PCR_INTR, tegra_wdt_t18x->wdt_timer +
					TOP_TKE_TMR_PCR);

	/* Setup routing for WDT petting and enable when any one core
	 * is online. Disabled in low power state
	 */
	tegra_wdt_t18x_setup_pet(tegra_wdt_t18x, index);

	watchdog_init_timeout(&tegra_wdt_t18x->wdt,
			tegra_wdt_t18x->heartbeat,  &pdev->dev);

	ret = watchdog_register_device(&tegra_wdt_t18x->wdt);
	if (ret) {
		dev_err(&pdev->dev, "failed to register watchdog device\n");
		__tegra_wdt_t18x_disable(tegra_wdt_t18x);
		return ret;
	}

	platform_set_drvdata(pdev, tegra_wdt_t18x);

	tegra_wdt_t18x_debugfs_init(tegra_wdt_t18x);

	dev_info(&pdev->dev, "%s done\n", __func__);
	return 0;
}

static void tegra_wdt_t18x_shutdown(struct platform_device *pdev)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = platform_get_drvdata(pdev);

	__tegra_wdt_t18x_ping(tegra_wdt_t18x);
}

static int tegra_wdt_t18x_remove(struct platform_device *pdev)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = platform_get_drvdata(pdev);

	__tegra_wdt_t18x_disable(tegra_wdt_t18x);

	watchdog_unregister_device(&tegra_wdt_t18x->wdt);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_wdt_t18x_suspend(struct device *dev)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = dev_get_drvdata(dev);

	__tegra_wdt_t18x_disable(tegra_wdt_t18x);
	return 0;
}

static int tegra_wdt_t18x_resume(struct device *dev)
{
	struct tegra_wdt_t18x *tegra_wdt_t18x = dev_get_drvdata(dev);

	if (watchdog_active(&tegra_wdt_t18x->wdt))
		__tegra_wdt_t18x_enable(tegra_wdt_t18x);

	return 0;
}
#endif

static const struct dev_pm_ops tegra_wdt_t18x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_wdt_t18x_suspend, tegra_wdt_t18x_resume)
};

static const struct of_device_id tegra_wdt_t18x_match[] = {
	{ .compatible = "nvidia,tegra-wdt-t18x", },
	{}
};
MODULE_DEVICE_TABLE(of, tegra_wdt_t18x_match);

static struct platform_driver tegra_wdt_t18x_driver = {
	.probe		= tegra_wdt_t18x_probe,
	.remove		= tegra_wdt_t18x_remove,
	.shutdown	= tegra_wdt_t18x_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tegra_wdt_t18x",
		.pm	= &tegra_wdt_t18x_pm_ops,
		.of_match_table = of_match_ptr(tegra_wdt_t18x_match),
	},
};

static int __init tegra_wdt_t18x_init(void)
{
	return platform_driver_register(&tegra_wdt_t18x_driver);
}

static void __exit tegra_wdt_t18x_exit(void)
{
	platform_driver_unregister(&tegra_wdt_t18x_driver);
}

subsys_initcall(tegra_wdt_t18x_init);
module_exit(tegra_wdt_t18x_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra Watchdog Driver");

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tegra_wdt_t18x");
