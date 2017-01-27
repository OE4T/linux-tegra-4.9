/*
 * arch/arm/mach-tegra/powergate.c
 *
 * Copyright (c) 2010 Google, Inc
 * Copyright (c) 2011 - 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/tegra-powergate.h>
#include <soc/tegra/fuse.h>
#include <trace/events/power.h>
#include <asm/atomic.h>

#include "board.h"
#include "powergate-priv.h"

struct powergate_ops *pg_ops;

static inline bool tegra_powergate_check_skip_list(int id)
{
	return pg_ops->powergate_skip ?
		pg_ops->powergate_skip(id) : false;
}

/* EXTERNALY VISIBLE APIS */

bool tegra_powergate_is_powered(int id)
{
	if (!pg_ops) {
		pr_debug("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains)
		return -EINVAL;

	if (pg_ops->powergate_is_powered)
		return pg_ops->powergate_is_powered(id);

	return true;
}
EXPORT_SYMBOL(tegra_powergate_is_powered);

int tegra_cpu_powergate_id(int cpuid)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (cpuid < 0 || cpuid >= pg_ops->num_cpu_domains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (pg_ops->cpu_domains)
		return pg_ops->cpu_domains[cpuid];
	else
		WARN_ON_ONCE("This SOC does not support CPU powergate\n");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_cpu_powergate_id);

int tegra_powergate_partition(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (tegra_powergate_check_skip_list(id))
		printk_once("%s: %s is in powergate skip list\n", __func__,
			tegra_powergate_get_name(id));

	if (pg_ops->powergate_partition)
		return pg_ops->powergate_partition(id);
	else
		WARN_ON_ONCE("This SOC doesn't support powergating");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_partition);

int tegra_unpowergate_partition(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support un-powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (tegra_powergate_check_skip_list(id))
		printk_once("%s: %s is in powergate skip list\n", __func__,
			tegra_powergate_get_name(id));

	if (pg_ops->unpowergate_partition)
		return pg_ops->unpowergate_partition(id);
	else
		WARN_ON_ONCE("This SOC doesn't support un-powergating");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_unpowergate_partition);

int tegra_powergate_partition_with_clk_off(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (tegra_powergate_check_skip_list(id))
		printk_once("%s: %s is in powergate skip list\n", __func__,
			tegra_powergate_get_name(id));

	if (pg_ops->powergate_partition_with_clk_off)
		return pg_ops->powergate_partition_with_clk_off(id);
	else
		WARN_ON_ONCE("This SOC doesn't support powergating with clk off");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_partition_with_clk_off);

int tegra_unpowergate_partition_with_clk_on(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (tegra_powergate_check_skip_list(id))
		printk_once("%s: %s is in powergate skip list\n", __func__,
			tegra_powergate_get_name(id));

	if (pg_ops->unpowergate_partition_with_clk_on)
		return pg_ops->unpowergate_partition_with_clk_on(id);
	else
		WARN_ON_ONCE("This SOC doesn't support power un-gating with clk on");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_unpowergate_partition_with_clk_on);

int tegra_powergate_mc_enable(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (pg_ops->powergate_mc_enable)
		return pg_ops->powergate_mc_enable(id);
	else
		WARN_ON_ONCE("This SOC does not support powergate mc enable");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_mc_enable);

int tegra_powergate_mc_disable(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (pg_ops->powergate_mc_disable)
		return pg_ops->powergate_mc_disable(id);
	else
		WARN_ON_ONCE("This SOC does not support powergate mc disable");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_mc_disable);

int tegra_powergate_mc_flush(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (pg_ops->powergate_mc_flush)
		return pg_ops->powergate_mc_flush(id);
	else
		WARN_ON_ONCE("This SOC does not support powergate mc flush");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_mc_flush);

int tegra_powergate_mc_flush_done(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("%s: invalid powergate id\n", __func__);
		return -EINVAL;
	}

	if (pg_ops->powergate_mc_flush_done)
		return pg_ops->powergate_mc_flush_done(id);
	else
		WARN_ON_ONCE("This SOC does not support powergate mc flush done");

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_mc_flush_done);

const char *tegra_powergate_get_name(int id)
{
	if (!pg_ops) {
		WARN_ON_ONCE("This SOC doesn't support powergating\n");
		return NULL;
	}

	if (id < 0 || id >= pg_ops->num_powerdomains) {
		pr_info("invalid powergate id\n");
		return "invalid";
	}

	if (pg_ops->get_powergate_domain_name)
		return pg_ops->get_powergate_domain_name(id);
	else
		WARN_ON_ONCE("This SOC does not support CPU powergate");

	return "invalid";
}
EXPORT_SYMBOL(tegra_powergate_get_name);

static int tegra_powergate_init_refcount(void)
{
	if ((!pg_ops) || (!pg_ops->powergate_init_refcount))
		return 0;

	return pg_ops->powergate_init_refcount();
}

int __init tegra_powergate_init(void)
{
	switch (tegra_get_chip_id()) {
		case TEGRA210:
			pg_ops = tegra210_powergate_init_chip_support();
			break;

		default:
/*
 * TODO: Add correct chipid and remove this later
 */
#if defined(CONFIG_ARCH_TEGRA_18x_SOC)
			pg_ops = tegra186_powergate_init_chip_support();
#else
			pg_ops = NULL;
			pr_info("%s: Unknown Tegra variant. Disabling powergate\n", __func__);
#endif
			break;
	}

	tegra_powergate_init_refcount();

	pr_info("%s: DONE\n", __func__);

	return (pg_ops ? 0 : -EINVAL);
}
arch_initcall(tegra_powergate_init);

#ifdef CONFIG_DEBUG_FS

static int powergate_show(struct seq_file *s, void *data)
{
	int i;
	const char *name;
	bool is_pg_skip;

	if (!pg_ops) {
		seq_printf(s, "This SOC doesn't support powergating\n");
		return -EINVAL;
	}

	seq_printf(s, " powergate powered\n");
	seq_printf(s, "------------------\n");

	for (i = 0; i < pg_ops->num_powerdomains; i++) {
		name = tegra_powergate_get_name(i);
		if (name) {
			is_pg_skip = tegra_powergate_check_skip_list(i);
			seq_printf(s, " %9s %7s\n", name,
				(is_pg_skip ? "skip" : \
				(tegra_powergate_is_powered(i) ? \
				"yes" : "no")));
		}
	}

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open		= powergate_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *pg_debugfs_root;

static int state_set(void *data, u64 val)
{
	int ret;
	unsigned long id = (unsigned long)data;

	if (val)
		ret = tegra_unpowergate_partition(id);
	else
		ret = tegra_powergate_partition(id);

	return ret;
}

static int state_get(void *data, u64 *val)
{
	unsigned long id = (unsigned long)data;

	if (tegra_powergate_is_powered(id))
		*val = 1;
	else
		*val = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(state_fops, state_get, state_set, "%llu\n");

static int powergate_debugfs_register_one(unsigned long id, const char *name)
{
	struct dentry *dir, *d;

	dir = debugfs_create_dir(name, pg_debugfs_root);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file("state", S_IRUGO | S_IWUSR, dir, (void *)id, &state_fops);
	if (!d) {
		debugfs_remove_recursive(dir);
		return -ENOMEM;
	}

	return 0;
}

int __init tegra_powergate_debugfs_init(void)
{
	struct dentry *d;
	int i, ret;
	const char *name;

	if (!pg_ops)
		return -ENOMEM;

	d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
		&powergate_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_dir("pg_domains", NULL);
	if (!d)
		return -ENOMEM;

	pg_debugfs_root = d;

	for (i = 0; i < pg_ops->num_powerdomains; i++) {
		name = tegra_powergate_get_name(i);
		if (name) {
			ret = powergate_debugfs_register_one(i, name);
			if (ret)
				goto err_out;
		}
	}

	return 0;

err_out:
	debugfs_remove_recursive(pg_debugfs_root);
	return -ENOMEM;
}
late_initcall(tegra_powergate_debugfs_init);

#endif
