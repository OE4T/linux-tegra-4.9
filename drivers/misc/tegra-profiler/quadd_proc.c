/*
 * drivers/misc/tegra-profiler/quadd_proc.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
 */

#ifdef CONFIG_PROC_FS

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/tegra_profiler.h>

#include "quadd.h"
#include "version.h"
#include "quadd_proc.h"

#define YES_NO(x) ((x) ? "yes" : "no")

static struct quadd_ctx *ctx;

static int show_version(struct seq_file *f, void *offset)
{
	seq_printf(f, "version:         %s\n", QUADD_MODULE_VERSION);
	seq_printf(f, "branch:          %s\n", QUADD_MODULE_BRANCH);
	seq_printf(f, "samples version: %d\n", QUADD_SAMPLES_VERSION);
	seq_printf(f, "io version:      %d\n", QUADD_IO_VERSION);

	return 0;
}

static int show_version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_version, NULL);
}

static const struct file_operations version_proc_fops = {
	.open		= show_version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_capabilities(struct seq_file *f, void *offset)
{
	struct quadd_comm_cap *cap = &ctx->cap;
	struct quadd_events_cap *event = &cap->events_cap;

	seq_printf(f, "pmu:                    %s\n",
		   YES_NO(cap->pmu));
	seq_printf(f, "tegra 3 LP cluster:     %s\n",
		   YES_NO(cap->tegra_lp_cluster));
	seq_printf(f, "power rate samples:     %s\n",
		   YES_NO(cap->power_rate));

	seq_printf(f, "l2 cache:               %s\n",
		   YES_NO(cap->l2_cache));
	if (cap->l2_cache) {
		seq_printf(f, "Multiple l2 events:     %s\n",
			   YES_NO(cap->l2_multiple_events));
	}

	seq_printf(f, "\n");
	seq_printf(f, "Supported events:\n");
	seq_printf(f, "cpu_cycles:             %s\n",
		   YES_NO(event->cpu_cycles));
	seq_printf(f, "instructions:           %s\n",
		   YES_NO(event->instructions));
	seq_printf(f, "branch_instructions:    %s\n",
		   YES_NO(event->branch_instructions));
	seq_printf(f, "branch_misses:          %s\n",
		   YES_NO(event->branch_misses));
	seq_printf(f, "bus_cycles:             %s\n",
		   YES_NO(event->bus_cycles));
	seq_printf(f, "l1_dcache_read_misses:  %s\n",
		   YES_NO(event->l1_dcache_read_misses));
	seq_printf(f, "l1_dcache_write_misses: %s\n",
		   YES_NO(event->l1_dcache_write_misses));
	seq_printf(f, "l1_icache_misses:       %s\n",
		   YES_NO(event->l1_icache_misses));
	seq_printf(f, "l2_dcache_read_misses:  %s\n",
		   YES_NO(event->l2_dcache_read_misses));
	seq_printf(f, "l2_dcache_write_misses: %s\n",
		   YES_NO(event->l2_dcache_write_misses));
	seq_printf(f, "l2_icache_misses:       %s\n",
		   YES_NO(event->l2_icache_misses));

	return 0;
}

static int show_capabilities_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_capabilities, NULL);
}

static const struct file_operations capabilities_proc_fops = {
	.open		= show_capabilities_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void quadd_proc_init(struct quadd_ctx *context)
{
	ctx = context;

	proc_mkdir("quadd", NULL);
	proc_create("quadd/version", 0, NULL, &version_proc_fops);
	proc_create("quadd/capabilities", 0, NULL, &capabilities_proc_fops);
}

void quadd_proc_deinit(void)
{
	remove_proc_entry("quadd/version", NULL);
	remove_proc_entry("quadd/capabilities", NULL);
	remove_proc_entry("quadd", NULL);
}

#endif	/* CONFIG_PROC_FS */
