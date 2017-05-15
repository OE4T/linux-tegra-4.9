/*
 * arch/arm/mach-tegra/mcerr.c
 *
 * MC error code common to T3x and T11x. T20 has been left alone.
 *
 * Copyright (c) 2010-2017, NVIDIA Corporation. All rights reserved.
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

#define pr_fmt(fmt) "mc-err: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/atomic.h>

#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mcerr.h>
#include <linux/platform/tegra/tegra_emc_err.h>

static bool mcerr_throttle_enabled = true;
u32  mcerr_silenced;
static atomic_t error_count;

static void unthrottle_prints(struct work_struct *work);
static DECLARE_DELAYED_WORK(unthrottle_prints_work, unthrottle_prints);
static struct dentry *mcerr_debugfs_dir;
u32 mc_int_mask;

/*
 * Chip specific functions.
 */
static struct mcerr_chip_specific chip_specific;
static struct mcerr_chip_specific *cs_ops;

static void unthrottle_prints(struct work_struct *work)
{
	atomic_set(&error_count, 0);
}

static void disable_interrupt(unsigned int irq)
{
	mc_writel(0, MC_INTMASK);
}

static void enable_interrupt(unsigned int irq)
{
	mc_writel(mc_int_mask, MC_INTMASK);
}

static irqreturn_t tegra_mcerr_thread(int irq, void *data)
{
	unsigned long count;

	cancel_delayed_work(&unthrottle_prints_work);
	count = atomic_inc_return(&error_count);

	if (mcerr_throttle_enabled && count >= MAX_PRINTS) {
		schedule_delayed_work(&unthrottle_prints_work, HZ/2);
		if (count == MAX_PRINTS)
			mcerr_pr("Too many MC errors; throttling prints\n");
		cs_ops->clear_interrupt(irq);
		goto exit;
	}

	cs_ops->log_mcerr_fault(irq);
exit:
	cs_ops->enable_interrupt(irq);

	return IRQ_HANDLED;
}

/*
 * The actual error handling takes longer than is ideal so this must be
 * threaded.
 */
static irqreturn_t tegra_mcerr_hard_irq(int irq, void *data)
{
	trace_printk("MCERR detected.\n");
	 /*
	  * Disable MC Error interrupt till the MC Error info is logged.
	  * MC Errors can be lost as MC HW holds one MC error at a time.
	  * The first MC Error is good enough to point out potential memory
	  * access issues in SW and allow debugging further.
	  */
	cs_ops->disable_interrupt(irq);
	return IRQ_WAKE_THREAD;
}

/*
 * Print the MC err stats for each client.
 */
static int mcerr_default_debugfs_show(struct seq_file *s, void *v)
{
	int i, j;
	int do_print;

	seq_printf(s, "%-18s %-18s", "swgroup", "client");
	for (i = 0; i < (sizeof(u32) * 8); i++) {
		if (chip_specific.intr_descriptions[i])
			seq_printf(s, " %-12s",
				   chip_specific.intr_descriptions[i]);
	}
	seq_puts(s, "\n");

	for (i = 0; i < chip_specific.nr_clients; i++) {
		do_print = 0;

		/* Only print clients who actually have errors. */
		for (j = 0; j < (sizeof(u32) * 8); j++) {
			if (chip_specific.intr_descriptions[j] &&
			    mc_clients[i].intr_counts[j]) {
				do_print = 1;
				break;
			}
		}

		if (do_print) {
			seq_printf(s, "%-18s %-18s",
				   mc_clients[i].name,
				   mc_clients[i].swgroup);
			for (j = 0; j < (sizeof(u32) * 8); j++) {
				if (!chip_specific.intr_descriptions[j])
					continue;
				seq_printf(s, " %-12u",
					   mc_clients[i].intr_counts[j]);
			}
			seq_puts(s, "\n");
		}
	}

	return 0;
}

static int mcerr_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, chip_specific.mcerr_debugfs_show, NULL);
}

static const struct file_operations mcerr_debugfs_fops = {
	.open           = mcerr_debugfs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __get_throttle(void *data, u64 *val)
{
	*val = mcerr_throttle_enabled;
	return 0;
}

static int __set_throttle(void *data, u64 val)
{
	atomic_set(&error_count, 0);

	mcerr_throttle_enabled = (bool) val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mcerr_throttle_debugfs_fops, __get_throttle,
			__set_throttle, "%llu\n");

/*
 * This will always be successful. However, if something goes wrong in the
 * init a message will be printed to the kernel log. Since this is a
 * non-essential piece of the kernel no reason to fail the entire MC init
 * if this fails.
 */
int tegra_mcerr_init(struct dentry *mc_parent, struct platform_device *pdev)
{
	int irq;
	const void *prop;

	chip_specific.enable_interrupt   = enable_interrupt;
	chip_specific.disable_interrupt  = disable_interrupt;
	chip_specific.mcerr_debugfs_show = mcerr_default_debugfs_show;

	/*
	 * mcerr_chip_specific_setup() can override any of the default
	 * functions as it wishes.
	 */
	mcerr_chip_specific_setup(&chip_specific);
	if (chip_specific.nr_clients == 0 ||
	    chip_specific.intr_descriptions == NULL) {
		pr_err("Missing necessary chip_specific functionality!\n");
		return -ENODEV;
	}

	prop = of_get_property(pdev->dev.of_node, "int_mask", NULL);
	if (!prop) {
		pr_err("No int_mask prop for mcerr!\n");
		return -EINVAL;
	}

	mc_int_mask = be32_to_cpup(prop);
	mc_writel(mc_int_mask, MC_INTMASK);
	pr_debug("Set intmask: 0x%x\n", mc_readl(MC_INTMASK));

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq < 0) {
		pr_err("Unable to parse/map MC error interrupt\n");
		goto done;
	}

	if (request_threaded_irq(irq, tegra_mcerr_hard_irq,
				 tegra_mcerr_thread, 0, "mc_status", NULL)) {
		pr_err("Unable to register MC error interrupt\n");
		goto done;
	}

	tegra_emcerr_init(mc_parent, pdev);

	if (!mc_parent)
		goto done;

	mcerr_debugfs_dir = debugfs_create_dir("err", mc_parent);
	if (mcerr_debugfs_dir == NULL) {
		pr_err("Failed to make debugfs node: %ld\n",
		       PTR_ERR(mcerr_debugfs_dir));
		goto done;
	}
	debugfs_create_file("mcerr", 0644, mcerr_debugfs_dir, NULL,
			    &mcerr_debugfs_fops);
	debugfs_create_file("mcerr_throttle", S_IRUGO | S_IWUSR,
			    mcerr_debugfs_dir, NULL,
			    &mcerr_throttle_debugfs_fops);
	debugfs_create_u32("quiet", 0644, mcerr_debugfs_dir, &mcerr_silenced);
done:
	return 0;
}
