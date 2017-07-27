/*
 * Copyright (C) 2017 NVIDIA Corporation.  All rights reserved.
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

#include "debug_cde.h"
#include "debug_ce.h"
#include "debug_fifo.h"
#include "debug_gr.h"
#include "debug_mm.h"
#include "debug_allocator.h"
#include "debug_kmem.h"
#include "debug_pmu.h"
#include "debug_sched.h"
#include "os_linux.h"

#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <nvgpu/debug.h>

unsigned int gk20a_debug_trace_cmdbuf;

static inline void gk20a_debug_write_printk(void *ctx, const char *str,
					    size_t len)
{
	pr_info("%s", str);
}

static inline void gk20a_debug_write_to_seqfile(void *ctx, const char *str,
						size_t len)
{
	seq_write((struct seq_file *)ctx, str, len);
}

void gk20a_debug_output(struct gk20a_debug_output *o,
					const char *fmt, ...)
{
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(o->buf, sizeof(o->buf), fmt, args);
	va_end(args);
	o->fn(o->ctx, o->buf, len);
}

static int gk20a_gr_dump_regs(struct gk20a *g,
		struct gk20a_debug_output *o)
{
	if (g->ops.gr.dump_gr_regs)
		gr_gk20a_elpg_protected_call(g, g->ops.gr.dump_gr_regs(g, o));

	return 0;
}

int gk20a_gr_debug_dump(struct gk20a *g)
{
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	gk20a_gr_dump_regs(g, &o);

	return 0;
}

static int gk20a_gr_debug_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct gk20a *g = gk20a_get_platform(dev)->g;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};
	int err;

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu: %d", err);
		return -EINVAL;
	}

	gk20a_gr_dump_regs(g, &o);

	gk20a_idle(g);

	return 0;
}

void gk20a_debug_dump(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	if (platform->dump_platform_dependencies)
		platform->dump_platform_dependencies(dev_from_gk20a(g));

	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);
}

static int gk20a_debug_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};
	struct gk20a *g;
	int err;

	g = gk20a_get_platform(dev)->g;

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu: %d", err);
		return -EFAULT;
	}

	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);

	gk20a_idle(g);
	return 0;
}

static int gk20a_gr_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_gr_debug_show, inode->i_private);
}

static int gk20a_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_debug_show, inode->i_private);
}

static const struct file_operations gk20a_gr_debug_fops = {
	.open		= gk20a_gr_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations gk20a_debug_fops = {
	.open		= gk20a_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void gk20a_debug_show_dump(struct gk20a *g, struct gk20a_debug_output *o)
{
	g->ops.fifo.dump_pbdma_status(g, o);
	g->ops.fifo.dump_eng_status(g, o);

	gk20a_debug_dump_all_channel_status_ramfc(g, o);
}

static int railgate_residency_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));
	unsigned long time_since_last_state_transition_ms;
	unsigned long total_rail_gate_time_ms;
	unsigned long total_rail_ungate_time_ms;

	if (platform->is_railgated(dev_from_gk20a(g))) {
		time_since_last_state_transition_ms =
				jiffies_to_msecs(jiffies -
				g->pstats.last_rail_gate_complete);
		total_rail_ungate_time_ms = g->pstats.total_rail_ungate_time_ms;
		total_rail_gate_time_ms =
					g->pstats.total_rail_gate_time_ms +
					time_since_last_state_transition_ms;
	} else {
		time_since_last_state_transition_ms =
				jiffies_to_msecs(jiffies -
				g->pstats.last_rail_ungate_complete);
		total_rail_gate_time_ms = g->pstats.total_rail_gate_time_ms;
		total_rail_ungate_time_ms =
					g->pstats.total_rail_ungate_time_ms +
					time_since_last_state_transition_ms;
	}

	seq_printf(s, "Time with Rails Gated: %lu ms\n"
			"Time with Rails UnGated: %lu ms\n"
			"Total railgating cycles: %lu\n",
			total_rail_gate_time_ms,
			total_rail_ungate_time_ms,
			g->pstats.railgating_cycle_count - 1);
	return 0;

}

static int railgate_residency_open(struct inode *inode, struct file *file)
{
	return single_open(file, railgate_residency_show, inode->i_private);
}

static const struct file_operations railgate_residency_fops = {
	.open		= railgate_residency_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int gk20a_railgating_debugfs_init(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));
	struct dentry *d;

	if (!g->can_railgate)
		return 0;

	d = debugfs_create_file(
		"railgate_residency", S_IRUGO|S_IWUSR, platform->debugfs, g,
						&railgate_residency_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

void gk20a_debug_init(struct gk20a *g, const char *debugfs_symlink)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = dev_get_drvdata(dev);

	platform->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	if (!platform->debugfs)
		return;

	if (debugfs_symlink)
		platform->debugfs_alias =
			debugfs_create_symlink(debugfs_symlink,
					NULL, dev_name(dev));

	debugfs_create_file("status", S_IRUGO, platform->debugfs,
		dev, &gk20a_debug_fops);
	debugfs_create_file("gr_status", S_IRUGO, platform->debugfs,
		dev, &gk20a_gr_debug_fops);
	debugfs_create_u32("trace_cmdbuf", S_IRUGO|S_IWUSR,
		platform->debugfs, &gk20a_debug_trace_cmdbuf);

	debugfs_create_u32("ch_wdt_timeout_ms", S_IRUGO|S_IWUSR,
		platform->debugfs, &g->ch_wdt_timeout_ms);

	debugfs_create_u32("disable_syncpoints", S_IRUGO|S_IWUSR,
		platform->debugfs, &g->disable_syncpoints);

	/* Legacy debugging API. */
	debugfs_create_u32("dbg_mask", S_IRUGO|S_IWUSR,
		platform->debugfs, &nvgpu_dbg_mask);

	/* New debug logging API. */
	debugfs_create_u32("log_mask", S_IRUGO|S_IWUSR,
		platform->debugfs, &g->log_mask);
	debugfs_create_u32("log_trace", S_IRUGO|S_IWUSR,
		platform->debugfs, &g->log_trace);

	g->debugfs_ltc_enabled =
			debugfs_create_bool("ltc_enabled", S_IRUGO|S_IWUSR,
				 platform->debugfs,
				 &g->mm.ltc_enabled_target);

	g->debugfs_gr_idle_timeout_default =
			debugfs_create_u32("gr_idle_timeout_default_us",
					S_IRUGO|S_IWUSR, platform->debugfs,
					 &g->gr_idle_timeout_default);
	g->debugfs_timeouts_enabled =
			debugfs_create_bool("timeouts_enabled",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->timeouts_enabled);

	g->debugfs_bypass_smmu =
			debugfs_create_bool("bypass_smmu",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->mm.bypass_smmu);
	g->debugfs_disable_bigpage =
			debugfs_create_bool("disable_bigpage",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->mm.disable_bigpage);

	g->debugfs_timeslice_low_priority_us =
			debugfs_create_u32("timeslice_low_priority_us",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->timeslice_low_priority_us);
	g->debugfs_timeslice_medium_priority_us =
			debugfs_create_u32("timeslice_medium_priority_us",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->timeslice_medium_priority_us);
	g->debugfs_timeslice_high_priority_us =
			debugfs_create_u32("timeslice_high_priority_us",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->timeslice_high_priority_us);
	g->debugfs_runlist_interleave =
			debugfs_create_bool("runlist_interleave",
					S_IRUGO|S_IWUSR,
					platform->debugfs,
					&g->runlist_interleave);
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	g->gr.t18x.ctx_vars.debugfs_force_preemption_gfxp =
		debugfs_create_bool("force_preemption_gfxp", S_IRUGO|S_IWUSR,
		platform->debugfs,
		&g->gr.t18x.ctx_vars.force_preemption_gfxp);

	g->gr.t18x.ctx_vars.debugfs_force_preemption_cilp =
		debugfs_create_bool("force_preemption_cilp", S_IRUGO|S_IWUSR,
		platform->debugfs,
		&g->gr.t18x.ctx_vars.force_preemption_cilp);

	g->gr.t18x.ctx_vars.debugfs_dump_ctxsw_stats =
		debugfs_create_bool("dump_ctxsw_stats_on_channel_close",
			S_IRUGO|S_IWUSR, platform->debugfs,
			&g->gr.t18x.
				ctx_vars.dump_ctxsw_stats_on_channel_close);
#endif

	gr_gk20a_debugfs_init(g);
	gk20a_pmu_debugfs_init(g);
	gk20a_railgating_debugfs_init(g);
	gk20a_cde_debugfs_init(g);
	gk20a_ce_debugfs_init(g);
	nvgpu_alloc_debugfs_init(g);
	gk20a_mm_debugfs_init(g);
	gk20a_fifo_debugfs_init(g);
	gk20a_sched_debugfs_init(g);
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	nvgpu_kmem_debugfs_init(g);
#endif
}

void gk20a_debug_deinit(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));

	if (!platform->debugfs)
		return;

	gk20a_fifo_debugfs_deinit(g);

	debugfs_remove_recursive(platform->debugfs);
	debugfs_remove_recursive(platform->debugfs_alias);
}
