/*
 * Copyright (C) 2011-2017 NVIDIA Corporation.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/io.h>

#include <nvgpu/log.h>
#include <nvgpu/kmem.h>
#include <nvgpu/semaphore.h>

#include "gk20a.h"
#include "debug_gk20a.h"

#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_fifo_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

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

static void gk20a_debug_dump_all_channel_status_ramfc(struct gk20a *g,
		 struct gk20a_debug_output *o)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	struct ch_state **ch_state;

	ch_state = nvgpu_kzalloc(g, sizeof(*ch_state) * f->num_channels);
	if (!ch_state) {
		gk20a_debug_output(o, "cannot alloc memory for channels\n");
		return;
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (gk20a_channel_get(ch)) {
			ch_state[chid] =
				nvgpu_kmalloc(g, sizeof(struct ch_state) +
					ram_in_alloc_size_v());
			/* ref taken stays to below loop with
			 * successful allocs */
			if (!ch_state[chid])
				gk20a_channel_put(ch);
		}
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (!ch_state[chid])
			continue;

		ch_state[chid]->pid = ch->pid;
		ch_state[chid]->refs = atomic_read(&ch->ref_count);
		gk20a_mem_rd_n(g, &ch->inst_block, 0,
				&ch_state[chid]->inst_block[0],
				ram_in_alloc_size_v());
		gk20a_channel_put(ch);
	}
	for (chid = 0; chid < f->num_channels; chid++) {
		if (ch_state[chid]) {
			g->ops.fifo.dump_channel_status_ramfc(g, o, chid,
						 ch_state[chid]);
			nvgpu_kfree(g, ch_state[chid]);
		}
	}
	nvgpu_kfree(g, ch_state);
}

void gk20a_debug_show_dump(struct gk20a *g, struct gk20a_debug_output *o)
{
	g->ops.fifo.dump_pbdma_status(g, o);
	g->ops.fifo.dump_eng_status(g, o);

	gk20a_debug_dump_all_channel_status_ramfc(g, o);
}

static int gk20a_gr_dump_regs(struct device *dev,
		struct gk20a_debug_output *o)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;

	if (g->ops.gr.dump_gr_regs)
		gr_gk20a_elpg_protected_call(g, g->ops.gr.dump_gr_regs(g, o));

	return 0;
}

int gk20a_gr_debug_dump(struct device *dev)
{
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	gk20a_gr_dump_regs(dev, &o);

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
		gk20a_err(dev, "failed to power on gpu: %d", err);
		return -EINVAL;
	}

	gk20a_gr_dump_regs(dev, &o);

	gk20a_idle(g);

	return 0;
}

void gk20a_debug_dump(struct device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct gk20a *g = platform->g;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	if (platform->dump_platform_dependencies)
		platform->dump_platform_dependencies(dev);

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
		gk20a_err(g->dev, "failed to power on gpu: %d", err);
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

void gk20a_init_debug_ops(struct gpu_ops *gops)
{
	gops->debug.show_dump = gk20a_debug_show_dump;
}

void gk20a_debug_init(struct device *dev, const char *debugfs_symlink)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
#ifdef CONFIG_DEBUG_FS
	struct gk20a *g = platform->g;

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
		platform->debugfs, &platform->ch_wdt_timeout_ms);

	debugfs_create_bool("disable_syncpoints", S_IRUGO|S_IWUSR,
		platform->debugfs, &platform->disable_syncpoints);

	/* Legacy debugging API. */
	debugfs_create_u32("dbg_mask", S_IRUGO|S_IWUSR,
		platform->debugfs, &nvgpu_dbg_mask);

	/* New debug logging API. */
	debugfs_create_u32("log_mask", S_IRUGO|S_IWUSR,
		platform->debugfs, &g->log_mask);
	debugfs_create_u32("log_trace", S_IRUGO|S_IWUSR,
		platform->debugfs, &g->log_trace);

	nvgpu_spinlock_init(&g->debugfs_lock);

	g->mm.ltc_enabled = true;
	g->mm.ltc_enabled_debug = true;

	g->debugfs_ltc_enabled =
			debugfs_create_bool("ltc_enabled", S_IRUGO|S_IWUSR,
				 platform->debugfs,
				 &g->mm.ltc_enabled_debug);

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

	gr_gk20a_debugfs_init(g);
	gk20a_pmu_debugfs_init(g->dev);
	gk20a_railgating_debugfs_init(g->dev);
	gk20a_cde_debugfs_init(g->dev);
	gk20a_ce_debugfs_init(g->dev);
	nvgpu_alloc_debugfs_init(g->dev);
	gk20a_mm_debugfs_init(g->dev);
	gk20a_fifo_debugfs_init(g->dev);
	gk20a_sched_debugfs_init(g->dev);
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	nvgpu_kmem_debugfs_init(g->dev);
#endif
#endif

}
