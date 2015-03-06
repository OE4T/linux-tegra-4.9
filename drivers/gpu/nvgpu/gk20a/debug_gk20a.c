/*
 * drivers/video/tegra/host/t20/debug_gk20a.c
 *
 * Copyright (C) 2011-2015 NVIDIA Corporation.  All rights reserved.
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

#ifdef CONFIG_TEGRA_GK20A
#include <linux/nvhost.h>
#endif

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/io.h>

#include "gk20a.h"
#include "debug_gk20a.h"

#include "hw_ram_gk20a.h"
#include "hw_fifo_gk20a.h"
#include "hw_ccsr_gk20a.h"
#include "hw_pbdma_gk20a.h"

unsigned int gk20a_debug_trace_cmdbuf;
static struct platform_device *gk20a_device;

struct ch_state {
	int pid;
	int refs;
	u8 inst_block[0];
};

static const char * const ccsr_chan_status_str[] = {
	"idle",
	"pending",
	"pending_ctx_reload",
	"pending_acquire",
	"pending_acq_ctx_reload",
	"on_pbdma",
	"on_pbdma_and_eng",
	"on_eng",
	"on_eng_pending_acquire",
	"on_eng_pending",
	"on_pbdma_ctx_reload",
	"on_pbdma_and_eng_ctx_reload",
	"on_eng_ctx_reload",
	"on_eng_pending_ctx_reload",
	"on_eng_pending_acq_ctx_reload",
};

static const char * const chan_status_str[] = {
	"invalid",
	"valid",
	"chsw_load",
	"chsw_save",
	"chsw_switch",
};

static const char * const ctx_status_str[] = {
	"invalid",
	"valid",
	NULL,
	NULL,
	NULL,
	"ctxsw_load",
	"ctxsw_save",
	"ctxsw_switch",
};

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

static void gk20a_debug_show_channel(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 hw_chid,
				     struct ch_state *ch_state)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(hw_chid));
	u32 status = ccsr_channel_status_v(channel);
	u32 syncpointa, syncpointb;
	void *inst_ptr;

	if (!ch_state)
		return;

	inst_ptr = &ch_state->inst_block[0];

	syncpointa = gk20a_mem_rd32(inst_ptr, ram_fc_syncpointa_w());
	syncpointb = gk20a_mem_rd32(inst_ptr, ram_fc_syncpointb_w());

	gk20a_debug_output(o, "%d-%s, pid %d, refs: %d: ", hw_chid,
			g->dev->name,
			ch_state->pid,
			ch_state->refs);
	gk20a_debug_output(o, "%s in use %s %s\n",
			ccsr_channel_enable_v(channel) ? "" : "not",
			ccsr_chan_status_str[status],
			ccsr_channel_busy_v(channel) ? "busy" : "not busy");
	gk20a_debug_output(o, "TOP: %016llx PUT: %016llx GET: %016llx "
			"FETCH: %016llx\nHEADER: %08x COUNT: %08x\n"
			"SYNCPOINT %08x %08x SEMAPHORE %08x %08x %08x %08x\n",
		(u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_top_level_get_w()) +
		((u64)gk20a_mem_rd32(inst_ptr,
			ram_fc_pb_top_level_get_hi_w()) << 32ULL),
		(u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_put_w()) +
		((u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_put_hi_w()) << 32ULL),
		(u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_get_w()) +
		((u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_get_hi_w()) << 32ULL),
		(u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_fetch_w()) +
		((u64)gk20a_mem_rd32(inst_ptr, ram_fc_pb_fetch_hi_w()) << 32ULL),
		gk20a_mem_rd32(inst_ptr, ram_fc_pb_header_w()),
		gk20a_mem_rd32(inst_ptr, ram_fc_pb_count_w()),
		syncpointa,
		syncpointb,
		gk20a_mem_rd32(inst_ptr, ram_fc_semaphorea_w()),
		gk20a_mem_rd32(inst_ptr, ram_fc_semaphoreb_w()),
		gk20a_mem_rd32(inst_ptr, ram_fc_semaphorec_w()),
		gk20a_mem_rd32(inst_ptr, ram_fc_semaphored_w()));

#ifdef CONFIG_TEGRA_GK20A
	if ((pbdma_syncpointb_op_v(syncpointb) == pbdma_syncpointb_op_wait_v())
		&& (pbdma_syncpointb_wait_switch_v(syncpointb) ==
			pbdma_syncpointb_wait_switch_en_v()))
		gk20a_debug_output(o, "%s on syncpt %u (%s) val %u\n",
			(status == 3 || status == 8) ? "Waiting" : "Waited",
			pbdma_syncpointb_syncpt_index_v(syncpointb),
			nvhost_syncpt_get_name(g->host1x_dev,
				pbdma_syncpointb_syncpt_index_v(syncpointb)),
			pbdma_syncpointa_payload_v(syncpointa));
#endif

	gk20a_debug_output(o, "\n");
}

void gk20a_debug_show_dump(struct gk20a *g, struct gk20a_debug_output *o)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	int i, err;

	struct ch_state **ch_state;

	err = gk20a_busy(g->dev);
	if (err) {
		gk20a_debug_output(o, "failed to power on gpu: %d\n", err);
		return;
	}

	for (i = 0; i < fifo_pbdma_status__size_1_v(); i++) {
		u32 status = gk20a_readl(g, fifo_pbdma_status_r(i));
		u32 chan_status = fifo_pbdma_status_chan_status_v(status);

		gk20a_debug_output(o, "%s pbdma %d: ", g->dev->name, i);
		gk20a_debug_output(o,
				"id: %d (%s), next_id: %d (%s) status: %s\n",
				fifo_pbdma_status_id_v(status),
				fifo_pbdma_status_id_type_v(status) ?
					"tsg" : "channel",
				fifo_pbdma_status_next_id_v(status),
				fifo_pbdma_status_next_id_type_v(status) ?
					"tsg" : "channel",
				chan_status_str[chan_status]);
		gk20a_debug_output(o, "PUT: %016llx GET: %016llx "
				"FETCH: %08x HEADER: %08x\n",
			(u64)gk20a_readl(g, pbdma_put_r(i)) +
			((u64)gk20a_readl(g, pbdma_put_hi_r(i)) << 32ULL),
			(u64)gk20a_readl(g, pbdma_get_r(i)) +
			((u64)gk20a_readl(g, pbdma_get_hi_r(i)) << 32ULL),
			gk20a_readl(g, pbdma_gp_fetch_r(i)),
			gk20a_readl(g, pbdma_pb_header_r(i)));
	}
	gk20a_debug_output(o, "\n");

	for (i = 0; i < fifo_engine_status__size_1_v(); i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);

		gk20a_debug_output(o, "%s eng %d: ", g->dev->name, i);
		gk20a_debug_output(o,
				"id: %d (%s), next_id: %d (%s), ctx: %s ",
				fifo_engine_status_id_v(status),
				fifo_engine_status_id_type_v(status) ?
					"tsg" : "channel",
				fifo_engine_status_next_id_v(status),
				fifo_engine_status_next_id_type_v(status) ?
					"tsg" : "channel",
				ctx_status_str[ctx_status]);

		if (fifo_engine_status_faulted_v(status))
			gk20a_debug_output(o, "faulted ");
		if (fifo_engine_status_engine_v(status))
			gk20a_debug_output(o, "busy ");
		gk20a_debug_output(o, "\n");
	}
	gk20a_debug_output(o, "\n");

	ch_state = kzalloc(sizeof(*ch_state)
				 * f->num_channels, GFP_KERNEL);
	if (!ch_state) {
		gk20a_debug_output(o, "cannot alloc memory for channels\n");
		goto done;
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (gk20a_channel_get(ch)) {
			ch_state[chid] =
				kmalloc(sizeof(struct ch_state) +
					ram_in_alloc_size_v(), GFP_KERNEL);
			/* ref taken stays to below loop with
			 * successful allocs */
			if (!ch_state[chid])
				gk20a_channel_put(ch);
		}
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (ch_state[chid]) {
			if (ch->inst_block.cpu_va) {
				ch_state[chid]->pid = ch->pid;
				ch_state[chid]->refs =
					atomic_read(&ch->ref_count);
				memcpy(&ch_state[chid]->inst_block[0],
						ch->inst_block.cpu_va,
						ram_in_alloc_size_v());
			}
			gk20a_channel_put(ch);
		}
	}
	for (chid = 0; chid < f->num_channels; chid++) {
		if (ch_state[chid]) {
			gk20a_debug_show_channel(g, o, chid, ch_state[chid]);
			kfree(ch_state[chid]);
		}
	}
	kfree(ch_state);
done:
	gk20a_idle(g->dev);
}

static int gk20a_gr_dump_regs(struct platform_device *pdev,
		struct gk20a_debug_output *o)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a *g = platform->g;
	int err;

	err = gk20a_busy(g->dev);
	if (err) {
		gk20a_err(&pdev->dev, "failed to power on gpu: %d\n", err);
		return -EINVAL;
	}

	gr_gk20a_elpg_protected_call(g, g->ops.gr.dump_gr_regs(g, o));

	gk20a_idle(g->dev);

	return 0;
}

int gk20a_gr_debug_dump(struct platform_device *pdev)
{
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	gk20a_gr_dump_regs(pdev, &o);

	return 0;
}

static int gk20a_gr_debug_show(struct seq_file *s, void *unused)
{
	struct platform_device *pdev = s->private;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};

	gk20a_gr_dump_regs(pdev, &o);

	return 0;
}

void gk20a_debug_dump(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a *g = platform->g;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	if (platform->dump_platform_dependencies)
		platform->dump_platform_dependencies(pdev);

	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);
}

void gk20a_debug_dump_device(struct platform_device *pdev)
{
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};
	struct gk20a *g;

	/* In pre-silicon we don't need full spew on stuck syncpoint */
	if (!tegra_platform_is_silicon())
		return;

	/* Dump the first device if no info is provided */
	if (!pdev) {
		if (!gk20a_device)
			return;

		pdev = gk20a_device;
	}

	g = gk20a_get_platform(pdev)->g;
	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);
}
EXPORT_SYMBOL(gk20a_debug_dump_device);

static int gk20a_debug_show(struct seq_file *s, void *unused)
{
	struct platform_device *pdev = s->private;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};
	struct gk20a *g;

	g = gk20a_get_platform(pdev)->g;
	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);
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

void gk20a_debug_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);

	/* Store the first device */
	if (!gk20a_device)
		gk20a_device = pdev;

	platform->debugfs = debugfs_create_dir(pdev->name, NULL);

	debugfs_create_file("status", S_IRUGO, platform->debugfs,
			pdev, &gk20a_debug_fops);
	debugfs_create_file("gr_status", S_IRUGO, platform->debugfs,
			pdev, &gk20a_gr_debug_fops);
	debugfs_create_u32("trace_cmdbuf", S_IRUGO|S_IWUSR, platform->debugfs,
			&gk20a_debug_trace_cmdbuf);

#if defined(GK20A_DEBUG)
	debugfs_create_u32("dbg_mask", S_IRUGO|S_IWUSR, platform->debugfs,
			&gk20a_dbg_mask);
	debugfs_create_u32("dbg_ftrace", S_IRUGO|S_IWUSR, platform->debugfs,
			&gk20a_dbg_ftrace);
#endif
}
