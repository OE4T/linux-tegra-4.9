/*
 * drivers/video/tegra/host/t20/debug_gk20a.c
 *
 * Copyright (C) 2011-2014 NVIDIA Corporation.  All rights reserved.
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

#include <linux/nvhost.h>
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
struct platform_device *gk20a_device;

struct gk20a_debug_output {
	void (*fn)(void *ctx, const char *str, size_t len);
	void *ctx;
	char buf[256];
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

void gk20a_debug_output(struct gk20a_debug_output *o, const char *fmt, ...)
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
				     struct channel_gk20a *ch)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(ch->hw_chid));
	u32 status = ccsr_channel_status_v(channel);
	u32 syncpointa, syncpointb;
	void *inst_ptr;

	inst_ptr = ch->inst_block.cpuva;
	if (!inst_ptr)
		return;

	syncpointa = gk20a_mem_rd32(inst_ptr, ram_fc_syncpointa_w());
	syncpointb = gk20a_mem_rd32(inst_ptr, ram_fc_syncpointb_w());

	gk20a_debug_output(o, "%d-%s, pid %d: ", ch->hw_chid,
			ch->g->dev->name,
			ch->pid);
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

	if ((pbdma_syncpointb_op_v(syncpointb) == pbdma_syncpointb_op_wait_v())
		&& (pbdma_syncpointb_wait_switch_v(syncpointb) ==
			pbdma_syncpointb_wait_switch_en_v()))
		gk20a_debug_output(o, "%s on syncpt %u (%s) val %u\n",
			(status == 3 || status == 8) ? "Waiting" : "Waited",
			pbdma_syncpointb_syncpt_index_v(syncpointb),
			nvhost_syncpt_get_name(
				to_platform_device(g->dev->dev.parent),
				pbdma_syncpointb_syncpt_index_v(syncpointb)),
			pbdma_syncpointa_payload_v(syncpointa));

	gk20a_debug_output(o, "\n");
}

void gk20a_debug_show_dump(struct platform_device *pdev,
			   struct gk20a_debug_output *o)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a *g = platform->g;
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	int i;

	gk20a_busy(g->dev);
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

	for (chid = 0; chid < f->num_channels; chid++) {
		if (f->channel[chid].in_use) {
			struct channel_gk20a *gpu_ch = &f->channel[chid];
			gk20a_debug_show_channel(g, o, gpu_ch);
		}
	}
	gk20a_idle(g->dev);
}

void gk20a_debug_dump(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	if (platform->dump_platform_dependencies)
		platform->dump_platform_dependencies(pdev);

	gk20a_debug_show_dump(pdev, &o);
}

void gk20a_debug_dump_device(struct platform_device *pdev)
{
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	/* Dump the first device if no info is provided */
	if (!pdev && gk20a_device)
		pdev = gk20a_device;

	gk20a_debug_show_dump(pdev, &o);
}
EXPORT_SYMBOL(gk20a_debug_dump_device);

static int gk20a_debug_show(struct seq_file *s, void *unused)
{
	struct platform_device *pdev = s->private;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};
	gk20a_debug_show_dump(pdev, &o);
	return 0;
}

static int gk20a_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_debug_show, inode->i_private);
}

static const struct file_operations gk20a_debug_fops = {
	.open		= gk20a_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void gk20a_debug_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);

	/* Store the first device */
	if (!gk20a_device)
		gk20a_device = pdev;

	platform->debugfs = debugfs_create_dir(pdev->name, NULL);

	debugfs_create_file("status", S_IRUGO, platform->debugfs,
			pdev, &gk20a_debug_fops);
	debugfs_create_u32("trace_cmdbuf", S_IRUGO|S_IWUSR, platform->debugfs,
			&gk20a_debug_trace_cmdbuf);

#if defined(GK20A_DEBUG)
	debugfs_create_u32("dbg_mask", S_IRUGO|S_IWUSR, platform->debugfs,
			&gk20a_dbg_mask);
	debugfs_create_u32("dbg_ftrace", S_IRUGO|S_IWUSR, platform->debugfs,
			&gk20a_dbg_ftrace);
#endif
}
