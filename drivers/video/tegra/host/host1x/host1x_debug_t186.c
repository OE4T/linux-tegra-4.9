/*
 * Tegra Graphics Host Hardware Debug Functions
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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
#include <linux/mm.h>
#include <linux/scatterlist.h>

#include <linux/io.h>

#include "dev.h"
#include "debug.h"
#include "nvhost_cdma.h"
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "chip_support.h"

#define NVHOST_DEBUG_MAX_PAGE_OFFSET 102400

enum {
	NVHOST_DBG_STATE_CMD = 0,
	NVHOST_DBG_STATE_DATA = 1,
	NVHOST_DBG_STATE_GATHER = 2
};

static void do_show_channel_gather(struct output *o,
		phys_addr_t phys_addr,
		u32 words, struct nvhost_cdma *cdma,
		phys_addr_t pin_addr, u32 *map_addr)
{
	/* Map dmaget cursor to corresponding nvmap_handle */
	u32 offset;
	int state, i;

	offset = phys_addr - pin_addr;
	/*
	 * Sometimes we're given different hardware address to the same
	 * page - in these cases the offset will get an invalid number and
	 * we just have to bail out.
	 */
	if (offset > NVHOST_DEBUG_MAX_PAGE_OFFSET) {
		nvhost_debug_output(o, "[address mismatch]\n");
	} else {
		/* GATHER buffer starts always with commands */
		state = NVHOST_DBG_STATE_CMD;
		for (i = 0; i < words; i++)
			nvhost_debug_output(o,
					"%08x ", *(map_addr + offset/4 + i));
		nvhost_debug_output(o, "\n");
	}
}

static void show_channel_gathers(struct output *o, struct nvhost_cdma *cdma)
{
	struct nvhost_job *job;
	int i;

	if (list_empty(&cdma->sync_queue)) {
		nvhost_debug_output(o, "The CDMA sync queue is empty.\n");
		return;
	}

	job = list_first_entry(&cdma->sync_queue, struct nvhost_job, list);

	nvhost_debug_output(o, "\n%p: JOB, syncpt_id=%d, syncpt_val=%d,"
			" first_get=%08x, timeout=%d, ctx=%p,"
			" num_slots=%d, num_handles=%d\n",
			job,
			job->sp ? job->sp->id : -1,
			job->sp ? job->sp->fence : -1,
			job->first_get,
			job->timeout,
			job->hwctx,
			job->num_slots,
			job->num_unpins);

	for (i = 0; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];
		u32 *mapped = dma_buf_vmap(g->buf);
		if (!mapped) {
			nvhost_debug_output(o, "[could not mmap]\n");
			continue;
		}

		nvhost_debug_output(o,
			"    GATHER at %08llx+%04x, %u words\n",
			(u64)g->mem_base, g->offset, g->words);

		do_show_channel_gather(o, g->mem_base + g->offset,
				g->words, cdma, g->mem_base, mapped);
		dma_buf_vunmap(g->buf, mapped);
	}
}

static void debug_show_channel_cdma(struct nvhost_master *m,
	struct nvhost_channel *ch, struct output *o, int chid)
{
	struct nvhost_cdma *cdma = &ch->cdma;

	show_channel_gathers(o, cdma);
	nvhost_debug_output(o, "\n");
}

static void debug_show_channel_fifo(struct nvhost_master *m,
	struct nvhost_channel *ch, struct output *o, int chid)
{
}

static void debug_show_mlocks(struct nvhost_master *m, struct output *o)
{
}

static const struct nvhost_debug_ops host1x_debug_ops = {
	.show_channel_cdma = debug_show_channel_cdma,
	.show_channel_fifo = debug_show_channel_fifo,
	.show_mlocks = debug_show_mlocks,
};
