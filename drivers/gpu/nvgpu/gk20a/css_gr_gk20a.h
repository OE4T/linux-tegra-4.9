/*
 * GK20A Cycle stats snapshots support (subsystem for gr_gk20a).
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CSS_GR_GK20A_H
#define CSS_GR_GK20A_H

/* the minimal size of HW buffer - should be enough to avoid HW overflows */
#define CSS_MIN_HW_SNAPSHOT_SIZE	(8 * 1024 * 1024)

/* cycle stats fifo header (must match NvSnapshotBufferFifo) */
struct gk20a_cs_snapshot_fifo {
	/* layout description of the buffer */
	u32	start;
	u32	end;

	/* snafu bits */
	u32	hw_overflow_events_occured;
	u32	sw_overflow_events_occured;

	/* the kernel copies new entries to put and
	 * increment the put++. if put == get then
	 * overflowEventsOccured++
	 */
	u32	put;
	u32	_reserved10;
	u32	_reserved11;
	u32	_reserved12;

	/* the driver/client reads from get until
	 * put==get, get++ */
	u32	get;
	u32	_reserved20;
	u32	_reserved21;
	u32	_reserved22;

	/* unused */
	u32	_reserved30;
	u32	_reserved31;
	u32	_reserved32;
	u32	_reserved33;
};

/* cycle stats fifo entry (must match NvSnapshotBufferFifoEntry) */
struct gk20a_cs_snapshot_fifo_entry {
	/* global 48 timestamp */
	u32	timestamp31_00:32;
	u32	timestamp39_32:8;

	/* id of perfmon, should correlate with CSS_MAX_PERFMON_IDS */
	u32	perfmon_id:8;

	/* typically samples_counter is wired to #pmtrigger count */
	u32	samples_counter:12;

	/* DS=Delay Sample, SZ=Size (0=32B, 1=16B) */
	u32	ds:1;
	u32	sz:1;
	u32	zero0:1;
	u32	zero1:1;

	/* counter results */
	u32	event_cnt:32;
	u32	trigger0_cnt:32;
	u32	trigger1_cnt:32;
	u32	sample_cnt:32;

	/* Local PmTrigger results for Maxwell+ or padding otherwise */
	u16	local_trigger_b_count:16;
	u16	book_mark_b:16;
	u16	local_trigger_a_count:16;
	u16	book_mark_a:16;
};

/* cycle stats snapshot client data (e.g. associated with channel) */
struct gk20a_cs_snapshot_client {
	struct list_head	list;
	u32			dmabuf_fd;
	struct dma_buf		*dma_handler;
	struct gk20a_cs_snapshot_fifo	*snapshot;
	u32			snapshot_size;
	u32			perfmon_start;
	u32			perfmon_count;
};

/* should correlate with size of gk20a_cs_snapshot_fifo_entry::perfmon_id */
#define CSS_MAX_PERFMON_IDS	256

/* local definitions to avoid hardcodes sizes and shifts */
#define PM_BITMAP_SIZE	DIV_ROUND_UP(CSS_MAX_PERFMON_IDS, BITS_PER_LONG)

/* cycle stats snapshot control structure for one HW entry and many clients */
struct gk20a_cs_snapshot {
	unsigned long perfmon_ids[PM_BITMAP_SIZE];
	struct list_head	clients;
	struct mem_desc		hw_memdesc;
	/* pointer to allocated cpu_va memory where GPU place data */
	struct gk20a_cs_snapshot_fifo_entry	*hw_snapshot;
	struct gk20a_cs_snapshot_fifo_entry	*hw_end;
	struct gk20a_cs_snapshot_fifo_entry	*hw_get;
};

void gk20a_init_css_ops(struct gpu_ops *gops);

#endif /* CSS_GR_GK20A_H */
