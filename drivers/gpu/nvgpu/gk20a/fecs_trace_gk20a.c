/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <asm/barrier.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/bug.h>
#include <nvgpu/hashtable.h>
#include <nvgpu/circ_buf.h>
#include <nvgpu/thread.h>
#include <nvgpu/barrier.h>

#include "ctxsw_trace_gk20a.h"
#include "fecs_trace_gk20a.h"
#include "gk20a.h"
#include "gr_gk20a.h"
#include "common/linux/os_linux.h"

#include <nvgpu/log.h>

#include <nvgpu/hw/gk20a/hw_ctxsw_prog_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>

/*
 * If HW circular buffer is getting too many "buffer full" conditions,
 * increasing this constant should help (it drives Linux' internal buffer size).
 */
#define GK20A_FECS_TRACE_NUM_RECORDS		(1 << 6)
#define GK20A_FECS_TRACE_HASH_BITS		8 /* 2^8 */
#define GK20A_FECS_TRACE_FRAME_PERIOD_US	(1000000ULL/60ULL)
#define GK20A_FECS_TRACE_PTIMER_SHIFT		5

struct gk20a_fecs_trace_record {
	u32 magic_lo;
	u32 magic_hi;
	u32 context_id;
	u32 context_ptr;
	u32 new_context_id;
	u32 new_context_ptr;
	u64 ts[];
};

struct gk20a_fecs_trace_hash_ent {
	u32 context_ptr;
	pid_t pid;
	struct hlist_node node;
};

struct gk20a_fecs_trace {

	struct nvgpu_mem trace_buf;
	DECLARE_HASHTABLE(pid_hash_table, GK20A_FECS_TRACE_HASH_BITS);
	struct nvgpu_mutex hash_lock;
	struct nvgpu_mutex poll_lock;
	struct nvgpu_thread poll_task;
	bool init;
};

#ifdef CONFIG_GK20A_CTXSW_TRACE
static inline u32 gk20a_fecs_trace_record_ts_tag_v(u64 ts)
{
	return ctxsw_prog_record_timestamp_timestamp_hi_tag_v((u32) (ts >> 32));
}

static inline u64 gk20a_fecs_trace_record_ts_timestamp_v(u64 ts)
{
	return ts & ~(((u64)ctxsw_prog_record_timestamp_timestamp_hi_tag_m()) << 32);
}


static u32 gk20a_fecs_trace_fecs_context_ptr(struct gk20a *g, struct channel_gk20a *ch)
{
	return (u32) (gk20a_mm_inst_block_addr(g, &ch->inst_block) >> 12LL);
}

static inline int gk20a_fecs_trace_num_ts(void)
{
	return (ctxsw_prog_record_timestamp_record_size_in_bytes_v()
		- sizeof(struct gk20a_fecs_trace_record)) / sizeof(u64);
}

static struct gk20a_fecs_trace_record *gk20a_fecs_trace_get_record(
	struct gk20a_fecs_trace *trace, int idx)
{
	return (struct gk20a_fecs_trace_record *)
		((u8 *) trace->trace_buf.cpu_va
		+ (idx * ctxsw_prog_record_timestamp_record_size_in_bytes_v()));
}

static bool gk20a_fecs_trace_is_valid_record(struct gk20a_fecs_trace_record *r)
{
	/*
	 * testing magic_hi should suffice. magic_lo is sometimes used
	 * as a sequence number in experimental ucode.
	 */
	return (r->magic_hi
		== ctxsw_prog_record_timestamp_magic_value_hi_v_value_v());
}

static int gk20a_fecs_trace_get_read_index(struct gk20a *g)
{
	return gr_gk20a_elpg_protected_call(g,
			gk20a_readl(g, gr_fecs_mailbox1_r()));
}

static int gk20a_fecs_trace_get_write_index(struct gk20a *g)
{
	return gr_gk20a_elpg_protected_call(g,
			gk20a_readl(g, gr_fecs_mailbox0_r()));
}

static int gk20a_fecs_trace_set_read_index(struct gk20a *g, int index)
{
	gk20a_dbg(gpu_dbg_ctxsw, "set read=%d", index);
	return gr_gk20a_elpg_protected_call(g,
			(gk20a_writel(g, gr_fecs_mailbox1_r(), index), 0));
}

void gk20a_fecs_trace_hash_dump(struct gk20a *g)
{
	u32 bkt;
	struct gk20a_fecs_trace_hash_ent *ent;
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	gk20a_dbg(gpu_dbg_ctxsw, "dumping hash table");

	nvgpu_mutex_acquire(&trace->hash_lock);
	hash_for_each(trace->pid_hash_table, bkt, ent, node)
	{
		gk20a_dbg(gpu_dbg_ctxsw, " ent=%p bkt=%x context_ptr=%x pid=%d",
			ent, bkt, ent->context_ptr, ent->pid);

	}
	nvgpu_mutex_release(&trace->hash_lock);
}

static int gk20a_fecs_trace_hash_add(struct gk20a *g, u32 context_ptr, pid_t pid)
{
	struct gk20a_fecs_trace_hash_ent *he;
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_ctxsw,
		"adding hash entry context_ptr=%x -> pid=%d", context_ptr, pid);

	he = nvgpu_kzalloc(g, sizeof(*he));
	if (unlikely(!he)) {
		nvgpu_warn(g,
			"can't alloc new hash entry for context_ptr=%x pid=%d",
			context_ptr, pid);
		return -ENOMEM;
	}

	he->context_ptr = context_ptr;
	he->pid = pid;
	nvgpu_mutex_acquire(&trace->hash_lock);
	hash_add(trace->pid_hash_table, &he->node, context_ptr);
	nvgpu_mutex_release(&trace->hash_lock);
	return 0;
}

static void gk20a_fecs_trace_hash_del(struct gk20a *g, u32 context_ptr)
{
	struct hlist_node *tmp;
	struct gk20a_fecs_trace_hash_ent *ent;
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_ctxsw,
		"freeing hash entry context_ptr=%x", context_ptr);

	nvgpu_mutex_acquire(&trace->hash_lock);
	hash_for_each_possible_safe(trace->pid_hash_table, ent, tmp, node,
		context_ptr) {
		if (ent->context_ptr == context_ptr) {
			hash_del(&ent->node);
			gk20a_dbg(gpu_dbg_ctxsw,
				"freed hash entry=%p context_ptr=%x", ent,
				ent->context_ptr);
			nvgpu_kfree(g, ent);
			break;
		}
	}
	nvgpu_mutex_release(&trace->hash_lock);
}

static void gk20a_fecs_trace_free_hash_table(struct gk20a *g)
{
	u32 bkt;
	struct hlist_node *tmp;
	struct gk20a_fecs_trace_hash_ent *ent;
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_ctxsw, "trace=%p", trace);

	nvgpu_mutex_acquire(&trace->hash_lock);
	hash_for_each_safe(trace->pid_hash_table, bkt, tmp, ent, node) {
		hash_del(&ent->node);
		nvgpu_kfree(g, ent);
	}
	nvgpu_mutex_release(&trace->hash_lock);

}

static pid_t gk20a_fecs_trace_find_pid(struct gk20a *g, u32 context_ptr)
{
	struct gk20a_fecs_trace_hash_ent *ent;
	struct gk20a_fecs_trace *trace = g->fecs_trace;
	pid_t pid = 0;

	nvgpu_mutex_acquire(&trace->hash_lock);
	hash_for_each_possible(trace->pid_hash_table, ent, node, context_ptr) {
		if (ent->context_ptr == context_ptr) {
			gk20a_dbg(gpu_dbg_ctxsw,
				"found context_ptr=%x -> pid=%d",
				ent->context_ptr, ent->pid);
			pid = ent->pid;
			break;
		}
	}
	nvgpu_mutex_release(&trace->hash_lock);

	return pid;
}

/*
 * Converts HW entry format to userspace-facing format and pushes it to the
 * queue.
 */
static int gk20a_fecs_trace_ring_read(struct gk20a *g, int index)
{
	int i;
	struct nvgpu_ctxsw_trace_entry entry = { };
	struct gk20a_fecs_trace *trace = g->fecs_trace;
	pid_t cur_pid;
	pid_t new_pid;

	/* for now, only one VM */
	const int vmid = 0;

	struct gk20a_fecs_trace_record *r = gk20a_fecs_trace_get_record(
		trace, index);

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_ctxsw,
		"consuming record trace=%p read=%d record=%p", trace, index, r);

	if (unlikely(!gk20a_fecs_trace_is_valid_record(r))) {
		nvgpu_warn(g,
			"trace=%p read=%d record=%p magic_lo=%08x magic_hi=%08x (invalid)",
			trace, index, r, r->magic_lo, r->magic_hi);
		return -EINVAL;
	}

	/* Clear magic_hi to detect cases where CPU could read write index
	 * before FECS record is actually written to DRAM. This should not
	 * as we force FECS writes to SYSMEM by reading through PRAMIN.
	 */
	r->magic_hi = 0;

	cur_pid = gk20a_fecs_trace_find_pid(g, r->context_ptr);
	new_pid = gk20a_fecs_trace_find_pid(g, r->new_context_ptr);

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_ctxsw,
		"context_ptr=%x (pid=%d) new_context_ptr=%x (pid=%d)",
		r->context_ptr, cur_pid, r->new_context_ptr, new_pid);

	entry.context_id = r->context_id;
	entry.vmid = vmid;

	/* break out FECS record into trace events */
	for (i = 0; i < gk20a_fecs_trace_num_ts(); i++) {

		entry.tag = gk20a_fecs_trace_record_ts_tag_v(r->ts[i]);
		entry.timestamp = gk20a_fecs_trace_record_ts_timestamp_v(r->ts[i]);
		entry.timestamp <<= GK20A_FECS_TRACE_PTIMER_SHIFT;

		gk20a_dbg(gpu_dbg_ctxsw,
			"tag=%x timestamp=%llx context_id=%08x new_context_id=%08x",
			entry.tag, entry.timestamp, r->context_id,
			r->new_context_id);

		switch (entry.tag) {
		case NVGPU_CTXSW_TAG_RESTORE_START:
		case NVGPU_CTXSW_TAG_CONTEXT_START:
			entry.context_id = r->new_context_id;
			entry.pid = new_pid;
			break;

		case NVGPU_CTXSW_TAG_CTXSW_REQ_BY_HOST:
		case NVGPU_CTXSW_TAG_FE_ACK:
		case NVGPU_CTXSW_TAG_FE_ACK_WFI:
		case NVGPU_CTXSW_TAG_FE_ACK_GFXP:
		case NVGPU_CTXSW_TAG_FE_ACK_CTAP:
		case NVGPU_CTXSW_TAG_FE_ACK_CILP:
		case NVGPU_CTXSW_TAG_SAVE_END:
			entry.context_id = r->context_id;
			entry.pid = cur_pid;
			break;

		default:
			/* tags are not guaranteed to start at the beginning */
			WARN_ON(entry.tag && (entry.tag != NVGPU_CTXSW_TAG_INVALID_TIMESTAMP));
			continue;
		}

		gk20a_dbg(gpu_dbg_ctxsw, "tag=%x context_id=%x pid=%lld",
			entry.tag, entry.context_id, entry.pid);

		if (!entry.context_id)
			continue;

		gk20a_ctxsw_trace_write(g, &entry);
	}

	gk20a_ctxsw_trace_wake_up(g, vmid);
	return 0;
}

int gk20a_fecs_trace_poll(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	int read = 0;
	int write = 0;
	int cnt;
	int err;

	err = gk20a_busy(g);
	if (unlikely(err))
		return err;

	nvgpu_mutex_acquire(&trace->poll_lock);
	write = gk20a_fecs_trace_get_write_index(g);
	if (unlikely((write < 0) || (write >= GK20A_FECS_TRACE_NUM_RECORDS))) {
		nvgpu_err(g,
			"failed to acquire write index, write=%d", write);
		err = write;
		goto done;
	}

	read = gk20a_fecs_trace_get_read_index(g);

	cnt = CIRC_CNT(write, read, GK20A_FECS_TRACE_NUM_RECORDS);
	if (!cnt)
		goto done;

	gk20a_dbg(gpu_dbg_ctxsw,
		"circular buffer: read=%d (mailbox=%d) write=%d cnt=%d",
		read, gk20a_fecs_trace_get_read_index(g), write, cnt);

	/* Ensure all FECS writes have made it to SYSMEM */
	g->ops.mm.fb_flush(g);

	while (read != write) {
		/* Ignore error code, as we want to consume all records */
		(void)gk20a_fecs_trace_ring_read(g, read);

		/* Get to next record. */
		read = (read + 1) & (GK20A_FECS_TRACE_NUM_RECORDS - 1);
	}

	/* ensure FECS records has been updated before incrementing read index */
	nvgpu_smp_wmb();
	gk20a_fecs_trace_set_read_index(g, read);

done:
	nvgpu_mutex_release(&trace->poll_lock);
	gk20a_idle(g);
	return err;
}

static int gk20a_fecs_trace_periodic_polling(void *arg)
{
	struct gk20a *g = (struct gk20a *)arg;
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	pr_info("%s: running\n", __func__);

	while (!nvgpu_thread_should_stop(&trace->poll_task)) {

		nvgpu_usleep_range(GK20A_FECS_TRACE_FRAME_PERIOD_US,
				   GK20A_FECS_TRACE_FRAME_PERIOD_US * 2);

		gk20a_fecs_trace_poll(g);
	}

	return 0;
}

static int gk20a_fecs_trace_alloc_ring(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	return nvgpu_dma_alloc_sys(g, GK20A_FECS_TRACE_NUM_RECORDS
			* ctxsw_prog_record_timestamp_record_size_in_bytes_v(),
			&trace->trace_buf);
}

static void gk20a_fecs_trace_free_ring(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	nvgpu_dma_free(g, &trace->trace_buf);
}

#ifdef CONFIG_DEBUG_FS
/*
 * The sequence iterator functions.  We simply use the count of the
 * next line as our internal position.
 */
static void *gk20a_fecs_trace_debugfs_ring_seq_start(
		struct seq_file *s, loff_t *pos)
{
	if (*pos >= GK20A_FECS_TRACE_NUM_RECORDS)
		return NULL;

	return pos;
}

static void *gk20a_fecs_trace_debugfs_ring_seq_next(
		struct seq_file *s, void *v, loff_t *pos)
{
	++(*pos);
	if (*pos >= GK20A_FECS_TRACE_NUM_RECORDS)
		return NULL;
	return pos;
}

static void gk20a_fecs_trace_debugfs_ring_seq_stop(
		struct seq_file *s, void *v)
{
}

static int gk20a_fecs_trace_debugfs_ring_seq_show(
		struct seq_file *s, void *v)
{
	loff_t *pos = (loff_t *) v;
	struct gk20a *g = *(struct gk20a **)s->private;
	struct gk20a_fecs_trace *trace = g->fecs_trace;
	struct gk20a_fecs_trace_record *r = gk20a_fecs_trace_get_record(trace, *pos);
	int i;
	const u32 invalid_tag =
	    ctxsw_prog_record_timestamp_timestamp_hi_tag_invalid_timestamp_v();
	u32 tag;
	u64 timestamp;

	seq_printf(s, "record #%lld (%p)\n", *pos, r);
	seq_printf(s, "\tmagic_lo=%08x\n", r->magic_lo);
	seq_printf(s, "\tmagic_hi=%08x\n", r->magic_hi);
	if (gk20a_fecs_trace_is_valid_record(r)) {
		seq_printf(s, "\tcontext_ptr=%08x\n", r->context_ptr);
		seq_printf(s, "\tcontext_id=%08x\n", r->context_id);
		seq_printf(s, "\tnew_context_ptr=%08x\n", r->new_context_ptr);
		seq_printf(s, "\tnew_context_id=%08x\n", r->new_context_id);
		for (i = 0; i < gk20a_fecs_trace_num_ts(); i++) {
			tag = gk20a_fecs_trace_record_ts_tag_v(r->ts[i]);
			if (tag == invalid_tag)
				continue;
			timestamp = gk20a_fecs_trace_record_ts_timestamp_v(r->ts[i]);
			timestamp <<= GK20A_FECS_TRACE_PTIMER_SHIFT;
			seq_printf(s, "\ttag=%02x timestamp=%012llx\n", tag, timestamp);
		}
	}
	return 0;
}

/*
 * Tie them all together into a set of seq_operations.
 */
static const struct seq_operations gk20a_fecs_trace_debugfs_ring_seq_ops = {
	.start = gk20a_fecs_trace_debugfs_ring_seq_start,
	.next = gk20a_fecs_trace_debugfs_ring_seq_next,
	.stop = gk20a_fecs_trace_debugfs_ring_seq_stop,
	.show = gk20a_fecs_trace_debugfs_ring_seq_show
};

/*
 * Time to set up the file operations for our /proc file.  In this case,
 * all we need is an open function which sets up the sequence ops.
 */

static int gk20a_ctxsw_debugfs_ring_open(struct inode *inode,
	struct file *file)
{
	struct gk20a **p;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	p = __seq_open_private(file, &gk20a_fecs_trace_debugfs_ring_seq_ops,
		sizeof(struct gk20a *));
	if (!p)
		return -ENOMEM;

	*p = (struct gk20a *)inode->i_private;
	return 0;
};

/*
 * The file operations structure contains our open function along with
 * set of the canned seq_ ops.
 */
static const struct file_operations gk20a_fecs_trace_debugfs_ring_fops = {
	.owner = THIS_MODULE,
	.open = gk20a_ctxsw_debugfs_ring_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release_private
};

static int gk20a_fecs_trace_debugfs_read(void *arg, u64 *val)
{
	*val = gk20a_fecs_trace_get_read_index((struct gk20a *)arg);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gk20a_fecs_trace_debugfs_read_fops,
	gk20a_fecs_trace_debugfs_read, NULL, "%llu\n");

static int gk20a_fecs_trace_debugfs_write(void *arg, u64 *val)
{
	*val = gk20a_fecs_trace_get_write_index((struct gk20a *)arg);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gk20a_fecs_trace_debugfs_write_fops,
	gk20a_fecs_trace_debugfs_write, NULL, "%llu\n");

static void gk20a_fecs_trace_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	debugfs_create_file("ctxsw_trace_read", 0600, l->debugfs, g,
		&gk20a_fecs_trace_debugfs_read_fops);
	debugfs_create_file("ctxsw_trace_write", 0600, l->debugfs, g,
		&gk20a_fecs_trace_debugfs_write_fops);
	debugfs_create_file("ctxsw_trace_ring", 0600, l->debugfs, g,
		&gk20a_fecs_trace_debugfs_ring_fops);
}

#else

static void gk20a_fecs_trace_debugfs_init(struct gk20a *g)
{
}

#endif /* CONFIG_DEBUG_FS */

int gk20a_fecs_trace_init(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace;
	int err;

	trace = nvgpu_kzalloc(g, sizeof(struct gk20a_fecs_trace));
	if (!trace) {
		nvgpu_warn(g, "failed to allocate fecs_trace");
		return -ENOMEM;
	}
	g->fecs_trace = trace;

	err = nvgpu_mutex_init(&trace->poll_lock);
	if (err)
		goto clean;
	err = nvgpu_mutex_init(&trace->hash_lock);
	if (err)
		goto clean_poll_lock;

	BUG_ON(!is_power_of_2(GK20A_FECS_TRACE_NUM_RECORDS));
	err = gk20a_fecs_trace_alloc_ring(g);
	if (err) {
		nvgpu_warn(g, "failed to allocate FECS ring");
		goto clean_hash_lock;
	}

	hash_init(trace->pid_hash_table);

	g->gpu_characteristics.flags |=
		NVGPU_GPU_FLAGS_SUPPORT_FECS_CTXSW_TRACE;

	gk20a_fecs_trace_debugfs_init(g);

	trace->init = true;

	return 0;

clean_hash_lock:
	nvgpu_mutex_destroy(&trace->hash_lock);
clean_poll_lock:
	nvgpu_mutex_destroy(&trace->poll_lock);
clean:
	nvgpu_kfree(g, trace);
	g->fecs_trace = NULL;
	return err;
}

int gk20a_fecs_trace_bind_channel(struct gk20a *g,
		struct channel_gk20a *ch)
{
	/*
	 * map our circ_buf to the context space and store the GPU VA
	 * in the context header.
	 */

	u32 lo;
	u32 hi;
	phys_addr_t pa;
	struct channel_ctx_gk20a *ch_ctx = &ch->ch_ctx;
	struct gk20a_fecs_trace *trace = g->fecs_trace;
	struct nvgpu_mem *mem = &ch_ctx->gr_ctx->mem;
	u32 context_ptr = gk20a_fecs_trace_fecs_context_ptr(g, ch);
	pid_t pid;
	u32 aperture;

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_ctxsw,
			"chid=%d context_ptr=%x inst_block=%llx",
			ch->chid, context_ptr,
			gk20a_mm_inst_block_addr(g, &ch->inst_block));

	if (!trace)
		return -ENOMEM;

	pa = gk20a_mm_inst_block_addr(g, &trace->trace_buf);
	if (!pa)
		return -ENOMEM;
	aperture = nvgpu_aperture_mask(g, &trace->trace_buf,
			ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_target_sys_mem_noncoherent_f(),
			ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_target_vid_mem_f());

	if (nvgpu_mem_begin(g, mem))
		return -ENOMEM;

	lo = u64_lo32(pa);
	hi = u64_hi32(pa);

	gk20a_dbg(gpu_dbg_ctxsw, "addr_hi=%x addr_lo=%x count=%d", hi,
		lo, GK20A_FECS_TRACE_NUM_RECORDS);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_context_timestamp_buffer_ptr_o(),
		lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_context_timestamp_buffer_ptr_hi_o(),
		ctxsw_prog_main_image_context_timestamp_buffer_ptr_v_f(hi) |
		aperture);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_context_timestamp_buffer_control_o(),
		ctxsw_prog_main_image_context_timestamp_buffer_control_num_records_f(
			GK20A_FECS_TRACE_NUM_RECORDS));

	nvgpu_mem_end(g, mem);

	/* pid (process identifier) in user space, corresponds to tgid (thread
	 * group id) in kernel space.
	 */
	if (gk20a_is_channel_marked_as_tsg(ch))
		pid = tsg_gk20a_from_ch(ch)->tgid;
	else
		pid = ch->tgid;
	gk20a_fecs_trace_hash_add(g, context_ptr, pid);

	return 0;
}

int gk20a_fecs_trace_unbind_channel(struct gk20a *g, struct channel_gk20a *ch)
{
	u32 context_ptr = gk20a_fecs_trace_fecs_context_ptr(g, ch);

	if (g->fecs_trace) {
		gk20a_dbg(gpu_dbg_fn|gpu_dbg_ctxsw,
			"ch=%p context_ptr=%x", ch, context_ptr);

		if (g->ops.fecs_trace.is_enabled(g)) {
			if (g->ops.fecs_trace.flush)
				g->ops.fecs_trace.flush(g);
			gk20a_fecs_trace_poll(g);
		}
		gk20a_fecs_trace_hash_del(g, context_ptr);
	}
	return 0;
}

int gk20a_fecs_trace_reset(struct gk20a *g)
{
	gk20a_dbg(gpu_dbg_fn|gpu_dbg_ctxsw, "");

	if (!g->ops.fecs_trace.is_enabled(g))
		return 0;

	gk20a_fecs_trace_poll(g);
	return gk20a_fecs_trace_set_read_index(g, 0);
}

int gk20a_fecs_trace_deinit(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	if (!trace->init)
		return 0;

	nvgpu_thread_stop(&trace->poll_task);
	gk20a_fecs_trace_free_ring(g);
	gk20a_fecs_trace_free_hash_table(g);

	nvgpu_mutex_destroy(&g->fecs_trace->hash_lock);
	nvgpu_mutex_destroy(&g->fecs_trace->poll_lock);

	nvgpu_kfree(g, g->fecs_trace);
	g->fecs_trace = NULL;
	return 0;
}

int gk20a_gr_max_entries(struct gk20a *g,
		struct nvgpu_ctxsw_trace_filter *filter)
{
	int n;
	int tag;

	/* Compute number of entries per record, with given filter */
	for (n = 0, tag = 0; tag < gk20a_fecs_trace_num_ts(); tag++)
		n += (NVGPU_CTXSW_FILTER_ISSET(tag, filter) != 0);

	/* Return max number of entries generated for the whole ring */
	return n * GK20A_FECS_TRACE_NUM_RECORDS;
}

int gk20a_fecs_trace_enable(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;
	int write;
	int err = 0;

	if (!trace)
		return -EINVAL;

	if (nvgpu_thread_is_running(&trace->poll_task))
		return 0;

	/* drop data in hw buffer */
	if (g->ops.fecs_trace.flush)
		g->ops.fecs_trace.flush(g);
	write = gk20a_fecs_trace_get_write_index(g);
	gk20a_fecs_trace_set_read_index(g, write);

	err = nvgpu_thread_create(&trace->poll_task, g,
			gk20a_fecs_trace_periodic_polling, __func__);
	if (err) {
		nvgpu_warn(g,
				"failed to create FECS polling task");
		return err;
	}

	return 0;
}

int gk20a_fecs_trace_disable(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	if (nvgpu_thread_is_running(&trace->poll_task))
		nvgpu_thread_stop(&trace->poll_task);

	return -EPERM;
}

bool gk20a_fecs_trace_is_enabled(struct gk20a *g)
{
	struct gk20a_fecs_trace *trace = g->fecs_trace;

	return (trace && nvgpu_thread_is_running(&trace->poll_task));
}
#endif /* CONFIG_GK20A_CTXSW_TRACE */
