/*
 * GK20A Graphics channel
 *
 * Copyright (c) 2011-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/nvhost.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/highmem.h> /* need for nvmap.h*/
#include <trace/events/gk20a.h>
#include <linux/scatterlist.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/dma-buf.h>
#include <linux/vmalloc.h>

#include "debug_gk20a.h"

#include "gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "fence_gk20a.h"
#include "semaphore_gk20a.h"

#include "hw_ram_gk20a.h"
#include "hw_fifo_gk20a.h"
#include "hw_pbdma_gk20a.h"
#include "hw_ccsr_gk20a.h"
#include "hw_ltc_gk20a.h"

#define NVMAP_HANDLE_PARAM_SIZE 1

static struct channel_gk20a *acquire_unused_channel(struct fifo_gk20a *f);
static void release_used_channel(struct fifo_gk20a *f, struct channel_gk20a *c);

static void free_priv_cmdbuf(struct channel_gk20a *c,
			     struct priv_cmd_entry *e);
static void recycle_priv_cmdbuf(struct channel_gk20a *c);

static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c);
static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c);

static int channel_gk20a_commit_userd(struct channel_gk20a *c);
static int channel_gk20a_setup_userd(struct channel_gk20a *c);

static void channel_gk20a_bind(struct channel_gk20a *ch_gk20a);

static int channel_gk20a_update_runlist(struct channel_gk20a *c,
					bool add);
static void gk20a_free_error_notifiers(struct channel_gk20a *ch);

static struct channel_gk20a *acquire_unused_channel(struct fifo_gk20a *f)
{
	struct channel_gk20a *ch = NULL;
	int chid;

	mutex_lock(&f->ch_inuse_mutex);
	for (chid = 0; chid < f->num_channels; chid++) {
		if (!f->channel[chid].in_use) {
			f->channel[chid].in_use = true;
			ch = &f->channel[chid];
			break;
		}
	}
	mutex_unlock(&f->ch_inuse_mutex);

	return ch;
}

static void release_used_channel(struct fifo_gk20a *f, struct channel_gk20a *c)
{
	mutex_lock(&f->ch_inuse_mutex);
	f->channel[c->hw_chid].in_use = false;
	mutex_unlock(&f->ch_inuse_mutex);
}

int channel_gk20a_commit_va(struct channel_gk20a *c)
{
	gk20a_dbg_fn("");

	if (!c->inst_block.cpu_va)
		return -ENOMEM;

	gk20a_init_inst_block(&c->inst_block, c->vm,
			c->vm->gmmu_page_sizes[gmmu_page_size_big]);

	return 0;
}

static int channel_gk20a_commit_userd(struct channel_gk20a *c)
{
	u32 addr_lo;
	u32 addr_hi;
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpu_va;
	if (!inst_ptr)
		return -ENOMEM;

	addr_lo = u64_lo32(c->userd_iova >> ram_userd_base_shift_v());
	addr_hi = u64_hi32(c->userd_iova);

	gk20a_dbg_info("channel %d : set ramfc userd 0x%16llx",
		c->hw_chid, (u64)c->userd_iova);

	gk20a_mem_wr32(inst_ptr, ram_in_ramfc_w() + ram_fc_userd_w(),
		 pbdma_userd_target_vid_mem_f() |
		 pbdma_userd_addr_f(addr_lo));

	gk20a_mem_wr32(inst_ptr, ram_in_ramfc_w() + ram_fc_userd_hi_w(),
		 pbdma_userd_target_vid_mem_f() |
		 pbdma_userd_hi_addr_f(addr_hi));

	return 0;
}

static int channel_gk20a_set_schedule_params(struct channel_gk20a *c,
				u32 timeslice_timeout)
{
	void *inst_ptr;
	int shift = 3;
	int value = timeslice_timeout;

	inst_ptr = c->inst_block.cpu_va;
	if (!inst_ptr)
		return -ENOMEM;

	/* disable channel */
	c->g->ops.fifo.disable_channel(c);

	/* preempt the channel */
	WARN_ON(c->g->ops.fifo.preempt_channel(c->g, c->hw_chid));

	/* value field is 8 bits long */
	while (value >= 1 << 8) {
		value >>= 1;
		shift++;
	}

	/* time slice register is only 18bits long */
	if ((value << shift) >= 1<<19) {
		pr_err("Requested timeslice value is clamped to 18 bits\n");
		value = 255;
		shift = 10;
	}

	/* set new timeslice */
	gk20a_mem_wr32(inst_ptr, ram_fc_runlist_timeslice_w(),
		value | (shift << 12) |
		fifo_runlist_timeslice_enable_true_f());

	/* enable channel */
	gk20a_writel(c->g, ccsr_channel_r(c->hw_chid),
		gk20a_readl(c->g, ccsr_channel_r(c->hw_chid)) |
		ccsr_channel_enable_set_true_f());

	return 0;
}

int channel_gk20a_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries)
{
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpu_va;
	if (!inst_ptr)
		return -ENOMEM;

	memset(inst_ptr, 0, ram_fc_size_val_v());

	gk20a_mem_wr32(inst_ptr, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	gk20a_mem_wr32(inst_ptr, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	gk20a_mem_wr32(inst_ptr, ram_fc_signature_w(),
		 c->g->ops.fifo.get_pbdma_signature(c->g));

	gk20a_mem_wr32(inst_ptr, ram_fc_formats_w(),
		pbdma_formats_gp_fermi0_f() |
		pbdma_formats_pb_fermi1_f() |
		pbdma_formats_mp_fermi0_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(1) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_target_w(), pbdma_target_engine_sw_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_acquire_w(),
		pbdma_acquire_retry_man_2_f() |
		pbdma_acquire_retry_exp_2_f() |
		pbdma_acquire_timeout_exp_max_f() |
		pbdma_acquire_timeout_man_max_f() |
		pbdma_acquire_timeout_en_disable_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_runlist_timeslice_w(),
		fifo_runlist_timeslice_timeout_128_f() |
		fifo_runlist_timeslice_timescale_3_f() |
		fifo_runlist_timeslice_enable_true_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_pb_timeslice_w(),
		fifo_pb_timeslice_timeout_16_f() |
		fifo_pb_timeslice_timescale_0_f() |
		fifo_pb_timeslice_enable_true_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_chid_w(), ram_fc_chid_id_f(c->hw_chid));

	return channel_gk20a_commit_userd(c);
}

static int channel_gk20a_setup_userd(struct channel_gk20a *c)
{
	BUG_ON(!c->userd_cpu_va);

	gk20a_dbg_fn("");

	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_put_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_get_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_ref_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_put_hi_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_ref_threshold_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_top_level_get_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_top_level_get_hi_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_get_hi_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_get_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_put_w(), 0);

	return 0;
}

static void channel_gk20a_bind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = ch_gk20a->g;
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *engine_info =
		f->engine_info + ENGINE_GR_GK20A;

	u32 inst_ptr = gk20a_mem_phys(&ch_gk20a->inst_block)
		>> ram_in_base_shift_v();

	gk20a_dbg_info("bind channel %d inst ptr 0x%08x",
		ch_gk20a->hw_chid, inst_ptr);

	ch_gk20a->bound = true;

	gk20a_writel(g, ccsr_channel_r(ch_gk20a->hw_chid),
		(gk20a_readl(g, ccsr_channel_r(ch_gk20a->hw_chid)) &
		 ~ccsr_channel_runlist_f(~0)) |
		 ccsr_channel_runlist_f(engine_info->runlist_id));

	gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->hw_chid),
		ccsr_channel_inst_ptr_f(inst_ptr) |
		ccsr_channel_inst_target_vid_mem_f() |
		ccsr_channel_inst_bind_true_f());

	gk20a_writel(g, ccsr_channel_r(ch_gk20a->hw_chid),
		(gk20a_readl(g, ccsr_channel_r(ch_gk20a->hw_chid)) &
		 ~ccsr_channel_enable_set_f(~0)) |
		 ccsr_channel_enable_set_true_f());
}

void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = ch_gk20a->g;

	gk20a_dbg_fn("");

	if (ch_gk20a->bound)
		gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->hw_chid),
			ccsr_channel_inst_ptr_f(0) |
			ccsr_channel_inst_bind_false_f());

	ch_gk20a->bound = false;

	/*
	 * if we are agrressive then we can destroy the syncpt
	 * resource at this point
	 * if not, then it will be destroyed at channel_free()
	 */
	if (ch_gk20a->sync && ch_gk20a->sync->aggressive_destroy) {
		ch_gk20a->sync->destroy(ch_gk20a->sync);
		ch_gk20a->sync = NULL;
	}
}

int channel_gk20a_alloc_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	int err;

	gk20a_dbg_fn("");

	err = gk20a_alloc_inst_block(g, &ch->inst_block);
	if (err)
		return err;

	gk20a_dbg_info("channel %d inst block physical addr: 0x%16llx",
		ch->hw_chid, (u64)gk20a_mem_phys(&ch->inst_block));

	gk20a_dbg_fn("done");
	return 0;
}

void channel_gk20a_free_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	gk20a_free_inst_block(g, &ch->inst_block);
}

static int channel_gk20a_update_runlist(struct channel_gk20a *c, bool add)
{
	return c->g->ops.fifo.update_runlist(c->g, 0, c->hw_chid, add, true);
}

void channel_gk20a_enable(struct channel_gk20a *ch)
{
	/* enable channel */
	gk20a_writel(ch->g, ccsr_channel_r(ch->hw_chid),
		gk20a_readl(ch->g, ccsr_channel_r(ch->hw_chid)) |
		ccsr_channel_enable_set_true_f());
}

void channel_gk20a_disable(struct channel_gk20a *ch)
{
	/* disable channel */
	gk20a_writel(ch->g, ccsr_channel_r(ch->hw_chid),
		gk20a_readl(ch->g,
			ccsr_channel_r(ch->hw_chid)) |
			ccsr_channel_enable_clr_true_f());
}

void gk20a_channel_abort(struct channel_gk20a *ch)
{
	struct channel_gk20a_job *job, *n;
	bool released_job_semaphore = false;

	/* ensure no fences are pending */
	mutex_lock(&ch->submit_lock);
	if (ch->sync)
		ch->sync->set_min_eq_max(ch->sync);
	mutex_unlock(&ch->submit_lock);

	/* release all job semaphores (applies only to jobs that use
	   semaphore synchronization) */
	mutex_lock(&ch->jobs_lock);
	list_for_each_entry_safe(job, n, &ch->jobs, list) {
		if (job->post_fence->semaphore) {
			gk20a_semaphore_release(job->post_fence->semaphore);
			released_job_semaphore = true;
		}
	}
	mutex_unlock(&ch->jobs_lock);

	ch->g->ops.fifo.disable_channel(ch);

	if (released_job_semaphore) {
		wake_up_interruptible_all(&ch->semaphore_wq);
		gk20a_channel_update(ch, 0);
	}
}

int gk20a_wait_channel_idle(struct channel_gk20a *ch)
{
	bool channel_idle = false;
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(gk20a_get_gr_idle_timeout(ch->g));

	do {
		mutex_lock(&ch->jobs_lock);
		channel_idle = list_empty(&ch->jobs);
		mutex_unlock(&ch->jobs_lock);
		if (channel_idle)
			break;

		usleep_range(1000, 3000);
	} while (time_before(jiffies, end_jiffies)
			|| !tegra_platform_is_silicon());

	if (!channel_idle) {
		gk20a_err(dev_from_gk20a(ch->g), "jobs not freed for channel %d\n",
				ch->hw_chid);
		return -EBUSY;
	}

	return 0;
}

void gk20a_disable_channel(struct channel_gk20a *ch,
			   bool finish,
			   unsigned long finish_timeout)
{
	if (finish) {
		int err = gk20a_channel_finish(ch, finish_timeout);
		WARN_ON(err);
	}

	/* disable the channel from hw and increment syncpoints */
	gk20a_channel_abort(ch);

	gk20a_wait_channel_idle(ch);

	/* preempt the channel */
	ch->g->ops.fifo.preempt_channel(ch->g, ch->hw_chid);

	/* remove channel from runlist */
	channel_gk20a_update_runlist(ch, false);
}

#if defined(CONFIG_GK20A_CYCLE_STATS)

static void gk20a_free_cycle_stats_buffer(struct channel_gk20a *ch)
{
	/* disable existing cyclestats buffer */
	mutex_lock(&ch->cyclestate.cyclestate_buffer_mutex);
	if (ch->cyclestate.cyclestate_buffer_handler) {
		dma_buf_vunmap(ch->cyclestate.cyclestate_buffer_handler,
				ch->cyclestate.cyclestate_buffer);
		dma_buf_put(ch->cyclestate.cyclestate_buffer_handler);
		ch->cyclestate.cyclestate_buffer_handler = NULL;
		ch->cyclestate.cyclestate_buffer = NULL;
		ch->cyclestate.cyclestate_buffer_size = 0;
	}
	mutex_unlock(&ch->cyclestate.cyclestate_buffer_mutex);
}

static int gk20a_channel_cycle_stats(struct channel_gk20a *ch,
		       struct nvgpu_cycle_stats_args *args)
{
	struct dma_buf *dmabuf;
	void *virtual_address;

	if (args->dmabuf_fd && !ch->cyclestate.cyclestate_buffer_handler) {

		/* set up new cyclestats buffer */
		dmabuf = dma_buf_get(args->dmabuf_fd);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		virtual_address = dma_buf_vmap(dmabuf);
		if (!virtual_address)
			return -ENOMEM;

		ch->cyclestate.cyclestate_buffer_handler = dmabuf;
		ch->cyclestate.cyclestate_buffer = virtual_address;
		ch->cyclestate.cyclestate_buffer_size = dmabuf->size;
		return 0;

	} else if (!args->dmabuf_fd &&
			ch->cyclestate.cyclestate_buffer_handler) {
		gk20a_free_cycle_stats_buffer(ch);
		return 0;

	} else if (!args->dmabuf_fd &&
			!ch->cyclestate.cyclestate_buffer_handler) {
		/* no requst from GL */
		return 0;

	} else {
		pr_err("channel already has cyclestats buffer\n");
		return -EINVAL;
	}
}
#endif

static int gk20a_init_error_notifier(struct channel_gk20a *ch,
		struct nvgpu_set_error_notifier *args) {
	void *va;

	struct dma_buf *dmabuf;

	if (!args->mem) {
		pr_err("gk20a_init_error_notifier: invalid memory handle\n");
		return -EINVAL;
	}

	dmabuf = dma_buf_get(args->mem);

	if (ch->error_notifier_ref)
		gk20a_free_error_notifiers(ch);

	if (IS_ERR(dmabuf)) {
		pr_err("Invalid handle: %d\n", args->mem);
		return -EINVAL;
	}
	/* map handle */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		pr_err("Cannot map notifier handle\n");
		return -ENOMEM;
	}

	/* set channel notifiers pointer */
	ch->error_notifier_ref = dmabuf;
	ch->error_notifier = va + args->offset;
	ch->error_notifier_va = va;
	memset(ch->error_notifier, 0, sizeof(struct nvgpu_notification));
	return 0;
}

void gk20a_set_error_notifier(struct channel_gk20a *ch, __u32 error)
{
	if (ch->error_notifier_ref) {
		struct timespec time_data;
		u64 nsec;
		getnstimeofday(&time_data);
		nsec = ((u64)time_data.tv_sec) * 1000000000u +
				(u64)time_data.tv_nsec;
		ch->error_notifier->time_stamp.nanoseconds[0] =
				(u32)nsec;
		ch->error_notifier->time_stamp.nanoseconds[1] =
				(u32)(nsec >> 32);
		ch->error_notifier->info32 = error;
		ch->error_notifier->status = 0xffff;
		gk20a_err(dev_from_gk20a(ch->g),
		    "error notifier set to %d for ch %d\n", error, ch->hw_chid);
	}
}

static void gk20a_free_error_notifiers(struct channel_gk20a *ch)
{
	if (ch->error_notifier_ref) {
		dma_buf_vunmap(ch->error_notifier_ref, ch->error_notifier_va);
		dma_buf_put(ch->error_notifier_ref);
		ch->error_notifier_ref = NULL;
		ch->error_notifier = NULL;
		ch->error_notifier_va = NULL;
	}
}

void gk20a_free_channel(struct channel_gk20a *ch, bool finish)
{
	struct gk20a *g = ch->g;
	struct fifo_gk20a *f = &g->fifo;
	struct gr_gk20a *gr = &g->gr;
	struct vm_gk20a *ch_vm = ch->vm;
	unsigned long timeout = gk20a_get_gr_idle_timeout(g);
	struct dbg_session_gk20a *dbg_s;

	gk20a_dbg_fn("");

	/* if engine reset was deferred, perform it now */
	mutex_lock(&f->deferred_reset_mutex);
	if (g->fifo.deferred_reset_pending) {
		gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg, "engine reset was"
			   " deferred, running now");
		gk20a_fifo_reset_engine(g, g->fifo.mmu_fault_engines);
		g->fifo.mmu_fault_engines = 0;
		g->fifo.deferred_reset_pending = false;
	}
	mutex_unlock(&f->deferred_reset_mutex);

	if (!ch->bound)
		return;

	if (!gk20a_channel_as_bound(ch))
		goto unbind;

	gk20a_dbg_info("freeing bound channel context, timeout=%ld",
			timeout);

	gk20a_disable_channel(ch, finish && !ch->has_timedout, timeout);

	gk20a_free_error_notifiers(ch);

	/* release channel ctx */
	g->ops.gr.free_channel_ctx(ch);

	gk20a_gr_flush_channel_tlb(gr);

	memset(&ch->ramfc, 0, sizeof(struct mem_desc_sub));

	gk20a_gmmu_unmap_free(ch_vm, &ch->gpfifo.mem);

	memset(&ch->gpfifo, 0, sizeof(struct gpfifo_desc));

#if defined(CONFIG_GK20A_CYCLE_STATS)
	gk20a_free_cycle_stats_buffer(ch);
#endif

	channel_gk20a_free_priv_cmdbuf(ch);

	/* sync must be destroyed before releasing channel vm */
	if (ch->sync) {
		ch->sync->destroy(ch->sync);
		ch->sync = NULL;
	}

	/* release channel binding to the as_share */
	if (ch_vm->as_share)
		gk20a_as_release_share(ch_vm->as_share);
	else
		gk20a_vm_put(ch_vm);

	spin_lock(&ch->update_fn_lock);
	ch->update_fn = NULL;
	ch->update_fn_data = NULL;
	spin_unlock(&ch->update_fn_lock);
	cancel_work_sync(&ch->update_fn_work);

unbind:
	if (gk20a_is_channel_marked_as_tsg(ch))
		gk20a_tsg_unbind_channel(ch);

	g->ops.fifo.unbind_channel(ch);
	g->ops.fifo.free_inst(g, ch);

	ch->vpr = false;
	ch->vm = NULL;

	mutex_lock(&ch->submit_lock);
	gk20a_fence_put(ch->last_submit.pre_fence);
	gk20a_fence_put(ch->last_submit.post_fence);
	ch->last_submit.pre_fence = NULL;
	ch->last_submit.post_fence = NULL;
	mutex_unlock(&ch->submit_lock);
	WARN_ON(ch->sync);

	/* unlink all debug sessions */
	mutex_lock(&ch->dbg_s_lock);

	list_for_each_entry(dbg_s, &ch->dbg_s_list, dbg_s_list_node) {
		dbg_s->ch = NULL;
		list_del_init(&dbg_s->dbg_s_list_node);
	}

	mutex_unlock(&ch->dbg_s_lock);

	/* ALWAYS last */
	release_used_channel(f, ch);
}

int gk20a_channel_release(struct inode *inode, struct file *filp)
{
	struct channel_gk20a *ch = (struct channel_gk20a *)filp->private_data;
	struct gk20a *g = ch ? ch->g : NULL;
	int err;

	if (!ch)
		return 0;

	trace_gk20a_channel_release(dev_name(&g->dev->dev));

	err = gk20a_busy(ch->g->dev);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to release channel %d",
			ch->hw_chid);
		return err;
	}
	gk20a_free_channel(ch, true);
	gk20a_idle(ch->g->dev);

	filp->private_data = NULL;
	return 0;
}

static void gk20a_channel_update_runcb_fn(struct work_struct *work)
{
	struct channel_gk20a *ch =
		container_of(work, struct channel_gk20a, update_fn_work);
	void (*update_fn)(struct channel_gk20a *, void *);
	void *update_fn_data;

	spin_lock(&ch->update_fn_lock);
	update_fn = ch->update_fn;
	update_fn_data = ch->update_fn_data;
	spin_unlock(&ch->update_fn_lock);

	if (update_fn)
		update_fn(ch, update_fn_data);
}

struct channel_gk20a *gk20a_open_new_channel_with_cb(struct gk20a *g,
		void (*update_fn)(struct channel_gk20a *, void *),
		void *update_fn_data)
{
	struct channel_gk20a *ch = gk20a_open_new_channel(g);

	if (ch) {
		spin_lock(&ch->update_fn_lock);
		ch->update_fn = update_fn;
		ch->update_fn_data = update_fn_data;
		spin_unlock(&ch->update_fn_lock);
	}

	return ch;
}

struct channel_gk20a *gk20a_open_new_channel(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch;

	ch = acquire_unused_channel(f);
	if (ch == NULL) {
		/* TBD: we want to make this virtualizable */
		gk20a_err(dev_from_gk20a(g), "out of hw chids");
		return NULL;
	}

	ch->g = g;

	if (g->ops.fifo.alloc_inst(g, ch)) {
		ch->in_use = false;
		gk20a_err(dev_from_gk20a(g),
			   "failed to open gk20a channel, out of inst mem");

		return NULL;
	}
	ch->pid = current->pid;

	/* By default, channel is regular (non-TSG) channel */
	ch->tsgid = NVGPU_INVALID_TSG_ID;

	/* reset timeout counter and update timestamp */
	ch->timeout_accumulated_ms = 0;
	ch->timeout_gpfifo_get = 0;
	/* set gr host default timeout */
	ch->timeout_ms_max = gk20a_get_gr_idle_timeout(g);
	ch->timeout_debug_dump = true;
	ch->has_timedout = false;
	ch->obj_class = 0;

	/* The channel is *not* runnable at this point. It still needs to have
	 * an address space bound and allocate a gpfifo and grctx. */

	init_waitqueue_head(&ch->notifier_wq);
	init_waitqueue_head(&ch->semaphore_wq);
	init_waitqueue_head(&ch->submit_wq);

	mutex_init(&ch->poll_events.lock);
	ch->poll_events.events_enabled = false;
	ch->poll_events.num_pending_events = 0;

	ch->update_fn = NULL;
	ch->update_fn_data = NULL;
	spin_lock_init(&ch->update_fn_lock);
	INIT_WORK(&ch->update_fn_work, gk20a_channel_update_runcb_fn);

	return ch;
}

static int __gk20a_channel_open(struct gk20a *g, struct file *filp)
{
	int err;
	struct channel_gk20a *ch;

	trace_gk20a_channel_open(dev_name(&g->dev->dev));

	err = gk20a_busy(g->dev);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to power on, %d", err);
		return err;
	}
	ch = gk20a_open_new_channel(g);
	gk20a_idle(g->dev);
	if (!ch) {
		gk20a_err(dev_from_gk20a(g),
			"failed to get f");
		return -ENOMEM;
	}

	filp->private_data = ch;
	return 0;
}

int gk20a_channel_open(struct inode *inode, struct file *filp)
{
	struct gk20a *g = container_of(inode->i_cdev,
			struct gk20a, channel.cdev);
	int ret;

	gk20a_dbg_fn("start");
	ret = __gk20a_channel_open(g, filp);

	gk20a_dbg_fn("end");
	return ret;
}

int gk20a_channel_open_ioctl(struct gk20a *g,
		struct nvgpu_channel_open_args *args)
{
	int err;
	int fd;
	struct file *file;
	char *name;

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		return err;
	fd = err;

	name = kasprintf(GFP_KERNEL, "nvhost-%s-fd%d",
			dev_name(&g->dev->dev), fd);
	if (!name) {
		err = -ENOMEM;
		goto clean_up;
	}

	file = anon_inode_getfile(name, g->channel.cdev.ops, NULL, O_RDWR);
	kfree(name);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	err = __gk20a_channel_open(g, file);
	if (err)
		goto clean_up_file;

	fd_install(fd, file);
	args->channel_fd = fd;
	return 0;

clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(fd);
	return err;
}

/* allocate private cmd buffer.
   used for inserting commands before/after user submitted buffers. */
static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c)
{
	struct device *d = dev_from_gk20a(c->g);
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	u32 i = 0, size;
	int err = 0;

	/* Kernel can insert gpfifos before and after user gpfifos.
	   Before user gpfifos, kernel inserts fence_wait, which takes
	   syncpoint_a (2 dwords) + syncpoint_b (2 dwords) = 4 dwords.
	   After user gpfifos, kernel inserts fence_get, which takes
	   wfi (2 dwords) + syncpoint_a (2 dwords) + syncpoint_b (2 dwords)
	   = 6 dwords.
	   Worse case if kernel adds both of them for every user gpfifo,
	   max size of priv_cmdbuf is :
	   (gpfifo entry number * (2 / 3) * (4 + 6) * 4 bytes */
	size = roundup_pow_of_two(
		c->gpfifo.entry_num * 2 * 12 * sizeof(u32) / 3);

	err = gk20a_gmmu_alloc_map(ch_vm, size, &q->mem);
	if (err) {
		gk20a_err(d, "%s: memory allocation failed\n", __func__);
		goto clean_up;
	}

	q->size = q->mem.size / sizeof (u32);

	INIT_LIST_HEAD(&q->head);
	INIT_LIST_HEAD(&q->free);

	/* pre-alloc 25% of priv cmdbuf entries and put them on free list */
	for (i = 0; i < q->size / 4; i++) {
		e = kzalloc(sizeof(struct priv_cmd_entry), GFP_KERNEL);
		if (!e) {
			gk20a_err(d, "ch %d: fail to pre-alloc cmd entry",
				c->hw_chid);
			err = -ENOMEM;
			goto clean_up;
		}
		e->pre_alloc = true;
		list_add(&e->list, &q->free);
	}

	return 0;

clean_up:
	channel_gk20a_free_priv_cmdbuf(c);
	return err;
}

static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c)
{
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	struct list_head *pos, *tmp, *head;

	if (q->size == 0)
		return;

	gk20a_gmmu_unmap_free(ch_vm, &q->mem);

	/* free used list */
	head = &q->head;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		free_priv_cmdbuf(c, e);
	}

	/* free free list */
	head = &q->free;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		e->pre_alloc = false;
		free_priv_cmdbuf(c, e);
	}

	memset(q, 0, sizeof(struct priv_cmd_queue));
}

/* allocate a cmd buffer with given size. size is number of u32 entries */
int gk20a_channel_alloc_priv_cmdbuf(struct channel_gk20a *c, u32 orig_size,
			     struct priv_cmd_entry **entry)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	struct list_head *node;
	u32 free_count;
	u32 size = orig_size;
	bool no_retry = false;

	gk20a_dbg_fn("size %d", orig_size);

	*entry = NULL;

	/* if free space in the end is less than requested, increase the size
	 * to make the real allocated space start from beginning. */
	if (q->put + size > q->size)
		size = orig_size + (q->size - q->put);

	gk20a_dbg_info("ch %d: priv cmd queue get:put %d:%d",
			c->hw_chid, q->get, q->put);

TRY_AGAIN:
	free_count = (q->size - (q->put - q->get) - 1) % q->size;

	if (size > free_count) {
		if (!no_retry) {
			recycle_priv_cmdbuf(c);
			no_retry = true;
			goto TRY_AGAIN;
		} else
			return -EAGAIN;
	}

	if (unlikely(list_empty(&q->free))) {

		gk20a_dbg_info("ch %d: run out of pre-alloc entries",
			c->hw_chid);

		e = kzalloc(sizeof(struct priv_cmd_entry), GFP_KERNEL);
		if (!e) {
			gk20a_err(dev_from_gk20a(c->g),
				"ch %d: fail to allocate priv cmd entry",
				c->hw_chid);
			return -ENOMEM;
		}
	} else  {
		node = q->free.next;
		list_del(node);
		e = container_of(node, struct priv_cmd_entry, list);
	}

	e->size = orig_size;
	e->gp_get = c->gpfifo.get;
	e->gp_put = c->gpfifo.put;
	e->gp_wrap = c->gpfifo.wrap;

	/* if we have increased size to skip free space in the end, set put
	   to beginning of cmd buffer (0) + size */
	if (size != orig_size) {
		e->ptr = (u32 *)q->mem.cpu_va;
		e->gva = q->mem.gpu_va;
		q->put = orig_size;
	} else {
		e->ptr = (u32 *)q->mem.cpu_va + q->put;
		e->gva = q->mem.gpu_va + q->put * sizeof(u32);
		q->put = (q->put + orig_size) & (q->size - 1);
	}

	/* we already handled q->put + size > q->size so BUG_ON this */
	BUG_ON(q->put > q->size);

	/* add new entry to head since we free from head */
	list_add(&e->list, &q->head);

	*entry = e;

	gk20a_dbg_fn("done");

	return 0;
}

/* Don't call this to free an explict cmd entry.
 * It doesn't update priv_cmd_queue get/put */
static void free_priv_cmdbuf(struct channel_gk20a *c,
			     struct priv_cmd_entry *e)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;

	if (!e)
		return;

	list_del(&e->list);

	if (unlikely(!e->pre_alloc))
		kfree(e);
	else {
		memset(e, 0, sizeof(struct priv_cmd_entry));
		e->pre_alloc = true;
		list_add(&e->list, &q->free);
	}
}

/* free entries if they're no longer being used */
static void recycle_priv_cmdbuf(struct channel_gk20a *c)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e, *tmp;
	struct list_head *head = &q->head;
	bool wrap_around, found = false;

	gk20a_dbg_fn("");

	/* Find the most recent free entry. Free it and everything before it */
	list_for_each_entry(e, head, list) {

		gk20a_dbg_info("ch %d: cmd entry get:put:wrap %d:%d:%d "
			"curr get:put:wrap %d:%d:%d",
			c->hw_chid, e->gp_get, e->gp_put, e->gp_wrap,
			c->gpfifo.get, c->gpfifo.put, c->gpfifo.wrap);

		wrap_around = (c->gpfifo.wrap != e->gp_wrap);
		if (e->gp_get < e->gp_put) {
			if (c->gpfifo.get >= e->gp_put ||
			    wrap_around) {
				found = true;
				break;
			} else
				e->gp_get = c->gpfifo.get;
		} else if (e->gp_get > e->gp_put) {
			if (wrap_around &&
			    c->gpfifo.get >= e->gp_put) {
				found = true;
				break;
			} else
				e->gp_get = c->gpfifo.get;
		}
	}

	if (found)
		q->get = (e->ptr - (u32 *)q->mem.cpu_va) + e->size;
	else {
		gk20a_dbg_info("no free entry recycled");
		return;
	}

	list_for_each_entry_safe_continue(e, tmp, head, list) {
		free_priv_cmdbuf(c, e);
	}

	gk20a_dbg_fn("done");
}


int gk20a_alloc_channel_gpfifo(struct channel_gk20a *c,
		struct nvgpu_alloc_gpfifo_args *args)
{
	struct gk20a *g = c->g;
	struct device *d = dev_from_gk20a(g);
	struct vm_gk20a *ch_vm;
	u32 gpfifo_size;
	int err = 0;

	/* Kernel can insert one extra gpfifo entry before user submitted gpfifos
	   and another one after, for internal usage. Triple the requested size. */
	gpfifo_size = roundup_pow_of_two(args->num_entries * 3);

	if (args->flags & NVGPU_ALLOC_GPFIFO_FLAGS_VPR_ENABLED)
		c->vpr = true;

	/* an address space needs to have been bound at this point. */
	if (!gk20a_channel_as_bound(c)) {
		gk20a_err(d,
			    "not bound to an address space at time of gpfifo"
			    " allocation.");
		return -EINVAL;
	}
	ch_vm = c->vm;

	c->cmds_pending = false;
	mutex_lock(&c->submit_lock);
	gk20a_fence_put(c->last_submit.pre_fence);
	gk20a_fence_put(c->last_submit.post_fence);
	c->last_submit.pre_fence = NULL;
	c->last_submit.post_fence = NULL;
	mutex_unlock(&c->submit_lock);

	c->ramfc.offset = 0;
	c->ramfc.size = ram_in_ramfc_s() / 8;

	if (c->gpfifo.mem.cpu_va) {
		gk20a_err(d, "channel %d :"
			   "gpfifo already allocated", c->hw_chid);
		return -EEXIST;
	}

	err = gk20a_gmmu_alloc_map(ch_vm, gpfifo_size * sizeof(struct gpfifo),
			&c->gpfifo.mem);
	if (err) {
		gk20a_err(d, "%s: memory allocation failed\n", __func__);
		goto clean_up;
	}

	c->gpfifo.entry_num = gpfifo_size;
	c->gpfifo.get = c->gpfifo.put = 0;

	gk20a_dbg_info("channel %d : gpfifo_base 0x%016llx, size %d",
		c->hw_chid, c->gpfifo.mem.gpu_va, c->gpfifo.entry_num);

	channel_gk20a_setup_userd(c);

	err = g->ops.fifo.setup_ramfc(c, c->gpfifo.mem.gpu_va, c->gpfifo.entry_num);
	if (err)
		goto clean_up_unmap;

	/* TBD: setup engine contexts */

	err = channel_gk20a_alloc_priv_cmdbuf(c);
	if (err)
		goto clean_up_unmap;

	err = channel_gk20a_update_runlist(c, true);
	if (err)
		goto clean_up_unmap;

	g->ops.fifo.bind_channel(c);

	gk20a_dbg_fn("done");
	return 0;

clean_up_unmap:
	gk20a_gmmu_unmap_free(ch_vm, &c->gpfifo.mem);
clean_up:
	memset(&c->gpfifo, 0, sizeof(struct gpfifo_desc));
	gk20a_err(d, "fail");
	return err;
}

static inline bool check_gp_put(struct gk20a *g,
				struct channel_gk20a *c)
{
	u32 put;
	/* gp_put changed unexpectedly since last update? */
	put = gk20a_bar1_readl(g,
	       c->userd_gpu_va + 4 * ram_userd_gp_put_w());
	if (c->gpfifo.put != put) {
		/*TBD: BUG_ON/teardown on this*/
		gk20a_err(dev_from_gk20a(g), "gp_put changed unexpectedly "
			  "since last update, channel put = %u, ram put = %u\n",
			  c->gpfifo.put, put);
		c->gpfifo.put = put;
		return false; /* surprise! */
	}
	return true; /* checked out ok */
}

/* Update with this periodically to determine how the gpfifo is draining. */
static inline u32 update_gp_get(struct gk20a *g,
				struct channel_gk20a *c)
{
	u32 new_get = gk20a_bar1_readl(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_gp_get_w());
	if (new_get < c->gpfifo.get)
		c->gpfifo.wrap = !c->gpfifo.wrap;
	c->gpfifo.get = new_get;
	return new_get;
}

static inline u32 gp_free_count(struct channel_gk20a *c)
{
	return (c->gpfifo.entry_num - (c->gpfifo.put - c->gpfifo.get) - 1) %
		c->gpfifo.entry_num;
}

bool gk20a_channel_update_and_check_timeout(struct channel_gk20a *ch,
		u32 timeout_delta_ms)
{
	u32 gpfifo_get = update_gp_get(ch->g, ch);
	/* Count consequent timeout isr */
	if (gpfifo_get == ch->timeout_gpfifo_get) {
		/* we didn't advance since previous channel timeout check */
		ch->timeout_accumulated_ms += timeout_delta_ms;
	} else {
		/* first timeout isr encountered */
		ch->timeout_accumulated_ms = timeout_delta_ms;
	}

	ch->timeout_gpfifo_get = gpfifo_get;

	return ch->g->timeouts_enabled &&
		ch->timeout_accumulated_ms > ch->timeout_ms_max;
}


/* Issue a syncpoint increment *preceded* by a wait-for-idle
 * command.  All commands on the channel will have been
 * consumed at the time the fence syncpoint increment occurs.
 */
static int gk20a_channel_submit_wfi(struct channel_gk20a *c)
{
	struct priv_cmd_entry *cmd = NULL;
	struct gk20a *g = c->g;
	u32 free_count;
	int err = 0;

	if (c->has_timedout)
		return -ETIMEDOUT;

	update_gp_get(g, c);
	free_count = gp_free_count(c);
	if (unlikely(!free_count)) {
		gk20a_err(dev_from_gk20a(g),
			   "not enough gpfifo space");
		return -EAGAIN;
	}

	mutex_lock(&c->submit_lock);

	if (!c->sync) {
		c->sync = gk20a_channel_sync_create(c);
		if (!c->sync) {
			mutex_unlock(&c->submit_lock);
			return -ENOMEM;
		}
		if (g->ops.fifo.resetup_ramfc)
			err = g->ops.fifo.resetup_ramfc(c);
		if (err)
			return err;
	}

	gk20a_fence_put(c->last_submit.pre_fence);
	gk20a_fence_put(c->last_submit.post_fence);
	c->last_submit.pre_fence = NULL;
	c->last_submit.post_fence = NULL;

	err = c->sync->incr_wfi(c->sync, &cmd, &c->last_submit.post_fence);
	if (unlikely(err)) {
		mutex_unlock(&c->submit_lock);
		return err;
	}

	WARN_ON(!c->last_submit.post_fence->wfi);

	((struct gpfifo *)(c->gpfifo.mem.cpu_va))[c->gpfifo.put].entry0 = u64_lo32(cmd->gva);
	((struct gpfifo *)(c->gpfifo.mem.cpu_va))[c->gpfifo.put].entry1 = u64_hi32(cmd->gva) |
		pbdma_gp_entry1_length_f(cmd->size);

	c->gpfifo.put = (c->gpfifo.put + 1) & (c->gpfifo.entry_num - 1);

	/* save gp_put */
	cmd->gp_put = c->gpfifo.put;

	gk20a_bar1_writel(g,
		c->userd_gpu_va + 4 * ram_userd_gp_put_w(),
		c->gpfifo.put);

	mutex_unlock(&c->submit_lock);

	gk20a_dbg_info("post-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	return 0;
}

static u32 get_gp_free_count(struct channel_gk20a *c)
{
	update_gp_get(c->g, c);
	return gp_free_count(c);
}

static void trace_write_pushbuffer(struct channel_gk20a *c,
				   struct nvgpu_gpfifo *g)
{
	void *mem = NULL;
	unsigned int words;
	u64 offset;
	struct dma_buf *dmabuf = NULL;

	if (gk20a_debug_trace_cmdbuf) {
		u64 gpu_va = (u64)g->entry0 |
			(u64)((u64)pbdma_gp_entry1_get_hi_v(g->entry1) << 32);
		int err;

		words = pbdma_gp_entry1_length_v(g->entry1);
		err = gk20a_vm_find_buffer(c->vm, gpu_va, &dmabuf, &offset);
		if (!err)
			mem = dma_buf_vmap(dmabuf);
	}

	if (mem) {
		u32 i;
		/*
		 * Write in batches of 128 as there seems to be a limit
		 * of how much you can output to ftrace at once.
		 */
		for (i = 0; i < words; i += 128U) {
			trace_gk20a_push_cmdbuf(
				c->g->dev->name,
				0,
				min(words - i, 128U),
				offset + i * sizeof(u32),
				mem);
		}
		dma_buf_vunmap(dmabuf, mem);
	}
}

static void trace_write_pushbuffer_range(struct channel_gk20a *c,
					 struct nvgpu_gpfifo *g,
					 int count)
{
	if (gk20a_debug_trace_cmdbuf) {
		int i;
		struct nvgpu_gpfifo *gp = g;
		for (i = 0; i < count; i++, gp++)
			trace_write_pushbuffer(c, gp);
	}
}

static int gk20a_channel_add_job(struct channel_gk20a *c,
				 struct gk20a_fence *pre_fence,
				 struct gk20a_fence *post_fence)
{
	struct vm_gk20a *vm = c->vm;
	struct channel_gk20a_job *job = NULL;
	struct mapped_buffer_node **mapped_buffers = NULL;
	int err = 0, num_mapped_buffers;

	/* job needs reference to this vm */
	gk20a_vm_get(vm);

	err = gk20a_vm_get_buffers(vm, &mapped_buffers, &num_mapped_buffers);
	if (err) {
		gk20a_vm_put(vm);
		return err;
	}

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job) {
		gk20a_vm_put_buffers(vm, mapped_buffers, num_mapped_buffers);
		gk20a_vm_put(vm);
		return -ENOMEM;
	}

	job->num_mapped_buffers = num_mapped_buffers;
	job->mapped_buffers = mapped_buffers;
	job->pre_fence = gk20a_fence_get(pre_fence);
	job->post_fence = gk20a_fence_get(post_fence);

	mutex_lock(&c->jobs_lock);
	list_add_tail(&job->list, &c->jobs);
	mutex_unlock(&c->jobs_lock);

	return 0;
}

void gk20a_channel_update(struct channel_gk20a *c, int nr_completed)
{
	struct vm_gk20a *vm = c->vm;
	struct channel_gk20a_job *job, *n;

	trace_gk20a_channel_update(c);

	wake_up(&c->submit_wq);

	mutex_lock(&c->submit_lock);
	mutex_lock(&c->jobs_lock);
	list_for_each_entry_safe(job, n, &c->jobs, list) {
		bool completed = gk20a_fence_is_expired(job->post_fence);
		if (!completed)
			break;

		c->sync->signal_timeline(c->sync);

		gk20a_vm_put_buffers(vm, job->mapped_buffers,
				job->num_mapped_buffers);

		/* Close the fences (this will unref the semaphores and release
		 * them to the pool). */
		gk20a_fence_put(job->pre_fence);
		gk20a_fence_put(job->post_fence);

		/* job is done. release its reference to vm */
		gk20a_vm_put(vm);

		list_del_init(&job->list);
		kfree(job);
		gk20a_idle(c->g->dev);
	}

	/*
	 * If job list is empty then channel is idle and we can free
	 * the syncpt here (given aggressive_destroy flag is set)
	 * Note: check if last submit is complete before destroying
	 * the sync resource
	 */
	if (list_empty(&c->jobs)) {
		if (c->sync && c->sync->aggressive_destroy &&
			  gk20a_fence_is_expired(c->last_submit.post_fence)) {
			c->sync->destroy(c->sync);
			c->sync = NULL;
		}
	}
	mutex_unlock(&c->jobs_lock);
	mutex_unlock(&c->submit_lock);

	if (c->update_fn)
		schedule_work(&c->update_fn_work);
}

int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvgpu_gpfifo *gpfifo,
				u32 num_entries,
				u32 flags,
				struct nvgpu_fence *fence,
				struct gk20a_fence **fence_out)
{
	struct gk20a *g = c->g;
	struct device *d = dev_from_gk20a(g);
	int err = 0;
	int start, end;
	int wait_fence_fd = -1;
	struct priv_cmd_entry *wait_cmd = NULL;
	struct priv_cmd_entry *incr_cmd = NULL;
	struct gk20a_fence *pre_fence = NULL;
	struct gk20a_fence *post_fence = NULL;
	/* we might need two extra gpfifo entries - one for pre fence
	 * and one for post fence. */
	const int extra_entries = 2;
	bool need_wfi = !(flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SUPPRESS_WFI);

	if (c->has_timedout)
		return -ETIMEDOUT;

	/* fifo not large enough for request. Return error immediately.
	 * Kernel can insert gpfifo entries before and after user gpfifos.
	 * So, add extra_entries in user request. Also, HW with fifo size N
	 * can accept only N-1 entreis and so the below condition */
	if (c->gpfifo.entry_num - 1 < num_entries + extra_entries) {
		gk20a_err(d, "not enough gpfifo space allocated");
		return -ENOMEM;
	}

	if ((flags & (NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT |
		      NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET)) &&
	    !fence)
		return -EINVAL;

	/* an address space needs to have been bound at this point. */
	if (!gk20a_channel_as_bound(c)) {
		gk20a_err(d,
			    "not bound to an address space at time of gpfifo"
			    " submission.");
		return -EINVAL;
	}

#ifdef CONFIG_DEBUG_FS
	/* update debug settings */
	if (g->ops.ltc.sync_debugfs)
		g->ops.ltc.sync_debugfs(g);
#endif

	gk20a_dbg_info("channel %d", c->hw_chid);

	/* gk20a_channel_update releases this ref. */
	err = gk20a_busy(g->dev);
	if (err) {
		gk20a_err(d, "failed to host gk20a to submit gpfifo");
		return err;
	}

	trace_gk20a_channel_submit_gpfifo(c->g->dev->name,
					  c->hw_chid,
					  num_entries,
					  flags,
					  fence ? fence->id : 0,
					  fence ? fence->value : 0);
	check_gp_put(g, c);
	update_gp_get(g, c);

	gk20a_dbg_info("pre-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	/* Make sure we have enough space for gpfifo entries. If not,
	 * wait for signals from completed submits */
	if (gp_free_count(c) < num_entries + extra_entries) {
		/* we can get here via locked ioctl and other paths too */
		int locked_path = mutex_is_locked(&c->ioctl_lock);
		if (locked_path)
			mutex_unlock(&c->ioctl_lock);

		trace_gk20a_gpfifo_submit_wait_for_space(c->g->dev->name);
		err = wait_event_interruptible(c->submit_wq,
			get_gp_free_count(c) >= num_entries + extra_entries ||
			c->has_timedout);
		trace_gk20a_gpfifo_submit_wait_for_space_done(c->g->dev->name);

		if (locked_path)
			mutex_lock(&c->ioctl_lock);
	}

	if (c->has_timedout) {
		err = -ETIMEDOUT;
		goto clean_up;
	}

	if (err) {
		gk20a_err(d, "timeout waiting for gpfifo space");
		err = -EAGAIN;
		goto clean_up;
	}

	mutex_lock(&c->submit_lock);

	if (!c->sync) {
		c->sync = gk20a_channel_sync_create(c);
		if (!c->sync) {
			err = -ENOMEM;
			mutex_unlock(&c->submit_lock);
			goto clean_up;
		}
		if (g->ops.fifo.resetup_ramfc)
			err = g->ops.fifo.resetup_ramfc(c);
		if (err)
			return err;
	}

	/*
	 * optionally insert syncpt wait in the beginning of gpfifo submission
	 * when user requested and the wait hasn't expired.
	 * validate that the id makes sense, elide if not
	 * the only reason this isn't being unceremoniously killed is to
	 * keep running some tests which trigger this condition
	 */
	if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT) {
		if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE) {
			wait_fence_fd = fence->id;
			err = c->sync->wait_fd(c->sync, wait_fence_fd,
					&wait_cmd, &pre_fence);
		} else {
			err = c->sync->wait_syncpt(c->sync, fence->id,
					fence->value, &wait_cmd, &pre_fence);
		}
	}
	if (err) {
		mutex_unlock(&c->submit_lock);
		goto clean_up;
	}


	/* always insert syncpt increment at end of gpfifo submission
	   to keep track of method completion for idle railgating */
	if (flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET)
		err = c->sync->incr_user(c->sync, wait_fence_fd, &incr_cmd,
					 &post_fence, need_wfi);
	else
		err = c->sync->incr(c->sync, &incr_cmd,
				    &post_fence);
	if (err) {
		mutex_unlock(&c->submit_lock);
		goto clean_up;
	}

	if (wait_cmd) {
		((struct gpfifo *)(c->gpfifo.mem.cpu_va))[c->gpfifo.put].entry0 =
			u64_lo32(wait_cmd->gva);
		((struct gpfifo *)(c->gpfifo.mem.cpu_va))[c->gpfifo.put].entry1 =
			u64_hi32(wait_cmd->gva) |
			pbdma_gp_entry1_length_f(wait_cmd->size);
		trace_gk20a_push_cmdbuf(c->g->dev->name,
			0, wait_cmd->size, 0, wait_cmd->ptr);

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);

		/* save gp_put */
		wait_cmd->gp_put = c->gpfifo.put;
	}

	/*
	 * Copy source gpfifo entries into the gpfifo ring buffer,
	 * potentially splitting into two memcpies to handle the
	 * ring buffer wrap-around case.
	 */
	start = c->gpfifo.put;
	end = start + num_entries;

	if (end > c->gpfifo.entry_num) {
		int length0 = c->gpfifo.entry_num - start;
		int length1 = num_entries - length0;

		memcpy((struct gpfifo *)c->gpfifo.mem.cpu_va + start, gpfifo,
		       length0 * sizeof(*gpfifo));

		memcpy((struct gpfifo *)c->gpfifo.mem.cpu_va, gpfifo + length0,
		       length1 * sizeof(*gpfifo));

		trace_write_pushbuffer_range(c, gpfifo, length0);
		trace_write_pushbuffer_range(c, gpfifo + length0, length1);
	} else {
		memcpy((struct gpfifo *)c->gpfifo.mem.cpu_va + start, gpfifo,
		       num_entries * sizeof(*gpfifo));

		trace_write_pushbuffer_range(c, gpfifo, num_entries);
	}
	c->gpfifo.put = (c->gpfifo.put + num_entries) &
		(c->gpfifo.entry_num - 1);

	if (incr_cmd) {
		((struct gpfifo *)(c->gpfifo.mem.cpu_va))[c->gpfifo.put].entry0 =
			u64_lo32(incr_cmd->gva);
		((struct gpfifo *)(c->gpfifo.mem.cpu_va))[c->gpfifo.put].entry1 =
			u64_hi32(incr_cmd->gva) |
			pbdma_gp_entry1_length_f(incr_cmd->size);
		trace_gk20a_push_cmdbuf(c->g->dev->name,
			0, incr_cmd->size, 0, incr_cmd->ptr);

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);

		/* save gp_put */
		incr_cmd->gp_put = c->gpfifo.put;
	}

	gk20a_fence_put(c->last_submit.pre_fence);
	gk20a_fence_put(c->last_submit.post_fence);
	c->last_submit.pre_fence = pre_fence;
	c->last_submit.post_fence = post_fence;
	if (fence_out)
		*fence_out = gk20a_fence_get(post_fence);

	/* TODO! Check for errors... */
	gk20a_channel_add_job(c, pre_fence, post_fence);

	c->cmds_pending = true;
	gk20a_bar1_writel(g,
		c->userd_gpu_va + 4 * ram_userd_gp_put_w(),
		c->gpfifo.put);

	mutex_unlock(&c->submit_lock);

	trace_gk20a_channel_submitted_gpfifo(c->g->dev->name,
					     c->hw_chid,
					     num_entries,
					     flags,
					     post_fence->syncpt_id,
					     post_fence->syncpt_value);

	gk20a_dbg_info("post-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	gk20a_dbg_fn("done");
	return err;

clean_up:
	gk20a_err(d, "fail");
	free_priv_cmdbuf(c, wait_cmd);
	free_priv_cmdbuf(c, incr_cmd);
	gk20a_fence_put(pre_fence);
	gk20a_fence_put(post_fence);
	gk20a_idle(g->dev);
	return err;
}

int gk20a_init_channel_support(struct gk20a *g, u32 chid)
{
	struct channel_gk20a *c = g->fifo.channel+chid;
	c->g = g;
	c->in_use = false;
	c->hw_chid = chid;
	c->bound = false;
	mutex_init(&c->ioctl_lock);
	mutex_init(&c->jobs_lock);
	mutex_init(&c->submit_lock);
	INIT_LIST_HEAD(&c->jobs);
#if defined(CONFIG_GK20A_CYCLE_STATS)
	mutex_init(&c->cyclestate.cyclestate_buffer_mutex);
#endif
	INIT_LIST_HEAD(&c->dbg_s_list);
	mutex_init(&c->dbg_s_lock);

	return 0;
}

int gk20a_channel_finish(struct channel_gk20a *ch, unsigned long timeout)
{
	int err = 0;
	struct gk20a_fence *fence = ch->last_submit.post_fence;

	if (!ch->cmds_pending)
		return 0;

	/* Do not wait for a timedout channel */
	if (ch->has_timedout)
		return -ETIMEDOUT;

	if (!(fence && fence->wfi) && ch->obj_class != KEPLER_C) {
		gk20a_dbg_fn("issuing wfi, incr to finish the channel");
		err = gk20a_channel_submit_wfi(ch);
		fence = ch->last_submit.post_fence;
	}
	if (err)
		return err;

	BUG_ON(!(fence && fence->wfi) && ch->obj_class != KEPLER_C);

	gk20a_dbg_fn("waiting for channel to finish thresh:%d sema:%p",
		     fence->syncpt_value, fence->semaphore);

	err = gk20a_fence_wait(fence, timeout);
	if (WARN_ON(err))
		dev_warn(dev_from_gk20a(ch->g),
		       "timed out waiting for gk20a channel to finish");
	else
		ch->cmds_pending = false;

	return err;
}

static int gk20a_channel_wait_semaphore(struct channel_gk20a *ch,
					ulong id, u32 offset,
					u32 payload, long timeout)
{
	struct platform_device *pdev = ch->g->dev;
	struct dma_buf *dmabuf;
	void *data;
	u32 *semaphore;
	int ret = 0;
	long remain;

	/* do not wait if channel has timed out */
	if (ch->has_timedout)
		return -ETIMEDOUT;

	dmabuf = dma_buf_get(id);
	if (IS_ERR(dmabuf)) {
		gk20a_err(&pdev->dev, "invalid notifier nvmap handle 0x%lx",
			   id);
		return -EINVAL;
	}

	data = dma_buf_kmap(dmabuf, offset >> PAGE_SHIFT);
	if (!data) {
		gk20a_err(&pdev->dev, "failed to map notifier memory");
		ret = -EINVAL;
		goto cleanup_put;
	}

	semaphore = data + (offset & ~PAGE_MASK);

	remain = wait_event_interruptible_timeout(
			ch->semaphore_wq,
			*semaphore == payload || ch->has_timedout,
			timeout);

	if (remain == 0 && *semaphore != payload)
		ret = -ETIMEDOUT;
	else if (remain < 0)
		ret = remain;

	dma_buf_kunmap(dmabuf, offset >> PAGE_SHIFT, data);
cleanup_put:
	dma_buf_put(dmabuf);
	return ret;
}

static int gk20a_channel_wait(struct channel_gk20a *ch,
			      struct nvgpu_wait_args *args)
{
	struct device *d = dev_from_gk20a(ch->g);
	struct dma_buf *dmabuf;
	struct notification *notif;
	struct timespec tv;
	u64 jiffies;
	ulong id;
	u32 offset;
	unsigned long timeout;
	int remain, ret = 0;

	gk20a_dbg_fn("");

	if (ch->has_timedout)
		return -ETIMEDOUT;

	if (args->timeout == NVGPU_NO_TIMEOUT)
		timeout = MAX_SCHEDULE_TIMEOUT;
	else
		timeout = (u32)msecs_to_jiffies(args->timeout);

	switch (args->type) {
	case NVGPU_WAIT_TYPE_NOTIFIER:
		id = args->condition.notifier.dmabuf_fd;
		offset = args->condition.notifier.offset;

		dmabuf = dma_buf_get(id);
		if (IS_ERR(dmabuf)) {
			gk20a_err(d, "invalid notifier nvmap handle 0x%lx",
				   id);
			return -EINVAL;
		}

		notif = dma_buf_vmap(dmabuf);
		if (!notif) {
			gk20a_err(d, "failed to map notifier memory");
			return -ENOMEM;
		}

		notif = (struct notification *)((uintptr_t)notif + offset);

		/* user should set status pending before
		 * calling this ioctl */
		remain = wait_event_interruptible_timeout(
				ch->notifier_wq,
				notif->status == 0 || ch->has_timedout,
				timeout);

		if (remain == 0 && notif->status != 0) {
			ret = -ETIMEDOUT;
			goto notif_clean_up;
		} else if (remain < 0) {
			ret = -EINTR;
			goto notif_clean_up;
		}

		/* TBD: fill in correct information */
		jiffies = get_jiffies_64();
		jiffies_to_timespec(jiffies, &tv);
		notif->timestamp.nanoseconds[0] = tv.tv_nsec;
		notif->timestamp.nanoseconds[1] = tv.tv_sec;
		notif->info32 = 0xDEADBEEF; /* should be object name */
		notif->info16 = ch->hw_chid; /* should be method offset */

notif_clean_up:
		dma_buf_vunmap(dmabuf, notif);
		return ret;

	case NVGPU_WAIT_TYPE_SEMAPHORE:
		ret = gk20a_channel_wait_semaphore(ch,
				args->condition.semaphore.dmabuf_fd,
				args->condition.semaphore.offset,
				args->condition.semaphore.payload,
				timeout);

		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* poll events for semaphores */

static void gk20a_channel_events_enable(struct channel_gk20a_poll_events *ev)
{
	gk20a_dbg_fn("");

	mutex_lock(&ev->lock);

	ev->events_enabled = true;
	ev->num_pending_events = 0;

	mutex_unlock(&ev->lock);
}

static void gk20a_channel_events_disable(struct channel_gk20a_poll_events *ev)
{
	gk20a_dbg_fn("");

	mutex_lock(&ev->lock);

	ev->events_enabled = false;
	ev->num_pending_events = 0;

	mutex_unlock(&ev->lock);
}

static void gk20a_channel_events_clear(struct channel_gk20a_poll_events *ev)
{
	gk20a_dbg_fn("");

	mutex_lock(&ev->lock);

	if (ev->events_enabled &&
			ev->num_pending_events > 0)
		ev->num_pending_events--;

	mutex_unlock(&ev->lock);
}

static int gk20a_channel_events_ctrl(struct channel_gk20a *ch,
			  struct nvgpu_channel_events_ctrl_args *args)
{
	int ret = 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_info,
			"channel events ctrl cmd %d", args->cmd);

	switch (args->cmd) {
	case NVGPU_IOCTL_CHANNEL_EVENTS_CTRL_CMD_ENABLE:
		gk20a_channel_events_enable(&ch->poll_events);
		break;

	case NVGPU_IOCTL_CHANNEL_EVENTS_CTRL_CMD_DISABLE:
		gk20a_channel_events_disable(&ch->poll_events);
		break;

	case NVGPU_IOCTL_CHANNEL_EVENTS_CTRL_CMD_CLEAR:
		gk20a_channel_events_clear(&ch->poll_events);
		break;

	default:
		gk20a_err(dev_from_gk20a(ch->g),
			   "unrecognized channel events ctrl cmd: 0x%x",
			   args->cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

void gk20a_channel_event(struct channel_gk20a *ch)
{
	mutex_lock(&ch->poll_events.lock);

	if (ch->poll_events.events_enabled) {
		gk20a_dbg_info("posting event on channel id %d",
				ch->hw_chid);
		gk20a_dbg_info("%d channel events pending",
				ch->poll_events.num_pending_events);

		ch->poll_events.num_pending_events++;
		/* not waking up here, caller does that */
	}

	mutex_unlock(&ch->poll_events.lock);
}

unsigned int gk20a_channel_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct channel_gk20a *ch = filep->private_data;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_info, "");

	poll_wait(filep, &ch->semaphore_wq, wait);

	mutex_lock(&ch->poll_events.lock);

	if (ch->poll_events.events_enabled &&
			ch->poll_events.num_pending_events > 0) {
		gk20a_dbg_info("found pending event on channel id %d",
				ch->hw_chid);
		gk20a_dbg_info("%d channel events pending",
				ch->poll_events.num_pending_events);
		mask = (POLLPRI | POLLIN);
	}

	mutex_unlock(&ch->poll_events.lock);

	return mask;
}

static int gk20a_channel_set_priority(struct channel_gk20a *ch,
		u32 priority)
{
	u32 timeslice_timeout;
	/* set priority of graphics channel */
	switch (priority) {
	case NVGPU_PRIORITY_LOW:
		/* 64 << 3 = 512us */
		timeslice_timeout = 64;
		break;
	case NVGPU_PRIORITY_MEDIUM:
		/* 128 << 3 = 1024us */
		timeslice_timeout = 128;
		break;
	case NVGPU_PRIORITY_HIGH:
		/* 255 << 3 = 2048us */
		timeslice_timeout = 255;
		break;
	default:
		pr_err("Unsupported priority");
		return -EINVAL;
	}
	channel_gk20a_set_schedule_params(ch,
			timeslice_timeout);
	return 0;
}

static int gk20a_channel_zcull_bind(struct channel_gk20a *ch,
			    struct nvgpu_zcull_bind_args *args)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;

	gk20a_dbg_fn("");

	return g->ops.gr.bind_ctxsw_zcull(g, gr, ch,
				args->gpu_va, args->mode);
}

/* in this context the "channel" is the host1x channel which
 * maps to *all* gk20a channels */
int gk20a_channel_suspend(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	bool channels_in_use = false;
	int err;

	gk20a_dbg_fn("");

	/* wait for engine idle */
	err = g->ops.fifo.wait_engine_idle(g);
	if (err)
		return err;

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (ch->in_use) {

			gk20a_dbg_info("suspend channel %d", chid);
			/* disable channel */
			g->ops.fifo.disable_channel(ch);
			/* preempt the channel */
			g->ops.fifo.preempt_channel(g, chid);
			/* wait for channel update notifiers */
			if (ch->update_fn &&
					work_pending(&ch->update_fn_work))
				flush_work(&ch->update_fn_work);

			channels_in_use = true;
		}
	}

	if (channels_in_use) {
		g->ops.fifo.update_runlist(g, 0, ~0, false, true);

		for (chid = 0; chid < f->num_channels; chid++) {
			if (f->channel[chid].in_use)
				g->ops.fifo.unbind_channel(&f->channel[chid]);
		}
	}

	gk20a_dbg_fn("done");
	return 0;
}

/* in this context the "channel" is the host1x channel which
 * maps to *all* gk20a channels */
int gk20a_channel_resume(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	bool channels_in_use = false;

	gk20a_dbg_fn("");

	for (chid = 0; chid < f->num_channels; chid++) {
		if (f->channel[chid].in_use) {
			gk20a_dbg_info("resume channel %d", chid);
			g->ops.fifo.bind_channel(&f->channel[chid]);
			channels_in_use = true;
		}
	}

	if (channels_in_use)
		g->ops.fifo.update_runlist(g, 0, ~0, true, true);

	gk20a_dbg_fn("done");
	return 0;
}

void gk20a_channel_semaphore_wakeup(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;

	gk20a_dbg_fn("");

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *c = g->fifo.channel+chid;
		if (c->in_use) {
			gk20a_channel_event(c);
			wake_up_interruptible_all(&c->semaphore_wq);
			gk20a_channel_update(c, 0);
		}
	}
}

static int gk20a_ioctl_channel_submit_gpfifo(
	struct channel_gk20a *ch,
	struct nvgpu_submit_gpfifo_args *args)
{
	struct gk20a_fence *fence_out;
	void *gpfifo = NULL;
	u32 size;
	int ret = 0;

	gk20a_dbg_fn("");

	if (ch->has_timedout)
		return -ETIMEDOUT;

	/* zero-sized submits are allowed, since they can be used for
	 * synchronization; we might still wait and do an increment */
	size = args->num_entries * sizeof(struct nvgpu_gpfifo);
	if (size) {
		gpfifo = vmalloc(size);
		if (!gpfifo)
			return -ENOMEM;

		if (copy_from_user(gpfifo,
					(void __user *)(uintptr_t)args->gpfifo,
					size)) {
			ret = -EINVAL;
			goto clean_up;
		}
	}

	ret = gk20a_submit_channel_gpfifo(ch, gpfifo, args->num_entries,
					  args->flags, &args->fence,
					  &fence_out);

	if (ret)
		goto clean_up;

	/* Convert fence_out to something we can pass back to user space. */
	if (args->flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET) {
		if (args->flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE) {
			int fd = gk20a_fence_install_fd(fence_out);
			if (fd < 0)
				ret = fd;
			else
				args->fence.id = fd;
		} else {
			args->fence.id = fence_out->syncpt_id;
			args->fence.value = fence_out->syncpt_value;
		}
	}
	gk20a_fence_put(fence_out);

clean_up:
	vfree(gpfifo);
	return ret;
}

void gk20a_init_channel(struct gpu_ops *gops)
{
	gops->fifo.bind_channel = channel_gk20a_bind;
	gops->fifo.unbind_channel = channel_gk20a_unbind;
	gops->fifo.disable_channel = channel_gk20a_disable;
	gops->fifo.alloc_inst = channel_gk20a_alloc_inst;
	gops->fifo.free_inst = channel_gk20a_free_inst;
	gops->fifo.setup_ramfc = channel_gk20a_setup_ramfc;
}

long gk20a_channel_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct channel_gk20a *ch = filp->private_data;
	struct platform_device *dev = ch->g->dev;
	u8 buf[NVGPU_IOCTL_CHANNEL_MAX_ARG_SIZE];
	int err = 0;

	gk20a_dbg_fn("start %d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_IOCTL_CHANNEL_LAST) ||
		(_IOC_SIZE(cmd) > NVGPU_IOCTL_CHANNEL_MAX_ARG_SIZE))
		return -EINVAL;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	/* protect our sanity for threaded userspace - most of the channel is
	 * not thread safe */
	mutex_lock(&ch->ioctl_lock);

	switch (cmd) {
	case NVGPU_IOCTL_CHANNEL_OPEN:
		err = gk20a_channel_open_ioctl(ch->g,
			(struct nvgpu_channel_open_args *)buf);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_NVMAP_FD:
		break;
	case NVGPU_IOCTL_CHANNEL_ALLOC_OBJ_CTX:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = ch->g->ops.gr.alloc_obj_ctx(ch,
				(struct nvgpu_alloc_obj_ctx_args *)buf);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_FREE_OBJ_CTX:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = ch->g->ops.gr.free_obj_ctx(ch,
				(struct nvgpu_free_obj_ctx_args *)buf);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_ALLOC_GPFIFO:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_alloc_channel_gpfifo(ch,
				(struct nvgpu_alloc_gpfifo_args *)buf);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_SUBMIT_GPFIFO:
		err = gk20a_ioctl_channel_submit_gpfifo(ch,
				(struct nvgpu_submit_gpfifo_args *)buf);
		break;
	case NVGPU_IOCTL_CHANNEL_WAIT:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}

		/* waiting is thread-safe, not dropping this mutex could
		 * deadlock in certain conditions */
		mutex_unlock(&ch->ioctl_lock);

		err = gk20a_channel_wait(ch,
				(struct nvgpu_wait_args *)buf);

		mutex_lock(&ch->ioctl_lock);

		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_ZCULL_BIND:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_channel_zcull_bind(ch,
				(struct nvgpu_zcull_bind_args *)buf);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_SET_ERROR_NOTIFIER:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_init_error_notifier(ch,
				(struct nvgpu_set_error_notifier *)buf);
		gk20a_idle(dev);
		break;
#ifdef CONFIG_GK20A_CYCLE_STATS
	case NVGPU_IOCTL_CHANNEL_CYCLE_STATS:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_channel_cycle_stats(ch,
				(struct nvgpu_cycle_stats_args *)buf);
		gk20a_idle(dev);
		break;
#endif
	case NVGPU_IOCTL_CHANNEL_SET_TIMEOUT:
	{
		u32 timeout =
			(u32)((struct nvgpu_set_timeout_args *)buf)->timeout;
		gk20a_dbg(gpu_dbg_gpu_dbg, "setting timeout (%d ms) for chid %d",
			   timeout, ch->hw_chid);
		ch->timeout_ms_max = timeout;
		break;
	}
	case NVGPU_IOCTL_CHANNEL_SET_TIMEOUT_EX:
	{
		u32 timeout =
			(u32)((struct nvgpu_set_timeout_args *)buf)->timeout;
		bool timeout_debug_dump = !((u32)
			((struct nvgpu_set_timeout_ex_args *)buf)->flags &
			(1 << NVGPU_TIMEOUT_FLAG_DISABLE_DUMP));
		gk20a_dbg(gpu_dbg_gpu_dbg, "setting timeout (%d ms) for chid %d",
			   timeout, ch->hw_chid);
		ch->timeout_ms_max = timeout;
		ch->timeout_debug_dump = timeout_debug_dump;
		break;
	}
	case NVGPU_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvgpu_get_param_args *)buf)->value =
			ch->has_timedout;
		break;
	case NVGPU_IOCTL_CHANNEL_SET_PRIORITY:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		gk20a_channel_set_priority(ch,
			((struct nvgpu_set_priority_args *)buf)->priority);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_ENABLE:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		/* enable channel */
		gk20a_writel(ch->g, ccsr_channel_r(ch->hw_chid),
			gk20a_readl(ch->g, ccsr_channel_r(ch->hw_chid)) |
			ccsr_channel_enable_set_true_f());
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_DISABLE:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		/* disable channel */
		gk20a_writel(ch->g, ccsr_channel_r(ch->hw_chid),
			gk20a_readl(ch->g, ccsr_channel_r(ch->hw_chid)) |
			ccsr_channel_enable_clr_true_f());
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_PREEMPT:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_fifo_preempt(ch->g, ch);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_FORCE_RESET:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			break;
		}
		err = gk20a_fifo_force_reset_ch(ch, true);
		gk20a_idle(dev);
		break;
	case NVGPU_IOCTL_CHANNEL_EVENTS_CTRL:
		err = gk20a_channel_events_ctrl(ch,
			   (struct nvgpu_channel_events_ctrl_args *)buf);
		break;
	default:
		dev_dbg(&dev->dev, "unrecognized ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	gk20a_dbg_fn("end");

	mutex_unlock(&ch->ioctl_lock);

	return err;
}
