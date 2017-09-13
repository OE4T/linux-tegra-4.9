/*
 * Virtualized GPU Fifo
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/dma-mapping.h>
#include <trace/events/gk20a.h>

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/barrier.h>

#include "vgpu/vgpu.h"
#include "gk20a/ctxsw_trace_gk20a.h"

#include <nvgpu/hw/gk20a/hw_fifo_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>

static void vgpu_channel_bind(struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_info("bind channel %d", ch->chid);

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	nvgpu_smp_wmb();
	nvgpu_atomic_set(&ch->bound, true);
}

static void vgpu_channel_unbind(struct channel_gk20a *ch)
{

	gk20a_dbg_fn("");

	if (nvgpu_atomic_cmpxchg(&ch->bound, true, false)) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_channel_config_params *p =
				&msg.params.channel_config;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_UNBIND;
		msg.handle = vgpu_get_handle(ch->g);
		p->handle = ch->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);
	}

}

static int vgpu_channel_alloc_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_hwctx_params *p = &msg.params.channel_hwctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ALLOC_HWCTX;
	msg.handle = vgpu_get_handle(g);
	p->id = ch->chid;
	p->pid = (u64)current->tgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret) {
		nvgpu_err(g, "fail");
		return -ENOMEM;
	}

	ch->virt_ctx = p->handle;
	gk20a_dbg_fn("done");
	return 0;
}

static void vgpu_channel_free_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_hwctx_params *p = &msg.params.channel_hwctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FREE_HWCTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

static void vgpu_channel_enable(struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ENABLE;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

static void vgpu_channel_disable(struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_DISABLE;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

static int vgpu_channel_setup_ramfc(struct channel_gk20a *ch, u64 gpfifo_base,
				u32 gpfifo_entries,
				unsigned long acquire_timeout, u32 flags)
{
	struct device __maybe_unused *d = dev_from_gk20a(ch->g);
	struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(d);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ramfc_params *p = &msg.params.ramfc;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SETUP_RAMFC;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	p->gpfifo_va = gpfifo_base;
	p->num_entries = gpfifo_entries;
	p->userd_addr = ch->userd_iova;
	p->iova = mapping ? 1 : 0;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

static int vgpu_fifo_init_engine_info(struct fifo_gk20a *f)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(f->g);
	struct tegra_vgpu_engines_info *engines = &priv->constants.engines_info;
	u32 i;

	gk20a_dbg_fn("");

	if (engines->num_engines > TEGRA_VGPU_MAX_ENGINES) {
		nvgpu_err(f->g, "num_engines %d larger than max %d",
			engines->num_engines, TEGRA_VGPU_MAX_ENGINES);
		return -EINVAL;
	}

	f->num_engines = engines->num_engines;
	for (i = 0; i < f->num_engines; i++) {
		struct fifo_engine_info_gk20a *info =
					&f->engine_info[engines->info[i].engine_id];

		if (engines->info[i].engine_id >= f->max_engines) {
			nvgpu_err(f->g, "engine id %d larger than max %d",
				engines->info[i].engine_id,
				f->max_engines);
			return -EINVAL;
		}

		info->intr_mask = engines->info[i].intr_mask;
		info->reset_mask = engines->info[i].reset_mask;
		info->runlist_id = engines->info[i].runlist_id;
		info->pbdma_id = engines->info[i].pbdma_id;
		info->inst_id = engines->info[i].inst_id;
		info->pri_base = engines->info[i].pri_base;
		info->engine_enum = engines->info[i].engine_enum;
		info->fault_id = engines->info[i].fault_id;
		f->active_engines_list[i] = engines->info[i].engine_id;
	}

	gk20a_dbg_fn("done");

	return 0;
}

static int init_runlist(struct gk20a *g, struct fifo_gk20a *f)
{
	struct fifo_runlist_info_gk20a *runlist;
	struct device *d = dev_from_gk20a(g);
	unsigned int runlist_id = -1;
	u32 i;
	u64 runlist_size;

	gk20a_dbg_fn("");

	f->max_runlists = g->ops.fifo.eng_runlist_base_size();
	f->runlist_info = nvgpu_kzalloc(g,
				sizeof(struct fifo_runlist_info_gk20a) *
				f->max_runlists);
	if (!f->runlist_info)
		goto clean_up_runlist;

	memset(f->runlist_info, 0, (sizeof(struct fifo_runlist_info_gk20a) *
		f->max_runlists));

	for (runlist_id = 0; runlist_id < f->max_runlists; runlist_id++) {
		runlist = &f->runlist_info[runlist_id];

		runlist->active_channels =
			nvgpu_kzalloc(g, DIV_ROUND_UP(f->num_channels,
						      BITS_PER_BYTE));
		if (!runlist->active_channels)
			goto clean_up_runlist;

			runlist_size  = sizeof(u16) * f->num_channels;
			for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
				int err = nvgpu_dma_alloc_sys(g, runlist_size,
						&runlist->mem[i]);
				if (err) {
					dev_err(d, "memory allocation failed\n");
					goto clean_up_runlist;
				}
			}
		nvgpu_mutex_init(&runlist->mutex);

		/* None of buffers is pinned if this value doesn't change.
		    Otherwise, one of them (cur_buffer) must have been pinned. */
		runlist->cur_buffer = MAX_RUNLIST_BUFFERS;
	}

	gk20a_dbg_fn("done");
	return 0;

clean_up_runlist:
	gk20a_fifo_delete_runlist(f);
	gk20a_dbg_fn("fail");
	return -ENOMEM;
}

static int vgpu_init_fifo_setup_sw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct device *d = dev_from_gk20a(g);
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	unsigned int chid;
	int err = 0;

	gk20a_dbg_fn("");

	if (f->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	f->g = g;
	f->num_channels = priv->constants.num_channels;
	f->max_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	f->userd_entry_size = 1 << ram_userd_base_shift_v();

	err = nvgpu_dma_alloc_sys(g, f->userd_entry_size * f->num_channels,
			&f->userd);
	if (err) {
		dev_err(d, "memory allocation failed\n");
		goto clean_up;
	}

	/* bar1 va */
	if (g->ops.mm.is_bar1_supported(g)) {
		f->userd.gpu_va = vgpu_bar1_map(g, &f->userd.priv.sgt,
						f->userd.size);
		if (!f->userd.gpu_va) {
			dev_err(d, "gmmu mapping failed\n");
			goto clean_up;
		}
		/* if reduced BAR1 range is specified, use offset of 0
		 * (server returns offset assuming full BAR1 range)
		 */
		if (resource_size(g->bar1_mem) ==
				(resource_size_t)f->userd.size)
			f->userd.gpu_va = 0;
	}

	gk20a_dbg(gpu_dbg_map_v, "userd bar1 va = 0x%llx", f->userd.gpu_va);

	f->channel = nvgpu_vzalloc(g, f->num_channels * sizeof(*f->channel));
	f->tsg = nvgpu_vzalloc(g, f->num_channels * sizeof(*f->tsg));
	f->engine_info = nvgpu_kzalloc(g, f->max_engines *
				       sizeof(*f->engine_info));
	f->active_engines_list = nvgpu_kzalloc(g, f->max_engines * sizeof(u32));

	if (!(f->channel && f->tsg && f->engine_info && f->active_engines_list)) {
		err = -ENOMEM;
		goto clean_up;
	}
	memset(f->active_engines_list, 0xff, (f->max_engines * sizeof(u32)));

	g->ops.fifo.init_engine_info(f);

	init_runlist(g, f);

	nvgpu_init_list_node(&f->free_chs);
	nvgpu_mutex_init(&f->free_chs_mutex);

	for (chid = 0; chid < f->num_channels; chid++) {
		f->channel[chid].userd_iova =
			nvgpu_mem_get_addr(g, &f->userd) +
			chid * f->userd_entry_size;
		f->channel[chid].userd_gpu_va =
			f->userd.gpu_va + chid * f->userd_entry_size;

		gk20a_init_channel_support(g, chid);
		gk20a_init_tsg_support(g, chid);
	}
	nvgpu_mutex_init(&f->tsg_inuse_mutex);

	err = nvgpu_channel_worker_init(g);
	if (err)
		goto clean_up;

	f->deferred_reset_pending = false;
	nvgpu_mutex_init(&f->deferred_reset_mutex);

	f->channel_base = priv->constants.channel_base;

	f->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;

clean_up:
	gk20a_dbg_fn("fail");
	/* FIXME: unmap from bar1 */
	nvgpu_dma_free(g, &f->userd);

	memset(&f->userd, 0, sizeof(f->userd));

	nvgpu_vfree(g, f->channel);
	f->channel = NULL;
	nvgpu_vfree(g, f->tsg);
	f->tsg = NULL;
	nvgpu_kfree(g, f->engine_info);
	f->engine_info = NULL;
	nvgpu_kfree(g, f->active_engines_list);
	f->active_engines_list = NULL;

	return err;
}

static int vgpu_init_fifo_setup_hw(struct gk20a *g)
{
	gk20a_dbg_fn("");

	/* test write, read through bar1 @ userd region before
	 * turning on the snooping */
	{
		struct fifo_gk20a *f = &g->fifo;
		u32 v, v1 = 0x33, v2 = 0x55;

		u32 bar1_vaddr = f->userd.gpu_va;
		volatile u32 *cpu_vaddr = f->userd.cpu_va;

		gk20a_dbg_info("test bar1 @ vaddr 0x%x",
			   bar1_vaddr);

		v = gk20a_bar1_readl(g, bar1_vaddr);

		*cpu_vaddr = v1;
		nvgpu_smp_mb();

		if (v1 != gk20a_bar1_readl(g, bar1_vaddr)) {
			nvgpu_err(g, "bar1 broken @ gk20a!");
			return -EINVAL;
		}

		gk20a_bar1_writel(g, bar1_vaddr, v2);

		if (v2 != gk20a_bar1_readl(g, bar1_vaddr)) {
			nvgpu_err(g, "bar1 broken @ gk20a!");
			return -EINVAL;
		}

		/* is it visible to the cpu? */
		if (*cpu_vaddr != v2) {
			nvgpu_err(g, "cpu didn't see bar1 write @ %p!",
				cpu_vaddr);
		}

		/* put it back */
		gk20a_bar1_writel(g, bar1_vaddr, v);
	}

	gk20a_dbg_fn("done");

	return 0;
}

int vgpu_init_fifo_support(struct gk20a *g)
{
	u32 err;

	gk20a_dbg_fn("");

	err = vgpu_init_fifo_setup_sw(g);
	if (err)
		return err;

	if (g->ops.fifo.init_fifo_setup_hw)
		err = g->ops.fifo.init_fifo_setup_hw(g);
	return err;
}

static int vgpu_fifo_preempt_channel(struct gk20a *g, u32 chid)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch = &f->channel[chid];
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_fn("");

	if (!nvgpu_atomic_read(&ch->bound))
		return 0;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_PREEMPT;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret) {
		nvgpu_err(g,
			"preempt channel %d failed", chid);
		err = -ENOMEM;
	}

	return err;
}

static int vgpu_fifo_preempt_tsg(struct gk20a *g, u32 tsgid)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_tsg_preempt_params *p =
			&msg.params.tsg_preempt;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_TSG_PREEMPT;
	msg.handle = vgpu_get_handle(g);
	p->tsg_id = tsgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;

	if (err) {
		nvgpu_err(g,
			"preempt tsg %u failed", tsgid);
	}

	return err;
}

static int vgpu_submit_runlist(struct gk20a *g, u64 handle, u8 runlist_id,
			       u16 *runlist, u32 num_entries)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_runlist_params *p;
	int err;
	void *oob_handle;
	void *oob;
	size_t size, oob_size;

	oob_handle = tegra_gr_comm_oob_get_ptr(TEGRA_GR_COMM_CTX_CLIENT,
			tegra_gr_comm_get_server_vmid(), TEGRA_VGPU_QUEUE_CMD,
			&oob, &oob_size);
	if (!oob_handle)
		return -EINVAL;

	size = sizeof(*runlist) * num_entries;
	if (oob_size < size) {
		err = -ENOMEM;
		goto done;
	}

	msg.cmd = TEGRA_VGPU_CMD_SUBMIT_RUNLIST;
	msg.handle = handle;
	p = &msg.params.runlist;
	p->runlist_id = runlist_id;
	p->num_entries = num_entries;

	memcpy(oob, runlist, size);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	err = (err || msg.ret) ? -1 : 0;

done:
	tegra_gr_comm_oob_put_ptr(oob_handle);
	return err;
}

static int vgpu_fifo_update_runlist_locked(struct gk20a *g, u32 runlist_id,
					u32 chid, bool add,
					bool wait_for_finish)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	u16 *runlist_entry = NULL;
	u32 count = 0;

	gk20a_dbg_fn("");

	runlist = &f->runlist_info[runlist_id];

	/* valid channel, add/remove it from active list.
	   Otherwise, keep active list untouched for suspend/resume. */
	if (chid != (u32)~0) {
		if (add) {
			if (test_and_set_bit(chid,
				runlist->active_channels) == 1)
				return 0;
		} else {
			if (test_and_clear_bit(chid,
				runlist->active_channels) == 0)
				return 0;
		}
	}

	if (chid != (u32)~0 || /* add/remove a valid channel */
	    add /* resume to add all channels back */) {
		u32 chid;

		runlist_entry = runlist->mem[0].cpu_va;
		for_each_set_bit(chid,
			runlist->active_channels, f->num_channels) {
			gk20a_dbg_info("add channel %d to runlist", chid);
			runlist_entry[0] = chid;
			runlist_entry++;
			count++;
		}
	} else	/* suspend to remove all channels */
		count = 0;

	return vgpu_submit_runlist(g, vgpu_get_handle(g), runlist_id,
				runlist->mem[0].cpu_va, count);
}

/* add/remove a channel from runlist
   special cases below: runlist->active_channels will NOT be changed.
   (chid == ~0 && !add) means remove all active channels from runlist.
   (chid == ~0 &&  add) means restore all active channels on runlist. */
static int vgpu_fifo_update_runlist(struct gk20a *g, u32 runlist_id,
				u32 chid, bool add, bool wait_for_finish)
{
	struct fifo_runlist_info_gk20a *runlist = NULL;
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;

	gk20a_dbg_fn("");

	runlist = &f->runlist_info[runlist_id];

	nvgpu_mutex_acquire(&runlist->mutex);

	ret = vgpu_fifo_update_runlist_locked(g, runlist_id, chid, add,
					wait_for_finish);

	nvgpu_mutex_release(&runlist->mutex);
	return ret;
}

static int vgpu_fifo_wait_engine_idle(struct gk20a *g)
{
	gk20a_dbg_fn("");

	return 0;
}

static int vgpu_channel_set_priority(struct channel_gk20a *ch, u32 priority)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_priority_params *p =
			&msg.params.channel_priority;
	int err;

	gk20a_dbg_info("channel %d set priority %u", ch->chid, priority);

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_PRIORITY;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	p->priority = priority;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

static int vgpu_fifo_tsg_set_runlist_interleave(struct gk20a *g,
					u32 tsgid,
					u32 runlist_id,
					u32 new_level)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_tsg_runlist_interleave_params *p =
			&msg.params.tsg_interleave;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_TSG_SET_RUNLIST_INTERLEAVE;
	msg.handle = vgpu_get_handle(g);
	p->tsg_id = tsgid;
	p->level = new_level;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
	return err ? err : msg.ret;
}

static int vgpu_fifo_set_runlist_interleave(struct gk20a *g,
					u32 id,
					bool is_tsg,
					u32 runlist_id,
					u32 new_level)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_runlist_interleave_params *p =
			&msg.params.channel_interleave;
	struct channel_gk20a *ch;
	int err;

	gk20a_dbg_fn("");

	if (is_tsg)
		return vgpu_fifo_tsg_set_runlist_interleave(g, id,
						runlist_id, new_level);

	ch = &g->fifo.channel[id];
	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_RUNLIST_INTERLEAVE;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	p->level = new_level;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
	return err ? err : msg.ret;
}

static int vgpu_channel_set_timeslice(struct channel_gk20a *ch, u32 timeslice)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_timeslice_params *p =
			&msg.params.channel_timeslice;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_TIMESLICE;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	p->timeslice_us = timeslice;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	if (!err)
		ch->timeslice_us = p->timeslice_us;
	return err;
}

static int vgpu_fifo_force_reset_ch(struct channel_gk20a *ch,
					u32 err_code, bool verbose)
{
	struct tsg_gk20a *tsg = NULL;
	struct channel_gk20a *ch_tsg = NULL;
	struct gk20a *g = ch->g;
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_fn("");

	if (gk20a_is_channel_marked_as_tsg(ch)) {
		tsg = &g->fifo.tsg[ch->tsgid];

		down_read(&tsg->ch_list_lock);

		list_for_each_entry(ch_tsg, &tsg->ch_list, ch_entry) {
			if (gk20a_channel_get(ch_tsg)) {
				gk20a_set_error_notifier(ch_tsg, err_code);
				ch_tsg->has_timedout = true;
				gk20a_channel_put(ch_tsg);
			}
		}

		up_read(&tsg->ch_list_lock);
	} else {
		gk20a_set_error_notifier(ch, err_code);
		ch->has_timedout = true;
	}

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FORCE_RESET;
	msg.handle = vgpu_get_handle(ch->g);
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
	return err ? err : msg.ret;
}

static void vgpu_fifo_set_ctx_mmu_error(struct gk20a *g,
		struct channel_gk20a *ch)
{
	nvgpu_mutex_acquire(&ch->error_notifier_mutex);
	if (ch->error_notifier_ref) {
		if (ch->error_notifier->status == 0xffff) {
			/* If error code is already set, this mmu fault
			 * was triggered as part of recovery from other
			 * error condition.
			 * Don't overwrite error flag. */
		} else {
			gk20a_set_error_notifier_locked(ch,
				NVGPU_CHANNEL_FIFO_ERROR_MMU_ERR_FLT);
		}
	}
	nvgpu_mutex_release(&ch->error_notifier_mutex);

	/* mark channel as faulted */
	ch->has_timedout = true;
	nvgpu_smp_wmb();
	/* unblock pending waits */
	nvgpu_cond_broadcast_interruptible(&ch->semaphore_wq);
	nvgpu_cond_broadcast_interruptible(&ch->notifier_wq);
}

int vgpu_fifo_isr(struct gk20a *g, struct tegra_vgpu_fifo_intr_info *info)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch = gk20a_channel_get(&f->channel[info->chid]);

	gk20a_dbg_fn("");
	if (!ch)
		return 0;

	nvgpu_err(g, "fifo intr (%d) on ch %u",
		info->type, info->chid);

	trace_gk20a_channel_reset(ch->chid, ch->tsgid);

	switch (info->type) {
	case TEGRA_VGPU_FIFO_INTR_PBDMA:
		gk20a_set_error_notifier(ch, NVGPU_CHANNEL_PBDMA_ERROR);
		break;
	case TEGRA_VGPU_FIFO_INTR_CTXSW_TIMEOUT:
		gk20a_set_error_notifier(ch,
					NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT);
		break;
	case TEGRA_VGPU_FIFO_INTR_MMU_FAULT:
		vgpu_fifo_set_ctx_mmu_error(g, ch);
		gk20a_channel_abort(ch, false);
		break;
	default:
		WARN_ON(1);
		break;
	}

	gk20a_channel_put(ch);
	return 0;
}

int vgpu_fifo_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_fifo_nonstall_intr_info *info)
{
	gk20a_dbg_fn("");

	switch (info->type) {
	case TEGRA_VGPU_FIFO_NONSTALL_INTR_CHANNEL:
		gk20a_channel_semaphore_wakeup(g, false);
		break;
	default:
		WARN_ON(1);
		break;
	}

	return 0;
}

u32 vgpu_fifo_default_timeslice_us(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	return priv->constants.default_timeslice_us;
}

void vgpu_init_fifo_ops(struct gpu_ops *gops)
{
	gops->fifo.init_fifo_setup_hw = vgpu_init_fifo_setup_hw;
	gops->fifo.bind_channel = vgpu_channel_bind;
	gops->fifo.unbind_channel = vgpu_channel_unbind;
	gops->fifo.enable_channel = vgpu_channel_enable;
	gops->fifo.disable_channel = vgpu_channel_disable;
	gops->fifo.alloc_inst = vgpu_channel_alloc_inst;
	gops->fifo.free_inst = vgpu_channel_free_inst;
	gops->fifo.setup_ramfc = vgpu_channel_setup_ramfc;
	gops->fifo.preempt_channel = vgpu_fifo_preempt_channel;
	gops->fifo.preempt_tsg = vgpu_fifo_preempt_tsg;
	gops->fifo.enable_tsg = gk20a_enable_tsg;
	gops->fifo.disable_tsg = gk20a_disable_tsg;
	gops->fifo.update_runlist = vgpu_fifo_update_runlist;
	gops->fifo.wait_engine_idle = vgpu_fifo_wait_engine_idle;
	gops->fifo.channel_set_priority = vgpu_channel_set_priority;
	gops->fifo.set_runlist_interleave = vgpu_fifo_set_runlist_interleave;
	gops->fifo.channel_set_timeslice = vgpu_channel_set_timeslice;
	gops->fifo.force_reset_ch = vgpu_fifo_force_reset_ch;
	gops->fifo.init_engine_info = vgpu_fifo_init_engine_info;
	gops->fifo.default_timeslice_us = vgpu_fifo_default_timeslice_us;
}
