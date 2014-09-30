/*
 * Virtualized GPU Fifo
 *
 * Copyright (c) 2014 NVIDIA CORPORATION.  All rights reserved.
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
#include "vgpu/vgpu.h"
#include "gk20a/hw_fifo_gk20a.h"
#include "gk20a/hw_ram_gk20a.h"

static void vgpu_channel_bind(struct channel_gk20a *ch)
{
	struct gk20a_platform *platform = gk20a_get_platform(ch->g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_info("bind channel %d", ch->hw_chid);

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND;
	msg.handle = platform->virt_handle;
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	ch->bound = true;
}

static void vgpu_channel_unbind(struct channel_gk20a *ch)
{
	struct gk20a_platform *platform = gk20a_get_platform(ch->g->dev);

	gk20a_dbg_fn("");

	if (ch->bound) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_channel_config_params *p =
				&msg.params.channel_config;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_UNBIND;
		msg.handle = platform->virt_handle;
		p->handle = ch->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);
	}

	ch->bound = false;

	/*
	 * if we are agrressive then we can destroy the syncpt
	 * resource at this point
	 * if not, then it will be destroyed at channel_free()
	 */
	if (ch->sync && ch->sync->aggressive_destroy) {
		ch->sync->destroy(ch->sync);
		ch->sync = NULL;
	}
}

static int vgpu_channel_alloc_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_hwctx_params *p = &msg.params.channel_hwctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ALLOC_HWCTX;
	msg.handle = platform->virt_handle;
	p->id = ch->hw_chid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret) {
		gk20a_err(dev_from_gk20a(g), "fail");
		return -ENOMEM;
	}

	ch->virt_ctx = p->handle;
	gk20a_dbg_fn("done");
	return 0;
}

static void vgpu_channel_free_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_hwctx_params *p = &msg.params.channel_hwctx;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_FREE_HWCTX;
	msg.handle = platform->virt_handle;
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

static void vgpu_channel_disable(struct channel_gk20a *ch)
{
	struct gk20a_platform *platform = gk20a_get_platform(ch->g->dev);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_DISABLE;
	msg.handle = platform->virt_handle;
	p->handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

static int vgpu_channel_setup_ramfc(struct channel_gk20a *ch, u64 gpfifo_base,
				u32 gpfifo_entries)
{
	struct gk20a_platform *platform = gk20a_get_platform(ch->g->dev);
	struct device __maybe_unused *d = dev_from_gk20a(ch->g);
	struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(d);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ramfc_params *p = &msg.params.ramfc;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SETUP_RAMFC;
	msg.handle = platform->virt_handle;
	p->handle = ch->virt_ctx;
	p->gpfifo_va = gpfifo_base;
	p->num_entries = gpfifo_entries;
	p->userd_addr = ch->userd_iova;
	p->iova = mapping ? 1 : 0;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

static int init_engine_info(struct fifo_gk20a *f)
{
	struct fifo_engine_info_gk20a *gr_info;
	const u32 gr_sw_id = ENGINE_GR_GK20A;

	gk20a_dbg_fn("");

	/* all we really care about finding is the graphics entry    */
	/* especially early on in sim it probably thinks it has more */
	f->num_engines = 1;

	gr_info = f->engine_info + gr_sw_id;

	gr_info->sw_id = gr_sw_id;
	gr_info->name = "gr";
	/* FIXME: retrieve this from server */
	gr_info->runlist_id = 0;
	return 0;
}

static int init_runlist(struct gk20a *g, struct fifo_gk20a *f)
{
	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_runlist_info_gk20a *runlist;
	struct device *d = dev_from_gk20a(g);
	u32 runlist_id;
	u32 i;
	u64 runlist_size;

	gk20a_dbg_fn("");

	f->max_runlists = fifo_eng_runlist_base__size_1_v();
	f->runlist_info = kzalloc(sizeof(struct fifo_runlist_info_gk20a) *
				  f->max_runlists, GFP_KERNEL);
	if (!f->runlist_info)
		goto clean_up;

	engine_info = f->engine_info + ENGINE_GR_GK20A;
	runlist_id = engine_info->runlist_id;
	runlist = &f->runlist_info[runlist_id];

	runlist->active_channels =
		kzalloc(DIV_ROUND_UP(f->num_channels, BITS_PER_BYTE),
			GFP_KERNEL);
	if (!runlist->active_channels)
		goto clean_up_runlist_info;

	runlist_size  = sizeof(u16) * f->num_channels;
	for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
		dma_addr_t iova;

		runlist->mem[i].cpuva =
			dma_alloc_coherent(d,
					runlist_size,
					&iova,
					GFP_KERNEL);
		if (!runlist->mem[i].cpuva) {
			dev_err(d, "memory allocation failed\n");
			goto clean_up_runlist;
		}
		runlist->mem[i].iova = iova;
		runlist->mem[i].size = runlist_size;
	}
	mutex_init(&runlist->mutex);

	/* None of buffers is pinned if this value doesn't change.
	    Otherwise, one of them (cur_buffer) must have been pinned. */
	runlist->cur_buffer = MAX_RUNLIST_BUFFERS;

	gk20a_dbg_fn("done");
	return 0;

clean_up_runlist:
	for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
		if (runlist->mem[i].cpuva)
			dma_free_coherent(d,
				runlist->mem[i].size,
				runlist->mem[i].cpuva,
				runlist->mem[i].iova);
		runlist->mem[i].cpuva = NULL;
		runlist->mem[i].iova = 0;
	}

	kfree(runlist->active_channels);
	runlist->active_channels = NULL;

clean_up_runlist_info:
	kfree(f->runlist_info);
	f->runlist_info = NULL;

clean_up:
	gk20a_dbg_fn("fail");
	return -ENOMEM;
}

static int vgpu_init_fifo_setup_sw(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct fifo_gk20a *f = &g->fifo;
	struct device *d = dev_from_gk20a(g);
	int chid, err = 0;
	dma_addr_t iova;

	gk20a_dbg_fn("");

	if (f->sw_ready) {
		gk20a_dbg_fn("skip init");
		return 0;
	}

	f->g = g;

	err = vgpu_get_attribute(platform->virt_handle,
				TEGRA_VGPU_ATTRIB_NUM_CHANNELS,
				&f->num_channels);
	if (err)
		return -ENXIO;

	f->max_engines = ENGINE_INVAL_GK20A;

	f->userd_entry_size = 1 << ram_userd_base_shift_v();
	f->userd_total_size = f->userd_entry_size * f->num_channels;

	f->userd.cpuva = dma_alloc_coherent(d,
					f->userd_total_size,
					&iova,
					GFP_KERNEL);
	if (!f->userd.cpuva) {
		dev_err(d, "memory allocation failed\n");
		goto clean_up;
	}

	f->userd.iova = iova;
	err = gk20a_get_sgtable(d, &f->userd.sgt,
				f->userd.cpuva, f->userd.iova,
				f->userd_total_size);
	if (err) {
		dev_err(d, "failed to create sg table\n");
		goto clean_up;
	}

	/* bar1 va */
	f->userd.gpu_va = vgpu_bar1_map(g, &f->userd.sgt, f->userd_total_size);
	if (!f->userd.gpu_va) {
		dev_err(d, "gmmu mapping failed\n");
		goto clean_up;
	}

	gk20a_dbg(gpu_dbg_map, "userd bar1 va = 0x%llx", f->userd.gpu_va);

	f->userd.size = f->userd_total_size;

	f->channel = kzalloc(f->num_channels * sizeof(*f->channel),
				GFP_KERNEL);
	f->engine_info = kzalloc(f->max_engines * sizeof(*f->engine_info),
				GFP_KERNEL);

	if (!(f->channel && f->engine_info)) {
		err = -ENOMEM;
		goto clean_up;
	}

	init_engine_info(f);

	init_runlist(g, f);

	for (chid = 0; chid < f->num_channels; chid++) {
		f->channel[chid].userd_cpu_va =
			f->userd.cpuva + chid * f->userd_entry_size;
		f->channel[chid].userd_iova =
			NV_MC_SMMU_VADDR_TRANSLATE(f->userd.iova)
				+ chid * f->userd_entry_size;
		f->channel[chid].userd_gpu_va =
			f->userd.gpu_va + chid * f->userd_entry_size;

		gk20a_init_channel_support(g, chid);
	}
	mutex_init(&f->ch_inuse_mutex);

	f->deferred_reset_pending = false;
	mutex_init(&f->deferred_reset_mutex);

	f->sw_ready = true;

	gk20a_dbg_fn("done");
	return 0;

clean_up:
	gk20a_dbg_fn("fail");
	/* FIXME: unmap from bar1 */
	if (f->userd.sgt)
		gk20a_free_sgtable(&f->userd.sgt);
	if (f->userd.cpuva)
		dma_free_coherent(d,
				f->userd_total_size,
				f->userd.cpuva,
				f->userd.iova);
	f->userd.cpuva = NULL;
	f->userd.iova = 0;

	memset(&f->userd, 0, sizeof(struct userd_desc));

	kfree(f->channel);
	f->channel = NULL;
	kfree(f->engine_info);
	f->engine_info = NULL;

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
		volatile u32 *cpu_vaddr = f->userd.cpuva;

		gk20a_dbg_info("test bar1 @ vaddr 0x%x",
			   bar1_vaddr);

		v = gk20a_bar1_readl(g, bar1_vaddr);

		*cpu_vaddr = v1;
		smp_mb();

		if (v1 != gk20a_bar1_readl(g, bar1_vaddr)) {
			gk20a_err(dev_from_gk20a(g), "bar1 broken @ gk20a!");
			return -EINVAL;
		}

		gk20a_bar1_writel(g, bar1_vaddr, v2);

		if (v2 != gk20a_bar1_readl(g, bar1_vaddr)) {
			gk20a_err(dev_from_gk20a(g), "bar1 broken @ gk20a!");
			return -EINVAL;
		}

		/* is it visible to the cpu? */
		if (*cpu_vaddr != v2) {
			gk20a_err(dev_from_gk20a(g),
				"cpu didn't see bar1 write @ %p!",
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

	err = vgpu_init_fifo_setup_hw(g);
	return err;
}

static int vgpu_fifo_preempt_channel(struct gk20a *g, u32 hw_chid)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct fifo_gk20a *f = &g->fifo;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_config_params *p =
			&msg.params.channel_config;
	int err;

	gk20a_dbg_fn("");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_PREEMPT;
	msg.handle = platform->virt_handle;
	p->handle = f->channel[hw_chid].virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret) {
		gk20a_err(dev_from_gk20a(g),
			"preempt channel %d failed\n", hw_chid);
		err = -ENOMEM;
	}

	return err;
}

static int vgpu_submit_runlist(u64 handle, u8 runlist_id, u16 *runlist,
			u32 num_entries)
{
	struct tegra_vgpu_cmd_msg *msg;
	struct tegra_vgpu_runlist_params *p;
	size_t size = sizeof(*msg) + sizeof(*runlist) * num_entries;
	char *ptr;
	int err;

	msg = kmalloc(size, GFP_KERNEL);
	if (!msg)
		return -1;

	msg->cmd = TEGRA_VGPU_CMD_SUBMIT_RUNLIST;
	msg->handle = handle;
	p = &msg->params.runlist;
	p->runlist_id = runlist_id;
	p->num_entries = num_entries;

	ptr = (char *)msg + sizeof(*msg);
	memcpy(ptr, runlist, sizeof(*runlist) * num_entries);
	err = vgpu_comm_sendrecv(msg, size, sizeof(*msg));

	err = (err || msg->ret) ? -1 : 0;
	kfree(msg);
	return err;
}

static int vgpu_fifo_update_runlist_locked(struct gk20a *g, u32 runlist_id,
					u32 hw_chid, bool add,
					bool wait_for_finish)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	u16 *runlist_entry = NULL;
	u32 count = 0;

	gk20a_dbg_fn("");

	runlist = &f->runlist_info[runlist_id];

	/* valid channel, add/remove it from active list.
	   Otherwise, keep active list untouched for suspend/resume. */
	if (hw_chid != ~0) {
		if (add) {
			if (test_and_set_bit(hw_chid,
				runlist->active_channels) == 1)
				return 0;
		} else {
			if (test_and_clear_bit(hw_chid,
				runlist->active_channels) == 0)
				return 0;
		}
	}

	if (hw_chid != ~0 || /* add/remove a valid channel */
	    add /* resume to add all channels back */) {
		u32 chid;

		runlist_entry = runlist->mem[0].cpuva;
		for_each_set_bit(chid,
			runlist->active_channels, f->num_channels) {
			gk20a_dbg_info("add channel %d to runlist", chid);
			runlist_entry[0] = chid;
			runlist_entry++;
			count++;
		}
	} else	/* suspend to remove all channels */
		count = 0;

	return vgpu_submit_runlist(platform->virt_handle, runlist_id,
				runlist->mem[0].cpuva, count);
}

/* add/remove a channel from runlist
   special cases below: runlist->active_channels will NOT be changed.
   (hw_chid == ~0 && !add) means remove all active channels from runlist.
   (hw_chid == ~0 &&  add) means restore all active channels on runlist. */
static int vgpu_fifo_update_runlist(struct gk20a *g, u32 runlist_id,
				u32 hw_chid, bool add, bool wait_for_finish)
{
	struct fifo_runlist_info_gk20a *runlist = NULL;
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;

	gk20a_dbg_fn("");

	runlist = &f->runlist_info[runlist_id];

	mutex_lock(&runlist->mutex);

	ret = vgpu_fifo_update_runlist_locked(g, runlist_id, hw_chid, add,
					wait_for_finish);

	mutex_unlock(&runlist->mutex);
	return ret;
}

static int vgpu_fifo_wait_engine_idle(struct gk20a *g)
{
	gk20a_dbg_fn("");

	return 0;
}

void vgpu_init_fifo_ops(struct gpu_ops *gops)
{
	gops->fifo.bind_channel = vgpu_channel_bind;
	gops->fifo.unbind_channel = vgpu_channel_unbind;
	gops->fifo.disable_channel = vgpu_channel_disable;
	gops->fifo.alloc_inst = vgpu_channel_alloc_inst;
	gops->fifo.free_inst = vgpu_channel_free_inst;
	gops->fifo.setup_ramfc = vgpu_channel_setup_ramfc;
	gops->fifo.preempt_channel = vgpu_fifo_preempt_channel;
	gops->fifo.update_runlist = vgpu_fifo_update_runlist;
	gops->fifo.wait_engine_idle = vgpu_fifo_wait_engine_idle;
}

