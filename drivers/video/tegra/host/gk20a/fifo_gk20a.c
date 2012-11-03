/*
 * drivers/video/tegra/host/gk20a/fifo_gk20a.c
 *
 * GK20A Graphics FIFO (gr host)
 *
 * Copyright (c) 2011, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>

#include "../dev.h"
#include "../nvhost_as.h"

#include "gk20a.h"
#include "hw_fifo_gk20a.h"
#include "hw_pbdma_gk20a.h"
#include "hw_ccsr_gk20a.h"
#include "hw_ram_gk20a.h"
#include "hw_proj_gk20a.h"
#include "hw_top_gk20a.h"
#include "hw_mc_gk20a.h"

static int init_engine_info_gk20a(struct fifo_gk20a *f)
{
	struct fifo_engine_info_gk20a *gr_info;
	const u32 gr_sw_id = ENGINE_GR_GK20A;
	u32 i;
	u32 max_info_entries = top_device_info__size_1_v();

	nvhost_dbg_fn("");

	/* all we really care about finding is the graphics entry    */
	/* especially early on in sim it probably thinks it has more */
	f->num_engines = 1;

	gr_info = f->engine_info + gr_sw_id;

	gr_info->sw_id = gr_sw_id;
	gr_info->name = "gr";
	gr_info->dev_info_id = top_device_info_type_enum_graphics_v();
	gr_info->mmu_fault_id = fifo_intr_mmu_fault_eng_id_graphics_v();
	gr_info->runlist_id = ~0;
	gr_info->pbdma_id   = ~0;
	gr_info->engine_id  = ~0;

	for (i = 0; i < max_info_entries; i++) {
		u32 table_entry = gk20a_readl(f->g, top_device_info_r(i));
		u32 entry = top_device_info_entry_v(table_entry);
		u32 engine_enum = top_device_info_type_enum_v(table_entry);
		u32 table_entry2 = 0;

		if (entry == top_device_info_entry_not_valid_v())
			continue;

		if (top_device_info_chain_v(table_entry) ==
		    top_device_info_chain_enable_v()) {

			table_entry2 = gk20a_readl(f->g,
						   top_device_info_r(++i));

			engine_enum = top_device_info_type_enum_v(table_entry2);
		}

		if (entry == top_device_info_entry_enum_v() &&
		    engine_enum == gr_info->dev_info_id) {
			int pbdma_id;
			u32 runlist_bit;

			gr_info->runlist_id =
				top_device_info_runlist_enum_v(table_entry);
			nvhost_dbg_info("gr info: runlist_id %d", gr_info->runlist_id);

			gr_info->engine_id =
				top_device_info_engine_enum_v(table_entry);
			nvhost_dbg_info("gr info: engine_id %d", gr_info->engine_id);

			runlist_bit = 1 << gr_info->runlist_id;

			for (pbdma_id = 0; pbdma_id < f->num_pbdma; pbdma_id++) {
				nvhost_dbg_info("gr info: pbdma_map[%d]=%d",
					pbdma_id, f->pbdma_map[pbdma_id]);
				if (f->pbdma_map[pbdma_id] & runlist_bit)
					break;
			}

			if (pbdma_id == f->num_pbdma) {
				nvhost_dbg(dbg_err, "busted pbmda map");
				return -EINVAL;
			}
			gr_info->pbdma_id = pbdma_id;

			break;
		}
	}

	if (gr_info->runlist_id == ~0) {
		nvhost_dbg(dbg_err, "busted device info");
		return -EINVAL;
	}

	return 0;
}

static void gk20a_remove_fifo_support(struct fifo_gk20a *f)
{
	struct mem_mgr *memmgr = mem_mgr_from_g(f->g);

	nvhost_dbg_fn("");

	if (f->channel) {
		int c;
		for (c = 0; c < f->num_channels; c++) {
			if (f->channel[c].remove_support)
				f->channel[c].remove_support(f->channel+c);
		}
		kfree(f->channel);
		f->channel = 0;
	}

	mem_op().munmap(f->userd.mem.ref, f->userd.cpu_va);
	mem_op().unpin(memmgr, f->userd.mem.ref, f->userd.mem.sgt);
	mem_op().put(memmgr, f->userd.mem.ref);
	memset(&f->userd, 0, sizeof(struct userd_desc));

	kfree(f->pbdma_map);
	f->pbdma_map = NULL;

	kfree(f->engine_info);
	f->engine_info = NULL;
}

static int fifo_gk20a_init_runlist(struct gk20a *g, struct fifo_gk20a *f)
{
	struct mem_mgr *memmgr = mem_mgr_from_g(g);
	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_runlist_info_gk20a *runlist;
	u32 engine_id;
	u32 runlist_id;
	u32 i;
	u64 runlist_size;

	nvhost_dbg_fn("");

	f->max_runlists = fifo_eng_runlist_base__size_1_v();
	f->runlist_info = kzalloc(sizeof(struct fifo_runlist_info_gk20a) *
				  f->max_runlists, GFP_KERNEL);
	if (!f->runlist_info)
		goto clean_up;

	for (engine_id = 0; engine_id < ENGINE_INVAL_GK20A; engine_id++) {
		engine_info = f->engine_info + engine_id;
		runlist_id = engine_info->runlist_id;
		runlist = &f->runlist_info[runlist_id];

		runlist->active_channels =
			kzalloc(DIV_ROUND_UP(f->num_channels, BITS_PER_BYTE),
				GFP_KERNEL);
		if (!runlist->active_channels)
			goto clean_up;

		runlist_size  = ram_rl_entry_size_v() * f->num_channels;
		for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
			runlist->mem[i].ref =
				mem_op().alloc(memmgr, runlist_size,
					    DEFAULT_ALLOC_ALIGNMENT,
					    DEFAULT_ALLOC_FLAGS,
					    0);
			if (!runlist->mem[i].ref)
				goto clean_up;
			runlist->mem[i].size = runlist_size;
		}
		mutex_init(&runlist->mutex);
		init_waitqueue_head(&runlist->runlist_wq);
	}

	return 0;

clean_up:
	nvhost_dbg_fn("fail");
	for (engine_id = 0; engine_id < ENGINE_INVAL_GK20A; engine_id++) {
		engine_info = f->engine_info + engine_id;
		runlist_id = engine_info->runlist_id;
		runlist = &f->runlist_info[runlist_id];

		for (i = 0; i < MAX_RUNLIST_BUFFERS; i++)
			mem_op().put(memmgr,
				   runlist->mem[i].ref);

		kfree(runlist->active_channels);
	}

	kfree(f->runlist_info);
	f->runlist_info = NULL;

	return -ENOMEM;
}

static int gk20a_init_fifo_reset_enable_hw(struct gk20a *g)
{
	u32 pmc_enable;
	u32 intr_stall;
	u32 mask;
	u32 timeout;
	int i;

	nvhost_dbg_fn("");

	/* enable pmc pfifo */
	pmc_enable = gk20a_readl(g, mc_enable_r());
	pmc_enable &= ~mc_enable_pfifo_enabled_f();
	pmc_enable &= ~mc_enable_ce2_enabled_f();
	pmc_enable &= ~mc_enable_priv_ring_enabled_f();
	gk20a_writel(g, mc_enable_r(), pmc_enable);

	pmc_enable = gk20a_readl(g, mc_enable_r());
	pmc_enable |= mc_enable_pfifo_enabled_f();
	pmc_enable |= mc_enable_ce2_enabled_f();
	pmc_enable |= mc_enable_priv_ring_enabled_f();
	gk20a_writel(g, mc_enable_r(), pmc_enable);
	gk20a_readl(g, mc_enable_r());

	/* enable pbdma */
	mask = 0;
	for (i = 0; i < proj_host_num_pbdma_v(); ++i)
		mask |= mc_enable_pb_sel_f(mc_enable_pb_0_enabled_v(), i);
	gk20a_writel(g, mc_enable_pb_r(), mask);

	/* enable pfifo interrupt */
	gk20a_writel(g, fifo_intr_0_r(), 0xFFFFFFFF);
	gk20a_writel(g, fifo_intr_en_0_r(), 0xFFFFFFFF); /* TBD: alternative intr tree*/
	gk20a_writel(g, fifo_intr_en_1_r(), 0xFFFFFFFF); /* TBD: alternative intr tree*/

	/* enable pbdma interrupt */
	mask = 0;
	for (i = 0; i < proj_host_num_pbdma_v(); i++) {
		intr_stall = gk20a_readl(g, pbdma_intr_stall_r(i));
		intr_stall &= ~pbdma_intr_stall_lbreq_enabled_f();
		gk20a_writel(g, pbdma_intr_stall_r(i), intr_stall);
		gk20a_writel(g, pbdma_intr_0_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_en_0_r(i),
			(~0) & ~pbdma_intr_en_0_lbreq_enabled_f());
		gk20a_writel(g, pbdma_intr_1_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_en_1_r(i), 0xFFFFFFFF);
	}

	/* TBD: apply overrides */

	/* TBD: BLCG prod */

	/* reset runlist interrupts */
	gk20a_writel(g, fifo_intr_runlist_r(), ~0);

	/* TBD: do we need those? */
	timeout = gk20a_readl(g, fifo_fb_timeout_r());
	timeout = set_field(timeout, fifo_fb_timeout_period_m(),
			fifo_fb_timeout_period_max_f());
	gk20a_writel(g, fifo_fb_timeout_r(), timeout);

	timeout = gk20a_readl(g, fifo_pb_timeout_r());
	timeout &= ~fifo_pb_timeout_detection_enabled_f();
	gk20a_writel(g, fifo_pb_timeout_r(), timeout);

	gk20a_reset_priv_ring(g);

	nvhost_dbg_fn("done");

	return 0;
}

static int gk20a_init_fifo_setup_sw(struct gk20a *g, bool reinit)
{
	struct mem_mgr *memmgr = mem_mgr_from_g(g);
	struct fifo_gk20a *f = &g->fifo;
	int chid, i, err;

	nvhost_dbg_fn("");

	if (reinit) {
		nvhost_dbg_fn("skip init");
		return 0;
	}

	f->g = g;

	f->num_channels = ccsr_channel__size_1_v();
	f->num_pbdma = proj_host_num_pbdma_v();
	f->max_engines = ENGINE_INVAL_GK20A;

	f->userd_entry_size = 1 << ram_userd_base_shift_v();
	f->userd_total_size = f->userd_entry_size * f->num_channels;

	f->userd.mem.ref = mem_op().alloc(memmgr, f->userd_total_size,
				       DEFAULT_ALLOC_ALIGNMENT,
				       DEFAULT_ALLOC_FLAGS,
				       0);
	if (IS_ERR_OR_NULL(f->userd.mem.ref)) {
		err = -ENOMEM;
		goto clean_up;
	}

	f->userd.cpu_va = mem_op().mmap(f->userd.mem.ref);
	/* f->userd.cpu_va = g->bar1; */
	if (IS_ERR_OR_NULL(f->userd.cpu_va)) {
		f->userd.cpu_va = NULL;
		err = -ENOMEM;
		goto clean_up;
	}

	f->userd.mem.sgt = mem_op().pin(memmgr, f->userd.mem.ref);
	f->userd.cpu_pa = sg_dma_address(f->userd.mem.sgt->sgl);
	nvhost_dbg_info("userd physical address : 0x%08x",
		   (u32)f->userd.cpu_pa);

	if (f->userd.cpu_pa == -EINVAL ||
	    f->userd.cpu_pa == -EINTR) {
		f->userd.cpu_pa = 0;
		err = -ENOMEM;
		goto clean_up;
	}

	/* bar1 va */
	f->userd.gpu_va = g->mm.bar1.vm.map(&g->mm.bar1.vm,
					    memmgr,
					    f->userd.mem.ref,
					    /*offset_align, flags, kind*/
					    4096, 0, 0);
	nvhost_dbg_info("userd bar1 va = 0x%llx", f->userd.gpu_va);

	f->userd.mem.size = f->userd_total_size;

	f->channel = kzalloc(f->num_channels * sizeof(*f->channel),
				GFP_KERNEL);
	f->pbdma_map = kzalloc(f->num_pbdma * sizeof(*f->pbdma_map),
				GFP_KERNEL);
	f->engine_info = kzalloc(f->max_engines * sizeof(*f->engine_info),
				GFP_KERNEL);

	if (!(f->channel && f->pbdma_map && f->engine_info)) {
		err = -ENOMEM;
		goto clean_up;
	}

	/* pbdma map needs to be in place before calling engine info init */
	for (i = 0; i < f->num_pbdma; ++i)
		f->pbdma_map[i] = gk20a_readl(g, fifo_pbdma_map_r(i));

	init_engine_info_gk20a(f);

	fifo_gk20a_init_runlist(g, f);

	for (chid = 0; chid < f->num_channels; chid++) {
		f->channel[chid].userd_cpu_va =
			f->userd.cpu_va + chid * f->userd_entry_size;
		f->channel[chid].userd_cpu_pa =
			f->userd.cpu_pa + chid * f->userd_entry_size;
		f->channel[chid].userd_gpu_va =
			f->userd.gpu_va + chid * f->userd_entry_size;

		gk20a_init_channel_support(g, chid);
	}
	mutex_init(&f->ch_inuse_mutex);

	f->remove_support = gk20a_remove_fifo_support;

	nvhost_dbg_fn("done");
	return 0;

clean_up:
	nvhost_dbg_fn("fail");
	mem_op().munmap(f->userd.mem.ref, f->userd.cpu_va);
	mem_op().unpin(memmgr, f->userd.mem.ref, f->userd.mem.sgt);
	mem_op().put(memmgr, f->userd.mem.ref);
	memset(&f->userd, 0, sizeof(struct userd_desc));

	kfree(f->channel);
	f->channel = NULL;
	kfree(f->pbdma_map);
	f->pbdma_map = NULL;
	kfree(f->engine_info);
	f->engine_info = NULL;

	return err;
}

static void gk20a_fifo_handle_runlist_event(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	unsigned long runlist_event;
	u32 runlist_id;

	runlist_event = gk20a_readl(g, fifo_intr_runlist_r());
	gk20a_writel(g, fifo_intr_runlist_r(), runlist_event);

	for_each_set_bit(runlist_id, &runlist_event, f->max_runlists) {
		runlist = &f->runlist_info[runlist_id];
		wake_up(&runlist->runlist_wq);
	}
}

static int gk20a_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;

	nvhost_dbg_fn("");

	/* test write, read through bar1 @ userd region before
	 * turning on the snooping */
	{
		struct fifo_gk20a *f = &g->fifo;
		u32 v, v1 = 0x33, v2 = 0x55;

		u32 bar1_vaddr = f->userd.gpu_va;
		volatile u32 *cpu_vaddr = f->userd.cpu_va;

		nvhost_dbg_info("test bar1 @ vaddr 0x%x",
			   bar1_vaddr);

		v = gk20a_bar1_readl(g, bar1_vaddr);

		*cpu_vaddr = v1;
		smp_mb();

		if (v1 != gk20a_bar1_readl(g, bar1_vaddr)) {
			nvhost_err(dev_from_gk20a(g), "bar1 broken @ gk20a!");
			return -EINVAL;
		}

		gk20a_bar1_writel(g, bar1_vaddr, v2);

		if (v2 != gk20a_bar1_readl(g, bar1_vaddr)) {
			nvhost_err(dev_from_gk20a(g), "bar1 broken @ gk20a!");
			return -EINVAL;
		}

		/* is it visible to the cpu? */
		if (*cpu_vaddr != v2) {
			nvhost_err(dev_from_gk20a(g),
				"cpu didn't see bar1 write @ %p!",
				cpu_vaddr);
			return -EINVAL;
		}

		/* put it back */
		gk20a_bar1_writel(g, bar1_vaddr, v);
	}

	/*XXX all manner of flushes and caching worries, etc */

	/* set the base for the userd region now */
	gk20a_writel(g, fifo_bar1_base_r(),
			fifo_bar1_base_ptr_f(f->userd.gpu_va >> 12) |
			fifo_bar1_base_valid_true_f());

	nvhost_dbg_fn("done");

	return 0;
}

int gk20a_init_fifo_support(struct gk20a *g, bool reinit)
{
	u32 err;

	err = gk20a_init_fifo_reset_enable_hw(g);
	if (err)
		return err;

	err = gk20a_init_fifo_setup_sw(g, reinit);
	if (err)
		return err;

	err = gk20a_init_fifo_setup_hw(g);
	if (err)
		return err;

	return err;
}

static void gk20a_fifo_handle_mmu_fault(struct gk20a *g)
{
	u32 fault_id = gk20a_readl(g, fifo_intr_mmu_fault_id_r());
	u32 fault_info;
	u32 engine_id;

	for (engine_id = 0;
	     engine_id < fifo_intr_mmu_fault_id_field__size_1_v();
	     engine_id++) {
		if ((fault_id & (1 << engine_id)) ==
		    fifo_intr_mmu_fault_id_field_not_pending_v())
			continue;

		fault_info = gk20a_readl(g,
			fifo_intr_mmu_fault_info_r(engine_id));

		nvhost_err(dev_from_gk20a(g), "mmu fault on engine %d, "
			"engine_subid %d, client %d, "
			"addr 0x%08x:0x%08x, type %d, info 0x%08x\n",
			engine_id,
			fifo_intr_mmu_fault_info_engine_subid_v(fault_info),
			fifo_intr_mmu_fault_info_client_v(fault_info),
			fifo_intr_mmu_fault_hi_r(engine_id),
			fifo_intr_mmu_fault_lo_r(engine_id),
			fifo_intr_mmu_fault_info_type_v(fault_info),
			fault_info);

		/* don't clear it yet */
		/* gk20a_writel(g, fifo_intr_mmu_fault_id_r(), fault_id); */
	}
}

void gk20a_fifo_isr(struct gk20a *g)
{
	u32 fifo_intr = gk20a_readl(g, fifo_intr_0_r());

	/* handle runlist update */
	if (fifo_intr & fifo_intr_0_runlist_event_pending_f()) {
		gk20a_fifo_handle_runlist_event(g);
		fifo_intr &= ~fifo_intr_0_runlist_event_pending_f();
	}

	/* don't clear this for now
	 * print more info for debugging */
	if (fifo_intr & fifo_intr_0_sched_error_pending_f()) {
		nvhost_err(dev_from_gk20a(g),
			"fifo sched error : 0x%08x",
			gk20a_readl(g, fifo_intr_sched_error_r()));
	}

	/* don't clear this for now
	 * print more info for debugging */
	if (fifo_intr & fifo_intr_0_mmu_fault_pending_f())
		gk20a_fifo_handle_mmu_fault(g);

	if (fifo_intr)
		nvhost_err(dev_from_gk20a(g),
			   "unhandled fifo interrupt 0x%08x\n",
			   fifo_intr);
}

int gk20a_fifo_preempt_channel(struct gk20a *g, u32 engine_id, u32 hw_chid)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	u32 runlist_id;
	u32 timeout = 2000; /* 2 sec */
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 elpg_off = 0;

	nvhost_dbg_fn("%d", hw_chid);

	runlist_id = f->engine_info[engine_id].runlist_id;
	runlist = &f->runlist_info[runlist_id];

	mutex_lock(&runlist->mutex);

	/* disable elpg if failed to acquire pmu mutex */
	elpg_off = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	if (elpg_off)
		gk20a_pmu_disable_elpg(g);

	/* issue preempt */
	gk20a_writel(g, fifo_preempt_r(),
		fifo_preempt_chid_f(hw_chid) |
		fifo_preempt_type_channel_f());

	/* wait for preempt */
	do {
		if (!(gk20a_readl(g, fifo_preempt_r()) &
			fifo_preempt_pending_true_f()))
			break;

		if (--timeout == 0) {
			nvhost_err(dev_from_gk20a(g),
				    "preempt channel %d timeout\n",
				    hw_chid);
			ret = -EBUSY;
			break;
		}
		mdelay(1);
	} while (1);

	/* re-enable elpg or release pmu mutex */
	if (elpg_off)
		gk20a_pmu_enable_elpg(g);
	else
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	mutex_unlock(&runlist->mutex);

	return ret;
}

int gk20a_fifo_enable_engine_activity(struct gk20a *g,
				struct fifo_engine_info_gk20a *eng_info)
{
	u32 enable = gk20a_readl(g, fifo_sched_disable_r());
	enable &= ~(fifo_sched_disable_true_v() >> eng_info->runlist_id);
	gk20a_writel(g, fifo_sched_disable_r(), enable);

	/* no buffered-mode ? */

	return 0;
}

int gk20a_fifo_disable_engine_activity(struct gk20a *g,
				struct fifo_engine_info_gk20a *eng_info,
				bool wait_for_idle)
{
	u32 gr_stat, pbdma_stat, chan_stat, eng_stat, ctx_stat;
	u32 pbdma_chid = ~0, engine_chid = ~0, disable;
	u32 err;

	gr_stat =
		gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	if (fifo_engine_status_engine_v(gr_stat) ==
	    fifo_engine_status_engine_busy_v() && !wait_for_idle)
		return -EBUSY;

	disable = gk20a_readl(g, fifo_sched_disable_r());
	disable = set_field(disable,
			fifo_sched_disable_runlist_m(eng_info->runlist_id),
			fifo_sched_disable_runlist_f(fifo_sched_disable_true_v(),
				eng_info->runlist_id));
	gk20a_writel(g, fifo_sched_disable_r(), disable);

	/* no buffered-mode ? */

	/* chid from pbdma status */
	pbdma_stat = gk20a_readl(g, fifo_pbdma_status_r(eng_info->pbdma_id));
	chan_stat  = fifo_pbdma_status_chan_status_v(pbdma_stat);
	if (chan_stat == fifo_pbdma_status_chan_status_valid_v() ||
	    chan_stat == fifo_pbdma_status_chan_status_chsw_save_v())
		pbdma_chid = fifo_pbdma_status_id_v(pbdma_stat);
	else if (chan_stat == fifo_pbdma_status_chan_status_chsw_load_v() ||
		 chan_stat == fifo_pbdma_status_chan_status_chsw_switch_v())
		pbdma_chid = fifo_pbdma_status_next_id_v(pbdma_stat);

	if (pbdma_chid != ~0) {
		err = gk20a_fifo_preempt_channel(g,
				eng_info->engine_id, pbdma_chid);
		if (err)
			goto clean_up;
	}

	/* chid from engine status */
	eng_stat = gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	ctx_stat  = fifo_engine_status_ctx_status_v(eng_stat);
	if (ctx_stat == fifo_engine_status_ctx_status_valid_v() ||
	    ctx_stat == fifo_engine_status_ctx_status_ctxsw_save_v())
		engine_chid = fifo_engine_status_id_v(eng_stat);
	else if (ctx_stat == fifo_engine_status_ctx_status_ctxsw_load_v() ||
		 ctx_stat == fifo_engine_status_ctx_status_ctxsw_switch_v())
		engine_chid = fifo_engine_status_next_id_v(eng_stat);

	if (engine_chid != ~0 && engine_chid != pbdma_chid) {
		err = gk20a_fifo_preempt_channel(g,
				eng_info->engine_id, engine_chid);
		if (err)
			goto clean_up;
	}

	return 0;

clean_up:
	gk20a_fifo_enable_engine_activity(g, eng_info);
	return err;
}

/* add/remove a channel from runlist
   special cases below: runlist->active_channels will NOT be changed.
   (hw_chid == ~0 && !add) means remove all active channels from runlist.
   (hw_chid == ~0 &&  add) means restore all active channels on runlist. */
int gk20a_fifo_update_runlist(struct gk20a *g,
	u32 engine_id, u32 hw_chid, bool add)
{
	struct fifo_gk20a *f = &g->fifo;
	struct mem_mgr *memmgr = mem_mgr_from_g(g);
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u32 runlist_id = ~0;
	u32 *runlist_entry_base = NULL;
	u32 *runlist_entry = NULL;
	phys_addr_t runlist_pa;
	u32 old_buf, new_buf;
	u32 chid;
	u32 count = 0;
	int remain;
	bool pending;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 elpg_off;

	runlist_id = f->engine_info[engine_id].runlist_id;
	runlist = &f->runlist_info[runlist_id];

	mutex_lock(&runlist->mutex);

	/* disable elpg if failed to acquire pmu mutex */
	elpg_off = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	if (elpg_off)
		gk20a_pmu_disable_elpg(g);

	/* valid channel, add/remove it from active list.
	   Otherwise, keep active list untouched for suspend/resume. */
	if (hw_chid != ~0) {
		if (add) {
			if (test_and_set_bit(hw_chid,
				runlist->active_channels) == 1)
				goto done;
		} else {
			if (test_and_clear_bit(hw_chid,
				runlist->active_channels) == 0)
				goto done;
		}
	}

	old_buf = runlist->cur_buffer;
	new_buf = !runlist->cur_buffer;

	nvhost_dbg_info("runlist_id : %d, switch to new buffer %p",
		runlist_id, runlist->mem[new_buf].ref);

	runlist->mem[new_buf].sgt =
		mem_op().pin(memmgr, runlist->mem[new_buf].ref);

	runlist_pa = sg_dma_address(runlist->mem[new_buf].sgt->sgl);
	if (!runlist_pa) {
		ret = -ENOMEM;
		goto clean_up;
	}

	runlist_entry_base = mem_op().mmap(runlist->mem[new_buf].ref);
	if (IS_ERR_OR_NULL(runlist_entry_base)) {
		ret = -ENOMEM;
		goto clean_up;
	}

	if (hw_chid != ~0 || /* add/remove a valid channel */
	    add /* resume to add all channels back */) {
		runlist_entry = runlist_entry_base;
		for_each_set_bit(chid,
			runlist->active_channels, f->num_channels) {
			nvhost_dbg_info("add channel %d to runlist", chid);
			runlist_entry[0] = chid;
			runlist_entry[1] = 0;
			runlist_entry += 2;
			count++;
		}
	} else	/* suspend to remove all channels */
		count = 0;

	if (count != 0) {
		gk20a_writel(g, fifo_runlist_base_r(),
			fifo_runlist_base_ptr_f(u64_lo32(runlist_pa >> 12)) |
			fifo_runlist_base_target_vid_mem_f());
	}

	gk20a_writel(g, fifo_runlist_r(),
		fifo_runlist_engine_f(runlist_id) |
		fifo_eng_runlist_length_f(count));

	remain =
		wait_event_interruptible(
			runlist->runlist_wq,
			((pending =
				gk20a_readl(g, fifo_eng_runlist_r(runlist_id)) &
				fifo_eng_runlist_pending_true_f()) == 0));

	if (remain == 0 && pending != 0) {
		nvhost_err(dev_from_gk20a(g), "runlist update timeout");
		ret = -ETIMEDOUT;
		goto clean_up;
	} else if (remain < 0) {
		nvhost_err(dev_from_gk20a(g), "runlist update interrupted");
		ret = -EINTR;
		goto clean_up;
	}

	runlist->cur_buffer = new_buf;

clean_up:
	if (ret != 0)
		mem_op().unpin(memmgr, runlist->mem[new_buf].ref,
			runlist->mem[new_buf].sgt);
	else
		mem_op().unpin(memmgr, runlist->mem[old_buf].ref,
			runlist->mem[old_buf].sgt);

	mem_op().munmap(runlist->mem[new_buf].ref,
		     runlist_entry_base);
done:

	/* re-enable elpg or release pmu mutex */
	if (elpg_off)
		gk20a_pmu_enable_elpg(g);
	else
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	mutex_unlock(&runlist->mutex);
	return ret;
}
