/*
 * drivers/video/tegra/host/gk20a/channel_gk20a.c
 *
 * GK20A Graphics channel
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
#include <linux/highmem.h> /* need for nvmap.h*/
#include <trace/events/nvhost.h>
#include <linux/scatterlist.h>


#include "../dev.h"
#include "../nvhost_as.h"

#include "gk20a.h"

#include "hw_ram_gk20a.h"
#include "hw_fifo_gk20a.h"
#include "hw_pbdma_gk20a.h"
#include "hw_ccsr_gk20a.h"
#include "chip_support.h"

static struct channel_gk20a *acquire_unused_channel(struct fifo_gk20a *f);
static void release_used_channel(struct fifo_gk20a *f, struct channel_gk20a *c);

static int alloc_priv_cmdbuf(struct channel_gk20a *c, u32 size,
			     struct priv_cmd_entry **entry);
static void free_priv_cmdbuf(struct priv_cmd_queue *q,
			     struct priv_cmd_entry *e);
static void recycle_priv_cmdbuf(struct channel_gk20a *c);

static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c);
static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c);

static int channel_gk20a_commit_userd(struct channel_gk20a *c);
static int channel_gk20a_setup_userd(struct channel_gk20a *c);
static int channel_gk20a_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries);

static void channel_gk20a_bind(struct channel_gk20a *ch_gk20a);
static void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a);

static int channel_gk20a_alloc_inst(struct gk20a *g,
				struct channel_gk20a *ch);
static void channel_gk20a_free_inst(struct gk20a *g,
				struct channel_gk20a *ch);

static int channel_gk20a_update_runlist(struct channel_gk20a *c,
					bool add);

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
	u32 addr_lo;
	u32 addr_hi;
	void *inst_ptr;

	nvhost_dbg_fn("");

	inst_ptr = mem_op().mmap(c->inst_block.mem.ref);
	if (IS_ERR(inst_ptr))
		return -ENOMEM;

	addr_lo = u64_lo32(c->vm->pdes.phys) >> 12;
	addr_hi = u64_hi32(c->vm->pdes.phys);

	nvhost_dbg_info("pde pa=0x%x addr_lo=0x%x addr_hi=0x%x",
		   c->vm->pdes.phys, addr_lo, addr_hi);

	mem_wr32(inst_ptr, ram_in_page_dir_base_lo_w(),
		ram_in_page_dir_base_target_vid_mem_f() |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(addr_lo));

	mem_wr32(inst_ptr, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(addr_hi));

	mem_wr32(inst_ptr, ram_in_adr_limit_lo_w(),
		 u64_lo32(c->vm->va_limit) | 0xFFF);

	mem_wr32(inst_ptr, ram_in_adr_limit_hi_w(),
		ram_in_adr_limit_hi_f(u64_hi32(c->vm->va_limit)));

	mem_op().munmap(c->inst_block.mem.ref, inst_ptr);

	return 0;
}

static int channel_gk20a_commit_userd(struct channel_gk20a *c)
{
	u32 addr_lo;
	u32 addr_hi;
	void *inst_ptr;

	nvhost_dbg_fn("");

	inst_ptr = mem_op().mmap(c->inst_block.mem.ref);
	if (IS_ERR(inst_ptr))
		return -ENOMEM;

	addr_lo = u64_lo32(c->userd_cpu_pa >> ram_userd_base_shift_v());
	addr_hi = u64_hi32(c->userd_cpu_pa);

	nvhost_dbg_info("channel %d : set ramfc userd 0x%08x",
		c->hw_chid, c->userd_cpu_pa);

	mem_wr32(inst_ptr, ram_in_ramfc_w() + ram_fc_userd_w(),
		 pbdma_userd_target_vid_mem_f() |
		 pbdma_userd_addr_f(addr_lo));

	mem_wr32(inst_ptr, ram_in_ramfc_w() + ram_fc_userd_hi_w(),
		 pbdma_userd_target_vid_mem_f() |
		 pbdma_userd_hi_addr_f(addr_hi));

	mem_op().munmap(c->inst_block.mem.ref, inst_ptr);

	return 0;
}

static int channel_gk20a_setup_ramfc(struct channel_gk20a *c,
				u64 gpfifo_base, u32 gpfifo_entries)
{
	void *inst_ptr;

	nvhost_dbg_fn("");

	inst_ptr = mem_op().mmap(c->inst_block.mem.ref);
	if (IS_ERR(inst_ptr))
		return -ENOMEM;

	memset(inst_ptr, 0, ram_fc_size_val_v());

	mem_wr32(inst_ptr, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	mem_wr32(inst_ptr, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	mem_wr32(inst_ptr, ram_fc_signature_w(),
		 pbdma_signature_hw_valid_f() | pbdma_signature_sw_zero_f());

	mem_wr32(inst_ptr, ram_fc_formats_w(),
		pbdma_formats_gp_fermi0_f() |
		pbdma_formats_pb_fermi1_f() |
		pbdma_formats_mp_fermi0_f());

	mem_wr32(inst_ptr, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	mem_wr32(inst_ptr, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(1) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	mem_wr32(inst_ptr, ram_fc_target_w(), pbdma_target_engine_sw_f());

	mem_wr32(inst_ptr, ram_fc_acquire_w(),
		pbdma_acquire_retry_man_2_f() |
		pbdma_acquire_retry_exp_2_f() |
		pbdma_acquire_timeout_exp_max_f() |
		pbdma_acquire_timeout_man_max_f() |
		pbdma_acquire_timeout_en_disable_f());

	mem_wr32(inst_ptr, ram_fc_eng_timeslice_w(),
		fifo_eng_timeslice_timeout_128_f() |
		fifo_eng_timeslice_timescale_3_f() |
		fifo_eng_timeslice_enable_true_f());

	mem_wr32(inst_ptr, ram_fc_pb_timeslice_w(),
		fifo_pb_timeslice_timeout_16_f() |
		fifo_pb_timeslice_timescale_0_f() |
		fifo_pb_timeslice_enable_true_f());

	mem_wr32(inst_ptr, ram_fc_chid_w(), ram_fc_chid_f(c->hw_chid));

	/* TBD: alwasy priv mode? */
	mem_wr32(inst_ptr, ram_fc_hce_ctrl_w(),
		 pbdma_hce_ctrl_hce_priv_mode_yes_f());

	mem_op().munmap(c->inst_block.mem.ref, inst_ptr);

	return 0;
}

static int channel_gk20a_setup_userd(struct channel_gk20a *c)
{
	BUG_ON(!c->userd_cpu_va);

	nvhost_dbg_fn("");

	mem_wr32(c->userd_cpu_va, ram_userd_put_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_get_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_ref_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_put_hi_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_ref_threshold_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_gp_top_level_get_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_gp_top_level_get_hi_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_get_hi_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_gp_get_w(), 0);
	mem_wr32(c->userd_cpu_va, ram_userd_gp_put_w(), 0);

	return 0;
}

static void channel_gk20a_bind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = get_gk20a(ch_gk20a->ch->dev);
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *engine_info =
		f->engine_info + ENGINE_GR_GK20A;

	u32 inst_ptr = ch_gk20a->inst_block.cpu_pa >> ram_in_base_shift_v();

	nvhost_dbg_info("bind channel %d inst ptr 0x%08x",
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

static void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = get_gk20a(ch_gk20a->ch->dev);

	nvhost_dbg_fn("");

	if (ch_gk20a->bound)
		gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->hw_chid),
			ccsr_channel_inst_ptr_f(0) |
			ccsr_channel_inst_bind_false_f());

	ch_gk20a->bound = false;
}

static int channel_gk20a_alloc_inst(struct gk20a *g,
				struct channel_gk20a *ch)
{
	struct mem_mgr *memmgr = mem_mgr_from_g(g);

	nvhost_dbg_fn("");

	ch->inst_block.mem.ref =
		mem_op().alloc(memmgr, ram_in_alloc_size_v(),
			    DEFAULT_ALLOC_ALIGNMENT,
			    DEFAULT_ALLOC_FLAGS,
			    0);

	if (IS_ERR(ch->inst_block.mem.ref)) {
		ch->inst_block.mem.ref = 0;
		goto clean_up;
	}

	ch->inst_block.mem.sgt = 
		mem_op().pin(memmgr, ch->inst_block.mem.ref);
	ch->inst_block.cpu_pa = sg_dma_address(ch->inst_block.mem.sgt->sgl);

	/* IS_ERR throws a warning here (expecting void *) */
	if (ch->inst_block.cpu_pa == -EINVAL ||
	    ch->inst_block.cpu_pa == -EINTR) {
		ch->inst_block.cpu_pa = 0;
		goto clean_up;
	}

	nvhost_dbg_info("channel %d inst block physical addr: 0x%08x",
		ch->hw_chid, ch->inst_block.cpu_pa);

	ch->inst_block.mem.size = ram_in_alloc_size_v();

	nvhost_dbg_fn("done");
	return 0;

clean_up:
	nvhost_dbg(dbg_fn | dbg_err, "fail");
	channel_gk20a_free_inst(g, ch);
	return -ENOMEM;
}

static void channel_gk20a_free_inst(struct gk20a *g,
				struct channel_gk20a *ch)
{
	struct mem_mgr *memmgr = mem_mgr_from_g(g);

	mem_op().unpin(memmgr, ch->inst_block.mem.ref, ch->inst_block.mem.sgt);
	mem_op().put(memmgr, ch->inst_block.mem.ref);
	memset(&ch->inst_block, 0, sizeof(struct inst_desc));
}

static int channel_gk20a_update_runlist(struct channel_gk20a *c,
					bool add)
{
	return gk20a_fifo_update_runlist(c->g,
		ENGINE_GR_GK20A, c->hw_chid, add);
}

void gk20a_free_channel(struct nvhost_hwctx *ctx)
{
	struct channel_gk20a *ch = ctx->priv;
	struct gk20a *g = ch->g;
	struct fifo_gk20a *f = &g->fifo;
	struct gr_gk20a *gr = &g->gr;
	struct mem_mgr *memmgr = gk20a_channel_mem_mgr(ch);
	struct vm_gk20a *ch_vm = ch->vm;

	if (!ch->bound)
		return;

	if (!gk20a_channel_as_bound(ch))
		goto unbind;

	/* stop, verify stopage */

	/* disable channel */
	gk20a_writel(g, ccsr_channel_r(ch->hw_chid),
		gk20a_readl(g, ccsr_channel_r(ch->hw_chid)) |
		ccsr_channel_enable_clr_true_f());

	/* preempt the channel */
	gk20a_fifo_preempt_channel(g,
		ENGINE_GR_GK20A, ch->hw_chid);

	/* remove channel from runlist */
	channel_gk20a_update_runlist(ch, false);

	/* release channel ctx */
	gk20a_free_channel_ctx(ch);

	gk20a_gr_flush_channel_tlb(gr);

	memset(&ch->ramfc, 0, sizeof(struct mem_desc_sub));

	/* free gpfifo */
	ch_vm->unmap(ch_vm, ch->gpfifo.gpu_va);
	mem_op().munmap(ch->gpfifo.mem.ref, ch->gpfifo.cpu_va);
	mem_op().put(memmgr, ch->gpfifo.mem.ref);
	memset(&ch->gpfifo, 0, sizeof(struct gpfifo_desc));

	ctx->priv = NULL;
	channel_gk20a_free_priv_cmdbuf(ch);

	/* release hwctx binding to the as_share */
	nvhost_as_release_share(ch_vm->as_share, ctx);

unbind:
	channel_gk20a_unbind(ch);
	channel_gk20a_free_inst(g, ch);

	ch->vpr = false;

	/* ALWAYS last */
	release_used_channel(f, ch);
}

struct nvhost_hwctx *gk20a_open_channel(struct nvhost_channel *ch,
					 struct nvhost_hwctx *ctx)
{
	struct gk20a *g = get_gk20a(ch->dev);
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch_gk20a;

	ch_gk20a = acquire_unused_channel(f);
	if (ch_gk20a == NULL) {
		/* TBD: we want to make this virtualizable */
		nvhost_err(dev_from_gk20a(g), "out of hw chids");
		return 0;
	}

	ctx->priv = ch_gk20a;
	ch_gk20a->g = g;
	/* note the ch here is the same for *EVERY* gk20a channel */
	ch_gk20a->ch = ch;
	/* but thre's one hwctx per gk20a channel */
	ch_gk20a->hwctx = ctx;

	if (channel_gk20a_alloc_inst(g, ch_gk20a)) {
		ch_gk20a->in_use = false;
		ctx->priv = 0;
		nvhost_err(dev_from_gk20a(g),
			   "failed to open gk20a channel, out of inst mem");

		return 0;
	}
	channel_gk20a_bind(ch_gk20a);

	/* The channel is *not* runnable at this point. It still needs to have
	 * an address space bound and allocate a gpfifo and grctx. */


	init_waitqueue_head(&ch_gk20a->notifier_wq);
	init_waitqueue_head(&ch_gk20a->semaphore_wq);

	return ctx;
}

#if 0
/* move to debug_gk20a.c ... */
static void dump_gpfifo(struct channel_gk20a *c)
{
	void *inst_ptr;
	u32 chid = c->hw_chid;

	nvhost_dbg_fn("");

	inst_ptr = mem_op().mmap(c->inst_block.mem.ref);
	if (IS_ERR(inst_ptr))
		return;

	nvhost_dbg_info("ramfc for channel %d:\n"
		"ramfc: gp_base 0x%08x, gp_base_hi 0x%08x, "
		"gp_fetch 0x%08x, gp_get 0x%08x, gp_put 0x%08x, "
		"pb_fetch 0x%08x, pb_fetch_hi 0x%08x, "
		"pb_get 0x%08x, pb_get_hi 0x%08x, "
		"pb_put 0x%08x, pb_put_hi 0x%08x\n"
		"userd: gp_put 0x%08x, gp_get 0x%08x, "
		"get 0x%08x, get_hi 0x%08x, "
		"put 0x%08x, put_hi 0x%08x\n"
		"pbdma: status 0x%08x, channel 0x%08x, userd 0x%08x, "
		"gp_base 0x%08x, gp_base_hi 0x%08x, "
		"gp_fetch 0x%08x, gp_get 0x%08x, gp_put 0x%08x, "
		"pb_fetch 0x%08x, pb_fetch_hi 0x%08x, "
		"get 0x%08x, get_hi 0x%08x, put 0x%08x, put_hi 0x%08x\n"
		"channel: ccsr_channel 0x%08x",
		chid,
		mem_rd32(inst_ptr, ram_fc_gp_base_w()),
		mem_rd32(inst_ptr, ram_fc_gp_base_hi_w()),
		mem_rd32(inst_ptr, ram_fc_gp_fetch_w()),
		mem_rd32(inst_ptr, ram_fc_gp_get_w()),
		mem_rd32(inst_ptr, ram_fc_gp_put_w()),
		mem_rd32(inst_ptr, ram_fc_pb_fetch_w()),
		mem_rd32(inst_ptr, ram_fc_pb_fetch_hi_w()),
		mem_rd32(inst_ptr, ram_fc_pb_get_w()),
		mem_rd32(inst_ptr, ram_fc_pb_get_hi_w()),
		mem_rd32(inst_ptr, ram_fc_pb_put_w()),
		mem_rd32(inst_ptr, ram_fc_pb_put_hi_w()),
		mem_rd32(c->userd_cpu_va, ram_userd_gp_put_w()),
		mem_rd32(c->userd_cpu_va, ram_userd_gp_get_w()),
		mem_rd32(c->userd_cpu_va, ram_userd_get_w()),
		mem_rd32(c->userd_cpu_va, ram_userd_get_hi_w()),
		mem_rd32(c->userd_cpu_va, ram_userd_put_w()),
		mem_rd32(c->userd_cpu_va, ram_userd_put_hi_w()),
		gk20a_readl(c->g, pbdma_status_r(0)),
		gk20a_readl(c->g, pbdma_channel_r(0)),
		gk20a_readl(c->g, pbdma_userd_r(0)),
		gk20a_readl(c->g, pbdma_gp_base_r(0)),
		gk20a_readl(c->g, pbdma_gp_base_hi_r(0)),
		gk20a_readl(c->g, pbdma_gp_fetch_r(0)),
		gk20a_readl(c->g, pbdma_gp_get_r(0)),
		gk20a_readl(c->g, pbdma_gp_put_r(0)),
		gk20a_readl(c->g, pbdma_pb_fetch_r(0)),
		gk20a_readl(c->g, pbdma_pb_fetch_hi_r(0)),
		gk20a_readl(c->g, pbdma_get_r(0)),
		gk20a_readl(c->g, pbdma_get_hi_r(0)),
		gk20a_readl(c->g, pbdma_put_r(0)),
		gk20a_readl(c->g, pbdma_put_hi_r(0)),
		gk20a_readl(c->g, ccsr_channel_r(chid)));

	mem_op().munmap(c->inst_block.mem.ref, inst_ptr);
}
#endif

/* allocate private cmd buffer.
   used for inserting commands before/after user submitted buffers. */
static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c)
{
	struct device *d = dev_from_gk20a(c->g);
	struct mem_mgr *memmgr = gk20a_channel_mem_mgr(c);
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	u32 i = 0, size;

	size = GK20A_PRIV_CMDBUF_ENTRY_NUM * sizeof(u32);
	q->mem.ref = mem_op().alloc(memmgr,
			size,
			DEFAULT_ALLOC_ALIGNMENT,
			DEFAULT_ALLOC_FLAGS,
			0);
	if (IS_ERR_OR_NULL(q->mem.ref)) {
		nvhost_err(d, "ch %d : failed to allocate"
			   " priv cmd buffer(size: %d bytes)",
			   c->hw_chid, size);
		goto clean_up;
	}
	q->mem.size = size;

	q->base_ptr = (u32 *)mem_op().mmap(q->mem.ref);
	if (IS_ERR_OR_NULL(q->base_ptr)) {
		nvhost_err(d, "ch %d : failed to map cpu va"
			   "for priv cmd buffer", c->hw_chid);
		goto clean_up;
	}

	memset(q->base_ptr, 0, size);

	q->base_gva = ch_vm->map(ch_vm, memmgr,
			q->mem.ref,
			 /*offset_align, flags, kind*/
			0, 0, 0);
	if (!q->base_gva) {
		nvhost_err(d, "ch %d : failed to map gpu va"
			   "for priv cmd buffer", c->hw_chid);
		goto clean_up;
	}

	q->size = GK20A_PRIV_CMDBUF_ENTRY_NUM;

	INIT_LIST_HEAD(&q->head);
	INIT_LIST_HEAD(&q->free);

	/* pre-alloc a few entries and put them on free list */
	for (i = 0; i < GK20A_PRIV_CMDBUF_ENTRY_PRE_ALLOC_NUM; i++) {
		e = kzalloc(sizeof(struct priv_cmd_entry), GFP_KERNEL);
		if (!e) {
			nvhost_err(d, "ch %d: fail to pre-alloc cmd entry",
				c->hw_chid);
			goto clean_up;
		}
		e->pre_alloc = true;
		list_add(&e->list, &q->free);
	}

	return 0;

clean_up:
	channel_gk20a_free_priv_cmdbuf(c);
	return -ENOMEM;
}

static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c)
{
	struct mem_mgr *memmgr = gk20a_channel_mem_mgr(c);
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	struct list_head *pos, *tmp, *head;

	if (q->size == 0)
		return;

	ch_vm->unmap(ch_vm, q->base_gva);
	mem_op().munmap(q->mem.ref, q->base_ptr);
	mem_op().put(memmgr, q->mem.ref);

	/* free used list */
	head = &q->head;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		free_priv_cmdbuf(q, e);
	}

	/* free free list */
	head = &q->free;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		e->pre_alloc = false;
		free_priv_cmdbuf(q, e);
	}

	memset(q, 0, sizeof(struct priv_cmd_queue));
}

/* allocate a cmd buffer with given size. size is number of u32 entries */
static int alloc_priv_cmdbuf(struct channel_gk20a *c, u32 orig_size,
			     struct priv_cmd_entry **entry)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	struct list_head *node;
	u32 free_count;
	u32 size = orig_size;
	bool no_retry = false;

	nvhost_dbg_fn("size %d", orig_size);

	*entry = NULL;

	/* if free space in the end is less than requested, increase the size
	 * to make the real allocated space start from beginning. */
	if (q->put + size > q->size)
		size = orig_size + (q->size - q->put);

	nvhost_dbg_info("ch %d: priv cmd queue get:put %d:%d",
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

		nvhost_dbg_info("ch %d: run out of pre-alloc entries",
			c->hw_chid);

		e = kzalloc(sizeof(struct priv_cmd_entry), GFP_KERNEL);
		if (!e) {
			nvhost_err(dev_from_gk20a(c->g),
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
		e->ptr = q->base_ptr;
		e->gva = q->base_gva;
		q->put = orig_size;
	} else {
		e->ptr = q->base_ptr + q->put;
		e->gva = q->base_gva + q->put * sizeof(u32);
		q->put = (q->put + orig_size) & (q->size - 1);
	}

	/* we already handled q->put + size > q->size so BUG_ON this */
	BUG_ON(q->put > q->size);

	/* add new entry to head since we free from head */
	list_add(&e->list, &q->head);

	*entry = e;

	nvhost_dbg_fn("done");

	return 0;
}

/* Don't call this to free an explict cmd entry.
 * It doesn't update priv_cmd_queue get/put */
static void free_priv_cmdbuf(struct priv_cmd_queue *q,
			     struct priv_cmd_entry *e)
{
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
	struct priv_cmd_entry *e;
	struct list_head *pos, *tmp, *head = &q->head;
	bool wrap_around;

	nvhost_dbg_fn("");

	/* Find the most recent free entry. Free it and everything before it */
	list_for_each(pos, head) {

		e = list_entry(pos, struct priv_cmd_entry, list);

		nvhost_dbg_info("ch %d: cmd entry get:put:wrap %d:%d:%d "
			"curr get:put:wrap %d:%d:%d",
			c->hw_chid, e->gp_get, e->gp_put, e->gp_wrap,
			c->gpfifo.get, c->gpfifo.put, c->gpfifo.wrap);

		wrap_around = (c->gpfifo.wrap != e->gp_wrap);
		if (e->gp_get < e->gp_put) {
			if (c->gpfifo.get >= e->gp_put ||
			    wrap_around)
				break;
			else
				e->gp_get = c->gpfifo.get;
		} else if (e->gp_get > e->gp_put) {
			if (wrap_around &&
			    c->gpfifo.get >= e->gp_put)
				break;
			else
				e->gp_get = c->gpfifo.get;
		}
	}

	if (pos != head)
		q->get = (e->ptr - q->base_ptr) + e->size;
	else
		nvhost_dbg_info("no free entry recycled");
		return;

	head = pos->prev;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		free_priv_cmdbuf(q, e);
	}

	nvhost_dbg_fn("done");
}


int gk20a_alloc_channel_gpfifo(struct channel_gk20a *c,
			       struct nvhost_alloc_gpfifo_args *args)
{
	struct mem_mgr *memmgr = gk20a_channel_mem_mgr(c);
	struct gk20a *g = c->g;
	struct device *d = dev_from_gk20a(g);
	struct vm_gk20a *ch_vm;
	u32 gpfifo_size = roundup_pow_of_two(args->num_entries);
	u32 ret;

	/* TBD: add kernel ioctl change
	if (args->flags & NVHOST_ALLOC_GPFIFO_FLAGS_VPR_ENABLED)
		c->vpr = true; */

	/* an address space needs to have been bound at this point.   */
	if (!gk20a_channel_as_bound(c)) {
		int err;
		nvhost_warn(dev_from_gk20a(g),
			    "not bound to an address space at time of gpfifo"
			    " allocation.  Attempting to create and bind to"
			    " one...");
		/*
		 * Eventually this will be a fatal error. For now attempt to
		 * create and bind a share here.  This helps until we change
		 * clients to use the new address space API.  However doing this
		 * can mask errors in programming access to the address space
		 * through the front door...
		 */
		err = nvhost_as_alloc_and_bind_share(c->ch, c->hwctx);
		if (err || !gk20a_channel_as_bound(c)) {
			nvhost_err(dev_from_gk20a(g),
				   "not bound to address space at time"
				   " of gpfifo allocation");
			return err;
		}
	}
	ch_vm = c->vm;

	c->ramfc.offset = 0;
	c->ramfc.size = ram_in_ramfc_s() / 8;

	if (c->gpfifo.mem.ref) {
		nvhost_err(d, "channel %d :"
			   "gpfifo already allocated", c->hw_chid);
		return -EEXIST;
	}

	c->gpfifo.mem.ref = mem_op().alloc(memmgr,
			gpfifo_size * sizeof(struct gpfifo),
			DEFAULT_ALLOC_ALIGNMENT,
			DEFAULT_ALLOC_FLAGS,
			0);
	if (IS_ERR_OR_NULL(c->gpfifo.mem.ref)) {
		nvhost_err(d, "channel %d :"
			   " failed to allocate gpfifo (size: %d bytes)",
			   c->hw_chid, gpfifo_size);
		c->gpfifo.mem.ref = 0;
		return -ENOMEM;
	}
	c->gpfifo.entry_num = gpfifo_size;

	c->gpfifo.cpu_va = (struct gpfifo *)mem_op().mmap(c->gpfifo.mem.ref);
	if (IS_ERR_OR_NULL(c->gpfifo.cpu_va))
		goto clean_up;

	c->gpfifo.get = c->gpfifo.put = 0;

	c->gpfifo.gpu_va = ch_vm->map(ch_vm, memmgr,
				c->gpfifo.mem.ref,
				/*offset_align, flags, kind*/
				0, 0, 0);
	if (!c->gpfifo.gpu_va) {
		nvhost_err(d, "channel %d : failed to map"
			   " gpu_va for gpfifo", c->hw_chid);
		goto clean_up;
	}

	nvhost_dbg_info("channel %d : gpfifo_base 0x%016llx, size %d",
		c->hw_chid, c->gpfifo.gpu_va, c->gpfifo.entry_num);

	channel_gk20a_setup_ramfc(c, c->gpfifo.gpu_va, c->gpfifo.entry_num);

	channel_gk20a_setup_userd(c);
	channel_gk20a_commit_userd(c);

	/* TBD: setup engine contexts */

	ret = channel_gk20a_alloc_priv_cmdbuf(c);
	if (ret)
		goto clean_up;

	ret = channel_gk20a_update_runlist(c, true);
	if (ret)
		goto clean_up;

	nvhost_dbg_fn("done");
	return 0;

clean_up:
	nvhost_dbg(dbg_fn | dbg_err, "fail");
	ch_vm->unmap(ch_vm, c->gpfifo.gpu_va);
	mem_op().munmap(c->gpfifo.mem.ref, c->gpfifo.cpu_va);
	mem_op().put(memmgr, c->gpfifo.mem.ref);
	memset(&c->gpfifo, 0, sizeof(struct gpfifo_desc));
	return -ENOMEM;
}

int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvhost_gpfifo *gpfifo,
				u32 num_entries,
				struct nvhost_fence *fence,
				u32 flags)
{
	struct gk20a *g = c->g;
	struct nvhost_device_data *pdata = nvhost_get_devdata(g->dev);
	struct device *d = dev_from_gk20a(g);
	struct nvhost_syncpt *sp = syncpt_from_gk20a(g);
	u32 new_put, new_get;
	u32 free_count;
	u32 extra_count = 0;
	u32 i;
	u32 err = 0;
	struct priv_cmd_entry *wait_cmd = NULL;
	struct priv_cmd_entry *get_cmd = NULL;

	nvhost_dbg_info("channel %d", c->hw_chid);

	/* gp_put changed unexpectedly since last update */
	new_put = gk20a_bar1_readl(g,
			c->userd_gpu_va + 4 * ram_userd_gp_put_w());
	if (c->gpfifo.put != new_put) {
		/* BUG_ON this */
		nvhost_err(dev_from_gk20a(g), "gp_put changed unexpectedly "
			   "since last update");
		c->gpfifo.put = new_put;
	}

	/* update gp_get from userd before a new submission */
	new_get = gk20a_bar1_readl(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_gp_get_w());
	if (new_get < c->gpfifo.get)
		c->gpfifo.wrap = !c->gpfifo.wrap;

	c->gpfifo.get = new_get;

	nvhost_dbg_info("put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	free_count = (c->gpfifo.entry_num -
		(c->gpfifo.put - c->gpfifo.get) - 1) %
		c->gpfifo.entry_num;

	if ((flags & NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT) &&
	    !nvhost_syncpt_is_expired(sp, fence->syncpt_id, fence->value)) {
		alloc_priv_cmdbuf(c, 4, &wait_cmd);
		if (wait_cmd == NULL) {
			nvhost_err(d, "not enough priv cmd buffer space");
			err = -EAGAIN;
			goto clean_up;
		}
		extra_count++;
	}
	if (flags & NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_GET) {
		alloc_priv_cmdbuf(c, 6, &get_cmd);
		if (get_cmd == NULL) {
			nvhost_err(d, "not enough priv cmd buffer space");
			err = -EAGAIN;
			goto clean_up;
		}
		extra_count++;
	}

	if (num_entries + extra_count > free_count) {
		nvhost_err(d, "not enough gpfifo space");
		err = -EAGAIN;
		goto clean_up;
	}

	if (wait_cmd) {
		/* syncpoint_a */
		wait_cmd->ptr[0] = 0x2001001C;
		/* payload */
		wait_cmd->ptr[1] = fence->value;
		/* syncpoint_b */
		wait_cmd->ptr[2] = 0x2001001D;
		/* syncpt_id, switch_en, wait */
		wait_cmd->ptr[3] = (fence->syncpt_id << 8) | 0x10;

		nvhost_dbg_info("cmds for syncpt wait :\n"
			"0x%08x, 0x%08x, 0x%08x, 0x%08x",
			wait_cmd->ptr[0],
			wait_cmd->ptr[1],
			wait_cmd->ptr[2],
			wait_cmd->ptr[3]);

		nvhost_dbg_info("put %d, get %d, size %d",
			c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

		c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
			u64_lo32(wait_cmd->gva);
		c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
			u64_hi32(wait_cmd->gva) |
			(wait_cmd->size << 10);

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);

		/* save gp_put */
		wait_cmd->gp_put = c->gpfifo.put;
	}

	for (i = 0; i < num_entries; i++) {

		nvhost_dbg_info("put %d, get %d, size %d",
			c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

		/* TBD: remove else condition and deprecate the flag */
		if (flags & NVHOST_SUBMIT_GPFIFO_FLAGS_HW_FORMAT) {
			struct nvhost_gpfifo_hw *gpfifo_hw =
				(struct nvhost_gpfifo_hw *)gpfifo;
			c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
				gpfifo_hw[i].entry0;
			c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
				gpfifo_hw[i].entry1;
		} else {
			c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
				u64_lo32(gpfifo[i].gpu_va);
			c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
				u64_hi32(gpfifo[i].gpu_va) |
				(gpfifo[i].words << 10);
		}

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);
	}

	if (get_cmd) {
		fence->syncpt_id = c->hw_chid + pdata->syncpt_base;
		fence->value     = nvhost_syncpt_incr_max(sp, fence->syncpt_id, 1);

		trace_nvhost_ioctl_ctrl_syncpt_incr(fence->syncpt_id);

		/* wfi */
		get_cmd->ptr[0] = 0x2001001E;
		/* handle, ignored */
		get_cmd->ptr[1] = 0x00000000;
		/* syncpoint_a */
		get_cmd->ptr[2] = 0x2001001C;
		/* payload, ignored */
		get_cmd->ptr[3] = 0;
		/* syncpoint_b */
		get_cmd->ptr[4] = 0x2001001D;
		/* syncpt_id, incr */
		get_cmd->ptr[5] = (fence->syncpt_id << 8) | 0x1;

		nvhost_dbg_info("cmds for syncpt incr :\n"
			"0x%08x, 0x%08x, 0x%08x, 0x%08x, 0x%08x, 0x%08x",
			get_cmd->ptr[0],
			get_cmd->ptr[1],
			get_cmd->ptr[2],
			get_cmd->ptr[3],
			get_cmd->ptr[4],
			get_cmd->ptr[5]);

		nvhost_dbg_info("put %d, get %d, size %d",
			c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

		c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
			u64_lo32(get_cmd->gva);
		c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
			u64_hi32(get_cmd->gva) |
			(get_cmd->size << 10);

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);

		/* save gp_put */
		get_cmd->gp_put = c->gpfifo.put;
	}

	nvhost_dbg_info("put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	gk20a_bar1_writel(g,
		c->userd_gpu_va + 4 * ram_userd_gp_put_w(),
		c->gpfifo.put);

	nvhost_dbg_fn("done");
	return 0;

clean_up:
	nvhost_dbg(dbg_fn | dbg_err, "fail");
	return err;
}

void gk20a_remove_channel_support(struct channel_gk20a *c)
{

}

int gk20a_init_channel_support(struct gk20a *g, u32 chid)
{
	struct channel_gk20a *c = g->fifo.channel+chid;
	c->g = g;
	c->in_use = false;
	c->hw_chid = chid;
	c->bound = false;
	c->remove_support = gk20a_remove_channel_support;
	return 0;
}

int gk20a_channel_init(struct nvhost_channel *ch,
		       struct nvhost_master *host, int index)
{
	return 0;
}

int gk20a_channel_submit(struct nvhost_job *job)
{
	nvhost_dbg_fn("");
	return 0;
}

int gk20a_channel_alloc_obj(struct nvhost_channel *channel,
			u32 class_num,
			u32 *obj_id,
			u32 vaspace_share)
{
	nvhost_dbg_fn("");
	return 0;
}

int gk20a_channel_free_obj(struct nvhost_channel *channel, u32 obj_id)
{
	nvhost_dbg_fn("");
	return 0;
}

int gk20a_channel_wait(struct channel_gk20a *ch,
		       struct nvhost_wait_args *args)
{
	struct device *d = dev_from_gk20a(ch->g);
	struct platform_device *dev = ch->ch->dev;
	struct mem_mgr *memmgr = gk20a_channel_mem_mgr(ch);
	struct mem_handle *handle_ref;
	struct notification *notif;
	struct timespec tv;
	u64 jiffies;
	u32 id;
	u32 offset;
	u32 timeout;
	int remain, ret = 0;

	if (args->timeout == NVHOST_NO_TIMEOUT)
		timeout = MAX_SCHEDULE_TIMEOUT;
	else
		timeout = (u32)msecs_to_jiffies(args->timeout);

	switch (args->type) {
	case NVHOST_WAIT_TYPE_NOTIFIER:
		id = args->condition.notifier.nvmap_handle;
		offset = args->condition.notifier.offset;

		handle_ref = mem_op().get(memmgr, id, dev);
		if (!handle_ref) {
			nvhost_err(d, "invalid notifier nvmap handle 0x%08x",
				   id);
			return -EINVAL;
		}

		notif = mem_op().mmap(handle_ref);
		if (IS_ERR_OR_NULL(notif)) {
			nvhost_err(d, "failed to map notifier memory");
			return -ENOMEM;
		}

		notif = (struct notification *)((u32)notif + offset);

		/* user should set status pending before
		 * calling this ioctl */
		remain = wait_event_interruptible_timeout(
				ch->notifier_wq,
				notif->status == 0,
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
		mem_op().munmap(handle_ref, notif);
		return ret;
	case NVHOST_WAIT_TYPE_SEMAPHORE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int gk20a_channel_zcull_bind(struct channel_gk20a *ch,
			    struct nvhost_zcull_bind_args *args)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;

	nvhost_dbg_fn("");

	return gr_gk20a_bind_ctxsw_zcull(g, gr, ch,
				args->gpu_va, args->mode);
}

int gk20a_channel_zbc_set_table(struct channel_gk20a *ch,
				struct nvhost_zbc_set_table_args *args)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;
	struct zbc_entry zbc_val;
	int i;

	nvhost_dbg_fn("");

	zbc_val.format = args->format;
	zbc_val.type = args->type;

	switch (zbc_val.type) {
	case GK20A_ZBC_TYPE_COLOR:
		for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
			zbc_val.color_ds[i] = args->color_ds[i];
			zbc_val.color_l2[i] = args->color_l2[i];
		}
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		zbc_val.depth = args->depth;
		break;
	default:
		return -EINVAL;
	}

	return gr_gk20a_elpg_protected_call(g,
		gr_gk20a_add_zbc(g, gr, &zbc_val));
}

int gk20a_channel_zbc_query_table(struct channel_gk20a *ch,
				struct nvhost_zbc_query_table_args *args)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;
	struct zbc_query_params zbc_tbl;
	int i, err;

	nvhost_dbg_fn("");

	zbc_tbl.type = args->type;
	zbc_tbl.index_size = args->index_size;

	err = gr_gk20a_query_zbc(g, gr, &zbc_tbl);

	if (!err) {
		switch (zbc_tbl.type) {
		case GK20A_ZBC_TYPE_COLOR:
			for (i = 0; i < GK20A_ZBC_COLOR_VALUE_SIZE; i++) {
				args->color_ds[i] = zbc_tbl.color_ds[i];
				args->color_l2[i] = zbc_tbl.color_l2[i];
			}
			break;
		case GK20A_ZBC_TYPE_DEPTH:
			args->depth = zbc_tbl.depth;
			break;
		case GK20A_ZBC_TYPE_INVALID:
			args->index_size = zbc_tbl.index_size;
			break;
		default:
			return -EINVAL;
		}
		args->format = zbc_tbl.format;
		args->ref_cnt = zbc_tbl.ref_cnt;
	}

	return err;
}
