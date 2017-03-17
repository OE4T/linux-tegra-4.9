/*
 * GV11B fifo
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/delay.h>
#include <linux/types.h>

#include "nvgpu/semaphore.h"
#include <nvgpu/timers.h>

#include "gk20a/gk20a.h"
#include "gk20a/fifo_gk20a.h"

#include "gp10b/fifo_gp10b.h"

#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ccsr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>

#include "fifo_gv11b.h"
#include "subctx_gv11b.h"
#include "gr_gv11b.h"

#define CHANNEL_INFO_VEID0  0
#define PBDMA_SUBDEVICE_ID  1

static void gv11b_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist)
{

	u32 runlist_entry_0 = ram_rl_entry_type_tsg_v();

	if (tsg->timeslice_timeout)
		runlist_entry_0 |=
		ram_rl_entry_tsg_timeslice_scale_f(tsg->timeslice_scale) |
		ram_rl_entry_tsg_timeslice_timeout_f(tsg->timeslice_timeout);
	else
		runlist_entry_0 |=
			ram_rl_entry_tsg_timeslice_scale_f(
				ram_rl_entry_tsg_timeslice_scale_3_v()) |
			ram_rl_entry_tsg_timeslice_timeout_f(
				ram_rl_entry_tsg_timeslice_timeout_128_v());

	runlist[0] = runlist_entry_0;
	runlist[1] = ram_rl_entry_tsg_length_f(tsg->num_active_channels);
	runlist[2] = ram_rl_entry_tsg_tsgid_f(tsg->tsgid);
	runlist[3] = 0;

	gk20a_dbg_info("gv11b tsg runlist [0] %x [1]  %x [2] %x [3] %x\n",
		runlist[0], runlist[1], runlist[2], runlist[3]);

}

static void gv11b_get_ch_runlist_entry(struct channel_gk20a *c, u32 *runlist)
{
	struct gk20a *g = c->g;
	u32 addr_lo, addr_hi;
	u32 runlist_entry;

	/* Time being use 0 pbdma sequencer */
	runlist_entry = ram_rl_entry_type_channel_v() |
			ram_rl_entry_chan_runqueue_selector_f(0) |
			ram_rl_entry_chan_userd_target_f(
			ram_rl_entry_chan_userd_target_sys_mem_ncoh_v()) |
			ram_rl_entry_chan_inst_target_f(
			ram_rl_entry_chan_userd_target_sys_mem_ncoh_v());

	addr_lo = u64_lo32(c->userd_iova) >>
			ram_rl_entry_chan_userd_ptr_align_shift_v();
	addr_hi = u64_hi32(c->userd_iova);
	runlist[0] = runlist_entry | ram_rl_entry_chan_userd_ptr_lo_f(addr_lo);
	runlist[1] = ram_rl_entry_chan_userd_ptr_hi_f(addr_hi);

	addr_lo = u64_lo32(gk20a_mm_inst_block_addr(g, &c->inst_block)) >>
			ram_rl_entry_chan_inst_ptr_align_shift_v();
	addr_hi = u64_hi32(gk20a_mm_inst_block_addr(g, &c->inst_block));

	runlist[2] = ram_rl_entry_chan_inst_ptr_lo_f(addr_lo) |
				ram_rl_entry_chid_f(c->hw_chid);
	runlist[3] = ram_rl_entry_chan_inst_ptr_hi_f(addr_hi);

	gk20a_dbg_info("gv11b channel runlist [0] %x [1]  %x [2] %x [3] %x\n",
			runlist[0], runlist[1], runlist[2], runlist[3]);
}

static void gv11b_userd_writeback_config(struct gk20a *g)
{
	gk20a_writel(g, fifo_userd_writeback_r(), fifo_userd_writeback_timer_f(
				fifo_userd_writeback_timer_100us_v()));


}

static int channel_gv11b_setup_ramfc(struct channel_gk20a *c,
		u64 gpfifo_base, u32 gpfifo_entries,
		unsigned long acquire_timeout, u32 flags)
{
	struct gk20a *g = c->g;
	struct mem_desc *mem = &c->inst_block;
	u32 data;

	gk20a_dbg_fn("");

	gk20a_memset(g, mem, 0, 0, ram_fc_size_val_v());

        gk20a_mem_wr32(g, mem, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	gk20a_mem_wr32(g, mem, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	gk20a_mem_wr32(g, mem, ram_fc_signature_w(),
		c->g->ops.fifo.get_pbdma_signature(c->g));

	gk20a_mem_wr32(g, mem, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	gk20a_mem_wr32(g, mem, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(PBDMA_SUBDEVICE_ID) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	gk20a_mem_wr32(g, mem, ram_fc_target_w(),
		pbdma_target_eng_ctx_valid_true_f() |
		pbdma_target_ce_ctx_valid_true_f() |
		pbdma_target_engine_sw_f());

	gk20a_mem_wr32(g, mem, ram_fc_acquire_w(),
		g->ops.fifo.pbdma_acquire_val(acquire_timeout));

	gk20a_mem_wr32(g, mem, ram_fc_runlist_timeslice_w(),
		pbdma_runlist_timeslice_timeout_128_f() |
		pbdma_runlist_timeslice_timescale_3_f() |
		pbdma_runlist_timeslice_enable_true_f());


	gk20a_mem_wr32(g, mem, ram_fc_chid_w(), ram_fc_chid_id_f(c->hw_chid));

	/* Until full subcontext is supported, always use VEID0 */
	gk20a_mem_wr32(g, mem, ram_fc_set_channel_info_w(),
		pbdma_set_channel_info_scg_type_graphics_compute0_f() |
		pbdma_set_channel_info_veid_f(CHANNEL_INFO_VEID0));

	if (c->is_privileged_channel) {
		/* Set privilege level for channel */
		gk20a_mem_wr32(g, mem, ram_fc_config_w(),
			pbdma_config_auth_level_privileged_f());

		gk20a_fifo_setup_ramfc_for_privileged_channel(c);
	}

	/* Enable userd writeback */
	data = gk20a_mem_rd32(g, mem, ram_fc_config_w());
	data = data | pbdma_config_userd_writeback_enable_f();
	gk20a_mem_wr32(g, mem, ram_fc_config_w(),data);

	gv11b_userd_writeback_config(g);

	return channel_gp10b_commit_userd(c);
}


static void gv11b_ring_channel_doorbell(struct channel_gk20a *c)
{
	gk20a_dbg_info("channel ring door bell %d\n", c->hw_chid);

	gk20a_writel(c->g, usermode_notify_channel_pending_r(),
		usermode_notify_channel_pending_id_f(c->hw_chid));
}

static u32 gv11b_userd_gp_get(struct gk20a *g, struct channel_gk20a *c)
{
	struct mem_desc *userd_mem = &g->fifo.userd;
	u32 offset = c->hw_chid * (g->fifo.userd_entry_size / sizeof(u32));

	return gk20a_mem_rd32(g, userd_mem,
			offset + ram_userd_gp_get_w());
}

static void gv11b_userd_gp_put(struct gk20a *g, struct channel_gk20a *c)
{
	struct mem_desc *userd_mem = &g->fifo.userd;
	u32 offset = c->hw_chid * (g->fifo.userd_entry_size / sizeof(u32));

	gk20a_mem_wr32(g, userd_mem, offset + ram_userd_gp_put_w(),
							c->gpfifo.put);
	/* commit everything to cpu */
	smp_mb();

	gv11b_ring_channel_doorbell(c);
}

static void channel_gv11b_unbind(struct channel_gk20a *ch)
{
	gk20a_dbg_fn("");

	gk20a_fifo_channel_unbind(ch);
}

static u32 gv11b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

static bool gv11b_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid)
{
	return (engine_subid == gmmu_fault_client_type_gpc_v());
}

static void gv11b_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 hw_chid,
				     struct ch_state *ch_state)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(hw_chid));
	u32 status = ccsr_channel_status_v(channel);
	u32 *inst_mem;
	struct channel_gk20a *c = g->fifo.channel + hw_chid;
	struct nvgpu_semaphore_int *hw_sema = NULL;

	if (c->hw_sema)
		hw_sema = c->hw_sema;

	if (!ch_state)
		return;

	inst_mem = &ch_state->inst_block[0];

	gk20a_debug_output(o, "%d-%s, pid %d, refs: %d: ", hw_chid,
			dev_name(g->dev),
			ch_state->pid,
			ch_state->refs);
	gk20a_debug_output(o, "channel status: %s in use %s %s\n",
			ccsr_channel_enable_v(channel) ? "" : "not",
			gk20a_decode_ccsr_chan_status(status),
			ccsr_channel_busy_v(channel) ? "busy" : "not busy");
	gk20a_debug_output(o, "RAMFC : TOP: %016llx PUT: %016llx GET: %016llx "
			"FETCH: %016llx\nHEADER: %08x COUNT: %08x\n"
			"SEMAPHORE: addr hi: %08x addr lo: %08x\n"
			"payload %08x execute %08x\n",
		(u64)inst_mem[ram_fc_pb_top_level_get_w()] +
		((u64)inst_mem[ram_fc_pb_top_level_get_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_put_w()] +
		((u64)inst_mem[ram_fc_pb_put_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_get_w()] +
		((u64)inst_mem[ram_fc_pb_get_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_fetch_w()] +
		((u64)inst_mem[ram_fc_pb_fetch_hi_w()] << 32ULL),
		inst_mem[ram_fc_pb_header_w()],
		inst_mem[ram_fc_pb_count_w()],
		inst_mem[ram_fc_sem_addr_hi_w()],
		inst_mem[ram_fc_sem_addr_lo_w()],
		inst_mem[ram_fc_sem_payload_lo_w()],
		inst_mem[ram_fc_sem_execute_w()]);
	if (hw_sema)
		gk20a_debug_output(o, "SEMA STATE: value: 0x%08x "
				   "next_val: 0x%08x addr: 0x%010llx\n",
				   readl(hw_sema->value),
				   atomic_read(&hw_sema->next_value),
				   nvgpu_hw_sema_addr(hw_sema));
	gk20a_debug_output(o, "\n");
}

static void gv11b_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o)
{
	u32 i, host_num_engines;

	host_num_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	for (i = 0; i < host_num_engines; i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);

		gk20a_debug_output(o, "%s eng %d: ", dev_name(g->dev), i);
		gk20a_debug_output(o,
			"id: %d (%s), next_id: %d (%s), ctx status: %s ",
			fifo_engine_status_id_v(status),
			fifo_engine_status_id_type_v(status) ?
				"tsg" : "channel",
			fifo_engine_status_next_id_v(status),
			fifo_engine_status_next_id_type_v(status) ?
				"tsg" : "channel",
			gk20a_decode_pbdma_chan_eng_ctx_status(ctx_status));

		if (fifo_engine_status_eng_reload_v(status))
			gk20a_debug_output(o, "ctx_reload ");
		if (fifo_engine_status_faulted_v(status))
			gk20a_debug_output(o, "faulted ");
		if (fifo_engine_status_engine_v(status))
			gk20a_debug_output(o, "busy ");
		gk20a_debug_output(o, "\n");
	}
	gk20a_debug_output(o, "\n");
}

static u32 gv11b_fifo_intr_0_error_mask(struct gk20a *g)
{
	u32 intr_0_error_mask =
		fifo_intr_0_bind_error_pending_f() |
		fifo_intr_0_sched_error_pending_f() |
		fifo_intr_0_chsw_error_pending_f() |
		fifo_intr_0_fb_flush_timeout_pending_f() |
		fifo_intr_0_lb_error_pending_f();

	return intr_0_error_mask;
}

static int gv11b_fifo_poll_pbdma_chan_status(struct gk20a *g, u32 id,
				 u32 pbdma_id, unsigned int timeout_rc_type)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	u32 pbdma_stat;
	u32 chan_stat;
	int ret = -EBUSY;

	/*
	 * If the PBDMA has a stalling interrupt and receives a NACK, the PBDMA
	 * won't save out until the STALLING interrupt is cleared. Note that
	 * the stalling interrupt need not be directly addressed, as simply
	 * clearing of the interrupt bit will be sufficient to allow the PBDMA
	 * to save out. If the stalling interrupt was due to a SW method or
	 * another deterministic failure, the PBDMA will assert it when the
	 * channel is reloaded/resumed. Note that the fault will still be
	 * reported to SW.
	 */

	if (timeout_rc_type == PREEMPT_TIMEOUT_NORC) {
		/* called from recovery */
		u32 pbdma_intr_0, pbdma_intr_1;

		pbdma_intr_0 = gk20a_readl(g, pbdma_intr_0_r(pbdma_id));
		pbdma_intr_1 = gk20a_readl(g, pbdma_intr_1_r(pbdma_id));

		if (pbdma_intr_0)
			gk20a_writel(g, pbdma_intr_0_r(pbdma_id), pbdma_intr_0);
		if (pbdma_intr_1)
			gk20a_writel(g, pbdma_intr_1_r(pbdma_id), pbdma_intr_1);
	}

	/* Verify that ch/tsg is no longer on the pbdma */
	do {
		pbdma_stat = gk20a_readl(g, fifo_pbdma_status_r(pbdma_id));
		chan_stat  = fifo_pbdma_status_chan_status_v(pbdma_stat);

		gk20a_dbg_info("wait preempt pbdma");

		if (chan_stat ==
			 fifo_pbdma_status_chan_status_valid_v() ||
			chan_stat ==
			 fifo_pbdma_status_chan_status_chsw_save_v()) {

			if (id != fifo_pbdma_status_id_v(pbdma_stat)) {
				ret = 0;
				break;
			}

		} else if (chan_stat ==
			fifo_pbdma_status_chan_status_chsw_load_v()) {

			if (id != fifo_pbdma_status_next_id_v(pbdma_stat)) {
				ret = 0;
				break;
			}

		} else if (chan_stat ==
				fifo_pbdma_status_chan_status_chsw_switch_v()) {

			if ((id != fifo_pbdma_status_next_id_v(pbdma_stat)) &&
				 (id != fifo_pbdma_status_id_v(pbdma_stat))) {
				ret = 0;
				break;
			}
		} else {
			/* pbdma status is invalid i.e. it is not loaded */
			ret = 0;
			break;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(unsigned long,
				delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired_msg(&timeout,
				 "preempt timeout pbdma"));
	return ret;
}

static int gv11b_fifo_poll_eng_ctx_status(struct gk20a *g, u32 id,
			 u32 engine_idx, u32 *reset_eng_bitmask,
			 unsigned int timeout_rc_type)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	u32 eng_stat;
	u32 ctx_stat;
	int ret = -EBUSY;

	/* Check if ch/tsg has saved off the engine or if ctxsw is hung */
	do {
		eng_stat = gk20a_readl(g, fifo_engine_status_r(engine_idx));
		ctx_stat  = fifo_engine_status_ctx_status_v(eng_stat);

		if (ctx_stat ==
			 fifo_engine_status_ctx_status_ctxsw_switch_v()) {
			gk20a_dbg_info("engine save hasn't started yet");

		} else if (ctx_stat ==
			 fifo_engine_status_ctx_status_valid_v() ||
				ctx_stat ==
			 fifo_engine_status_ctx_status_ctxsw_save_v()) {

			if (id == fifo_engine_status_id_v(eng_stat)) {
				if (timeout_rc_type == PREEMPT_TIMEOUT_NORC) {
				/* called from recovery, eng seems to be hung */
					*reset_eng_bitmask |= BIT(engine_idx);
					ret = 0;
					break;
				} else {
					gk20a_dbg_info("wait preempt engine. "
					"ctx_status (valid/save)=%u", ctx_stat);
				}
			} else {
				/* context is not running on the engine */
				ret = 0;
				break;
			}

		} else if (ctx_stat ==
			 fifo_engine_status_ctx_status_ctxsw_load_v()) {

			if (id == fifo_engine_status_next_id_v(eng_stat)) {

				if (timeout_rc_type == PREEMPT_TIMEOUT_NORC) {
				/* called from recovery, eng seems to be hung */
					*reset_eng_bitmask |= BIT(engine_idx);
					ret = 0;
					break;
				} else {
					gk20a_dbg_info("wait preempt engine. "
					"ctx_status (load)=%u", ctx_stat);
				}
			} else {
				/* context is not running on the engine */
				ret = 0;
				break;
			}

		} else {
			/* Preempt should be finished */
			ret = 0;
			break;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(unsigned long,
				delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired_msg(&timeout,
				 "preempt timeout engine"));
	return ret;
}

int gv11b_fifo_is_preempt_pending(struct gk20a *g, u32 id,
		 unsigned int id_type, unsigned int timeout_rc_type)
{
	struct fifo_gk20a *f = &g->fifo;
	unsigned long runlist_served_pbdmas;
	unsigned long runlist_served_engines;
	u32 pbdma_id;
	u32 act_eng_id;
	u32 runlist_id;
	int func_ret;
	int ret = 0;

	gk20a_dbg_fn("");

	if (id_type == ID_TYPE_TSG)
		runlist_id = f->tsg[id].runlist_id;
	else
		runlist_id = f->channel[id].runlist_id;

	runlist_served_pbdmas = f->runlist_info[runlist_id].pbdma_bitmask;
	runlist_served_engines = f->runlist_info[runlist_id].eng_bitmask;

	for_each_set_bit(pbdma_id, &runlist_served_pbdmas, f->num_pbdma) {

		func_ret = gv11b_fifo_poll_pbdma_chan_status(g, id, pbdma_id,
							 timeout_rc_type);
		if (func_ret != 0) {
			gk20a_dbg_info("preempt timeout pbdma %d", pbdma_id);
			ret |= func_ret;
		}
	}

	f->runlist_info[runlist_id].reset_eng_bitmask = 0;

	for_each_set_bit(act_eng_id, &runlist_served_engines, f->num_engines) {

		func_ret = gv11b_fifo_poll_eng_ctx_status(g, id, act_eng_id,
				&f->runlist_info[runlist_id].reset_eng_bitmask,
				 timeout_rc_type);

		if (func_ret != 0) {
			gk20a_dbg_info("preempt timeout engine %d", act_eng_id);
			ret |= func_ret;
		}
	}

	return ret;
}

static int gv11b_fifo_preempt_channel(struct gk20a *g, u32 hw_chid)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 runlist_id;

	gk20a_dbg_fn("%d", hw_chid);

	runlist_id = f->channel[hw_chid].runlist_id;
	gk20a_dbg_fn("runlist_id %d", runlist_id);

	nvgpu_mutex_acquire(&f->runlist_info[runlist_id].mutex);

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, hw_chid, false);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	nvgpu_mutex_release(&f->runlist_info[runlist_id].mutex);

	return ret;
}

static int gv11b_fifo_preempt_tsg(struct gk20a *g, u32 tsgid)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 runlist_id;

	gk20a_dbg_fn("%d", tsgid);

	runlist_id = f->tsg[tsgid].runlist_id;
	gk20a_dbg_fn("runlist_id %d", runlist_id);

	nvgpu_mutex_acquire(&f->runlist_info[runlist_id].mutex);

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, tsgid, true);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	nvgpu_mutex_release(&f->runlist_info[runlist_id].mutex);

	return ret;
}

static int __locked_fifo_preempt_ch_tsg(struct gk20a *g, u32 id,
			 unsigned int id_type, unsigned int timeout_rc_type)
{
	int ret;

	/* issue preempt */
	gk20a_fifo_issue_preempt(g, id, id_type);

	/* wait for preempt */
	ret = g->ops.fifo.is_preempt_pending(g, id, id_type,
					 timeout_rc_type);

	if (ret && (timeout_rc_type == PREEMPT_TIMEOUT_RC))
		__locked_fifo_preempt_timeout_rc(g, id, id_type);

	return ret;
}


static int gv11b_fifo_preempt_ch_tsg(struct gk20a *g, u32 id,
			 unsigned int id_type, unsigned int timeout_rc_type)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 runlist_id;

	if (id_type == ID_TYPE_TSG)
		runlist_id = f->tsg[id].runlist_id;
	else if (id_type == ID_TYPE_CHANNEL)
		runlist_id = f->channel[id].runlist_id;
	else
		return -EINVAL;

	if (runlist_id >= g->fifo.max_runlists) {
		gk20a_dbg_info("runlist_id = %d", runlist_id);
		return -EINVAL;
	}

	gk20a_dbg_fn("preempt id = %d, runlist_id = %d", id, runlist_id);

	nvgpu_mutex_acquire(&f->runlist_info[runlist_id].mutex);

	mutex_ret = pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt_ch_tsg(g, id, id_type, timeout_rc_type);

	if (!mutex_ret)
		pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	nvgpu_mutex_release(&f->runlist_info[runlist_id].mutex);

	return ret;
}

static void gv11b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
{
	/*
	 * These are all errors which indicate something really wrong
	 * going on in the device
	 */
	f->intr.pbdma.device_fatal_0 =
		pbdma_intr_0_memreq_pending_f() |
		pbdma_intr_0_memack_timeout_pending_f() |
		pbdma_intr_0_memack_extra_pending_f() |
		pbdma_intr_0_memdat_timeout_pending_f() |
		pbdma_intr_0_memdat_extra_pending_f() |
		pbdma_intr_0_memflush_pending_f() |
		pbdma_intr_0_memop_pending_f() |
		pbdma_intr_0_lbconnect_pending_f() |
		pbdma_intr_0_lback_timeout_pending_f() |
		pbdma_intr_0_lback_extra_pending_f() |
		pbdma_intr_0_lbdat_timeout_pending_f() |
		pbdma_intr_0_lbdat_extra_pending_f() |
		pbdma_intr_0_pri_pending_f();

	/*
	 * These are data parsing, framing errors or others which can be
	 * recovered from with intervention... or just resetting the
	 * channel
	 */
	f->intr.pbdma.channel_fatal_0 =
		pbdma_intr_0_gpfifo_pending_f() |
		pbdma_intr_0_gpptr_pending_f() |
		pbdma_intr_0_gpentry_pending_f() |
		pbdma_intr_0_gpcrc_pending_f() |
		pbdma_intr_0_pbptr_pending_f() |
		pbdma_intr_0_pbentry_pending_f() |
		pbdma_intr_0_pbcrc_pending_f() |
		pbdma_intr_0_method_pending_f() |
		pbdma_intr_0_methodcrc_pending_f() |
		pbdma_intr_0_pbseg_pending_f() |
		pbdma_intr_0_clear_faulted_error_pending_f() |
		pbdma_intr_0_eng_reset_pending_f() |
		pbdma_intr_0_semaphore_pending_f() |
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}

static u32 gv11b_fifo_intr_0_en_mask(struct gk20a *g)
{
	u32 intr_0_en_mask;

	intr_0_en_mask = g->ops.fifo.intr_0_error_mask(g);

	intr_0_en_mask |= fifo_intr_0_runlist_event_pending_f() |
				 fifo_intr_0_pbdma_intr_pending_f();

	return intr_0_en_mask;
}

int gv11b_init_fifo_reset_enable_hw(struct gk20a *g)
{
	u32 intr_stall;
	u32 mask;
	u32 timeout;
	unsigned int i;
	u32 host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);

	gk20a_dbg_fn("");

	/* enable pmc pfifo */
	g->ops.mc.reset(g, mc_enable_pfifo_enabled_f());

	if (g->ops.clock_gating.slcg_ce2_load_gating_prod)
		g->ops.clock_gating.slcg_ce2_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.slcg_fifo_load_gating_prod)
		g->ops.clock_gating.slcg_fifo_load_gating_prod(g,
				g->slcg_enabled);
	if (g->ops.clock_gating.blcg_fifo_load_gating_prod)
		g->ops.clock_gating.blcg_fifo_load_gating_prod(g,
				g->blcg_enabled);

	/* enable pbdma */
	mask = 0;
	for (i = 0; i < host_num_pbdma; ++i)
		mask |= mc_enable_pb_sel_f(mc_enable_pb_0_enabled_v(), i);
	gk20a_writel(g, mc_enable_pb_r(), mask);


	timeout = gk20a_readl(g, fifo_fb_timeout_r());
	timeout = set_field(timeout, fifo_fb_timeout_period_m(),
			fifo_fb_timeout_period_init_f());
	gk20a_dbg_info("fifo_fb_timeout reg val = 0x%08x", timeout);
	gk20a_writel(g, fifo_fb_timeout_r(), timeout);

	/* write pbdma timeout value */
	for (i = 0; i < host_num_pbdma; i++) {
		timeout = gk20a_readl(g, pbdma_timeout_r(i));
		timeout = set_field(timeout, pbdma_timeout_period_m(),
				    pbdma_timeout_period_init_f());
		gk20a_dbg_info("pbdma_timeout reg val = 0x%08x", timeout);
		gk20a_writel(g, pbdma_timeout_r(i), timeout);
	}
	/* clear and enable pbdma interrupt */
	for (i = 0; i < host_num_pbdma; i++) {
		gk20a_writel(g, pbdma_intr_0_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_1_r(i), 0xFFFFFFFF);

		intr_stall = gk20a_readl(g, pbdma_intr_stall_r(i));
		gk20a_dbg_info("pbdma id:%u, intr_en_0 0x%08x", i, intr_stall);
		gk20a_writel(g, pbdma_intr_en_0_r(i), intr_stall);

		intr_stall = gk20a_readl(g, pbdma_intr_stall_1_r(i));
		gk20a_dbg_info("pbdma id:%u, intr_en_1 0x%08x", i, intr_stall);
		gk20a_writel(g, pbdma_intr_en_1_r(i), intr_stall);
	}

	/* clear runlist interrupts */
	gk20a_writel(g, fifo_intr_runlist_r(), ~0);

	/* clear and enable pfifo interrupt */
	gk20a_writel(g, fifo_intr_0_r(), 0xFFFFFFFF);
	mask = gv11b_fifo_intr_0_en_mask(g);
	gk20a_dbg_info("fifo_intr_en_0 0x%08x", mask);
	gk20a_writel(g, fifo_intr_en_0_r(), mask);
	gk20a_dbg_info("fifo_intr_en_1 = 0x80000000");
	gk20a_writel(g, fifo_intr_en_1_r(), 0x80000000);

	gk20a_dbg_fn("done");

	return 0;
}

void gv11b_init_fifo(struct gpu_ops *gops)
{
	gp10b_init_fifo(gops);
	/* for gv11b no need to do any thing special for fifo hw setup */
	gops->fifo.init_fifo_setup_hw = NULL;
	gops->fifo.runlist_entry_size = ram_rl_entry_size_v;
	gops->fifo.get_tsg_runlist_entry = gv11b_get_tsg_runlist_entry;
	gops->fifo.get_ch_runlist_entry = gv11b_get_ch_runlist_entry;
	gops->fifo.get_num_fifos = gv11b_fifo_get_num_fifos;
	gops->fifo.userd_gp_get = gv11b_userd_gp_get;
	gops->fifo.userd_gp_put = gv11b_userd_gp_put;
	gops->fifo.setup_ramfc = channel_gv11b_setup_ramfc;
	gops->fifo.resetup_ramfc = NULL;
	gops->fifo.unbind_channel = channel_gv11b_unbind;
	gops->fifo.eng_runlist_base_size = fifo_eng_runlist_base__size_1_v;
	gops->fifo.free_channel_ctx_header = gv11b_free_subctx_header;
	gops->fifo.device_info_fault_id = top_device_info_data_fault_id_enum_v;
	gops->fifo.is_fault_engine_subid_gpc = gv11b_is_fault_engine_subid_gpc;
	gops->fifo.trigger_mmu_fault = NULL;
	gops->fifo.dump_pbdma_status = gk20a_dump_pbdma_status;
	gops->fifo.dump_eng_status = gv11b_dump_eng_status;
	gops->fifo.dump_channel_status_ramfc = gv11b_dump_channel_status_ramfc;
	gops->fifo.intr_0_error_mask = gv11b_fifo_intr_0_error_mask;
	gops->fifo.preempt_channel = gv11b_fifo_preempt_channel;
	gops->fifo.preempt_tsg = gv11b_fifo_preempt_tsg;
	gops->fifo.is_preempt_pending = gv11b_fifo_is_preempt_pending;
	gops->fifo.preempt_ch_tsg = gv11b_fifo_preempt_ch_tsg;
	gops->fifo.init_pbdma_intr_descs = gv11b_fifo_init_pbdma_intr_descs;
	gops->fifo.reset_enable_hw = gv11b_init_fifo_reset_enable_hw;
}
