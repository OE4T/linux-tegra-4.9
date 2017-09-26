/*
 * GV11B fifo
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include <linux/delay.h>
#include <linux/types.h>

#include <nvgpu/semaphore.h>
#include <nvgpu/timers.h>
#include <nvgpu/log.h>
#include <nvgpu/dma.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/soc.h>
#include <nvgpu/debug.h>
#include <nvgpu/nvhost_t19x.h>
#include <nvgpu/barrier.h>

#include "gk20a/gk20a.h"
#include "gk20a/fifo_gk20a.h"
#include "gk20a/ctxsw_trace_gk20a.h"
#include "gk20a/channel_gk20a.h"

#include "gp10b/fifo_gp10b.h"

#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ccsr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>

#include "fifo_gv11b.h"
#include "subctx_gv11b.h"
#include "gr_gv11b.h"

#define PBDMA_SUBDEVICE_ID  1

static void gv11b_fifo_init_ramfc_eng_method_buffer(struct gk20a *g,
			struct channel_gk20a *ch, struct nvgpu_mem *mem);

static inline void gv11b_usermode_writel(struct gk20a *g, u32 r, u32 v)
{
	struct fifo_gk20a *f = &g->fifo;
	void __iomem *reg = f->t19x.usermode_regs + (r - usermode_cfg0_r());

	writel_relaxed(v, reg);
	gk20a_dbg(gpu_dbg_reg, "usermode r=0x%x v=0x%x", r, v);
}

void gv11b_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist)
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

void gv11b_get_ch_runlist_entry(struct channel_gk20a *c, u32 *runlist)
{
	struct gk20a *g = c->g;
	u32 addr_lo, addr_hi;
	u32 runlist_entry;

	/* Time being use 0 pbdma sequencer */
	runlist_entry = ram_rl_entry_type_channel_v() |
			ram_rl_entry_chan_runqueue_selector_f(
						c->t19x.runqueue_sel) |
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
				ram_rl_entry_chid_f(c->chid);
	runlist[3] = ram_rl_entry_chan_inst_ptr_hi_f(addr_hi);

	gk20a_dbg_info("gv11b channel runlist [0] %x [1]  %x [2] %x [3] %x\n",
			runlist[0], runlist[1], runlist[2], runlist[3]);
}

static void gv11b_userd_writeback_config(struct gk20a *g)
{
	gk20a_writel(g, fifo_userd_writeback_r(), fifo_userd_writeback_timer_f(
				fifo_userd_writeback_timer_100us_v()));


}

int channel_gv11b_setup_ramfc(struct channel_gk20a *c,
		u64 gpfifo_base, u32 gpfifo_entries,
		unsigned long acquire_timeout, u32 flags)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *mem = &c->inst_block;
	u32 data;

	gk20a_dbg_fn("");

	nvgpu_memset(g, mem, 0, 0, ram_fc_size_val_v());

        nvgpu_mem_wr32(g, mem, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	nvgpu_mem_wr32(g, mem, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	nvgpu_mem_wr32(g, mem, ram_fc_signature_w(),
		c->g->ops.fifo.get_pbdma_signature(c->g));

	nvgpu_mem_wr32(g, mem, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	nvgpu_mem_wr32(g, mem, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(PBDMA_SUBDEVICE_ID) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	nvgpu_mem_wr32(g, mem, ram_fc_target_w(),
		pbdma_target_eng_ctx_valid_true_f() |
		pbdma_target_ce_ctx_valid_true_f() |
		pbdma_target_engine_sw_f());

	nvgpu_mem_wr32(g, mem, ram_fc_acquire_w(),
		g->ops.fifo.pbdma_acquire_val(acquire_timeout));

	nvgpu_mem_wr32(g, mem, ram_fc_runlist_timeslice_w(),
		pbdma_runlist_timeslice_timeout_128_f() |
		pbdma_runlist_timeslice_timescale_3_f() |
		pbdma_runlist_timeslice_enable_true_f());


	nvgpu_mem_wr32(g, mem, ram_fc_chid_w(), ram_fc_chid_id_f(c->chid));

	if (c->t19x.subctx_id == CHANNEL_INFO_VEID0)
		nvgpu_mem_wr32(g, mem, ram_fc_set_channel_info_w(),
			pbdma_set_channel_info_scg_type_graphics_compute0_f() |
			pbdma_set_channel_info_veid_f(c->t19x.subctx_id));
	else
		nvgpu_mem_wr32(g, mem, ram_fc_set_channel_info_w(),
			pbdma_set_channel_info_scg_type_compute1_f() |
			pbdma_set_channel_info_veid_f(c->t19x.subctx_id));

	gv11b_fifo_init_ramfc_eng_method_buffer(g, c, mem);

	if (c->is_privileged_channel) {
		/* Set privilege level for channel */
		nvgpu_mem_wr32(g, mem, ram_fc_config_w(),
			pbdma_config_auth_level_privileged_f());

		gk20a_fifo_setup_ramfc_for_privileged_channel(c);
	}

	/* Enable userd writeback */
	data = nvgpu_mem_rd32(g, mem, ram_fc_config_w());
	data = data | pbdma_config_userd_writeback_enable_f();
	nvgpu_mem_wr32(g, mem, ram_fc_config_w(),data);

	gv11b_userd_writeback_config(g);

	return channel_gp10b_commit_userd(c);
}


static void gv11b_ring_channel_doorbell(struct channel_gk20a *c)
{
	struct fifo_gk20a *f = &c->g->fifo;
	u32 hw_chid = f->channel_base + c->chid;

	gk20a_dbg_info("channel ring door bell %d\n", c->chid);

	gv11b_usermode_writel(c->g, usermode_notify_channel_pending_r(),
		usermode_notify_channel_pending_id_f(hw_chid));
}

u32 gv11b_userd_gp_get(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_mem *userd_mem = &g->fifo.userd;
	u32 offset = c->chid * (g->fifo.userd_entry_size / sizeof(u32));

	return nvgpu_mem_rd32(g, userd_mem,
			offset + ram_userd_gp_get_w());
}

u64 gv11b_userd_pb_get(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_mem *userd_mem = &g->fifo.userd;
	u32 offset = c->chid * (g->fifo.userd_entry_size / sizeof(u32));
	u32 lo = nvgpu_mem_rd32(g, userd_mem, offset + ram_userd_get_w());
	u32 hi = nvgpu_mem_rd32(g, userd_mem, offset + ram_userd_get_hi_w());

	return ((u64)hi << 32) | lo;
}

void gv11b_userd_gp_put(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_mem *userd_mem = &g->fifo.userd;
	u32 offset = c->chid * (g->fifo.userd_entry_size / sizeof(u32));

	nvgpu_mem_wr32(g, userd_mem, offset + ram_userd_gp_put_w(),
							c->gpfifo.put);
	/* commit everything to cpu */
	nvgpu_smp_mb();

	gv11b_ring_channel_doorbell(c);
}

void channel_gv11b_unbind(struct channel_gk20a *ch)
{
	gk20a_dbg_fn("");

	gk20a_fifo_channel_unbind(ch);
}

u32 gv11b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

bool gv11b_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid)
{
	return (engine_subid == gmmu_fault_client_type_gpc_v());
}

void gv11b_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 chid,
				     struct ch_state *ch_state)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));
	u32 status = ccsr_channel_status_v(channel);
	u32 *inst_mem;
	struct channel_gk20a *c = g->fifo.channel + chid;
	struct nvgpu_semaphore_int *hw_sema = NULL;

	if (c->hw_sema)
		hw_sema = c->hw_sema;

	if (!ch_state)
		return;

	inst_mem = &ch_state->inst_block[0];

	gk20a_debug_output(o, "%d-%s, pid %d, refs: %d: ", chid,
			g->name,
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
		gk20a_debug_output(o, "SEMA STATE: value: 0x%08x next_val: 0x%08x addr: 0x%010llx\n",
				  __nvgpu_semaphore_read(hw_sema),
				  nvgpu_atomic_read(&hw_sema->next_value),
				  nvgpu_hw_sema_addr(hw_sema));
	gk20a_debug_output(o, "\n");
}

void gv11b_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o)
{
	u32 i, host_num_engines;

	host_num_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	for (i = 0; i < host_num_engines; i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);

		gk20a_debug_output(o, "%s eng %d: ", g->name, i);
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

u32 gv11b_fifo_intr_0_error_mask(struct gk20a *g)
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

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

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

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

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
				 "preempt timeout eng"));
	return ret;
}

static void gv11b_reset_eng_faulted_ch(struct gk20a *g, u32 chid)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, ccsr_channel_r(chid));
	reg_val |= ccsr_channel_eng_faulted_reset_f();
	gk20a_writel(g, ccsr_channel_r(chid), reg_val);
}

static void gv11b_reset_eng_faulted_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	down_read(&tsg->ch_list_lock);
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		gv11b_reset_eng_faulted_ch(g, ch->chid);
	}
	up_read(&tsg->ch_list_lock);
}

static void gv11b_reset_pbdma_faulted_ch(struct gk20a *g, u32 chid)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, ccsr_channel_r(chid));
	reg_val |= ccsr_channel_pbdma_faulted_reset_f();
	gk20a_writel(g, ccsr_channel_r(chid), reg_val);
}

static void gv11b_reset_pbdma_faulted_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	down_read(&tsg->ch_list_lock);
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		gv11b_reset_pbdma_faulted_ch(g, ch->chid);
	}
	up_read(&tsg->ch_list_lock);
}

void gv11b_fifo_reset_pbdma_and_eng_faulted(struct gk20a *g,
			struct channel_gk20a *refch,
			u32 faulted_pbdma, u32 faulted_engine)
{
	struct tsg_gk20a *tsg;

	nvgpu_log(g, gpu_dbg_intr, "reset faulted pbdma:0x%x eng:0x%x",
				faulted_pbdma, faulted_engine);

	if (gk20a_is_channel_marked_as_tsg(refch)) {
		tsg = &g->fifo.tsg[refch->tsgid];
		if (faulted_pbdma != FIFO_INVAL_PBDMA_ID)
			gv11b_reset_pbdma_faulted_tsg(tsg);
		if (faulted_engine != FIFO_INVAL_ENGINE_ID)
			gv11b_reset_eng_faulted_tsg(tsg);
	} else {
		if (faulted_pbdma != FIFO_INVAL_PBDMA_ID)
			gv11b_reset_pbdma_faulted_ch(g, refch->chid);
		if (faulted_engine != FIFO_INVAL_ENGINE_ID)
			gv11b_reset_eng_faulted_ch(g, refch->chid);
	}
}

static u32 gv11b_fifo_get_runlists_mask(struct gk20a *g, u32 act_eng_bitmask,
			u32 id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault)
{
	u32 runlists_mask = 0;
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	u32 pbdma_bitmask = 0;

	if (id_type != ID_TYPE_UNKNOWN) {
		if (id_type == ID_TYPE_TSG)
			runlists_mask |= fifo_sched_disable_runlist_m(
						f->tsg[id].runlist_id);
		else
			runlists_mask |= fifo_sched_disable_runlist_m(
						f->channel[id].runlist_id);
	}

	if (rc_type == RC_TYPE_MMU_FAULT && mmfault) {
		if (mmfault->faulted_pbdma != FIFO_INVAL_PBDMA_ID)
			pbdma_bitmask = BIT(mmfault->faulted_pbdma);

		for (id = 0; id < f->max_runlists; id++) {

			runlist = &f->runlist_info[id];

			if (runlist->eng_bitmask & act_eng_bitmask)
				runlists_mask |=
				 fifo_sched_disable_runlist_m(id);

			if (runlist->pbdma_bitmask & pbdma_bitmask)
				runlists_mask |=
				 fifo_sched_disable_runlist_m(id);
		}
	}

	if (id_type == ID_TYPE_UNKNOWN) {
		for (id = 0; id < f->max_runlists; id++) {
			if (act_eng_bitmask) {
				/* eng ids are known */
				runlist = &f->runlist_info[id];
				if (runlist->eng_bitmask & act_eng_bitmask)
					runlists_mask |=
					fifo_sched_disable_runlist_m(id);
			} else {
				runlists_mask |=
					fifo_sched_disable_runlist_m(id);
			}
		}
	}
	gk20a_dbg_info("runlists_mask =  %08x", runlists_mask);
	return runlists_mask;
}

static void gv11b_fifo_runlist_event_intr_disable(struct gk20a *g)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fifo_intr_en_0_r());
	reg_val &= fifo_intr_0_runlist_event_pending_f();
	gk20a_writel(g, fifo_intr_en_0_r(), reg_val);
}

static void gv11b_fifo_runlist_event_intr_enable(struct gk20a *g)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fifo_intr_en_0_r());
	reg_val |= fifo_intr_0_runlist_event_pending_f();
	gk20a_writel(g, fifo_intr_en_0_r(), reg_val);
}

static void gv11b_fifo_issue_runlist_preempt(struct gk20a *g,
					 u32 runlists_mask)
{
	u32 reg_val;

	/* issue runlist preempt */
	reg_val = gk20a_readl(g, fifo_runlist_preempt_r());
	reg_val |= runlists_mask;
	gk20a_writel(g, fifo_runlist_preempt_r(), reg_val);
}

static int gv11b_fifo_poll_runlist_preempt_pending(struct gk20a *g,
					 u32 runlists_mask)
{
	struct nvgpu_timeout timeout;
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -EBUSY;

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);
	do {
		if (!((gk20a_readl(g, fifo_runlist_preempt_r())) &
				 runlists_mask)) {
			ret = 0;
			break;
		}

		usleep_range(delay, delay * 2);
		delay = min_t(unsigned long,
				delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired_msg(&timeout,
				 "runlist preempt timeout"));
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
	u32 tsgid;

	if (id_type == ID_TYPE_TSG) {
		runlist_id = f->tsg[id].runlist_id;
		tsgid = id;
	} else {
		runlist_id = f->channel[id].runlist_id;
		tsgid = f->channel[id].tsgid;
	}

	nvgpu_log_info(g, "Check preempt pending for tsgid = %u", tsgid);

	runlist_served_pbdmas = f->runlist_info[runlist_id].pbdma_bitmask;
	runlist_served_engines = f->runlist_info[runlist_id].eng_bitmask;

	for_each_set_bit(pbdma_id, &runlist_served_pbdmas, f->num_pbdma) {

		func_ret = gv11b_fifo_poll_pbdma_chan_status(g, tsgid, pbdma_id,
							 timeout_rc_type);
		if (func_ret != 0) {
			gk20a_dbg_info("preempt timeout pbdma %d", pbdma_id);
			ret |= func_ret;
		}
	}

	f->runlist_info[runlist_id].reset_eng_bitmask = 0;

	for_each_set_bit(act_eng_id, &runlist_served_engines, f->num_engines) {

		func_ret = gv11b_fifo_poll_eng_ctx_status(g, tsgid, act_eng_id,
				&f->runlist_info[runlist_id].reset_eng_bitmask,
				 timeout_rc_type);

		if (func_ret != 0) {
			gk20a_dbg_info("preempt timeout engine %d", act_eng_id);
			ret |= func_ret;
		}
	}

	return ret;
}

int gv11b_fifo_preempt_channel(struct gk20a *g, u32 chid)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 tsgid;

	tsgid = f->channel[chid].tsgid;
	nvgpu_log_info(g, "chid:%d tsgid:%d", chid, tsgid);

	/* Preempt tsg. Channel preempt is NOOP */
	return g->ops.fifo.preempt_tsg(g, tsgid);
}

static int __locked_fifo_preempt_runlists(struct gk20a *g, u32 runlists_mask)
{
	int ret;

	/*
	 * Disable runlist event interrupt as it will get
	 * triggered after runlist preempt finishes
	 */
	gv11b_fifo_runlist_event_intr_disable(g);

	/* issue runlist preempt */
	gv11b_fifo_issue_runlist_preempt(g, runlists_mask);

	/* poll for runlist preempt done */
	ret = gv11b_fifo_poll_runlist_preempt_pending(g, runlists_mask);

	/* Clear outstanding runlist event */
	gk20a_fifo_handle_runlist_event(g);

	/* Enable runlist event interrupt*/
	gv11b_fifo_runlist_event_intr_enable(g);

	return ret;
}

/* TSG enable sequence applicable for Volta and onwards */
int gv11b_fifo_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		g->ops.fifo.enable_channel(ch);
	}
	up_read(&tsg->ch_list_lock);

	return 0;
}

int gv11b_fifo_preempt_tsg(struct gk20a *g, u32 tsgid)
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

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, tsgid, true);

	if (!mutex_ret)
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	nvgpu_mutex_release(&f->runlist_info[runlist_id].mutex);

	return ret;
}


static int gv11b_fifo_preempt_runlists(struct gk20a *g, u32 runlists_mask)
{
	int ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 runlist_id;

	gk20a_dbg_fn("");

	for (runlist_id = 0; runlist_id < g->fifo.max_runlists; runlist_id++) {
		if (runlists_mask & fifo_runlist_preempt_runlist_m(runlist_id))
			nvgpu_mutex_acquire(&g->fifo.
				runlist_info[runlist_id].mutex);
	}

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt_runlists(g, runlists_mask);

	if (!mutex_ret)
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	for (runlist_id = 0; runlist_id < g->fifo.max_runlists; runlist_id++) {
		if (runlists_mask & fifo_runlist_preempt_runlist_m(runlist_id))
			nvgpu_mutex_release(&g->fifo.
				runlist_info[runlist_id].mutex);
	}

	return ret;
}

static int __locked_fifo_preempt_ch_tsg(struct gk20a *g, u32 id,
			 unsigned int id_type, unsigned int timeout_rc_type)
{
	int ret;
	struct fifo_gk20a *f = &g->fifo;

	nvgpu_log_fn(g, "id:%d id_type:%d", id, id_type);

	/* Issue tsg preempt. Channel preempt is noop */
	if (id_type == ID_TYPE_CHANNEL)
		gk20a_fifo_issue_preempt(g, f->channel[id].tsgid, true);
	else
		gk20a_fifo_issue_preempt(g, id, true);

	/* wait for preempt */
	ret = g->ops.fifo.is_preempt_pending(g, id, id_type,
					 timeout_rc_type);

	if (ret && (timeout_rc_type == PREEMPT_TIMEOUT_RC))
		__locked_fifo_preempt_timeout_rc(g, id, id_type);

	return ret;
}


int gv11b_fifo_preempt_ch_tsg(struct gk20a *g, u32 id,
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

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt_ch_tsg(g, id, id_type, timeout_rc_type);

	if (!mutex_ret)
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	nvgpu_mutex_release(&f->runlist_info[runlist_id].mutex);

	return ret;

}

void gv11b_fifo_teardown_ch_tsg(struct gk20a *g, u32 act_eng_bitmask,
			u32 id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault)
{
	bool verbose = false;
	struct tsg_gk20a *tsg = NULL;
	struct channel_gk20a *refch = NULL;
	u32 runlists_mask, runlist_id;
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u32 engine_id, client_type = ~0;

	gk20a_dbg_info("active engine ids bitmask =0x%x", act_eng_bitmask);
	gk20a_dbg_info("hw id     =%d", id);
	gk20a_dbg_info("id_type   =%d", id_type);
	gk20a_dbg_info("rc_type   =%d", rc_type);
	gk20a_dbg_info("mmu_fault =0x%p", mmfault);

	runlists_mask =  gv11b_fifo_get_runlists_mask(g, act_eng_bitmask, id,
					 id_type, rc_type, mmfault);

	gk20a_fifo_set_runlist_state(g, runlists_mask, RUNLIST_DISABLED,
					 !RUNLIST_INFO_MUTEX_LOCKED);

	g->fifo.deferred_reset_pending = false;

	/* Disable power management */
	if (g->support_pmu && g->elpg_enabled) {
		if (nvgpu_pmu_disable_elpg(g))
			nvgpu_err(g, "failed to set disable elpg");
	}
	if (g->ops.clock_gating.slcg_gr_load_gating_prod)
		g->ops.clock_gating.slcg_gr_load_gating_prod(g,
				false);
	if (g->ops.clock_gating.slcg_perf_load_gating_prod)
		g->ops.clock_gating.slcg_perf_load_gating_prod(g,
				false);
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod)
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g,
				false);

	gr_gk20a_init_cg_mode(g, ELCG_MODE, ELCG_RUN);

	if (rc_type == RC_TYPE_MMU_FAULT)
		gk20a_debug_dump(g);

	/* get the channel/TSG */
	if (rc_type == RC_TYPE_MMU_FAULT && mmfault && mmfault->refch) {
		refch = mmfault->refch;
		client_type = mmfault->client_type;
		if (gk20a_is_channel_marked_as_tsg(refch))
			tsg = &g->fifo.tsg[refch->tsgid];
		gv11b_fifo_reset_pbdma_and_eng_faulted(g, refch,
			mmfault->faulted_pbdma,
			mmfault->faulted_engine);
	} else {
		if (id_type == ID_TYPE_TSG)
			tsg = &g->fifo.tsg[id];
		else if (id_type == ID_TYPE_CHANNEL)
			refch = gk20a_channel_get(&g->fifo.channel[id]);
	}

	if (id_type == ID_TYPE_TSG || id_type == ID_TYPE_CHANNEL) {
		g->ops.fifo.preempt_ch_tsg(g, id, id_type,
					 PREEMPT_TIMEOUT_NORC);
	} else {
		gv11b_fifo_preempt_runlists(g, runlists_mask);
	}

	if (tsg) {
		if (!g->fifo.deferred_reset_pending) {
			if (rc_type == RC_TYPE_MMU_FAULT) {
				gk20a_fifo_set_ctx_mmu_error_tsg(g, tsg);
				verbose = gk20a_fifo_error_tsg(g, tsg);
			}
		}
		gk20a_fifo_abort_tsg(g, tsg->tsgid, false);
		if (refch)
			gk20a_channel_put(refch);
	} else if (refch) {
		if (!g->fifo.deferred_reset_pending) {
			if (rc_type == RC_TYPE_MMU_FAULT) {
				gk20a_fifo_set_ctx_mmu_error_ch(g, refch);
				verbose = gk20a_fifo_error_ch(g, refch);
			}
		}
		gk20a_channel_abort(refch, false);
		gk20a_channel_put(refch);
	} else {
		nvgpu_err(g, "id unknown, abort runlist");
		for (runlist_id = 0; runlist_id < g->fifo.max_runlists;
						 runlist_id++) {
			if (runlists_mask & BIT(runlist_id))
				g->ops.fifo.update_runlist(g, runlist_id,
					 FIFO_INVAL_CHANNEL_ID, false, true);
		}
	}

	/* check if engine reset should be deferred */
	for (runlist_id = 0; runlist_id < g->fifo.max_runlists; runlist_id++) {

		runlist = &g->fifo.runlist_info[runlist_id];
		if ((runlists_mask & BIT(runlist_id)) &&
					runlist->reset_eng_bitmask) {

			unsigned long __reset_eng_bitmask =
				 runlist->reset_eng_bitmask;

			for_each_set_bit(engine_id, &__reset_eng_bitmask, 32) {
				if ((refch || tsg) &&
					 gk20a_fifo_should_defer_engine_reset(g,
					engine_id, client_type, false)) {

				g->fifo.deferred_fault_engines |=
							 BIT(engine_id);

				/* handled during channel free */
				g->fifo.deferred_reset_pending = true;
				gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg,
				   "sm debugger attached,"
				   " deferring channel recovery to channel free");
				} else {
					/*
					 * if lock is already taken, a reset is
					 * taking place so no need to repeat
					 */
					if (nvgpu_mutex_tryacquire(
						&g->fifo.gr_reset_mutex)) {

						gk20a_fifo_reset_engine(g,
								 engine_id);

						nvgpu_mutex_release(
						 &g->fifo.gr_reset_mutex);
					}
				}
			}
		}
	}

	if (refch)
		gk20a_ctxsw_trace_channel_reset(g, refch);
	else if (tsg)
		gk20a_ctxsw_trace_tsg_reset(g, tsg);

	gk20a_fifo_set_runlist_state(g, runlists_mask, RUNLIST_ENABLED,
					 !RUNLIST_INFO_MUTEX_LOCKED);

	/* It is safe to enable ELPG again. */
	if (g->support_pmu && g->elpg_enabled)
		nvgpu_pmu_enable_elpg(g);
}

void gv11b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
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
				 fifo_intr_0_pbdma_intr_pending_f() |
				 fifo_intr_0_ctxsw_timeout_pending_f();

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
	nvgpu_log_info(g, "fifo_fb_timeout reg val = 0x%08x", timeout);
	if (!nvgpu_platform_is_silicon(g)) {
		timeout = set_field(timeout, fifo_fb_timeout_period_m(),
					fifo_fb_timeout_period_max_f());
		timeout = set_field(timeout, fifo_fb_timeout_detection_m(),
					fifo_fb_timeout_detection_disabled_f());
		nvgpu_log_info(g, "new fifo_fb_timeout reg val = 0x%08x",
					timeout);
		gk20a_writel(g, fifo_fb_timeout_r(), timeout);
	}

	for (i = 0; i < host_num_pbdma; i++) {
		timeout = gk20a_readl(g, pbdma_timeout_r(i));
		nvgpu_log_info(g, "pbdma_timeout reg val = 0x%08x",
						 timeout);
		if (!nvgpu_platform_is_silicon(g)) {
			timeout = set_field(timeout, pbdma_timeout_period_m(),
					pbdma_timeout_period_max_f());
			nvgpu_log_info(g, "new pbdma_timeout reg val = 0x%08x",
						 timeout);
			gk20a_writel(g, pbdma_timeout_r(i), timeout);
		}
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

	/* clear ctxsw timeout interrupts */
	gk20a_writel(g, fifo_intr_ctxsw_timeout_r(), ~0);

	if (nvgpu_platform_is_silicon(g)) {
		/* enable ctxsw timeout */
		timeout = GRFIFO_TIMEOUT_CHECK_PERIOD_US;
		timeout = scale_ptimer(timeout,
			ptimer_scalingfactor10x(g->ptimer_src_freq));
		timeout |= fifo_eng_ctxsw_timeout_detection_enabled_f();
		gk20a_writel(g, fifo_eng_ctxsw_timeout_r(), timeout);
	} else {
		timeout = gk20a_readl(g, fifo_eng_ctxsw_timeout_r());
		nvgpu_log_info(g, "fifo_eng_ctxsw_timeout reg val = 0x%08x",
						 timeout);
		timeout = set_field(timeout, fifo_eng_ctxsw_timeout_period_m(),
				fifo_eng_ctxsw_timeout_period_max_f());
		timeout = set_field(timeout,
				fifo_eng_ctxsw_timeout_detection_m(),
				fifo_eng_ctxsw_timeout_detection_disabled_f());
		nvgpu_log_info(g, "new fifo_eng_ctxsw_timeout reg val = 0x%08x",
						 timeout);
		gk20a_writel(g, fifo_eng_ctxsw_timeout_r(), timeout);
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

static const char *const gv11b_sched_error_str[] = {
	"xxx-0",
	"xxx-1",
	"xxx-2",
	"xxx-3",
	"xxx-4",
	"engine_reset",
	"rl_ack_timeout",
	"rl_ack_extra",
	"rl_rdat_timeout",
	"rl_rdat_extra",
	"xxx-a",
	"xxx-b",
	"rl_req_timeout",
	"new_runlist",
	"code_config_while_busy",
	"xxx-f",
	"xxx-0x10",
	"xxx-0x11",
	"xxx-0x12",
	"xxx-0x13",
	"xxx-0x14",
	"xxx-0x15",
	"xxx-0x16",
	"xxx-0x17",
	"xxx-0x18",
	"xxx-0x19",
	"xxx-0x1a",
	"xxx-0x1b",
	"xxx-0x1c",
	"xxx-0x1d",
	"xxx-0x1e",
	"xxx-0x1f",
	"bad_tsg",
};

bool gv11b_fifo_handle_sched_error(struct gk20a *g)
{
	u32 sched_error;

	sched_error = gk20a_readl(g, fifo_intr_sched_error_r());

	if (sched_error < ARRAY_SIZE(gv11b_sched_error_str))
		nvgpu_err(g, "fifo sched error :%s",
			gv11b_sched_error_str[sched_error]);
	else
		nvgpu_err(g, "fifo sched error code not supported");

	if (sched_error == SCHED_ERROR_CODE_BAD_TSG ) {
		/* id is unknown, preempt all runlists and do recovery */
		gk20a_fifo_recover(g, 0, 0, false, false, false);
	}

	return false;
}

static u32 gv11b_fifo_ctxsw_timeout_info(struct gk20a *g, u32 active_eng_id)
{
	u32 tsgid = FIFO_INVAL_TSG_ID;
	u32 timeout_info;
	u32 ctx_status, info_status;

	timeout_info = gk20a_readl(g,
			 fifo_intr_ctxsw_timeout_info_r(active_eng_id));

	/*
	 * ctxsw_state and tsgid are snapped at the point of the timeout and
	 * will not change while the corresponding INTR_CTXSW_TIMEOUT_ENGINE bit
	 * is PENDING.
	 */
	ctx_status = fifo_intr_ctxsw_timeout_info_ctxsw_state_v(timeout_info);
	if (ctx_status ==
		fifo_intr_ctxsw_timeout_info_ctxsw_state_load_v()) {

		tsgid = fifo_intr_ctxsw_timeout_info_next_tsgid_v(timeout_info);

	} else if (ctx_status ==
		       fifo_intr_ctxsw_timeout_info_ctxsw_state_switch_v() ||
			ctx_status ==
			fifo_intr_ctxsw_timeout_info_ctxsw_state_save_v()) {

		tsgid = fifo_intr_ctxsw_timeout_info_prev_tsgid_v(timeout_info);
	}
	gk20a_dbg_info("ctxsw timeout info: tsgid = %d", tsgid);

	/*
	 * STATUS indicates whether the context request ack was eventually
	 * received and whether a subsequent request timed out.  This field is
	 * updated live while the corresponding INTR_CTXSW_TIMEOUT_ENGINE bit
	 * is PENDING. STATUS starts in AWAITING_ACK, and progresses to
	 * ACK_RECEIVED and finally ends with DROPPED_TIMEOUT.
	 *
	 * AWAITING_ACK - context request ack still not returned from engine.
	 * ENG_WAS_RESET - The engine was reset via a PRI write to NV_PMC_ENABLE
	 * or NV_PMC_ELPG_ENABLE prior to receiving the ack.  Host will not
	 * expect ctx ack to return, but if it is already in flight, STATUS will
	 * transition shortly to ACK_RECEIVED unless the interrupt is cleared
	 * first.  Once the engine is reset, additional context switches can
	 * occur; if one times out, STATUS will transition to DROPPED_TIMEOUT
	 * if the interrupt isn't cleared first.
	 * ACK_RECEIVED - The ack for the timed-out context request was
	 * received between the point of the timeout and this register being
	 * read.  Note this STATUS can be reported during the load stage of the
	 * same context switch that timed out if the timeout occurred during the
	 * save half of a context switch.  Additional context requests may have
	 * completed or may be outstanding, but no further context timeout has
	 * occurred.  This simplifies checking for spurious context switch
	 * timeouts.
	 * DROPPED_TIMEOUT - The originally timed-out context request acked,
	 * but a subsequent context request then timed out.
	 * Information about the subsequent timeout is not stored; in fact, that
	 * context request may also have already been acked by the time SW
	 * SW reads this register.  If not, there is a chance SW can get the
	 * dropped information by clearing the corresponding
	 * INTR_CTXSW_TIMEOUT_ENGINE bit and waiting for the timeout to occur
	 * again. Note, however, that if the engine does time out again,
	 * it may not be from the  original request that caused the
	 * DROPPED_TIMEOUT state, as that request may
	 * be acked in the interim.
	 */
	info_status = fifo_intr_ctxsw_timeout_info_status_v(timeout_info);
	if (info_status ==
		 fifo_intr_ctxsw_timeout_info_status_awaiting_ack_v()) {

		gk20a_dbg_info("ctxsw timeout info : awaiting ack");

	} else if (info_status ==
		 fifo_intr_ctxsw_timeout_info_status_eng_was_reset_v()) {

		gk20a_dbg_info("ctxsw timeout info : eng was reset");

	} else if (info_status ==
		 fifo_intr_ctxsw_timeout_info_status_ack_received_v()) {

		gk20a_dbg_info("ctxsw timeout info : ack received");
		/* no need to recover */
		tsgid = FIFO_INVAL_TSG_ID;

	} else if (info_status ==
		fifo_intr_ctxsw_timeout_info_status_dropped_timeout_v()) {

		gk20a_dbg_info("ctxsw timeout info : dropped timeout");
		/* no need to recover */
		tsgid = FIFO_INVAL_TSG_ID;

	} else {
		gk20a_dbg_info("ctxsw timeout info status = %u", info_status);
	}

	return tsgid;
}

bool gv11b_fifo_handle_ctxsw_timeout(struct gk20a *g, u32 fifo_intr)
{
	bool ret = false;
	u32 tsgid = FIFO_INVAL_TSG_ID;
	u32 engine_id, active_eng_id;
	u32 timeout_val, ctxsw_timeout_engines;


	if (!(fifo_intr & fifo_intr_0_ctxsw_timeout_pending_f()))
		return ret;

	/* get ctxsw timedout engines */
	ctxsw_timeout_engines = gk20a_readl(g, fifo_intr_ctxsw_timeout_r());
	if (ctxsw_timeout_engines == 0) {
		nvgpu_err(g, "no eng ctxsw timeout pending");
		return ret;
	}

	timeout_val = gk20a_readl(g, fifo_eng_ctxsw_timeout_r());
	timeout_val = fifo_eng_ctxsw_timeout_period_v(timeout_val);

	gk20a_dbg_info("eng ctxsw timeout period = 0x%x", timeout_val);

	for (engine_id = 0; engine_id < g->fifo.num_engines; engine_id++) {
		active_eng_id = g->fifo.active_engines_list[engine_id];

		if (ctxsw_timeout_engines &
			fifo_intr_ctxsw_timeout_engine_pending_f(
				active_eng_id)) {

			struct fifo_gk20a *f = &g->fifo;
			u32 ms = 0;
			bool verbose = false;

			tsgid = gv11b_fifo_ctxsw_timeout_info(g, active_eng_id);

			if (tsgid == FIFO_INVAL_TSG_ID)
				continue;

			if (gk20a_fifo_check_tsg_ctxsw_timeout(
				&f->tsg[tsgid], &verbose, &ms)) {
				ret = true;
				nvgpu_err(g,
				 "ctxsw timeout error:"
				"active engine id =%u, %s=%d, ms=%u",
				active_eng_id, "tsg", tsgid, ms);

				/* Cancel all channels' timeout */
				gk20a_channel_timeout_restart_all_channels(g);
				gk20a_fifo_recover(g, BIT(active_eng_id), tsgid,
						true, true, verbose);
			} else {
				gk20a_dbg_info(
					"fifo is waiting for ctx switch: "
					"for %d ms, %s=%d", ms, "tsg", tsgid);
			}
		}
	}
	/* clear interrupt */
	gk20a_writel(g, fifo_intr_ctxsw_timeout_r(), ctxsw_timeout_engines);
	return ret;
}

unsigned int gv11b_fifo_handle_pbdma_intr_0(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_0,
			u32 *handled, u32 *error_notifier)
{
	unsigned int rc_type = RC_TYPE_NO_RC;

	rc_type = gk20a_fifo_handle_pbdma_intr_0(g, pbdma_id,
			 pbdma_intr_0, handled, error_notifier);

	if (pbdma_intr_0 & pbdma_intr_0_clear_faulted_error_pending_f()) {
		gk20a_dbg(gpu_dbg_intr, "clear faulted error on pbdma id %d",
				 pbdma_id);
		gk20a_fifo_reset_pbdma_method(g, pbdma_id, 0);
		*handled |= pbdma_intr_0_clear_faulted_error_pending_f();
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	if (pbdma_intr_0 & pbdma_intr_0_eng_reset_pending_f()) {
		gk20a_dbg(gpu_dbg_intr, "eng reset intr on pbdma id %d",
				 pbdma_id);
		*handled |= pbdma_intr_0_eng_reset_pending_f();
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	return rc_type;
}

/*
 * Pbdma which encountered the ctxnotvalid interrupt will stall and
 * prevent the channel which was loaded at the time the interrupt fired
 * from being swapped out until the interrupt is cleared.
 * CTXNOTVALID pbdma interrupt indicates error conditions related
 * to the *_CTX_VALID fields for a channel.  The following
 * conditions trigger the interrupt:
 * * CTX_VALID bit for the targeted engine is FALSE
 * * At channel start/resume, all preemptible eng have CTX_VALID FALSE but:
 *       - CTX_RELOAD is set in CCSR_CHANNEL_STATUS,
 *       - PBDMA_TARGET_SHOULD_SEND_HOST_TSG_EVENT is TRUE, or
 *       - PBDMA_TARGET_NEEDS_HOST_TSG_EVENT is TRUE
 * The field is left NOT_PENDING and the interrupt is not raised if the PBDMA is
 * currently halted.  This allows SW to unblock the PBDMA and recover.
 * SW may read METHOD0, CHANNEL_STATUS and TARGET to determine whether the
 * interrupt was due to an engine method, CTX_RELOAD, SHOULD_SEND_HOST_TSG_EVENT
 * or NEEDS_HOST_TSG_EVENT.  If METHOD0 VALID is TRUE, lazy context creation
 * can be used or the TSG may be destroyed.
 * If METHOD0 VALID is FALSE, the error is likely a bug in SW, and the TSG
 * will have to be destroyed.
 */

unsigned int gv11b_fifo_handle_pbdma_intr_1(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_1,
			u32 *handled, u32 *error_notifier)
{
	unsigned int rc_type = RC_TYPE_PBDMA_FAULT;
	u32 pbdma_intr_1_current = gk20a_readl(g, pbdma_intr_1_r(pbdma_id));

	/* minimize race with the gpu clearing the pending interrupt */
	if (!(pbdma_intr_1_current &
			pbdma_intr_1_ctxnotvalid_pending_f()))
		pbdma_intr_1 &= ~pbdma_intr_1_ctxnotvalid_pending_f();

	if (pbdma_intr_1 == 0)
		return RC_TYPE_NO_RC;

	if (pbdma_intr_1 & pbdma_intr_1_ctxnotvalid_pending_f()) {
		gk20a_dbg(gpu_dbg_intr, "ctxnotvalid intr on pbdma id %d",
				 pbdma_id);
		nvgpu_err(g, "pbdma_intr_1(%d)= 0x%08x ",
				pbdma_id, pbdma_intr_1);
		*handled |= pbdma_intr_1_ctxnotvalid_pending_f();
	} else{
		/*
		 * rest of the interrupts in _intr_1 are "host copy engine"
		 * related, which is not supported. For now just make them
		 * channel fatal.
		 */
		nvgpu_err(g, "hce err: pbdma_intr_1(%d):0x%08x",
			pbdma_id, pbdma_intr_1);
		*handled |= pbdma_intr_1;
	}

	return rc_type;
}

static void gv11b_fifo_init_ramfc_eng_method_buffer(struct gk20a *g,
			struct channel_gk20a *ch, struct nvgpu_mem *mem)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_mem *method_buffer_per_runque;

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg == NULL) {
		nvgpu_err(g, "channel is not part of tsg");
		return;
	}
	if (tsg->eng_method_buffers == NULL) {
		nvgpu_log_info(g, "eng method buffer NULL");
		return;
	}
	if (tsg->runlist_id == gk20a_fifo_get_fast_ce_runlist_id(g))
		method_buffer_per_runque =
			&tsg->eng_method_buffers[ASYNC_CE_RUNQUE];
	else
		method_buffer_per_runque =
			&tsg->eng_method_buffers[GR_RUNQUE];

	nvgpu_mem_wr32(g, mem, ram_in_eng_method_buffer_addr_lo_w(),
			u64_lo32(method_buffer_per_runque->gpu_va));
	nvgpu_mem_wr32(g, mem, ram_in_eng_method_buffer_addr_hi_w(),
			u64_hi32(method_buffer_per_runque->gpu_va));

	nvgpu_log_info(g, "init ramfc with method buffer");
}

unsigned int gv11b_fifo_get_eng_method_buffer_size(struct gk20a *g)
{
	unsigned int buffer_size;

	buffer_size =  ((9 + 1 + 3) * g->ops.ce2.get_num_pce(g)) + 2;
	buffer_size = (27 * 5 * buffer_size);
	buffer_size = roundup(buffer_size, PAGE_SIZE);
	nvgpu_log_info(g, "method buffer size in bytes %d", buffer_size);

	return buffer_size;
}

void gv11b_fifo_init_eng_method_buffers(struct gk20a *g,
					struct tsg_gk20a *tsg)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;
	int err = 0;
	int i;
	unsigned int runque, method_buffer_size;
	unsigned int num_pbdma = g->fifo.num_pbdma;

	if (tsg->eng_method_buffers != NULL)
		return;

	method_buffer_size = gv11b_fifo_get_eng_method_buffer_size(g);
	if (method_buffer_size == 0) {
		nvgpu_info(g, "ce will hit MTHD_BUFFER_FAULT");
		return;
	}

	tsg->eng_method_buffers = nvgpu_kzalloc(g,
					num_pbdma * sizeof(struct nvgpu_mem));

	for (runque = 0; runque < num_pbdma; runque++) {
		err = nvgpu_dma_alloc_map_sys(vm, method_buffer_size,
					&tsg->eng_method_buffers[runque]);
		if (err)
			break;
	}
	if (err) {
		for (i = (runque - 1); i >= 0; i--)
			nvgpu_dma_unmap_free(vm,
				 &tsg->eng_method_buffers[i]);

		nvgpu_kfree(g, tsg->eng_method_buffers);
		tsg->eng_method_buffers = NULL;
		nvgpu_err(g, "could not alloc eng method buffers");
		return;
	}
	nvgpu_log_info(g, "eng method buffers allocated");

}

void gv11b_fifo_deinit_eng_method_buffers(struct gk20a *g,
					struct tsg_gk20a *tsg)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;
	unsigned int runque;

	if (tsg->eng_method_buffers == NULL)
		return;

	for (runque = 0; runque < g->fifo.num_pbdma; runque++)
		nvgpu_dma_unmap_free(vm, &tsg->eng_method_buffers[runque]);

	nvgpu_kfree(g, tsg->eng_method_buffers);
	tsg->eng_method_buffers = NULL;

	nvgpu_log_info(g, "eng method buffers de-allocated");
}

#ifdef CONFIG_TEGRA_GK20A_NVHOST
int gv11b_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
			u32 syncpt_id, struct nvgpu_mem *syncpt_buf)
{
	struct page **pages;
	u32 nr_pages;
	u32 i;
	int err = 0;
	struct gk20a *g = c->g;

	/*
	 * Add rw mapping for entire syncpt shim for current channel vm
	 * TODO : This needs to replaced with a new mecahnism where
	 * only current syncpoint range will be rw and other sync
	 * points range is read only for current channel vm. Also share
	 * these mapping accross channels if they share same vm
	*/
	nr_pages = DIV_ROUND_UP(g->syncpt_unit_size, PAGE_SIZE);
	pages = nvgpu_kzalloc(g,  sizeof(struct page *) * nr_pages);
	for (i = 0; i < nr_pages; i++)
		pages[i] = phys_to_page(g->syncpt_unit_base +
				PAGE_SIZE * i);
	__nvgpu_mem_create_from_pages(g, syncpt_buf, pages, nr_pages);
	nvgpu_kfree(g, pages);
	syncpt_buf->gpu_va = nvgpu_gmmu_map(c->vm, syncpt_buf,
			g->syncpt_unit_size, 0, gk20a_mem_flag_none,
			false, APERTURE_SYSMEM);

	if (!syncpt_buf->gpu_va) {
		nvgpu_err(c->g, "failed to map syncpt buffer");
		nvgpu_dma_free(c->g, syncpt_buf);
		err = -ENOMEM;
	}
	return err;
}

void gv11b_fifo_free_syncpt_buf(struct channel_gk20a *c,
					struct nvgpu_mem *syncpt_buf)
{
	nvgpu_gmmu_unmap(c->vm, syncpt_buf, syncpt_buf->gpu_va);
	nvgpu_dma_free(c->g, syncpt_buf);
}

void gv11b_fifo_add_syncpt_wait_cmd(struct gk20a *g,
		struct priv_cmd_entry *cmd, u32 off,
		u32 id, u32 thresh, u64 gpu_va_base)
{
	u64 gpu_va = gpu_va_base +
		nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(id);

	gk20a_dbg_fn("");

	off = cmd->off + off;

	/* semaphore_a */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010004);
	nvgpu_mem_wr32(g, cmd->mem, off++,
			(gpu_va >> 32) & 0xff);
	/* semaphore_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010005);
	/* offset */
	nvgpu_mem_wr32(g, cmd->mem, off++, gpu_va & 0xffffffff);

	/* semaphore_c */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010006);
	/* payload */
	nvgpu_mem_wr32(g, cmd->mem, off++, thresh);
	/* semaphore_d */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010007);
	/* operation: acq_geq, switch_en */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x4 | (0x1 << 12));
}

u32 gv11b_fifo_get_syncpt_wait_cmd_size(void)
{
	return 8;
}

void gv11b_fifo_add_syncpt_incr_cmd(struct gk20a *g,
		bool wfi_cmd, struct priv_cmd_entry *cmd,
		u32 id, u64 gpu_va_base)
{
	u32 off = cmd->off;
	u64 gpu_va = gpu_va_base +
		nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(id);

	gk20a_dbg_fn("");

	/* semaphore_a */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010004);
	nvgpu_mem_wr32(g, cmd->mem, off++,
			(gpu_va >> 32) & 0xff);
	/* semaphore_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010005);
	/* offset */
	nvgpu_mem_wr32(g, cmd->mem, off++, gpu_va & 0xffffffff);

	/* semaphore_c */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010006);
	/* payload */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x0);
	/* semaphore_d */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010007);

	/* operation: release, wfi */
	nvgpu_mem_wr32(g, cmd->mem, off++,
		0x2 | ((wfi_cmd ? 0x0 : 0x1) << 20));
	/* ignored */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0);
}

u32 gv11b_fifo_get_syncpt_incr_cmd_size(bool wfi_cmd)
{
	return 9;
}
#endif /* CONFIG_TEGRA_GK20A_NVHOST */

int gv11b_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;

	f->t19x.usermode_regs = g->regs + usermode_cfg0_r();
	f->t19x.max_subctx_count =
		gr_pri_fe_chip_def_info_max_veid_count_init_v();
	return 0;
}

static u32 gv11b_mmu_fault_id_to_gr_veid(struct gk20a *g, u32 gr_eng_fault_id,
				 u32 mmu_fault_id)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 num_subctx;
	u32 veid = FIFO_INVAL_VEID;

	num_subctx = f->t19x.max_subctx_count;

	if (mmu_fault_id >= gr_eng_fault_id &&
			mmu_fault_id < (gr_eng_fault_id + num_subctx))
		veid = mmu_fault_id - gr_eng_fault_id;

	return veid;
}

static u32 gv11b_mmu_fault_id_to_eng_id_and_veid(struct gk20a *g,
			 u32 mmu_fault_id, u32 *veid)
{
	u32 engine_id;
	u32 active_engine_id;
	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_gk20a *f = &g->fifo;


	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];
		engine_info = &g->fifo.engine_info[active_engine_id];

		if (active_engine_id == ENGINE_GR_GK20A) {
			/* get faulted subctx id */
			*veid = gv11b_mmu_fault_id_to_gr_veid(g,
					engine_info->fault_id, mmu_fault_id);
			if (*veid != FIFO_INVAL_VEID)
				break;
		} else {
			if (engine_info->fault_id == mmu_fault_id)
				break;
		}

		active_engine_id = FIFO_INVAL_ENGINE_ID;
	}
	return active_engine_id;
}

static u32 gv11b_mmu_fault_id_to_pbdma_id(struct gk20a *g, u32 mmu_fault_id)
{
	u32 num_pbdma, reg_val, fault_id_pbdma0;

	reg_val = gk20a_readl(g, fifo_cfg0_r());
	num_pbdma = fifo_cfg0_num_pbdma_v(reg_val);
	fault_id_pbdma0 = fifo_cfg0_pbdma_fault_id_v(reg_val);

	if (mmu_fault_id >= fault_id_pbdma0 &&
		mmu_fault_id <= fault_id_pbdma0 + num_pbdma - 1)
		return mmu_fault_id - fault_id_pbdma0;

	return FIFO_INVAL_PBDMA_ID;
}

void gv11b_mmu_fault_id_to_eng_pbdma_id_and_veid(struct gk20a *g,
	u32 mmu_fault_id, u32 *active_engine_id, u32 *veid, u32 *pbdma_id)
{
	*active_engine_id = gv11b_mmu_fault_id_to_eng_id_and_veid(g,
				 mmu_fault_id, veid);

	if (*active_engine_id == FIFO_INVAL_ENGINE_ID)
		*pbdma_id = gv11b_mmu_fault_id_to_pbdma_id(g, mmu_fault_id);
	else
		*pbdma_id = FIFO_INVAL_PBDMA_ID;
}

static bool gk20a_fifo_channel_status_is_eng_faulted(struct gk20a *g, u32 chid)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));

	return ccsr_channel_eng_faulted_v(channel) ==
		ccsr_channel_eng_faulted_true_v();
}

void gv11b_fifo_tsg_verify_status_faulted(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	struct tsg_gk20a *tsg = &g->fifo.tsg[ch->tsgid];

	/*
	 * If channel has FAULTED set, clear the CE method buffer
	 * if saved out channel is same as faulted channel
	 */
	if (!gk20a_fifo_channel_status_is_eng_faulted(g, ch->chid))
		return;

	if (tsg->eng_method_buffers == NULL)
		return;

	/*
	 * CE method buffer format :
	 * DWord0 = method count
	 * DWord1 = channel id
	 *
	 * It is sufficient to write 0 to method count to invalidate
	 */
	if ((u32)ch->chid ==
	    nvgpu_mem_rd32(g, &tsg->eng_method_buffers[ASYNC_CE_RUNQUE], 1))
		nvgpu_mem_wr32(g, &tsg->eng_method_buffers[ASYNC_CE_RUNQUE], 0, 0);
}
