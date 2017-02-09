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

#include "gk20a/gk20a.h"

#include "gp10b/fifo_gp10b.h"

#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ccsr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>

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
		u64 gpfifo_base, u32 gpfifo_entries, u32 flags)
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
		channel_gk20a_pbdma_acquire_val(c));

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

		gk20a_channel_setup_ramfc_for_privileged_channel(c);
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

	channel_gk20a_unbind(ch);
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
}
