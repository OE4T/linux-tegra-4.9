/*
 * GP10B fifo
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "gm20b/fifo_gm20b.h"
#include "hw_pbdma_gp10b.h"
#include "fifo_gp10b.h"
#include "hw_ccsr_gp10b.h"
#include "hw_fifo_gp10b.h"
#include "hw_ram_gp10b.h"

static void gp10b_set_pdb_fault_replay_flags(struct gk20a *g,
				void *inst_ptr)
{
	u32 val;

	gk20a_dbg_fn("");

	val = gk20a_mem_rd32(inst_ptr,
			ram_in_page_dir_base_fault_replay_tex_w());
	val &= ~ram_in_page_dir_base_fault_replay_tex_m();
	val |= ram_in_page_dir_base_fault_replay_tex_true_f();
	gk20a_mem_wr32(inst_ptr,
		ram_in_page_dir_base_fault_replay_tex_w(), val);

	val = gk20a_mem_rd32(inst_ptr,
			ram_in_page_dir_base_fault_replay_gcc_w());
	val &= ~ram_in_page_dir_base_fault_replay_gcc_m();
	val |= ram_in_page_dir_base_fault_replay_gcc_true_f();
	gk20a_mem_wr32(inst_ptr,
		ram_in_page_dir_base_fault_replay_gcc_w(), val);

	gk20a_dbg_fn("done");
}

static int channel_gp10b_commit_userd(struct channel_gk20a *c)
{
	u32 addr_lo;
	u32 addr_hi;
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpuva;
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

static int channel_gp10b_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries)
{
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpuva;
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
		 pbdma_signature_hw_valid_f() | pbdma_signature_sw_zero_f());

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
		pbdma_runlist_timeslice_timeout_128_f() |
		pbdma_runlist_timeslice_timescale_3_f() |
		pbdma_runlist_timeslice_enable_true_f());

	gp10b_set_pdb_fault_replay_flags(c->g, inst_ptr);


	gk20a_mem_wr32(inst_ptr, ram_fc_chid_w(), ram_fc_chid_id_f(c->hw_chid));

	return channel_gp10b_commit_userd(c);
}

static u32 gp10b_fifo_get_pbdma_signature(struct gk20a *g)
{
	return g->gpu_characteristics.gpfifo_class 
		| pbdma_signature_sw_zero_f();
}

void gp10b_init_fifo(struct gpu_ops *gops)
{
	gm20b_init_fifo(gops);
	gops->fifo.setup_ramfc = channel_gp10b_setup_ramfc;
	gops->fifo.get_pbdma_signature = gp10b_fifo_get_pbdma_signature;

}
