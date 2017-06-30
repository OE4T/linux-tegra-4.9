/*
 * GP10B fifo
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

#include <nvgpu/dma.h>
#include <nvgpu/bug.h>
#include <nvgpu/log2.h>

#include "fifo_gp10b.h"

#include "gk20a/gk20a.h"
#include "gm20b/fifo_gm20b.h"

#include <nvgpu/hw/gp10b/hw_pbdma_gp10b.h>
#include <nvgpu/hw/gp10b/hw_ccsr_gp10b.h>
#include <nvgpu/hw/gp10b/hw_fifo_gp10b.h>
#include <nvgpu/hw/gp10b/hw_ram_gp10b.h>
#include <nvgpu/hw/gp10b/hw_top_gp10b.h>

static void gp10b_set_pdb_fault_replay_flags(struct gk20a *g,
				struct nvgpu_mem *mem)
{
	u32 val;

	gk20a_dbg_fn("");

	val = nvgpu_mem_rd32(g, mem,
			ram_in_page_dir_base_fault_replay_tex_w());
	val &= ~ram_in_page_dir_base_fault_replay_tex_m();
	val |= ram_in_page_dir_base_fault_replay_tex_true_f();
	nvgpu_mem_wr32(g, mem,
		ram_in_page_dir_base_fault_replay_tex_w(), val);

	val = nvgpu_mem_rd32(g, mem,
			ram_in_page_dir_base_fault_replay_gcc_w());
	val &= ~ram_in_page_dir_base_fault_replay_gcc_m();
	val |= ram_in_page_dir_base_fault_replay_gcc_true_f();
	nvgpu_mem_wr32(g, mem,
		ram_in_page_dir_base_fault_replay_gcc_w(), val);

	gk20a_dbg_fn("done");
}

int channel_gp10b_commit_userd(struct channel_gk20a *c)
{
	u32 addr_lo;
	u32 addr_hi;
	struct gk20a *g = c->g;

	gk20a_dbg_fn("");

	addr_lo = u64_lo32(c->userd_iova >> ram_userd_base_shift_v());
	addr_hi = u64_hi32(c->userd_iova);

	gk20a_dbg_info("channel %d : set ramfc userd 0x%16llx",
		c->chid, (u64)c->userd_iova);

	nvgpu_mem_wr32(g, &c->inst_block,
		       ram_in_ramfc_w() + ram_fc_userd_w(),
		       nvgpu_aperture_mask(g, &g->fifo.userd,
			pbdma_userd_target_sys_mem_ncoh_f(),
			pbdma_userd_target_vid_mem_f()) |
		       pbdma_userd_addr_f(addr_lo));

	nvgpu_mem_wr32(g, &c->inst_block,
		       ram_in_ramfc_w() + ram_fc_userd_hi_w(),
		       pbdma_userd_hi_addr_f(addr_hi));

	return 0;
}

int channel_gp10b_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries,
			unsigned long acquire_timeout, u32 flags)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *mem = &c->inst_block;

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

	nvgpu_mem_wr32(g, mem, ram_fc_formats_w(),
		pbdma_formats_gp_fermi0_f() |
		pbdma_formats_pb_fermi1_f() |
		pbdma_formats_mp_fermi0_f());

	nvgpu_mem_wr32(g, mem, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	nvgpu_mem_wr32(g, mem, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(1) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	nvgpu_mem_wr32(g, mem, ram_fc_target_w(), pbdma_target_engine_sw_f());

	nvgpu_mem_wr32(g, mem, ram_fc_acquire_w(),
		g->ops.fifo.pbdma_acquire_val(acquire_timeout));

	nvgpu_mem_wr32(g, mem, ram_fc_runlist_timeslice_w(),
		pbdma_runlist_timeslice_timeout_128_f() |
		pbdma_runlist_timeslice_timescale_3_f() |
		pbdma_runlist_timeslice_enable_true_f());

	if ( flags & NVGPU_ALLOC_GPFIFO_FLAGS_REPLAYABLE_FAULTS_ENABLE)
		gp10b_set_pdb_fault_replay_flags(c->g, mem);


	nvgpu_mem_wr32(g, mem, ram_fc_chid_w(), ram_fc_chid_id_f(c->chid));

	if (c->is_privileged_channel) {
		/* Set privilege level for channel */
		nvgpu_mem_wr32(g, mem, ram_fc_config_w(),
			pbdma_config_auth_level_privileged_f());

		gk20a_fifo_setup_ramfc_for_privileged_channel(c);
	}

	return channel_gp10b_commit_userd(c);
}

u32 gp10b_fifo_get_pbdma_signature(struct gk20a *g)
{
	return g->gpu_characteristics.gpfifo_class
		| pbdma_signature_sw_zero_f();
}

int gp10b_fifo_resetup_ramfc(struct channel_gk20a *c)
{
	u32 new_syncpt = 0, old_syncpt;
	u32 v;

	gk20a_dbg_fn("");

	v = nvgpu_mem_rd32(c->g, &c->inst_block,
			ram_fc_allowed_syncpoints_w());
	old_syncpt = pbdma_allowed_syncpoints_0_index_v(v);
	if (c->sync)
		new_syncpt = c->sync->syncpt_id(c->sync);

	if (new_syncpt && new_syncpt != old_syncpt) {
		/* disable channel */
		gk20a_disable_channel_tsg(c->g, c);

		/* preempt the channel */
		WARN_ON(gk20a_fifo_preempt(c->g, c));

		v = pbdma_allowed_syncpoints_0_valid_f(1);

		gk20a_dbg_info("Channel %d, syncpt id %d\n",
				c->chid, new_syncpt);

		v |= pbdma_allowed_syncpoints_0_index_f(new_syncpt);

		nvgpu_mem_wr32(c->g, &c->inst_block,
				ram_fc_allowed_syncpoints_w(), v);
	}

	/* enable channel */
	gk20a_enable_channel_tsg(c->g, c);

	gk20a_dbg_fn("done");

	return 0;
}

int gp10b_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
					u32 *inst_id)
{
	int ret = ENGINE_INVAL_GK20A;

	gk20a_dbg_info("engine type %d", engine_type);
	if (engine_type == top_device_info_type_enum_graphics_v())
		ret = ENGINE_GR_GK20A;
	else if (engine_type == top_device_info_type_enum_lce_v()) {
		/* Default assumptions - all the CE engine have separate runlist */
		ret = ENGINE_ASYNC_CE_GK20A;
	}

	return ret;
}

void gp10b_device_info_data_parse(struct gk20a *g, u32 table_entry,
				u32 *inst_id, u32 *pri_base, u32 *fault_id)
{
	if (top_device_info_data_type_v(table_entry) ==
	    top_device_info_data_type_enum2_v()) {
		if (inst_id)
			*inst_id = top_device_info_data_inst_id_v(table_entry);
		if (pri_base) {
			*pri_base =
			    (top_device_info_data_pri_base_v(table_entry)
			    << top_device_info_data_pri_base_align_v());
			gk20a_dbg_info("device info: pri_base: %d", *pri_base);
		}
		if (fault_id && (top_device_info_data_fault_id_v(table_entry) ==
		    top_device_info_data_fault_id_valid_v())) {
			*fault_id =
				 g->ops.fifo.device_info_fault_id(table_entry);
			gk20a_dbg_info("device info: fault_id: %d", *fault_id);
		}
	} else
		nvgpu_err(g, "unknown device_info_data %d",
			top_device_info_data_type_v(table_entry));
}

void gp10b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
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
		pbdma_intr_0_syncpoint_illegal_pending_f() |
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}

void gp10b_fifo_get_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
	struct mmu_fault_info *mmfault)
{
	u32 fault_info;
	u32 addr_lo, addr_hi;

	gk20a_dbg_fn("mmu_fault_id %d", mmu_fault_id);

	memset(mmfault, 0, sizeof(*mmfault));

	fault_info = gk20a_readl(g,
		fifo_intr_mmu_fault_info_r(mmu_fault_id));
	mmfault->fault_type =
		fifo_intr_mmu_fault_info_type_v(fault_info);
	mmfault->access_type =
		fifo_intr_mmu_fault_info_access_type_v(fault_info);
	mmfault->client_type =
		fifo_intr_mmu_fault_info_client_type_v(fault_info);
	mmfault->client_id =
		fifo_intr_mmu_fault_info_client_v(fault_info);

	addr_lo = gk20a_readl(g, fifo_intr_mmu_fault_lo_r(mmu_fault_id));
	addr_hi = gk20a_readl(g, fifo_intr_mmu_fault_hi_r(mmu_fault_id));
	mmfault->fault_addr = hi32_lo32_to_u64(addr_hi, addr_lo);
	/* note:ignoring aperture */
	mmfault->inst_ptr = fifo_intr_mmu_fault_inst_ptr_v(
		 gk20a_readl(g, fifo_intr_mmu_fault_inst_r(mmu_fault_id)));
	/* note: inst_ptr is a 40b phys addr.  */
	mmfault->inst_ptr <<= fifo_intr_mmu_fault_inst_ptr_align_shift_v();
}
