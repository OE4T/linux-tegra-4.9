/*
 * GM20B Fifo
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

#include "gk20a/gk20a.h"
#include "gk20a/fifo_gk20a.h"

#include "fifo_gm20b.h"

#include <nvgpu/timers.h>
#include <nvgpu/log.h>
#include <nvgpu/atomic.h>

#include <nvgpu/hw/gm20b/hw_ccsr_gm20b.h>
#include <nvgpu/hw/gm20b/hw_ram_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fifo_gm20b.h>
#include <nvgpu/hw/gm20b/hw_top_gm20b.h>
#include <nvgpu/hw/gm20b/hw_pbdma_gm20b.h>

void channel_gm20b_bind(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;

	u32 inst_ptr = gk20a_mm_inst_block_addr(g, &c->inst_block)
		>> ram_in_base_shift_v();

	gk20a_dbg_info("bind channel %d inst ptr 0x%08x",
		c->chid, inst_ptr);


	gk20a_writel(g, ccsr_channel_inst_r(c->chid),
		ccsr_channel_inst_ptr_f(inst_ptr) |
		nvgpu_aperture_mask(g, &c->inst_block,
		 ccsr_channel_inst_target_sys_mem_ncoh_f(),
		 ccsr_channel_inst_target_vid_mem_f()) |
		ccsr_channel_inst_bind_true_f());

	gk20a_writel(g, ccsr_channel_r(c->chid),
		(gk20a_readl(g, ccsr_channel_r(c->chid)) &
		 ~ccsr_channel_enable_set_f(~0)) |
		 ccsr_channel_enable_set_true_f());
	wmb();
	nvgpu_atomic_set(&c->bound, true);
}

static inline u32 gm20b_engine_id_to_mmu_id(struct gk20a *g, u32 engine_id)
{
	u32 fault_id = ~0;
	struct fifo_engine_info_gk20a *engine_info;

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		fault_id = engine_info->fault_id;
	} else {
		nvgpu_err(g, "engine_id is not in active list/invalid %d", engine_id);
	}
	return fault_id;
}

void gm20b_fifo_trigger_mmu_fault(struct gk20a *g,
		unsigned long engine_ids)
{
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	unsigned long engine_id;
	int ret = -EBUSY;
	struct nvgpu_timeout timeout;

	/* trigger faults for all bad engines */
	for_each_set_bit(engine_id, &engine_ids, 32) {
		if (!gk20a_fifo_is_valid_engine_id(g, engine_id)) {
			nvgpu_err(g, "faulting unknown engine %ld", engine_id);
		} else {
			u32 mmu_id = gm20b_engine_id_to_mmu_id(g,
								engine_id);
			if (mmu_id != (u32)~0)
				gk20a_writel(g, fifo_trigger_mmu_fault_r(mmu_id),
					     fifo_trigger_mmu_fault_enable_f(1));
		}
	}

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	/* Wait for MMU fault to trigger */
	do {
		if (gk20a_readl(g, fifo_intr_0_r()) &
				fifo_intr_0_mmu_fault_pending_f()) {
			ret = 0;
			break;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	if (ret)
		nvgpu_err(g, "mmu fault timeout");

	/* release mmu fault trigger */
	for_each_set_bit(engine_id, &engine_ids, 32)
		gk20a_writel(g, fifo_trigger_mmu_fault_r(engine_id), 0);
}

u32 gm20b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

void gm20b_device_info_data_parse(struct gk20a *g,
						u32 table_entry, u32 *inst_id,
						u32 *pri_base, u32 *fault_id)
{
	if (top_device_info_data_type_v(table_entry) ==
	    top_device_info_data_type_enum2_v()) {
		if (pri_base) {
			*pri_base =
				(top_device_info_data_pri_base_v(table_entry)
				<< top_device_info_data_pri_base_align_v());
		}
		if (fault_id && (top_device_info_data_fault_id_v(table_entry) ==
			top_device_info_data_fault_id_valid_v())) {
			*fault_id =
			    top_device_info_data_fault_id_enum_v(table_entry);
		}
	} else
		nvgpu_err(g, "unknown device_info_data %d",
				top_device_info_data_type_v(table_entry));
}

void gm20b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
{
	/*
	 * These are all errors which indicate something really wrong
	 * going on in the device.
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
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}
