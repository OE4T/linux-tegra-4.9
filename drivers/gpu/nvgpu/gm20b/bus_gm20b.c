/*
 * GM20B MMU
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

#include <nvgpu/timers.h>
#include <nvgpu/bus.h>

#include "bus_gm20b.h"
#include "gk20a/gk20a.h"
#include "gk20a/bus_gk20a.h"

#include <nvgpu/hw/gm20b/hw_bus_gm20b.h>

int gm20b_bus_bar1_bind(struct gk20a *g, struct nvgpu_mem *bar1_inst)
{
	struct nvgpu_timeout timeout;
	int err = 0;
	u64 iova = gk20a_mm_inst_block_addr(g, bar1_inst);
	u32 ptr_v = (u32)(iova >> bar1_instance_block_shift_gk20a());

	gk20a_dbg_info("bar1 inst block ptr: 0x%08x", ptr_v);

	gk20a_writel(g, bus_bar1_block_r(),
		     nvgpu_aperture_mask(g, bar1_inst,
		       bus_bar1_block_target_sys_mem_ncoh_f(),
		       bus_bar1_block_target_vid_mem_f()) |
		     bus_bar1_block_mode_virtual_f() |
		     bus_bar1_block_ptr_f(ptr_v));
	nvgpu_timeout_init(g, &timeout, 1000, NVGPU_TIMER_RETRY_TIMER);
	do {
		u32 val = gk20a_readl(g, bus_bind_status_r());
		u32 pending = bus_bind_status_bar1_pending_v(val);
		u32 outstanding = bus_bind_status_bar1_outstanding_v(val);
		if (!pending && !outstanding)
			break;

		nvgpu_udelay(5);
	} while (!nvgpu_timeout_expired(&timeout));

	if (nvgpu_timeout_peek_expired(&timeout))
		err = -EINVAL;

	return err;
}
