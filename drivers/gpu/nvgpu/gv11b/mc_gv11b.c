/*
 * GV11B master
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>

#include "gk20a/gk20a.h"

#include "gp10b/mc_gp10b.h"

#include "mc_gv11b.h"
#include "fb_gv11b.h"

#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>

void mc_gv11b_intr_enable(struct gk20a *g)
{
	u32 eng_intr_mask = gk20a_fifo_engine_interrupt_mask(g);

	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING),
				0xffffffff);
	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
				0xffffffff);
	gv11b_fb_disable_hub_intr(g, STALL_REG_INDEX, HUB_INTR_TYPE_ALL);

	g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING] =
				mc_intr_pfifo_pending_f() |
				mc_intr_hub_pending_f() |
				mc_intr_priv_ring_pending_f() |
				mc_intr_pbus_pending_f() |
				mc_intr_ltc_pending_f() |
				eng_intr_mask;

	g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING] =
				mc_intr_pfifo_pending_f()
			     | eng_intr_mask;

	/* TODO: Enable PRI faults for HUB ECC err intr */
	gv11b_fb_enable_hub_intr(g, STALL_REG_INDEX, g->mm.hub_intr_types);

	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_STALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING]);

	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_NONSTALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING]);

}

bool gv11b_mc_is_intr_hub_pending(struct gk20a *g, u32 mc_intr_0)
{
	return ((mc_intr_0 & mc_intr_hub_pending_f()) ? true : false);
}
