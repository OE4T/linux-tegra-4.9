/*
 * GV100 FB
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/types.h>

#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/gmmu.h>

#include "gk20a/gk20a.h"
#include "gv100/fb_gv100.h"

#include <nvgpu/hw/gv100/hw_fb_gv100.h>

#define HW_SCRUB_TIMEOUT_DEFAULT	100 /* usec */
#define HW_SCRUB_TIMEOUT_MAX		2000000 /* usec */

void gv100_fb_reset(struct gk20a *g)
{
	u32 val;
	int retries = HW_SCRUB_TIMEOUT_MAX / HW_SCRUB_TIMEOUT_DEFAULT;

	nvgpu_info(g, "reset gv100 fb");

	/* wait for memory to be accessible */
	do {
		u32 w = gk20a_readl(g, fb_niso_scrub_status_r());
		if (fb_niso_scrub_status_flag_v(w)) {
			nvgpu_info(g, "done");
			break;
		}
		nvgpu_udelay(HW_SCRUB_TIMEOUT_DEFAULT);
	} while (--retries);

	val = gk20a_readl(g, fb_mmu_priv_level_mask_r());
	val &= ~fb_mmu_priv_level_mask_write_violation_m();
	gk20a_writel(g, fb_mmu_priv_level_mask_r(), val);
}
