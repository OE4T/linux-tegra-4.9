/*
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

#include <linux/kernel.h>

#include "gm20b/hw_gr_gm20b.h"
#include "gk20a/gk20a.h"
#include "vgpu/vgpu.h"
#include "vgpu_gr_gm20b.h"

static void vgpu_gm20b_detect_sm_arch(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(g->dev);
	u32 v = 0;

	gk20a_dbg_fn("");

	if (vgpu_get_attribute(platform->virt_handle,
			TEGRA_VGPU_ATTRIB_GPC0_TPC0_SM_ARCH, &v))
		gk20a_err(dev_from_gk20a(g), "failed to retrieve SM arch");

	g->gpu_characteristics.sm_arch_spa_version =
		gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	g->gpu_characteristics.sm_arch_sm_version =
		gr_gpc0_tpc0_sm_arch_sm_version_v(v);
	g->gpu_characteristics.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

void vgpu_gm20b_init_gr_ops(struct gpu_ops *gops)
{
	gops->gr.detect_sm_arch = vgpu_gm20b_detect_sm_arch;
}
