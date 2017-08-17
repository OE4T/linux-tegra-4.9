/*
 * Virtualized GPU CE2
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

#include "vgpu/vgpu.h"

#include <nvgpu/bug.h>

int vgpu_ce2_nonstall_isr(struct gk20a *g,
			struct tegra_vgpu_ce2_nonstall_intr_info *info)
{
	gk20a_dbg_fn("");

	switch (info->type) {
	case TEGRA_VGPU_CE2_NONSTALL_INTR_NONBLOCKPIPE:
		gk20a_channel_semaphore_wakeup(g, true);
		break;
	default:
		WARN_ON(1);
		break;
	}

	return 0;
}

static u32 vgpu_ce_get_num_pce(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	return priv->constants.num_pce;
}

void vgpu_init_ce2_ops(struct gpu_ops *gops)
{
	gops->ce2.get_num_pce = vgpu_ce_get_num_pce;
}
