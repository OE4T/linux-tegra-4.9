/*
 * Virtualized GPU CE2
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
