/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <dev.h>
#include "gk20a/gk20a.h"
#include "hw_fb_gm20b.h"
#include "hw_top_gm20b.h"

static void fb_gm20b_init_fs_state(struct gk20a *g)
{
	nvhost_dbg_info("initialize gm20b fb");

	gk20a_writel(g, fb_fbhub_num_active_ltcs_r(),
			g->ltc_count);
}

void gm20b_init_fb(struct gpu_ops *gops)
{
	gops->fb.init_fs_state = fb_gm20b_init_fs_state;
}
