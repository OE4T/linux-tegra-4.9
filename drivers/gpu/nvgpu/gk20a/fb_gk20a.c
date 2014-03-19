/*
 * GK20A memory interface
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a.h"
#include "kind_gk20a.h"
#include "hw_mc_gk20a.h"

static void fb_gk20a_reset(struct gk20a *g)
{
	gk20a_dbg_info("reset gk20a fb");

	gk20a_reset(g, mc_enable_pfb_enabled_f()
			| mc_enable_l2_enabled_f()
			| mc_enable_xbar_enabled_f()
			| mc_enable_hub_enabled_f());
}

void gk20a_init_fb(struct gpu_ops *gops)
{
	gops->fb.reset = fb_gk20a_reset;
	gk20a_init_uncompressed_kind_map();
	gk20a_init_kind_attr();
}
