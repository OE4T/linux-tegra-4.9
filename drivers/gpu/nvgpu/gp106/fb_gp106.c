/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
#include "gp10b/fb_gp10b.h"

void gp106_init_fb(struct gpu_ops *gops)
{
	gp10b_init_fb(gops);

	gops->fb.init_fs_state = NULL;
	gops->fb.reset = NULL;
}
