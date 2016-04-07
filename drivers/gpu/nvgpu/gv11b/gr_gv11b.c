/*
 * GV11B GPU GR
 *
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

#include "gk20a/gk20a.h" /* FERMI and MAXWELL classes defined here */
#include <linux/delay.h>
#include <linux/tegra-fuse.h>

#include "gk20a/gr_gk20a.h"
#include "gk20a/semaphore_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"

#include "gm20b/gr_gm20b.h" /* for MAXWELL classes */
#include "gp10b/gr_gp10b.h"
#include "gv11b/gr_gv11b.h"

void gv11b_init_gr(struct gpu_ops *gops)
{
	gp10b_init_gr(gops);
}
