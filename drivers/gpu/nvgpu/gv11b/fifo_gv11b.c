/*
 * GV11B fifo
 *
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/types.h>

#include "gk20a/gk20a.h"
#include "gp10b/fifo_gp10b.h"
#include "hw_pbdma_gv11b.h"
#include "fifo_gv11b.h"
#include "hw_fifo_gv11b.h"

void gv11b_init_fifo(struct gpu_ops *gops)
{
	gp10b_init_fifo(gops);
}
