/*
 * GP10B fifo
 *
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

#include <linux/types.h>

#include "gk20a/gk20a.h"
#include "gm20b/fifo_gm20b.h"
#include "hw_pbdma_gp10b.h"

static u32 gp10b_fifo_get_pbdma_signature(struct gk20a *g)
{
	return g->gpu_characteristics.gpfifo_class 
		| pbdma_signature_sw_zero_f();
}

void gp10b_init_fifo(struct gpu_ops *gops)
{
	gm20b_init_fifo(gops);
	gops->fifo.get_pbdma_signature = gp10b_fifo_get_pbdma_signature;
}
