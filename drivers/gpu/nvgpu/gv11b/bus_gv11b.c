/*
 * GV11B BUS
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

#include "bus_gv11b.h"
#include "gk20a/gk20a.h"
#include "gk20a/bus_gk20a.h"

void gv11b_init_bus(struct gpu_ops *gops)
{
	gops->bus.init_hw = gk20a_bus_init_hw;
	gops->bus.isr = gk20a_bus_isr;
	gops->bus.read_ptimer = gk20a_read_ptimer;
	gops->bus.bar1_bind = NULL;
}
