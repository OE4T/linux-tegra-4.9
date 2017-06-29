/*
 * GK20A PRIV ringmaster
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __PRIV_RING_GK20A_H__
#define __PRIV_RING_GK20A_H__

struct gpu_ops;

void gk20a_priv_ring_isr(struct gk20a *g);
void gk20a_enable_priv_ring(struct gk20a *g);

#endif /*__PRIV_RING_GK20A_H__*/
