/*
 * GK20A L2
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef LTC_GK20A_H
#define LTC_GK20A_H
struct gk20a;

int gk20a_ltc_alloc_phys_cbc(struct gk20a *g, size_t compbit_backing_size);
int gk20a_ltc_alloc_virt_cbc(struct gk20a *g, size_t compbit_backing_size);
#endif
