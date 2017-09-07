/*
 * GM20B CDE
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVHOST_GM20B_CDE
#define _NVHOST_GM20B_CDE

struct gk20a;

void gm20b_cde_get_program_numbers(struct gk20a *g,
					  u32 block_height_log2,
					  u32 shader_parameter,
					  int *hprog_out, int *vprog_out);

#endif
