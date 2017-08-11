/*
 * GV11B MM
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef MM_GV11B_H
#define MM_GV11B_H

#define HW_FAULT_BUF_STATUS_ALLOC_TRUE	1
#define HW_FAULT_BUF_STATUS_ALLOC_FALSE	0

struct gpu_ops;

void gv11b_init_mm(struct gpu_ops *gops);
#endif
