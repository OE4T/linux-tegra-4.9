/*
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
#ifndef __NVGPU_BITOPS_H__
#define __NVGPU_BITOPS_H__

#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/bitmap.h>
#else
#include <nvgpu_rmos/include/bitops.h>
#endif

#endif
