/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_BIOS_GM206_H
#define NVGPU_BIOS_GM206_H

#define PERF_PTRS_WIDTH 0x4

enum {
	CLOCKS_TABLE = 2,
	CLOCK_PROGRAMMING_TABLE,
	NAFLL_TABLE,
	ADC_TABLE,
};

enum {
	CONTINUOUS_VIRTUAL_BINNING_TABLE,
};

struct bit_token {
	u8 token_id;
	u8 data_version;
	u16 data_size;
	u16 data_ptr;
} __packed;

struct gpu_ops;

void gm206_init_bios(struct gpu_ops *gops);
#endif
