/*
 * general p state infrastructure
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
#ifndef _ctrlvolt_h_
#define _ctrlvolt_h_

#define CTRL_VOLT_VOLT_RAIL_MAX_RAILS                                 \
	CTRL_BOARDOBJGRP_E32_MAX_OBJECTS

#include "ctrlperf.h"
#include "ctrlboardobj.h"

#define CTRL_VOLT_RAIL_VOLT_DELTA_MAX_ENTRIES                0x04
#define CTRL_VOLT_VOLT_DEV_VID_VSEL_MAX_ENTRIES              (8)
#define CTRL_VOLT_DOMAIN_INVALID                             0x00
#define CTRL_VOLT_DOMAIN_LOGIC                               0x01
#define CLK_PROG_VFE_ENTRY_LOGIC                             0x00

struct ctrl_volt_volt_rail_list_item {
	u8 rail_idx;
	u32 voltage_uv;
};

struct ctrl_volt_volt_rail_list {
	u8    num_rails;
	struct ctrl_volt_volt_rail_list_item
		rails[CTRL_VOLT_VOLT_RAIL_MAX_RAILS];
};

#endif
