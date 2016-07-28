/*
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
#ifndef _GPMUIFVOLT_H_
#define _GPMUIFVOLT_H_

#include "ctrl/ctrlvolt.h"

/*!
* Structure containing the number of voltage rails and the list of rail items
* @ref CTRL_PERF_VOLT_RAIL_LIST_ITEM.
*/
struct nv_pmu_volt_volt_rail_list {
	/*!
	* Number of VOLT_RAILs that require the voltage change.
	*/
	u8 num_rails;
	/*!
	* List of @ref CTRL_PERF_VOLT_RAIL_LIST_ITEM entries.
	*/
	struct ctrl_perf_volt_rail_list_item rails[2];
};

#endif  /* _GPMUIFVOLT_H_*/
