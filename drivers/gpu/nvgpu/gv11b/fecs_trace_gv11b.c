/*
 * GV11B GPU FECS traces
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

#include "gp10b/fecs_trace_gp10b.h"
#include "gv11b/fecs_trace_gv11b.h"

#ifdef CONFIG_GK20A_CTXSW_TRACE
int gv11b_init_fecs_trace_ops(struct gpu_ops *ops)
{
	gp10b_init_fecs_trace_ops(ops);
	return 0;
}
#else
int gv11b_init_fecs_trace_ops(struct gpu_ops *ops)
{
	return 0;
}
#endif /* CONFIG_GK20A_CTXSW_TRACE */
