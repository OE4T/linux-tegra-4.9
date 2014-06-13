/*
 * GM20B PMU
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "acr_gm20b.h"

void gm20b_init_pmu_ops(struct gpu_ops *gops)
{
#ifdef CONFIG_TEGRA_ACR
	gm20b_init_secure_pmu(gops);
#else
	gk20a_init_pmu_ops(gops);
#endif
}
