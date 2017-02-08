/*
 * NVIDIA GPU HAL interface.
 *
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

#include "gk20a.h"
#include "hal_gk20a.h"
#include "gm20b/hal_gm20b.h"
#include "gp10b/hal_gp10b.h"
#include "gp106/hal_gp106.h"

#ifdef CONFIG_TEGRA_19x_GPU
#include "nvgpu_gpuid_t19x.h"
#endif

int gpu_init_hal(struct gk20a *g)
{
	u32 ver = g->gpu_characteristics.arch + g->gpu_characteristics.impl;
	switch (ver) {
	case GK20A_GPUID_GK20A:
		gk20a_dbg_info("gk20a detected");
		gk20a_init_hal(g);
		break;
	case GK20A_GPUID_GM20B:
		gk20a_dbg_info("gm20b detected");
		if (gm20b_init_hal(g))
			return -ENODEV;
		break;
#if defined(CONFIG_ARCH_TEGRA_18x_SOC)
	case NVGPU_GPUID_GP10B:
		if (gp10b_init_hal(g))
			return -ENODEV;
		break;
	case NVGPU_GPUID_GP104:
	case NVGPU_GPUID_GP106:
		if (gp106_init_hal(g))
			return -ENODEV;
		break;
#endif
#ifdef CONFIG_TEGRA_19x_GPU
	case TEGRA_19x_GPUID:
		if (TEGRA_19x_GPUID_HAL(g))
			return -ENODEV;
		break;
#endif
	default:
		gk20a_err(g->dev, "no support for %x", ver);
		return -ENODEV;
	}

	return 0;
}
