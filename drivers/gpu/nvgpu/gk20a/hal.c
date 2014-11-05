/*
 * NVIDIA GPU HAL interface.
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

#include "gk20a.h"
#include "hal_gk20a.h"
#include "gm20b/hal_gm20b.h"

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "nvgpu_gpuid_t18x.h"
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
	case TEGRA_18x_GPUID:
		if (TEGRA_18x_GPUID_HAL(g))
			return -ENODEV;
		break;
#endif
	default:
		gk20a_err(&g->dev->dev, "no support for %x", ver);
		return -ENODEV;
	}

	return 0;
}
