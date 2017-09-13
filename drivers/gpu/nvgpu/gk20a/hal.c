/*
 * NVIDIA GPU HAL interface.
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "gk20a.h"
#include "hal.h"
#include "gm20b/hal_gm20b.h"
#include "gp10b/hal_gp10b.h"
#include "gp106/hal_gp106.h"

#ifdef CONFIG_TEGRA_19x_GPU
#include "nvgpu_gpuid_t19x.h"
#endif

#include <nvgpu/log.h>

int gpu_init_hal(struct gk20a *g)
{
	u32 ver = g->gpu_characteristics.arch + g->gpu_characteristics.impl;
	switch (ver) {
	case GK20A_GPUID_GM20B:
	case GK20A_GPUID_GM20B_B:
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
	case BIGGPU_19x_GPUID:
		if (BIGGPU_19x_GPUID_HAL(g))
			return -ENODEV;
		break;

#endif
	default:
		nvgpu_err(g, "no support for %x", ver);
		return -ENODEV;
	}

	return 0;
}
