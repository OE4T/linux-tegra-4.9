/*
 * Tegra Virtualized GPU Platform Interface
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include "platform_gk20a.h"
#include "vgpu/clk_vgpu.h"

#include <nvgpu/nvhost.h>

static int gk20a_tegra_probe(struct device *dev)
{
#ifdef CONFIG_TEGRA_GK20A_NVHOST
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	int ret;

	ret = nvgpu_get_nvhost_dev(platform->g);
	if (ret)
		return ret;

	vgpu_init_clk_support(platform->g);
	return 0;
#else
	return 0;
#endif
}

struct gk20a_platform vgpu_tegra_platform = {
	.has_syncpoints = true,
	.aggressive_sync_destroy_thresh = 64,

	/* power management configuration */
	.can_railgate_init	= false,
	.can_elpg_init          = false,
	.enable_slcg            = false,
	.enable_blcg            = false,
	.enable_elcg            = false,
	.enable_elpg            = false,
	.enable_aelpg           = false,

	.ch_wdt_timeout_ms = 5000,

	.probe = gk20a_tegra_probe,

	.clk_round_rate = vgpu_clk_round_rate,
	.get_clk_freqs = vgpu_clk_get_freqs,

	/* frequency scaling configuration */
	.devfreq_governor = "userspace",

	.virtual_dev = true,
};
