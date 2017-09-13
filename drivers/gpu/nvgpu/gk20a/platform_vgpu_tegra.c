/*
 * Tegra Virtualized GPU Platform Interface
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
