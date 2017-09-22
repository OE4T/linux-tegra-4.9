/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "vgpu/clk_vgpu.h"

#include <nvgpu/nvhost.h>
#include <linux/platform_device.h>

static int gv11b_vgpu_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct resource *r;
	void __iomem *regs;
	struct fifo_gk20a *f = &platform->g->fifo;
	int ret;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usermode");
	if (!r) {
		dev_err(dev, "failed to get usermode regs\n");
		return -ENXIO;
	}
	regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(regs)) {
		dev_err(dev, "failed to map usermode regs\n");
		return PTR_ERR(regs);
	}
	f->t19x.usermode_regs = regs;

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	ret = nvgpu_get_nvhost_dev(platform->g);
	if (ret) {
		f->t19x.usermode_regs = NULL;
		return ret;
	}
#endif
	vgpu_init_clk_support(platform->g);

	return 0;
}

struct gk20a_platform gv11b_vgpu_tegra_platform = {
	.has_syncpoints = false,
	.aggressive_sync_destroy_thresh = 64,

	/* power management configuration */
	.can_railgate_init	= false,
	.can_elpg_init          = false,
	.enable_slcg            = false,
	.enable_blcg            = false,
	.enable_elcg            = false,
	.enable_elpg            = false,
	.enable_aelpg           = false,
	.can_slcg               = false,
	.can_blcg               = false,
	.can_elcg               = false,

	.ch_wdt_timeout_ms = 5000,

	.probe = gv11b_vgpu_probe,

	.clk_round_rate = vgpu_clk_round_rate,
	.get_clk_freqs = vgpu_clk_get_freqs,

	/* frequency scaling configuration */
	.devfreq_governor = "userspace",

	.virtual_dev = true,
};
