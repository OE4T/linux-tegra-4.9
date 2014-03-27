/*
 * drivers/video/tegra/host/gk20a/platform_gk20a_tegra.c
 *
 * GK20A Tegra Platform Interface
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

#include <linux/debugfs.h>
#include <linux/tegra-powergate.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/nvhost_ioctl.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/pm_domains.h>

#include "../../../../../arch/arm/mach-tegra/iomap.h"

#include "gk20a.h"
#include "hal_gk20a.h"
#include "platform_gk20a.h"
#include "gk20a_scale.h"

#define TEGRA_GK20A_INTR		INT_GPU
#define TEGRA_GK20A_INTR_NONSTALL	INT_GPU_NONSTALL

#define TEGRA_GK20A_SIM_BASE 0x538F0000 /*tbd: get from iomap.h */
#define TEGRA_GK20A_SIM_SIZE 0x1000     /*tbd: this is a high-side guess */

extern struct device tegra_vpr_dev;
struct gk20a_platform t132_gk20a_tegra_platform;

struct gk20a_emc_params {
	long				emc_slope;
	long				emc_offset;
	long				emc_dip_slope;
	long				emc_dip_offset;
	long				emc_xmid;
	bool				linear;
};

/*
 * 20.12 fixed point arithmetic
 */

static const int FXFRAC = 12;
static const int FX_HALF = (1 << 12) / 2;

#define INT_TO_FX(x) ((x) << FXFRAC)
#define FX_TO_INT(x) ((x) >> FXFRAC)

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define HZ_TO_MHZ(x) ((x) / 1000000)

int FXMUL(int x, int y)
{
	return ((long long) x * (long long) y) >> FXFRAC;
}

int FXDIV(int x, int y)
{
	/* long long div operation not supported, must shift manually. This
	 * would have been
	 *
	 *    return (((long long) x) << FXFRAC) / (long long) y;
	 */
	int pos, t;
	if (x == 0)
		return 0;

	/* find largest allowable right shift to numerator, limit to FXFRAC */
	t = x < 0 ? -x : x;
	pos = 31 - fls(t); /* fls can't be 32 if x != 0 */
	if (pos > FXFRAC)
		pos = FXFRAC;

	y >>= FXFRAC - pos;
	if (y == 0)
		return 0x7FFFFFFF; /* overflow, return MAX_FIXED */

	return (x << pos) / y;
}

static int gk20a_tegra_channel_busy(struct platform_device *dev)
{
	int ret = 0;

	/* Explicitly turn on the host1x clocks
	 * - This is needed as host1x driver sets ignore_children = true
	 * to cater the use case of display clock ON but host1x clock OFF
	 * in OS-Idle-Display-ON case
	 * - This was easily done in ACM as it only checked the ref count
	 * of host1x (or any device for that matter) to be zero before
	 * turning off its clock
	 * - However, runtime PM checks to see if *ANY* child of device is
	 * in ACTIVE state and if yes, it doesn't suspend the parent. As a
	 * result of this, display && host1x clocks remains ON during
	 * OS-Idle-Display-ON case
	 * - The code below fixes this use-case
	 */
	if (to_platform_device(dev->dev.parent))
		ret = nvhost_module_busy_ext(
			to_platform_device(dev->dev.parent));

	return ret;
}

static void gk20a_tegra_channel_idle(struct platform_device *dev)
{
	/* Explicitly turn off the host1x clocks */
	if (to_platform_device(dev->dev.parent))
		nvhost_module_idle_ext(to_platform_device(dev->dev.parent));
}

static void gk20a_tegra_secure_destroy(struct platform_device *pdev,
				       struct gr_ctx_buffer_desc *desc)
{
	gk20a_free_sgtable(&desc->sgt);
	dma_free_attrs(&tegra_vpr_dev, desc->size,
			(void *)(uintptr_t)&desc->iova,
			desc->iova, &desc->attrs);
}

static int gk20a_tegra_secure_alloc(struct platform_device *pdev,
				    struct gr_ctx_buffer_desc *desc,
				    size_t size)
{
	struct device *dev = &pdev->dev;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t iova;
	struct sg_table *sgt;
	struct page *page;
	int err = 0;

	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);

	(void)dma_alloc_attrs(&tegra_vpr_dev, size, &iova,
				      GFP_KERNEL, &attrs);
	if (dma_mapping_error(&tegra_vpr_dev, iova))
		return -ENOMEM;

	desc->iova = iova;
	desc->size = size;
	desc->attrs = attrs;
	desc->destroy = gk20a_tegra_secure_destroy;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		gk20a_err(dev, "failed to allocate memory\n");
		goto fail;
	}
	err = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (err) {
		gk20a_err(dev, "failed to allocate sg_table\n");
		goto fail_sgt;
	}
	page = phys_to_page(iova);
	sg_set_page(sgt->sgl, page, size, 0);
	sg_dma_address(sgt->sgl) = iova;

	desc->sgt = sgt;

	return err;

fail_sgt:
	kfree(sgt);
fail:
	dma_free_attrs(&tegra_vpr_dev, desc->size,
			(void *)(uintptr_t)&desc->iova,
			desc->iova, &desc->attrs);
	return err;
}

/*
 * gk20a_tegra_get_emc_rate()
 *
 * This function returns the minimum emc clock based on gpu frequency
 */

long gk20a_tegra_get_emc_rate(struct gk20a_emc_params *emc_params, long freq)
{
	long hz;

	freq = INT_TO_FX(HZ_TO_MHZ(freq));
	hz = FXMUL(freq, emc_params->emc_slope) + emc_params->emc_offset;

	hz -= FXMUL(emc_params->emc_dip_slope,
		FXMUL(freq - emc_params->emc_xmid,
			freq - emc_params->emc_xmid)) +
		emc_params->emc_dip_offset;

	hz = MHZ_TO_HZ(FX_TO_INT(hz + FX_HALF)); /* round to nearest */
	hz = (hz < 0) ? 0 : hz;

	return hz;
}

/*
 * gk20a_tegra_postscale(profile, freq)
 *
 * This function sets emc frequency based on current gpu frequency
 */

static void gk20a_tegra_postscale(struct platform_device *pdev,
				  unsigned long freq)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *emc_params = profile->private_data;
	struct gk20a *g = get_gk20a(pdev);

	long after = gk20a_clk_get_rate(g);
	long emc_target = gk20a_tegra_get_emc_rate(emc_params, after);

	clk_set_rate(platform->clk[2], emc_target);
}

/*
 * gk20a_tegra_prescale(profile, freq)
 *
 * This function informs EDP about changed constraints.
 */

static void gk20a_tegra_prescale(struct platform_device *pdev)
{
	struct gk20a *g = get_gk20a(pdev);
	u32 avg = 0;

	gk20a_pmu_load_norm(g, &avg);
	tegra_edp_notify_gpu_load(avg);
}

/*
 * gk20a_tegra_calibrate_emc()
 *
 * Compute emc scaling parameters
 *
 * Remc = S * R3d + O - (Sd * (R3d - Rm)^2 + Od)
 *
 * Remc - 3d.emc rate
 * R3d  - 3d.cbus rate
 * Rm   - 3d.cbus 'middle' rate = (max + min)/2
 * S    - emc_slope
 * O    - emc_offset
 * Sd   - emc_dip_slope
 * Od   - emc_dip_offset
 *
 * this superposes a quadratic dip centered around the middle 3d
 * frequency over a linear correlation of 3d.emc to 3d clock
 * rates.
 *
 * S, O are chosen so that the maximum 3d rate produces the
 * maximum 3d.emc rate exactly, and the minimum 3d rate produces
 * at least the minimum 3d.emc rate.
 *
 * Sd and Od are chosen to produce the largest dip that will
 * keep 3d.emc frequencies monotonously decreasing with 3d
 * frequencies. To achieve this, the first derivative of Remc
 * with respect to R3d should be zero for the minimal 3d rate:
 *
 *   R'emc = S - 2 * Sd * (R3d - Rm)
 *   R'emc(R3d-min) = 0
 *   S = 2 * Sd * (R3d-min - Rm)
 *     = 2 * Sd * (R3d-min - R3d-max) / 2
 *
 *   +------------------------------+
 *   | Sd = S / (R3d-min - R3d-max) |
 *   +------------------------------+
 *
 *   dip = Sd * (R3d - Rm)^2 + Od
 *
 * requiring dip(R3d-min) = 0 and dip(R3d-max) = 0 gives
 *
 *   Sd * (R3d-min - Rm)^2 + Od = 0
 *   Od = -Sd * ((R3d-min - R3d-max) / 2)^2
 *      = -Sd * ((R3d-min - R3d-max)^2) / 4
 *
 *   +------------------------------+
 *   | Od = (emc-max - emc-min) / 4 |
 *   +------------------------------+
 *
 */

void gk20a_tegra_calibrate_emc(struct gk20a_emc_params *emc_params,
			       struct clk *clk_3d, struct clk *clk_3d_emc)
{
	long correction;
	unsigned long max_emc;
	unsigned long min_emc;
	unsigned long min_rate_3d;
	unsigned long max_rate_3d;

	max_emc = clk_round_rate(clk_3d_emc, UINT_MAX);
	max_emc = INT_TO_FX(HZ_TO_MHZ(max_emc));

	min_emc = clk_round_rate(clk_3d_emc, 0);
	min_emc = INT_TO_FX(HZ_TO_MHZ(min_emc));

	max_rate_3d = clk_round_rate(clk_3d, UINT_MAX);
	max_rate_3d = INT_TO_FX(HZ_TO_MHZ(max_rate_3d));

	min_rate_3d = clk_round_rate(clk_3d, 0);
	min_rate_3d = INT_TO_FX(HZ_TO_MHZ(min_rate_3d));

	emc_params->emc_slope =
		FXDIV((max_emc - min_emc), (max_rate_3d - min_rate_3d));
	emc_params->emc_offset = max_emc -
		FXMUL(emc_params->emc_slope, max_rate_3d);
	/* Guarantee max 3d rate maps to max emc rate */
	emc_params->emc_offset += max_emc -
		(FXMUL(emc_params->emc_slope, max_rate_3d) +
		emc_params->emc_offset);

	emc_params->emc_dip_offset = (max_emc - min_emc) / 4;
	emc_params->emc_dip_slope =
		-FXDIV(emc_params->emc_slope, max_rate_3d - min_rate_3d);
	emc_params->emc_xmid = (max_rate_3d + min_rate_3d) / 2;
	correction =
		emc_params->emc_dip_offset +
			FXMUL(emc_params->emc_dip_slope,
			FXMUL(max_rate_3d - emc_params->emc_xmid,
				max_rate_3d - emc_params->emc_xmid));
	emc_params->emc_dip_offset -= correction;
}

/*
 * gk20a_tegra_railgate()
 *
 * Gate (disable) gk20a power rail
 */

static int gk20a_tegra_railgate(struct platform_device *pdev)
{
	if (tegra_powergate_is_powered(TEGRA_POWERGATE_GPU))
		tegra_powergate_partition(TEGRA_POWERGATE_GPU);
	return 0;
}

/*
 * gk20a_tegra_unrailgate()
 *
 * Ungate (enable) gk20a power rail
 */

static int gk20a_tegra_unrailgate(struct platform_device *pdev)
{
	tegra_unpowergate_partition(TEGRA_POWERGATE_GPU);
	return 0;
}

struct {
	char *name;
	unsigned long default_rate;
} tegra_gk20a_clocks[] = {
	{"PLLG_ref", UINT_MAX},
	{"pwr", 204000000},
	{"emc", UINT_MAX} };

/*
 * gk20a_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gk20a platform data.
 */

static int gk20a_tegra_get_clocks(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	char devname[16];
	int i;
	int ret = 0;

	snprintf(devname, sizeof(devname),
		 (pdev->id <= 0) ? "tegra_%s" : "tegra_%s.%d\n",
		 pdev->name, pdev->id);

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gk20a_clocks); i++) {
		long rate = tegra_gk20a_clocks[i].default_rate;
		struct clk *c;

		c = clk_get_sys(devname, tegra_gk20a_clocks[i].name);
		if (IS_ERR(c)) {
			ret = PTR_ERR(c);
			goto err_get_clock;
		}
		rate = clk_round_rate(c, rate);
		clk_set_rate(c, rate);
		platform->clk[i] = c;
	}
	platform->num_clks = i;

	return 0;

err_get_clock:

	while (i--)
		clk_put(platform->clk[i]);
	return ret;
}

static void gk20a_tegra_scale_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
		struct gk20a_emc_params *emc_params;

	if (!profile)
		return;

	emc_params = kzalloc(sizeof(*emc_params), GFP_KERNEL);
	if (!emc_params)
		return;

	gk20a_tegra_calibrate_emc(emc_params, gk20a_clk_get(platform->g),
				  platform->clk[2]);

	profile->private_data = emc_params;
}

static void gk20a_tegra_debug_dump(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a *g = platform->g;
	nvhost_debug_dump_device(g->dev);
}

static int gk20a_tegra_probe(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA13) {
		t132_gk20a_tegra_platform.g = platform->g;
		*platform = t132_gk20a_tegra_platform;
	}

	gk20a_tegra_get_clocks(dev);

	return 0;
}

static int gk20a_tegra_late_probe(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	/* Make gk20a power domain a subdomain of mc */
	tegra_pd_add_sd(&platform->g->pd);

	/* Initialise tegra specific scaling quirks */
	gk20a_tegra_scale_init(dev);

	return 0;
}

static int gk20a_tegra_suspend(struct device *dev)
{
	tegra_edp_notify_gpu_load(0);
	return 0;
}

static struct resource gk20a_tegra_resources[] = {
	{
	.start = TEGRA_GK20A_BAR0_BASE,
	.end   = TEGRA_GK20A_BAR0_BASE + TEGRA_GK20A_BAR0_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{
	.start = TEGRA_GK20A_BAR1_BASE,
	.end   = TEGRA_GK20A_BAR1_BASE + TEGRA_GK20A_BAR1_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{ /* Used on ASIM only */
	.start = TEGRA_GK20A_SIM_BASE,
	.end   = TEGRA_GK20A_SIM_BASE + TEGRA_GK20A_SIM_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{
	.start = TEGRA_GK20A_INTR,
	.end   = TEGRA_GK20A_INTR,
	.flags = IORESOURCE_IRQ,
	},
	{
	.start = TEGRA_GK20A_INTR_NONSTALL,
	.end   = TEGRA_GK20A_INTR_NONSTALL,
	.flags = IORESOURCE_IRQ,
	},
};

struct gk20a_platform t132_gk20a_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,

	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,
	.railgate = gk20a_tegra_railgate,
	.unrailgate = gk20a_tegra_unrailgate,

	/* frequency scaling configuration */
	.prescale = gk20a_tegra_prescale,
	.postscale = gk20a_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",
	.qos_id = PM_QOS_GPU_FREQ_MIN,

	.channel_busy = gk20a_tegra_channel_busy,
	.channel_idle = gk20a_tegra_channel_idle,
	.secure_alloc = gk20a_tegra_secure_alloc,
	.dump_platform_dependencies = gk20a_tegra_debug_dump,
};

struct gk20a_platform gk20a_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,
	.can_railgate		= true,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,

	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,
	.railgate = gk20a_tegra_railgate,
	.unrailgate = gk20a_tegra_unrailgate,

	/* frequency scaling configuration */
	.prescale = gk20a_tegra_prescale,
	.postscale = gk20a_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",
	.qos_id = PM_QOS_GPU_FREQ_MIN,

	.channel_busy = gk20a_tegra_channel_busy,
	.channel_idle = gk20a_tegra_channel_idle,
	.secure_alloc = gk20a_tegra_secure_alloc,
	.dump_platform_dependencies = gk20a_tegra_debug_dump,
};

struct platform_device tegra_gk20a_device = {
	.name		= "gk20a",
	.resource	= gk20a_tegra_resources,
	.num_resources	= ARRAY_SIZE(gk20a_tegra_resources),
	.dev		= {
		.platform_data = &gk20a_tegra_platform,
	},
};
