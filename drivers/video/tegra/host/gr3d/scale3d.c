/*
 * drivers/video/tegra/host/gr3d/scale3d.c
 *
 * Tegra Graphics Host 3D clock scaling
 *
 * Copyright (c) 2010-2013, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * 3d clock scaling mechanism
 *
 * module3d_notify_busy() is called upon submit, module3d_notify_idle() is
 * called when all outstanding submits are completed. Both functions notify
 * the governor about changed state.
 *
 * 3d.emc clock is scaled proportionately to 3d clock, with a quadratic-
 * bezier-like factor added to pull 3d.emc rate a bit lower.
 */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/ftrace.h>
#include <linux/platform_data/tegra_edp.h>

#include <mach/hardware.h>

#include "chip_support.h"
#include "dev.h"
#include "scale3d.h"
#include "nvhost_acm.h"
#include "nvhost_scale.h"

#define POW2(x) ((x) * (x))

/*
 * 20.12 fixed point arithmetic
 */

static const int FXFRAC = 12;
static const int FX_HALF = (1 << 12) / 2;

#define INT_TO_FX(x) ((x) << FXFRAC)
#define FX_TO_INT(x) ((x) >> FXFRAC)

static int FXMUL(int x, int y);
static int FXDIV(int x, int y);


#define MHZ_TO_HZ(x) ((x) * 1000000)
#define HZ_TO_MHZ(x) ((x) / 1000000)

/*
 * nvhost_gr3d_params - Parameters for emc (and gr3d2) scaling
 */

struct nvhost_gr3d_params {
	long				emc_slope;
	long				emc_offset;
	long				emc_dip_slope;
	long				emc_dip_offset;
	long				emc_xmid;

	int				clk_3d;
	int				clk_3d2;
	int				clk_3d_emc;
};

/* Convert clk index to struct clk * */
static inline struct clk *clk(struct nvhost_device_profile *profile, int index)
{
	struct nvhost_device_data *pdata =
		platform_get_drvdata(profile->pdev);
	return pdata->clk[index];
}

void nvhost_scale3d_callback(struct nvhost_device_profile *profile,
			     unsigned long freq)
{
	struct nvhost_gr3d_params *emc_params = profile->private_data;
	long hz;
	long after;

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3)
		nvhost_module_set_devfreq_rate(profile->pdev,
					       emc_params->clk_3d2, freq);

	/* Set EMC clockrate */
	after = (long) clk_get_rate(clk(profile, emc_params->clk_3d));
	after = INT_TO_FX(HZ_TO_MHZ(after));
	hz = FXMUL(after, emc_params->emc_slope) + emc_params->emc_offset;

	hz -= FXMUL(emc_params->emc_dip_slope,
		FXMUL(after - emc_params->emc_xmid,
			after - emc_params->emc_xmid)) +
		emc_params->emc_dip_offset;

	hz = MHZ_TO_HZ(FX_TO_INT(hz + FX_HALF)); /* round to nearest */

	hz = (hz < 0) ? 0 : hz;

	nvhost_module_set_devfreq_rate(profile->pdev, emc_params->clk_3d_emc,
				       hz);

	if (profile->actmon) {
		u32 avg = 0;
		actmon_op().read_avg_norm(profile->actmon, &avg);
		tegra_edp_notify_gpu_load(avg);
	}
}

/*
 * nvhost_scale3d_calibrate_emc()
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
 *   Sd = S / (R3d-min - R3d-max)
 *
 *   +---------------------------------------------------+
 *   | Sd = -(emc-max - emc-min) / (R3d-min - R3d-max)^2 |
 *   +---------------------------------------------------+
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

static void nvhost_scale3d_calibrate_emc(struct nvhost_device_profile *profile)
{
	struct nvhost_gr3d_params *emc_params = profile->private_data;
	long correction;
	unsigned long max_emc;
	unsigned long min_emc;
	unsigned long min_rate_3d;
	unsigned long max_rate_3d;

	max_emc =
		clk_round_rate(clk(profile, emc_params->clk_3d_emc), UINT_MAX);
	max_emc = INT_TO_FX(HZ_TO_MHZ(max_emc));

	min_emc = clk_round_rate(clk(profile, emc_params->clk_3d_emc), 0);
	min_emc = INT_TO_FX(HZ_TO_MHZ(min_emc));

	max_rate_3d =
		clk_round_rate(clk(profile, emc_params->clk_3d), UINT_MAX);
	max_rate_3d = INT_TO_FX(HZ_TO_MHZ(max_rate_3d));

	min_rate_3d =
		clk_round_rate(clk(profile, emc_params->clk_3d), 0);
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
		-4 * FXDIV(emc_params->emc_dip_offset,
		(FXMUL(max_rate_3d - min_rate_3d,
			max_rate_3d - min_rate_3d)));
	emc_params->emc_xmid = (max_rate_3d + min_rate_3d) / 2;
	correction =
		emc_params->emc_dip_offset +
			FXMUL(emc_params->emc_dip_slope,
			FXMUL(max_rate_3d - emc_params->emc_xmid,
				max_rate_3d - emc_params->emc_xmid));
	emc_params->emc_dip_offset -= correction;
}

/*
 * nvhost_scale3d_init(dev)
 *
 * Initialise 3d clock scaling for the given device. This function installs
 * pod_scaling governor to handle the clock scaling.
 */

void nvhost_scale3d_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile;
	struct nvhost_gr3d_params *emc_params;

	nvhost_scale_init(pdev);
	profile = pdata->power_profile;
	if (!profile)
		return;

	emc_params = kzalloc(sizeof(*emc_params), GFP_KERNEL);
	if (!emc_params)
		goto err_allocate_emc_params;

	emc_params->clk_3d = 0;
	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3) {
		emc_params->clk_3d2 = 1;
		emc_params->clk_3d_emc = 2;
	} else
		emc_params->clk_3d_emc = 1;

	profile->private_data = emc_params;

	nvhost_scale3d_calibrate_emc(profile);

	return;

err_allocate_emc_params:
	nvhost_scale_deinit(pdev);
}

/*
 * nvhost_scale3d_deinit(dev)
 *
 * Stop 3d scaling for the given device.
 */

void nvhost_scale3d_deinit(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	if (!pdata->power_profile)
		return;

	kfree(pdata->power_profile->private_data);
	pdata->power_profile->private_data = NULL;

	nvhost_scale_deinit(pdev);
}

/*
 * 20.12 fixed point arithmetic
 *
 * int FXMUL(int x, int y)
 * int FXDIV(int x, int y)
 */

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
