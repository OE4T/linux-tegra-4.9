/*
 * Tegra Graphics Host Unit clock scaling
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

#include <linux/devfreq.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/slab.h>

#include <mach/clk.h>
#include <mach/hardware.h>

#include <governor.h>

#include "dev.h"
#include "chip_support.h"
#include "nvhost_acm.h"
#include "nvhost_scale.h"
#include "host1x/host1x_actmon.h"

/*
 * nvhost_scale_target(dev, *freq, flags)
 *
 * This function scales the clock
 */

static int nvhost_scale_target(struct device *dev, unsigned long *freq,
			       u32 flags)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	if (!tegra_is_clk_enabled(profile->clk)) {
		*freq = 0;
		return 0;
	}

	*freq = clk_round_rate(clk_get_parent(profile->clk), *freq);
	if (clk_get_rate(profile->clk) == *freq)
		return 0;

	nvhost_module_set_devfreq_rate(profile->pdev, 0, *freq);
	if (pdata->scaling_post_cb)
		pdata->scaling_post_cb(profile, *freq);

	*freq = (long) clk_get_rate(profile->clk);

	return 0;
}

/*
 * update_load_estimate(profile)
 *
 * Update load estimate using busy/idle flag.
 */

static void update_load_estimate(struct nvhost_device_profile *profile,
				 bool busy)
{
	ktime_t t;
	unsigned long dt;

	t = ktime_get();
	dt = ktime_us_delta(t, profile->last_event_time);

	profile->dev_stat.total_time += dt;
	profile->last_event_time = t;

	if (profile->busy)
		profile->dev_stat.busy_time += dt;

	profile->busy = busy;
}

/*
 * update_load_estimate_actmon(profile)
 *
 * Update load estimate using hardware actmon. The actmon value is normalised
 * based on the time it was asked last time.
 */

static void update_load_estimate_actmon(struct nvhost_device_profile *profile)
{
	ktime_t t;
	unsigned long dt;
	u32 busy_time;

	t = ktime_get();
	dt = ktime_us_delta(t, profile->last_event_time);

	profile->dev_stat.total_time = dt;
	profile->last_event_time = t;
	actmon_op().read_avg_norm(profile->actmon, &busy_time);
	profile->dev_stat.busy_time = (busy_time * dt) / 1000;
}

/*
 * nvhost_scale_notify(pdev, busy)
 *
 * Calling this function informs that the device is idling (..or busy). This
 * data is used to estimate the current load
 */

static void nvhost_scale_notify(struct platform_device *pdev, bool busy)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;
	struct devfreq *devfreq = pdata->power_manager;

	/* Is the device profile initialised? */
	if (!profile)
		return;

	/* If defreq is disabled, set the freq to max or min */
	if (!devfreq) {
		unsigned long freq = busy ? UINT_MAX : 0;
		nvhost_scale_target(&pdev->dev, &freq, 0);
		return;
	}

	mutex_lock(&devfreq->lock);
	if (!profile->actmon)
		update_load_estimate(profile, busy);
	profile->last_event_type = busy ? DEVICE_BUSY : DEVICE_IDLE;
	update_devfreq(devfreq);
	mutex_unlock(&devfreq->lock);
}

void nvhost_scale_notify_idle(struct platform_device *pdev)
{
	nvhost_scale_notify(pdev, false);

}

void nvhost_scale_notify_busy(struct platform_device *pdev)
{
	nvhost_scale_notify(pdev, true);
}

/*
 * nvhost_scale_get_dev_status(dev, *stat)
 *
 * This function queries the current device status.
 */

static int nvhost_scale_get_dev_status(struct device *dev,
					      struct devfreq_dev_status *stat)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	/* Make sure there are correct values for the current frequency */
	profile->dev_stat.current_frequency = clk_get_rate(profile->clk);

	if (profile->actmon)
		update_load_estimate_actmon(profile);

	/* Copy the contents of the current device status */
	*stat = profile->dev_stat;

	/* Finally, clear out the local values */
	profile->dev_stat.total_time = 0;
	profile->dev_stat.busy_time = 0;
	profile->last_event_type = DEVICE_UNKNOWN;

	return 0;
}

static struct devfreq_dev_profile nvhost_scale_devfreq_profile = {
	.initial_freq   = 0,
	.polling_ms     = 0,
	.target         = nvhost_scale_target,
	.get_dev_status = nvhost_scale_get_dev_status,
};

/*
 * nvhost_scale_init(pdev)
 */

void nvhost_scale_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile;

	if (pdata->power_profile)
		return;

	profile = kzalloc(sizeof(struct nvhost_device_profile), GFP_KERNEL);
	if (!profile)
		return;
	pdata->power_profile = profile;
	profile->pdev = pdev;
	profile->clk = pdata->clk[0];
	profile->last_event_type = DEVICE_UNKNOWN;

	/* Initialize devfreq related structures */
	profile->dev_stat.private_data = &profile->ext_stat;
	profile->ext_stat.min_freq =
		clk_round_rate(clk_get_parent(profile->clk), 0);
	profile->ext_stat.max_freq =
		clk_round_rate(clk_get_parent(profile->clk), UINT_MAX);
	profile->ext_stat.busy = DEVICE_UNKNOWN;

	if (profile->ext_stat.min_freq == profile->ext_stat.max_freq) {
		dev_warn(&pdev->dev, "max rate = min rate (%lu), disabling scaling\n",
			 profile->ext_stat.min_freq);
		goto err_fetch_clocks;
	}

	/* Initialize actmon */
	if (pdata->actmon_enabled) {
		profile->actmon = kzalloc(sizeof(struct host1x_actmon),
					  GFP_KERNEL);
		if (!profile->actmon)
			goto err_allocate_actmon;

		profile->actmon->host = nvhost_get_host(pdev);
		profile->actmon->regs = nvhost_get_host(pdev)->aperture +
			pdata->actmon_regs;

		actmon_op().init(profile->actmon);
		actmon_op().debug_init(profile->actmon, pdata->debugfs);
		actmon_op().deinit(profile->actmon);
	}

	if (pdata->devfreq_governor) {
		struct devfreq *devfreq = devfreq_add_device(&pdev->dev,
					&nvhost_scale_devfreq_profile,
					pdata->devfreq_governor, NULL);

		if (IS_ERR(devfreq))
			devfreq = NULL;

		pdata->power_manager = devfreq;
	}

	return;

err_allocate_actmon:
err_fetch_clocks:
	kfree(pdata->power_profile);
	pdata->power_profile = NULL;
}

/*
 * nvhost_scale_deinit(dev)
 *
 * Stop scaling for the given device.
 */

void nvhost_scale_deinit(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	if (!profile)
		return;

	if (pdata->power_manager)
		devfreq_remove_device(pdata->power_manager);

	kfree(profile->actmon);
	kfree(profile);
	pdata->power_profile = NULL;
}

/*
 * nvhost_scale_hw_init(dev)
 *
 * Initialize hardware portion of the device
 */

void nvhost_scale_hw_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	if (profile && profile->actmon)
		actmon_op().init(profile->actmon);
}

/*
 * nvhost_scale_hw_deinit(dev)
 *
 * Deinitialize the hw partition related to scaling
 */

void nvhost_scale_hw_deinit(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_device_profile *profile = pdata->power_profile;

	if (profile && profile->actmon)
		actmon_op().deinit(profile->actmon);
}
