/*
 * gk20a clock scaling profile
 *
 * Copyright (c) 2013-2014, NVIDIA Corporation. All rights reserved.
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
#include <linux/clk/tegra.h>
#include <linux/tegra-soc.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/pm_qos.h>

#include <governor.h>

#include "gk20a.h"
#include "pmu_gk20a.h"
#include "clk_gk20a.h"
#include "gk20a_scale.h"

static ssize_t gk20a_scale_load_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gk20a *g = get_gk20a(pdev);
	u32 busy_time;
	ssize_t res;

	if (!g->power_on) {
		busy_time = 0;
	} else {
		gk20a_busy(g->dev);
		gk20a_pmu_load_norm(g, &busy_time);
		gk20a_idle(g->dev);
	}

	res = snprintf(buf, PAGE_SIZE, "%u\n", busy_time);

	return res;
}

static DEVICE_ATTR(load, S_IRUGO, gk20a_scale_load_show, NULL);

/*
 * gk20a_scale_qos_notify()
 *
 * This function is called when the minimum QoS requirement for the device
 * has changed. The function calls postscaling callback if it is defined.
 */

static int gk20a_scale_qos_notify(struct notifier_block *nb,
				  unsigned long n, void *p)
{
	struct gk20a_scale_profile *profile =
		container_of(nb, struct gk20a_scale_profile,
			     qos_notify_block);
	struct gk20a_platform *platform = platform_get_drvdata(profile->pdev);
	struct gk20a *g = get_gk20a(profile->pdev);
	unsigned long freq;

	if (!platform->postscale)
		return NOTIFY_OK;

	/* get the frequency requirement. if devfreq is enabled, check if it
	 * has higher demand than qos */
	freq = gk20a_clk_round_rate(g, pm_qos_request(platform->qos_id));
	if (g->devfreq)
		freq = max(g->devfreq->previous_freq, freq);

	platform->postscale(profile->pdev, freq);

	return NOTIFY_OK;
}

/*
 * gk20a_scale_make_freq_table(profile)
 *
 * This function initialises the frequency table for the given device profile
 */

static int gk20a_scale_make_freq_table(struct gk20a_scale_profile *profile)
{
	struct gk20a *g = get_gk20a(profile->pdev);
	unsigned long *freqs;
	int num_freqs, err;

	/* make sure the clock is available */
	if (!gk20a_clk_get(g))
		return -ENOSYS;

	/* get gpu dvfs table */
	err = tegra_dvfs_get_freqs(clk_get_parent(g->clk.tegra_clk),
				   &freqs, &num_freqs);
	if (err)
		return -ENOSYS;

	profile->devfreq_profile.freq_table = (unsigned long *)freqs;
	profile->devfreq_profile.max_state = num_freqs;

	return 0;
}

/*
 * gk20a_scale_target(dev, *freq, flags)
 *
 * This function scales the clock
 */

static int gk20a_scale_target(struct device *dev, unsigned long *freq,
			      u32 flags)
{
	struct gk20a *g = get_gk20a(to_platform_device(dev));
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	unsigned long rounded_rate = gk20a_clk_round_rate(g, *freq);

	if (gk20a_clk_get_rate(g) == rounded_rate) {
		*freq = rounded_rate;
		return 0;
	}

	gk20a_clk_set_rate(g, rounded_rate);
	if (platform->postscale)
		platform->postscale(profile->pdev, rounded_rate);
	*freq = gk20a_clk_get_rate(g);

	return 0;
}

/*
 * update_load_estimate_gpmu(profile)
 *
 * Update load estimate using gpmu. The gpmu value is normalised
 * based on the time it was asked last time.
 */

static void update_load_estimate_gpmu(struct platform_device *pdev)
{
	struct gk20a *g = get_gk20a(pdev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	unsigned long dt;
	u32 busy_time;
	ktime_t t;

	t = ktime_get();
	dt = ktime_us_delta(t, profile->last_event_time);

	profile->dev_stat.total_time = dt;
	profile->last_event_time = t;
	gk20a_pmu_load_norm(g, &busy_time);
	profile->dev_stat.busy_time = (busy_time * dt) / 1000;
}

/*
 * gk20a_scale_suspend(pdev)
 *
 * This function informs devfreq of suspend
 */

void gk20a_scale_suspend(struct platform_device *pdev)
{
	struct gk20a *g = get_gk20a(pdev);
	struct devfreq *devfreq = g->devfreq;

	if (!devfreq)
		return;

	devfreq_suspend_device(devfreq);
}

/*
 * gk20a_scale_resume(pdev)
 *
 * This functions informs devfreq of resume
 */

void gk20a_scale_resume(struct platform_device *pdev)
{
	struct gk20a *g = get_gk20a(pdev);
	struct devfreq *devfreq = g->devfreq;

	if (!devfreq)
		return;

	devfreq_resume_device(devfreq);
}

/*
 * gk20a_scale_notify(pdev, busy)
 *
 * Calling this function informs that the device is idling (..or busy). This
 * data is used to estimate the current load
 */

static void gk20a_scale_notify(struct platform_device *pdev, bool busy)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a *g = get_gk20a(pdev);
	struct gk20a_scale_profile *profile = g->scale_profile;
	struct devfreq *devfreq = g->devfreq;

	/* inform edp about new constraint */
	if (platform->prescale)
		platform->prescale(pdev);

	/* Is the device profile initialised? */
	if (!(profile && devfreq))
		return;

	mutex_lock(&devfreq->lock);
	profile->dev_stat.busy = busy;
	update_devfreq(devfreq);
	mutex_unlock(&devfreq->lock);
}

void gk20a_scale_notify_idle(struct platform_device *pdev)
{
	gk20a_scale_notify(pdev, false);

}

void gk20a_scale_notify_busy(struct platform_device *pdev)
{
	gk20a_scale_notify(pdev, true);
}

/*
 * gk20a_scale_get_dev_status(dev, *stat)
 *
 * This function queries the current device status.
 */

static int gk20a_scale_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct gk20a *g = get_gk20a(to_platform_device(dev));
	struct gk20a_scale_profile *profile = g->scale_profile;

	/* Make sure there are correct values for the current frequency */
	profile->dev_stat.current_frequency = gk20a_clk_get_rate(g);

	/* Update load estimate */
	update_load_estimate_gpmu(to_platform_device(dev));

	/* Copy the contents of the current device status */
	*stat = profile->dev_stat;

	/* Finally, clear out the local values */
	profile->dev_stat.total_time = 0;
	profile->dev_stat.busy_time = 0;

	return 0;
}

/*
 * gk20a_scale_init(pdev)
 */

void gk20a_scale_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a *g = platform->g;
	struct gk20a_scale_profile *profile;
	int err;

	if (g->scale_profile)
		return;

	profile = kzalloc(sizeof(*profile), GFP_KERNEL);

	profile->pdev = pdev;
	profile->dev_stat.busy = false;

	/* Create frequency table */
	err = gk20a_scale_make_freq_table(profile);
	if (err || !profile->devfreq_profile.max_state)
		goto err_get_freqs;

	if (device_create_file(&pdev->dev, &dev_attr_load))
		goto err_create_sysfs_entry;

	/* Store device profile so we can access it if devfreq governor
	 * init needs that */
	g->scale_profile = profile;

	if (platform->devfreq_governor) {
		struct devfreq *devfreq;

		profile->devfreq_profile.initial_freq =
			profile->devfreq_profile.freq_table[0];
		profile->devfreq_profile.target = gk20a_scale_target;
		profile->devfreq_profile.get_dev_status =
			gk20a_scale_get_dev_status;

		devfreq = devfreq_add_device(&pdev->dev,
					&profile->devfreq_profile,
					platform->devfreq_governor, NULL);

		if (IS_ERR(devfreq))
			devfreq = NULL;

		g->devfreq = devfreq;
	}

	/* Should we register QoS callback for this device? */
	if (platform->qos_id < PM_QOS_NUM_CLASSES &&
	    platform->qos_id != PM_QOS_RESERVED &&
	    platform->postscale) {
		profile->qos_notify_block.notifier_call =
			&gk20a_scale_qos_notify;
		pm_qos_add_notifier(platform->qos_id,
				    &profile->qos_notify_block);
	}

	return;

err_get_freqs:
	device_remove_file(&pdev->dev, &dev_attr_load);
err_create_sysfs_entry:
	kfree(g->scale_profile);
	g->scale_profile = NULL;
}

/*
 * gk20a_scale_hw_init(dev)
 *
 * Initialize hardware portion of the device
 */

void gk20a_scale_hw_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;

	/* make sure that scaling has bee initialised */
	if (!profile)
		return;

	profile->dev_stat.total_time = 0;
	profile->last_event_time = ktime_get();
}
