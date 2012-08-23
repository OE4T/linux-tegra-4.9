/*
 * drivers/misc/therm_est.c
 *
 * Copyright (C) 2010-2012 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/therm_est.h>
#include <linux/thermal.h>
#include <linux/module.h>

struct therm_estimator {
	long cur_temp;
	long polling_period;
	struct workqueue_struct *workqueue;
	struct delayed_work therm_est_work;
	long toffset;
	int ntemp;
	int ndevs;
	struct therm_est_subdevice *devs;
	struct thermal_zone_device *thz;
	struct thermal_cooling_device *cdev;
	long trip_temp;
};

static void therm_est_work_func(struct work_struct *work)
{
	int i, j, index, sum = 0;
	long temp;
	struct delayed_work *dwork = container_of (work,
					struct delayed_work, work);
	struct therm_estimator *est = container_of(
					dwork,
					struct therm_estimator,
					therm_est_work);

	for (i = 0; i < est->ndevs; i++) {
		if (est->devs[i].get_temp(est->devs[i].dev_data, &temp))
			continue;
		est->devs[i].hist[(est->ntemp % HIST_LEN)] = temp;
	}

	for (i = 0; i < est->ndevs; i++) {
		for (j = 0; j < HIST_LEN; j++) {
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			sum += est->devs[i].hist[index] *
				est->devs[i].coeffs[j];
		}
	}

	est->cur_temp = sum / 100 + est->toffset;

	est->ntemp++;

	if (est->cur_temp >= est->trip_temp)
		if (est->thz && !est->thz->passive)
			thermal_zone_device_update(est->thz);

	queue_delayed_work(est->workqueue, &est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
}

static int therm_est_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	struct therm_estimator *est = thz->devdata;

	if (cdev == est->cdev)
		thermal_zone_bind_cooling_device(thz, 0, cdev);

	return 0;
}

static int therm_est_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	struct therm_estimator *est = thz->devdata;

	if (cdev == est->cdev)
		thermal_zone_unbind_cooling_device(thz, 0, cdev);

	return 0;
}

static int therm_est_get_trip_type(struct thermal_zone_device *thz,
					int trip,
					enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

static int therm_est_get_trip_temp(struct thermal_zone_device *thz,
					int trip,
					unsigned long *temp)
{
	struct therm_estimator *est = thz->devdata;

	*temp = est->trip_temp;

	return 0;
}

static int therm_est_get_temp(struct thermal_zone_device *thz,
				unsigned long *temp)
{
	struct therm_estimator *est = thz->devdata;
	*temp = est->cur_temp;
	return 0;
}

static struct thermal_zone_device_ops therm_est_ops = {
	.bind = therm_est_bind,
	.unbind = therm_est_unbind,
	.get_trip_type = therm_est_get_trip_type,
	.get_trip_temp = therm_est_get_trip_temp,
	.get_temp = therm_est_get_temp,
};

static int __devinit therm_est_probe(struct platform_device *pdev)
{
	int i, j;
	long temp;
	struct therm_estimator *est;
	struct therm_est_subdevice *dev;
	struct therm_est_data *data;

	est = kzalloc(sizeof(struct therm_estimator), GFP_KERNEL);
	if (IS_ERR_OR_NULL(est))
		return -ENOMEM;

	platform_set_drvdata(pdev, est);

	data = pdev->dev.platform_data;

	est->devs = data->devs;
	est->ndevs = data->ndevs;
	est->toffset = data->toffset;
	est->polling_period = data->polling_period;

	/* initialize history */
	for (i = 0; i < data->ndevs; i++) {
		dev = &est->devs[i];

		if (dev->get_temp(dev->dev_data, &temp))
			goto err;

		for (j = 0; j < HIST_LEN; j++)
			dev->hist[j] = temp;
	}

	est->workqueue = alloc_workqueue("therm_est",
				    WQ_HIGHPRI | WQ_UNBOUND, 1);
	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));

	est->cdev = data->cdev;
	est->trip_temp = data->trip_temp;

	est->thz = thermal_zone_device_register("therm_est",
					1,
					est,
					&therm_est_ops,
					data->tc1,
					data->tc2,
					data->passive_delay,
					0);
	if (IS_ERR_OR_NULL(est->thz))
		goto err;

	return 0;
err:
	kfree(est);
	return -EINVAL;
}

static int __devexit therm_est_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver therm_est_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "therm_est",
	},
	.probe  = therm_est_probe,
	.remove = __devexit_p(therm_est_remove),
};

static int __init therm_est_driver_init(void)
{
	return platform_driver_register(&therm_est_driver);
}
module_init(therm_est_driver_init);
