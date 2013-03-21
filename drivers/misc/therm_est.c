/*
 * drivers/misc/therm_est.c
 *
 * Copyright (c) 2010-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/hwmon-sysfs.h>
#include <linux/suspend.h>

struct therm_estimator {
	struct thermal_zone_device *thz;
	int num_trips;
	struct thermal_trip_info *trips;
	struct thermal_zone_params *tzp;

	struct workqueue_struct *workqueue;
	struct delayed_work therm_est_work;
	long cur_temp;
	long low_limit;
	long high_limit;
	int ntemp;
	long toffset;
	long polling_period;
	int tc1;
	int tc2;
	int ndevs;
	struct therm_est_subdevice *devs;

#ifdef CONFIG_PM
	struct notifier_block pm_nb;
#endif
};

static int therm_est_subdev_match(struct thermal_zone_device *thz, void *data)
{
	return strcmp((char *)data, thz->type) == 0;
}

static int therm_est_subdev_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, therm_est_subdev_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static void therm_est_update_limits(struct therm_estimator *est)
{
	const int MAX_HIGH_TEMP = 128000;
	long low_temp = 0, high_temp = MAX_HIGH_TEMP;
	long trip_temp, passive_low_temp = MAX_HIGH_TEMP;
	enum thermal_trip_type trip_type;
	struct thermal_trip_info *trip_state;
	int i;

	for (i = 0; i < est->num_trips; i++) {
		trip_state = &est->trips[i];
		est->thz->ops->get_trip_temp(est->thz, i, &trip_temp);
		est->thz->ops->get_trip_type(est->thz, i, &trip_type);

		if (!trip_state->tripped) { /* not tripped? update high */
			if (trip_temp < high_temp)
				high_temp = trip_temp;
		} else { /* tripped? update low */
			if (trip_type != THERMAL_TRIP_PASSIVE) {
				/* get highest ACTIVE */
				if (trip_temp > low_temp)
					low_temp = trip_temp;
			} else {
				/* get lowest PASSIVE */
				if (trip_temp < passive_low_temp)
					passive_low_temp = trip_temp;
			}
		}
	}

	if (passive_low_temp != MAX_HIGH_TEMP)
		low_temp = max(low_temp, passive_low_temp);

	est->low_limit = low_temp;
	est->high_limit = high_temp;
}

static void therm_est_work_func(struct work_struct *work)
{
	int i, j, index, sum = 0;
	long temp;
	struct delayed_work *dwork = container_of(work,
					struct delayed_work, work);
	struct therm_estimator *est = container_of(dwork,
					struct therm_estimator,
					therm_est_work);

	for (i = 0; i < est->ndevs; i++) {
		if (therm_est_subdev_get_temp(est->devs[i].dev_data, &temp))
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

	if (est->thz && ((est->cur_temp < est->low_limit) ||
			(est->cur_temp >= est->high_limit))) {
		thermal_zone_device_update(est->thz);
		therm_est_update_limits(est);
	}

	queue_delayed_work(est->workqueue, &est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
}

static int therm_est_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	struct therm_estimator *est = thz->devdata;
	struct thermal_trip_info *trip_state;
	int i;

	for (i = 0; i < est->num_trips; i++) {
		trip_state = &est->trips[i];
		if (trip_state->cdev_type &&
		    !strncmp(trip_state->cdev_type, cdev->type,
			     THERMAL_NAME_LENGTH))
			thermal_zone_bind_cooling_device(thz, i, cdev,
							 trip_state->upper,
							 trip_state->lower);
	}

	return 0;
}

static int therm_est_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	struct therm_estimator *est = thz->devdata;
	struct thermal_trip_info *trip_state;
	int i;

	for (i = 0; i < est->num_trips; i++) {
		trip_state = &est->trips[i];
		if (trip_state->cdev_type &&
		    !strncmp(trip_state->cdev_type, cdev->type,
			     THERMAL_NAME_LENGTH))
			thermal_zone_unbind_cooling_device(thz, i, cdev);
	}

	return 0;
}

static int therm_est_get_trip_type(struct thermal_zone_device *thz,
				   int trip, enum thermal_trip_type *type)
{
	struct therm_estimator *est = thz->devdata;

	*type = est->trips[trip].trip_type;
	return 0;
}

static int therm_est_get_trip_temp(struct thermal_zone_device *thz,
				   int trip, unsigned long *temp)
{
	struct therm_estimator *est = thz->devdata;
	struct thermal_trip_info *trip_state = &est->trips[trip];
	unsigned long trip_temp, zone_temp;

	trip_temp = trip_state->trip_temp;
	zone_temp = thz->temperature;

	if (zone_temp >= trip_temp) {
		trip_temp -= trip_state->hysteresis;
		trip_state->tripped = true;
	} else if (trip_state->tripped) {
		trip_temp -= trip_state->hysteresis;
		if (zone_temp < trip_temp)
			trip_state->tripped = false;
	}

	*temp = trip_temp;

	return 0;
}

static int therm_est_set_trip_temp(struct thermal_zone_device *thz,
				   int trip, unsigned long temp)
{
	struct therm_estimator *est = thz->devdata;

	est->trips[trip].trip_temp = temp;
	return 0;
}

static int therm_est_get_temp(struct thermal_zone_device *thz,
				unsigned long *temp)
{
	struct therm_estimator *est = thz->devdata;

	*temp = est->cur_temp;
	return 0;
}

static int therm_est_get_trend(struct thermal_zone_device *thz,
			       int trip, enum thermal_trend *trend)
{
	struct therm_estimator *est = thz->devdata;
	struct thermal_trip_info *trip_state = &est->trips[trip];
	long trip_temp;
	int new_trend;
	int cur_temp;

	thz->ops->get_trip_temp(thz, trip, &trip_temp);

	cur_temp = thz->temperature;
	new_trend = (est->tc1 * (cur_temp - thz->last_temperature)) +
		    (est->tc2 * (cur_temp - trip_temp));

	switch (trip_state->trip_type) {
	case THERMAL_TRIP_ACTIVE:
		/* aggressive active cooling */
		*trend = THERMAL_TREND_RAISING;
		break;
	case THERMAL_TRIP_PASSIVE:
		if (new_trend > 0)
			*trend = THERMAL_TREND_RAISING;
		else if (new_trend < 0)
			*trend = THERMAL_TREND_DROPPING;
		else
			*trend = THERMAL_TREND_STABLE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct thermal_zone_device_ops therm_est_ops = {
	.bind = therm_est_bind,
	.unbind = therm_est_unbind,
	.get_trip_type = therm_est_get_trip_type,
	.get_trip_temp = therm_est_get_trip_temp,
	.set_trip_temp = therm_est_set_trip_temp,
	.get_temp = therm_est_get_temp,
	.get_trend = therm_est_get_trend,
};

static ssize_t show_coeff(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	ssize_t len, total_len = 0;
	int i, j;
	for (i = 0; i < est->ndevs; i++) {
		len = snprintf(buf + total_len,
				PAGE_SIZE - total_len, "[%d]", i);
		total_len += len;
		for (j = 0; j < HIST_LEN; j++) {
			len = snprintf(buf + total_len,
					PAGE_SIZE - total_len, " %ld",
					est->devs[i].coeffs[j]);
			total_len += len;
		}
		len = snprintf(buf + total_len, PAGE_SIZE - total_len, "\n");
		total_len += len;
	}
	return strlen(buf);
}

static ssize_t set_coeff(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int devid, scount;
	long coeff[20];

	if (HIST_LEN > 20)
		return -EINVAL;

	scount = sscanf(buf, "[%d] %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld " \
			"%ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
			&devid,
			&coeff[0],
			&coeff[1],
			&coeff[2],
			&coeff[3],
			&coeff[4],
			&coeff[5],
			&coeff[6],
			&coeff[7],
			&coeff[8],
			&coeff[9],
			&coeff[10],
			&coeff[11],
			&coeff[12],
			&coeff[13],
			&coeff[14],
			&coeff[15],
			&coeff[16],
			&coeff[17],
			&coeff[18],
			&coeff[19]);

	if (scount != HIST_LEN + 1)
		return -1;

	if (devid < 0 || devid >= est->ndevs)
		return -EINVAL;

	/* This has obvious locking issues but don't worry about it */
	memcpy(est->devs[devid].coeffs, coeff, sizeof(coeff[0]) * HIST_LEN);

	return count;
}

static ssize_t show_offset(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%ld\n", est->toffset);
	return strlen(buf);
}

static ssize_t set_offset(struct device *dev,
				struct device_attribute *da,
				const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int offset;

	if (kstrtoint(buf, 0, &offset))
		return -EINVAL;

	est->toffset = offset;

	return count;
}

static ssize_t show_temps(struct device *dev,
				struct device_attribute *da,
				char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	ssize_t total_len = 0;
	int i, j;
	int index;

	/* This has obvious locking issues but don't worry about it */
	for (i = 0; i < est->ndevs; i++) {
		total_len += snprintf(buf + total_len,
					PAGE_SIZE - total_len, "[%d]", i);
		for (j = 0; j < HIST_LEN; j++) {
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			total_len += snprintf(buf + total_len,
						PAGE_SIZE - total_len, " %ld",
						est->devs[i].hist[index]);
		}
		total_len += snprintf(buf + total_len,
					PAGE_SIZE - total_len, "\n");
	}
	return strlen(buf);
}

static ssize_t show_tc1(struct device *dev,
			struct device_attribute *da,
			char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%d\n", est->tc1);
	return strlen(buf);
}

static ssize_t set_tc1(struct device *dev,
			struct device_attribute *da,
			const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int tc1;

	if (kstrtoint(buf, 0, &tc1))
		return -EINVAL;

	est->tc1 = tc1;

	return count;
}

static ssize_t show_tc2(struct device *dev,
			struct device_attribute *da,
			char *buf)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%d\n", est->tc2);
	return strlen(buf);
}

static ssize_t set_tc2(struct device *dev,
			struct device_attribute *da,
			const char *buf, size_t count)
{
	struct therm_estimator *est = dev_get_drvdata(dev);
	int tc2;

	if (kstrtoint(buf, 0, &tc2))
		return -EINVAL;

	est->tc2 = tc2;

	return count;
}

static struct sensor_device_attribute therm_est_nodes[] = {
	SENSOR_ATTR(coeff, S_IRUGO | S_IWUSR, show_coeff, set_coeff, 0),
	SENSOR_ATTR(offset, S_IRUGO | S_IWUSR, show_offset, set_offset, 0),
	SENSOR_ATTR(tc1, S_IRUGO | S_IWUSR, show_tc1, set_tc1, 0),
	SENSOR_ATTR(tc2, S_IRUGO | S_IWUSR, show_tc2, set_tc2, 0),
	SENSOR_ATTR(temps, S_IRUGO, show_temps, 0, 0),
};

static int therm_est_init_history(struct therm_estimator *est)
{
	int i, j;
	struct therm_est_subdevice *dev;
	long temp;

	for (i = 0; i < est->ndevs; i++) {
		dev = &est->devs[i];

		if (therm_est_subdev_get_temp(dev->dev_data, &temp))
			return -EINVAL;

		for (j = 0; j < HIST_LEN; j++)
			dev->hist[j] = temp;
	}

	return 0;
}

#ifdef CONFIG_PM
static int therm_est_pm_notify(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct therm_estimator *est = container_of(
					nb,
					struct therm_estimator,
					pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&est->therm_est_work);
		break;
	case PM_POST_SUSPEND:
		est->low_limit = 0;
		est->high_limit = 0;
		therm_est_init_history(est);
		queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
		break;
	}

	return NOTIFY_OK;
}
#endif

static int __devinit therm_est_probe(struct platform_device *pdev)
{
	int i;
	struct therm_estimator *est;
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
	est->tc1 = data->tc1;
	est->tc2 = data->tc2;

	/* initialize history */
	therm_est_init_history(est);

	est->workqueue = alloc_workqueue(dev_name(&pdev->dev),
				    WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!est->workqueue)
		goto err;

	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));

	est->num_trips = data->num_trips;
	est->trips = data->trips;
	est->tzp = data->tzp;

	est->thz = thermal_zone_device_register(dev_name(&pdev->dev),
						est->num_trips,
						(1 << est->num_trips) - 1,
						est,
						&therm_est_ops,
						est->tzp,
						data->passive_delay,
						0);
	if (IS_ERR_OR_NULL(est->thz))
		goto err;

	for (i = 0; i < ARRAY_SIZE(therm_est_nodes); i++)
		device_create_file(&pdev->dev, &therm_est_nodes[i].dev_attr);

#ifdef CONFIG_PM
	est->pm_nb.notifier_call = therm_est_pm_notify,
	register_pm_notifier(&est->pm_nb);
#endif

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
