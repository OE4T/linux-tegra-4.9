/*
 * drivers/misc/therm_est.c
 *
 * Copyright (c) 2010-2014, NVIDIA CORPORATION.  All rights reserved.
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

#define	DEFAULT_TSKIN			25000 /* default tskin in mC */

struct therm_estimator {
	struct thermal_zone_device *thz;

	struct thermal_cooling_device *cdev; /* activation device */
	struct workqueue_struct *workqueue;
	struct delayed_work therm_est_work;
	long cur_temp;
	long low_limit;
	long high_limit;
	int ntemp;
	long toffset;
	long polling_period;
	int polling_enabled;
	int tc1;
	int tc2;
	int ndevs;
	struct therm_est_subdevice *devs;
	bool *tripped_info;

	int use_activator;
#ifdef CONFIG_PM
	struct notifier_block pm_nb;
#endif
};

static int therm_est_subdev_match(struct thermal_zone_device *thz, void *data)
{
	return strcmp((char *)data, thz->type) == 0;
}

static int therm_est_subdev_get_temp(struct thermal_zone_device *thz,
					long *temp)
{
	if (!thz || !thz->ops->get_temp || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static void therm_est_update_limits(struct therm_estimator *est)
{
	const int MAX_HIGH_TEMP = 128000;
	long low_temp = 0, high_temp = MAX_HIGH_TEMP;
	long trip_temp, passive_low_temp = MAX_HIGH_TEMP;
	enum thermal_trip_type trip_type;
	long hysteresis, zone_temp;
	int i;

	zone_temp = est->thz->temperature;
	for (i = 0; i < est->thz->trips; i++) {
		est->thz->ops->get_trip_temp(est->thz, i, &trip_temp);
		est->thz->ops->get_trip_hyst(est->thz, i, &hysteresis);
		est->thz->ops->get_trip_type(est->thz, i, &trip_type);

		if (zone_temp >= trip_temp) {
			trip_temp -= hysteresis;
			est->tripped_info[i] = true;
		} else if (est->tripped_info[i]) {
			trip_temp -= hysteresis;
			if (zone_temp < trip_temp)
				est->tripped_info[i] = false;
		}

		if (!est->tripped_info[i]) { /* not tripped? update high */
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
		if (therm_est_subdev_get_temp(est->devs[i].sub_thz, &temp))
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

	if (est->polling_enabled > 0 || !est->use_activator) {
		queue_delayed_work(est->workqueue, &est->therm_est_work,
			msecs_to_jiffies(est->polling_period));
	}
}

static int therm_est_get_temp(void *of_data, long *temp)
{
	struct therm_estimator *est = (struct therm_estimator *)of_data;

	*temp = est->cur_temp;
	return 0;
}

static int therm_est_get_trend(void *of_data, long *trend)
{
	struct therm_estimator *est = (struct therm_estimator *)of_data;

	if (est->thz->temperature > est->thz->last_temperature + 100)
		*trend = THERMAL_TREND_RAISING;
	else if (est->thz->temperature < est->thz->last_temperature - 100)
		*trend = THERMAL_TREND_DROPPING;
	else
		*trend = THERMAL_TREND_STABLE;

	return 0;
}

static int therm_est_init_history(struct therm_estimator *est)
{
	int i, j;
	struct therm_est_subdevice *dev;
	long temp;

	for (i = 0; i < est->ndevs; i++) {
		dev = &est->devs[i];

		if (therm_est_subdev_get_temp(dev->sub_thz, &temp))
			return -EINVAL;

		for (j = 0; j < HIST_LEN; j++)
			dev->hist[j] = temp;
	}

	return 0;
}

static int therm_est_polling(struct therm_estimator *est,
				int polling)
{
	est->polling_enabled = polling > 0;

	if (est->polling_enabled > 0) {
		est->low_limit = 0;
		est->high_limit = 0;
		therm_est_init_history(est);
		queue_delayed_work(est->workqueue,
			&est->therm_est_work,
			msecs_to_jiffies(est->polling_period));
	} else {
		cancel_delayed_work_sync(&est->therm_est_work);
		est->cur_temp = DEFAULT_TSKIN;
	}
	return 0;
}

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

static int
thermal_est_activation_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *max_state)
{
	*max_state = 1;
	return 0;
}

static int
thermal_est_activation_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *cur_state)
{
	struct therm_estimator *est = cdev->devdata;
	*cur_state = est->polling_enabled;
	return 0;
}

static int
thermal_est_activation_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long cur_state)
{
	struct therm_estimator *est = cdev->devdata;
	if (est->use_activator)
		therm_est_polling(est, cur_state > 0);

	return 0;
}

static struct thermal_cooling_device_ops thermal_est_activation_device_ops = {
	.get_max_state = thermal_est_activation_get_max_state,
	.get_cur_state = thermal_est_activation_get_cur_state,
	.set_cur_state = thermal_est_activation_set_cur_state,
};

struct thermal_cooling_device *thermal_est_activation_device_register(
						struct therm_estimator *est,
						char *type)
{
	struct thermal_cooling_device *cdev;

	cdev = thermal_cooling_device_register(
		type,
		est,
		&thermal_est_activation_device_ops);

	if (IS_ERR(cdev))
		return NULL;

	pr_debug("Therm_est: Cooling-device REGISTERED\n");

	return cdev;
}

static int __parse_dt_subdev(struct device_node *np,
			     struct therm_est_subdevice *subdev)
{
	const char *str;
	char *sbegin;
	int i = 0;
	int ret;

	subdev->dev_data = (void *)of_get_property(np, "dev-data", NULL);
	if (!subdev->dev_data)
		return -ENODATA;

	ret = of_property_read_string(np, "coeffs", &str);
	if (ret < 0)
		return ret;

	while (str && (i < HIST_LEN)) {
		str = skip_spaces(str);
		sbegin = strsep((char **)&str, " ");
		if (!sbegin || (kstrtol((const char *)sbegin, 10,
				&subdev->coeffs[i++]) < 0))
			break;
	}
	if (i != HIST_LEN)
		return -EINVAL;

	return 0;
}

static struct therm_est_data *therm_est_get_pdata(struct device *dev)
{
	struct therm_est_data *data;
	struct device_node *np;
	struct device_node *ch;
	u32 val;
	int num_subdev;
	int ret;

	np = dev->of_node;
	if (!np)
		return dev->platform_data;

	data = devm_kzalloc(dev, sizeof(struct therm_est_data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "toffset", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	data->toffset = val;

	ret = of_property_read_u32(np, "polling-period", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	data->polling_period = val;

	ret = of_property_read_u32(np, "tc1", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	data->tc1 = val;

	ret = of_property_read_u32(np, "tc2", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	data->tc2 = val;

	ret = of_property_read_u32(np, "use_activator", &val);
	if (ret < 0)
		return ERR_PTR(ret);
	data->use_activator = val;

	num_subdev = 0;
	ch = np;
	while ((ch = of_find_node_by_type(ch, "therm-est-subdev")))
		num_subdev++;

	/* subdevices are must required data. */
	if (num_subdev == 0)
		return ERR_PTR(-ENOENT);

	data->devs = devm_kzalloc(dev,
			sizeof(struct therm_est_subdevice) * num_subdev,
			GFP_KERNEL);
	if (!data->devs)
		return ERR_PTR(-ENOMEM);

	num_subdev = 0;
	ch = np;
	while ((ch = of_find_node_by_type(ch, "therm-est-subdev"))) {
		ret = __parse_dt_subdev(ch, &data->devs[num_subdev++]);
		if (ret < 0)
			return ERR_PTR(ret);
		}

	data->ndevs = num_subdev;

	return data;
}

static struct thermal_zone_of_device_ops sops = {
	.get_temp = therm_est_get_temp,
	.get_trend = therm_est_get_trend,
};

static int therm_est_probe(struct platform_device *pdev)
{
	int i, ret;
	struct therm_estimator *est;
	struct therm_est_data *data;
	struct thermal_zone_device *thz;

	est = kzalloc(sizeof(struct therm_estimator), GFP_KERNEL);
	if (IS_ERR_OR_NULL(est))
		return -ENOMEM;

	platform_set_drvdata(pdev, est);

	data = pdev->dev.platform_data;
	if (!data)
		data = therm_est_get_pdata(&pdev->dev);

	for (i = 0; i < data->ndevs; i++) {
		thz = thermal_zone_device_find(data->devs[i].dev_data,
							therm_est_subdev_match);
		if (!thz)
			goto err;
		data->devs[i].sub_thz = thz;
	}

	est->devs = data->devs;
	est->ndevs = data->ndevs;
	est->toffset = data->toffset;
	est->polling_period = data->polling_period;
	est->polling_enabled = 0; /* By default polling is switched off */
	est->tc1 = data->tc1;
	est->tc2 = data->tc2;
	est->cur_temp = DEFAULT_TSKIN;
	est->use_activator = data->use_activator;

	/* initialize history */
	therm_est_init_history(est);

	est->workqueue = alloc_workqueue(dev_name(&pdev->dev),
				    WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!est->workqueue)
		goto err;

	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	est->cdev = thermal_est_activation_device_register(est,
							"therm_est_activ");

	est->thz = thermal_zone_of_sensor_register(&pdev->dev, 0, est, &sops);
	if (IS_ERR(est->thz)) {
		ret = PTR_ERR(est->thz);
		dev_err(&pdev->dev,
			"Device can not register as thermal sensor: %d\n", ret);
		goto err;
	}

	est->tripped_info = devm_kzalloc(&pdev->dev,
				sizeof(bool) * est->thz->trips,
				GFP_KERNEL);
	if (IS_ERR_OR_NULL(est->thz))
		goto err;

	for (i = 0; i < ARRAY_SIZE(therm_est_nodes); i++)
		device_create_file(&pdev->dev, &therm_est_nodes[i].dev_attr);

#ifdef CONFIG_PM
	est->pm_nb.notifier_call = therm_est_pm_notify,
	register_pm_notifier(&est->pm_nb);
#endif

	if (!est->use_activator)
		queue_delayed_work(est->workqueue, &est->therm_est_work,
			msecs_to_jiffies(est->polling_period));

	return 0;

err:
	if (est->workqueue)
		destroy_workqueue(est->workqueue);
	kfree(est);
	return -EINVAL;
}

static int therm_est_remove(struct platform_device *pdev)
{
	struct therm_estimator *est = platform_get_drvdata(pdev);
	int i;

	cancel_delayed_work_sync(&est->therm_est_work);

#ifdef CONFIG_PM
	unregister_pm_notifier(&est->pm_nb);
#endif
	for (i = 0; i < ARRAY_SIZE(therm_est_nodes); i++)
		device_remove_file(&pdev->dev, &therm_est_nodes[i].dev_attr);
	thermal_cooling_device_unregister(est->cdev);
	kfree(est->thz);
	destroy_workqueue(est->workqueue);
	kfree(est);
	return 0;
}

static void therm_est_shutdown(struct platform_device *pdev)
{
	struct therm_estimator *est = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&est->therm_est_work);
	thermal_cooling_device_unregister(est->cdev);
}

static struct of_device_id therm_est_of_match[] = {
	{ .compatible = "nvidia,therm-est", },
	{ },
};
static struct platform_driver therm_est_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "therm_est",
		.of_match_table = of_match_ptr(therm_est_of_match),
	},
	.probe  = therm_est_probe,
	.remove = therm_est_remove,
	.shutdown = therm_est_shutdown,
};

static int __init therm_est_driver_init(void)
{
	return platform_driver_register(&therm_est_driver);
}
device_initcall_sync(therm_est_driver_init);
