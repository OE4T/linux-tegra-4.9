/*
 * drivers/thermal/continuous_thermal_gov.c
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/pwm.h>
#include <trace/events/thermal.h>
#include <linux/platform_data/pwm_fan.h>

#include "thermal_core.h"

#define DRV_NAME	"continuous_therm_gov"

#define IIR_POWER (7)
#define UPPER_WIDTH (5000)
#define LOWER_WIDTH (15000)

#define QPOINT (10000)
#define IIR_GAIN_QP (1000)
#define IIR_MIN (IIR_GAIN_QP / 100)
#define IIR_MAX (IIR_GAIN_QP * 1)

#define MAX_ACTIVE_STATES	10

#define ABS(x, y) ((x > y)?(x - y):(y - x))

int (*fetch_trip_pwm)(struct thermal_cooling_device *cdev, int trip);

/*function defined to fetch active pwm table to caculate slope*/
/*Notice that the register action must be done to get the pwm table*/
void register_fetch_pwm_func(int (*func)(struct thermal_cooling_device *cdev, int trip))
{
	fetch_trip_pwm = func;
}
EXPORT_SYMBOL(register_fetch_pwm_func);

struct continuous_thermal_gov_params {
	int iir_power;
	int iir_min;
	int iir_max;
	int iir_gain_qp;
	int iir_upper_width;
	int iir_lower_width;
	long long iir_gain;
};

static struct continuous_thermal_gov_params pm_default = {
	.iir_power		    = IIR_POWER,
	.iir_min		    = IIR_MIN,
	.iir_max		    = IIR_MAX,
	.iir_gain_qp		= IIR_GAIN_QP,
	.iir_upper_width	= UPPER_WIDTH,
	.iir_lower_width	= LOWER_WIDTH,
	.iir_gain = 0,
};

struct continuous_thermal_gov_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct attribute *attr,
			char *buf);
	ssize_t (*store)(struct kobject *kobj, struct attribute *attr,
			 const char *buf, size_t count);
};

struct continuous_thermal_governor {
	struct kobject kobj;
	struct continuous_thermal_gov_params pm;

	int trip;
	int trip_level;
	int target_pwm;
	int prelta;
	int newlta;

	int pwm_tbl[MAX_ACTIVE_STATES];
	s32 active_trip_temps[MAX_ACTIVE_STATES];
	s32 active_hysteresis[MAX_ACTIVE_STATES];

	/* params used to debug */
	int rawtemp;
	int cur_width;
	long delttemp;
	long long iir_gain;
};

#define tz_to_gov(t)		\
	(t->governor_data)

#define kobj_to_gov(k)		\
	container_of(k, struct continuous_thermal_governor, kobj)

#define attr_to_gov_attr(a)	\
	container_of(a, struct continuous_thermal_gov_attribute, attr)

#define kobjp_to_kobj(kp)		\
	container_of(kp, struct kobject, parent)


static long long multi_gain(int iirconst, int delta, int width,  int iir_power)
{
	int index;
	long long multi = iirconst * delta / width;

	for (index = 0; index < iir_power - 1; index++)
		multi = multi * delta / width;

	return multi;
}

static ssize_t iir_power_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	return sprintf(buf, "%d\n", gov->pm.iir_power);
}

static ssize_t iir_power_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);
	int val;

	if (!gov)
		return -ENODEV;

	if (!sscanf(buf, "%d\n", &val) || (val < 0))
		return -EINVAL;

	gov->pm.iir_power = val;
	return count;
}

static struct continuous_thermal_gov_attribute iir_power_attr =
	__ATTR(iir_power, 0644, iir_power_show, iir_power_store);

static ssize_t iir_min_show(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	return sprintf(buf, "%d\n", gov->pm.iir_min);
}

static ssize_t iir_min_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);
	int val;

	if (!gov)
		return -ENODEV;

	if (!sscanf(buf, "%d\n", &val) || (val < 0))
		return -EINVAL;

	gov->pm.iir_min = val;
	return count;
}

static struct continuous_thermal_gov_attribute iir_min_attr =
	__ATTR(iir_min, 0644, iir_min_show, iir_min_store);

static ssize_t iir_width_show(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	return sprintf(buf, "%d %d\n", gov->pm.iir_upper_width,
			gov->pm.iir_lower_width);
}

static ssize_t iir_width_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);
	int upper, lower;

	if (!gov)
		return -ENODEV;

	if (!sscanf(buf, "%d %d\n", &upper, &lower) || (upper <= 0) || (lower <= 0))
		return -EINVAL;

	gov->pm.iir_upper_width = upper;
	gov->pm.iir_lower_width = lower;
	return count;
}

static struct continuous_thermal_gov_attribute iir_width_attr =
	__ATTR(iir_width, 0644, iir_width_show, iir_width_store);

static ssize_t gov_param_show(struct kobject *kobj, struct attribute *attr,
					char *buf)
{
	int ret = 0;
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	ret += sprintf(buf + ret, "prevLTA   raw   newSTA  raw-prevLTA gain width  pwm\n");
	ret += sprintf(buf + ret, "%7d %7d %7d %11d %4lld %5d %5d\n",
				gov->prelta, gov->rawtemp, gov->newlta,
				gov->rawtemp - gov->prelta, gov->iir_gain,
				gov->cur_width, gov->target_pwm);

	return ret;
}

static struct continuous_thermal_gov_attribute gov_param_attr =
	__ATTR(gov_param, 0444, gov_param_show, NULL);

static ssize_t trip_temps_show(struct kobject *kobj, struct attribute *attr,
					char *buf)
{
	int ret = 0, trip;
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	for (trip = 0; trip < gov->trip; trip++) {
		ret += sprintf(buf + ret, "[%d]:%d ",
			trip, gov->active_trip_temps[trip]);
	}
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static struct continuous_thermal_gov_attribute trip_temps_attr =
	__ATTR(trip_temps, 0444, trip_temps_show, NULL);

static ssize_t trip_hyst_show(struct kobject *kobj, struct attribute *attr,
			   char *buf)
{
	int ret = 0, trip;
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	for (trip = 0; trip < gov->trip; trip++) {
		ret += sprintf(buf + ret, "[%d]:%d ",
				trip, gov->active_hysteresis[trip]);
	}
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static struct continuous_thermal_gov_attribute trip_hyst_attr =
	__ATTR(trip_hyst, 0444, trip_hyst_show, NULL);

static ssize_t pwm_table_show(struct kobject *kobj, struct attribute *attr,
			   char *buf)
{
	int ret = 0, trip;
	struct continuous_thermal_governor *gov = kobj_to_gov(kobj);

	if (!gov)
		return -ENODEV;

	for (trip = 0; trip < gov->trip; trip++) {
		ret += sprintf(buf + ret, "[%d]:%d ",
				trip, gov->pwm_tbl[trip]);
	}
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static struct continuous_thermal_gov_attribute pwm_table_attr =
	__ATTR(pwm_table, 0444, pwm_table_show, NULL);

static struct attribute *continuous_thermal_gov_default_attrs[] = {
	&iir_power_attr.attr,
	&iir_min_attr.attr,
	&iir_width_attr.attr,
	&gov_param_attr.attr,
	&trip_temps_attr.attr,
	&trip_hyst_attr.attr,
	&pwm_table_attr.attr,
	NULL,
};

static ssize_t continuous_thermal_gov_show(struct kobject *kobj,
				    struct attribute *attr, char *buf)
{
	struct continuous_thermal_gov_attribute *gov_attr = attr_to_gov_attr(attr);

	if (!gov_attr->show)
		return -EIO;

	return gov_attr->show(kobj, attr, buf);
}

static ssize_t continuous_thermal_gov_store(struct kobject *kobj,
				struct attribute *attr, const char *buf,
				size_t len)
{
	struct continuous_thermal_gov_attribute *gov_attr = attr_to_gov_attr(attr);

	if (!gov_attr->store)
		return -EIO;

	return gov_attr->store(kobj, attr, buf, len);
}

static const struct sysfs_ops continuous_thermal_gov_sysfs_ops = {
	.show	= continuous_thermal_gov_show,
	.store	= continuous_thermal_gov_store,
};

static struct kobj_type continuous_thermal_gov_ktype = {
	.default_attrs	= continuous_thermal_gov_default_attrs,
	.sysfs_ops	= &continuous_thermal_gov_sysfs_ops,
};

static int continuous_thermal_gov_bind(struct thermal_zone_device *tz)
{
	struct continuous_thermal_governor *gov;
	struct continuous_thermal_gov_params *params;
	int ret, trip;

	gov = kzalloc(sizeof(struct continuous_thermal_governor), GFP_KERNEL);
	if (!gov) {
		dev_err(&tz->device, "%s: Can't alloc governor data\n",
			DRV_NAME);
		return -ENOMEM;
	}

	ret = kobject_init_and_add(&gov->kobj, &continuous_thermal_gov_ktype,
				&tz->device.kobj, DRV_NAME);
	if (ret) {
		dev_err(&tz->device, "%s: Can't init kobject\n", DRV_NAME);
		kobject_put(&gov->kobj);
		kfree(gov);
		return ret;
	}

	params = (struct continuous_thermal_gov_params *)tz->tzp->governor_params;
	gov->pm = params ? *params : pm_default;
	if (params)
		pr_info("%s: DTB IIR params used\n", DRV_NAME);
	else
		pr_warn("%s: Default IIR params used\n", DRV_NAME);

	if (!gov->pm.iir_upper_width)
		gov->pm.iir_upper_width = UPPER_WIDTH;
	if (!gov->pm.iir_lower_width)
		gov->pm.iir_upper_width = LOWER_WIDTH;

	gov->trip = tz->trips;

	if (tz->ops && tz->ops->get_trip_temp && tz->ops->get_trip_hyst) {
		for (trip = 0; trip < tz->trips; trip++) {
			tz->ops->get_trip_temp(tz, trip, &gov->active_trip_temps[trip]);
			tz->ops->get_trip_hyst(tz, trip, &gov->active_hysteresis[trip]);
		}
	} else
		dev_err(&tz->device, "%s: Can't get trip temp and hyst data\n", DRV_NAME);

	tz_to_gov(tz) = gov;
	pr_info("GOV gov_bind tz:%s\n", tz->type);

	return 0;
}

static void continuous_thermal_gov_unbind(struct thermal_zone_device *tz)
{
	struct continuous_thermal_governor *gov = tz_to_gov(tz);

	if (!gov)
		return;

	kobject_put(&gov->kobj);
	kfree(gov);
}

static long continuous_thermal_gov_get_lta(struct thermal_zone_device *tz,
					struct continuous_thermal_governor *gov)
{
	int rawtemp, width;
	long delttemp = 0, lta = 0;
	long long iir_gain = 0;

	tz->ops->get_temp(tz, &rawtemp);
	gov->rawtemp = rawtemp;
	gov->prelta = gov->newlta;

	delttemp = ABS(gov->prelta, rawtemp);
	gov->delttemp = delttemp;

	if (gov->prelta - rawtemp < 0)
		width = gov->pm.iir_upper_width;
	else
		width = gov->pm.iir_lower_width;

	iir_gain = gov->pm.iir_min + multi_gain((gov->pm.iir_max - gov->pm.iir_min),
				delttemp, width, gov->pm.iir_power);
	if (iir_gain > gov->pm.iir_max) {
		pr_debug("iir_gain:%lld iir_min:%d iir_max:%d prelta:%d rawtemp:%d newlta:%d delta:%ld width:%d\n",
				iir_gain, gov->pm.iir_min, gov->pm.iir_max, gov->prelta, rawtemp, gov->newlta, delttemp, width);
		iir_gain = gov->pm.iir_max;
	}

	lta = gov->prelta + iir_gain * (rawtemp - gov->prelta) / gov->pm.iir_gain_qp;

	//check error
	if ((gov->delttemp >= 100) && (gov->prelta == lta)) {
		pr_err("GOV delttemp:%ld is big enough, rawtemp:%d gain:%lld; newlta:%ld == prelta:%d is error, should check this\n",
				gov->delttemp, gov->rawtemp, gov->iir_gain, lta, gov->prelta);
	}

	gov->newlta = lta;
	gov->cur_width = width;
	gov->iir_gain = iir_gain;

	return lta;
}

static unsigned long
continuous_thermal_gov_get_target_pwm(struct thermal_zone_device *tz,
				struct continuous_thermal_governor *gov,
				int index, long lta)
{
	long long m, b;
	unsigned long target_pwm = 0;

	m = (gov->pwm_tbl[index] - gov->pwm_tbl[index - 1]) * QPOINT /
		(gov->active_trip_temps[index] - gov->active_trip_temps[index - 1]);
	b = gov->pwm_tbl[index] * QPOINT -
		m * gov->active_trip_temps[index];

	pr_debug("GOV m:%lld b:%lld iir_gain:%lld, delttemp:%ld\n"
			"fan pwm[%d]:%d [%d]:%d \n"
			"trip temp[%d]:%d [%d]:%d\n"
			"prev lta:%d cur lta: %ld\n",
			m, b, gov->iir_gain, gov->delttemp,
			index, gov->pwm_tbl[index],
			index - 1, gov->pwm_tbl[index - 1],
			index, gov->active_trip_temps[index],
			index - 1, gov->active_trip_temps[index - 1],
			gov->prelta, lta);

	//check error
	if ((lta < gov->active_trip_temps[index - 1])
		|| (lta > gov->active_trip_temps[index])) {
		pr_err("GOV newLTA:%ld not in scope [%d, %d)\n", lta,
			gov->active_trip_temps[index - 1],
			gov->active_trip_temps[index]);
	}

	target_pwm = (int)(m * lta + b) / QPOINT;

	/*"2" means the actual index begins at 2*/
	for (index = 2; index < tz->trips; index++) {
		if (lta == gov->active_trip_temps[index]) {
			if (!gov->active_hysteresis[index])
				target_pwm = gov->pwm_tbl[index];
			break;
		}
	}

	return target_pwm;
}

static unsigned long
continuous_thermal_gov_get_target(struct thermal_zone_device *tz,
					struct thermal_cooling_device *cdev,
					enum thermal_trip_type trip_type,
					int trip_temp, int hyst, int trip)
{
	struct continuous_thermal_governor *gov = tz_to_gov(tz);
	static int last_lta, index;
	static bool is_fan_on = false;
	int level = 0, count;
	long lta = 0;
	unsigned long target_pwm = 0;

	if (!gov || !trip)
		return -1;

	gov->active_trip_temps[trip] = trip_temp; /*trip_*_temp updated manually*/
	gov->active_hysteresis[trip] = hyst;
	if (fetch_trip_pwm)
		gov->pwm_tbl[trip] = fetch_trip_pwm(cdev, trip);
	else {
		dev_err(&tz->device, "%s: Can't get cdev pwm table\n", DRV_NAME);
		return -1;
	}

	for (count = 1; count < tz->trips; count++) {
		if (!gov->active_trip_temps[count]) {
			dev_err(&tz->device, "%s: trip temp[%d] is %d\n",
					DRV_NAME, count, gov->active_trip_temps[count]);
			return -1;
		}
	}

	if (!last_lta) {
		lta = continuous_thermal_gov_get_lta(tz, gov);
		last_lta  = lta;
	} else {
		lta = last_lta;
	}

	//fan table lookup
	if (!is_fan_on) {
		if (hyst && (lta >= trip_temp))
			is_fan_on = !!trip;
	} else if (!hyst
		&& (lta < trip_temp)
		&& !index) {
		index = trip;
	}

	if (trip < tz->trips - 1)
		return -1;

	/*"level" means cooling level of cdev*/
	/*"index" means index of active pwm table*/
	/*"- 2" means the actual index begins at 2*/
	level = index - 2;

	if (level >= tz->trips)
		level = tz->trips - 1;
	else if (level < 0) {
		level = 0;
	}

	last_lta = 0;

	//check if doing step based fan control, or smooth ramp
	if (level) {//level > 0 means fan is ruuning.
		target_pwm = continuous_thermal_gov_get_target_pwm(tz, gov, index, lta);

	} else if (lta < (gov->active_trip_temps[1] - gov->active_hysteresis[1])) {
		target_pwm = 0;
		level = 0;
	} else if (gov->trip_level) {//(trip_temp[1] - hyst[1]) < lta < trip_temp[1]
		target_pwm = gov->pwm_tbl[1];
		level = 1;
	}

	index = 0;
	is_fan_on = !!level;
	gov->trip_level = level;
	gov->target_pwm = target_pwm;

	pr_debug("%7d %7d %7d %11d %4lld %5d %5d\n",
			gov->prelta, gov->rawtemp, gov->newlta,
			gov->rawtemp - gov->prelta, gov->iir_gain,
			gov->cur_width, gov->target_pwm);

	return target_pwm;
}

static int continuous_thermal_gov_throttle(struct thermal_zone_device *tz, int trip)
{
	struct thermal_instance *instance;
	enum thermal_trip_type trip_type;
	int trip_temp, hyst = 0;
	unsigned long target;
	tz->ops->get_trip_type(tz, trip, &trip_type);
	tz->ops->get_trip_temp(tz, trip, &trip_temp);
	if (tz->ops->get_trip_hyst)
		tz->ops->get_trip_hyst(tz, trip, &hyst);
	else {
		dev_err(&tz->device, "%s: Can't get trip hyst data\n", DRV_NAME);
		return 0;
	}

	mutex_lock(&tz->lock);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip != trip)
			continue;

		target = continuous_thermal_gov_get_target(tz, instance->cdev,
				trip_type, trip_temp, hyst, trip);

		if ((instance->target == target) || (target == -1))
			continue;

		instance->target = target;
		instance->cdev->updated = false;
	}

	list_for_each_entry(instance, &tz->thermal_instances, tz_node)
		thermal_cdev_update(instance->cdev);

	mutex_unlock(&tz->lock);

	return 0;
}

static int continuous_thermal_gov_of_parse(struct thermal_zone_params *tzp,
					struct device_node *np)
{
	u32 val;
	int of_err = 0;
	struct continuous_thermal_gov_params *gpm;

	gpm = kzalloc(sizeof(struct continuous_thermal_gov_params), GFP_KERNEL);
	if (!gpm)
		return -ENOMEM;

	*gpm = pm_default;

	if (!(of_err |= of_property_read_u32(np, "iir_power", &val)))
		gpm->iir_power = val;
	if (!(of_err |= of_property_read_u32(np, "iir_min", &val)))
		gpm->iir_min = val;
	if (!(of_err |= of_property_read_u32(np, "iir_max", &val)))
		gpm->iir_max = val;
	if (!(of_err |= of_property_read_u32(np, "iir_gain_qp", &val)))
		gpm->iir_gain_qp = val;
	if (!(of_err |= of_property_read_u32(np, "iir_upper_width", &val)))
		gpm->iir_upper_width = val;
	if (!(of_err |= of_property_read_u32(np, "iir_lower_width", &val)))
		gpm->iir_lower_width = val;

	if (of_err)
		pr_warn("%s: Can't init iir parameters\n", DRV_NAME);

	pr_debug("GOV of parse\n");
	tzp->governor_params = gpm;
	return 0;
}

static struct thermal_governor continuous_thermal_gov = {
	.name		= DRV_NAME,
	.bind_to_tz	= continuous_thermal_gov_bind,
	.unbind_from_tz	= continuous_thermal_gov_unbind,
	.throttle	= continuous_thermal_gov_throttle,
	.of_parse	= continuous_thermal_gov_of_parse,
};

int thermal_gov_continuous_register(void)
{
	return thermal_register_governor(&continuous_thermal_gov);
}

void thermal_gov_continuous_unregister(void)
{
	thermal_unregister_governor(&continuous_thermal_gov);
}
