/*
 * include/linux/nct1008.h
 *
 * NCT1008, temperature monitoring device from ON Semiconductors
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _LINUX_NCT1008_H
#define _LINUX_NCT1008_H

#include <linux/types.h>

#include <mach/edp.h>

#define MAX_ZONES	16

struct nct1008_data;

struct nct1008_platform_data {
	bool supported_hwrev;
	bool ext_range;
	u8 conv_rate;
	u8 offset;
	u8 hysteresis;
	s8 shutdown_ext_limit;
	s8 shutdown_local_limit;
	s8 throttling_ext_limit;
	s8 thermal_zones[MAX_ZONES];
	u8 thermal_zones_sz;
	void (*alarm_fn)(bool raised);
	void (*probe_callback)(struct nct1008_data *);
};

struct nct1008_data {
	struct work_struct work;
	struct i2c_client *client;
	struct nct1008_platform_data plat_data;
	struct mutex mutex;
	struct dentry *dent;
	u8 config;
	s8 *limits;
	u8 limits_sz;
	void (*alarm_fn)(bool raised);
	struct regulator *nct_reg;
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	struct thermal_zone_device *thz;
#endif
	long current_lo_limit;
	long current_hi_limit;

	long shutdown_temp;
	void (*alert_func)(void *);
	void *alert_data;
};

#ifdef CONFIG_SENSORS_NCT1008
int nct1008_thermal_get_temp(struct nct1008_data *data, long *temp);
int nct1008_thermal_set_limits(struct nct1008_data *data,
				long lo_limit_milli,
				long hi_limit_milli);
int nct1008_thermal_set_alert(struct nct1008_data *data,
				void (*alert_func)(void *),
				void *alert_data);
int nct1008_thermal_set_shutdown_temp(struct nct1008_data *data,
					long shutdown_temp);
#else
static inline int nct1008_thermal_get_temp(struct nct1008_data *data,
						long *temp)
{ return -EINVAL; }
static inline int nct1008_thermal_set_limits(struct nct1008_data *data,
				long lo_limit_milli,
				long hi_limit_milli)
{ return -EINVAL; }
static inline int nct1008_thermal_set_alert(struct nct1008_data *data,
				void (*alert_func)(void *),
				void *alert_data)
{ return -EINVAL; }
static inline int nct1008_thermal_set_shutdown_temp(struct nct1008_data *data,
					long shutdown_temp)
{ return -EINVAL; }
#endif

#endif /* _LINUX_NCT1008_H */
