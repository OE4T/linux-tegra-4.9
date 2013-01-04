/*
 * include/linux/therm_est.h
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#ifndef _LINUX_THERM_EST_H
#define _LINUX_THERM_EST_H

#include <linux/workqueue.h>

#define HIST_LEN (20)

#define MAX_ACTIVE_STATES 10

struct therm_est_subdevice {
	void *dev_data;
	int (*get_temp)(void *, long *);
	long coeffs[HIST_LEN];
	long hist[HIST_LEN];
};

struct therm_est_data {
	/* trip point info : there's only 1 trip point */
	char *cdev_type; /* cooling device for this trip */
	long trip_temp;

	/* zone parameters */
	long toffset;
	long polling_period;
	int passive_delay;
	int ndevs;
	int tc1;
	int tc2;
	struct therm_est_subdevice devs[];
};

struct therm_fan_est_subdevice {
	void *dev_data;
	int (*get_temp)(void *, long *);
	long coeffs[HIST_LEN];
	long hist[HIST_LEN];
};

struct therm_fan_est_data {
	long toffset;
	long polling_period;
	int ndevs;
	int active_trip_temps[MAX_ACTIVE_STATES];
	struct therm_fan_est_subdevice devs[];
};
#endif /* _LINUX_THERM_EST_H */
