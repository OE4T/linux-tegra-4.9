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

struct therm_est_subdevice {
	void *dev_data;
	int (*get_temp)(void *, long *);
	long coeffs[HIST_LEN];
	long hist[HIST_LEN];
};

struct therm_est_data {
	long toffset;
	long polling_period;
	struct thermal_cooling_device *cdev;
	long trip_temp;
	int tc1;
	int tc2;
	int passive_delay;
	int ndevs;
	struct therm_est_subdevice devs[];
};

#endif /* _LINUX_THERM_EST_H */
