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

struct therm_estimator {
	long therm_est_lo_limit;
	long therm_est_hi_limit;
	void (*callback)(void *);
	void *callback_data;
	long cur_temp;
	long polling_period;
	struct workqueue_struct *workqueue;
	struct delayed_work therm_est_work;
	long toffset;
	int ntemp;
	int ndevs;
	struct therm_est_subdevice **devs;
};

#ifdef CONFIG_THERM_EST
struct therm_estimator *therm_est_register(
			struct therm_est_subdevice **devs,
			int ndevs,
			long toffset,
			long pperiod);
int therm_est_get_temp(struct therm_estimator *est, long *temp);
int therm_est_set_limits(struct therm_estimator *est,
			long lo_limit,
			long hi_limit);
int therm_est_set_alert(struct therm_estimator *est,
			void (*cb)(void *),
			void *cb_data);
#else
static inline struct therm_estimator *therm_est_register(
			struct therm_est_subdevice **devs,
			int ndevs,
			long toffset,
			long pperiod)
{ return NULL; }
static inline int therm_est_get_temp(struct therm_estimator *est, long *temp)
{ return -EINVAL; }
static inline int therm_est_set_limits(struct therm_estimator *est,
			long lo_limit,
			long hi_limit)
{ return -EINVAL; }
static inline int therm_est_set_alert(struct therm_estimator *est,
			void (*cb)(void *),
			void *cb_data)
{ return -EINVAL; }
#endif
#endif /* _LINUX_THERM_EST_H */
