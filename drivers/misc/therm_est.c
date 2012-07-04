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

int therm_est_get_temp(struct therm_estimator *est, long *temp)
{
	*temp = est->cur_temp;
	return 0;
}

int therm_est_set_limits(struct therm_estimator *est,
				long lo_limit,
				long hi_limit)
{
	est->therm_est_lo_limit = lo_limit;
	est->therm_est_hi_limit = hi_limit;
	return 0;
}

int therm_est_set_alert(struct therm_estimator *est,
			void (*cb)(void *),
			void *cb_data)
{
	if ((!cb) || est->callback)
		BUG();

	est->callback = cb;
	est->callback_data = cb_data;

	return 0;
}

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
		if (est->devs[i]->get_temp(est->devs[i]->dev_data, &temp))
			continue;
		est->devs[i]->hist[(est->ntemp % HIST_LEN)] = temp;
	}

	for (i = 0; i < est->ndevs; i++) {
		for (j = 0; j < HIST_LEN; j++) {
			index = (est->ntemp - j + HIST_LEN) % HIST_LEN;
			sum += est->devs[i]->hist[index] *
				est->devs[i]->coeffs[j];
		}
	}

	est->cur_temp = sum / 100 + est->toffset;

	est->ntemp++;

	if (est->callback && ((est->cur_temp >= est->therm_est_hi_limit) ||
			 (est->cur_temp <= est->therm_est_lo_limit)))
		est->callback(est->callback_data);

	queue_delayed_work(est->workqueue, &est->therm_est_work,
				msecs_to_jiffies(est->polling_period));
}

struct therm_estimator *therm_est_register(
			struct therm_est_subdevice **devs,
			int ndevs,
			long toffset,
			long polling_period)
{
	int i, j;
	long temp;
	struct therm_estimator *est;
	struct therm_est_subdevice *dev;

	est = kzalloc(sizeof(struct therm_estimator), GFP_KERNEL);
	if (IS_ERR_OR_NULL(est))
		return ERR_PTR(-ENOMEM);

	est->devs = devs;
	est->ndevs = ndevs;
	est->toffset = toffset;
	est->polling_period = polling_period;

	/* initialize history */
	for (i = 0; i < ndevs; i++) {
		dev = est->devs[i];

		if (dev->get_temp(dev->dev_data, &temp)) {
			kfree(est);
			return ERR_PTR(-EINVAL);
		}

		for (j = 0; j < HIST_LEN; j++) {
			dev->hist[j] = temp;
		}
	}

	est->workqueue = alloc_workqueue("therm_est",
				    WQ_HIGHPRI | WQ_UNBOUND, 1);
	INIT_DELAYED_WORK(&est->therm_est_work, therm_est_work_func);

	queue_delayed_work(est->workqueue,
				&est->therm_est_work,
				msecs_to_jiffies(est->polling_period));

	return est;
}
