/*
 * gk20a clock scaling profile
 *
 * Copyright (c) 2013-2016, NVIDIA Corporation. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef GK20A_SCALE_H
#define GK20A_SCALE_H

#include <linux/devfreq.h>

struct clk;

struct gk20a_scale_profile {
	struct device			*dev;
	ktime_t				last_event_time;
	struct devfreq_dev_profile	devfreq_profile;
	struct devfreq_dev_status	dev_stat;
	struct notifier_block		qos_notify_block;
	unsigned long			qos_min_freq;
	unsigned long			qos_max_freq;
	void				*private_data;
};

/* Initialization and de-initialization for module */
void gk20a_scale_init(struct device *);
void gk20a_scale_exit(struct device *);
void gk20a_scale_hw_init(struct device *dev);

#if defined(CONFIG_GK20A_DEVFREQ)
/*
 * call when performing submit to notify scaling mechanism that the module is
 * in use
 */
void gk20a_scale_notify_busy(struct device *);
void gk20a_scale_notify_idle(struct device *);

void gk20a_scale_suspend(struct device *);
void gk20a_scale_resume(struct device *);
int gk20a_scale_qos_notify(struct notifier_block *nb,
			unsigned long n, void *p);
#else
static inline void gk20a_scale_notify_busy(struct device *dev) {}
static inline void gk20a_scale_notify_idle(struct device *dev) {}
static inline void gk20a_scale_suspend(struct device *dev) {}
static inline void gk20a_scale_resume(struct device *dev) {}
static inline int gk20a_scale_qos_notify(struct notifier_block *nb,
			unsigned long n, void *p)
{
	return -ENOSYS;
}
#endif

#endif
