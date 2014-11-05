/*
 * gk20a clock scaling profile
 *
 * Copyright (c) 2013-2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GK20A_SCALE_H
#define GK20A_SCALE_H

#include <linux/devfreq.h>

struct platform_device;
struct clk;

struct gk20a_scale_profile {
	struct platform_device		*pdev;
	ktime_t				last_event_time;
	struct devfreq_dev_profile	devfreq_profile;
	struct devfreq_dev_status	dev_stat;
	struct notifier_block		qos_notify_block;
	void				*private_data;
};

/* Initialization and de-initialization for module */
void gk20a_scale_init(struct platform_device *);
void gk20a_scale_exit(struct platform_device *);
void gk20a_scale_hw_init(struct platform_device *pdev);

#ifdef CONFIG_GK20A_DEVFREQ
/*
 * call when performing submit to notify scaling mechanism that the module is
 * in use
 */
void gk20a_scale_notify_busy(struct platform_device *);
void gk20a_scale_notify_idle(struct platform_device *);

void gk20a_scale_suspend(struct platform_device *);
void gk20a_scale_resume(struct platform_device *);
#else
static inline void gk20a_scale_notify_busy(struct platform_device *pdev) {}
static inline void gk20a_scale_notify_idle(struct platform_device *pdev) {}
static inline void gk20a_scale_suspend(struct platform_device *pdev) {}
static inline void gk20a_scale_resume(struct platform_device *pdev) {}
#endif

#endif
