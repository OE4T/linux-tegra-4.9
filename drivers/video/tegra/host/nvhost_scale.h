/*
 * drivers/video/tegra/host/nvhost_scale.h
 *
 * Tegra Graphics Host 3D clock scaling
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef __NVHOST_SCALE_H
#define __NVHOST_SCALE_H

#include <mach/clk.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

/*
 * debugfs parameters to control 3d clock scaling test
 *
 * period        - time period for clock rate evaluation
 * fast_response - time period for evaluation of 'busy' spikes
 * idle_min      - if less than [idle_min] percent idle over [fast_response]
 *                 microseconds, clock up.
 * idle_max      - if over [idle_max] percent idle over [period] microseconds,
 *                 clock down.
 * max_scale     - limits rate changes to no less than (100 - max_scale)% or
 *                 (100 + 2 * max_scale)% of current clock rate
 * verbosity     - set above 5 for debug printouts
 */

struct scale3d_info_rec {
	spinlock_t lock; /* lock for timestamps etc */
	struct mutex set_lock; /* lock for clock setting */
	int enable;
	int init;
	ktime_t idle_frame;
	ktime_t fast_frame;
	ktime_t last_idle;
	ktime_t last_busy;
	int is_idle;
	unsigned long idle_total;
	struct work_struct work;
	unsigned int scale;
	unsigned int p_period;
	unsigned int p_idle_min;
	unsigned int p_idle_max;
	unsigned int p_fast_response;
	unsigned int p_verbosity;
	struct clk *clk_3d;
	struct clk *clk_3d2;
};

/* reset 3d module load counters, called on resume */
void scale3d_reset(void);

int scale3d_is_enabled(void);
void scale3d_enable(int enable);

#endif /* __NVHOST_SCALE_H */
