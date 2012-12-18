/*
 * drivers/video/tegra/host/t20/scale3d.c
 *
 * Tegra Graphics Host 3D clock scaling
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

/*
 * 3d clock scaling
 *
 * module3d_notify_busy() is called upon submit, module3d_notify_idle() is
 * called when all outstanding submits are completed. Idle times are measured
 * over a fixed time period (scale3d.p_period). If the 3d module idle time
 * percentage goes over the limit (set in scale3d.p_idle_max), 3d clocks are
 * scaled down. If the percentage goes under the minimum limit (set in
 * scale3d.p_idle_min), 3d clocks are scaled up. An additional test is made
 * over the time frame given in scale3d.p_fast_response for clocking up
 * quickly in response to load peaks.
 *
 * 3d.emc clock is scaled proportionately to 3d clock, with a quadratic-
 * bezier-like factor added to pull 3d.emc rate a bit lower.
 */

#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <mach/clk.h>
#include <mach/hardware.h>
#include "scale3d.h"
#include "dev.h"

static int scale3d_is_enabled(void);
static void scale3d_enable(int enable);

#define POW2(x) ((x) * (x))

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
	struct mutex lock; /* lock for timestamps etc */
	int enable;
	int init;
	ktime_t idle_frame;
	ktime_t fast_frame;
	ktime_t last_idle;
	ktime_t last_short_term_idle;
	int is_idle;
	ktime_t last_tweak;
	ktime_t last_down;
	int fast_up_count;
	int slow_down_count;
	int is_scaled;
	int fast_responses;
	unsigned long idle_total;
	unsigned long idle_short_term_total;
	unsigned long max_rate_3d;
	long emc_slope;
	long emc_offset;
	long emc_dip_slope;
	long emc_dip_offset;
	long emc_xmid;
	unsigned long min_rate_3d;
	struct work_struct work;
	struct delayed_work idle_timer;
	unsigned int scale;
	unsigned int p_period;
	unsigned int period;
	unsigned int p_idle_min;
	unsigned int idle_min;
	unsigned int p_idle_max;
	unsigned int idle_max;
	unsigned int p_fast_response;
	unsigned int fast_response;
	unsigned int p_adjust;
	unsigned int p_scale_emc;
	unsigned int p_emc_dip;
	unsigned int p_verbosity;
	struct clk *clk_3d;
	struct clk *clk_3d2;
	struct clk *clk_3d_emc;
};

static struct scale3d_info_rec scale3d;

static void scale3d_clocks(unsigned long percent)
{
	unsigned long hz, curr;

	if (!tegra_is_clk_enabled(scale3d.clk_3d))
		return;

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3)
		if (!tegra_is_clk_enabled(scale3d.clk_3d2))
			return;

	curr = clk_get_rate(scale3d.clk_3d);
	hz = percent * (curr / 100);

	if (!(hz >= scale3d.max_rate_3d && curr == scale3d.max_rate_3d)) {
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3)
			clk_set_rate(scale3d.clk_3d2, 0);
		clk_set_rate(scale3d.clk_3d, hz);

		if (scale3d.p_scale_emc) {
			long after = (long) clk_get_rate(scale3d.clk_3d);
			hz = after * scale3d.emc_slope + scale3d.emc_offset;
			if (scale3d.p_emc_dip)
				hz -=
					(scale3d.emc_dip_slope *
					POW2(after / 1000 - scale3d.emc_xmid) +
					scale3d.emc_dip_offset);
			clk_set_rate(scale3d.clk_3d_emc, hz);
		}
	}
}

static void scale3d_clocks_handler(struct work_struct *work)
{
	unsigned int scale;

	mutex_lock(&scale3d.lock);
	scale = scale3d.scale;
	mutex_unlock(&scale3d.lock);

	if (scale != 0)
		scale3d_clocks(scale);
}

void nvhost_scale3d_suspend(struct nvhost_device *dev)
{
	if (!scale3d.enable)
		return;

	cancel_work_sync(&scale3d.work);
	cancel_delayed_work(&scale3d.idle_timer);
}

/* set 3d clocks to max */
static void reset_3d_clocks(void)
{
	if (clk_get_rate(scale3d.clk_3d) != scale3d.max_rate_3d) {
		clk_set_rate(scale3d.clk_3d, scale3d.max_rate_3d);
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3)
			clk_set_rate(scale3d.clk_3d2, scale3d.max_rate_3d);
		if (scale3d.p_scale_emc)
			clk_set_rate(scale3d.clk_3d_emc,
				clk_round_rate(scale3d.clk_3d_emc, UINT_MAX));
	}
}

static int scale3d_is_enabled(void)
{
	int enable;

	if (!scale3d.enable)
		return 0;

	mutex_lock(&scale3d.lock);
	enable = scale3d.enable;
	mutex_unlock(&scale3d.lock);

	return enable;
}

static void scale3d_enable(int enable)
{
	int disable = 0;

	mutex_lock(&scale3d.lock);

	if (enable) {
		if (scale3d.max_rate_3d != scale3d.min_rate_3d)
			scale3d.enable = 1;
	} else {
		scale3d.enable = 0;
		disable = 1;
	}

	mutex_unlock(&scale3d.lock);

	if (disable)
		reset_3d_clocks();
}

static void reset_scaling_counters(ktime_t time)
{
	scale3d.idle_total = 0;
	scale3d.idle_short_term_total = 0;
	scale3d.last_idle = time;
	scale3d.last_short_term_idle = time;
	scale3d.idle_frame = time;
}

/* scaling_adjust - use scale up / scale down hint counts to adjust scaling
 * parameters.
 *
 * hint_ratio is 100 x the ratio of scale up to scale down hints. Three cases
 * are distinguished:
 *
 * hint_ratio < HINT_RATIO_MIN - set parameters to maximize scaling effect
 * hint_ratio > HINT_RATIO_MAX - set parameters to minimize scaling effect
 * hint_ratio between limits - scale parameters linearly
 *
 * the parameters adjusted are
 *
 * * fast_response time
 * * period - time for scaling down estimate
 * * idle_min percentage
 * * idle_max percentage
 */
#define SCALING_ADJUST_PERIOD 1000000
#define HINT_RATIO_MAX 400
#define HINT_RATIO_MIN 100
#define HINT_RATIO_MID ((HINT_RATIO_MAX + HINT_RATIO_MIN) / 2)
#define HINT_RATIO_DIFF (HINT_RATIO_MAX - HINT_RATIO_MIN)

static void scaling_adjust(ktime_t time)
{
	long hint_ratio;
	long fast_response_adjustment;
	long period_adjustment;
	int idle_min_adjustment;
	int idle_max_adjustment;
	unsigned long dt;

	dt = (unsigned long) ktime_us_delta(time, scale3d.last_tweak);
	if (dt < SCALING_ADJUST_PERIOD)
		return;

	hint_ratio = (100 * (scale3d.fast_up_count + 1)) /
				 (scale3d.slow_down_count + 1);

	if (hint_ratio > HINT_RATIO_MAX) {
		fast_response_adjustment = -((int) scale3d.p_fast_response) / 4;
		period_adjustment = scale3d.p_period / 2;
		idle_min_adjustment = scale3d.p_idle_min;
		idle_max_adjustment = scale3d.p_idle_max;
	} else if (hint_ratio < HINT_RATIO_MIN) {
		fast_response_adjustment = scale3d.p_fast_response / 2;
		period_adjustment = -((int) scale3d.p_period) / 4;
		idle_min_adjustment = -((int) scale3d.p_idle_min) / 2;
		idle_max_adjustment = -((int) scale3d.p_idle_max) / 2;
	} else {
		int diff;
		int factor;

		diff = HINT_RATIO_MID - hint_ratio;
		if (diff < 0)
			factor = -diff * 2;
		else {
			factor = -diff;
			diff *= 2;
		}

		fast_response_adjustment = diff *
			(scale3d.p_fast_response / (HINT_RATIO_DIFF * 2));
		period_adjustment =
			diff * (scale3d.p_period / HINT_RATIO_DIFF);
		idle_min_adjustment =
			(factor * (int) scale3d.p_idle_min) / HINT_RATIO_DIFF;
		idle_max_adjustment =
			(factor * (int) scale3d.p_idle_max) / HINT_RATIO_DIFF;
	}

	scale3d.fast_response =
		scale3d.p_fast_response + fast_response_adjustment;
	scale3d.period = scale3d.p_period + period_adjustment;
		scale3d.idle_min = scale3d.p_idle_min + idle_min_adjustment;
	scale3d.idle_max = scale3d.p_idle_max + idle_max_adjustment;

	if (scale3d.p_verbosity >= 10)
		pr_info("scale3d stats: + %d - %d (/ %d) f %u p %u min %u max %u\n",
			scale3d.fast_up_count, scale3d.slow_down_count,
			scale3d.fast_responses, scale3d.fast_response,
			scale3d.period, scale3d.idle_min, scale3d.idle_max);

	scale3d.fast_up_count = 0;
	scale3d.slow_down_count = 0;
	scale3d.fast_responses = 0;
	scale3d.last_down = time;
	scale3d.last_tweak = time;
}

#undef SCALING_ADJUST_PERIOD
#undef HINT_RATIO_MAX
#undef HINT_RATIO_MIN
#undef HINT_RATIO_MID
#undef HINT_RATIO_DIFF

static void scaling_state_check(ktime_t time)
{
	unsigned long dt;

	/* adjustment: set scale parameters (fast_response, period) +/- 25%
	 * based on ratio of scale up to scale down hints
	 */
	if (scale3d.p_adjust)
		scaling_adjust(time);
	else {
		scale3d.fast_response = scale3d.p_fast_response;
		scale3d.period = scale3d.p_period;
		scale3d.idle_min = scale3d.p_idle_min;
		scale3d.idle_max = scale3d.p_idle_max;
	}

	/* check for load peaks */
	dt = (unsigned long) ktime_us_delta(time, scale3d.fast_frame);
	if (dt > scale3d.fast_response) {
		unsigned long idleness =
			(scale3d.idle_short_term_total * 100) / dt;
		scale3d.fast_responses++;
		scale3d.fast_frame = time;
		/* if too busy, scale up */
		if (idleness < scale3d.idle_min) {
			scale3d.is_scaled = 0;
			scale3d.fast_up_count++;
			if (scale3d.p_verbosity >= 5)
				pr_info("scale3d: %ld%% busy\n",
					100 - idleness);

			reset_3d_clocks();
			reset_scaling_counters(time);
			return;
		}
		scale3d.idle_short_term_total = 0;
		scale3d.last_short_term_idle = time;
	}

	dt = (unsigned long) ktime_us_delta(time, scale3d.idle_frame);
	if (dt > scale3d.period) {
		unsigned long idleness = (scale3d.idle_total * 100) / dt;

		if (scale3d.p_verbosity >= 5)
			pr_info("scale3d: idle %lu, ~%lu%%\n",
				scale3d.idle_total, idleness);

		if (idleness > scale3d.idle_max) {
			if (!scale3d.is_scaled) {
				scale3d.is_scaled = 1;
				scale3d.last_down = time;
			}
			scale3d.slow_down_count++;
			/* if idle time is high, clock down */
			scale3d.scale = 100 - (idleness - scale3d.idle_min);
			schedule_work(&scale3d.work);
		}

		reset_scaling_counters(time);
	}
}

void nvhost_scale3d_notify_idle(struct nvhost_device *dev)
{
	ktime_t t;
	unsigned long dt;

	if (!scale3d.enable)
		return;

	mutex_lock(&scale3d.lock);

	t = ktime_get();

	if (scale3d.is_idle) {
		dt = ktime_us_delta(t, scale3d.last_idle);
		scale3d.idle_total += dt;
		dt = ktime_us_delta(t, scale3d.last_short_term_idle);
		scale3d.idle_short_term_total += dt;
	} else
		scale3d.is_idle = 1;

	scale3d.last_idle = t;
	scale3d.last_short_term_idle = t;

	scaling_state_check(scale3d.last_idle);

	/* delay idle_max % of 2 * fast_response time (given in microseconds) */
	schedule_delayed_work(&scale3d.idle_timer,
		msecs_to_jiffies((scale3d.idle_max * scale3d.fast_response)
			/ 50000));

	mutex_unlock(&scale3d.lock);
}

void nvhost_scale3d_notify_busy(struct nvhost_device *dev)
{
	unsigned long idle;
	unsigned long short_term_idle;
	ktime_t t;

	if (!scale3d.enable)
		return;

	mutex_lock(&scale3d.lock);

	cancel_delayed_work(&scale3d.idle_timer);

	t = ktime_get();

	if (scale3d.is_idle) {
		idle = (unsigned long)
			ktime_us_delta(t, scale3d.last_idle);
		scale3d.idle_total += idle;
		short_term_idle =
			ktime_us_delta(t, scale3d.last_short_term_idle);
		scale3d.idle_short_term_total += short_term_idle;
		scale3d.is_idle = 0;
	}

	scaling_state_check(t);

	mutex_unlock(&scale3d.lock);
}

static void scale3d_idle_handler(struct work_struct *work)
{
	int notify_idle = 0;

	if (!scale3d.enable)
		return;

	mutex_lock(&scale3d.lock);

	if (scale3d.is_idle && tegra_is_clk_enabled(scale3d.clk_3d)) {
		unsigned long curr = clk_get_rate(scale3d.clk_3d);
		if (curr > scale3d.min_rate_3d)
			notify_idle = 1;
	}

	mutex_unlock(&scale3d.lock);

	if (notify_idle)
		nvhost_scale3d_notify_idle(NULL);
}

void nvhost_scale3d_reset()
{
	ktime_t t;

	if (!scale3d.enable)
		return;

	t = ktime_get();
	mutex_lock(&scale3d.lock);
	reset_scaling_counters(t);
	mutex_unlock(&scale3d.lock);
}

/*
 * debugfs parameters to control 3d clock scaling
 */

void nvhost_scale3d_debug_init(struct dentry *de)
{
	struct dentry *d, *f;

	d = debugfs_create_dir("scaling", de);
	if (!d) {
		pr_err("scale3d: can\'t create debugfs directory\n");
		return;
	}

#define CREATE_SCALE3D_FILE(fname) \
	do {\
		f = debugfs_create_u32(#fname, S_IRUGO | S_IWUSR, d,\
			&scale3d.p_##fname);\
		if (NULL == f) {\
			pr_err("scale3d: can\'t create file " #fname "\n");\
			return;\
		} \
	} while (0)

	CREATE_SCALE3D_FILE(fast_response);
	CREATE_SCALE3D_FILE(idle_min);
	CREATE_SCALE3D_FILE(idle_max);
	CREATE_SCALE3D_FILE(period);
	CREATE_SCALE3D_FILE(adjust);
	CREATE_SCALE3D_FILE(scale_emc);
	CREATE_SCALE3D_FILE(emc_dip);
	CREATE_SCALE3D_FILE(verbosity);
#undef CREATE_SCALE3D_FILE
}

static ssize_t enable_3d_scaling_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	ssize_t res;

	res = snprintf(buf, PAGE_SIZE, "%d\n", scale3d_is_enabled());

	return res;
}

static ssize_t enable_3d_scaling_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = 0;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	scale3d_enable(val);

	return count;
}

static DEVICE_ATTR(enable_3d_scaling, S_IRUGO | S_IWUSR,
	enable_3d_scaling_show, enable_3d_scaling_store);

void nvhost_scale3d_init(struct nvhost_device *d)
{
	if (!scale3d.init) {
		int error;
		unsigned long max_emc, min_emc;
		long correction;
		mutex_init(&scale3d.lock);

		INIT_WORK(&scale3d.work, scale3d_clocks_handler);
		INIT_DELAYED_WORK(&scale3d.idle_timer, scale3d_idle_handler);

		scale3d.clk_3d = d->clk[0];
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA3) {
			scale3d.clk_3d2 = d->clk[1];
			scale3d.clk_3d_emc = d->clk[2];
		} else
			scale3d.clk_3d_emc = d->clk[1];

		scale3d.max_rate_3d = clk_round_rate(scale3d.clk_3d, UINT_MAX);
		scale3d.min_rate_3d = clk_round_rate(scale3d.clk_3d, 0);

		if (scale3d.max_rate_3d == scale3d.min_rate_3d) {
			pr_warn("scale3d: 3d max rate = min rate (%lu), "
				"disabling\n", scale3d.max_rate_3d);
			scale3d.enable = 0;
			return;
		}

		/* emc scaling:
		 *
		 * Remc = S * R3d + O - (Sd * (R3d - Rm)^2 + Od)
		 *
		 * Remc - 3d.emc rate
		 * R3d  - 3d.cbus rate
		 * Rm   - 3d.cbus 'middle' rate = (max + min)/2
		 * S    - emc_slope
		 * O    - emc_offset
		 * Sd   - emc_dip_slope
		 * Od   - emc_dip_offset
		 *
		 * this superposes a quadratic dip centered around the middle 3d
		 * frequency over a linear correlation of 3d.emc to 3d clock
		 * rates.
		 *
		 * S, O are chosen so that the maximum 3d rate produces the
		 * maximum 3d.emc rate exactly, and the minimum 3d rate produces
		 * at least the minimum 3d.emc rate.
		 *
		 * Sd and Od are chosen to produce the largest dip that will
		 * keep 3d.emc frequencies monotonously decreasing with 3d
		 * frequencies. To achieve this, the first derivative of Remc
		 * with respect to R3d should be zero for the minimal 3d rate:
		 *
		 *   R'emc = S - 2 * Sd * (R3d - Rm)
		 *   R'emc(R3d-min) = 0
		 *   S = 2 * Sd * (R3d-min - Rm)
		 *     = 2 * Sd * (R3d-min - R3d-max) / 2
		 *   Sd = S / (R3d-min - R3d-max)
		 *
		 *   +---------------------------------------------------+
		 *   | Sd = -(emc-max - emc-min) / (R3d-min - R3d-max)^2 |
		 *   +---------------------------------------------------+
		 *
		 *   dip = Sd * (R3d - Rm)^2 + Od
		 *
		 * requiring dip(R3d-min) = 0 and dip(R3d-max) = 0 gives
		 *
		 *   Sd * (R3d-min - Rm)^2 + Od = 0
		 *   Od = -Sd * ((R3d-min - R3d-max) / 2)^2
		 *      = -Sd * ((R3d-min - R3d-max)^2) / 4
		 *
		 *   +------------------------------+
		 *   | Od = (emc-max - emc-min) / 4 |
		 *   +------------------------------+
		 */

		max_emc = clk_round_rate(scale3d.clk_3d_emc, UINT_MAX);
		min_emc = clk_round_rate(scale3d.clk_3d_emc, 0);

		scale3d.emc_slope = (max_emc - min_emc) /
			 (scale3d.max_rate_3d - scale3d.min_rate_3d);
		scale3d.emc_offset = max_emc -
			scale3d.emc_slope * scale3d.max_rate_3d;
		/* guarantee max 3d rate maps to max emc rate */
		scale3d.emc_offset += max_emc -
			(scale3d.emc_slope * scale3d.max_rate_3d +
			scale3d.emc_offset);

		scale3d.emc_dip_offset = (max_emc - min_emc) / 4;
		scale3d.emc_dip_slope =
			-4 * (scale3d.emc_dip_offset /
			(POW2(scale3d.max_rate_3d - scale3d.min_rate_3d)));
		scale3d.emc_xmid =
			(scale3d.max_rate_3d + scale3d.min_rate_3d) / 2;
		correction =
			scale3d.emc_dip_offset +
				scale3d.emc_dip_slope *
				POW2(scale3d.max_rate_3d - scale3d.emc_xmid);
		scale3d.emc_dip_offset -= correction;

		/* set scaling parameter defaults */
		scale3d.enable = 1;
		scale3d.period = scale3d.p_period = 100000;
		scale3d.idle_min = scale3d.p_idle_min = 10;
		scale3d.idle_max = scale3d.p_idle_max = 15;
		scale3d.fast_response = scale3d.p_fast_response = 7000;
		scale3d.p_scale_emc = 1;
		scale3d.p_emc_dip = 1;
		scale3d.p_verbosity = 0;
		scale3d.p_adjust = 1;

		error = device_create_file(&d->dev,
				&dev_attr_enable_3d_scaling);
		if (error)
			dev_err(&d->dev, "failed to create sysfs attributes");

		scale3d.init = 1;
	}

	nvhost_scale3d_reset();
}

void nvhost_scale3d_deinit(struct nvhost_device *dev)
{
	device_remove_file(&dev->dev, &dev_attr_enable_3d_scaling);
	scale3d.init = 0;
}
