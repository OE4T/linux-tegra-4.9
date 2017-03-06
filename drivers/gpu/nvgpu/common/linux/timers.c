/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/jiffies.h>

#include <nvgpu/timers.h>

#include "gk20a/gk20a.h"

/*
 * Returns 1 if the platform is pre-Si and should ignore the timeout checking.
 * Setting %NVGPU_TIMER_NO_PRE_SI will make this always return 0 (i.e do the
 * timeout check regardless of platform).
 */
static int nvgpu_timeout_is_pre_silicon(struct nvgpu_timeout *timeout)
{
	if (timeout->flags & NVGPU_TIMER_NO_PRE_SI)
		return 0;

	return !tegra_platform_is_silicon();
}

/**
 * nvgpu_timeout_init - Init timer.
 *
 * @g        - nvgpu device.
 * @timeout  - The timer.
 * @duration - Timeout in milliseconds or number of retries.
 * @flags    - Flags for timer.
 *
 * This configures the timeout to start the timeout duration now, i.e: when this
 * function is called. Available flags to pass to @flags:
 *
 *   %NVGPU_TIMER_CPU_TIMER
 *   %NVGPU_TIMER_RETRY_TIMER
 *   %NVGPU_TIMER_NO_PRE_SI
 *   %NVGPU_TIMER_SILENT_TIMEOUT
 *
 * If neither %NVGPU_TIMER_CPU_TIMER or %NVGPU_TIMER_RETRY_TIMER is passed then
 * a CPU timer is used by default.
 */
int nvgpu_timeout_init(struct gk20a *g, struct nvgpu_timeout *timeout,
		       int duration, unsigned long flags)
{
	if (flags & ~NVGPU_TIMER_FLAG_MASK)
		return -EINVAL;

	memset(timeout, 0, sizeof(*timeout));

	timeout->g = g;
	timeout->flags = flags;

	if (flags & NVGPU_TIMER_RETRY_TIMER)
		timeout->retries.max = duration;
	else
		timeout->time = jiffies + msecs_to_jiffies(duration);

	return 0;
}

static int __nvgpu_timeout_expired_msg_cpu(struct nvgpu_timeout *timeout,
					 void *caller,
					 const char *fmt, va_list args)
{
	struct gk20a *g = timeout->g;
	unsigned long now = jiffies;

	if (nvgpu_timeout_is_pre_silicon(timeout))
		return 0;

	if (time_after(now, (unsigned long)timeout->time)) {
		if (!(timeout->flags & NVGPU_TIMER_SILENT_TIMEOUT)) {
			char buf[128];

			vsnprintf(buf, sizeof(buf), fmt, args);

			dev_err(dev_from_gk20a(g),
				"Timeout detected @ %pF %s\n", caller, buf);
		}

		return -ETIMEDOUT;
	}

	return 0;
}

static int __nvgpu_timeout_expired_msg_retry(struct nvgpu_timeout *timeout,
					   void *caller,
					   const char *fmt, va_list args)
{
	struct gk20a *g = timeout->g;

	if (nvgpu_timeout_is_pre_silicon(timeout))
		return 0;

	if (timeout->retries.attempted >= timeout->retries.max) {
		if (!(timeout->flags & NVGPU_TIMER_SILENT_TIMEOUT)) {
			char buf[128];

			vsnprintf(buf, sizeof(buf), fmt, args);

			dev_err(dev_from_gk20a(g),
				"No more retries @ 0x%pF %s\n", caller, buf);
		}

		return -ETIMEDOUT;
	}

	timeout->retries.attempted++;

	return 0;
}

/**
 * __nvgpu_timeout_expired_msg - Check if a timeout has expired.
 *
 * @timeout - The timeout to check.
 * @caller  - Address of the caller of this function.
 * @fmt     - The fmt string.
 *
 * Returns -ETIMEDOUT if the timeout has expired, 0 otherwise.
 *
 * If a timeout occurs and %NVGPU_TIMER_SILENT_TIMEOUT is not set in the timeout
 * then a message is printed based on %fmt.
 */
int __nvgpu_timeout_expired_msg(struct nvgpu_timeout *timeout,
			      void *caller, const char *fmt, ...)
{
	int ret;
	va_list args;

	va_start(args, fmt);
	if (timeout->flags & NVGPU_TIMER_RETRY_TIMER)
		ret = __nvgpu_timeout_expired_msg_retry(timeout, caller, fmt,
						      args);
	else
		ret = __nvgpu_timeout_expired_msg_cpu(timeout, caller, fmt,
						    args);
	va_end(args);

	return ret;
}

/**
 * nvgpu_timeout_peek_expired - Check the status of a timeout.
 *
 * @timeout - The timeout to check.
 *
 * Returns non-zero if the timeout is expired, zero otherwise. In the case of
 * retry timers this will not increment the underlying retry count. Also if the
 * timer has expired no messages will be printed.
 *
 * This function honors the pre-Si check as well.
 */
int nvgpu_timeout_peek_expired(struct nvgpu_timeout *timeout)
{
	if (nvgpu_timeout_is_pre_silicon(timeout))
		return 0;

	if (timeout->flags & NVGPU_TIMER_RETRY_TIMER)
		return timeout->retries.attempted >= timeout->retries.max;
	else
		return time_after(jiffies, (unsigned long)timeout->time);
}
