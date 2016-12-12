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

#ifndef __NVGPU_TIMERS_H__
#define __NVGPU_TIMERS_H__

struct gk20a;

/*
 * struct nvgpu_timeout - define a timeout.
 *
 * There are two types of timer suported:
 *
 *   o  NVGPU_TIMER_CPU_TIMER
 *        Timer uses the CPU to measure the timeout.
 *
 *   o  NVGPU_TIMER_RETRY_TIMER
 *        Instead of measuring a time limit keep track of the number of times
 *        something has been attempted. After said limit, "expire" the timer.
 *
 * Available flags:
 *
 *   o  NVGPU_TIMER_NO_PRE_SI
 *        By default when the system is not running on silicon the timeout
 *        code will ignore the requested timeout. Specifying this flag will
 *        override that behavior and honor the timeout regardless of platform.
 *
 *   o  NVGPU_TIMER_SILENT_TIMEOUT
 *        Do not print any messages on timeout. Normally a simple message is
 *        printed that specifies where the timeout occurred.
 */
struct nvgpu_timeout {
	struct gk20a		*g;

	unsigned int		 flags;

	union {
		unsigned long	 time;
		struct {
			int	 max;
			int	 attempted;
		} retries;
	};
};

/*
 * Bit 0 specifies the type of timer: CPU or retry.
 */
#define NVGPU_TIMER_CPU_TIMER		(0x0)
#define NVGPU_TIMER_RETRY_TIMER		(0x1)

/*
 * Bits 1 through 7 are reserved; bits 8 and up are flags:
 */
#define NVGPU_TIMER_NO_PRE_SI		(0x1 << 8)
#define NVGPU_TIMER_SILENT_TIMEOUT	(0x1 << 9)

#define NVGPU_TIMER_FLAG_MASK		(NVGPU_TIMER_RETRY_TIMER |	\
					 NVGPU_TIMER_NO_PRE_SI |	\
					 NVGPU_TIMER_SILENT_TIMEOUT)

int nvgpu_timeout_init(struct gk20a *g, struct nvgpu_timeout *timeout,
		       int duration, unsigned long flags);
int nvgpu_timeout_peek_expired(struct nvgpu_timeout *timeout);

#define nvgpu_timeout_expired(__timeout)				\
	__nvgpu_timeout_expired_msg(__timeout,				\
				  __builtin_return_address(0), "")

#define nvgpu_timeout_expired_msg(__timeout, fmt, args...)		\
	__nvgpu_timeout_expired_msg(__timeout,				\
				  __builtin_return_address(0),		\
				  fmt, ##args)

/*
 * Don't use this directly.
 */
int __nvgpu_timeout_expired_msg(struct nvgpu_timeout *timeout,
			      void *caller, const char *fmt, ...);

#endif
