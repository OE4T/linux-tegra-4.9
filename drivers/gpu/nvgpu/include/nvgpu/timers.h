/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_TIMERS_H__
#define __NVGPU_TIMERS_H__

#include <nvgpu/types.h>

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
		s64		 time;
		struct {
			u32	 max;
			u32	 attempted;
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
		       u32 duration, unsigned long flags);
int nvgpu_timeout_peek_expired(struct nvgpu_timeout *timeout);

#define nvgpu_timeout_expired(__timeout)				\
	__nvgpu_timeout_expired_msg(__timeout, (void *)_THIS_IP_, "")

#define nvgpu_timeout_expired_msg(__timeout, fmt, args...)		\
	__nvgpu_timeout_expired_msg(__timeout, (void *)_THIS_IP_,	\
				    fmt, ##args)

/*
 * Don't use this directly.
 */
int __nvgpu_timeout_expired_msg(struct nvgpu_timeout *timeout,
			      void *caller, const char *fmt, ...);


/*
 * Waits and delays.
 */
void nvgpu_msleep(unsigned int msecs);
void nvgpu_usleep_range(unsigned int min_us, unsigned int max_us);
void nvgpu_udelay(unsigned int usecs);

/*
 * Timekeeping.
 */
s64 nvgpu_current_time_ms(void);

#endif
