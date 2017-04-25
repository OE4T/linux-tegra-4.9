/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_COND_LINUX_H__
#define __NVGPU_COND_LINUX_H__

#include <linux/wait.h>

struct nvgpu_cond {
	bool initialized;
	wait_queue_head_t wq;
};

#define NVGPU_COND_WAIT(c, condition, timeout_ms) \
	wait_event_timeout((c)->wq, condition, timeout_ms)

#define NVGPU_COND_WAIT_INTERRUPTIBLE(c, condition, timeout_ms) \
	wait_event_interruptible_timeout((c)->wq, condition, timeout_ms)

#endif /* __NVGPU_LOCK_LINUX_H__ */
