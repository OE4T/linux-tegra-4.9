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

#ifndef __NVGPU_COND_H__
#define __NVGPU_COND_H__

#ifdef __KERNEL__
#include <nvgpu/linux/cond.h>
#else
#include <nvgpu_rmos/include/cond.h>
#endif

/*
 * struct nvgpu_cond
 *
 * Should be implemented per-OS in a separate library
 */
struct nvgpu_cond;

/**
 * nvgpu_cond_init - Initialize a condition variable
 *
 * @cond - The condition variable to initialize
 *
 * Initialize a condition variable before using it.
 */
int nvgpu_cond_init(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_signal - Signal a condition variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up a waiter for a condition variable to check if its condition has been
 * satisfied.
 *
 * The waiter is using an uninterruptible wait.
 */
int nvgpu_cond_signal(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_signal_interruptible - Signal a condition variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up a waiter for a condition variable to check if its condition has been
 * satisfied.
 *
 * The waiter is using an interruptible wait.
 */
int nvgpu_cond_signal_interruptible(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_broadcast - Signal all waiters of a condition variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up all waiters for a condition variable to check if their conditions
 * have been satisfied.
 *
 * The waiters are using an uninterruptible wait.
 */
int nvgpu_cond_broadcast(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_broadcast_interruptible - Signal all waiters of a condition
 * variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up all waiters for a condition variable to check if their conditions
 * have been satisfied.
 *
 * The waiters are using an interruptible wait.
 */
int nvgpu_cond_broadcast_interruptible(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_destroy - Destroy a condition variable
 *
 * @cond - The condition variable to destroy
 */
void nvgpu_cond_destroy(struct nvgpu_cond *cond);

#endif /* __NVGPU_COND_H__ */
