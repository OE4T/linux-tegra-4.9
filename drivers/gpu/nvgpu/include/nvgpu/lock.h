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

#ifndef NVGPU_LOCK_H
#define NVGPU_LOCK_H

#include <nvgpu/lock_linux.h>

/*
 * struct nvgpu_mutex
 *
 * Should be implemented per-OS in a separate library
 * But implementation should adhere to mutex implementation
 * as specified in Linux Documentation
 */
struct nvgpu_mutex;

/*
 * struct nvgpu_spinlock
 *
 * Should be implemented per-OS in a separate library
 * But implementation should adhere to spinlock implementation
 * as specified in Linux Documentation
 */
struct nvgpu_spinlock;

/*
 * struct nvgpu_raw_spinlock
 *
 * Should be implemented per-OS in a separate library
 * But implementation should adhere to raw_spinlock implementation
 * as specified in Linux Documentation
 */
struct nvgpu_raw_spinlock;

int nvgpu_mutex_init(struct nvgpu_mutex *mutex);
void nvgpu_mutex_acquire(struct nvgpu_mutex *mutex);
void nvgpu_mutex_release(struct nvgpu_mutex *mutex);
int nvgpu_mutex_tryacquire(struct nvgpu_mutex *mutex);
void nvgpu_mutex_destroy(struct nvgpu_mutex *mutex);

void nvgpu_spinlock_init(struct nvgpu_spinlock *spinlock);
void nvgpu_spinlock_acquire(struct nvgpu_spinlock *spinlock);
void nvgpu_spinlock_release(struct nvgpu_spinlock *spinlock);

void nvgpu_raw_spinlock_init(struct nvgpu_raw_spinlock *spinlock);
void nvgpu_raw_spinlock_acquire(struct nvgpu_raw_spinlock *spinlock);
void nvgpu_raw_spinlock_release(struct nvgpu_raw_spinlock *spinlock);

#endif /* NVGPU_LOCK_H */
