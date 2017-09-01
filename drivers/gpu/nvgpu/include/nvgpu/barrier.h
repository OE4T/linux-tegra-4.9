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

/* This file contains NVGPU_* high-level abstractions for various
 * memor-barrier operations available in linux/kernel. Every OS
 * should provide their own OS specific calls under this common API
 */

#ifndef __NVGPU_BARRIER_H__
#define __NVGPU_BARRIER_H__

#ifdef __KERNEL__
#include <nvgpu/linux/barrier.h>
#endif

#define nvgpu_mb()	__nvgpu_mb()
#define nvgpu_rmb()	__nvgpu_rmb()
#define nvgpu_wmb()	__nvgpu_wmb()

#define nvgpu_smp_mb()	__nvgpu_smp_mb()
#define nvgpu_smp_rmb()	__nvgpu_smp_rmb()
#define nvgpu_smp_wmb()	__nvgpu_smp_wmb()

#define nvgpu_read_barrier_depends() __nvgpu_read_barrier_depends()
#define nvgpu_smp_read_barrier_depends() __nvgpu_smp_read_barrier_depends()

#define NV_ACCESS_ONCE(x)	__NV_ACCESS_ONCE(x)

#endif /* __NVGPU_BARRIER_H__ */
