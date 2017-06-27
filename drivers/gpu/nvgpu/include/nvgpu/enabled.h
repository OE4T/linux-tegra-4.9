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

#ifndef __NVGPU_ENABLED_H__
#define __NVGPU_ENABLED_H__

struct gk20a;

#include <nvgpu/types.h>

/*
 * Available flags that describe what's enabled and what's not in the GPU. Each
 * flag here is defined by it's offset in a bitmap.
 */
#define NVGPU_IS_FMODEL				1
#define NVGPU_DRIVER_IS_DYING			2

/*
 * MM flags.
 */
#define NVGPU_MM_UNIFY_ADDRESS_SPACES		16
/* false if vidmem aperture actually points to sysmem */
#define NVGPU_MM_HONORS_APERTURE		17
/* unified or split memory with separate vidmem? */
#define NVGPU_MM_UNIFIED_MEMORY			18

/*
 * PMU flags.
 */
/* perfmon enabled or disabled for PMU */
#define NVGPU_PMU_PERFMON			48
#define NVGPU_PMU_PSTATE			49

/*
 * Must be greater than the largest bit offset in the above list.
 */
#define NVGPU_MAX_ENABLED_BITS			64

/**
 * nvgpu_is_enabled - Check if the passed flag is enabled.
 *
 * @g     - The GPU.
 * @flag  - Which flag to check.
 *
 * Returns true if the passed @flag is true; false otherwise.
 */
bool nvgpu_is_enabled(struct gk20a *g, int flag);

/**
 * __nvgpu_set_enabled - Set the state of a flag.
 *
 * @g     - The GPU.
 * @flag  - Which flag to modify.
 * @state - The state to set the flag to.
 *
 * Set the state of the passed @flag to @state. This will return the previous
 * state of the passed @flag.
 */
bool __nvgpu_set_enabled(struct gk20a *g, int flag, bool state);

int nvgpu_init_enabled_flags(struct gk20a *g);

#endif
