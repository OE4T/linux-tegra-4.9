/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#define NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP 3
/*
 * MM flags.
 */
#define NVGPU_MM_UNIFY_ADDRESS_SPACES		16
/* false if vidmem aperture actually points to sysmem */
#define NVGPU_MM_HONORS_APERTURE		17
/* unified or split memory with separate vidmem? */
#define NVGPU_MM_UNIFIED_MEMORY			18

/*
 * Security flags
 */

#define NVGPU_SEC_SECUREGPCCS			32
#define NVGPU_SEC_PRIVSECURITY			33

/*
 * PMU flags.
 */
/* perfmon enabled or disabled for PMU */
#define NVGPU_PMU_PERFMON				48
#define NVGPU_PMU_PSTATE				49
#define NVGPU_PMU_ZBC_SAVE				50
#define NVGPU_PMU_FECS_BOOTSTRAP_DONE			51
#define NVGPU_GPU_CAN_BLCG                              52
#define NVGPU_GPU_CAN_SLCG                              53
#define NVGPU_GPU_CAN_ELCG                              54

/* whether to run PREOS binary on dGPUs */
#define NVGPU_PMU_RUN_PREOS			52

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
