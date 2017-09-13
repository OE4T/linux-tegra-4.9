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

#ifndef __NVGPU_PRAMIN_H__
#define __NVGPU_PRAMIN_H__

#include <linux/types.h>

struct gk20a;
struct mm_gk20a;
struct nvgpu_mem;

/*
 * This typedef is for functions that get called during the access_batched()
 * operation.
 */
typedef void (*pramin_access_batch_fn)(struct gk20a *g, u32 start, u32 words,
				       u32 **arg);

/*
 * Generally useful batch functions.
 */
void pramin_access_batch_rd_n(struct gk20a *g, u32 start, u32 words, u32 **arg);
void pramin_access_batch_wr_n(struct gk20a *g, u32 start, u32 words, u32 **arg);
void pramin_access_batch_set(struct gk20a *g, u32 start, u32 words, u32 **arg);

void nvgpu_pramin_access_batched(struct gk20a *g, struct nvgpu_mem *mem,
				 u32 offset, u32 size,
				 pramin_access_batch_fn loop, u32 **arg);

void nvgpu_init_pramin(struct mm_gk20a *mm);

#endif
