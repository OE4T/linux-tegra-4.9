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

#ifndef __NVGPU_PRAMIN_H__
#define __NVGPU_PRAMIN_H__

#include <linux/types.h>

struct gk20a;
struct mm_gk20a;
struct mem_desc;

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

void nvgpu_pramin_access_batched(struct gk20a *g, struct mem_desc *mem,
				 u32 offset, u32 size,
				 pramin_access_batch_fn loop, u32 **arg);

void nvgpu_init_pramin(struct mm_gk20a *mm);

#endif
