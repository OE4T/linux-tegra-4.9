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

#ifndef __NVGPU_COMPTAGS__
#define __NVGPU_COMPTAGS__

#include <nvgpu/lock.h>

struct gk20a;

struct gk20a_comptags {
	u32 offset;
	u32 lines;
	u32 allocated_lines;
	bool user_mappable;
};

struct gk20a_comptag_allocator {
	struct gk20a *g;

	struct nvgpu_mutex lock;

	/* This bitmap starts at ctag 1. 0th cannot be taken. */
	unsigned long *bitmap;

	/* Size of bitmap, not max ctags, so one less. */
	unsigned long size;
};

/* real size here, but first (ctag 0) isn't used */
int gk20a_comptag_allocator_init(struct gk20a *g,
				 struct gk20a_comptag_allocator *allocator,
				 unsigned long size);
void gk20a_comptag_allocator_destroy(struct gk20a *g,
				     struct gk20a_comptag_allocator *allocator);

int gk20a_comptaglines_alloc(struct gk20a_comptag_allocator *allocator,
			     u32 *offset, u32 len);
void gk20a_comptaglines_free(struct gk20a_comptag_allocator *allocator,
			     u32 offset, u32 len);



#endif
