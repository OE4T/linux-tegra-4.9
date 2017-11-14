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
struct nvgpu_os_buffer;

struct gk20a_comptags {
	u32 offset;
	u32 lines;

	/*
	 * This signals whether allocation has been attempted. Observe 'lines'
	 * to see whether the comptags were actually allocated. We try alloc
	 * only once per buffer in order not to break multiple compressible-kind
	 * mappings.
	 */
	bool allocated;

	/*
	 * Do comptags need to be cleared before mapping?
	 */
	bool needs_clear;
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

/*
 * Defined by OS specific code since comptags are stored in a highly OS specific
 * way.
 */
int gk20a_alloc_or_get_comptags(struct gk20a *g,
				struct nvgpu_os_buffer *buf,
				struct gk20a_comptag_allocator *allocator,
				struct gk20a_comptags *comptags);
void gk20a_get_comptags(struct nvgpu_os_buffer *buf,
			struct gk20a_comptags *comptags);

/*
 * These functions must be used to synchronize comptags clear. The usage:
 *
 *   if (gk20a_comptags_start_clear(os_buf)) {
 *           // we now hold the buffer lock for clearing
 *
 *           bool successful = hw_clear_comptags();
 *
 *           // mark the buf cleared (or not) and release the buffer lock
 *           gk20a_comptags_finish_clear(os_buf, successful);
 *   }
 *
 *  If gk20a_start_comptags_clear() returns false, another caller has
 *  already cleared the comptags.
 */
bool gk20a_comptags_start_clear(struct nvgpu_os_buffer *buf);
void gk20a_comptags_finish_clear(struct nvgpu_os_buffer *buf,
				 bool clear_successful);

#endif
