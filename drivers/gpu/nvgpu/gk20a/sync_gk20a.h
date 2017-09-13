/*
 * drivers/video/tegra/host/gk20a/sync_gk20a.h
 *
 * GK20A Sync Framework Integration
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _GK20A_SYNC_H_
#define _GK20A_SYNC_H_

struct sync_timeline;
struct sync_fence;
struct sync_pt;
struct nvgpu_semaphore;
struct fence;

int gk20a_is_sema_backed_sync_fence(struct sync_fence *fence);
struct nvgpu_semaphore *gk20a_sync_fence_get_sema(struct sync_fence *f);

#ifdef CONFIG_SYNC
struct sync_timeline *gk20a_sync_timeline_create(const char *fmt, ...);
void gk20a_sync_timeline_destroy(struct sync_timeline *);
void gk20a_sync_timeline_signal(struct sync_timeline *);
struct sync_fence *gk20a_sync_fence_create(
		struct gk20a *g,
		struct sync_timeline *,
		struct nvgpu_semaphore *,
		const char *fmt, ...);
struct sync_fence *gk20a_sync_fence_fdget(int fd);
#else
static inline void gk20a_sync_timeline_destroy(struct sync_timeline *obj) {}
static inline void gk20a_sync_timeline_signal(struct sync_timeline *obj) {}
static inline struct sync_fence *gk20a_sync_fence_fdget(int fd)
{
	return NULL;
}
#endif

#endif
