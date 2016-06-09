/*
 * drivers/video/tegra/host/gk20a/sync_gk20a.h
 *
 * GK20A Sync Framework Integration
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _GK20A_SYNC_H_
#define _GK20A_SYNC_H_

#include <linux/types.h>
#include <linux/version.h>

struct sync_timeline;
struct sync_fence;
struct sync_pt;
struct gk20a_semaphore;
struct fence;

int __gk20a_is_gk20a_sync_pt_inst(struct sync_pt *pt);
int gk20a_is_gk20a_sync_pt_inst(struct fence *f);
int gk20a_is_sema_backed_sync_fence(struct sync_fence *fence);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,18,0)
struct gk20a_semaphore *gk20a_sync_pt_inst_get_sema(struct fence *f);
#else
struct gk20a_semaphore *gk20a_sync_pt_inst_get_sema(struct sync_pt *pt);
#endif

#ifdef CONFIG_SYNC
struct sync_timeline *gk20a_sync_timeline_create(const char *fmt, ...);
void gk20a_sync_timeline_destroy(struct sync_timeline *);
void gk20a_sync_timeline_signal(struct sync_timeline *);
struct sync_fence *gk20a_sync_fence_create(struct sync_timeline *,
		struct gk20a_semaphore *,
		struct sync_fence *dependency,
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
