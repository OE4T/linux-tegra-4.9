/*
 * drivers/video/tegra/host/gk20a/channel_sync_gk20a.h
 *
 * GK20A Channel Synchronization Abstraction
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

#ifndef _GK20A_CHANNEL_SYNC_H_
#define _GK20A_CHANNEL_SYNC_H_

#include <linux/types.h>

struct gk20a_channel_sync;
struct priv_cmd_entry;
struct channel_gk20a;

struct gk20a_channel_fence {
	bool valid;
	bool wfi; /* was issued with preceding wfi */
	u32 thresh; /* either semaphore or syncpoint value */
};

struct gk20a_channel_sync {
	/* CPU wait for a fence returned by incr_syncpt() or incr_fd(). */
	int (*wait_cpu)(struct gk20a_channel_sync *s,
			struct gk20a_channel_fence *fence,
			int timeout);

	/* Test whether a fence returned by incr_syncpt() or incr_fd() is
	 * expired. */
	bool (*is_expired)(struct gk20a_channel_sync *s,
			   struct gk20a_channel_fence *fence);

	/* Generate a gpu wait cmdbuf from syncpoint. */
	int (*wait_syncpt)(struct gk20a_channel_sync *s, u32 id, u32 thresh,
			   struct priv_cmd_entry **entry);

	/* Generate a gpu wait cmdbuf from sync fd. */
	int (*wait_fd)(struct gk20a_channel_sync *s, int fd,
		       struct priv_cmd_entry **entry);

	/* Increment syncpoint/semaphore.
	 * Returns
	 *  - a gpu cmdbuf that performs the increment when executed,
	 *  - a fence that can be passed to wait_cpu() and is_expired().
	 */
	int (*incr)(struct gk20a_channel_sync *s,
		    struct priv_cmd_entry **entry,
		    struct gk20a_channel_fence *fence);

	/* Increment syncpoint/semaphore, preceded by a wfi.
	 * Returns
	 *  - a gpu cmdbuf that performs the increment when executed,
	 *  - a fence that can be passed to wait_cpu() and is_expired().
	 */
	int (*incr_wfi)(struct gk20a_channel_sync *s,
			struct priv_cmd_entry **entry,
			struct gk20a_channel_fence *fence);

	/* Increment syncpoint, so that the returned fence represents
	 * work completion (may need wfi) and can be returned to user space.
	 * Returns
	 *  - a gpu cmdbuf that performs the increment when executed,
	 *  - a fence that can be passed to wait_cpu() and is_expired(),
	 *  - a syncpoint id/value pair that can be returned to user space.
	 */
	int (*incr_user_syncpt)(struct gk20a_channel_sync *s,
				struct priv_cmd_entry **entry,
				struct gk20a_channel_fence *fence,
				bool wfi,
				u32 *id, u32 *thresh);

	/* Increment syncpoint/semaphore, so that the returned fence represents
	 * work completion (may need wfi) and can be returned to user space.
	 * Returns
	 *  - a gpu cmdbuf that performs the increment when executed,
	 *  - a fence that can be passed to wait_cpu() and is_expired(),
	 *  - a sync fd that can be returned to user space.
	 */
	int (*incr_user_fd)(struct gk20a_channel_sync *s,
			    struct priv_cmd_entry **entry,
			    struct gk20a_channel_fence *fence,
			    bool wfi,
			    int *fd);

	/* Reset the channel syncpoint/semaphore. */
	void (*set_min_eq_max)(struct gk20a_channel_sync *s);

	/* flag to set syncpt destroy aggressiveness */
	bool syncpt_aggressive_destroy;

	/* Free the resources allocated by gk20a_channel_sync_create. */
	void (*destroy)(struct gk20a_channel_sync *s);
};

struct gk20a_channel_sync *gk20a_channel_sync_create(struct channel_gk20a *c);
#endif
