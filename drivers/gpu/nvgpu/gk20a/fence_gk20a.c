/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include "fence_gk20a.h"

#include <linux/gk20a.h>
#include <linux/file.h>

#include "gk20a.h"
#include "semaphore_gk20a.h"
#include "channel_gk20a.h"
#include "sync_gk20a.h"

#ifdef CONFIG_SYNC
#include "../../../staging/android/sync.h"
#endif

#ifdef CONFIG_TEGRA_GK20A
#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>
#endif

struct gk20a_fence_ops {
	int (*wait)(struct gk20a_fence *, long timeout);
	bool (*is_expired)(struct gk20a_fence *);
	void *(*free)(struct kref *);
};

static void gk20a_fence_free(struct kref *ref)
{
	struct gk20a_fence *f =
		container_of(ref, struct gk20a_fence, ref);
#ifdef CONFIG_SYNC
	if (f->sync_fence)
		sync_fence_put(f->sync_fence);
#endif
	if (f->semaphore)
		gk20a_semaphore_put(f->semaphore);
	kfree(f);
}

void gk20a_fence_put(struct gk20a_fence *f)
{
	if (f)
		kref_put(&f->ref, gk20a_fence_free);
}

struct gk20a_fence *gk20a_fence_get(struct gk20a_fence *f)
{
	if (f)
		kref_get(&f->ref);
	return f;
}

int gk20a_fence_wait(struct gk20a_fence *f, int timeout)
{
	if (!tegra_platform_is_silicon())
		timeout = (u32)MAX_SCHEDULE_TIMEOUT;
	return f->ops->wait(f, timeout);
}

bool gk20a_fence_is_expired(struct gk20a_fence *f)
{
	if (f && f->ops)
		return f->ops->is_expired(f);
	else
		return true;
}

int gk20a_fence_install_fd(struct gk20a_fence *f)
{
#ifdef CONFIG_SYNC
	int fd;

	if (!f->sync_fence)
		return -EINVAL;

	fd = get_unused_fd();
	if (fd < 0)
		return fd;

	sync_fence_get(f->sync_fence);
	sync_fence_install(f->sync_fence, fd);
	return fd;
#else
	return -ENODEV;
#endif
}

static struct gk20a_fence *alloc_fence(const struct gk20a_fence_ops *ops,
				struct sync_fence *sync_fence, bool wfi)
{
	struct gk20a_fence *f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return NULL;
	kref_init(&f->ref);
	f->ops = ops;
	f->sync_fence = sync_fence;
	f->wfi = wfi;
	f->syncpt_id = -1;
	return f;
}

/* Fences that are backed by GPU semaphores: */

static int gk20a_semaphore_fence_wait(struct gk20a_fence *f, long timeout)
{
	long remain;

	if (!gk20a_semaphore_is_acquired(f->semaphore))
		return 0;

	remain = wait_event_interruptible_timeout(
		*f->semaphore_wq,
		!gk20a_semaphore_is_acquired(f->semaphore),
		timeout);
	if (remain == 0 && gk20a_semaphore_is_acquired(f->semaphore))
		return -ETIMEDOUT;
	else if (remain < 0)
		return remain;
	return 0;
}

static bool gk20a_semaphore_fence_is_expired(struct gk20a_fence *f)
{
	return !gk20a_semaphore_is_acquired(f->semaphore);
}

static const struct gk20a_fence_ops gk20a_semaphore_fence_ops = {
	.wait = &gk20a_semaphore_fence_wait,
	.is_expired = &gk20a_semaphore_fence_is_expired,
};

/* This function takes ownership of the semaphore */
struct gk20a_fence *gk20a_fence_from_semaphore(
		struct sync_timeline *timeline,
		struct gk20a_semaphore *semaphore,
		wait_queue_head_t *semaphore_wq,
		struct sync_fence *dependency,
		bool wfi)
{
	struct gk20a_fence *f;
	struct sync_fence *sync_fence = NULL;

#ifdef CONFIG_SYNC
	sync_fence = gk20a_sync_fence_create(timeline, semaphore,
					     dependency, "fence");
	if (!sync_fence)
		return NULL;
#endif

	f  = alloc_fence(&gk20a_semaphore_fence_ops, sync_fence, wfi);
	if (!f) {
#ifdef CONFIG_SYNC
		sync_fence_put(sync_fence);
#endif
		return NULL;
	}

	f->semaphore = semaphore;
	f->semaphore_wq = semaphore_wq;
	return f;
}

#ifdef CONFIG_TEGRA_GK20A
/* Fences that are backed by host1x syncpoints: */

static int gk20a_syncpt_fence_wait(struct gk20a_fence *f, long timeout)
{
	return nvhost_syncpt_wait_timeout_ext(
			f->host1x_pdev, f->syncpt_id, f->syncpt_value,
			(u32)timeout, NULL, NULL);
}

static bool gk20a_syncpt_fence_is_expired(struct gk20a_fence *f)
{
	return nvhost_syncpt_is_expired_ext(f->host1x_pdev, f->syncpt_id,
					    f->syncpt_value);
}

static const struct gk20a_fence_ops gk20a_syncpt_fence_ops = {
	.wait = &gk20a_syncpt_fence_wait,
	.is_expired = &gk20a_syncpt_fence_is_expired,
};

struct gk20a_fence *gk20a_fence_from_syncpt(struct platform_device *host1x_pdev,
					    u32 id, u32 value, bool wfi,
					    bool need_sync_fence)
{
	struct gk20a_fence *f;
	struct sync_fence *sync_fence = NULL;

#ifdef CONFIG_SYNC
	struct nvhost_ctrl_sync_fence_info pt = {
		.id = id,
		.thresh = value
	};

	if (need_sync_fence) {
		sync_fence = nvhost_sync_create_fence(host1x_pdev, &pt, 1,
						      "fence");
		if (IS_ERR(sync_fence))
			return NULL;
	}
#endif

	f = alloc_fence(&gk20a_syncpt_fence_ops, sync_fence, wfi);
	if (!f) {
#ifdef CONFIG_SYNC
		sync_fence_put(sync_fence);
#endif
		return NULL;
	}
	f->host1x_pdev = host1x_pdev;
	f->syncpt_id = id;
	f->syncpt_value = value;
	return f;
}
#else
struct gk20a_fence *gk20a_fence_from_syncpt(struct platform_device *host1x_pdev,
					    u32 id, u32 value, bool wfi)
{
	return NULL;
}
#endif
