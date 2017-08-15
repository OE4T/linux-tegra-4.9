/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/file.h>
#include <linux/version.h>
#include <linux/fs.h>

#include <nvgpu/semaphore.h>
#include <nvgpu/kmem.h>
#include <nvgpu/soc.h>
#include <nvgpu/nvhost.h>

#include "gk20a.h"
#include "channel_gk20a.h"
#include "sync_gk20a.h"

#ifdef CONFIG_SYNC
#include "../drivers/staging/android/sync.h"
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
	struct gk20a *g = f->g;

#ifdef CONFIG_SYNC
	if (f->sync_fence)
		sync_fence_put(f->sync_fence);
#endif
	if (f->semaphore)
		nvgpu_semaphore_put(f->semaphore);

	if (f->allocator) {
		if (nvgpu_alloc_initialized(f->allocator))
			nvgpu_free(f->allocator, (u64)(uintptr_t)f);
	} else
		nvgpu_kfree(g, f);
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

static inline bool gk20a_fence_is_valid(struct gk20a_fence *f)
{
	bool valid = f->valid;

	rmb();
	return valid;
}

int gk20a_fence_wait(struct gk20a *g, struct gk20a_fence *f,
							unsigned long timeout)
{
	if (f && gk20a_fence_is_valid(f)) {
		if (!nvgpu_platform_is_silicon(g))
			timeout = MAX_SCHEDULE_TIMEOUT;
		return f->ops->wait(f, timeout);
	}
	return 0;
}

bool gk20a_fence_is_expired(struct gk20a_fence *f)
{
	if (f && gk20a_fence_is_valid(f) && f->ops)
		return f->ops->is_expired(f);
	else
		return true;
}

int gk20a_fence_install_fd(struct gk20a_fence *f)
{
#ifdef CONFIG_SYNC
	int fd;

	if (!f || !gk20a_fence_is_valid(f) || !f->sync_fence)
		return -EINVAL;

	fd = get_unused_fd_flags(O_RDWR);
	if (fd < 0)
		return fd;

	sync_fence_get(f->sync_fence);
	sync_fence_install(f->sync_fence, fd);
	return fd;
#else
	return -ENODEV;
#endif
}

int gk20a_alloc_fence_pool(struct channel_gk20a *c, unsigned int count)
{
	int err;
	size_t size;
	struct gk20a_fence *fence_pool = NULL;

	size = sizeof(struct gk20a_fence);
	if (count <= UINT_MAX / size) {
		size = count * size;
		fence_pool = nvgpu_vzalloc(c->g, size);
	}

	if (!fence_pool)
		return -ENOMEM;

	err = nvgpu_lockless_allocator_init(c->g, &c->fence_allocator,
				"fence_pool", (size_t)fence_pool, size,
				sizeof(struct gk20a_fence), 0);
	if (err)
		goto fail;

	return 0;

fail:
	nvgpu_vfree(c->g, fence_pool);
	return err;
}

void gk20a_free_fence_pool(struct channel_gk20a *c)
{
	if (nvgpu_alloc_initialized(&c->fence_allocator)) {
		void *base = (void *)(uintptr_t)
				nvgpu_alloc_base(&c->fence_allocator);

		nvgpu_alloc_destroy(&c->fence_allocator);
		nvgpu_vfree(c->g, base);
	}
}

struct gk20a_fence *gk20a_alloc_fence(struct channel_gk20a *c)
{
	struct gk20a_fence *fence = NULL;

	if (channel_gk20a_is_prealloc_enabled(c)) {
		if (nvgpu_alloc_initialized(&c->fence_allocator)) {
			fence = (struct gk20a_fence *)(uintptr_t)
				nvgpu_alloc(&c->fence_allocator,
					sizeof(struct gk20a_fence));

			/* clear the node and reset the allocator pointer */
			if (fence) {
				memset(fence, 0, sizeof(*fence));
				fence->allocator = &c->fence_allocator;
			}
		}
	} else
		fence = nvgpu_kzalloc(c->g, sizeof(struct gk20a_fence));

	if (fence) {
		kref_init(&fence->ref);
		fence->g = c->g;
	}

	return fence;
}

void gk20a_init_fence(struct gk20a_fence *f,
		const struct gk20a_fence_ops *ops,
		struct sync_fence *sync_fence, bool wfi)
{
	if (!f)
		return;
	f->ops = ops;
	f->sync_fence = sync_fence;
	f->wfi = wfi;
	f->syncpt_id = -1;
}

/* Fences that are backed by GPU semaphores: */

static int nvgpu_semaphore_fence_wait(struct gk20a_fence *f, long timeout)
{
	if (!nvgpu_semaphore_is_acquired(f->semaphore))
		return 0;

	return NVGPU_COND_WAIT_INTERRUPTIBLE(
		f->semaphore_wq,
		!nvgpu_semaphore_is_acquired(f->semaphore),
		timeout);
}

static bool nvgpu_semaphore_fence_is_expired(struct gk20a_fence *f)
{
	return !nvgpu_semaphore_is_acquired(f->semaphore);
}

static const struct gk20a_fence_ops nvgpu_semaphore_fence_ops = {
	.wait = &nvgpu_semaphore_fence_wait,
	.is_expired = &nvgpu_semaphore_fence_is_expired,
};

/* This function takes ownership of the semaphore */
int gk20a_fence_from_semaphore(
		struct gk20a *g,
		struct gk20a_fence *fence_out,
		struct sync_timeline *timeline,
		struct nvgpu_semaphore *semaphore,
		struct nvgpu_cond *semaphore_wq,
		bool wfi, bool need_sync_fence)
{
	struct gk20a_fence *f = fence_out;
	struct sync_fence *sync_fence = NULL;

#ifdef CONFIG_SYNC
	if (need_sync_fence) {
		sync_fence = gk20a_sync_fence_create(g, timeline, semaphore,
					"f-gk20a-0x%04x",
					nvgpu_semaphore_gpu_ro_va(semaphore));
		if (!sync_fence)
			return -1;
	}
#endif

	gk20a_init_fence(f, &nvgpu_semaphore_fence_ops, sync_fence, wfi);
	if (!f) {
#ifdef CONFIG_SYNC
		if (sync_fence)
			sync_fence_put(sync_fence);
#endif
		return -EINVAL;
	}

	f->semaphore = semaphore;
	f->semaphore_wq = semaphore_wq;

	/* commit previous writes before setting the valid flag */
	wmb();
	f->valid = true;

	return 0;
}

#ifdef CONFIG_TEGRA_GK20A_NVHOST
/* Fences that are backed by host1x syncpoints: */

static int gk20a_syncpt_fence_wait(struct gk20a_fence *f, long timeout)
{
	return nvgpu_nvhost_syncpt_wait_timeout_ext(
			f->nvhost_dev, f->syncpt_id, f->syncpt_value,
			(u32)timeout, NULL, NULL);
}

static bool gk20a_syncpt_fence_is_expired(struct gk20a_fence *f)
{

	/*
	 * In cases we don't register a notifier, we can't expect the
	 * syncpt value to be updated. For this case, we force a read
	 * of the value from HW, and then check for expiration.
	 */
	if (!nvgpu_nvhost_syncpt_is_expired_ext(f->nvhost_dev, f->syncpt_id,
				f->syncpt_value)) {
		u32 val;

		if (!nvgpu_nvhost_syncpt_read_ext_check(f->nvhost_dev,
				f->syncpt_id, &val)) {
			return nvgpu_nvhost_syncpt_is_expired_ext(
					f->nvhost_dev,
					f->syncpt_id, f->syncpt_value);
		}
	}

	return true;
}

static const struct gk20a_fence_ops gk20a_syncpt_fence_ops = {
	.wait = &gk20a_syncpt_fence_wait,
	.is_expired = &gk20a_syncpt_fence_is_expired,
};

int gk20a_fence_from_syncpt(
		struct gk20a_fence *fence_out,
		struct nvgpu_nvhost_dev *nvhost_dev,
		u32 id, u32 value, bool wfi,
		bool need_sync_fence)
{
	struct gk20a_fence *f = fence_out;
	struct sync_fence *sync_fence = NULL;

#ifdef CONFIG_SYNC
	if (need_sync_fence) {
		sync_fence = nvgpu_nvhost_sync_create_fence(nvhost_dev,
					id, value, 1, "fence");
		if (IS_ERR(sync_fence))
			return -1;
	}
#endif

	gk20a_init_fence(f, &gk20a_syncpt_fence_ops, sync_fence, wfi);
	if (!f) {
#ifdef CONFIG_SYNC
		if (sync_fence)
			sync_fence_put(sync_fence);
#endif
		return -EINVAL;
	}
	f->nvhost_dev = nvhost_dev;
	f->syncpt_id = id;
	f->syncpt_value = value;

	/* commit previous writes before setting the valid flag */
	wmb();
	f->valid = true;

	return 0;
}
#else
int gk20a_fence_from_syncpt(
		struct gk20a_fence *fence_out,
		struct nvgpu_nvhost_dev *nvhost_dev,
		u32 id, u32 value, bool wfi,
		bool need_sync_fence)
{
	return -EINVAL;
}
#endif
