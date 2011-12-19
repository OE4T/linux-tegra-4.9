/*
 * drivers/video/tegra/host/t20/t20.c
 *
 * Tegra Graphics Init for T20 Architecture Chips
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/slab.h>
#include "../dev.h"

#include "t20.h"

static struct nvhost_device devices[] = {
	{.name   = "gr3d", .id = -1 },
	{.name = "gr2d", .id = -1 },
	{.name = "isp", .id = -1 },
	{.name = "vi", .id = -1 },
	{.name = "mpe", .id = -1 },
	{.name = "dsi", .id = -1 }
};

int nvhost_init_t20_support(struct nvhost_master *host)
{
	int err;
	int i;

	for (i = 0; i < ARRAY_SIZE(devices); i++)
		nvhost_device_register(&devices[i]);

	/* don't worry about cleaning up on failure... "remove" does it. */
	err = nvhost_init_t20_channel_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_cdma_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_debug_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_syncpt_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_intr_support(host);
	if (err)
		return err;
	err = nvhost_init_t20_cpuaccess_support(host);
	if (err)
		return err;
	return 0;
}

int nvhost_t20_save_context(struct nvhost_module *mod, u32 syncpt_id)
{
	struct nvhost_channel *ch =
			container_of(mod, struct nvhost_channel, mod);
	struct nvhost_hwctx *hwctx_to_save;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	u32 syncpt_incrs, syncpt_val;
	int err = 0;
	void *ref;
	void *ctx_waiter = NULL, *wakeup_waiter = NULL;
	struct nvhost_job *job;

	ctx_waiter = nvhost_intr_alloc_waiter();
	wakeup_waiter = nvhost_intr_alloc_waiter();
	if (!ctx_waiter || !wakeup_waiter) {
		err = -ENOMEM;
		goto done;
	}

	if (mod->desc->busy)
		mod->desc->busy(mod);

	mutex_lock(&ch->submitlock);
	hwctx_to_save = ch->cur_ctx;
	if (!hwctx_to_save) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	job = nvhost_job_alloc(ch, hwctx_to_save,
			NULL,
			ch->dev->nvmap, 0, 0);
	if (IS_ERR_OR_NULL(job)) {
		err = PTR_ERR(job);
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	hwctx_to_save->valid = true;
	ch->ctxhandler.get(hwctx_to_save);
	ch->cur_ctx = NULL;

	syncpt_incrs = hwctx_to_save->save_incrs;
	syncpt_val = nvhost_syncpt_incr_max(&ch->dev->syncpt,
					syncpt_id, syncpt_incrs);

	job->syncpt_id = syncpt_id;
	job->syncpt_incrs = syncpt_incrs;
	job->syncpt_end = syncpt_val;

	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	ch->ctxhandler.save_push(&ch->cdma, hwctx_to_save);
	nvhost_cdma_end(&ch->cdma, job);
	nvhost_job_put(job);
	job = NULL;

	err = nvhost_intr_add_action(&ch->dev->intr, syncpt_id,
			syncpt_val - syncpt_incrs + hwctx_to_save->save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctx_waiter,
			NULL);
	ctx_waiter = NULL;
	WARN(err, "Failed to set context save interrupt");

	err = nvhost_intr_add_action(&ch->dev->intr, syncpt_id, syncpt_val,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			wakeup_waiter,
			&ref);
	wakeup_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_min_cmp(&ch->dev->syncpt,
				syncpt_id, syncpt_val));

	nvhost_intr_put_ref(&ch->dev->intr, ref);

	nvhost_cdma_update(&ch->cdma);

	mutex_unlock(&ch->submitlock);

done:
	kfree(ctx_waiter);
	kfree(wakeup_waiter);
	return err;
}
