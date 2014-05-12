/*
 * drivers/video/tegra/host/gk20a/channel_sync_gk20a.c
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

#include <linux/gk20a.h>

#include "channel_sync_gk20a.h"
#include "gk20a.h"

#ifdef CONFIG_SYNC
#include "../../../staging/android/sync.h"
#endif

#ifdef CONFIG_TEGRA_GK20A
#include <linux/nvhost.h>
#endif

#ifdef CONFIG_TEGRA_GK20A

struct gk20a_channel_syncpt {
	struct gk20a_channel_sync ops;
	struct channel_gk20a *c;
	struct platform_device *host1x_pdev;
	u32 id;
};

static void add_wait_cmd(u32 *ptr, u32 id, u32 thresh)
{
	/* syncpoint_a */
	ptr[0] = 0x2001001C;
	/* payload */
	ptr[1] = thresh;
	/* syncpoint_b */
	ptr[2] = 0x2001001D;
	/* syncpt_id, switch_en, wait */
	ptr[3] = (id << 8) | 0x10;
}

int gk20a_channel_syncpt_wait_cpu(struct gk20a_channel_sync *s,
				  struct gk20a_channel_fence *fence,
				  int timeout)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	if (!fence->valid)
		return 0;
	return nvhost_syncpt_wait_timeout_ext(
			sp->host1x_pdev, sp->id, fence->thresh,
			timeout, NULL, NULL);
}

bool gk20a_channel_syncpt_is_expired(struct gk20a_channel_sync *s,
				     struct gk20a_channel_fence *fence)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	if (!fence->valid)
		return true;
	return nvhost_syncpt_is_expired_ext(sp->host1x_pdev, sp->id,
			fence->thresh);
}

int gk20a_channel_syncpt_wait_syncpt(struct gk20a_channel_sync *s, u32 id,
		u32 thresh, struct priv_cmd_entry **entry)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	struct priv_cmd_entry *wait_cmd = NULL;

	if (id >= nvhost_syncpt_nb_pts_ext(sp->host1x_pdev)) {
		dev_warn(dev_from_gk20a(sp->c->g),
				"invalid wait id in gpfifo submit, elided");
		return 0;
	}

	if (nvhost_syncpt_is_expired_ext(sp->host1x_pdev, id, thresh))
		return 0;

	gk20a_channel_alloc_priv_cmdbuf(sp->c, 4, &wait_cmd);
	if (wait_cmd == NULL) {
		gk20a_err(dev_from_gk20a(sp->c->g),
				"not enough priv cmd buffer space");
		return -EAGAIN;
	}

	add_wait_cmd(&wait_cmd->ptr[0], id, thresh);

	*entry = wait_cmd;
	return 0;
}

int gk20a_channel_syncpt_wait_fd(struct gk20a_channel_sync *s, int fd,
		       struct priv_cmd_entry **entry)
{
#ifdef CONFIG_SYNC
	int i;
	int num_wait_cmds;
	struct sync_pt *pt;
	struct sync_fence *sync_fence;
	struct priv_cmd_entry *wait_cmd = NULL;
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	struct channel_gk20a *c = sp->c;

	sync_fence = nvhost_sync_fdget(fd);
	if (!sync_fence)
		return -EINVAL;

	/* validate syncpt ids */
	list_for_each_entry(pt, &sync_fence->pt_list_head, pt_list) {
		u32 wait_id = nvhost_sync_pt_id(pt);
		if (!wait_id ||
			 wait_id >= nvhost_syncpt_nb_pts_ext(sp->host1x_pdev)) {
			sync_fence_put(sync_fence);
			return -EINVAL;
		}
	}

	num_wait_cmds = nvhost_sync_num_pts(sync_fence);
	gk20a_channel_alloc_priv_cmdbuf(c, 4 * num_wait_cmds, &wait_cmd);
	if (wait_cmd == NULL) {
		gk20a_err(dev_from_gk20a(c->g),
				"not enough priv cmd buffer space");
		sync_fence_put(sync_fence);
		return -EAGAIN;
	}

	i = 0;
	list_for_each_entry(pt, &sync_fence->pt_list_head, pt_list) {
		u32 wait_id = nvhost_sync_pt_id(pt);
		u32 wait_value = nvhost_sync_pt_thresh(pt);

		if (nvhost_syncpt_is_expired_ext(sp->host1x_pdev,
				wait_id, wait_value)) {
			wait_cmd->ptr[i * 4 + 0] = 0;
			wait_cmd->ptr[i * 4 + 1] = 0;
			wait_cmd->ptr[i * 4 + 2] = 0;
			wait_cmd->ptr[i * 4 + 3] = 0;
		} else
			add_wait_cmd(&wait_cmd->ptr[i * 4], wait_id,
					wait_value);
		i++;
	}
	WARN_ON(i != num_wait_cmds);
	sync_fence_put(sync_fence);

	*entry = wait_cmd;
	return 0;
#else
	return -ENODEV;
#endif
}

static void gk20a_channel_syncpt_update(void *priv, int nr_completed)
{
	struct channel_gk20a *ch20a = priv;
	gk20a_channel_update(ch20a, nr_completed);
}

static int __gk20a_channel_syncpt_incr(struct gk20a_channel_sync *s,
				       bool gfx_class, bool wfi_cmd,
				       bool register_irq,
				       struct priv_cmd_entry **entry,
				       struct gk20a_channel_fence *fence)
{
	u32 thresh;
	int incr_cmd_size;
	int j = 0;
	int err;
	struct priv_cmd_entry *incr_cmd = NULL;
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	struct channel_gk20a *c = sp->c;

	incr_cmd_size = 4;
	if (wfi_cmd)
		incr_cmd_size += 2;

	gk20a_channel_alloc_priv_cmdbuf(c, incr_cmd_size, &incr_cmd);
	if (incr_cmd == NULL) {
		gk20a_err(dev_from_gk20a(c->g),
				"not enough priv cmd buffer space");
		return -EAGAIN;
	}

	if (gfx_class) {
		WARN_ON(wfi_cmd); /* No sense to use gfx class + wfi. */
		/* setobject KEPLER_C */
		incr_cmd->ptr[j++] = 0x20010000;
		incr_cmd->ptr[j++] = KEPLER_C;
		/* syncpt incr */
		incr_cmd->ptr[j++] = 0x200100B2;
		incr_cmd->ptr[j++] = sp->id |
			(0x1 << 20) | (0x1 << 16);
	} else {
		if (wfi_cmd) {
			/* wfi */
			incr_cmd->ptr[j++] = 0x2001001E;
			/* handle, ignored */
			incr_cmd->ptr[j++] = 0x00000000;
		}
		/* syncpoint_a */
		incr_cmd->ptr[j++] = 0x2001001C;
		/* payload, ignored */
		incr_cmd->ptr[j++] = 0;
		/* syncpoint_b */
		incr_cmd->ptr[j++] = 0x2001001D;
		/* syncpt_id, incr */
		incr_cmd->ptr[j++] = (sp->id << 8) | 0x1;
	}
	WARN_ON(j != incr_cmd_size);

	thresh = nvhost_syncpt_incr_max_ext(sp->host1x_pdev, sp->id, 1);

	if (register_irq) {
		/* nvhost action_gpfifo_submit_complete releases this ref. */
		err = gk20a_channel_busy(c->g->dev);

		if (!err) {
			err = nvhost_intr_register_notifier(sp->host1x_pdev,
					sp->id, thresh,
					gk20a_channel_syncpt_update, c);
			if (err)
				gk20a_channel_idle(c->g->dev);
		}

		/* Adding interrupt action should never fail. A proper error
		 * handling here would require us to decrement the syncpt max
		 * back to its original value. */
		WARN(err, "failed to set submit complete interrupt");
	}

	fence->thresh = thresh;
	fence->valid = true;
	fence->wfi = wfi_cmd;
	*entry = incr_cmd;
	return 0;
}

int gk20a_channel_syncpt_incr_wfi(struct gk20a_channel_sync *s,
				  struct priv_cmd_entry **entry,
				  struct gk20a_channel_fence *fence)
{
	return __gk20a_channel_syncpt_incr(s,
			false /* use host class */,
			true /* wfi */,
			false /* no irq handler */,
			entry, fence);
}

int gk20a_channel_syncpt_incr(struct gk20a_channel_sync *s,
			      struct priv_cmd_entry **entry,
			      struct gk20a_channel_fence *fence)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	/* Don't put wfi cmd to this one since we're not returning
	 * a fence to user space. */
	return __gk20a_channel_syncpt_incr(s,
			sp->c->obj_class == KEPLER_C /* may use gfx class */,
			false /* no wfi */,
			true /* register irq */,
			entry, fence);
}

int gk20a_channel_syncpt_incr_user_syncpt(struct gk20a_channel_sync *s,
					  struct priv_cmd_entry **entry,
					  struct gk20a_channel_fence *fence,
					  u32 *id, u32 *thresh)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	/* Need to do 'host incr + wfi' or 'gfx incr' since we return the fence
	 * to user space. */
	int err = __gk20a_channel_syncpt_incr(s,
			sp->c->obj_class == KEPLER_C /* use gfx class? */,
			sp->c->obj_class != KEPLER_C /* wfi if host class */,
			true /* register irq */,
			entry, fence);
	if (err)
		return err;
	*id = sp->id;
	*thresh = fence->thresh;
	return 0;
}

int gk20a_channel_syncpt_incr_user_fd(struct gk20a_channel_sync *s,
				      struct priv_cmd_entry **entry,
				      struct gk20a_channel_fence *fence,
				      int *fd)
{
#ifdef CONFIG_SYNC
	int err;
	struct nvhost_ctrl_sync_fence_info pt;
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	err = gk20a_channel_syncpt_incr_user_syncpt(s, entry, fence,
						    &pt.id, &pt.thresh);
	if (err)
		return err;
	return nvhost_sync_create_fence_fd(sp->host1x_pdev, &pt, 1,
					   "fence", fd);
#else
	return -ENODEV;
#endif
}

void gk20a_channel_syncpt_set_min_eq_max(struct gk20a_channel_sync *s)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	nvhost_syncpt_set_min_eq_max_ext(sp->host1x_pdev, sp->id);
}

static void gk20a_channel_syncpt_destroy(struct gk20a_channel_sync *s)
{
	struct gk20a_channel_syncpt *sp =
		container_of(s, struct gk20a_channel_syncpt, ops);
	nvhost_free_syncpt(sp->id);
	kfree(sp);
}

static struct gk20a_channel_sync *
gk20a_channel_syncpt_create(struct channel_gk20a *c)
{
	struct gk20a_channel_syncpt *sp;

	sp = kzalloc(sizeof(*sp), GFP_KERNEL);
	if (!sp)
		return NULL;

	sp->c = c;
	sp->host1x_pdev = c->g->host1x_dev;
	sp->id = nvhost_get_syncpt_host_managed(sp->host1x_pdev, c->hw_chid);

	sp->ops.wait_cpu		= gk20a_channel_syncpt_wait_cpu;
	sp->ops.is_expired		= gk20a_channel_syncpt_is_expired;
	sp->ops.wait_syncpt		= gk20a_channel_syncpt_wait_syncpt;
	sp->ops.wait_fd			= gk20a_channel_syncpt_wait_fd;
	sp->ops.incr			= gk20a_channel_syncpt_incr;
	sp->ops.incr_wfi		= gk20a_channel_syncpt_incr_wfi;
	sp->ops.incr_user_syncpt	= gk20a_channel_syncpt_incr_user_syncpt;
	sp->ops.incr_user_fd		= gk20a_channel_syncpt_incr_user_fd;
	sp->ops.set_min_eq_max		= gk20a_channel_syncpt_set_min_eq_max;
	sp->ops.destroy			= gk20a_channel_syncpt_destroy;

	sp->ops.syncpt_aggressive_destroy = true;

	return &sp->ops;
}
#endif /* CONFIG_TEGRA_GK20A */

struct gk20a_channel_sync *gk20a_channel_sync_create(struct channel_gk20a *c)
{
#ifdef CONFIG_TEGRA_GK20A
	if (gk20a_platform_has_syncpoints(c->g->dev))
		return gk20a_channel_syncpt_create(c);
#endif
	WARN_ON(1);
	return NULL;
}
