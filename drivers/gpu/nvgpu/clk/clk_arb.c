/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"

#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/nvgpu.h>
#include <linux/bitops.h>

#include "clk/clk_arb.h"

static int nvgpu_clk_arb_release_session_dev(struct inode *inode, struct file *filp);
static unsigned int nvgpu_clk_arb_poll_session_dev(struct file *filp, poll_table *wait);

static void nvgpu_clk_arb_run_arbiter_cb(struct work_struct *work);

struct nvgpu_clk_arb {
	struct mutex wlock;
	struct mutex users_lock;
	struct list_head users;
	u32 gpc2clk_current_mhz;
	u32 gpc2clk_target_mhz;
	u32 gpc2clk_default_mhz;
	u32 mclk_current_mhz;
	u32 mclk_target_mhz;
	u32 mclk_default_mhz;
	atomic_t usercount;
	struct work_struct update_fn_work;

	atomic_t req_nr;	/* for allocations */
	atomic_t last_req_nr;	/* last completed by arbiter */
};

struct nvgpu_clk_session {
	struct gk20a *g;
	int fd;
	atomic_t req_nr;
	struct kref refcount;
	wait_queue_head_t readout_wq;
	atomic_t poll_mask;
	struct list_head user;
	u32 gpc2clk_target_mhz;
	u32 mclk_target_mhz;
};

const struct file_operations clk_dev_ops = {
	.owner = THIS_MODULE,
	.release = nvgpu_clk_arb_release_session_dev,
	.poll = nvgpu_clk_arb_poll_session_dev,
};

int nvgpu_clk_arb_init_arbiter(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb;
	u16 default_mhz;
	int err;

	gk20a_dbg_fn("");

	if (!g->ops.clk_arb.get_arbiter_clk_domains)
		return 0;

	arb = kzalloc(sizeof(struct nvgpu_clk_arb), GFP_KERNEL);
	if (!arb)
		return -ENOMEM;

	g->clk_arb = arb;

	mutex_init(&arb->wlock);
	mutex_init(&arb->users_lock);

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			NVGPU_GPU_CLK_DOMAIN_MCLK, &default_mhz);
	if (err)
		return -EINVAL;

	arb->mclk_target_mhz = default_mhz;
	arb->mclk_current_mhz = default_mhz;
	arb->mclk_default_mhz = default_mhz;

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			NVGPU_GPU_CLK_DOMAIN_GPC2CLK, &default_mhz);
	if (err)
		return -EINVAL;

	arb->gpc2clk_target_mhz = default_mhz;
	arb->gpc2clk_current_mhz = default_mhz;
	arb->gpc2clk_default_mhz = default_mhz;

	atomic_set(&arb->usercount, 0);
	atomic_set(&arb->req_nr, 0);
	atomic_set(&arb->last_req_nr, 0);

	INIT_LIST_HEAD(&arb->users);
	INIT_WORK(&arb->update_fn_work, nvgpu_clk_arb_run_arbiter_cb);

	return 0;
}

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g)
{
	kfree(g->clk_arb);
}


int nvgpu_clk_arb_install_session_fd(struct gk20a *g,
		struct nvgpu_clk_session *session)
{
	struct file *file;
	char *name;
	int fd;
	int err;

	gk20a_dbg_fn("");

	if (session->fd >= 0)
		goto done;

	fd = get_unused_fd_flags(O_RDWR);
	if (fd < 0)
		return fd;

	name = kasprintf(GFP_KERNEL, "%s-clk-fd%d", dev_name(g->dev), fd);
	file = anon_inode_getfile(name, &clk_dev_ops, session, O_RDWR);
	kfree(name);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up_fd;
	}

	BUG_ON(file->private_data != session);

	fd_install(fd, file);
	kref_get(&session->refcount);

	session->fd = fd;
done:
	return session->fd;

clean_up_fd:
	put_unused_fd(fd);

	return err;
}

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_session *session = *(_session);

	gk20a_dbg_fn("");

	*_session = NULL;

	if (!g->ops.clk_arb.get_arbiter_clk_domains)
		return 0;

	session = kzalloc(sizeof(struct nvgpu_clk_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;
	session->g = g;
	session->fd = -1;

	kref_init(&session->refcount);
	init_waitqueue_head(&session->readout_wq);

	atomic_set(&session->poll_mask, 0);
	atomic_set(&session->req_nr, 0);

	mutex_lock(&arb->users_lock);
	list_add_tail(&session->user, &arb->users);
	mutex_unlock(&arb->users_lock);
	atomic_inc(&arb->usercount);

	mutex_lock(&arb->wlock);
	session->mclk_target_mhz = arb->mclk_default_mhz;
	session->gpc2clk_target_mhz = arb->gpc2clk_default_mhz;
	mutex_unlock(&arb->wlock);

	*_session = session;

	return 0;
}

void nvgpu_clk_arb_free_session(struct kref *refcount)
{
	struct nvgpu_clk_session *session = container_of(refcount,
			struct nvgpu_clk_session, refcount);
	struct gk20a *g = session->g;
	struct nvgpu_clk_arb *arb = g->clk_arb;

	mutex_lock(&arb->users_lock);
	list_del_init(&session->user);
	mutex_unlock(&arb->users_lock);

	if (atomic_dec_and_test(&arb->usercount))
		nvgpu_clk_arb_apply_session_constraints(g, NULL);

	kfree(session);
}

void nvgpu_clk_arb_cleanup_session(struct gk20a *g,
		struct nvgpu_clk_session *session)
{
	kref_put(&session->refcount, nvgpu_clk_arb_free_session);
}

static void nvgpu_clk_arb_run_arbiter_cb(struct work_struct *work)
{
	struct nvgpu_clk_arb *arb =
		container_of(work, struct nvgpu_clk_arb, update_fn_work);
	struct nvgpu_clk_session *session;

	mutex_lock(&arb->wlock);

	/* TODO: loop up higher or equal VF points */

	arb->mclk_current_mhz = arb->mclk_target_mhz;
	arb->gpc2clk_current_mhz = arb->gpc2clk_target_mhz;

	/* TODO: actually program the clocks */

	atomic_set(&arb->last_req_nr, atomic_read(&arb->req_nr));
	mutex_unlock(&arb->wlock);

	mutex_lock(&arb->users_lock);
	list_for_each_entry(session, &arb->users, user) {
		atomic_set(&session->poll_mask, POLLIN | POLLRDNORM);
		wake_up_interruptible(&session->readout_wq);
	}
	mutex_unlock(&arb->users_lock);

}

void nvgpu_clk_arb_apply_session_constraints(struct gk20a *g,
		struct nvgpu_clk_session *session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	mutex_lock(&arb->wlock);
	atomic_inc(&arb->req_nr);

	/* TODO: arbitration between users.
	   For now, last session to run arbiter wins.
	 */

	if (session) {
		arb->mclk_target_mhz = session->mclk_target_mhz;
		arb->gpc2clk_target_mhz = session->gpc2clk_target_mhz;

		atomic_set(&session->req_nr, atomic_read(&arb->req_nr));
	} else {
		arb->mclk_target_mhz = arb->mclk_default_mhz;
		arb->gpc2clk_target_mhz = arb->gpc2clk_default_mhz;
	}
	mutex_unlock(&arb->wlock);

	schedule_work(&arb->update_fn_work);
}

static unsigned int nvgpu_clk_arb_poll_session_dev(struct file *filp, poll_table *wait)
{
	struct nvgpu_clk_session *session = filp->private_data;

	gk20a_dbg_fn("");

	poll_wait(filp, &session->readout_wq, wait);
	return atomic_xchg(&session->poll_mask, 0);
}

static int nvgpu_clk_arb_release_session_dev(struct inode *inode, struct file *filp)
{
	struct nvgpu_clk_session *session = filp->private_data;
	struct gk20a *g = session->g;

	session->fd = -1;
	nvgpu_clk_arb_cleanup_session(g, session);

	return 0;
}

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 target_mhz)
{

	gk20a_dbg_fn("domain=0x%08x target_mhz=%u", api_domain, target_mhz);

	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		session->mclk_target_mhz = target_mhz;
		return 0;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		session->gpc2clk_target_mhz = target_mhz;
		return 0;

	default:
		return -EINVAL;
	}
}

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *target_mhz)
{
	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		*target_mhz = session->mclk_target_mhz;
		return 0;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		*target_mhz = session->gpc2clk_target_mhz;
		return 0;

	default:
		*target_mhz = 0;
		return -EINVAL;
	}
}

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *actual_mhz)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	int err = 0;

	mutex_lock(&arb->wlock);
	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		*actual_mhz = arb->mclk_current_mhz;
		break;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		*actual_mhz = arb->gpc2clk_current_mhz;
		break;

	default:
		*actual_mhz = 0;
		err = -EINVAL;
	}
	mutex_unlock(&arb->wlock);

	return err;
}

u32 nvgpu_clk_arb_get_arbiter_req_nr(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	return atomic_read(&arb->last_req_nr);
}

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	return g->ops.clk_arb.get_arbiter_clk_range(g, api_domain,
			min_mhz, max_mhz);
}

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g)
{
	return g->ops.clk_arb.get_arbiter_clk_domains(g);
}

u32 nvgpu_clk_arb_get_session_req_nr(struct gk20a *g,
		struct nvgpu_clk_session *session)
{
	return atomic_read(&session->req_nr);
}

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints)
{
	return (int)clk_domain_get_f_points(g, api_domain, max_points, fpoints);
}
