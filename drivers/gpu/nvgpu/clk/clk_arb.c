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

static int nvgpu_clk_arb_release_event_dev(struct inode *inode,
		struct file *filp);
static int nvgpu_clk_arb_release_completion_dev(struct inode *inode,
		struct file *filp);
static unsigned int nvgpu_clk_arb_poll_dev(struct file *filp, poll_table *wait);

static void nvgpu_clk_arb_run_arbiter_cb(struct work_struct *work);

struct nvgpu_clk_arb {
	struct mutex req_lock;
	struct mutex users_lock;
	struct list_head users;
	struct list_head requests;

	u64 gpc2clk_current_hz;
	u64 gpc2clk_target_hz;
	u64 gpc2clk_default_hz;
	u64 mclk_current_hz;
	u64 mclk_target_hz;
	u64 mclk_default_hz;
	atomic_t usercount;
	struct work_struct update_fn_work;
};


struct nvgpu_clk_dev {
	struct nvgpu_clk_session *session;
	struct list_head link;
	wait_queue_head_t readout_wq;
	atomic_t poll_mask;
};

struct nvgpu_clk_session {
	bool zombie;
	struct gk20a *g;
	struct kref refcount;

	u64 gpc2clk_target_hz;
	u64 mclk_target_hz;
};

static const struct file_operations completion_dev_ops = {
	.owner = THIS_MODULE,
	.release = nvgpu_clk_arb_release_completion_dev,
	.poll = nvgpu_clk_arb_poll_dev,
};

static const struct file_operations event_dev_ops = {
	.owner = THIS_MODULE,
	.release = nvgpu_clk_arb_release_event_dev,
	.poll = nvgpu_clk_arb_poll_dev,
};

int nvgpu_clk_arb_init_arbiter(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb;
	u64 default_hz;
	int err;

	gk20a_dbg_fn("");

	if (!g->ops.clk_arb.get_arbiter_clk_domains)
		return 0;

	arb = kzalloc(sizeof(struct nvgpu_clk_arb), GFP_KERNEL);
	if (!arb)
		return -ENOMEM;

	g->clk_arb = arb;

	mutex_init(&arb->req_lock);
	mutex_init(&arb->users_lock);

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			NVGPU_GPU_CLK_DOMAIN_MCLK, &default_hz);
	if (err)
		return -EINVAL;

	arb->mclk_target_hz = default_hz;
	arb->mclk_current_hz = default_hz;
	arb->mclk_default_hz = default_hz;

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			NVGPU_GPU_CLK_DOMAIN_GPC2CLK, &default_hz);
	if (err)
		return -EINVAL;

	arb->gpc2clk_target_hz = default_hz;
	arb->gpc2clk_current_hz = default_hz;
	arb->gpc2clk_default_hz = default_hz;

	atomic_set(&arb->usercount, 0);

	INIT_LIST_HEAD(&arb->users);
	INIT_LIST_HEAD(&arb->requests);
	INIT_WORK(&arb->update_fn_work, nvgpu_clk_arb_run_arbiter_cb);

	return 0;
}

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g)
{
	kfree(g->clk_arb);
}

static int nvgpu_clk_arb_install_fd(struct gk20a *g,
		struct nvgpu_clk_session *session,
		const struct file_operations *fops,
		struct nvgpu_clk_dev **_dev)
{
	struct file *file;
	char *name;
	int fd;
	int err;
	struct nvgpu_clk_dev *dev;

	gk20a_dbg_fn("");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	fd = get_unused_fd_flags(O_RDWR);
	if (fd < 0)
		return fd;

	name = kasprintf(GFP_KERNEL, "%s-clk-fd%d", dev_name(g->dev), fd);
	file = anon_inode_getfile(name, fops, dev, O_RDWR);
	kfree(name);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto fail;
	}

	fd_install(fd, file);

	init_waitqueue_head(&dev->readout_wq);
	atomic_set(&dev->poll_mask, 0);

	dev->session = session;
	kref_get(&session->refcount);

	*_dev = dev;

	return fd;

fail:
	kfree(dev);
	put_unused_fd(fd);
	return err;
}

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_session *session = *(_session);

	gk20a_dbg_fn("");

	if (!g->ops.clk_arb.get_arbiter_clk_domains)
		return 0;

	session = kzalloc(sizeof(struct nvgpu_clk_session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;
	session->g = g;

	kref_init(&session->refcount);

	atomic_inc(&arb->usercount);

	session->zombie = false;
	session->mclk_target_hz = arb->mclk_default_hz;
	session->gpc2clk_target_hz = arb->gpc2clk_default_hz;

	*_session = session;

	return 0;
}

void nvgpu_clk_arb_free_session(struct kref *refcount)
{
	struct nvgpu_clk_session *session = container_of(refcount,
			struct nvgpu_clk_session, refcount);

	kfree(session);
}

void nvgpu_clk_arb_release_session(struct gk20a *g,
	struct nvgpu_clk_session *session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	session->zombie = true;
	kref_put(&session->refcount, nvgpu_clk_arb_free_session);

	/* schedule arbiter if no more user */
	if (!atomic_dec_and_test(&arb->usercount))
		schedule_work(&arb->update_fn_work);
}

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_dev *dev;
	int fd;

	fd = nvgpu_clk_arb_install_fd(g, session, &event_dev_ops, &dev);
	if (fd < 0)
		return fd;

	mutex_lock(&arb->users_lock);
	list_add_tail(&dev->link, &arb->users);
	mutex_unlock(&arb->users_lock);

	*event_fd = fd;

	return 0;
}

static void nvgpu_clk_arb_run_arbiter_cb(struct work_struct *work)
{
	struct nvgpu_clk_arb *arb =
		container_of(work, struct nvgpu_clk_arb, update_fn_work);
	struct nvgpu_clk_session *session;
	struct nvgpu_clk_dev *dev;
	struct nvgpu_clk_dev *tmp;

	mutex_lock(&arb->req_lock);

	arb->mclk_target_hz = arb->mclk_default_hz;
	arb->gpc2clk_target_hz = arb->gpc2clk_default_hz;

	list_for_each_entry(dev, &arb->requests, link) {
		session = dev->session;
		if (!session->zombie) {
			/* TODO: arbiter policy. For now last request wins */

			arb->mclk_target_hz = session->mclk_target_hz;
			arb->gpc2clk_target_hz = session->gpc2clk_target_hz;
		}
	}

	/* TODO: loop up higher or equal VF points */

	arb->mclk_current_hz = arb->mclk_target_hz;
	arb->gpc2clk_current_hz = arb->gpc2clk_target_hz;

	/* TODO: actually program the clocks */

	/* notify completion for all requests */
	list_for_each_entry_safe(dev, tmp, &arb->requests, link) {
		atomic_set(&dev->poll_mask, POLLIN | POLLRDNORM);
		wake_up_interruptible(&dev->readout_wq);
		list_del_init(&dev->link);
	}
	mutex_unlock(&arb->req_lock);

	/* notify event for all users */
	mutex_lock(&arb->users_lock);
	list_for_each_entry(dev, &arb->users, link) {
		atomic_set(&dev->poll_mask, POLLIN | POLLRDNORM);
		wake_up_interruptible(&dev->readout_wq);
	}
	mutex_unlock(&arb->users_lock);

}

int nvgpu_clk_arb_apply_session_constraints(struct gk20a *g,
		struct nvgpu_clk_session *session, int *completion_fd)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_dev *dev;
	int fd;

	fd = nvgpu_clk_arb_install_fd(g, session, &completion_dev_ops, &dev);
	if (fd < 0)
		return fd;

	*completion_fd = fd;

	mutex_lock(&arb->req_lock);
	list_add_tail(&dev->link, &arb->requests);
	mutex_unlock(&arb->req_lock);

	schedule_work(&arb->update_fn_work);

	return 0;
}

static unsigned int nvgpu_clk_arb_poll_dev(struct file *filp, poll_table *wait)
{
	struct nvgpu_clk_dev *dev = filp->private_data;

	gk20a_dbg_fn("");

	poll_wait(filp, &dev->readout_wq, wait);
	return atomic_xchg(&dev->poll_mask, 0);
}

static int nvgpu_clk_arb_release_completion_dev(struct inode *inode,
		struct file *filp)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct nvgpu_clk_session *session = dev->session;

	gk20a_dbg_fn("");

	kref_put(&session->refcount, nvgpu_clk_arb_free_session);
	kfree(dev);
	return 0;
}

static int nvgpu_clk_arb_release_event_dev(struct inode *inode,
		struct file *filp)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct nvgpu_clk_session *session = dev->session;
	struct nvgpu_clk_arb *arb = session->g->clk_arb;

	gk20a_dbg_fn("");

	mutex_lock(&arb->users_lock);
	list_del_init(&dev->link);
	mutex_unlock(&arb->users_lock);

	kref_put(&session->refcount, nvgpu_clk_arb_free_session);
	kfree(dev);
	return 0;
}

int nvgpu_clk_arb_set_session_target_hz(struct nvgpu_clk_session *session,
		u32 api_domain, u64 target_hz)
{

	gk20a_dbg_fn("domain=0x%08x target_hz=%llu", api_domain, target_hz);

	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		session->mclk_target_hz = target_hz;
		return 0;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		session->gpc2clk_target_hz = target_hz;
		return 0;

	default:
		return -EINVAL;
	}
}

int nvgpu_clk_arb_get_session_target_hz(struct nvgpu_clk_session *session,
		u32 api_domain, u64 *freq_hz)
{
	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		*freq_hz = session->mclk_target_hz;
		return 0;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		*freq_hz = session->gpc2clk_target_hz;
		return 0;

	default:
		*freq_hz = 0;
		return -EINVAL;
	}
}

int nvgpu_clk_arb_get_arbiter_actual_hz(struct gk20a *g,
		u32 api_domain, u64 *freq_hz)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	int err = 0;

	mutex_lock(&arb->req_lock);
	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		*freq_hz = arb->mclk_current_hz;
		break;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		*freq_hz = arb->gpc2clk_current_hz;
		break;

	default:
		*freq_hz = 0;
		err = -EINVAL;
	}
	mutex_unlock(&arb->req_lock);

	return err;
}

int nvgpu_clk_arb_get_arbiter_effective_hz(struct gk20a *g,
		u32 api_domain, u64 *freq_hz)
{
	/* TODO: measure clocks from counters */
	return nvgpu_clk_arb_get_arbiter_actual_hz(g, api_domain, freq_hz);
}

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u64 *min_hz, u64 *max_hz)
{
	return g->ops.clk_arb.get_arbiter_clk_range(g, api_domain,
			min_hz, max_hz);
}

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g)
{
	return g->ops.clk_arb.get_arbiter_clk_domains(g);
}

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints)
{
	return (int)clk_domain_get_f_points(g, api_domain, max_points, fpoints);
}
