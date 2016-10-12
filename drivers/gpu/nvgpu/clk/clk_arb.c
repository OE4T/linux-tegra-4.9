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
#include <linux/spinlock.h>

#include "clk/clk_arb.h"


#define MAX_F_POINTS 127

#ifdef CONFIG_DEBUG_FS
static int nvgpu_clk_arb_debugfs_init(struct gk20a *g);
#endif

static int nvgpu_clk_arb_release_event_dev(struct inode *inode,
		struct file *filp);
static int nvgpu_clk_arb_release_completion_dev(struct inode *inode,
		struct file *filp);
static unsigned int nvgpu_clk_arb_poll_dev(struct file *filp, poll_table *wait);

static void nvgpu_clk_arb_run_arbiter_cb(struct work_struct *work);
static void nvgpu_clk_arb_run_vftable_cb(struct work_struct *work);
static int nvgpu_clk_arb_update_vftable(struct nvgpu_clk_arb *);

struct nvgpu_clk_vf_point {
	u16 mhz;
	u32 uvolt;
	u32 uvolt_sram;
};

struct nvgpu_clk_arb {
	spinlock_t sessions_lock;
	spinlock_t users_lock;
	spinlock_t req_lock;

	struct list_head users;
	struct list_head sessions;
	struct list_head requests;

	struct gk20a *g;
	spinlock_t data_lock;
	spinlock_t vf_lock;

	u16 gpc2clk_actual_mhz;
	u16 gpc2clk_default_mhz;

	u16 mclk_actual_mhz;
	u16 mclk_default_mhz;
	u32 voltuv_actual;

	struct work_struct update_fn_work;
	struct work_struct vftable_fn_work;
	wait_queue_head_t vftable_wq;

	u16 *mclk_f_points;
	bool vftable_set;

	struct nvgpu_clk_vf_point *mclk_vf_points;
	u32 mclk_f_numpoints;
	u16 *gpc2clk_f_points;
	u32 gpc2clk_f_numpoints;
	struct nvgpu_clk_vf_point *gpc2clk_vf_points;

#ifdef CONFIG_DEBUG_FS
	struct mutex debug_lock;
	s64 switch_max;
	s64 switch_min;
	u64 switch_num;
	s64 switch_avg;
	s64 switch_std;
	bool debugfs_set;
#endif
};


struct nvgpu_clk_dev {
	struct nvgpu_clk_session *session;
	struct list_head link;
	wait_queue_head_t readout_wq;
	atomic_t poll_mask;
	u16 gpc2clk_target_mhz;
	u16 mclk_target_mhz;
};

struct nvgpu_clk_session {
	bool zombie;
	struct gk20a *g;
	struct kref refcount;
	struct list_head link;
	struct list_head targets;

	spinlock_t target_lock;
	u16 gpc2clk_target_mhz;
	u16 mclk_target_mhz;
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
	u16 default_mhz;
	int err;

	gk20a_dbg_fn("");

	if (!g->ops.clk_arb.get_arbiter_clk_domains)
		return 0;

	arb = kzalloc(sizeof(struct nvgpu_clk_arb), GFP_KERNEL);
	if (!arb) {
		err = -ENOMEM;
		goto init_fail;
	}

	arb->gpc2clk_f_numpoints = MAX_F_POINTS;
	arb->mclk_f_numpoints = MAX_F_POINTS;

	arb->gpc2clk_f_points = kcalloc(MAX_F_POINTS, sizeof(u16), GFP_KERNEL);
	if (!arb->gpc2clk_f_points) {
		err = -ENOMEM;
		goto init_fail;
	}

	arb->mclk_f_points = kcalloc(MAX_F_POINTS, sizeof(u16), GFP_KERNEL);
	if (!arb->mclk_f_points) {
		err = -ENOMEM;
		goto init_fail;
	}

	arb->gpc2clk_vf_points = kcalloc(MAX_F_POINTS,
		sizeof(struct nvgpu_clk_vf_point), GFP_KERNEL);
	if (!arb->gpc2clk_vf_points) {
		err = -ENOMEM;
		goto init_fail;
	}

	arb->mclk_vf_points = kcalloc(MAX_F_POINTS,
		sizeof(struct nvgpu_clk_vf_point), GFP_KERNEL);
	if (!arb->mclk_vf_points) {
		err = -ENOMEM;
		goto init_fail;
	}

	g->clk_arb = arb;
	arb->g = g;

	spin_lock_init(&arb->sessions_lock);
	spin_lock_init(&arb->users_lock);
	spin_lock_init(&arb->req_lock);
	spin_lock_init(&arb->data_lock);
	spin_lock_init(&arb->vf_lock);

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			NVGPU_GPU_CLK_DOMAIN_MCLK, &default_mhz);
	if (err) {
		err = -EINVAL;
		goto init_fail;
	}

	arb->mclk_default_mhz = default_mhz;

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			NVGPU_GPU_CLK_DOMAIN_GPC2CLK, &default_mhz);
	if (err) {
		err = -EINVAL;
		goto init_fail;
	}

	arb->gpc2clk_default_mhz = default_mhz;

	INIT_LIST_HEAD(&arb->users);
	INIT_LIST_HEAD(&arb->sessions);
	INIT_LIST_HEAD(&arb->requests);

	init_waitqueue_head(&arb->vftable_wq);

	INIT_WORK(&arb->vftable_fn_work, nvgpu_clk_arb_run_vftable_cb);

	INIT_WORK(&arb->update_fn_work, nvgpu_clk_arb_run_arbiter_cb);

#ifdef CONFIG_DEBUG_FS
	mutex_init(&arb->debug_lock);
	if (!arb->debugfs_set) {
		if (nvgpu_clk_arb_debugfs_init(g))
			arb->debugfs_set = true;
	}
#endif
	err = nvgpu_clk_arb_update_vftable(arb);
	if (err < 0)
		goto init_fail;

	/* Schedule first run */
	schedule_work(&arb->update_fn_work);

	return 0;

init_fail:

	kfree(arb->gpc2clk_f_points);
	kfree(arb->gpc2clk_vf_points);

	kfree(arb->mclk_f_points);
	kfree(arb->mclk_vf_points);

	kfree(arb);

	return err;
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
	spin_lock_init(&session->target_lock);

	session->zombie = false;
	session->mclk_target_mhz = arb->mclk_default_mhz;
	session->gpc2clk_target_mhz = arb->gpc2clk_default_mhz;
	INIT_LIST_HEAD(&session->targets);

	spin_lock(&arb->sessions_lock);
	list_add_tail(&session->link, &arb->sessions);
	spin_unlock(&arb->sessions_lock);

	*_session = session;

	return 0;
}

void nvgpu_clk_arb_free_session(struct kref *refcount)
{
	struct nvgpu_clk_session *session = container_of(refcount,
			struct nvgpu_clk_session, refcount);
	struct nvgpu_clk_arb *arb = session->g->clk_arb;

	gk20a_dbg_fn("");

	spin_lock(&arb->sessions_lock);
	list_del(&session->link);
	spin_unlock(&arb->sessions_lock);
	kfree(session);
;
}

void nvgpu_clk_arb_release_session(struct gk20a *g,
	struct nvgpu_clk_session *session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	gk20a_dbg_fn("");

	session->zombie = true;
	kref_put(&session->refcount, nvgpu_clk_arb_free_session);

	schedule_work(&arb->update_fn_work);
}

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_dev *dev;
	int fd;

	gk20a_dbg_fn("");

	fd = nvgpu_clk_arb_install_fd(g, session, &event_dev_ops, &dev);
	if (fd < 0)
		return fd;

	spin_lock(&arb->users_lock);
	list_add_tail(&dev->link, &arb->users);
	spin_unlock(&arb->users_lock);

	*event_fd = fd;

	return 0;
}

int nvgpu_clk_arb_install_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *request_fd)
{
	struct nvgpu_clk_dev *dev;
	int fd;

	gk20a_dbg_fn("");

	fd = nvgpu_clk_arb_install_fd(g, session, &completion_dev_ops, &dev);
	if (fd < 0)
		return fd;

	*request_fd = fd;

	return 0;
}

static int nvgpu_clk_arb_update_vftable(struct nvgpu_clk_arb *arb)
{
	struct gk20a *g = arb->g;

	int i;
	int status = 0;
	u32 gpc2clk_voltuv = 0, mclk_voltuv = 0;
	u32 gpc2clk_voltuv_sram = 0, mclk_voltuv_sram = 0;

	/* the flag must be visible in all threads */
	mb();
	ACCESS_ONCE(arb->vftable_set) = false;

	spin_lock(&arb->vf_lock);

	if (!clk_domain_get_f_points(arb->g, NVGPU_GPU_CLK_DOMAIN_GPC2CLK,
		&arb->gpc2clk_f_numpoints, arb->gpc2clk_f_points) < 0) {
		gk20a_err(dev_from_gk20a(g),
			"failed to fetch GPC2CLK frequency points");
		goto exit_vftable;
	}
	if (clk_domain_get_f_points(arb->g, NVGPU_GPU_CLK_DOMAIN_MCLK,
		&arb->mclk_f_numpoints, arb->mclk_f_points) < 0) {
		gk20a_err(dev_from_gk20a(g),
			"failed to fetch MCLK frequency points");
		goto exit_vftable;
	}


	memset(arb->mclk_vf_points, 0,
		arb->mclk_f_numpoints*sizeof(struct nvgpu_clk_vf_point));
	memset(arb->gpc2clk_vf_points, 0,
		arb->gpc2clk_f_numpoints*sizeof(struct nvgpu_clk_vf_point));

	for (i = 0 ; i < arb->mclk_f_numpoints; i++) {
		arb->mclk_vf_points[i].mhz = arb->mclk_f_points[i];
		mclk_voltuv = mclk_voltuv_sram = 0;

		status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_MCLK,
			&arb->mclk_vf_points[i].mhz, &mclk_voltuv,
			CTRL_VOLT_DOMAIN_LOGIC);
		if (status < 0) {
			gk20a_err(dev_from_gk20a(g),
				"failed to get MCLK LOGIC voltage");
			goto exit_vftable;
		}
		status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_MCLK,
			&arb->mclk_vf_points[i].mhz, &mclk_voltuv_sram,
			CTRL_VOLT_DOMAIN_SRAM);
		if (status < 0) {
			gk20a_err(dev_from_gk20a(g),
				"failed to get MCLK SRAM voltage");
			goto exit_vftable;
		}

		arb->mclk_vf_points[i].uvolt = mclk_voltuv;
		arb->mclk_vf_points[i].uvolt_sram = mclk_voltuv_sram;
	}

	for (i = 0 ; i < arb->gpc2clk_f_numpoints; i++) {
		arb->gpc2clk_vf_points[i].mhz = arb->gpc2clk_f_points[i];
		gpc2clk_voltuv = gpc2clk_voltuv_sram = 0;

		status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_GPC2CLK,
			&arb->gpc2clk_vf_points[i].mhz, &gpc2clk_voltuv,
			CTRL_VOLT_DOMAIN_LOGIC);
		if (status < 0) {
			gk20a_err(dev_from_gk20a(g),
				"failed to get GPC2CLK LOGIC voltage");
			goto exit_vftable;
		}
		status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_GPC2CLK,
			&arb->gpc2clk_vf_points[i].mhz, &gpc2clk_voltuv_sram,
			CTRL_VOLT_DOMAIN_SRAM);
		if (status < 0) {
			gk20a_err(dev_from_gk20a(g),
				"failed to get GPC2CLK SRAM voltage");
			goto exit_vftable;
		}

		arb->gpc2clk_vf_points[i].uvolt = gpc2clk_voltuv;
		arb->gpc2clk_vf_points[i].uvolt_sram = gpc2clk_voltuv_sram;

	}

	/* make flag visible when all data has resolved in the tables */
	wmb();
	ACCESS_ONCE(arb->vftable_set) = true;

	wake_up(&arb->vftable_wq);
exit_vftable:

	spin_unlock(&arb->vf_lock);

	return status;
}

void nvgpu_clk_arb_schedule_vftable_update(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	ACCESS_ONCE(arb->vftable_set) = false;
	/* Disable the flag in case arbiter gets scheduled first */
	mb();

	schedule_work(&arb->vftable_fn_work);
	schedule_work(&arb->update_fn_work);
}

static void nvgpu_clk_arb_run_vftable_cb(struct work_struct *work)
{
	struct nvgpu_clk_arb *arb =
		container_of(work, struct nvgpu_clk_arb, update_fn_work);

	nvgpu_clk_arb_update_vftable(arb);
}

static void nvgpu_clk_arb_run_arbiter_cb(struct work_struct *work)
{
	struct nvgpu_clk_arb *arb =
		container_of(work, struct nvgpu_clk_arb, update_fn_work);
	struct nvgpu_clk_session *session;
	struct nvgpu_clk_dev *dev;
	struct nvgpu_clk_dev *tmp;
	struct gk20a *g = arb->g;

	struct change_fll_clk fllclk;
	u32 gpc2clk_voltuv = 0, mclk_voltuv = 0;
	u32 gpc2clk_voltuv_sram = 0, mclk_voltuv_sram = 0;

	u32 voltuv, voltuv_sram;

	int status;

	/* Temporary variables for checking target frequency */
	u16 gpc2clk_target, mclk_target;

	/* iteration index */
	u32 index;

#ifdef CONFIG_DEBUG_FS
	u64 t0, t1;
#endif

	gk20a_dbg_fn("");

#ifdef CONFIG_DEBUG_FS
	g->ops.read_ptimer(g, &t0);
#endif

	/* Only one arbiter should be running */
	gpc2clk_target = 0;
	mclk_target = 0;

	spin_lock(&arb->sessions_lock);
	list_for_each_entry(session, &arb->sessions, link) {
		if (!session->zombie) {
			spin_lock(&arb->req_lock);
			spin_lock(&session->target_lock);

			mclk_target = mclk_target > session->mclk_target_mhz ?
				mclk_target : session->mclk_target_mhz;

			gpc2clk_target =
				gpc2clk_target > session->gpc2clk_target_mhz ?
				gpc2clk_target : session->gpc2clk_target_mhz;
			/* Move processed requests to notification list*/
			list_for_each_entry_safe(dev, tmp, &session->targets,
						link) {
				list_del_init(&dev->link);
				list_add_tail(&dev->link, &arb->requests);
			}
			spin_unlock(&session->target_lock);
			spin_unlock(&arb->req_lock);

		}
	}
	spin_unlock(&arb->sessions_lock);

	gpc2clk_target = (gpc2clk_target > 0) ? gpc2clk_target :
		arb->gpc2clk_actual_mhz ? gpc2clk_target :
		arb->gpc2clk_default_mhz;

	mclk_target = (mclk_target > 0) ? mclk_target :
		arb->mclk_actual_mhz ? mclk_target :
		arb->mclk_default_mhz;

	if (!gpc2clk_target && !mclk_target) {
		mclk_target = arb->mclk_default_mhz;
		gpc2clk_target = arb->gpc2clk_default_mhz;
	}

	if (!gpc2clk_target)
		gpc2clk_target = arb->gpc2clk_actual_mhz;

	do {
		/* Check that the table is set */
		mb();
		wait_event(arb->vftable_wq, arb->vftable_set);
	} while (!ACCESS_ONCE(arb->vftable_set));

	spin_lock(&arb->vf_lock);
	/* round up the freq requests */
	for (index = 0; index < arb->gpc2clk_f_numpoints; index++) {
		if (arb->gpc2clk_vf_points[index].mhz >= gpc2clk_target) {
			gpc2clk_target = arb->gpc2clk_vf_points[index].mhz;
			gpc2clk_voltuv = arb->gpc2clk_vf_points[index].uvolt;
			gpc2clk_voltuv_sram =
				arb->gpc2clk_vf_points[index].uvolt_sram;
			break;
		}
	}

	if (index == arb->gpc2clk_f_numpoints) {
		gpc2clk_target = arb->gpc2clk_vf_points[index].mhz;
		gpc2clk_voltuv = arb->gpc2clk_vf_points[index].uvolt;
		gpc2clk_voltuv_sram =
			arb->gpc2clk_vf_points[index].uvolt_sram;
	}

	if (!mclk_target)
		mclk_target = arb->mclk_actual_mhz;

	for (index = 0; index < arb->mclk_f_numpoints; index++) {
		if (arb->mclk_vf_points[index].mhz >= mclk_target) {
			mclk_target = arb->mclk_vf_points[index].mhz;
			mclk_voltuv = arb->mclk_vf_points[index].uvolt;
			mclk_voltuv_sram =
				arb->mclk_vf_points[index].uvolt_sram;
			break;
		}
	}
	if (index == arb->mclk_f_numpoints) {
		mclk_target = arb->mclk_vf_points[index].mhz;
		mclk_voltuv = arb->mclk_vf_points[index].uvolt;
		mclk_voltuv_sram =
			arb->mclk_vf_points[index].uvolt_sram;
	}
	spin_unlock(&arb->vf_lock);

	/* Program clocks */
	/* A change in both mclk of gpc2clk may require a change in voltage */
	if ((arb->gpc2clk_actual_mhz == gpc2clk_target) &&
		(arb->mclk_actual_mhz == mclk_target)) {
		goto exit_arb;
	}

	voltuv = gpc2clk_voltuv > mclk_voltuv ? gpc2clk_voltuv : mclk_voltuv;
	voltuv_sram = gpc2clk_voltuv_sram > mclk_voltuv_sram ?
		gpc2clk_voltuv_sram : mclk_voltuv_sram;

	/* if voltage ascends we do:
	 * (1) FLL change
	 * (2) Voltage change
	 * (3) MCLK change
	 * If it goes down
	 * (1) MCLK change
	 * (2) Voltage change
	 * (3) FLL change
	 */

	/* descending */
	if (voltuv <= arb->voltuv_actual) {
		status = g->clk_pmu.clk_mclk.change(g, mclk_target);
		if (status < 0)
			goto exit_arb;

		status = volt_set_voltage(g, voltuv, voltuv_sram);
		if (status < 0)
			goto exit_arb;

		fllclk.api_clk_domain = CTRL_CLK_DOMAIN_GPC2CLK;
		fllclk.clkmhz = gpc2clk_target;
		fllclk.voltuv = voltuv;
		status = clk_program_fll_clks(g, &fllclk);
		if (status < 0)
			goto exit_arb;
	} else {
		fllclk.api_clk_domain = CTRL_CLK_DOMAIN_GPC2CLK;
		fllclk.clkmhz = gpc2clk_target;
		fllclk.voltuv = voltuv;
		status = clk_program_fll_clks(g, &fllclk);
		if (status < 0)
			goto exit_arb;

		status = volt_set_voltage(g, voltuv, voltuv_sram);
		if (status < 0)
			goto exit_arb;

		status = g->clk_pmu.clk_mclk.change(g, mclk_target);
		if (status < 0)
			goto exit_arb;
	}

	spin_lock(&arb->data_lock);
	arb->gpc2clk_actual_mhz = gpc2clk_target;
	arb->mclk_actual_mhz = mclk_target;
	arb->voltuv_actual = voltuv;
	/* Make changes visible to other threads */
	wmb();

	spin_unlock(&arb->data_lock);

#ifdef CONFIG_DEBUG_FS
	g->ops.read_ptimer(g, &t1);
	arb->switch_num++;

	mutex_lock(&arb->debug_lock);
	if (arb->switch_num == 1) {
		arb->switch_max = arb->switch_min =
			arb->switch_avg = (t1-t0)/1000;
		arb->switch_std = 0;
	} else {
		s64 prev_avg;
		u64 curr = (t1-t0)/1000;

		arb->switch_max = curr > arb->switch_max ?
			curr : arb->switch_max;
		arb->switch_min = arb->switch_min ?
			(curr < arb->switch_min ?
				curr : arb->switch_min) : curr;
		prev_avg = arb->switch_avg;
		arb->switch_avg = (curr +
			(arb->switch_avg * (arb->switch_num-1))) /
			arb->switch_num;
		arb->switch_std +=
			(curr - arb->switch_avg) * (curr - prev_avg);
	}
	mutex_unlock(&arb->debug_lock);

#endif

exit_arb:

	spin_lock(&arb->req_lock);
	/* notify completion for all requests */
	list_for_each_entry_safe(dev, tmp, &arb->requests, link) {
		atomic_set(&dev->poll_mask, POLLIN | POLLRDNORM);
		wake_up_interruptible(&dev->readout_wq);
		list_del_init(&dev->link);
	}
	spin_unlock(&arb->req_lock);

	/* notify event for all users */
	spin_lock(&arb->users_lock);
	list_for_each_entry(dev, &arb->users, link) {
		atomic_set(&dev->poll_mask, POLLIN | POLLRDNORM);
		wake_up_interruptible(&dev->readout_wq);
	}
	spin_unlock(&arb->users_lock);
}

int nvgpu_clk_arb_commit_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int request_fd)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_dev *dev;
	struct fd fd;
	int err = 0;

	gk20a_dbg_fn("");

	fd  = fdget(request_fd);

	if (!fd.file)
		return -EINVAL;

	dev = (struct nvgpu_clk_dev *) fd.file->private_data;

	if (!dev || dev->session != session) {
		err = -EINVAL;
		goto fdput_fd;
	}
	spin_lock(&session->target_lock);
	session->mclk_target_mhz = dev->mclk_target_mhz ? dev->mclk_target_mhz :
		session->mclk_target_mhz;
	session->gpc2clk_target_mhz = dev->gpc2clk_target_mhz ?
		dev->gpc2clk_target_mhz :
		session->gpc2clk_target_mhz;

	list_add_tail(&dev->link, &session->targets);
	spin_unlock(&session->target_lock);

	schedule_work(&arb->update_fn_work);

fdput_fd:
	fdput(fd);
	return err;
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
	struct nvgpu_clk_arb *arb;

	arb = session->g->clk_arb;

	gk20a_dbg_fn("");

	spin_lock(&arb->req_lock);
	spin_lock(&session->target_lock);
	if (!list_empty(&dev->link))
		list_del_init(&dev->link);
	spin_unlock(&session->target_lock);
	spin_unlock(&arb->req_lock);

	kref_put(&session->refcount, nvgpu_clk_arb_free_session);
	kfree(dev);

	return 0;
}

static int nvgpu_clk_arb_release_event_dev(struct inode *inode,
		struct file *filp)
{
	struct nvgpu_clk_dev *dev = filp->private_data;
	struct nvgpu_clk_session *session = dev->session;
	struct nvgpu_clk_arb *arb;

	arb = session->g->clk_arb;

	gk20a_dbg_fn("");

	spin_lock(&arb->users_lock);
	list_del(&dev->link);
	spin_unlock(&arb->users_lock);

	kref_put(&session->refcount, nvgpu_clk_arb_free_session);
	kfree(dev);

	return 0;
}

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		int request_fd, u32 api_domain, u16 target_mhz)
{
	struct nvgpu_clk_dev *dev;
	struct fd fd;
	int err = 0;

	gk20a_dbg_fn("domain=0x%08x target_mhz=%u", api_domain, target_mhz);

	fd = fdget(request_fd);

	if (!fd.file)
		return -EINVAL;

	dev = fd.file->private_data;
	if (!dev || dev->session != session) {
		err = -EINVAL;
		goto fdput_fd;
	}

	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		dev->mclk_target_mhz = target_mhz;
		break;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		dev->gpc2clk_target_mhz = target_mhz;
		break;

	default:
		err = -EINVAL;
	}

fdput_fd:
	fdput(fd);
	return err;
}

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *freq_mhz)
{
	int err = 0;

	spin_lock(&session->target_lock);

	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		*freq_mhz = session->mclk_target_mhz;
		break;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		*freq_mhz = session->gpc2clk_target_mhz;
		break;

	default:
		*freq_mhz = 0;
		err = -EINVAL;
	}

	spin_unlock(&session->target_lock);
	return err;
}

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *freq_mhz)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	int err = 0;

	spin_lock(&arb->data_lock);

	switch (api_domain) {
	case NVGPU_GPU_CLK_DOMAIN_MCLK:
		*freq_mhz = arb->mclk_actual_mhz;
		break;

	case NVGPU_GPU_CLK_DOMAIN_GPC2CLK:
		*freq_mhz = arb->gpc2clk_actual_mhz;
		break;

	default:
		*freq_mhz = 0;
		err = -EINVAL;
	}

	spin_unlock(&arb->data_lock);
	return err;
}

int nvgpu_clk_arb_get_arbiter_effective_mhz(struct gk20a *g,
		u32 api_domain, u16 *freq_mhz)
{

	*freq_mhz = g->ops.clk.get_rate(g, api_domain);
	return 0;
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

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints)
{
	return (int)clk_domain_get_f_points(g, api_domain, max_points, fpoints);
}

#ifdef CONFIG_DEBUG_FS
static int nvgpu_clk_arb_stats_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	struct nvgpu_clk_arb *arb = g->clk_arb;
	u64 num;
	s64 tmp, avg, std, max, min;

	/* Make copy of structure to reduce time with lock held */
	mutex_lock(&arb->debug_lock);
	std = arb->switch_std;
	avg = arb->switch_avg;
	max = arb->switch_max;
	min = arb->switch_min;
	num = arb->switch_num;
	mutex_unlock(&arb->debug_lock);

	tmp = std;
	do_div(tmp, num);
	seq_printf(s, "Number of transitions: %lld\n",
		num);
	seq_printf(s, "max / min : %lld / %lld usec\n",
		max, min);
	seq_printf(s, "avg / std : %lld / %ld usec\n",
		avg, int_sqrt(tmp));

	return 0;
}

static int nvgpu_clk_arb_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvgpu_clk_arb_stats_show, inode->i_private);
}

static const struct file_operations nvgpu_clk_arb_stats_fops = {
	.open		= nvgpu_clk_arb_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int nvgpu_clk_arb_debugfs_init(struct gk20a *g)
{
	struct gk20a_platform *platform = dev_get_drvdata(g->dev);

	struct dentry *gpu_root = platform->debugfs;
	struct dentry *d;

	gk20a_dbg(gpu_dbg_info, "g=%p", g);

	d = debugfs_create_file(
			"arb_stats",
			S_IRUGO,
			gpu_root,
			g,
			&nvgpu_clk_arb_stats_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}
#endif
