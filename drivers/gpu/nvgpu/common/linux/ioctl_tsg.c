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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <uapi/linux/nvgpu.h>
#include <linux/anon_inodes.h>

#include <nvgpu/kmem.h>
#include <nvgpu/log.h>

#include "gk20a/gk20a.h"
#include "gk20a/platform_gk20a.h"
#include "gk20a/tsg_gk20a.h"
#include "ioctl_tsg.h"
#include "ioctl_channel.h"
#include "os_linux.h"
#ifdef CONFIG_TEGRA_19x_GPU
#include "tsg_t19x.h"
#endif

struct tsg_private {
	struct gk20a *g;
	struct tsg_gk20a *tsg;
};

static int gk20a_tsg_bind_channel_fd(struct tsg_gk20a *tsg, int ch_fd)
{
	struct channel_gk20a *ch;
	int err;

	ch = gk20a_get_channel_from_file(ch_fd);
	if (!ch)
		return -EINVAL;

	err = ch->g->ops.fifo.tsg_bind_channel(tsg, ch);
	return err;
}

static int gk20a_tsg_get_event_data_from_id(struct tsg_gk20a *tsg,
				unsigned int event_id,
				struct gk20a_event_id_data **event_id_data)
{
	struct gk20a_event_id_data *local_event_id_data;
	bool event_found = false;

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_for_each_entry(local_event_id_data, &tsg->event_id_list,
					gk20a_event_id_data, event_id_node) {
		if (local_event_id_data->event_id == event_id) {
			event_found = true;
			break;
		}
	}
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	if (event_found) {
		*event_id_data = local_event_id_data;
		return 0;
	} else {
		return -1;
	}
}

void gk20a_tsg_event_id_post_event(struct tsg_gk20a *tsg,
				       int event_id)
{
	struct gk20a_event_id_data *event_id_data;
	int err = 0;

	err = gk20a_tsg_get_event_data_from_id(tsg, event_id,
						&event_id_data);
	if (err)
		return;

	nvgpu_mutex_acquire(&event_id_data->lock);

	gk20a_dbg_info(
		"posting event for event_id=%d on tsg=%d\n",
		event_id, tsg->tsgid);
	event_id_data->event_posted = true;

	wake_up_interruptible_all(&event_id_data->event_id_wq);

	nvgpu_mutex_release(&event_id_data->lock);
}

static int gk20a_tsg_event_id_enable(struct tsg_gk20a *tsg,
					 int event_id,
					 int *fd)
{
	int err = 0;
	int local_fd;
	struct file *file;
	char name[64];
	struct gk20a_event_id_data *event_id_data;
	struct gk20a *g;

	g = gk20a_get(tsg->g);
	if (!g)
		return -ENODEV;

	err = gk20a_tsg_get_event_data_from_id(tsg,
				event_id, &event_id_data);
	if (err == 0) {
		/* We already have event enabled */
		err = -EINVAL;
		goto free_ref;
	}

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		goto free_ref;
	local_fd = err;

	snprintf(name, sizeof(name), "nvgpu-event%d-fd%d",
		 event_id, local_fd);

	file = anon_inode_getfile(name, &gk20a_event_id_ops,
				  NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	event_id_data = nvgpu_kzalloc(tsg->g, sizeof(*event_id_data));
	if (!event_id_data) {
		err = -ENOMEM;
		goto clean_up_file;
	}
	event_id_data->g = g;
	event_id_data->id = tsg->tsgid;
	event_id_data->is_tsg = true;
	event_id_data->event_id = event_id;

	init_waitqueue_head(&event_id_data->event_id_wq);
	err = nvgpu_mutex_init(&event_id_data->lock);
	if (err)
		goto clean_up_free;

	nvgpu_init_list_node(&event_id_data->event_id_node);

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_add_tail(&event_id_data->event_id_node, &tsg->event_id_list);
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	fd_install(local_fd, file);
	file->private_data = event_id_data;

	*fd = local_fd;

	return 0;

clean_up_free:
	nvgpu_kfree(g, event_id_data);
clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(local_fd);
free_ref:
	gk20a_put(g);
	return err;
}

static int gk20a_tsg_event_id_ctrl(struct gk20a *g, struct tsg_gk20a *tsg,
		struct nvgpu_event_id_ctrl_args *args)
{
	int err = 0;
	int fd = -1;

	if (args->event_id >= NVGPU_IOCTL_CHANNEL_EVENT_ID_MAX)
		return -EINVAL;

	switch (args->cmd) {
	case NVGPU_IOCTL_CHANNEL_EVENT_ID_CMD_ENABLE:
		err = gk20a_tsg_event_id_enable(tsg, args->event_id, &fd);
		if (!err)
			args->event_fd = fd;
		break;

	default:
		nvgpu_err(tsg->g, "unrecognized tsg event id cmd: 0x%x",
			   args->cmd);
		err = -EINVAL;
		break;
	}

	return err;
}

int nvgpu_ioctl_tsg_open(struct gk20a *g, struct file *filp)
{
	struct tsg_private *priv;
	struct tsg_gk20a *tsg;
	struct device *dev;
	int err;

	g = gk20a_get(g);
	if (!g)
		return -ENODEV;

	dev  = dev_from_gk20a(g);

	gk20a_dbg(gpu_dbg_fn, "tsg: %s", dev_name(dev));

	priv = nvgpu_kmalloc(g, sizeof(*priv));
	if (!priv) {
		err = -ENOMEM;
		goto free_ref;
	}

	tsg = gk20a_tsg_open(g);
	if (!tsg) {
		nvgpu_kfree(g, priv);
		err = -ENOMEM;
		goto free_ref;
	}

	priv->g = g;
	priv->tsg = tsg;
	filp->private_data = priv;

	return 0;

free_ref:
	gk20a_put(g);
	return err;
}

int nvgpu_ioctl_tsg_dev_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l;
	int ret;

	l = container_of(inode->i_cdev,
			 struct nvgpu_os_linux, tsg.cdev);
	gk20a_dbg_fn("");
	ret = nvgpu_ioctl_tsg_open(&l->g, filp);
	gk20a_dbg_fn("done");
	return ret;
}

int nvgpu_ioctl_tsg_dev_release(struct inode *inode, struct file *filp)
{
	struct tsg_private *priv = filp->private_data;
	struct tsg_gk20a *tsg = priv->tsg;

	nvgpu_ref_put(&tsg->refcount, gk20a_tsg_release);
	nvgpu_kfree(tsg->g, priv);
	return 0;
}

static int gk20a_tsg_ioctl_set_priority(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_set_priority_args *arg)
{
	struct gk20a_sched_ctrl *sched = &g->sched_ctrl;
	int err;

	nvgpu_mutex_acquire(&sched->control_lock);
	if (sched->control_locked) {
		err = -EPERM;
		goto done;
	}

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu");
		goto done;
	}

	err = gk20a_tsg_set_priority(g, tsg, arg->priority);

	gk20a_idle(g);
done:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}

static int gk20a_tsg_ioctl_set_runlist_interleave(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_runlist_interleave_args *arg)
{
	struct gk20a_sched_ctrl *sched = &g->sched_ctrl;
	int err;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	nvgpu_mutex_acquire(&sched->control_lock);
	if (sched->control_locked) {
		err = -EPERM;
		goto done;
	}
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu");
		goto done;
	}

	err = gk20a_tsg_set_runlist_interleave(tsg, arg->level);

	gk20a_idle(g);
done:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}

static int gk20a_tsg_ioctl_set_timeslice(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_timeslice_args *arg)
{
	struct gk20a_sched_ctrl *sched = &g->sched_ctrl;
	int err;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	nvgpu_mutex_acquire(&sched->control_lock);
	if (sched->control_locked) {
		err = -EPERM;
		goto done;
	}
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu");
		goto done;
	}
	err = gk20a_tsg_set_timeslice(tsg, arg->timeslice_us);
	gk20a_idle(g);
done:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}

static int gk20a_tsg_ioctl_get_timeslice(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_timeslice_args *arg)
{
	arg->timeslice_us = gk20a_tsg_get_timeslice(tsg);
	return 0;
}

long nvgpu_ioctl_tsg_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct tsg_private *priv = filp->private_data;
	struct tsg_gk20a *tsg = priv->tsg;
	struct gk20a *g = tsg->g;
	u8 __maybe_unused buf[NVGPU_TSG_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	gk20a_dbg_fn("start %d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_TSG_IOCTL_MAGIC) ||
	    (_IOC_NR(cmd) == 0) ||
	    (_IOC_NR(cmd) > NVGPU_TSG_IOCTL_LAST) ||
	    (_IOC_SIZE(cmd) > NVGPU_TSG_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->gr.sw_ready) {
		err = gk20a_busy(g);
		if (err)
			return err;

		gk20a_idle(g);
	}

	switch (cmd) {
	case NVGPU_TSG_IOCTL_BIND_CHANNEL:
		{
		int ch_fd = *(int *)buf;
		if (ch_fd < 0) {
			err = -EINVAL;
			break;
		}
		err = gk20a_tsg_bind_channel_fd(tsg, ch_fd);
		break;
		}

	case NVGPU_TSG_IOCTL_UNBIND_CHANNEL:
		/* We do not support explicitly unbinding channel from TSG.
		 * Channel will be unbounded from TSG when it is closed.
		 */
		break;

	case NVGPU_IOCTL_TSG_ENABLE:
		{
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			return err;
		}
		gk20a_enable_tsg(tsg);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_DISABLE:
		{
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			return err;
		}
		gk20a_disable_tsg(tsg);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_PREEMPT:
		{
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			return err;
		}
		/* preempt TSG */
		err = g->ops.fifo.preempt_tsg(g, tsg->tsgid);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_SET_PRIORITY:
		{
		err = gk20a_tsg_ioctl_set_priority(g, tsg,
			(struct nvgpu_set_priority_args *)buf);
		break;
		}

	case NVGPU_IOCTL_TSG_EVENT_ID_CTRL:
		{
		err = gk20a_tsg_event_id_ctrl(g, tsg,
			(struct nvgpu_event_id_ctrl_args *)buf);
		break;
		}

	case NVGPU_IOCTL_TSG_SET_RUNLIST_INTERLEAVE:
		err = gk20a_tsg_ioctl_set_runlist_interleave(g, tsg,
			(struct nvgpu_runlist_interleave_args *)buf);
		break;

	case NVGPU_IOCTL_TSG_SET_TIMESLICE:
		{
		err = gk20a_tsg_ioctl_set_timeslice(g, tsg,
			(struct nvgpu_timeslice_args *)buf);
		break;
		}
	case NVGPU_IOCTL_TSG_GET_TIMESLICE:
		{
		err = gk20a_tsg_ioctl_get_timeslice(g, tsg,
			(struct nvgpu_timeslice_args *)buf);
		break;
		}

	default:
#ifdef CONFIG_TEGRA_19x_GPU
		err = t19x_tsg_ioctl_handler(g, tsg, cmd, buf);
#else
		nvgpu_err(g, "unrecognized tsg gpu ioctl cmd: 0x%x",
			   cmd);
		err = -ENOTTY;
#endif
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg,
				   buf, _IOC_SIZE(cmd));

	return err;
}
