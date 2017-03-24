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
#include <linux/nvhost.h>
#include <uapi/linux/nvgpu.h>
#include <linux/anon_inodes.h>

#include <nvgpu/kmem.h>

#include "gk20a.h"

#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>

struct tsg_private {
	struct gk20a *g;
	struct tsg_gk20a *tsg;
};

bool gk20a_is_channel_marked_as_tsg(struct channel_gk20a *ch)
{
	return !(ch->tsgid == NVGPU_INVALID_TSG_ID);
}

int gk20a_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	down_read(&tsg->ch_list_lock);
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		g->ops.fifo.enable_channel(ch);
	}
	up_read(&tsg->ch_list_lock);

	return 0;
}

int gk20a_disable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	down_read(&tsg->ch_list_lock);
	list_for_each_entry(ch, &tsg->ch_list, ch_entry) {
		g->ops.fifo.disable_channel(ch);
	}
	up_read(&tsg->ch_list_lock);

	return 0;
}

static bool gk20a_is_channel_active(struct gk20a *g, struct channel_gk20a *ch)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	unsigned int i;

	for (i = 0; i < f->max_runlists; ++i) {
		runlist = &f->runlist_info[i];
		if (test_bit(ch->hw_chid, runlist->active_channels))
			return true;
	}

	return false;
}

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

/*
 * API to mark channel as part of TSG
 *
 * Note that channel is not runnable when we bind it to TSG
 */
int gk20a_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch)
{
	gk20a_dbg_fn("");

	/* check if channel is already bound to some TSG */
	if (gk20a_is_channel_marked_as_tsg(ch)) {
		return -EINVAL;
	}

	/* channel cannot be bound to TSG if it is already active */
	if (gk20a_is_channel_active(tsg->g, ch)) {
		return -EINVAL;
	}

	ch->tsgid = tsg->tsgid;

	/* all the channel part of TSG should need to be same runlist_id */
	if (tsg->runlist_id == FIFO_INVAL_TSG_ID)
		tsg->runlist_id = ch->runlist_id;
	else if (tsg->runlist_id != ch->runlist_id) {
		gk20a_err(dev_from_gk20a(tsg->g),
			"Error: TSG channel should be share same runlist ch[%d] tsg[%d]\n",
			ch->runlist_id, tsg->runlist_id);
		return -EINVAL;
	}

	down_write(&tsg->ch_list_lock);
	list_add_tail(&ch->ch_entry, &tsg->ch_list);
	up_write(&tsg->ch_list_lock);

	kref_get(&tsg->refcount);

	gk20a_dbg(gpu_dbg_fn, "BIND tsg:%d channel:%d\n",
					tsg->tsgid, ch->hw_chid);

	gk20a_dbg_fn("done");
	return 0;
}

int gk20a_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct fifo_gk20a *f = &ch->g->fifo;
	struct tsg_gk20a *tsg = &f->tsg[ch->tsgid];

	down_write(&tsg->ch_list_lock);
	list_del_init(&ch->ch_entry);
	up_write(&tsg->ch_list_lock);

	kref_put(&tsg->refcount, gk20a_tsg_release);

	ch->tsgid = NVGPU_INVALID_TSG_ID;

	return 0;
}

int gk20a_init_tsg_support(struct gk20a *g, u32 tsgid)
{
	struct tsg_gk20a *tsg = NULL;
	int err;

	if (tsgid >= g->fifo.num_channels)
		return -EINVAL;

	tsg = &g->fifo.tsg[tsgid];

	tsg->in_use = false;
	tsg->tsgid = tsgid;

	INIT_LIST_HEAD(&tsg->ch_list);
	init_rwsem(&tsg->ch_list_lock);

	INIT_LIST_HEAD(&tsg->event_id_list);
	err = nvgpu_mutex_init(&tsg->event_id_list_lock);
	if (err) {
		tsg->in_use = true; /* make this TSG unusable */
		return err;
	}

	return 0;
}

static int gk20a_tsg_set_priority(struct gk20a *g, struct tsg_gk20a *tsg,
				u32 priority)
{
	u32 timeslice_us;

	switch (priority) {
	case NVGPU_PRIORITY_LOW:
		timeslice_us = g->timeslice_low_priority_us;
		break;
	case NVGPU_PRIORITY_MEDIUM:
		timeslice_us = g->timeslice_medium_priority_us;
		break;
	case NVGPU_PRIORITY_HIGH:
		timeslice_us = g->timeslice_high_priority_us;
		break;
	default:
		pr_err("Unsupported priority");
		return -EINVAL;
	}

	return gk20a_tsg_set_timeslice(tsg, timeslice_us);
}

static int gk20a_tsg_get_event_data_from_id(struct tsg_gk20a *tsg,
				unsigned int event_id,
				struct gk20a_event_id_data **event_id_data)
{
	struct gk20a_event_id_data *local_event_id_data;
	bool event_found = false;

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	list_for_each_entry(local_event_id_data, &tsg->event_id_list,
						 event_id_node) {
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

	INIT_LIST_HEAD(&event_id_data->event_id_node);

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	list_add_tail(&event_id_data->event_id_node, &tsg->event_id_list);
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	fd_install(local_fd, file);
	file->private_data = event_id_data;

	*fd = local_fd;

	return 0;

clean_up_free:
	kfree(event_id_data);
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
		gk20a_err(dev_from_gk20a(tsg->g),
			   "unrecognized tsg event id cmd: 0x%x",
			   args->cmd);
		err = -EINVAL;
		break;
	}

	return err;
}

int gk20a_tsg_set_runlist_interleave(struct tsg_gk20a *tsg, u32 level)
{
	struct gk20a *g = tsg->g;
	int ret;

	gk20a_dbg(gpu_dbg_sched, "tsgid=%u interleave=%u", tsg->tsgid, level);

	switch (level) {
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW:
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_MEDIUM:
	case NVGPU_RUNLIST_INTERLEAVE_LEVEL_HIGH:
		ret = g->ops.fifo.set_runlist_interleave(g, tsg->tsgid,
							true, 0, level);
		if (!ret)
			tsg->interleave_level = level;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? ret : g->ops.fifo.update_runlist(g, tsg->runlist_id, ~0, true, true);
}

int gk20a_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct gk20a *g = tsg->g;

	gk20a_dbg(gpu_dbg_sched, "tsgid=%u timeslice=%u us", tsg->tsgid, timeslice);

	return g->ops.fifo.tsg_set_timeslice(tsg, timeslice);
}

static void release_used_tsg(struct fifo_gk20a *f, struct tsg_gk20a *tsg)
{
	nvgpu_mutex_acquire(&f->tsg_inuse_mutex);
	f->tsg[tsg->tsgid].in_use = false;
	nvgpu_mutex_release(&f->tsg_inuse_mutex);
}

static struct tsg_gk20a *acquire_unused_tsg(struct fifo_gk20a *f)
{
	struct tsg_gk20a *tsg = NULL;
	unsigned int tsgid;

	nvgpu_mutex_acquire(&f->tsg_inuse_mutex);
	for (tsgid = 0; tsgid < f->num_channels; tsgid++) {
		if (!f->tsg[tsgid].in_use) {
			f->tsg[tsgid].in_use = true;
			tsg = &f->tsg[tsgid];
			break;
		}
	}
	nvgpu_mutex_release(&f->tsg_inuse_mutex);

	return tsg;
}

int gk20a_tsg_open(struct gk20a *g, struct file *filp)
{
	struct tsg_private *priv;
	struct tsg_gk20a *tsg;
	struct device *dev;
	int err;

	g = gk20a_get(g);
	if (!g)
		return -ENODEV;

	dev  = dev_from_gk20a(g);

	gk20a_dbg(gpu_dbg_fn, "tsg: %s", g->name);

	priv = nvgpu_kmalloc(g, sizeof(*priv));
	if (!priv) {
		err = -ENOMEM;
		goto free_ref;
	}

	tsg = acquire_unused_tsg(&g->fifo);
	if (!tsg) {
		nvgpu_kfree(g, priv);
		err = -ENOMEM;
		goto free_ref;
	}

	tsg->g = g;
	tsg->num_active_channels = 0;
	kref_init(&tsg->refcount);

	tsg->tsg_gr_ctx = NULL;
	tsg->vm = NULL;
	tsg->interleave_level = NVGPU_RUNLIST_INTERLEAVE_LEVEL_LOW;
	tsg->timeslice_us = 0;
	tsg->timeslice_timeout = 0;
	tsg->timeslice_scale = 0;
	tsg->runlist_id = ~0;
	tsg->tgid = current->tgid;

	priv->g = g;
	priv->tsg = tsg;
	filp->private_data = priv;

	if (g->ops.fifo.tsg_open) {
		err = g->ops.fifo.tsg_open(tsg);
		if (err) {
			gk20a_err(dev, "tsg %d fifo open failed %d",
				tsg->tsgid, err);
			goto clean_up;
		}
	}

	gk20a_dbg(gpu_dbg_fn, "tsg opened %d\n", tsg->tsgid);

	gk20a_sched_ctrl_tsg_added(g, tsg);

	return 0;

clean_up:
	kref_put(&tsg->refcount, gk20a_tsg_release);
free_ref:
	gk20a_put(g);
	return err;
}

int gk20a_tsg_dev_open(struct inode *inode, struct file *filp)
{
	struct gk20a *g;
	int ret;

	g = container_of(inode->i_cdev,
			 struct gk20a, tsg.cdev);
	gk20a_dbg_fn("");
	ret = gk20a_tsg_open(g, filp);
	gk20a_dbg_fn("done");
	return ret;
}

void gk20a_tsg_release(struct kref *ref)
{
	struct tsg_gk20a *tsg = container_of(ref, struct tsg_gk20a, refcount);
	struct gk20a *g = tsg->g;
	struct gk20a_event_id_data *event_id_data, *event_id_data_temp;

	if (tsg->tsg_gr_ctx) {
		gr_gk20a_free_tsg_gr_ctx(tsg);
		tsg->tsg_gr_ctx = NULL;
	}
	if (tsg->vm) {
		gk20a_vm_put(tsg->vm);
		tsg->vm = NULL;
	}

	gk20a_sched_ctrl_tsg_removed(g, tsg);

	/* unhook all events created on this TSG */
	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	list_for_each_entry_safe(event_id_data, event_id_data_temp,
				&tsg->event_id_list,
				event_id_node) {
		list_del_init(&event_id_data->event_id_node);
	}
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	release_used_tsg(&g->fifo, tsg);

	tsg->runlist_id = ~0;

	gk20a_dbg(gpu_dbg_fn, "tsg released %d\n", tsg->tsgid);
	gk20a_put(g);
}

int gk20a_tsg_dev_release(struct inode *inode, struct file *filp)
{
	struct tsg_private *priv = filp->private_data;
	struct tsg_gk20a *tsg = priv->tsg;

	kref_put(&tsg->refcount, gk20a_tsg_release);
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
		gk20a_err(dev_from_gk20a(g), "failed to power on gpu");
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
		gk20a_err(dev_from_gk20a(g), "failed to power on gpu");
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
		gk20a_err(dev_from_gk20a(g), "failed to power on gpu");
		goto done;
	}
	err = gk20a_tsg_set_timeslice(tsg, arg->timeslice_us);
	gk20a_idle(g);
done:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}


long gk20a_tsg_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct tsg_private *priv = filp->private_data;
	struct tsg_gk20a *tsg = priv->tsg;
	struct gk20a *g = tsg->g;
	u8 __maybe_unused buf[NVGPU_TSG_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	gk20a_dbg(gpu_dbg_fn, "");

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
			gk20a_err(g->dev,
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
			gk20a_err(g->dev,
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
			gk20a_err(g->dev,
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

	default:
		gk20a_err(dev_from_gk20a(g),
			   "unrecognized tsg gpu ioctl cmd: 0x%x",
			   cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg,
				   buf, _IOC_SIZE(cmd));

	return err;
}

void gk20a_init_tsg_ops(struct gpu_ops *gops)
{
	gops->fifo.tsg_bind_channel = gk20a_tsg_bind_channel;
	gops->fifo.tsg_unbind_channel = gk20a_tsg_unbind_channel;
}
