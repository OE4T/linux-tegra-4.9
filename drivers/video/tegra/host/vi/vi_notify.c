/*
 * VI NOTIFY driver for T186
 *
 * Copyright (c) 2015 NVIDIA Corporation.  All rights reserved.
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

#include <asm/ioctls.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "vi_notify.h"

/* XXX: move ioctls to include/linux/ (after T18X merge) */
#include <linux/nvhost_vi_ioctl.h>
#define NVHOST_VI_SET_IGN_MASK _IOW(NVHOST_VI_IOCTL_MAGIC, 10, u32)
#define NVHOST_VI_SET_PRI_MASK _IOW(NVHOST_VI_IOCTL_MAGIC, 11, u32)

struct vi_notify_channel {
	struct vi_notify_dev *vnd;
	u32 ign_mask;
	u32 pri_mask;

	wait_queue_head_t readq;
	struct mutex read_lock;
	struct rcu_head rcu;

	atomic_t overruns;
	atomic_t errors;
	DECLARE_KFIFO(fifo, struct vi_notify_msg, 256);
};

struct vi_notify_dev {
	struct vi_notify_driver *driver;
	struct device *device;
	dev_t major;
	u8 num_channels;
	struct mutex lock;
	struct vi_notify_channel __rcu *channel;
};

static int vi_notify_dev_classify(struct vi_notify_dev *vnd)
{
	struct vi_notify_channel *chan;
	u32 ign_mask = 0xffffffff, pri_mask = 0;

	chan = rcu_access_pointer(vnd->channel);

	if (chan != NULL) {
		ign_mask &= chan->ign_mask;
		pri_mask |= chan->pri_mask;
	}

	WARN_ON(ign_mask & pri_mask);
	return vnd->driver->classify(vnd->device, ign_mask, pri_mask);
}

static int vi_notify_classify(struct vi_notify_channel *chan,
				u32 ign_mask, u32 pri_mask)
{
	u32 old_ign_mask, old_pri_mask;
	int err;

	old_ign_mask = chan->ign_mask;
	old_pri_mask = chan->pri_mask;
	chan->ign_mask = ign_mask;
	chan->pri_mask = pri_mask;

	err = vi_notify_dev_classify(chan->vnd);
	if (err) {
		chan->ign_mask = old_ign_mask;
		chan->pri_mask = old_pri_mask;
	}
	return err;
}

static unsigned vi_notify_occupancy(struct vi_notify_channel *chan)
{
	unsigned ret;

	mutex_lock(&chan->read_lock);
	ret = kfifo_len(&chan->fifo);
	mutex_unlock(&chan->read_lock);

	return ret;
}

/* Interrupt handlers */
void vi_notify_dev_error(struct vi_notify_dev *vnd)
{
	struct vi_notify_channel *chan;

	rcu_read_lock();
	chan = rcu_dereference(vnd->channel);

	if (chan != NULL) {
		atomic_set(&chan->errors, 1);
		wake_up(&chan->readq);
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL(vi_notify_dev_error);

void vi_notify_dev_recv(struct vi_notify_dev *vnd,
			const struct vi_notify_msg *msg)
{
	struct vi_notify_channel *chan;
	u8 channel = VI_NOTIFY_TAG_CHANNEL(msg->tag);

	if (channel >= vnd->num_channels) {
		dev_warn(vnd->device, "Channel %u out of range!\n", channel);
		return;
	}

	dev_dbg(vnd->device, "Message: tag:%2u channel:%02X frame:%04X\n",
		VI_NOTIFY_TAG_TAG(msg->tag), channel,
		VI_NOTIFY_TAG_FRAME(msg->tag));
	dev_dbg(vnd->device, "         timestamp %u data 0x%08x",
		msg->stamp, msg->data);

	rcu_read_lock();
	chan = rcu_dereference(vnd->channel);

	if (chan != NULL) {
		if (!kfifo_put(&chan->fifo, *msg))
			atomic_set(&chan->overruns, 1);
		wake_up(&chan->readq);
	}
	rcu_read_unlock();
}
EXPORT_SYMBOL(vi_notify_dev_recv);

/* File operations */
static ssize_t vi_notify_read(struct file *file, char __user *buf, size_t len,
				loff_t *offset)
{
	struct vi_notify_channel *chan = file->private_data;

	for (;;) {
		DEFINE_WAIT(wait);
		unsigned int copied;
		int ret = 0;

		if (len < sizeof(struct vi_notify_msg))
			return 0;
		if (mutex_lock_interruptible(&chan->read_lock))
			return -ERESTARTSYS;

		ret = kfifo_to_user(&chan->fifo, buf, len, &copied);
		mutex_unlock(&chan->read_lock);

		if (ret)
			return ret;
		if (copied > 0)
			return copied;

		prepare_to_wait(&chan->readq, &wait, TASK_INTERRUPTIBLE);

		if (atomic_xchg(&chan->overruns, 0))
			ret = -EOVERFLOW;
		else if (atomic_xchg(&chan->errors, 0))
			ret = -EIO;
		else if (signal_pending(current))
			ret = -ERESTARTSYS;
		else if (file->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else if (!vi_notify_occupancy(chan))
			schedule();

		finish_wait(&chan->readq, &wait);

		if (ret)
			return ret;
	}
}

static unsigned int vi_notify_poll(struct file *file,
					struct poll_table_struct *table)
{
	struct vi_notify_channel *chan = file->private_data;
	unsigned ret = 0;

	poll_wait(file, &chan->readq, table);

	if (vi_notify_occupancy(chan))
		ret |= POLLIN | POLLRDNORM;
	if (atomic_read(&chan->overruns) || atomic_read(&chan->errors))
		ret |= POLLERR;

	return ret;
}

static long vi_notify_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct vi_notify_channel *chan = file->private_data;
	struct vi_notify_dev *vnd = chan->vnd;

	switch (cmd) {
	case FIONREAD: {
		int val;

		val = vi_notify_occupancy(chan);
		val *= sizeof(struct vi_notify_msg);
		return put_user(val, (int __user *)arg);
	}

	case NVHOST_VI_SET_IGN_MASK: {
		u32 mask;
		int err;

		if (get_user(mask, (u32 __user *)arg))
			return -EFAULT;
		if (mutex_lock_interruptible(&vnd->lock))
			return -ERESTARTSYS;

		err = vi_notify_classify(chan, mask, chan->pri_mask);
		mutex_unlock(&vnd->lock);
		return err;
	}

	case NVHOST_VI_SET_PRI_MASK: {
		u32 mask;
		int err;

		if (get_user(mask , (u32 __user *)arg))
			return -EFAULT;
		if (mutex_lock_interruptible(&vnd->lock))
			return -ERESTARTSYS;

		err = vi_notify_classify(chan, chan->ign_mask, mask);
		mutex_unlock(&vnd->lock);
		return err;
	}
	}

	return -ENOIOCTLCMD;
}

static struct vi_notify_dev *vnd_;
static DEFINE_MUTEX(vnd_lock);

static int vi_notify_open(struct inode *inode, struct file *file)
{
	struct vi_notify_dev *vnd;
	struct vi_notify_channel *chan;

	if ((file->f_flags & O_ACCMODE) != O_RDONLY)
		return -EINVAL;
	if (mutex_lock_interruptible(&vnd_lock))
		return -ERESTARTSYS;

	vnd = vnd_;

	if (vnd == NULL || iminor(inode) > 0 ||
		!try_module_get(vnd->driver->owner)) {
		mutex_unlock(&vnd_lock);
		return -ENODEV;
	}
	mutex_unlock(&vnd_lock);

	chan = kmalloc(sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL)) {
		module_put(vnd->driver->owner);
		return -ENOMEM;
	}

	chan->vnd = vnd;
	chan->ign_mask = 0;
	chan->pri_mask = 0;
	init_waitqueue_head(&chan->readq);
	mutex_init(&chan->read_lock);

	atomic_set(&chan->overruns, 0);
	atomic_set(&chan->errors, 0);
	INIT_KFIFO(chan->fifo);

	mutex_lock(&vnd->lock);
	if (rcu_access_pointer(vnd->channel) != NULL) {
		mutex_unlock(&vnd->lock);
		kfree(chan);
		module_put(vnd->driver->owner);
		return -EBUSY;
	}

	rcu_assign_pointer(vnd->channel, chan);
	vi_notify_dev_classify(vnd);
	mutex_unlock(&vnd->lock);
	file->private_data = chan;

	return nonseekable_open(inode, file);
}

static int vi_notify_release(struct inode *inode, struct file *file)
{
	struct vi_notify_channel *chan = file->private_data;
	struct vi_notify_dev *vnd = chan->vnd;

	mutex_lock(&vnd->lock);
	WARN_ON(rcu_access_pointer(vnd->channel) != chan);
	RCU_INIT_POINTER(vnd->channel, NULL);

	vi_notify_dev_classify(vnd);
	mutex_unlock(&vnd->lock);
	kfree_rcu(chan, rcu);
	module_put(vnd->driver->owner);
	return 0;
}

static const struct file_operations vi_notify_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = vi_notify_read,
	.poll = vi_notify_poll,
	.unlocked_ioctl = vi_notify_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vi_notify_ioctl,
#endif
	.open = vi_notify_open,
	.release = vi_notify_release,
};

/* Character device */
static struct class *vi_notify_class;
static dev_t vi_notify_major;

int vi_notify_register(struct vi_notify_driver *drv, struct device *dev,
			u8 num_channels)
{
	struct vi_notify_dev *vnd;
	int err;

	vnd = devm_kzalloc(dev, sizeof(*vnd), GFP_KERNEL);
	if (unlikely(vnd == NULL))
		return -ENOMEM;

	vnd->driver = drv;
	vnd->device = dev;
	vnd->num_channels = num_channels;
	mutex_init(&vnd->lock);

	err = vnd->driver->probe(vnd->device, vnd);
	if (err)
		return err;

	err = vi_notify_dev_classify(vnd);
	if (err)
		goto error;

	mutex_lock(&vnd_lock);
	WARN_ON(vnd_ != NULL);
	vnd_ = vnd;
	mutex_unlock(&vnd_lock);

	device_create(vi_notify_class, vnd->device, MKDEV(vi_notify_major, 0),
			NULL, "tegra-vi-notify0");

	return 0;
error:
	if (vnd->driver->remove)
		vnd->driver->remove(dev);
	return err;
}
EXPORT_SYMBOL(vi_notify_register);

void vi_notify_unregister(struct vi_notify_driver *drv, struct device *dev)
{
	struct vi_notify_dev *vnd;

	mutex_lock(&vnd_lock);
	vnd = vnd_;
	vnd_ = NULL;
	WARN_ON(vnd->driver != drv);
	WARN_ON(vnd->device != dev);
	mutex_unlock(&vnd_lock);

	device_destroy(vi_notify_class, MKDEV(vi_notify_major, 0));

	if (vnd->driver->remove)
		vnd->driver->remove(vnd->device);
	devm_kfree(vnd->device, vnd);
}
EXPORT_SYMBOL(vi_notify_unregister);

static int __init vi_notify_init(void)
{
	vi_notify_class = class_create(THIS_MODULE, "tegra-vi-channel");
	if (IS_ERR(vi_notify_class))
		return PTR_ERR(vi_notify_class);

	vi_notify_major = register_chrdev(0, "tegra-vi-channel",
						&vi_notify_fops);
	if (vi_notify_major < 0) {
		class_destroy(vi_notify_class);
		return vi_notify_major;
	}
	return 0;
}

static void __exit vi_notify_exit(void)
{
	unregister_chrdev(vi_notify_major, "tegra-vi-channel");
	class_destroy(vi_notify_class);
}

subsys_initcall(vi_notify_init);
module_exit(vi_notify_exit);
