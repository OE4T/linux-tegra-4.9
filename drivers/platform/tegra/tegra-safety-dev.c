/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/wait.h>
#include <asm/ioctls.h>
#include <linux/uaccess.h>

#include <linux/tegra-safety-ivc.h>

#define DEVICE_COUNT 1

#define CCIOGNFRAMES _IOR('c', 1, int)
#define CCIOGNBYTES _IOR('c', 2, int)

struct tegra_safety_dev_data {
	struct tegra_safety_ivc_chan *ivc_chan;
	struct cdev cdev;
	struct mutex io_lock;
	wait_queue_head_t read_waitq;
	wait_queue_head_t write_waitq;
	int device_count;
};

static struct tegra_safety_dev_data *dev_data;
static DEFINE_MUTEX(tegra_safety_dev_lock_open);
static int tegra_safety_dev_major_number;
static struct class *tegra_safety_dev_class;

static inline struct ivc *get_file_to_ivc(struct file *fp)
{
	return &((struct tegra_safety_ivc_chan *)fp->private_data)->ivc;
}

static int tegra_safety_dev_open(struct inode *in, struct file *f)
{
	unsigned int minor = iminor(in);
	int ret;

	if (minor >= DEVICE_COUNT)
		return -EBADFD;

	ret = mutex_lock_interruptible(&tegra_safety_dev_lock_open);
	if (ret)
		return ret;

	dev_data->device_count++;
	f->private_data = dev_data->ivc_chan;
	nonseekable_open(in, f);

	mutex_unlock(&tegra_safety_dev_lock_open);

	return ret;
}

static int tegra_safety_dev_release(struct inode *in, struct file *fp)
{
	mutex_lock(&tegra_safety_dev_lock_open);
	dev_data->device_count--;
	mutex_unlock(&tegra_safety_dev_lock_open);

	return 0;
}

static unsigned int tegra_safety_dev_poll(struct file *fp, poll_table *pt)
{
	struct ivc *ivc = get_file_to_ivc(fp);
	unsigned int ret = 0;

	poll_wait(fp, &dev_data->read_waitq, pt);
	poll_wait(fp, &dev_data->write_waitq, pt);

	mutex_lock(&dev_data->io_lock);
	if (tegra_ivc_can_read(ivc))
		ret |= (POLLIN | POLLRDNORM);
	if (tegra_ivc_can_write(ivc))
		ret |= (POLLOUT | POLLWRNORM);
	mutex_unlock(&dev_data->io_lock);

	return ret;
}

static ssize_t tegra_safety_dev_read(struct file *fp, char __user *buffer,
		size_t len, loff_t *offset)
{
	struct ivc *ivc = get_file_to_ivc(fp);
	DEFINE_WAIT(wait);
	size_t maxbytes = len > ivc->frame_size ? ivc->frame_size : len;
	ssize_t ret = 0;
	bool done = false;

	/*
	 * here we are reading maxbytes of data from IVC. If data is
	 * present we will read it, otherwise do the wait.
	 */
	while (!ret && maxbytes) {
		ret = mutex_lock_interruptible(&dev_data->io_lock);
		if (ret)
			return ret;
		prepare_to_wait(&dev_data->read_waitq, &wait,
				TASK_INTERRUPTIBLE);

		done = tegra_ivc_can_read(ivc);
		if (done)
			ret = tegra_ivc_read_user(ivc, buffer, maxbytes);
		mutex_unlock(&dev_data->io_lock);

		if (done)
			goto finish;
		else if (signal_pending(current))
			ret = -EINTR;
		else if (fp->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else
			schedule();
finish:
		finish_wait(&dev_data->read_waitq, &wait);
	};

	return ret;
}

static ssize_t tegra_safety_dev_write(struct file *fp,
		const char __user *buffer, size_t len, loff_t *offset)
{
	struct ivc *ivc = get_file_to_ivc(fp);
	DEFINE_WAIT(wait);
	size_t maxbytes = len > ivc->frame_size ? ivc->frame_size : len;
	ssize_t ret = 0;
	int done = false;

	/*
	 * here we are writing maxbytes of data to IVC. If space is
	 * available we will write it, otherwise do the wait.
	 */
	while (!ret && maxbytes) {
		ret = mutex_lock_interruptible(&dev_data->io_lock);
		if (ret)
			return ret;
		prepare_to_wait(&dev_data->write_waitq, &wait,
				TASK_INTERRUPTIBLE);

		done = tegra_ivc_can_write(ivc);
		if (done)
			ret = tegra_ivc_write_user(ivc, buffer, maxbytes);
		mutex_unlock(&dev_data->io_lock);

		if (done)
			goto finish;
		else if (signal_pending(current))
			ret = -EINTR;
		else if (fp->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else
			schedule();
finish:
		finish_wait(&dev_data->write_waitq, &wait);
	}

	return ret;
}

static long tegra_safety_dev_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct ivc *ivc = get_file_to_ivc(fp);
	int val = 0;
	long ret;

	mutex_lock(&dev_data->io_lock);

	switch (cmd) {
	case CCIOGNFRAMES:
		val = ivc->nframes;
		ret = put_user(val, (int __user *)arg);
		break;
	case CCIOGNBYTES:
		val = ivc->frame_size;
		ret = put_user(val, (int __user *)arg);
		break;

	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&dev_data->io_lock);

	return ret;
}

static const struct file_operations tegra_safety_dev_fops = {
	.open = tegra_safety_dev_open,
	.poll = tegra_safety_dev_poll,
	.read = tegra_safety_dev_read,
	.write = tegra_safety_dev_write,
	.release = tegra_safety_dev_release,
	.unlocked_ioctl = tegra_safety_dev_ioctl,
	.compat_ioctl = tegra_safety_dev_ioctl,
	.llseek = no_llseek,
};

void tegra_safety_dev_notify(void)
{
	struct ivc *ivc;
	int can_read, can_write;

	if (!dev_data)
		return;

	ivc = &dev_data->ivc_chan->ivc;

	mutex_lock(&dev_data->io_lock);
	can_read = tegra_ivc_can_read(ivc);
	can_write = tegra_ivc_can_write(ivc);
	mutex_unlock(&dev_data->io_lock);

	if (can_read)
		wake_up_interruptible(&dev_data->read_waitq);

	if (can_write)
		wake_up_interruptible(&dev_data->write_waitq);
}

int tegra_safety_dev_init(struct device *dev)
{
	struct tegra_safety_ivc *safety_ivc = dev_get_drvdata(dev);
	struct tegra_safety_dev_data *data;
	struct device *char_dev;
	dev_t start, num;
	int ret;

	ret = alloc_chrdev_region(&start, 0, DEVICE_COUNT, "safety");
	if (ret) {
		dev_alert(dev, "safety: failed to allocate device numbers\n");
		goto error;
	}
	tegra_safety_dev_major_number = MAJOR(start);

	tegra_safety_dev_class = class_create(THIS_MODULE, "safety_class");
	if (IS_ERR(tegra_safety_dev_class)) {
		dev_alert(dev, "safety: failed to create class\n");
		ret = PTR_ERR(tegra_safety_dev_class);
		goto error;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_alert(dev, "safety: failed to allocate memory\n");
		ret = -ENOMEM;
		goto error;
	}

	cdev_init(&data->cdev, &tegra_safety_dev_fops);
	data->cdev.owner = THIS_MODULE;
	init_waitqueue_head(&data->read_waitq);
	init_waitqueue_head(&data->write_waitq);
	mutex_init(&data->io_lock);

	data->ivc_chan = safety_ivc->ivc_chan;
	dev_data = data;
	num = MKDEV(tegra_safety_dev_major_number, 0);

	ret = cdev_add(&data->cdev, num, 1);
	if (ret) {
		dev_err(dev, "safety: unable to add character device\n");
		goto error;
	}

	char_dev = device_create(tegra_safety_dev_class, dev, num,
			NULL, "cmdresp");
	if (IS_ERR(char_dev)) {
		dev_err(dev, "safety: could not create device\n");
		ret = PTR_ERR(char_dev);
		goto error;
	}

	dev_info(dev, "safety: cmd-resp character device registered\n");

	return ret;

error:
	tegra_safety_dev_exit(dev);
	return ret;
}

void tegra_safety_dev_exit(struct device *dev)
{
	dev_t num = MKDEV(tegra_safety_dev_major_number, 0);

	if (!dev_data)
		return;

	device_destroy(tegra_safety_dev_class, num);
	cdev_del(&dev_data->cdev);
	class_destroy(tegra_safety_dev_class);
	unregister_chrdev_region(num, DEVICE_COUNT);
	dev_data = NULL;
	dev_info(dev, "safety: cmd-resp character device unregistered\n");
}

MODULE_DESCRIPTION("A Character device for safety command response module");
MODULE_LICENSE("GPL v2");
