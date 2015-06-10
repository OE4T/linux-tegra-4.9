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

#include <linux/export.h>
#include <linux/module.h>
#include <asm/ioctls.h>
#include <linux/kfifo.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/tegra-soc.h> /* is_fpga() */

#include "dev.h"
#include "t186/t186.h"
#include "vi/vi.h"
#include "nvhost_acm.h"

/* XXX: move ioctls to include/linux/ (after T18X merge) */
#include <linux/nvhost_vi_ioctl.h>
#define NVHOST_VI_SET_IGN_MASK _IOW(NVHOST_VI_IOCTL_MAGIC, 10, u32)
#define NVHOST_VI_SET_PRI_MASK _IOW(NVHOST_VI_IOCTL_MAGIC, 11, u32)


#define VI_NOTIFY_FIFO_TAG_0_0			0x4000
#define VI_NOTIFY_FIFO_TIMESTAMP_0_0		(VI_NOTIFY_FIFO_TAG_0_0 + 4)
#define VI_NOTIFY_FIFO_DATA_0_0			(VI_NOTIFY_FIFO_TAG_0_0 + 8)
#define VI_NOTIFY_TAG_CLASSIFY_NO_OUTPUT_0	0x6000
#define VI_NOTIFY_TAG_CLASSIFY_HIGH_0		0x6004
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_0		0x6008
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0	0x600C
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_TEST_0	0x6010
#define VI_NOTIFY_OCCUPANCY_0			0x6014
#define VI_NOTIFY_OCCUPANCY_URGENT_0		0x6018
#define VI_NOTIFY_HIGHPRIO_0			0x601C
#define VI_NOTIFY_ERROR_0			0x6020

struct vi_notify_msg {
	u32 tag;
	u32 stamp;
	u32 data;
	u32 reserve;
};

#define VI_NOTIFY_TAG_VALID(tag)	((tag) & 1)
#define VI_NOTIFY_TAG_TAG(tag)		(((tag) >> 1) & 0x7f)
#define VI_NOTIFY_TAG_CHANNEL(tag)	(((tag) >> 8) & 0xff)
#define VI_NOTIFY_TAG_FRAME(tag)	(((tag) >> 16) & 0xffff)

struct vi_notify_private {
	struct vi_notify_dev *dev;
#ifdef CONFIG_TEGRA_VI_NOTIFY
	u32 ign_mask;
	u32 pri_mask;

	wait_queue_head_t readq;
	struct mutex read_lock;
	struct rcu_head rcu;

	atomic_t overruns;
	atomic_t errors;
	DECLARE_KFIFO(fifo, struct vi_notify_msg, 256);
#endif
};

struct vi_notify_dev {
	struct platform_device *pdev;
#ifdef CONFIG_TEGRA_VI_NOTIFY
	struct vi_notify_private *priv;
	struct mutex lock;

	int error_irq;
	int prio_irq;
	int norm_irq;
#endif
};

#ifdef CONFIG_TEGRA_VI_NOTIFY
static void vi_notify_dump_classify(struct vi_notify_dev *dev)
{
	struct platform_device *pdev = dev->pdev;
	u32 r;

#define DUMP_TAG(x, y) \
do { \
	r = host1x_readl(pdev, VI_NOTIFY_TAG_CLASSIFY_##x##_0); \
	dev_dbg(&pdev->dev, "Classify " y ": 0x%08X\n", r); \
} while (0)

	DUMP_TAG(NO_OUTPUT,	"no output");
	DUMP_TAG(HIGH,		"high prio");
	DUMP_TAG(SAFETY,	"safety   ");
	DUMP_TAG(SAFETY_ERROR,	"error    ");
	DUMP_TAG(SAFETY_TEST,	"test     ");
}

static void vi_notify_classify(struct vi_notify_dev *dev)
{
	struct platform_device *pdev = dev->pdev;
	struct vi_notify_private *priv;
	u32 ign_mask = 0xffffffff, pri_mask = 0;

	priv = rcu_access_pointer(dev->priv);
	if (priv != NULL) {
		ign_mask &= priv->ign_mask;
		pri_mask |= priv->pri_mask;
	}

	WARN_ON(ign_mask & pri_mask);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_NO_OUTPUT_0, ign_mask);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_HIGH_0, pri_mask);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_0, 0);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_TEST_0, 0);
	host1x_writel(pdev, VI_NOTIFY_OCCUPANCY_URGENT_0, 512);
	vi_notify_dump_classify(dev);
}

static void vi_notify_dump_status(struct vi_notify_dev *dev)
{
	struct platform_device *pdev = dev->pdev;
	u32 r = host1x_readl(pdev, VI_NOTIFY_OCCUPANCY_0);

	dev_dbg(&pdev->dev, "Occupancy: %u/%u (max: %u)\n",
		(r >> 10) & 0x3ff, r & 0x3ff,  (r >> 20) & 0x3ff);
	dev_dbg(&pdev->dev, "Urgent:    %u\n",
		host1x_readl(pdev, VI_NOTIFY_HIGHPRIO_0));
	dev_dbg(&pdev->dev, "Error:   0x%08X\n",
		host1x_readl(pdev, VI_NOTIFY_ERROR_0));
}

static unsigned vi_notify_occupancy(struct file *file)
{
	struct vi_notify_private *priv = file->private_data;
	unsigned ret;

	mutex_lock(&priv->read_lock);
	ret = kfifo_len(&priv->fifo);
	mutex_unlock(&priv->read_lock);

	return ret;
}

/* Interrupt handlers */
static irqreturn_t vi_notify_error_isr(int irq, void *dev_id)
{
	struct vi_notify_dev *dev = dev_id;
	struct platform_device *pdev = dev->pdev;
	struct vi_notify_private *priv;
	u32 err;

	err = host1x_readl(pdev, VI_NOTIFY_ERROR_0);
	host1x_writel(pdev, VI_NOTIFY_ERROR_0, err);
	dev_err(&pdev->dev, "master error 0x%08X\n", err);

	err = host1x_readl(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0);
	host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0, err);
	if (err)
		dev_err(&pdev->dev, "safety error mask 0x%08X\n", err);

	rcu_read_lock();
	priv = rcu_dereference(dev->priv);

	if (priv != NULL) {
		atomic_set(&priv->errors, 1);
		wake_up(&priv->readq);
	}
	rcu_read_unlock();

	return IRQ_HANDLED;
}

static irqreturn_t vi_notify_prio_isr(int irq, void *dev_id)
{
	struct vi_notify_dev *dev = dev_id;
	struct platform_device *pdev = dev->pdev;
	u32 count = host1x_readl(pdev, VI_NOTIFY_HIGHPRIO_0);

	/* Not clear what to do with prioritized events: There are no ways to
	 * dequeue them out-of-band. Let the regular ISR deal with them. */
	dev_dbg(&pdev->dev, "priority count: %u", count);
	host1x_writel(pdev, VI_NOTIFY_HIGHPRIO_0, count);
	return IRQ_HANDLED;
}

static irqreturn_t vi_notify_isr(int irq, void *dev_id)
{
	struct vi_notify_dev *dev = dev_id;
	struct platform_device *pdev = dev->pdev;

	for (;;) {
		struct vi_notify_private *priv;
		struct vi_notify_msg msg;
		u8 tag;

		msg.tag = host1x_readl(pdev, VI_NOTIFY_FIFO_TAG_0_0);
		vi_notify_dump_status(dev);

		if (!VI_NOTIFY_TAG_VALID(msg.tag))
			break;

		tag = VI_NOTIFY_TAG_TAG(msg.tag);
		if (tag >= 32) {
			dev_warn(&pdev->dev, "Tag %u out of range!\n", tag);
			continue;
		}

		dev_dbg(&pdev->dev,
			"Message: tag:%2u channel:%02X frame:%04X\n", tag,
			VI_NOTIFY_TAG_CHANNEL(msg.tag),
			VI_NOTIFY_TAG_FRAME(tag));
		msg.stamp = host1x_readl(pdev, VI_NOTIFY_FIFO_TIMESTAMP_0_0);
		msg.data = host1x_readl(pdev, VI_NOTIFY_FIFO_DATA_0_0);
		msg.reserve = 0;
		dev_dbg(&pdev->dev, "         timestamp %u data 0x%08x",
			msg.stamp, msg.data);

		rcu_read_lock();
		priv = rcu_dereference(dev->priv);

		if (priv != NULL) {
			if (!kfifo_put(&priv->fifo, msg))
				atomic_set(&priv->overruns, 1);
			wake_up(&priv->readq);
		}
		rcu_read_unlock();
	}

	return IRQ_HANDLED;
}

static int vi_notify_get_irq(struct vi_notify_dev *dev, unsigned num,
				irq_handler_t isr)
{
	struct platform_device *pdev = dev->pdev;
	int err, irq;

	irq = platform_get_irq(pdev, num);
	if (IS_ERR_VALUE(irq)) {
		dev_err(&pdev->dev, "missing IRQ\n");
		return irq;
	}

	err = request_threaded_irq(irq, NULL, isr, IRQF_ONESHOT,
					dev_name(&pdev->dev), dev);
	if (err) {
		dev_err(&pdev->dev, "cannot get IRQ\n");
		return err;
	}
	return irq;
}

int nvhost_vi_notify_prepare_poweroff(struct platform_device *pdev)
{
	struct vi *tegra_vi = nvhost_get_private_data(pdev);
	struct vi_notify_dev *dev = tegra_vi->vi_notify;

	disable_irq(dev->error_irq);
	disable_irq(dev->prio_irq);
	disable_irq(dev->norm_irq);

	return 0;
}

int nvhost_vi_notify_finalize_poweron(struct platform_device *pdev)
{
	struct vi *tegra_vi = nvhost_get_private_data(pdev);
	struct vi_notify_dev *dev = tegra_vi->vi_notify;

	enable_irq(dev->error_irq);
	enable_irq(dev->prio_irq);
	enable_irq(dev->norm_irq);

	return 0;
}

/* File operations */
static ssize_t vi_notify_read(struct file *file, char __user *buf, size_t len,
				loff_t *offset)
{
	struct vi_notify_private *priv = file->private_data;

	for (;;) {
		DEFINE_WAIT(wait);
		unsigned int copied;
		int ret = 0;

		if (len < sizeof(struct vi_notify_msg))
			return 0;
		if (mutex_lock_interruptible(&priv->read_lock))
			return -ERESTARTSYS;

		ret = kfifo_to_user(&priv->fifo, buf, len, &copied);
		mutex_unlock(&priv->read_lock);

		if (ret)
			return ret;
		if (copied > 0)
			return copied;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		prepare_to_wait(&priv->readq, &wait, TASK_INTERRUPTIBLE);

		if (atomic_xchg(&priv->overruns, 0))
			ret = -EOVERFLOW;
		else if (atomic_xchg(&priv->errors, 0))
			ret = -EIO;
		else if (signal_pending(current))
			ret = -ERESTARTSYS;
		else if (!vi_notify_occupancy(file))
			schedule();

		finish_wait(&priv->readq, &wait);

		if (ret)
			return ret;
	}
}

static unsigned int vi_notify_poll(struct file *file,
					struct poll_table_struct *table)
{
	struct vi_notify_private *priv = file->private_data;
	unsigned ret = 0;

	if (file->f_mode & FMODE_READ) {
		poll_wait(file, &priv->readq, table);

		if (vi_notify_occupancy(file))
			ret |= POLLIN | POLLRDNORM;
		if (atomic_read(&priv->overruns) || atomic_read(&priv->errors))
			ret |= POLLERR;
	}

	return ret;
}
#endif /* TEGRA_VI_NOTIFY */

static long vi_notify_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct vi_notify_private *priv = file->private_data;

	switch (cmd) {
#ifdef CONFIG_TEGRA_VI_NOTIFY
	case FIONREAD: {
		int val;

		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;

		val = vi_notify_occupancy(file) * sizeof(struct vi_notify_msg);
		return put_user(val, (int __user *)arg);
	}
#endif

	case NVHOST_VI_IOCTL_ENABLE_TPG:
		pr_warn("[%d] %s: obsolete ioctl %08x ignored.\n",
			task_pid_nr(current), current->comm, cmd);
		return 0;

	case NVHOST_VI_IOCTL_SET_EMC_INFO:
		pr_err("[%d] %s: unimplemented ioctl %08x.\n",
			task_pid_nr(current), current->comm, cmd);
		return -ENOTSUPP;

	case NVHOST_VI_IOCTL_SET_VI_CLK: {
		long rate;

		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		if (get_user(rate, (long __user *)arg))
			return -EFAULT;

		return nvhost_module_set_rate(priv->dev->pdev, priv, rate, 0,
						NVHOST_CLOCK);
	}

#ifdef CONFIG_TEGRA_VI_NOTIFY
	case NVHOST_VI_SET_IGN_MASK: {
		struct vi_notify_dev *dev = priv->dev;
		u32 mask;

		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		if (get_user(mask, (u32 __user *)arg))
			return -EFAULT;

		mutex_lock(&dev->lock);
		priv->ign_mask = mask;
		vi_notify_classify(dev);
		mutex_unlock(&dev->lock);
		return 0;
	}

	case NVHOST_VI_SET_PRI_MASK: {
		struct vi_notify_dev *dev = priv->dev;
		u32 mask;

		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		if (get_user(mask , (u32 __user *)arg))
			return -EFAULT;

		mutex_lock(&dev->lock);
		priv->pri_mask = mask;
		vi_notify_classify(dev);
		mutex_unlock(&dev->lock);
		return 0;
	}
#endif
	}

	return -ENOIOCTLCMD;
}

static int vi_notify_dev_open(struct inode *inode, struct file *file,
				struct vi_notify_dev *dev)
{
	struct vi_notify_private *priv;
	int err;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL))
		return -ENOMEM;

	priv->dev = dev;
#ifdef CONFIG_TEGRA_VI_NOTIFY
	priv->ign_mask = 0;
	priv->pri_mask = 0;
	init_waitqueue_head(&priv->readq);
	mutex_init(&priv->read_lock);

	atomic_set(&priv->overruns, 0);
	atomic_set(&priv->errors, 0);
	INIT_KFIFO(priv->fifo);

	if ((file->f_flags & O_ACCMODE) != O_WRONLY) {
		if (tegra_platform_is_fpga())
			return -ENODEV;

		mutex_lock(&dev->lock);
		if (rcu_access_pointer(dev->priv) != NULL) {
			mutex_unlock(&dev->lock);
			kfree(priv);
			return -EBUSY;
		}
		rcu_assign_pointer(dev->priv, priv);
		vi_notify_classify(dev);
		mutex_unlock(&dev->lock);
	}
#endif
	file->private_data = priv;

	err = nvhost_module_add_client(dev->pdev, priv);
	if (err)
		goto error;

	return nonseekable_open(inode, file);
error:
#ifdef CONFIG_TEGRA_VI_NOTIFY
	if ((file->f_flags & O_ACCMODE) != O_WRONLY) {
		mutex_lock(&dev->lock);
		RCU_INIT_POINTER(dev->priv, NULL);
		mutex_unlock(&dev->lock);
		kfree_rcu(priv, rcu);
	} else
#endif
		kfree(priv);
	return err;
}

static int vi_notify_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct vi *vi = nvhost_get_private_data(pdata->pdev);
	struct vi_notify_dev *dev = vi->vi_notify;

	if (unlikely(IS_ERR(dev)))
		return PTR_ERR(dev);

	return vi_notify_dev_open(inode, file, dev);
}

static int vi_notify_release(struct inode *inode, struct file *file)
{
	struct vi_notify_private *priv = file->private_data;
	struct vi_notify_dev *dev = priv->dev;

	nvhost_module_remove_client(dev->pdev, priv);

#ifdef CONFIG_TEGRA_VI_NOTIFY
	if ((file->f_flags & O_ACCMODE) != O_WRONLY) {
		mutex_lock(&dev->lock);
		BUG_ON(rcu_access_pointer(dev->priv) != priv);
		RCU_INIT_POINTER(dev->priv, NULL);

		vi_notify_classify(dev);
		mutex_unlock(&dev->lock);
		kfree_rcu(priv, rcu);
	} else
#endif
		kfree(priv);
	return 0;
}

const struct file_operations tegra_vi_notify_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
#ifdef CONFIG_TEGRA_VI_NOTIFY
	.read = vi_notify_read,
	.poll = vi_notify_poll,
#endif
	.unlocked_ioctl = vi_notify_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vi_notify_ioctl,
#endif
	.open = vi_notify_open,
	.release = vi_notify_release,
};

/* Platform device */
int nvhost_vi_notify_dev_probe(struct platform_device *pdev)
{
	struct vi *tegra_vi = nvhost_get_private_data(pdev);
	struct vi_notify_dev *dev;
#ifdef CONFIG_TEGRA_VI_NOTIFY
	int ret;
#endif

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (unlikely(dev == NULL))
		return -ENOMEM;

	dev->pdev = pdev;
	tegra_vi->vi_notify = dev;

#ifdef CONFIG_TEGRA_VI_NOTIFY
	mutex_init(&dev->lock);

	dev->error_irq = vi_notify_get_irq(dev, 0, vi_notify_error_isr);
	dev->prio_irq = vi_notify_get_irq(dev, 1, vi_notify_prio_isr);
	dev->norm_irq = vi_notify_get_irq(dev, 2, vi_notify_isr);

	disable_irq(dev->error_irq);
	disable_irq(dev->prio_irq);
	disable_irq(dev->norm_irq);

	ret = nvhost_module_busy(pdev);
	if (ret) {
		WARN_ON(1);
		return ret;
	}

	vi_notify_classify(dev);
	vi_notify_dump_status(dev);
	nvhost_module_idle(pdev);
#endif

	return 0;
}

int nvhost_vi_notify_dev_remove(struct platform_device *pdev)
{
	struct vi *tegra_vi = nvhost_get_private_data(pdev);
	struct vi_notify_dev *dev = tegra_vi->vi_notify;

	if (unlikely(dev == NULL))
		return 0;

#ifdef CONFIG_TEGRA_VI_NOTIFY
	if (!IS_ERR_VALUE(dev->norm_irq))
		free_irq(dev->norm_irq, dev);
	if (!IS_ERR_VALUE(dev->prio_irq))
		free_irq(dev->prio_irq, dev);
	if (!IS_ERR_VALUE(dev->error_irq))
		free_irq(dev->error_irq, dev);
#endif
	kfree(dev);
	return 0;
}
