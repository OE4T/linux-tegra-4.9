/*
 * Tegra Hypervisor manager
 *
 * Instantiates virtualization-related resources.
 *
 * Copyright (C) 2014-2015, NVIDIA CORPORATION. All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/workqueue.h>

#include <linux/tegra-soc.h>
#include <linux/tegra-ivc.h>

#include "syscalls.h"
#include <linux/tegra-ivc-instance.h>

struct tegra_hv_data;

struct ivc_dev {
	struct tegra_hv_data	*hvd;
	int			minor;
	dev_t			dev;
	struct cdev		cdev;
	struct device		*device;
	char			name[32];

	/*
	 * ivc_devs are stored in an id-indexed array; this field indicates
	 * a valid array entry.
	 */
	int			valid;

	/* channel configuration */
	struct ivc		ivc;
	const struct tegra_hv_queue_data *qd;
	const struct guest_ivc_info *givci;
	int			other_guestid;

	/* This lock synchronizes the reserved flag. */
	struct mutex		lock;
	int			reserved;

	const struct tegra_hv_ivc_ops *cookie_ops;
	struct tegra_hv_ivc_cookie cookie;

	/* File mode */
	wait_queue_head_t	wq;
	/*
	 * Lock for synchronizing access to the IVC channel between the threaded
	 * IRQ handler's notification processing and file ops.
	 */
	struct mutex		file_lock;
};

#define cookie_to_ivc_dev(_cookie) \
	container_of(_cookie, struct ivc_dev, cookie)

/* Describe all info needed to do IVC to one particular guest */
struct guest_ivc_info {
	uintptr_t shmem;	/* IO remapped shmem */
	size_t length;		/* length of shmem */
};

struct hv_mempool {
	struct tegra_hv_ivm_cookie ivmk;
	const struct ivc_mempool *mpd;
	struct mutex lock;
	int reserved;
};

struct tegra_hv_data {
	const struct ivc_info_page *info;
	struct platform_device *pdev;
	int guestid;

	struct guest_ivc_info *guest_ivc_info;

	/* ivc_devs is indexed by queue id */
	struct ivc_dev *ivc_devs;
	uint32_t max_qid;

	/* array with length info->nr_mempools */
	struct hv_mempool *mempools;

	struct class *ivc_class;
	int ivc_major;

	dev_t ivc_dev;
};

/*
 * Global HV state for read-only access by tegra_hv_... APIs
 *
 * This should be accessed only through get_hvd().
 */
static const struct tegra_hv_data *tegra_hv_data;

static void ivc_raise_irq(struct ivc *ivc_channel)
{
	struct ivc_dev *ivc = container_of(ivc_channel, struct ivc_dev, ivc);
	hyp_raise_irq(ivc->qd->raise_irq, ivc->other_guestid);
}

static const struct tegra_hv_data *get_hvd(void)
{
	if (!tegra_hv_data) {
		pr_err("%s: tegra_hv: not initialized yet\n", __func__);
		return ERR_PTR(-EPROBE_DEFER);
	} else
		return tegra_hv_data;
}

static void ivc_handle_notification(struct ivc_dev *ivc)
{
	struct tegra_hv_ivc_cookie *ivck = &ivc->cookie;

	/* This function should only be used when callbacks are specified. */
	BUG_ON(!ivc->cookie_ops);

	/* there are data in the queue, callback */
	if (ivc->cookie_ops->rx_rdy && tegra_ivc_can_read(&ivc->ivc))
		ivc->cookie_ops->rx_rdy(ivck);

	/* there is space in the queue to write, callback */
	if (ivc->cookie_ops->tx_rdy && tegra_ivc_can_write(&ivc->ivc))
		ivc->cookie_ops->tx_rdy(ivck);
}

static irqreturn_t ivc_dev_cookie_irq_handler(int irq, void *data)
{
	struct ivc_dev *ivcd = data;
	ivc_handle_notification(ivcd);
	return IRQ_HANDLED;
}

static irqreturn_t ivc_threaded_irq_handler(int irq, void *dev_id)
{
	/*
	 * Virtual IRQs are known to be edge-triggered, so no action is needed
	 * to acknowledge them.
	 */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ivc_dev_handler(int irq, void *data)
{
	struct ivc_dev *ivc = data;

	mutex_lock(&ivc->file_lock);
	tegra_ivc_channel_notified(&ivc->ivc);
	mutex_unlock(&ivc->file_lock);

	/* simple implementation, just kick all waiters */
	wake_up_interruptible_all(&ivc->wq);

	return IRQ_HANDLED;
}

static void ivc_release_irq(struct ivc_dev *ivc)
{
	BUG_ON(!ivc);

	devm_free_irq(ivc->device, ivc->qd->irq, ivc);
}

static int ivc_request_cookie_irq(struct ivc_dev *ivcd)
{
	return devm_request_irq(ivcd->device, ivcd->qd->irq,
			ivc_dev_cookie_irq_handler, 0, dev_name(ivcd->device),
			ivcd);
}

static int ivc_dev_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct ivc_dev *ivc = container_of(cdev, struct ivc_dev, cdev);
	int ret;

	mutex_lock(&ivc->lock);
	if (ivc->reserved) {
		ret = -EBUSY;
	} else {
		ivc->reserved = 1;
		ret = 0;
	}
	mutex_unlock(&ivc->lock);

	if (ret)
		return ret;

	ivc->cookie_ops = NULL;
	mutex_lock(&ivc->file_lock);
	tegra_ivc_channel_reset(&ivc->ivc);
	mutex_unlock(&ivc->file_lock);

	/* request our irq */
	ret = devm_request_threaded_irq(ivc->device, ivc->qd->irq,
			ivc_threaded_irq_handler, ivc_dev_handler, 0,
			dev_name(ivc->device), ivc);
	if (ret < 0) {
		dev_err(ivc->device, "Failed to request irq %d\n",
				ivc->qd->irq);
		mutex_lock(&ivc->lock);
		BUG_ON(!ivc->reserved);
		ivc->reserved = 0;
		mutex_unlock(&ivc->lock);
		return ret;
	}

	/* all done */
	filp->private_data = ivc;

	return 0;
}

static int ivc_dev_release(struct inode *inode, struct file *filp)
{
	struct ivc_dev *ivc = filp->private_data;

	if (ivc == NULL)
		return 0;
	filp->private_data = NULL;

	ivc_release_irq(ivc);

	mutex_lock(&ivc->lock);
	BUG_ON(!ivc->reserved);
	ivc->reserved = 0;
	mutex_unlock(&ivc->lock);

	return 0;
}

static int ivc_dump(struct ivc_dev *ivc)
{
	struct tegra_hv_data *hvd = ivc->hvd;
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;

	dev_info(dev, "IVC#%d: IRQ=%d nframes=%d frame_size=%d offset=%d\n",
			ivc->qd->id, ivc->qd->irq,
			ivc->qd->nframes, ivc->qd->frame_size, ivc->qd->offset);

	return 0;
}

static ssize_t ivc_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ivc_dev *ivcd = filp->private_data;
	struct ivc *ivc = &ivcd->ivc;
	int left = count, ret = 0, chunk;

	if (!tegra_ivc_can_read(ivc)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(ivcd->wq,
				tegra_ivc_can_read(ivc));
		if (ret)
			return ret;
	}

	while (left > 0 && tegra_ivc_can_read(ivc)) {

		chunk = ivcd->qd->frame_size;
		if (chunk > left)
			chunk = left;
		mutex_lock(&ivcd->file_lock);
		ret = tegra_ivc_read_user(ivc, buf, chunk);
		mutex_unlock(&ivcd->file_lock);
		if (ret < 0)
			break;

		buf += chunk;
		left -= chunk;
	}

	if (left >= count)
		return ret;

	return count - left;
}

static ssize_t ivc_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct ivc_dev *ivcd = filp->private_data;
	struct ivc *ivc;
	ssize_t done;
	size_t left, chunk;
	int ret = 0;

	BUG_ON(!ivcd);
	ivc = &ivcd->ivc;

	done = 0;
	while (done < count) {

		left = count - done;

		if (left < ivcd->qd->frame_size)
			chunk = left;
		else
			chunk = ivcd->qd->frame_size;

		/* is queue full? */
		if (!tegra_ivc_can_write(ivc)) {

			/* check non-blocking mode */
			if (filp->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}

			ret = wait_event_interruptible(ivcd->wq,
					tegra_ivc_can_write(ivc));
			if (ret)
				break;
		}

		mutex_lock(&ivcd->file_lock);
		ret = tegra_ivc_write_user(ivc, buf, chunk);
		mutex_unlock(&ivcd->file_lock);
		if (ret < 0)
			break;

		buf += chunk;

		done += chunk;
		*pos += chunk;
	}


	if (done == 0)
		return ret;

	return done;
}

static unsigned int ivc_dev_poll(struct file *filp, poll_table *wait)
{
	struct ivc_dev *ivcd = filp->private_data;
	struct ivc *ivc;
	int mask = 0;

	BUG_ON(!ivcd);
	ivc = &ivcd->ivc;

	poll_wait(filp, &ivcd->wq, wait);

	if (tegra_ivc_can_read(ivc))
		mask = POLLIN | POLLRDNORM;

	if (tegra_ivc_can_write(ivc))
		mask |= POLLOUT | POLLWRNORM;

	/* no exceptions */

	return mask;
}

static const struct file_operations ivc_fops = {
	.owner		= THIS_MODULE,
	.open		= ivc_dev_open,
	.release	= ivc_dev_release,
	.llseek		= noop_llseek,
	.read		= ivc_dev_read,
	.write		= ivc_dev_write,
	.poll		= ivc_dev_poll,
};

static ssize_t vmid_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	const struct tegra_hv_data *hvd = get_hvd();
	BUG_ON(!hvd);
	return snprintf(buf, PAGE_SIZE, "%d\n", hvd->guestid);
}

static ssize_t id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->qd->id);
}

static ssize_t frame_size_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->qd->frame_size);
}

static ssize_t nframes_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->qd->nframes);
}

static ssize_t reserved_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	int reserved;

	mutex_lock(&ivc->lock);
	reserved = ivc->reserved;
	mutex_unlock(&ivc->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", reserved);
}

static ssize_t peer_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->other_guestid);
}

static const struct device_attribute hv_vmid_attr = __ATTR_RO(vmid);

struct device_attribute ivc_dev_attrs[] = {
	__ATTR_RO(id),
	__ATTR_RO(frame_size),
	__ATTR_RO(nframes),
	__ATTR_RO(reserved),
	__ATTR_RO(peer),
	__ATTR_NULL
};

static int tegra_hv_add_ivc(struct tegra_hv_data *hvd,
		const struct tegra_hv_queue_data *qd)
{
	struct device *dev = &hvd->pdev->dev;
	struct ivc_dev *ivc;
	int ret;
	int rx_first;
	uintptr_t rx_base, tx_base;
	uint32_t i;

	ivc = &hvd->ivc_devs[qd->id];
	BUG_ON(ivc->valid);
	ivc->valid = 1;
	ivc->hvd = hvd;
	ivc->qd = qd;

	if (qd->peers[0] == hvd->guestid)
		ivc->other_guestid = qd->peers[1];
	else if (qd->peers[1] == hvd->guestid)
		ivc->other_guestid = qd->peers[0];
	else
		BUG();


	/*
	 * Locate the guest_ivc_info representing the remote guest accessed
	 * through this channel.
	 */
	for (i = 0; i < hvd->info->nr_areas; i++) {
		if (hvd->info->areas[i].guest == ivc->other_guestid) {
			ivc->givci = &hvd->guest_ivc_info[i];
			break;
		}
	}
	BUG_ON(i == hvd->info->nr_areas);

	BUG_ON(ivc->givci->shmem == 0);

	mutex_init(&ivc->lock);
	init_waitqueue_head(&ivc->wq);
	mutex_init(&ivc->file_lock);

	ivc->minor = qd->id;
	ivc->dev = MKDEV(hvd->ivc_major, ivc->minor);
	cdev_init(&ivc->cdev, &ivc_fops);
	snprintf(ivc->name, sizeof(ivc->name) - 1, "ivc%d", qd->id);
	ret = cdev_add(&ivc->cdev, ivc->dev, 1);
	if (ret != 0) {
		dev_err(dev, "cdev_add() failed\n");
		return ret;
	}
	/* parent is this hvd dev */
	ivc->device = device_create(hvd->ivc_class, dev, ivc->dev, ivc,
			ivc->name);
	if (IS_ERR(ivc->device)) {
		dev_err(dev, "device_create() failed for %s\n", ivc->name);
		return PTR_ERR(ivc->device);
	}
	/* point to ivc */
	dev_set_drvdata(ivc->device, ivc);

	if (qd->peers[0] == qd->peers[1]) {
		/*
		 * The queue ids of loopback queues are always consecutive, so
		 * the even-numbered one receives in the first area.
		 */
		rx_first = (qd->id & 1) == 0;
	} else {
		rx_first = hvd->guestid == qd->peers[0];
	}

	BUG_ON(qd->offset >= ivc->givci->length);
	BUG_ON(qd->offset + qd->size * 2 >= ivc->givci->length);

	if (rx_first) {
		rx_base = ivc->givci->shmem + qd->offset;
		tx_base = ivc->givci->shmem + qd->offset + qd->size;
	} else {
		tx_base = ivc->givci->shmem + qd->offset;
		rx_base = ivc->givci->shmem + qd->offset + qd->size;
	}

	dev_info(dev, "adding ivc%u: rx_base=%lx tx_base = %lx size=%x\n",
			qd->id, rx_base, tx_base, qd->size);
	tegra_ivc_init(&ivc->ivc, rx_base, tx_base, qd->nframes, qd->frame_size,
			NULL, ivc_raise_irq);

	/* We may have rebooted, so the channel could be active. */
	ret = tegra_ivc_channel_sync(&ivc->ivc);
	if (ret != 0)
		return  ret;

	dev_info(dev, "added %s\n", ivc->name);

	return 0;
}

struct ivc_dev *ivc_device_by_id(const struct tegra_hv_data *hvd, uint32_t id)
{
	if (id > hvd->max_qid)
		return NULL;
	else {
		struct ivc_dev *ivc = &hvd->ivc_devs[id];
		if (ivc->valid)
			return ivc;
		else
			return NULL;
	}
}

static void tegra_hv_ivc_cleanup(struct tegra_hv_data *hvd)
{
	uint32_t i;

	if (!hvd->ivc_devs)
		return;

	for (i = 0; i < hvd->info->nr_queues; i++) {
		struct ivc_dev *ivc = &hvd->ivc_devs[i];
		if (ivc->device) {
			BUG_ON(!ivc->valid);
			cdev_del(&ivc->cdev);
			device_del(ivc->device);
		}
	}

	kfree(hvd->ivc_devs);
	hvd->ivc_devs = NULL;
}

static void tegra_hv_cleanup(struct tegra_hv_data *hvd)
{
	/*
	 * Destroying IVC channels in use is not supported. Once it's possible
	 * for IVC channels to be reserved, we no longer clean up.
	 */
	BUG_ON(tegra_hv_data != NULL);

	kfree(hvd->mempools);
	hvd->mempools = NULL;

	device_remove_file(&hvd->pdev->dev, &hv_vmid_attr);
	tegra_hv_ivc_cleanup(hvd);

	if (hvd->ivc_dev) {
		unregister_chrdev_region(hvd->ivc_dev, hvd->max_qid);
		hvd->ivc_dev = 0;
	}

	if (!hvd->info)
		return;

	if (hvd->guest_ivc_info) {
		uint32_t i;
		for (i = 0; i < hvd->info->nr_areas; i++) {
			if (hvd->guest_ivc_info[i].shmem) {
				iounmap((void *)hvd->guest_ivc_info[i].shmem);
				hvd->guest_ivc_info[i].shmem = 0;
			}
		}

		kfree(hvd->guest_ivc_info);
		hvd->guest_ivc_info = NULL;
	}

	iounmap((void *)hvd->info);
	hvd->info = NULL;

	if (hvd->ivc_class) {
		class_destroy(hvd->ivc_class);
		hvd->ivc_class = NULL;
	}
}

static int tegra_hv_setup(struct tegra_hv_data *hvd)
{
	uint64_t info_page;
	uint32_t i;
	int ret;
	struct device *dev = &hvd->pdev->dev;

	ret = hyp_read_gid(&hvd->guestid);
	if (ret != 0) {
		dev_err(dev, "Failed to read guest id\n");
		return -ENODEV;
	}

	hvd->ivc_class = class_create(THIS_MODULE, "ivc");
	if (IS_ERR(hvd->ivc_class)) {
		dev_err(dev, "class_create() failed\n");
		return PTR_ERR(hvd->ivc_class);
	}

	/* set class attributes */
	hvd->ivc_class->dev_attrs = ivc_dev_attrs;

	ret = hyp_read_ivc_info(&info_page);
	if (ret != 0) {
		dev_err(dev, "failed to obtain IVC info page: %d\n", ret);
		return ret;
	}

	hvd->info = (struct ivc_info_page *)ioremap_cached(info_page,
			PAGE_SIZE);
	if (hvd->info == NULL) {
		dev_err(dev, "failed to map IVC info page (%llx)\n", info_page);
		return -ENOMEM;
	}

	hvd->guest_ivc_info = kzalloc(hvd->info->nr_areas *
			sizeof(*hvd->guest_ivc_info), GFP_KERNEL);
	if (hvd->guest_ivc_info == NULL) {
		dev_err(dev, "failed to allocate %u-entry givci\n",
				hvd->info->nr_areas);
		return -ENOMEM;
	}

	for (i = 0; i < hvd->info->nr_areas; i++) {
		hvd->guest_ivc_info[i].shmem = (uintptr_t)ioremap_cached(
				hvd->info->areas[i].pa,
				hvd->info->areas[i].size);
		if (hvd->guest_ivc_info[i].shmem == 0) {
			dev_err(dev, "can't map area for guest %u (%llx)\n",
					hvd->info->areas[i].guest,
					hvd->info->areas[i].pa);
			return -ENOMEM;
		}
		hvd->guest_ivc_info[i].length = hvd->info->areas[i].size;
	}

	/*
	 * Determine the largest queue id in order to allocate a queue id-
	 * indexed array and device nodes.
	 */
	hvd->max_qid = 0;
	for (i = 0; i < hvd->info->nr_queues; i++) {
		const struct tegra_hv_queue_data *qd =
				&ivc_info_queue_array(hvd->info)[i];
		if (qd->id > hvd->max_qid)
			hvd->max_qid = qd->id;
	}

	/* allocate the whole chardev range */
	ret = alloc_chrdev_region(&hvd->ivc_dev, 0, hvd->max_qid, "ivc");
	if (ret < 0) {
		dev_err(dev, "alloc_chrdev_region() failed\n");
		return ret;
	}
	hvd->ivc_major = MAJOR(hvd->ivc_dev);

	hvd->ivc_devs = kzalloc((hvd->max_qid + 1) * sizeof(*hvd->ivc_devs),
			GFP_KERNEL);
	if (hvd->ivc_devs == NULL) {
		dev_err(dev, "failed to allocate %u-entry ivc_devs array\n",
				hvd->info->nr_queues);
		return -ENOMEM;
	}

	/* instantiate the IVC */
	for (i = 0; i < hvd->info->nr_queues; i++) {
		const struct tegra_hv_queue_data *qd =
				&ivc_info_queue_array(hvd->info)[i];
		ret = tegra_hv_add_ivc(hvd, qd);
		if (ret != 0) {
			dev_err(dev, "failed to add queue #%u\n", qd->id);
			return ret;
		}
	}

	ret = device_create_file(dev, &hv_vmid_attr);
	if (ret != 0) {
		dev_err(dev, "failed to create vmid sysfs attribute: %d\n",
			ret);
		return ret;
	}

	hvd->mempools =
		kzalloc(hvd->info->nr_mempools * sizeof(*hvd->mempools),
								GFP_KERNEL);
	if (hvd->mempools == NULL) {
		dev_err(dev, "failed to allocate %u-entry mempools array\n",
				hvd->info->nr_mempools);
		return -ENOMEM;
	}

	/* Initialize mempools. */
	for (i = 0; i < hvd->info->nr_mempools; i++) {
		const struct ivc_mempool *mpd =
				&ivc_info_mempool_array(hvd->info)[i];
		struct tegra_hv_ivm_cookie *ivmk = &hvd->mempools[i].ivmk;

		hvd->mempools[i].mpd = mpd;
		mutex_init(&hvd->mempools[i].lock);

		ivmk->ipa = mpd->pa;
		ivmk->size = mpd->size;
		ivmk->peer_vmid = mpd->peer_vmid;

		dev_info(dev, "added mempool %u: ipa=%llx size=%llx peer=%u\n",
				mpd->id, mpd->pa, mpd->size, mpd->peer_vmid);
	}

	return 0;
}

static int tegra_hv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tegra_hv_data *hvd;
	int ret;

	BUG_ON(tegra_hv_data != NULL);

	if (!is_tegra_hypervisor_mode())
		return -ENODEV;

	hvd = devm_kzalloc(dev, sizeof(*hvd), GFP_KERNEL);
	if (hvd == NULL) {
		dev_err(dev, "Failed to allocate hvd structure\n");
		return -ENOMEM;
	}

	hvd->pdev = pdev;

	ret = tegra_hv_setup(hvd);
	if (ret != 0) {
		tegra_hv_cleanup(hvd);
		return ret;
	}

	/*
	 * Ensure that all contents of hvd are visible before they are visible
	 * to other threads.
	 */
	smp_wmb();

	tegra_hv_data = hvd;
	dev_info(dev, "initialized\n");

	return 0;
}

static int tegra_hv_release(struct platform_device *pdev)
{
	struct tegra_hv_data *hvd = platform_get_drvdata(pdev);

	/*
	 * Once IVC channels are available to the kernel, there is no supported
	 * way to remove them.
	 */
	if (tegra_hv_data != NULL)
		return -EINVAL;

	if (hvd != NULL)
		tegra_hv_cleanup(hvd);

	return 0;
}

static const struct of_device_id tegra_hv_of_match[] = {
	{ .compatible = "nvidia,tegra-hv", },
	{},
};

MODULE_DEVICE_TABLE(of, tegra_hv_of_match);

static struct platform_driver tegra_hv_driver = {
	.probe	= tegra_hv_probe,
	.remove = tegra_hv_release,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_hv",
		.of_match_table = of_match_ptr(tegra_hv_of_match),
	},
};

struct tegra_hv_ivc_cookie *tegra_hv_ivc_reserve(struct device_node *dn,
		int id, const struct tegra_hv_ivc_ops *ops)
{
	const struct tegra_hv_data *hvd = get_hvd();
	struct ivc_dev *ivc;
	struct tegra_hv_ivc_cookie *ivck;
	int ret;

	if (IS_ERR(hvd))
		return (void *)hvd;

	ivc = ivc_device_by_id(hvd, id);
	if (ivc == NULL)
		return ERR_PTR(-ENODEV);

	mutex_lock(&ivc->lock);
	if (ivc->reserved) {
		ret = -EBUSY;
	} else {
		ivc->reserved = 1;
		ret = 0;
	}
	mutex_unlock(&ivc->lock);

	if (ret != 0)
		return ERR_PTR(ret);

	ivc->cookie_ops = ops;

	ivck = &ivc->cookie;
	ivck->irq = ivc->qd->irq;
	ivck->peer_vmid = ivc->other_guestid;
	ivck->nframes = ivc->qd->nframes;
	ivck->frame_size = ivc->qd->frame_size;

	if (ivc->cookie_ops) {
		ivc_handle_notification(ivc);
		/* request our irq */
		ret = ivc_request_cookie_irq(ivc);
		if (ret) {
			mutex_lock(&ivc->lock);
			BUG_ON(!ivc->reserved);
			ivc->reserved = 0;
			mutex_unlock(&ivc->lock);
			return ERR_PTR(ret);
		}
	}

	/* return pointer to the cookie */
	return ivck;
}
EXPORT_SYMBOL(tegra_hv_ivc_reserve);

int tegra_hv_ivc_unreserve(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc;
	int ret;

	if (ivck == NULL)
		return -EINVAL;

	ivc = cookie_to_ivc_dev(ivck);

	mutex_lock(&ivc->lock);
	if (ivc->reserved) {
		if (ivc->cookie_ops)
			ivc_release_irq(ivc);
		ivc->cookie_ops = NULL;
		ivc->reserved = 0;
		ret = 0;
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&ivc->lock);

	return ret;
}
EXPORT_SYMBOL(tegra_hv_ivc_unreserve);

int tegra_hv_ivc_write(struct tegra_hv_ivc_cookie *ivck, const void *buf,
		int size)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_write(ivc, buf, size);
}
EXPORT_SYMBOL(tegra_hv_ivc_write);

int tegra_hv_ivc_read(struct tegra_hv_ivc_cookie *ivck, void *buf, int size)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_read(ivc, buf, size);
}
EXPORT_SYMBOL(tegra_hv_ivc_read);

int tegra_hv_ivc_read_peek(struct tegra_hv_ivc_cookie *ivck, void *buf,
			   int off, int count)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_read_peek(ivc, buf, off, count);
}
EXPORT_SYMBOL(tegra_hv_ivc_read_peek);

int tegra_hv_ivc_can_read(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_can_read(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_can_read);

int tegra_hv_ivc_can_write(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_can_write(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_can_write);

int tegra_hv_ivc_tx_empty(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_tx_empty(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_tx_empty);

uint32_t tegra_hv_ivc_tx_frames_available(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_tx_frames_available(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_tx_frames_available);

int tegra_hv_ivc_dump(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);
	return ivc_dump(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_dump);

void *tegra_hv_ivc_read_get_next_frame(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_read_get_next_frame(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_read_get_next_frame);

void *tegra_hv_ivc_write_get_next_frame(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_write_get_next_frame(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_write_get_next_frame);

int tegra_hv_ivc_write_advance(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_write_advance(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_write_advance);

int tegra_hv_ivc_read_advance(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_read_advance(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_read_advance);

struct tegra_hv_ivm_cookie *tegra_hv_mempool_reserve(struct device_node *dn,
		unsigned id)
{
	uint32_t i;
	struct hv_mempool *mempool;
	int reserved;

	if (!tegra_hv_data)
		return ERR_PTR(-EPROBE_DEFER);

	/* Locate a mempool with matching id. */
	for (i = 0; i < tegra_hv_data->info->nr_mempools; i++) {
		mempool = &tegra_hv_data->mempools[i];
		if (mempool->mpd->id == id)
			break;
	}

	if (i == tegra_hv_data->info->nr_mempools)
		return ERR_PTR(-ENODEV);

	mutex_lock(&mempool->lock);
	reserved = mempool->reserved;
	mempool->reserved = 1;
	mutex_unlock(&mempool->lock);

	return reserved ? ERR_PTR(-EBUSY) : &mempool->ivmk;
}
EXPORT_SYMBOL(tegra_hv_mempool_reserve);

int tegra_hv_mempool_unreserve(struct tegra_hv_ivm_cookie *ivmk)
{
	int reserved;
	struct hv_mempool *mempool = container_of(ivmk, struct hv_mempool,
			ivmk);

	mutex_lock(&mempool->lock);
	reserved = mempool->reserved;
	mempool->reserved = 0;
	mutex_unlock(&mempool->lock);

	return reserved ? 0 : -EINVAL;
}
EXPORT_SYMBOL(tegra_hv_mempool_unreserve);

int tegra_hv_ivc_channel_notified(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc *ivc = &cookie_to_ivc_dev(ivck)->ivc;

	return tegra_ivc_channel_notified(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_channel_notified);

void tegra_hv_ivc_channel_reset(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	if (ivc->cookie_ops) {
		dev_err(ivc->device, "reset unsupported with callbacks");
		BUG();
	}

	tegra_ivc_channel_reset(&ivc->ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_channel_reset);

static int __init tegra_hv_mod_init(void)
{
	return platform_driver_register(&tegra_hv_driver);
}

static void __exit tegra_hv_mod_exit(void)
{
	platform_driver_unregister(&tegra_hv_driver);
}

core_initcall(tegra_hv_mod_init);
module_exit(tegra_hv_mod_exit);

MODULE_LICENSE("GPL");
