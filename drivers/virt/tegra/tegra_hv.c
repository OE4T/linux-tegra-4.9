/*
 * Tegra Hypervisor manager
 *
 * Instantiates virtualization-related resources.
 *
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/tegra-ivc-config.h>

#include "syscalls.h"
#include "tegra-ivc-internal.h"

/* very simple checksum over the shared data */
static u32 tegra_hv_server_data_sum(const struct tegra_hv_shared_data *shd)
{
	const u32 *p;
	u32 sum;
	int i, sz;

	sz = tegra_hv_server_data_size(shd->nr_queues);
	p = (const u32 *)shd;
	sum = 0;
	for (i = 0; i < sz; i += sizeof(u32))
		sum += *p++;

	return (u32)-sum;
}

/* shared data are valid only when the magic signature is present and
 * the whole area's checksum is zero */
static int tegra_hv_server_data_valid(const struct tegra_hv_shared_data *shd)
{
	return shd->magic == TEGRA_HV_SHD_MAGIC &&
		tegra_hv_server_data_sum(shd) == 0;
}

struct tegra_hv_data;

enum ivc_user {
	ivc_user_none,
	ivc_user_kernel,
	ivc_user_loopback
};

struct ivc_dev {
	struct tegra_hv_data	*hvd;
	int			minor;
	struct list_head	list;
	dev_t			dev;
	struct cdev		cdev;
	struct device		*device;
	char			name[32];

	/* channel configuration */
	struct ivc		ivc;
	struct tegra_hv_queue_data qd;
	int			other_guestid;
	int			irq;

	/*
	 * This is the irq of the peer channel in the case of loopback pairs,
	 * otherwise it's the same as irq.
	 */
	int			raise_irq;

	/*
	 * Use of the channels is controlled by the user enum and synchronized
	 * by the lock. The rules are as follows:
	 *
	 * 1. Only the owner of the device can transition to unused.
	 * 2. Only the owner of the device can modify the cookie and irq
	 *    handler.
	 * 3. No transitions between non-unused users.
	 * 4. The kernel user indicates that whoever in the kernel took
	 *    ownership of the channel must return it to unused.
	 * 5. This sysfs loopback attribute can only transition between unused
	 *    and loopback.
	 */
	struct mutex		lock;
	enum ivc_user		user;

	const struct tegra_hv_ivc_ops *cookie_ops;
	struct tegra_hv_ivc_cookie cookie;

	wait_queue_head_t	wq;		/* wait queue for file mode */
};

#define cookie_to_ivc_dev(_cookie) \
	container_of(_cookie, struct ivc_dev, cookie)

#define NGUESTS_MAX		8

/* Describe all info needed to do IVC to one particular guest */
struct guest_ivc_info {
	struct hyp_ivc_info res;
	void __iomem *shmem;			/* IO remapped shmem */
	struct tegra_hv_shared_data *shd;
	unsigned int server:1;		/* set when we're server */
	unsigned int valid:1;		/* set on valid guest */
	int max_qid;			/* max queue id discovered */
};

struct tegra_hv_data {
	struct list_head list;
	struct platform_device *pdev;
	int guestid;
	int nguests;
	struct guest_ivc_info guest_ivc_info[NGUESTS_MAX];
	unsigned int server;

	/* ivc data */
	struct mutex ivc_devs_lock;
	struct list_head ivc_devs;
	struct class *ivc_class;
	int ivc_major;
	int ivc_max_qid;
	dev_t ivc_dev;
};

static struct list_head hv_list = LIST_HEAD_INIT(hv_list);
static DEFINE_SPINLOCK(hv_list_lock);

static void ivc_raise_irq(struct ivc *ivc_channel)
{
	struct ivc_dev *ivc = container_of(ivc_channel, struct ivc_dev, ivc);
	hyp_raise_irq(ivc->raise_irq, ivc->other_guestid);
}

struct parsed_queue_data {
	u32 reg;
	u32 peers[2];
	u32 nframes;
	u32 frame_size;
	u32 size;
};

static int tegra_hv_dt_parse_queue_node(struct tegra_hv_data *hvd,
		struct device_node *dn, struct parsed_queue_data *pqd,
		int target_guestid, int server)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct parsed_queue_data tpqd;

	/* if pqd is NULL perform validity check only (still need structure) */
	if (pqd == NULL)
		pqd = &tpqd;
	memset(pqd, 0, sizeof(*pqd));

	/* required properties */
	if (of_property_read_u32(dn, "reg", &pqd->reg) != 0) {
		if (pqd != &tpqd)
			dev_err(dev, "@%s reg property missing\n",
					dn->full_name);
		return -EINVAL;
	}

	if (of_property_read_u32_array(dn, "peers", pqd->peers, 2) != 0) {
		if (pqd != &tpqd)
			dev_err(dev, "@%s peers property missing\n",
					dn->full_name);
		return -EINVAL;
	}

	if (hvd->guestid != pqd->peers[0] && hvd->guestid != pqd->peers[1]) {
		if (pqd != &tpqd)
			dev_warn(dev, "@%s (%d not in <%d %d>)\n",
				dn->full_name, hvd->guestid,
				pqd->peers[0], pqd->peers[1]);
		return -EINVAL;
	}

	/* we should ignore peers that are not present */
	if (pqd->peers[0] >= (u32)hvd->nguests ||
		pqd->peers[1] >= (u32)hvd->nguests) {
		if (pqd != &tpqd)
			dev_warn(dev, "@%s (one of <%d %d> greater than %d)\n",
				dn->full_name,
				pqd->peers[0], pqd->peers[1],
				hvd->nguests - 1);
		return -EINVAL;
	}

	/* the target must be one of the peers */
	if (target_guestid != pqd->peers[0] && target_guestid != pqd->peers[1])
		return -EINVAL;

	/* finally if both the peers are us we only allow self */
	if (target_guestid == hvd->guestid &&
		(pqd->peers[0] != hvd->guestid ||
			pqd->peers[1] != hvd->guestid))
			return -EINVAL;

	/* if we're not a server we don't need the following properties */
	if (!server)
		return 0;

	/* read properties (note no error checking) */
	of_property_read_u32(dn, "nframes", &pqd->nframes);
	of_property_read_u32(dn, "frame-size", &pqd->frame_size);
	of_property_read_u32(dn, "size", &pqd->size);

	/* if no size property given, construct one */
	if (pqd->size == 0)
		pqd->size = pqd->nframes * pqd->frame_size;

	/* not valid */
	if (pqd->size == 0) {
		if (pqd != &tpqd)
			dev_err(dev, "@%s size=0\n", dn->full_name);
		return -EINVAL;
	}

	/* single byte */
	if (pqd->nframes == 0 || pqd->frame_size == 0) {
		pqd->nframes = pqd->size;
		pqd->frame_size = 1;
	}

	return 0;
}

static int tegra_hv_dt_parse(struct tegra_hv_data *hvd,
		int target_guestid)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *qdn, *dn;
	uintptr_t p;
	int count, ret = 0, i, cfgsize;
	struct parsed_queue_data pqd;
	u32 server_to;
	struct tegra_hv_queue_data *qd;
	struct guest_ivc_info *givci;

	if ((unsigned int)target_guestid >= hvd->nguests)
		return -EINVAL;

	givci = &hvd->guest_ivc_info[target_guestid];

	/*
	 * Skip areas disabled by the hypervisor, regardless of whether they're
	 * present in the device tree.
	 */
	if (!givci->shd)
		return 0;

	/* queues node must exist (on server or client both) */
	qdn = of_get_child_by_name(dev->of_node, "queues");
	if (qdn == NULL)
		return -EINVAL;

	/* find out if the target_guestid is in the server-to-peers property */
	for (i = 0; of_property_read_u32_index(dev->of_node,
			"server-to-peers", i,
			&server_to) == 0; i++) {
		if (server_to == target_guestid) {
			givci->server = 1;
			break;
		}
	}

	dev_info(dev, "we are guest #%d and we are a %s to guest #%d\n",
			hvd->guestid,
			givci->server ? "server" : "client",
			target_guestid);

	/* not a server, we don't configure the shared memory */
	if (!givci->server)
		return 0;

	/* iterate over the child nodes to find count */
	count = 0;
	for_each_child_of_node(qdn, dn) {
		/* only increase count for valid nodes */
		if (tegra_hv_dt_parse_queue_node(hvd, dn,
					NULL, target_guestid,
					givci->server) == 0)
			count++;
	}

	/* Loopback channels occupy 2 queues. */
	if (target_guestid == hvd->guestid)
		count *= 2;

	if (count == 0) {
		dev_warn(dev, "guest #%d, no queues found\n",
				target_guestid);
		ret = 0;
		goto out;
	}
	givci->valid = 1;	/* valid guest */

	cfgsize = tegra_ivc_align(tegra_hv_server_data_size(count));
	if (cfgsize >= givci->res.size) {
		dev_err(dev, "guest #%d, size %d too large (> %lu)\n",
				target_guestid, cfgsize, givci->res.size);
		ret = -ENOMEM;
		goto out;
	}

	/* point right after the queue configuration data */
	p = (uintptr_t)givci->shd + cfgsize;

	dev_info(dev, "guest #%d, starting at 0x%08x - #%d queues\n",
			target_guestid, cfgsize, count);

	i = 0;
	for_each_child_of_node(qdn, dn) {
		unsigned queue_size;

		/* don't process bad nodes or with non-participating guestid */
		if (tegra_hv_dt_parse_queue_node(hvd, dn, &pqd,
					target_guestid, givci->server) != 0)
			continue;

		/* make sure you don't overwrite the shared data */
		queue_size = tegra_ivc_total_queue_size(pqd.size);
		if (queue_size == 0) {
			dev_err(dev, "guest %d, queue %u: bad size: %u\n",
					target_guestid, pqd.reg, pqd.size);
			ret = -ENOMEM;
			goto out;
		}
		if (p + queue_size * 2 >= (uintptr_t)givci->shd +
				givci->res.size) {
			dev_err(dev, "guest #%d, overflow of shared memory\n",
					target_guestid);
			ret = -ENOMEM;
			goto out;
		}

		qd = tegra_hv_shd_to_queue_data(givci->shd) + i;

		qd->id = pqd.reg;
		qd->peers[0] = pqd.peers[0];
		qd->peers[1] = pqd.peers[1];
		qd->size = queue_size;
		qd->nframes = pqd.nframes;
		qd->frame_size = pqd.frame_size;
		qd->offset = p - (uintptr_t)givci->shd;
		qd->flags = 0;

		/*
		 * Add an extra queue for loopback channels. Synthesize an
		 * additional queue referring to the same region in the next
		 * position.
		 */
		if (pqd.peers[0] == pqd.peers[1]) {
			struct tegra_hv_queue_data *qd2 = qd + 1;
			*qd2 = *qd;
			qd2->id = pqd.reg + 1;
			/*
			 * flags is unused, so use it to stash the id of its
			 * loopback peer for now.
			 */
			qd->flags = pqd.reg + 1;
			qd2->flags = pqd.reg;
			i++;
		}

		if (tegra_ivc_init_shared_memory(p, p + queue_size, pqd.nframes,
				pqd.frame_size) != 0) {
			dev_err(dev, "guest %d, Q#%d: failed shmem init\n",
					target_guestid, qd->id);
			ret = -EINVAL;
			goto out;
		}

		dev_info(dev, "guest #%d, Q#%d: <%d %d> base=%lx, size=%06x",
				target_guestid, qd->id,
				qd->peers[0], qd->peers[1], p, queue_size);

		p += queue_size * 2;

		i++;
	}

	givci->shd->nr_queues = i;
	givci->shd->flags = 0;
	givci->shd->sum = 0;	/* initial */
	givci->shd->magic = TEGRA_HV_SHD_MAGIC;

	/*
	 * Ensure that a valid shd checksum is visible only after all other
	 * stores to the configuration area.
	 */
	smp_wmb();

	/* update checksum */
	givci->shd->sum = tegra_hv_server_data_sum(givci->shd);

out:
	of_node_put(qdn);
	return ret;
}

static void loopback_handler(struct tegra_hv_ivc_cookie *cookie)
{
	struct ivc_dev *ivcd = container_of(cookie, struct ivc_dev, cookie);
	struct ivc *ivc = &ivcd->ivc;

	/* got input and space is available */
	while (tegra_ivc_can_read(ivc) && tegra_ivc_can_write(ivc)) {

		void *src = tegra_ivc_read_get_next_frame(ivc);
		/*
		 * We can't trust the other end not to cause the channel to
		 * become empty, resulting in src being an invalid pointer.
		 */
		if (IS_ERR(src)) {
			dev_err(ivcd->device, "channel unexpectedly empty\n");
			break;
		}

		tegra_ivc_write_poke(ivc, src, 0, ivcd->qd.frame_size);
		tegra_ivc_read_advance(ivc);
		tegra_ivc_write_advance(ivc);
	}
}

struct tegra_hv_data *get_hvd(struct device_node *dn)
{
	struct tegra_hv_data *hvd = NULL;
	struct platform_device *pdev = NULL;

	if (dn == NULL) {
		/* grab the first in the list */
		spin_lock(&hv_list_lock);
		if (!list_empty(&hv_list)) {
			hvd = list_entry(hv_list.next,
					struct tegra_hv_data, list);
			pdev = hvd->pdev;
			get_device(&pdev->dev);
		}
		spin_unlock(&hv_list_lock);
	} else
		pdev = of_find_device_by_node(dn);

	if (pdev != NULL) {
		hvd = platform_get_drvdata(pdev);
		platform_device_put(pdev);
	}

	if (pdev == NULL || hvd == NULL) {
		if (pdev != NULL && hvd == NULL) {
			pr_err("%s: Called too early (return EPROBE_DEFER)\n",
					__func__);
			return ERR_PTR(-EPROBE_DEFER);
		}
	}

	return hvd;
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

static irqreturn_t ivc_dev_irq_handler(int irq, void *data)
{
	struct ivc_dev *ivc = data;

	/* simple implementation, just kick all waiters */
	wake_up_interruptible_all(&ivc->wq);

	return IRQ_HANDLED;
}

static void ivc_release_irq(struct ivc_dev *ivc)
{
	BUG_ON(!ivc);

	devm_free_irq(ivc->device, ivc->irq, ivc);
}

static int ivc_request_cookie_irq(struct ivc_dev *ivcd)
{
	return devm_request_irq(ivcd->device, ivcd->irq,
			ivc_dev_cookie_irq_handler, 0, dev_name(ivcd->device),
			ivcd);
}

static const struct tegra_hv_ivc_ops loopback_ops = {
	.rx_rdy = loopback_handler,
	.tx_rdy = loopback_handler,
};

static int ivc_enable_loopback(struct ivc_dev *ivc)
{
	int ret;
	struct tegra_hv_data *hvd = get_hvd(NULL);
	if (IS_ERR(hvd))
		return -EINVAL;

	mutex_lock(&ivc->lock);

	if (ivc->user == ivc_user_loopback) {
		ret = 0;
	} else if (ivc->user != ivc_user_none) {
		ret = -EBUSY;
	} else {
		ivc->user = ivc_user_loopback;
		ivc->cookie_ops = &loopback_ops;
		ret = ivc_request_cookie_irq(ivc);
	}

	mutex_unlock(&ivc->lock);

	return ret;
}

static int ivc_disable_loopback(struct ivc_dev *ivc)
{
	int ret;

	mutex_lock(&ivc->lock);

	if (ivc->user == ivc_user_loopback) {
		ivc_release_irq(ivc);
		ivc->cookie_ops = NULL;
		ivc->user = ivc_user_none;
		ret = 0;
	} else
		ret = -EINVAL;

	mutex_unlock(&ivc->lock);

	return ret;
}

static int ivc_dev_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct ivc_dev *ivc = container_of(cdev, struct ivc_dev, cdev);
	int ret;

	mutex_lock(&ivc->lock);
	if (ivc->user != ivc_user_none) {
		ret = -EBUSY;
	} else {
		/*
		 * The user is listed as kernel because in the file case, we
		 * can still depend on the kernel to release the device when
		 * appropriate.
		 */
		ivc->user = ivc_user_kernel;
		ret = 0;
	}
	mutex_unlock(&ivc->lock);

	ivc->cookie_ops = NULL;

	if (ret)
		return ret;

	/* request our irq */
	ret = devm_request_irq(ivc->device, ivc->irq, ivc_dev_irq_handler, 0,
			dev_name(ivc->device), ivc);
	if (ret < 0) {
		dev_err(ivc->device, "Failed to request irq %d\n", ivc->irq);
		mutex_lock(&ivc->lock);
		BUG_ON(ivc->user != ivc_user_kernel);
		ivc->user = ivc_user_none;
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
	BUG_ON(ivc->user != ivc_user_kernel);
	ivc->user = ivc_user_none;
	mutex_unlock(&ivc->lock);

	return 0;
}

static int ivc_dump(struct ivc_dev *ivc)
{
	struct tegra_hv_data *hvd = ivc->hvd;
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;

	dev_info(dev, "IVC#%d: IRQ=%d nframes=%d frame_size=%d offset=%d\n",
			ivc->qd.id, ivc->irq,
			ivc->qd.nframes, ivc->qd.frame_size, ivc->qd.offset);

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

		chunk = ivcd->qd.frame_size;
		if (chunk > left)
			chunk = left;
		ret = tegra_ivc_read_user(ivc, buf, chunk);
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

		if (left < ivcd->qd.frame_size)
			chunk = left;
		else
			chunk = ivcd->qd.frame_size;

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

		ret = tegra_ivc_write_user(ivc, buf, chunk);
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

static ssize_t id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->qd.id);
}

static ssize_t frame_size_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->qd.frame_size);
}

static ssize_t nframes_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->qd.nframes);
}

static ssize_t opened_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	int opened;

	mutex_lock(&ivc->lock);
	/* We can't distinguish between file and kernel users. */
	opened = ivc->user == ivc_user_kernel;
	mutex_unlock(&ivc->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", opened);
}

static ssize_t reserved_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	int reserved;

	mutex_lock(&ivc->lock);
	reserved = ivc->user != ivc_user_none;
	mutex_unlock(&ivc->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", reserved);
}

static ssize_t peer_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->other_guestid);
}

static ssize_t loopback_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	int loopback;

	mutex_lock(&ivc->lock);
	loopback = ivc->user == ivc_user_loopback;
	mutex_unlock(&ivc->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", loopback);
}

ssize_t loopback_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t len)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	long int mode;
	int ret;

	ret = kstrtol(buf, 10, &mode);
	if (ret)
		return -EINVAL;

	if (mode == 0)
		ret = ivc_disable_loopback(ivc);
	else if (mode == 1)
		ret = ivc_enable_loopback(ivc);
	else
		ret = -EINVAL;

	if (ret != 0)
		return ret;
	else
		return len;
}

struct device_attribute ivc_dev_attrs[] = {
	__ATTR_RO(id),
	__ATTR_RO(frame_size),
	__ATTR_RO(nframes),
	__ATTR_RO(opened),
	__ATTR_RO(reserved),
	__ATTR_RO(peer),
	__ATTR(loopback, 0644, loopback_show, loopback_store),
	__ATTR_NULL
};

static int tegra_hv_add_ivc(struct tegra_hv_data *hvd,
		struct tegra_hv_queue_data *qd)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct guest_ivc_info *givci;
	struct ivc_dev *ivc;
	int ret;
	int rx_first;
	uintptr_t rx_base, tx_base;

	ivc = kzalloc(sizeof(*ivc), GFP_KERNEL);
	if (ivc == NULL) {
		dev_err(dev, "Failed to allocate ivc structure\n");
		return -ENOMEM;
	}
	ivc->hvd = hvd;

	if (qd->peers[0] == hvd->guestid)
		ivc->other_guestid = qd->peers[1];
	else if (qd->peers[1] == hvd->guestid)
		ivc->other_guestid = qd->peers[0];
	else
		BUG();

	givci = &hvd->guest_ivc_info[ivc->other_guestid];

	/*
	 * Make another copy of qd in the ivc_dev because qd is presumed to be
	 * in local, temporary memory.
	 */
	ivc->qd = *qd;

	mutex_init(&ivc->lock);
	init_waitqueue_head(&ivc->wq);

	ivc->minor = qd->id;
	ivc->dev = MKDEV(hvd->ivc_major, ivc->minor);
	cdev_init(&ivc->cdev, &ivc_fops);
	snprintf(ivc->name, sizeof(ivc->name) - 1, "ivc%d", qd->id);
	ret = cdev_add(&ivc->cdev, ivc->dev, 1);
	if (ret != 0) {
		dev_err(dev, "cdev_add() failed\n");
		goto out_free;
	}
	/* parent is this hvd dev */
	ivc->device = device_create(hvd->ivc_class, dev, ivc->dev, ivc,
			ivc->name);
	if (IS_ERR(ivc->device)) {
		dev_err(dev, "device_create() failed for %s\n", ivc->name);
		goto out_clear_cdev;
	}
	/* point to ivc */
	dev_set_drvdata(ivc->device, ivc);

	if (qd->peers[0] == qd->peers[1]) {
		/*
		 * For loopback pairs, qd->flags holds the id of the peer.
		 * In this case, the channel with the smaller id receives in
		 * the first half of the area.
		 */
		rx_first = qd->id < qd->flags;
	} else {
		rx_first = hvd->guestid == qd->peers[0];
	}

	if (rx_first) {
		rx_base = (uintptr_t)givci->shd + qd->offset;
		tx_base = (uintptr_t)givci->shd + qd->offset + qd->size;
	} else {
		tx_base = (uintptr_t)givci->shd + qd->offset;
		rx_base = (uintptr_t)givci->shd + qd->offset + qd->size;
	}

	dev_info(dev, "adding ivc%u: rx_base=%lx tx_base = %lx size=%x\n",
			qd->id, rx_base, tx_base, qd->size);
	tegra_ivc_init(&ivc->ivc, rx_base, tx_base, qd->nframes, qd->frame_size,
			NULL, ivc_raise_irq);

	/* IRQ# of this IVC channel */
	ivc->irq = givci->res.virq_base + qd->id;

	/* Loopback channels raise the IRQs of their peers. */
	if (qd->peers[0] == qd->peers[1])
		ivc->raise_irq = givci->res.virq_base + qd->flags;
	else
		ivc->raise_irq = ivc->irq;

	/* add to the list */
	mutex_lock(&hvd->ivc_devs_lock);
	list_add_tail(&ivc->list, &hvd->ivc_devs);
	mutex_unlock(&hvd->ivc_devs_lock);

	dev_info(dev, "added %s\n", ivc->name);

	return 0;
out_clear_cdev:
	cdev_del(&ivc->cdev);
out_free:
	kfree(ivc);
	return ret;
}

struct ivc_dev *ivc_device_by_id(struct tegra_hv_data *hvd, int id)
{
	struct ivc_dev *ivc = NULL;

	/* Locate the requested ivc device. */
	mutex_lock(&hvd->ivc_devs_lock);
	list_for_each_entry(ivc, &hvd->ivc_devs, list) {
		if (ivc->qd.id == id)
			break;
	}
	mutex_unlock(&hvd->ivc_devs_lock);

	return ivc;
}

static void tegra_hv_ivc_cleanup(struct tegra_hv_data *hvd)
{
	struct ivc_dev *ivc, *ivcn;

	mutex_lock(&hvd->ivc_devs_lock);
	list_for_each_entry_safe(ivc, ivcn, &hvd->ivc_devs, list) {
		list_del(&ivc->list);

		if (ivc->device) {
			cdev_del(&ivc->cdev);
			device_del(ivc->device);
		}
		kfree(ivc);
	}
	mutex_unlock(&hvd->ivc_devs_lock);
}

static int tegra_hv_prepare_to_instantiate(struct tegra_hv_data *hvd,
		int target_guestid)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct guest_ivc_info *givci;
	struct tegra_hv_shared_data *shd;
	struct tegra_hv_queue_data *qd;
	unsigned long timeout;
	int i;

	if ((unsigned int)target_guestid >= hvd->nguests)
		return -EINVAL;

	givci = &hvd->guest_ivc_info[target_guestid];

	/*
	 * Skip areas disabled by the hypervisor, regardless of whether they're
	 * present in the device tree.
	 */
	if (!givci->shd)
		return 0;

	/* non-valid guest on server just return */
	if (givci->server && !givci->valid)
		return 0;

	shd = givci->shd;

	if (!givci->server) {
		dev_info(dev, "slave; waiting for valid signature from %d\n",
				target_guestid);
		/* 8 second timeout */
		timeout = jiffies + 8 * HZ;
		while (time_before(jiffies, timeout)) {
			if (tegra_hv_server_data_valid(shd)) {
				/*
				 * Order validation of shd before queue data
				 * reads.
				 */
				smp_rmb();
				break;
			}
			msleep(20);
		}
		if (!tegra_hv_server_data_valid(shd)) {
			dev_warn(dev, "guest #%d, no configuration found\n",
					target_guestid);
			return 0;
		}
	} else if (!tegra_hv_server_data_valid(shd)) {
		/* on server this is fatal */
		dev_err(dev, "server guest #%d, no configuration found\n",
				target_guestid);
		return -EINVAL;
	}

	/* client, data found, set valid flag */
	if (!givci->server)
		givci->valid = 1;

	/* find maximum queue id for this quest */
	givci->max_qid = 0;
	for (i = 0; i < shd->nr_queues; i++) {
		qd = tegra_hv_shd_to_queue_data(shd) + i;
		if (qd->id > givci->max_qid)
			givci->max_qid = qd->id;
	}
	if (givci->max_qid > hvd->ivc_max_qid)
		hvd->ivc_max_qid = givci->max_qid;

	dev_info(dev, "guest #%d, config found; #%d total queues, max qid %d\n",
			target_guestid, shd->nr_queues, givci->max_qid);

	return 0;
}

static int queue_fits_in_shared_memory(struct tegra_hv_queue_data *qd,
		struct guest_ivc_info *givci)
{
	/*
	 * The config parameters are all 32-bit unsigned values, so we can
	 * eliminate a number of overflow checks related to the multiplication
	 * of two 32-bit unsigned values by promoting to 64-bit unsigned values.
	 */
	uint64_t area_size, frames_size, end_offset;

	/* Ensure that no 64-bit overflows can happen. */
	BUG_ON(!(sizeof(qd->offset) == 4 && sizeof(qd->nframes) == 4 &&
			sizeof(qd->frame_size) == 4));

	/* Ensure that area_size calculation can't underflow. */
	BUG_ON(givci->res.size <= sizeof(*givci->shd));

	area_size = givci->res.size - sizeof(*givci->shd);
	frames_size = qd->nframes * qd->frame_size;

	end_offset = frames_size + qd->offset;
	if (end_offset < frames_size) {
		/* end_offset overflowed */
		return 0;
	}

	return end_offset <= area_size;
}

static int tegra_hv_instantiate(struct tegra_hv_data *hvd, int target_guestid)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct tegra_hv_queue_data qd;
	struct guest_ivc_info *givci;
	struct tegra_hv_shared_data *shd;
	int ret, i, our_count;

	if ((unsigned int)target_guestid >= hvd->nguests)
		return -EINVAL;

	givci = &hvd->guest_ivc_info[target_guestid];
	if (!givci->valid)
		return 0;

	shd = givci->shd;

	/* we now have to iterate over the queues and setup the ivc devices */
	our_count = 0;
	for (i = 0; i < shd->nr_queues; i++) {

		/*
		 * Copy the queue data structure out of shared memory before
		 * validating its contents. We must perform these checks on a
		 * local copy of the structure, otherwise the checks could be
		 * performed on values that could be asynchronously modified,
		 * potentially after we've checked them.
		 */
		qd = ACCESS_ONCE(*(tegra_hv_shd_to_queue_data(shd) + i));

		/* we have to be one of the peers */
		if (qd.peers[0] != hvd->guestid && qd.peers[1] != hvd->guestid)
			continue;

		/* the target must be one of the peers */
		if (qd.peers[0] != target_guestid &&
				qd.peers[1] != target_guestid) {
			continue;
		}

		if (!queue_fits_in_shared_memory(&qd, givci)) {
			dev_err(dev, "queue %u doesn't fit in shared memory!\n",
					qd.id);
			continue;
		}

		/* add the IVC device */
		ret = tegra_hv_add_ivc(hvd, &qd);
		if (ret != 0) {
			dev_err(dev, "tegra_hv_add_ivc() failed\n");
			tegra_hv_ivc_cleanup(hvd);
			return ret;
		}
		our_count++;
	}

	dev_info(dev, "guest #%d, #%d IVC devices\n",
			target_guestid, our_count);

	return 0;
}

static int tegra_hv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tegra_hv_data *hvd;
	struct guest_ivc_info *givci;
	int i, ret;

	if (!is_tegra_hypervisor_mode())
		return -ENODEV;

	if (dev->of_node == NULL) {
		dev_err(dev, "driver requires DT data\n");
		return -EINVAL;
	}

	hvd = devm_kzalloc(dev, sizeof(*hvd), GFP_KERNEL);
	if (hvd == NULL) {
		dev_err(dev, "Failed to allocate hvd structure\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&hvd->list);

	hvd->pdev = pdev;

	ret = hyp_read_gid(&hvd->guestid);
	if (ret != 0) {
		dev_err(dev, "Failed to read guest id\n");
		return -ENODEV;
	}

	/* Read total number of guests advertised to us */
	ret = hyp_read_nguests(&hvd->nguests);
	dev_info(dev, "Number of guests in the system: %d\n",
			hvd->nguests);

	if (ret != 0) {
		dev_err(dev, "Failed to read guest id\n");
		return -ENODEV;
	}

	if (hvd->nguests >= NGUESTS_MAX) {
		dev_err(dev, "nguests=%d > %d (clamping to %d)\n", hvd->nguests,
				NGUESTS_MAX, NGUESTS_MAX);
		hvd->nguests = NGUESTS_MAX;
	}

	/* initialize the standard IVC stuff */
	mutex_init(&hvd->ivc_devs_lock);
	INIT_LIST_HEAD(&hvd->ivc_devs);

	hvd->ivc_class = class_create(THIS_MODULE, "ivc");
	if (IS_ERR(hvd->ivc_class)) {
		dev_err(dev, "class_create() failed\n");
		ret = PTR_ERR(hvd->ivc_class);
		hvd->ivc_class = NULL;
		return ret;
	}
	/* set class attributes */
	hvd->ivc_class->dev_attrs = ivc_dev_attrs;

	/* For all advertised guests, read IVC comm info */
	for (i = 0; i < hvd->nguests; i++) {
		givci = &hvd->guest_ivc_info[i];

		/* Get info for this guest */
		ret = hyp_read_ivc_info(&givci->res, i);
		if (ret != 0) {
			dev_warn(dev, "No hyp shared memory for guest #%d\n",
					i);
			continue;
		}

		/* Map segment for this guest  */
		givci->shmem = ioremap_cached(givci->res.base,
				givci->res.size);
		if (givci->shmem == NULL) {
			dev_err(dev, "guest #%d, shmem ioremap failed\n", i);

			/* iounmap all previous ones */
			while (--i >= 0) {
				givci = &hvd->guest_ivc_info[i];
				iounmap(givci->shmem);
			}
			ret = -ENOMEM;
			goto out_class_rel;
		}

		/* Also assign shd field for this guest */
		givci->shd = (void *)givci->shmem;
	}

	/* parse the DT data */
	for (i = 0; i < hvd->nguests; i++) {
		ret = tegra_hv_dt_parse(hvd, i);
		if (ret != 0) {
			dev_err(dev, "Failed to parse DT data\n");
			goto out_unmap;
		}
	}

	/* prepare to instantiate the IVC */
	hvd->ivc_max_qid = 0;

	for (i = 0; i < hvd->nguests; i++) {
		ret = tegra_hv_prepare_to_instantiate(hvd, i);
		if (ret != 0) {
			dev_err(dev, "guest #%d, failed to prepare\n", i);
			goto out_unmap;
		}
	}

	/* have we found any IVCs? */
	if (hvd->ivc_max_qid == 0) {
		dev_err(dev, "No IVC channels\n");
		ret = -ENODEV;
		goto out_unmap;
	}

	dev_info(dev, "guestid=%d, total guests=%d\n",
			hvd->guestid, hvd->nguests);
	for (i = 0; i < hvd->nguests; i++) {
		givci = &hvd->guest_ivc_info[i];
		dev_info(dev, "guest #%d, shmem: mem=0x%lx-0x%lx irq=%lu-%u %c%c\n",
				i, givci->res.base, givci->res.size,
				givci->res.virq_base, givci->res.virq_total,
				givci->valid ? 'V' : '-',
				givci->server ? 'S' : '-');
	}

	/* allocate the whole chardev range */
	ret = alloc_chrdev_region(&hvd->ivc_dev, 0, hvd->ivc_max_qid + 1,
					"ivc");
	if (ret < 0) {
		dev_err(dev, "alloc_chrdev_region() failed\n");
		goto out_unmap;
	}
	hvd->ivc_major = MAJOR(hvd->ivc_dev);

	/* instantiate the IVC */
	for (i = 0; i < hvd->nguests; i++) {
		ret = tegra_hv_instantiate(hvd, i);
		if (ret != 0) {
			dev_err(dev, "guest #%d, failed to instantiate\n", i);
			goto out_unreg_chr;
		}
	}

	/* finally add it to the list */
	spin_lock(&hv_list_lock);
	platform_set_drvdata(pdev, hvd);
	list_add_tail(&hvd->list, &hv_list);
	spin_unlock(&hv_list_lock);

	dev_info(dev, "initialized\n");

	return 0;

out_unreg_chr:
	unregister_chrdev_region(hvd->ivc_dev, hvd->ivc_max_qid);

out_unmap:
	for (i = 0; i < hvd->nguests; i++)
		iounmap(hvd->guest_ivc_info[i].shmem);

out_class_rel:
	class_destroy(hvd->ivc_class);
	return ret;
}

static int tegra_hv_release(struct platform_device *pdev)
{
	struct tegra_hv_data *hvd = platform_get_drvdata(pdev);
	int i;

	/* remove from the list */
	spin_lock(&hv_list_lock);
	list_del(&hvd->list);
	spin_unlock(&hv_list_lock);

	tegra_hv_ivc_cleanup(hvd);

	unregister_chrdev_region(hvd->ivc_dev, hvd->ivc_max_qid);

	/* Unmap mapped areas for all guests */
	for (i = 0; i < hvd->nguests; i++)
		iounmap(hvd->guest_ivc_info[i].shmem);

	class_destroy(hvd->ivc_class);

	dev_info(&pdev->dev, "Released\n");

	/* no need to devm_free or release the irq (done automatically) */
	return 0;
}

static const struct of_device_id tegra_hv_of_match[] = {
	{ .compatible = "nvidia,tegra-hv", },
	{},
};

MODULE_DEVICE_TABLE(of, tegra_hv_of_match);

static struct platform_driver tegra_hv_driver = {
	.probe	= tegra_hv_probe,
	.remove	= tegra_hv_release,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tegra_hv",
		.of_match_table = of_match_ptr(tegra_hv_of_match),
	},
};

struct tegra_hv_ivc_cookie *tegra_hv_ivc_reserve(struct device_node *dn,
		int id, const struct tegra_hv_ivc_ops *ops)
{
	struct tegra_hv_data *hvd = get_hvd(dn);
	struct ivc_dev *ivc;
	struct tegra_hv_ivc_cookie *ivck;
	int ret;

	if (IS_ERR(hvd))
		return ERR_PTR(-ENODEV);

	ivc = ivc_device_by_id(hvd, id);
	if (ivc == NULL)
		return ERR_PTR(-ENODEV);

	mutex_lock(&ivc->lock);
	if (ivc->user != ivc_user_none) {
		ret = -EBUSY;
	} else {
		ivc->user = ivc_user_kernel;
		ret = 0;
	}
	mutex_unlock(&ivc->lock);

	if (ret != 0)
		return ERR_PTR(ret);

	ivc->cookie_ops = ops;

	ivck = &ivc->cookie;
	ivck->irq = ivc->irq;
	ivck->peer_vmid = ivc->other_guestid;
	ivck->nframes = ivc->qd.nframes;
	ivck->frame_size = ivc->qd.frame_size;

	if (ivc->cookie_ops) {
		ivc_handle_notification(ivc);
		/* request our irq */
		ret = ivc_request_cookie_irq(ivc);
		if (ret) {
			mutex_lock(&ivc->lock);
			BUG_ON(ivc->user != ivc_user_kernel);
			ivc->user = ivc_user_none;
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
	if (ivc->user == ivc_user_kernel) {
		if (ivc->cookie_ops)
			ivc_release_irq(ivc);
		ivc->cookie_ops = NULL;
		ivc->user = ivc_user_none;
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

int tegra_hv_ivc_set_loopback(struct tegra_hv_ivc_cookie *ivck, int mode)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	if (mode == 0)
		return ivc_disable_loopback(ivc);
	else if (mode == 1)
		return ivc_enable_loopback(ivc);
	else
		return -EINVAL;
}
EXPORT_SYMBOL(tegra_hv_ivc_set_loopback);

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
