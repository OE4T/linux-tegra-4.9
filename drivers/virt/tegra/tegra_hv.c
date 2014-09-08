/*
 * Tegra Hypervisor manager + IVC
 *
 * Implements both a kernel level interface as well as a
 * simple character level interface.
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
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/workqueue.h>

#include <linux/tegra-soc.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-config.h>

#include "syscalls.h"
#include "cbuf.h"

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

struct ivc_dev {
	struct tegra_hv_data	*hvd;
	int			minor;
	struct list_head	list;
	dev_t			dev;
	struct cdev		cdev;
	struct device		*device;
	char			name[32];

	/* channel configuration */
	struct tegra_hv_queue_data qd;
	struct cbuf		*rx_cbuf;
	struct cbuf		*tx_cbuf;
	int			other_guestid;
	int			irq;

	/* operation mode */
	spinlock_t		lock;		/* for accessing state */
	wait_queue_head_t	wq;		/* wait queue for file mode */
	unsigned int		opened:1;	/* set when opened */
	unsigned int		reserved:1;	/* set when reserved */
	unsigned int		loopback:1;	/* loopback active */

	int			local_loopback;	/* -1 no loopback */
	struct cbuf		*loopback_rx_cbuf;
	struct cbuf		*loopback_tx_cbuf;
	int			loopback_irq;

	/* in kernel cookies */
	const struct tegra_hv_ivc_ops *cookie_ops;
	struct tegra_hv_ivc_cookie cookie;
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
	unsigned int emulation:1;
	unsigned int server:1;

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

static inline int tegra_hv_get_queue_irq(struct tegra_hv_data *hvd,
		int guestid, int id)
{
	struct guest_ivc_info *givci;

	/* check if valid guest */
	if ((unsigned int)guestid >= hvd->nguests)
		return -1;

	givci = &hvd->guest_ivc_info[guestid];

	/* if non-valid guest fail */
	if (!givci->valid)
		return -1;

	/* check if valid queue id */
	if ((unsigned int)id >= givci->res.virq_total)
		return -1;

	/* simple mapping */
	return givci->res.virq_base + id;
}

static struct cbuf *ivc_tx_cbuf(struct ivc_dev *ivc)
{
	if (likely(!ivc->loopback_rx_cbuf))
		return ivc->tx_cbuf;

	/* on loopback we write to the rx queue of the other side */
	return ivc->loopback_rx_cbuf;
}

static int ivc_rx_empty(struct ivc_dev *ivc)
{
	return cbuf_is_empty(ivc->rx_cbuf);
}

static int ivc_tx_empty(struct ivc_dev *ivc)
{
	struct cbuf *cb = ivc_tx_cbuf(ivc);
	return cbuf_is_empty(cb);
}

static int ivc_tx_full(struct ivc_dev *ivc)
{
	struct cbuf *cb = ivc_tx_cbuf(ivc);
	return cbuf_is_full(cb);
}

static void ivc_raise_irq(struct ivc_dev *ivc)
{
	struct ivc_dev *ivcd;
	struct tegra_hv_ivc_cookie *ivckd;

	if (ivc->hvd->emulation) {
		if (ivc->local_loopback == -1)
			return;

		/* not really efficient, but emulation is a special case */
		mutex_lock(&ivc->hvd->ivc_devs_lock);
		list_for_each_entry(ivcd, &ivc->hvd->ivc_devs, list) {
			if (ivcd->qd.id == ivc->local_loopback) {
				ivckd = &ivcd->cookie;
				goto found;
			}
		}
		ivcd = NULL;
		ivckd = NULL;
found:
		if (ivcd != NULL)
			wake_up_interruptible_all(&ivcd->wq);
		mutex_unlock(&ivc->hvd->ivc_devs_lock);

		/* NOTE: callbacks are inherently racy */
		if (ivcd != NULL && ivcd->cookie_ops) {
			/* there are data in the queue, callback */
			if (ivcd->cookie_ops->rx_rdy && !ivc_rx_empty(ivcd))
				ivcd->cookie_ops->rx_rdy(ivckd);

			/* there is space in the queue to write, callback */
			if (ivcd->cookie_ops->tx_rdy && !ivc_tx_full(ivcd))
				ivcd->cookie_ops->tx_rdy(ivckd);
		}

		return;
	}

	if (!ivc->loopback_irq)
		hyp_raise_irq(ivc->irq, ivc->other_guestid);
	else
		hyp_raise_irq(ivc->loopback_irq, ivc->hvd->guestid);
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

	/*
	* Due to security implications only a star topology is supported.
	* What this means is that the server IVC is at the center of the
	* star, and communication is allowed between the server and
	* the clients at the tips of the star.
	*
	* The only exception is when this is a local loopback for another
	* peer.
	*/

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
	void *p;
	int count, ret = 0, i, cfgsize;
	struct parsed_queue_data pqd;
	u32 total_size, server_to;
	struct cbuf *tx_cbuf, *rx_cbuf;
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

	if (count == 0) {
		dev_warn(dev, "guest #%d, no queues found\n",
				target_guestid);
		ret = 0;
		goto out;
	}
	givci->valid = 1;	/* valid guest */

	cfgsize = tegra_hv_server_data_size(count);
	if (cfgsize >= givci->res.size) {
		dev_err(dev, "guest #%d, size %d too large (> %lu)\n",
				target_guestid, cfgsize, givci->res.size);
		ret = -ENOMEM;
		goto out;
	}

	/* point right after the queue configuration data */
	p = (void *)givci->shd + cfgsize;

	dev_info(dev, "guest #%d, starting at 0x%08x - #%d queues\n",
			target_guestid, cfgsize, count);

	i = 0;
	for_each_child_of_node(qdn, dn) {

		/* don't process bad nodes or with non-participating guestid */
		if (tegra_hv_dt_parse_queue_node(hvd, dn, &pqd,
					target_guestid, givci->server) != 0)
			continue;

		/* make sure you don't overwrite the shared data */
		total_size = pqd.size * 2;
		if (pqd.frame_size != 1)
			total_size += pqd.nframes * sizeof(u32);

		if (p + total_size >= (void *)givci->shd + givci->res.size) {
			dev_err(dev, "guest #%d, overflow of shared memory\n",
					target_guestid);
			ret = -ENOMEM;
			goto out;
		}

		qd = tegra_hv_shd_to_queue_data(givci->shd) + i;

		qd->id = pqd.reg;
		qd->peers[0] = pqd.peers[0];
		qd->peers[1] = pqd.peers[1];
		qd->size = pqd.size + sizeof(struct cbuf);
		qd->nframes = pqd.nframes;
		qd->frame_size = pqd.frame_size;
		qd->offset = p - (void *)givci->shd;
		qd->flags = 0;

		if (pqd.frame_size == 1)
			qd->flags |= TQF_STREAM_MODE;

		/*
		 * The first cbuf is guaranteed to be adequately aligned given
		 * that it follows a struct placed at the start of a page.
		 */
		BUG_ON((uintptr_t)p & (sizeof(int) - 1));
		tx_cbuf = cbuf_init(p, pqd.frame_size, pqd.nframes);
		p += pqd.size + sizeof(struct cbuf);
		if ((qd->flags & TQF_STREAM_MODE) != 0)
			p += pqd.nframes * sizeof(u32);

		/*
		 * The cbuf must at least be aligned enough for its counters
		 * to be accessed atomically.
		 */
		if ((uintptr_t)p & (sizeof(int) - 1)) {
			pr_err("qid %u: rx_cbuf start not aligned: %p\n",
					qd->id, p);
			return -EINVAL;
		}
		rx_cbuf = cbuf_init(p, pqd.frame_size, pqd.nframes);
		p += pqd.size + sizeof(struct cbuf);
		if ((qd->flags & TQF_STREAM_MODE) != 0)
			p += pqd.nframes * sizeof(u32);

		dev_info(dev, "guest #%d, Q#%d: <%d %d> cbuf #0 @ 0x%06x, cbuf #1 @ 0x%06x\n",
				target_guestid, qd->id,
				qd->peers[0], qd->peers[1],
				(void *)tx_cbuf - (void *)givci->shd,
				(void *)rx_cbuf - (void *)givci->shd);

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
	cbuf_wmb();

	/* update checksum */
	givci->shd->sum = tegra_hv_server_data_sum(givci->shd);

out:
	of_node_put(qdn);
	return ret;
}

/*
 * Confirm that ptr is the start of at least a frame-sized region within the
 * cbuf starting at cb using no information obtained from shared data. This is
 * important because ptr may have been formed using untrustworthy values
 * accessed through cb.
 */
static int ivc_valid_pointer(struct ivc_dev *ivc, struct cbuf *cb,
		const void *ptr)
{
	size_t size = ivc->qd.frame_size * ivc->qd.nframes;
	uintptr_t base = (uintptr_t)&cb->buf;
	uintptr_t first = (uintptr_t)ptr;
	uintptr_t last = first + ivc->qd.frame_size - 1;
	return (first - base < size) && (last - base < size);
}

static int ivc_perform_loopback(struct ivc_dev *ivc)
{
	struct cbuf *rx, *tx;
	int count;
	/* Use the struct_size that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;

	if (!ivc->loopback)
		return -EINVAL;

	rx = ivc->rx_cbuf;
	tx = ivc->tx_cbuf;

	/* got input and space is available */
	count = 0;
	while (!cbuf_is_empty(rx) && !cbuf_is_full(tx)) {
		void *dest = cbuf_write_data_ptr(tx);
		const void *src = cbuf_read_data_ptr(rx);

		if (!ivc_valid_pointer(ivc, tx, dest)) {
			dev_err(ivc->device,
					"cbuf write pointer out of range\n");
			return -EINVAL;
		}

		if (!ivc_valid_pointer(ivc, rx, src)) {
			dev_err(ivc->device,
					"cbuf read pointer out of range\n");
			return -EINVAL;
		}

		/* copy data directly from rx to tx */
		memcpy(dest, src, struct_size);
		cbuf_advance_r_pos(rx);
		cbuf_advance_w_pos(tx);
		count++;
	}

	if (count > 0)
		ivc_raise_irq(ivc);

	return 0;
}

static irqreturn_t ivc_dev_loopback_irq_handler(int irq, void *data)
{
	struct ivc_dev *ivc = data;

	ivc_perform_loopback(ivc);

	return IRQ_HANDLED;
}

static int ivc_set_loopback(struct ivc_dev *ivc, int mode)
{
	int ret;

	if ((unsigned int)mode >= 2)
		return -EINVAL;

	ret = -EINVAL;

	spin_lock(&ivc->lock);

	/* should not be opened, neither reserved */
	if (ivc->opened || ivc->reserved)
		goto out;

	/* if switching loopback mode install specific handler */
	if (ivc->loopback != mode) {

		if (!ivc->hvd->emulation) {
			if (mode) {
				/* request our irq */
				ret = devm_request_irq(
						ivc->device, ivc->irq,
						ivc_dev_loopback_irq_handler,
						0, dev_name(ivc->device), ivc);
				if (ret != 0)
					goto out;
			} else
				devm_free_irq(ivc->device, ivc->irq, ivc);
		}

		ivc->loopback = mode;
	}

	ret = 0;
out:
	spin_unlock(&ivc->lock);

	return ret;
}

static irqreturn_t ivc_dev_irq_handler(int irq, void *data)
{
	struct ivc_dev *ivc = data;

	/* simple implementation, just kick all waiters */
	wake_up_interruptible_all(&ivc->wq);

	return IRQ_HANDLED;
}

static int ivc_dev_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct ivc_dev *ivc = container_of(cdev, struct ivc_dev, cdev);
	int ret;

	spin_lock(&ivc->lock);
	if (ivc->opened || ivc->reserved || ivc->loopback) {
		ret = -EBUSY;
		goto out;
	}

	ivc->opened = 1;
	spin_unlock(&ivc->lock);

	if (!ivc->hvd->emulation) {
		/* request our irq */
		ret = devm_request_irq(
				ivc->device, ivc->irq,
				ivc_dev_irq_handler,
				0, dev_name(ivc->device), ivc);
		if (ret < 0) {
			dev_err(ivc->device, "Failed to request irq %d\n",
					ivc->irq);
			goto err_no_irq;
		}
	}

	/* all done */
	filp->private_data = ivc;

	return 0;
err_no_irq:

	spin_lock(&ivc->lock);
	ivc->opened = 0;
out:
	spin_unlock(&ivc->lock);

	filp->private_data = NULL;

	return ret;
}

static int ivc_dev_release(struct inode *inode, struct file *filp)
{
	struct ivc_dev *ivc = filp->private_data;

	if (ivc == NULL)
		return 0;

	if (!ivc->hvd->emulation)
		devm_free_irq(ivc->device, ivc->irq, ivc);

	spin_lock(&ivc->lock);
	ivc->opened = 0;
	filp->private_data = NULL;
	spin_unlock(&ivc->lock);

	return 0;
}

static long ivc_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct ivc_dev *ivc = filp->private_data;

	/* placeholder */
	(void)ivc;

	switch (cmd) {
	default:
		return -ENOTTY;
	}
}

static int ivc_dump(struct ivc_dev *ivc)
{
	struct tegra_hv_data *hvd = ivc->hvd;
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct cbuf *cb;

	dev_info(dev, "IVC#%d: IRQ=%d nframes=%d frame_size=%d offset=%d\n",
			ivc->qd.id, ivc->irq,
			ivc->qd.nframes, ivc->qd.frame_size, ivc->qd.offset);

	cb = ivc->rx_cbuf;
	dev_info(dev, " RXCB: end_idx=%d struct_size=%d w_pos=%d r_pos=%d\n",
			cb->end_idx, cb->struct_size, cb->w_pos, cb->r_pos);
	cb = ivc->tx_cbuf;
	dev_info(dev, " TXCB: end_idx=%d struct_size=%d w_pos=%d r_pos=%d\n",
			cb->end_idx, cb->struct_size, cb->w_pos, cb->r_pos);

	return 0;
}

static int ivc_rx_read(struct ivc_dev *ivc, void *buf, int max_read)
{
	struct cbuf *cb = ivc->rx_cbuf;
	int ret, chunk, left;
	const void *src;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;

	if (cbuf_is_empty(cb))
		return -ENOMEM;

	if (max_read > struct_size) {
		chunk = struct_size;
		left = max_read - chunk;
	} else {
		chunk = max_read;
		left = 0;
	}

	src = cbuf_read_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, src))
		return -EINVAL;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	memcpy(buf, src, chunk);
	memset(buf + chunk, 0, left);

	cbuf_advance_r_pos(cb);
	ivc_raise_irq(ivc);

	ret = chunk;

	return ret;
}

static int ivc_rx_read_user(struct ivc_dev *ivc, void __user *buf, int max_read)
{
	struct cbuf *cb = ivc->rx_cbuf;
	int ret, chunk, left;
	const void *src;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;

	if (cbuf_is_empty(cb))
		return -ENOMEM;

	if (max_read > struct_size) {
		chunk = struct_size;
		left = max_read - chunk;
	} else {
		chunk = max_read;
		left = 0;
	}

	src = cbuf_read_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, src))
		return -EINVAL;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	if (copy_to_user(buf, src, chunk))
		return -EFAULT;
	if (left > 0 && clear_user(buf + chunk, left))
		return -EFAULT;

	cbuf_advance_r_pos(cb);
	ivc_raise_irq(ivc);

	ret = chunk;

	return ret;
}

/* peek in the next rx buffer at offset off, the count bytes */
static int ivc_rx_peek(struct ivc_dev *ivc, void *buf, int off, int count)
{
	struct cbuf *cb = ivc->rx_cbuf;
	int chunk, rem;
	const void *src;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;

	if (cbuf_is_empty(cb))
		return -ENOMEM;

	/* get maximum available number of bytes */
	rem = struct_size - off;
	chunk = count;

	/* if request is for more than rem, return only rem */
	if (chunk > rem)
		chunk = rem;

	src = cbuf_read_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, src))
		return -EINVAL;

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	memcpy(buf, src + off, chunk);

	/* note, no interrupt is generated */

	return chunk;
}

/* directly peek at the next frame rx'ed */
static void *ivc_rx_get_next_frame(struct ivc_dev *ivc)
{
	struct cbuf *cb = ivc->rx_cbuf;
	void *src;

	if (cbuf_is_empty(cb))
		return ERR_PTR(-ENOMEM);

	/*
	 * Order observation of w_pos potentially indicating new data before
	 * data read.
	 */
	cbuf_rmb();

	src = cbuf_read_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, src))
		return ERR_PTR(-EINVAL);
	else
		return src;
}

/* advance the rx buffer */
static int ivc_rx_advance(struct ivc_dev *ivc)
{
	struct cbuf *cb = ivc->rx_cbuf;

	if (cbuf_is_empty(cb))
		return -ENOMEM;

	cbuf_advance_r_pos(cb);
	ivc_raise_irq(ivc);

	return 0;
}

static int ivc_tx_write(struct ivc_dev *ivc, const void *buf, int size)
{
	struct cbuf *cb;
	void __iomem *p;
	int ret, left, chunk;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;

	/* in emulation mode, only local loopback modes are possible */
	if (ivc->hvd->emulation && ivc->local_loopback == -1)
		return -EINVAL;

	cb = ivc_tx_cbuf(ivc);
	if (cbuf_is_full(cb))
		return -ENOMEM;

	if (size > struct_size) {
		chunk = struct_size;
		left = size - struct_size;
	} else {
		chunk = size;
		left = struct_size - chunk;
	}

	p = cbuf_write_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, p))
		return -EINVAL;

	memcpy(p, buf, chunk);
	memset(p + chunk, 0, left);

	/*
	 * Ensure that updated data is visible before the w_pos counter
	 * indicates that it is ready.
	 */
	cbuf_wmb();

	cbuf_advance_w_pos(cb);
	ivc_raise_irq(ivc);

	ret = chunk;

	return size;
}

static int ivc_tx_write_user(struct ivc_dev *ivc,
		const void __user *buf, int size)
{
	struct cbuf *cb;
	void __iomem *p;
	int ret, left, chunk;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;

	/* in emulation mode, only local loopback modes are possible */
	if (ivc->hvd->emulation && ivc->local_loopback == -1)
		return -EINVAL;

	cb = ivc_tx_cbuf(ivc);
	if (cbuf_is_full(cb))
		return -ENOMEM;

	if (size > struct_size) {
		chunk = struct_size;
		left = size - struct_size;
	} else {
		chunk = size;
		left = struct_size - chunk;
	}

	p = cbuf_write_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, p))
		return -EINVAL;
	if (copy_from_user(p, buf, chunk))
		return -EFAULT;
	if (left > 0)
		memset(p + chunk, 0, left);

	/*
	 * Ensure that updated data is visible before the w_pos counter
	 * indicates that it is ready.
	 */
	cbuf_wmb();

	cbuf_advance_w_pos(cb);
	ivc_raise_irq(ivc);

	ret = chunk;

	return size;
}

/* poke in the next tx buffer at offset off, the count bytes */
static int ivc_tx_poke(struct ivc_dev *ivc, const void *buf, int off, int count)
{
	struct cbuf *cb;
	int rem, chunk;
	/* Use the frame_size value that was previously validated. */
	size_t struct_size = ivc->qd.frame_size;
	void *dest;

	/* in emulation mode, only local loopback modes are possible */
	if (ivc->hvd->emulation && ivc->local_loopback == -1)
		return -EINVAL;

	cb = ivc_tx_cbuf(ivc);
	if (cbuf_is_full(cb))
		return -ENOMEM;

	rem = struct_size + off;
	chunk = count;
	if (chunk > rem)
		chunk = rem;

	dest = cbuf_write_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, dest))
		return -EINVAL;
	memcpy(dest + off, buf, chunk);

	return chunk;
}

/* directly poke at the next frame to be tx'ed */
static void *ivc_tx_get_next_frame(struct ivc_dev *ivc)
{
	struct cbuf *cb;
	void *dest;

	/* in emulation mode, only local loopback modes are possible */
	if (ivc->hvd->emulation && ivc->local_loopback == -1)
		return ERR_PTR(-EINVAL);

	cb = ivc_tx_cbuf(ivc);
	if (cbuf_is_full(cb))
		return ERR_PTR(-ENOMEM);

	dest = cbuf_write_data_ptr(cb);
	if (!ivc_valid_pointer(ivc, cb, dest))
		return ERR_PTR(-EINVAL);
	else
		return dest;
}

/* advance the tx buffer */
static int ivc_tx_advance(struct ivc_dev *ivc)
{
	struct cbuf *cb;

	/* in emulation mode, only local loopback modes are possible */
	if (ivc->hvd->emulation && ivc->local_loopback == -1)
		return -EINVAL;

	cb = ivc_tx_cbuf(ivc);
	if (cbuf_is_full(cb))
		return -ENOMEM;

	/*
	 * Order any possible stores to the frame before update of w_pos.
	 */
	cbuf_wmb();

	cbuf_advance_w_pos(cb);
	ivc_raise_irq(ivc);

	return 0;
}

static ssize_t ivc_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ivc_dev *ivc = filp->private_data;
	int left = count, ret = 0, chunk;

	if (ivc_rx_empty(ivc)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(ivc->wq, !ivc_rx_empty(ivc));
		if (ret) {
			if (ret != -ERESTARTSYS)
				dev_err(ivc->device,
					"wait_event_interruptible %d\n", ret);
			return ret;
		}
	}

	while (left > 0 && !ivc_rx_empty(ivc)) {

		chunk = ivc->qd.frame_size;
		if (chunk > left)
			chunk = left;
		ret = ivc_rx_read_user(ivc, buf, chunk);
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
	struct ivc_dev *ivc = filp->private_data;
	ssize_t done;
	size_t left, chunk;
	int ret = 0;

	done = 0;
	while (done < count) {

		left = count - done;

		if (left < ivc->qd.frame_size)
			chunk = left;
		else
			chunk = ivc->qd.frame_size;

		/* is queue full? */
		if (ivc_tx_full(ivc)) {

			/* check non-blocking mode */
			if (filp->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				break;
			}

			ret = wait_event_interruptible(ivc->wq,
					!ivc_tx_full(ivc));
			if (ret) {
				if (ret != -ERESTARTSYS)
					dev_err(ivc->device,
						"wait_event_interruptible %d\n",
							ret);
				break;
			}
		}

		ret = ivc_tx_write_user(ivc, buf, chunk);
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
	struct ivc_dev *ivc = filp->private_data;
	int mask = 0;

	poll_wait(filp, &ivc->wq, wait);

	if (!ivc_rx_empty(ivc))
		mask = POLLIN | POLLRDNORM;

	if (!ivc_tx_full(ivc))
		mask |= POLLOUT | POLLWRNORM;

	/* no exceptions */

	return mask;
}

static const struct file_operations ivc_fops = {
	.owner		= THIS_MODULE,
	.open		= ivc_dev_open,
	.release	= ivc_dev_release,
	.unlocked_ioctl	= ivc_dev_ioctl,
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

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->opened);
}

static ssize_t reserved_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ivc->reserved);
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

	return snprintf(buf, PAGE_SIZE, "%d\n", (int)ivc->loopback);
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

	ret = ivc_set_loopback(ivc, (int)mode);
	if (ret != 0)
		return ret;
	return len;
}

static ssize_t local_loopback_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	int ret;

	spin_lock(&ivc->lock);
	ret = ivc->local_loopback;
	spin_unlock(&ivc->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);

}

ssize_t local_loopback_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t len)
{
	struct ivc_dev *ivc = dev_get_drvdata(dev);
	struct ivc_dev *ivcd;
	struct tegra_hv_data *hvd = ivc->hvd;
	long int dest;
	int found, to_find;
	int ret;

	ret = kstrtol(buf, 10, &dest);
	if (ret)
		return -EINVAL;

	/* setting to the same value is a nop */
	if (dest == ivc->local_loopback)
		return len;

	/* can't bind self */
	if (dest == ivc->qd.id)
		return -EINVAL;

	found = 0;
	to_find = dest != -1 ? dest : ivc->local_loopback;

	/* add to the list */
	mutex_lock(&hvd->ivc_devs_lock);
	list_for_each_entry(ivcd, &hvd->ivc_devs, list) {
		if (ivcd->qd.id == to_find)
			goto found;
	}
	mutex_unlock(&hvd->ivc_devs_lock);

	return -ENODEV;

found:
	/* lock low to high id to avoid deadlocks */
	if (ivcd->qd.id < ivc->qd.id) {
		spin_lock(&ivcd->lock);
		spin_lock(&ivc->lock);
	} else {
		spin_lock(&ivc->lock);
		spin_lock(&ivcd->lock);
	}

	if (dest != -1) {
		/* link them together */
		ivc->loopback_rx_cbuf = ivcd->rx_cbuf;
		ivc->loopback_tx_cbuf = ivcd->tx_cbuf;
		ivc->loopback_irq = ivcd->irq;
		ivc->local_loopback = ivcd->qd.id;

		ivcd->loopback_rx_cbuf = ivc->rx_cbuf;
		ivcd->loopback_tx_cbuf = ivc->tx_cbuf;
		ivcd->loopback_irq = ivc->irq;
		ivcd->local_loopback = ivc->qd.id;
	} else {
		/* unlink them */
		ivc->loopback_rx_cbuf = NULL;
		ivc->loopback_tx_cbuf = NULL;
		ivc->loopback_irq = 0;
		ivc->local_loopback = -1;

		ivcd->loopback_rx_cbuf = NULL;
		ivcd->loopback_tx_cbuf = NULL;
		ivcd->loopback_irq = 0;
		ivcd->local_loopback = -1;
	}

	/* unlock high to low */
	if (ivcd->qd.id < ivc->qd.id) {
		spin_unlock(&ivc->lock);
		spin_unlock(&ivcd->lock);
	} else {
		spin_unlock(&ivcd->lock);
		spin_unlock(&ivc->lock);
	}
	mutex_unlock(&hvd->ivc_devs_lock);

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
	__ATTR(local_loopback, 0644, local_loopback_show, local_loopback_store),
	__ATTR_NULL
};

static int tegra_hv_add_ivc(struct tegra_hv_data *hvd,
		struct tegra_hv_queue_data *qd)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct ivc_dev *ivc;
	struct guest_ivc_info *givci;
	int ret;

	/* sanity check */
	if (qd->peers[0] != hvd->guestid && qd->peers[1] != hvd->guestid)
		return -EINVAL;

	ivc = kzalloc(sizeof(*ivc), GFP_KERNEL);
	if (ivc == NULL) {
		dev_err(dev, "Failed to allocate ivc structure\n");
		return -ENOMEM;
	}
	ivc->hvd = hvd;

	/*
	 * Make another copy of qd in the ivc_dev because qd is presumed to be
	 * in local, temporary memory.
	 */
	ivc->qd = *qd;

	spin_lock_init(&ivc->lock);
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

	/*
	 * cbufs are already initialized
	 * we only have to assign rx/tx
	 * peer[0] it's rx, tx
	 * peer[1] it's tx, rx
	 * both queues are of the same size
	 */
	if (hvd->guestid == qd->peers[0]) {
		ivc->other_guestid = qd->peers[1];
		givci = &hvd->guest_ivc_info[ivc->other_guestid];
		ivc->rx_cbuf = (void *)givci->shd + qd->offset;
		ivc->tx_cbuf = (void *)ivc->rx_cbuf + qd->size;
	} else {
		ivc->other_guestid = qd->peers[0];
		givci = &hvd->guest_ivc_info[ivc->other_guestid];
		ivc->tx_cbuf = (void *)givci->shd + qd->offset;
		ivc->rx_cbuf = (void *)ivc->tx_cbuf + qd->size;
	}

	if (!hvd->emulation) {
		/* IRQ# of this IVC channel */
		ivc->irq = tegra_hv_get_queue_irq(hvd, ivc->other_guestid,
				ivc->qd.id);
	}

	/* no loopback yet */
	ivc->local_loopback = -1;

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

/* reserve a channel for internal kernel use */
struct ivc_dev *ivc_reserve(struct tegra_hv_data *hvd, int id)
{
	struct ivc_dev *ivc;
	int found;

	found = 0;

	mutex_lock(&hvd->ivc_devs_lock);
	list_for_each_entry(ivc, &hvd->ivc_devs, list) {
		if (ivc->qd.id != id)
			continue;

		spin_lock(&ivc->lock);
		if (!ivc->opened && !ivc->reserved && !ivc->loopback) {
			ivc->reserved = 1;
			found = 1;
		}
		spin_unlock(&ivc->lock);

		if (found)
			goto out;
	}
	ivc = NULL;
out:
	mutex_unlock(&hvd->ivc_devs_lock);

	return ivc;
}

int ivc_unreserve(struct ivc_dev *ivc)
{
	struct tegra_hv_data *hvd;
	int found;

	if (ivc == NULL)
		return -EINVAL;

	hvd = ivc->hvd;

	found = 0;
	spin_lock(&ivc->lock);
	if (!ivc->opened && ivc->reserved) {
		ivc->reserved = 0;
		found = 1;
	}
	spin_unlock(&ivc->lock);

	return found ? 0 : -ENODEV;
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
				cbuf_rmb();
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

static int tegra_hv_perform_local_loopback(struct tegra_hv_data *hvd)
{
	struct platform_device *pdev = hvd->pdev;
	struct device *dev = &pdev->dev;
	struct ivc_dev *ivc, *ivct[2];
	u32 loop[2];
	int i;

	/* iterate over the loopback map */
	mutex_lock(&hvd->ivc_devs_lock);
	for (i = 0;
		of_property_read_u32_index(dev->of_node,
			"local-loopbacks", i * 2, loop) == 0 &&
		of_property_read_u32_index(dev->of_node,
			"local-loopbacks", i * 2 + 1, loop + 1) == 0; i++) {

		ivct[0] = ivct[1] = NULL;

		list_for_each_entry(ivc, &hvd->ivc_devs, list) {
			if (ivc->qd.id == loop[0])
				ivct[0] = ivc;
			if (ivc->qd.id == loop[1])
				ivct[1] = ivc;
		}

		/* this is normal for non-guest loopback, so no message */
		if (ivct[0] == NULL || ivct[1] == NULL)
			continue;

		/* link them together */
		ivct[0]->loopback_rx_cbuf = ivct[1]->rx_cbuf;
		ivct[0]->loopback_tx_cbuf = ivct[1]->tx_cbuf;
		ivct[0]->loopback_irq = ivct[1]->irq;
		ivct[0]->local_loopback = ivct[1]->qd.id;

		ivct[1]->loopback_rx_cbuf = ivct[0]->rx_cbuf;
		ivct[1]->loopback_tx_cbuf = ivct[0]->tx_cbuf;
		ivct[1]->loopback_irq = ivct[0]->irq;
		ivct[1]->local_loopback = ivct[0]->qd.id;

		dev_info(dev, "Local looback on %d <-> %d\n",
				ivct[0]->qd.id, ivct[1]->qd.id);
	}

	mutex_unlock(&hvd->ivc_devs_lock);

	return 0;
}

static int tegra_hv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tegra_hv_data *hvd;
	struct guest_ivc_info *givci;
	int i, ret;

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

	/* we're emulating only when running on non-hyp mode */
	hvd->emulation = !is_tegra_hypervisor_mode();

	if (!hvd->emulation) {
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
			dev_err(dev, "nguests=%d > %d (clamping to %d)\n",
					hvd->nguests, NGUESTS_MAX, NGUESTS_MAX);
			hvd->nguests = NGUESTS_MAX;
		}

	} else {
		dev_info(dev, "No hypervisor mode; loopback only\n");
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

	if (!hvd->emulation) {
		/* For all advertised guests, read IVC comm info */
		for (i = 0; i < hvd->nguests; i++) {
			givci = &hvd->guest_ivc_info[i];

			/* Get info for this guest */
			ret = hyp_read_ivc_info(&givci->res, i);
			if (ret != 0) {
				dev_warn(dev,
					"No hyp shared memory for guest #%d\n",
					i);
				continue;
			}

			/* Map segment for this guest  */
			givci->shmem = ioremap_cached(givci->res.base,
						givci->res.size);
			if (givci->shmem == NULL) {
				dev_err(dev,
					"guest #%d, shmem ioremap failed\n", i);

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

	} else {
		/* pretend we're guest #0 and nguests=1 */
		hvd->guestid = 0;
		hvd->nguests = 1;
		givci = &hvd->guest_ivc_info[0];

		memset(givci, 0, sizeof(*givci));
		givci->res.size = SZ_1M;      /* TODO: Hardcoded at 1M */
		givci->shd = kmalloc(givci->res.size, GFP_KERNEL);
		if (givci->shd == NULL) {
			dev_err(dev, "Failed to allocate local memory\n");
			ret = -ENOMEM;
			goto out_class_rel;
		}
		givci->server = 1;

		dev_warn(dev, "Non-hypervisor mode with 1MB of memory\n");
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

	ret = tegra_hv_perform_local_loopback(hvd);
	if (ret != 0) {
		dev_err(dev, "Local loopback failed\n");
		goto out_unreg_chr;
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
	if (!hvd->emulation) {
		for (i = 0; i < hvd->nguests; i++)
			iounmap(hvd->guest_ivc_info[i].shmem);
	} else {
		/* sanity checks */
		BUG_ON(hvd->guestid != 0);
		BUG_ON(hvd->nguests != 1);

		/* free the locally allocated memory */
		kfree(hvd->guest_ivc_info[0].shd);
	}

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

	if (!hvd->emulation) {
		/* Unmap mapped areas for all guests */
		for (i = 0; i < hvd->nguests; i++)
			iounmap(hvd->guest_ivc_info[i].shmem);
	} else {
		/* sanity checks */
		BUG_ON(hvd->guestid != 0);
		BUG_ON(hvd->nguests != 1);

		/* free the locally allocated memory */
		kfree(hvd->guest_ivc_info[0].shd);
	}

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

static irqreturn_t ivc_dev_cookie_irq_handler(int irq, void *data)
{
	struct ivc_dev *ivc = data;
	struct tegra_hv_ivc_cookie *ivck = &ivc->cookie;

	/* there are data in the queue, callback */
	if (ivc->cookie_ops->rx_rdy && !ivc_rx_empty(ivc))
		ivc->cookie_ops->rx_rdy(ivck);

	/* there is space in the queue to write, callback */
	if (ivc->cookie_ops->tx_rdy && !ivc_tx_full(ivc))
		ivc->cookie_ops->tx_rdy(ivck);

	return IRQ_HANDLED;
}

struct tegra_hv_ivc_cookie *tegra_hv_ivc_reserve(struct device_node *dn,
		int id, const struct tegra_hv_ivc_ops *ops)
{
	struct platform_device *pdev = NULL;
	struct tegra_hv_data *hvd = NULL;
	struct ivc_dev *ivc = NULL;
	struct tegra_hv_ivc_cookie *ivck = NULL;
	int ret;

	ret = -ENODEV;

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

	if (pdev != NULL)
		hvd = platform_get_drvdata(pdev);

	if (pdev == NULL || hvd == NULL) {
		if (pdev != NULL && hvd == NULL) {
			pr_err("%s: Called too early (return EPROBE_DEFER)\n",
					__func__);
			ret = -EPROBE_DEFER;
		}
		goto out;
	}

	ivc = ivc_reserve(hvd, id);
	if (ivc == NULL)
		goto out;

	ivck = &ivc->cookie;
	ivck->irq = ivc->irq;
	ivck->peer_vmid = ivc->other_guestid;
	ivck->nframes = ivc->qd.nframes;
	ivck->frame_size = ivc->qd.frame_size;

	ivc->cookie_ops = ops;

	if (ivc->cookie_ops) {
		/* there are data in the queue, callback */
		if (ivc->cookie_ops->rx_rdy &&
				!ivc_rx_empty(ivc))
			ivc->cookie_ops->rx_rdy(ivck);

		/* there is space in the queue to write, callback */
		if (ivc->cookie_ops->tx_rdy &&
				!ivc_tx_full(ivc))
			ivc->cookie_ops->tx_rdy(ivck);

		if (!ivc->hvd->emulation) {
			/* request our irq */
			ret = devm_request_irq(
					ivc->device, ivc->irq,
					ivc_dev_cookie_irq_handler,
					0, dev_name(ivc->device), ivc);
			if (ret < 0)
				goto out;
		}
	}

out:
	platform_device_put(pdev);

	if (ivck == NULL)
		return ERR_PTR(ret);

	/* return pointer to the cookie */
	return ivck;
}
EXPORT_SYMBOL(tegra_hv_ivc_reserve);

int tegra_hv_ivc_unreserve(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc;

	if (ivck == NULL)
		return -EINVAL;

	ivc = cookie_to_ivc_dev(ivck);

	if (ivc->cookie_ops) {
		if (!ivc->hvd->emulation)
			devm_free_irq(ivc->device, ivc->irq, ivc);
	}

	return ivc_unreserve(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_unreserve);

int tegra_hv_ivc_write(struct tegra_hv_ivc_cookie *ivck,
		const void *buf, int size)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return ivc_tx_write(ivc, buf, size);
}
EXPORT_SYMBOL(tegra_hv_ivc_write);

int tegra_hv_ivc_read(struct tegra_hv_ivc_cookie *ivck, void *buf, int size)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return ivc_rx_read(ivc, buf, size);
}
EXPORT_SYMBOL(tegra_hv_ivc_read);

int tegra_hv_ivc_can_read(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return !ivc_rx_empty(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_can_read);

int tegra_hv_ivc_can_write(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return !ivc_tx_full(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_can_write);

int tegra_hv_ivc_tx_empty(struct tegra_hv_ivc_cookie *ivck)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return ivc_tx_empty(ivc);
}
EXPORT_SYMBOL(tegra_hv_ivc_tx_empty);

int tegra_hv_ivc_set_loopback(struct tegra_hv_ivc_cookie *ivck, int mode)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return ivc_set_loopback(ivc, mode);
}
EXPORT_SYMBOL(tegra_hv_ivc_set_loopback);

int tegra_hv_ivc_perform_loopback(struct tegra_hv_ivc_cookie *ivck)
{
	return ivc_perform_loopback(cookie_to_ivc_dev(ivck));
}
EXPORT_SYMBOL(tegra_hv_ivc_perform_loopback);

int tegra_hv_ivc_dump(struct tegra_hv_ivc_cookie *ivck)
{
	return ivc_dump(cookie_to_ivc_dev(ivck));
}
EXPORT_SYMBOL(tegra_hv_ivc_dump);

int tegra_hv_ivc_read_peek(struct tegra_hv_ivc_cookie *ivck,
		void *buf, int off, int count)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return ivc_rx_peek(ivc, buf, off, count);
}
EXPORT_SYMBOL(tegra_hv_ivc_read_peek);

void *tegra_hv_ivc_read_get_next_frame(struct tegra_hv_ivc_cookie *ivck)
{
	return ivc_rx_get_next_frame(cookie_to_ivc_dev(ivck));
}
EXPORT_SYMBOL(tegra_hv_ivc_read_get_next_frame);

int tegra_hv_ivc_read_advance(struct tegra_hv_ivc_cookie *ivck)
{
	return ivc_rx_advance(cookie_to_ivc_dev(ivck));
}
EXPORT_SYMBOL(tegra_hv_ivc_read_advance);

int tegra_hv_ivc_write_poke(struct tegra_hv_ivc_cookie *ivck,
		const void *buf, int off, int count)
{
	struct ivc_dev *ivc = cookie_to_ivc_dev(ivck);

	return ivc_tx_poke(ivc, buf, off, count);
}
EXPORT_SYMBOL(tegra_hv_ivc_write_poke);

void *tegra_hv_ivc_write_get_next_frame(struct tegra_hv_ivc_cookie *ivck)
{
	return ivc_tx_get_next_frame(cookie_to_ivc_dev(ivck));
}
EXPORT_SYMBOL(tegra_hv_ivc_write_get_next_frame);

int tegra_hv_ivc_write_advance(struct tegra_hv_ivc_cookie *ivck)
{
	return ivc_tx_advance(cookie_to_ivc_dev(ivck));
}
EXPORT_SYMBOL(tegra_hv_ivc_write_advance);

static int __init tegra_hv_mod_init(void)
{
	return platform_driver_register(&tegra_hv_driver);
}

static void __exit tegra_hv_mod_exit(void)
{
	platform_driver_unregister(&tegra_hv_driver);
}

subsys_initcall(tegra_hv_mod_init);
module_exit(tegra_hv_mod_exit);

MODULE_LICENSE("GPL");
