/*
 * PVA cluster for T194
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/module.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "pva.h"
#include "pva_cluster.h"

/**
 * struct pva_private - Per-fd specific data
 *
 * @pdev:	Pointer the pva device
 */
struct pva_private {
	struct platform_device *pdev;
};

/**
 * struct pva_dev - Cluster list information
 *
 * @list:	List struct to link this to other pva structures
 * @pdev:	Pointer to PVA device
 */
struct pva_dev {
	struct list_head list;
	struct platform_device *pdev;
};

/**
 * struct pva_cluster_data - Data related to the PVA cluster
 *
 * @pva_list_mutex:	Mutex for pva_list variable
 * @pva_list:		Keep track of existing PVA devices
 * @devno:		Character device region for PVA
 * @device:		Device for the PVA cluster
 * @cdev:		Character device for the PVA cluster
 */
struct pva_cluster_data {
	struct list_head pva_list;
	struct mutex pva_list_mutex;

	dev_t devno;
	struct device *node;
	struct cdev cdev;
};

struct pva_cluster_data *pva_cluster;

static long pva_cluster_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	return -EINVAL;
}

static int pva_cluster_open(struct inode *inode, struct file *file)
{
	struct pva_private *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL))
		return -ENOMEM;

	file->private_data = priv;

	return nonseekable_open(inode, file);
}

static int pva_cluster_release(struct inode *inode, struct file *file)
{
	struct pva_private *priv = file->private_data;

	nvhost_dbg_fn("priv = %p", priv);

	kfree(priv);
	return 0;
}

static const struct file_operations tegra_pva_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = pva_cluster_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pva_cluster_ioctl,
#endif
	.open = pva_cluster_open,
	.release = pva_cluster_release,
};

#define NVHOST_NUM_CDEV 4
static int pva_cluster_init(struct platform_device *pdev)
{
	struct nvhost_master *host = nvhost_get_host(pdev);
	struct device *dev;
	int err = 0;

	/* Allocate space for the PVA cluster */
	pva_cluster = kzalloc(sizeof(*pva_cluster), GFP_KERNEL);
	if (!pva_cluster)
		return -ENOMEM;

	/* Initialize list for holding the PVA devices */
	INIT_LIST_HEAD(&pva_cluster->pva_list);
	mutex_init(&pva_cluster->pva_list_mutex);

	/* Allocate a character device region for PVA cluster */
	err = alloc_chrdev_region(&pva_cluster->devno, 0,
				  NVHOST_NUM_CDEV, IFACE_NAME);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to allocate devno\n");
		goto err_cluster_init;
	}

	/* Initialize the character device structures */
	cdev_init(&pva_cluster->cdev, &tegra_pva_ctrl_ops);
	pva_cluster->cdev.owner = THIS_MODULE;

	/* ..and add them */
	err = cdev_add(&pva_cluster->cdev, pva_cluster->devno, 1);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to add cdev\n");
		goto err_cdev_add;
	}

	/* Create the device for PVA */
	dev = device_create(host->nvhost_class, NULL, pva_cluster->devno, NULL,
			    IFACE_NAME "-pva-cluster");
	if (IS_ERR(dev)) {
		dev_err(&pdev->dev, "failed to create device for pva cluster\n");
		err = PTR_ERR(dev);
		goto err_device_create;
	}

	return 0;

err_device_create:
	cdev_del(&pva_cluster->cdev);
err_cdev_add:
	unregister_chrdev_region(pva_cluster->devno, NVHOST_NUM_CDEV);
err_cluster_init:
	kfree(pva_cluster);
	pva_cluster = NULL;

	return err;
}

int pva_cluster_add(struct platform_device *pdev)
{
	struct pva_dev *list;
	int err = 0;

	nvhost_dbg_fn("%s", pdev->name);

	if (!pva_cluster) {
		err = pva_cluster_init(pdev);
		if (err < 0) {
			dev_err(&pdev->dev, "Failed to initialize PVA cluster\n");
			return err;
		}
	}

	list = kzalloc(sizeof(struct pva_dev), GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	list->pdev = pdev;

	/* Add device to PVA cluster */
	mutex_lock(&pva_cluster->pva_list_mutex);
	list_add_tail(&list->list, &pva_cluster->pva_list);
	mutex_unlock(&pva_cluster->pva_list_mutex);

	return err;
}

int pva_cluster_remove(struct platform_device *pdev)
{
	struct pva_dev *pva_dev;

	nvhost_dbg_fn("%s", pdev->name);

	/* Remove device from PVA cluster list */
	mutex_lock(&pva_cluster->pva_list_mutex);
	list_for_each_entry(pva_dev, &pva_cluster->pva_list, list) {
		if (pva_dev->pdev == pdev) {
			list_del(&pva_dev->list);
			kfree(pva_dev);
			break;
		}
	}
	mutex_unlock(&pva_cluster->pva_list_mutex);

	return 0;
}
