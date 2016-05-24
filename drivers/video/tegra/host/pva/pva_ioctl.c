/*
 * PVA Ioctl Handling for T194
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <asm/ioctls.h>

#include <uapi/linux/nvhost_pva_ioctl.h>

#include "pva.h"
#include "dev.h"
#include "nvhost_acm.h"

/**
 * struct pva_private - Per-fd specific data
 *
 * @pdev:	Pointer the pva device
 * @queue:	Pointer the struct pva_queue
 *
 */
struct pva_private {
	struct platform_device *pdev;
	struct pva_queue *queue;
};

static int pva_get_characteristics(struct pva_private *priv,
		void *arg)
{
	/* TO DO :- Will remove these comments after implementation
	 * call the hw_config request here or on open one time
	 * This could be a place holder to get the R5 functionTable
	 * support if it is not set yet
	 */

	struct pva_characteristics_req pva_char_req;
	struct pva_characteristics pva_char;

	struct pva_characteristics_req *in_pva_char =
			(struct pva_characteristics_req *)arg;

	u64 in_size = in_pva_char->characteristics_size;
	u64 out_size = sizeof(struct pva_characteristics);
	int err = 0;

	memset(&pva_char, 0, out_size);
	pva_char.num_vpu = 2;

	/* if input_size more than output_size, copy kernel struct size */
	if (in_size > out_size)
		in_size = out_size;

	/* copy input_size of data to output*/
	pva_char_req.characteristics_filled = in_size;

	/* check whether the characteristics has NULL pointer */
	if (!in_pva_char->characteristics)
		return -EINVAL;

	err = copy_to_user((void __user *)in_pva_char->characteristics,
			&pva_char,
			in_size);

	return err;
}

static long pva_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct pva_private *priv = file->private_data;
	u8 buf[NVHOST_PVA_IOCTL_MAX_ARG_SIZE] __aligned(sizeof(u64));
	int err = 0;

	nvhost_dbg_fn("");

	if ((_IOC_TYPE(cmd) != NVHOST_PVA_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_PVA_IOCTL_LAST) ||
		(_IOC_SIZE(cmd) > NVHOST_PVA_IOCTL_MAX_ARG_SIZE))
		return -ENOIOCTLCMD;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {

	case PVA_IOCTL_CHARACTERISTICS:
	{
		err = pva_get_characteristics(priv, buf);
		break;
	}
	default:
		return -ENOIOCTLCMD;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}


static int pva_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct pva_private *priv;
	int err = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		err = -ENOMEM;
		goto err_alloc_priv;
	}

	file->private_data = priv;
	priv->pdev = pdev;

	/* add the pva client to nvhost */
	err = nvhost_module_add_client(pdev, priv);
	if (err < 0)
		goto err_add_client;

	priv->queue = pva_queue_alloc(pdev);
	if (IS_ERR(priv->queue))
		goto err_alloc_queue;

	return nonseekable_open(inode, file);

err_alloc_queue:
	nvhost_module_remove_client(pdev, priv);
err_add_client:
	kfree(priv);
err_alloc_priv:
	return err;
}

static int pva_release(struct inode *inode, struct file *file)
{
	struct pva_private *priv = file->private_data;

	pva_queue_abort(priv->queue);
	pva_queue_put(priv->queue);
	nvhost_module_remove_client(priv->pdev, priv);

	kfree(priv);

	return 0;
}

const struct file_operations tegra_pva_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = pva_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pva_ioctl,
#endif
	.open = pva_open,
	.release = pva_release,
};
