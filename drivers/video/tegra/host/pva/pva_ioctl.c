/*
 * PVA driver for T194
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

/**
 * struct pva_private - Per-fd specific data
 *
 * @pdev:	Pointer the pva device
 */
struct pva_private {
	struct platform_device *pdev;
};

static long pva_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	return -EINVAL;
}

static int pva_open(struct inode *inode, struct file *file)
{
	struct pva_private *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL))
		return -ENOMEM;

	file->private_data = priv;

	return nonseekable_open(inode, file);
}

static int pva_release(struct inode *inode, struct file *file)
{
	struct pva_private *priv = file->private_data;

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


