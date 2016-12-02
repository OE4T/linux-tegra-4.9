/*
 * NVDLA IOCTL for T194
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

#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_buffer.h"
#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"

#include "t194/t194.h"
#include "nvhost_queue.h"

#include "nvdla/nvdla.h"
#include "nvdla/nvdla_debug.h"
#include <linux/nvhost_nvdla_ioctl.h>
#include "dla_os_interface.h"

#define DEBUG_BUFFER_SIZE 0x100
#define FLCN_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */
#define ALIGNED_DMA(x) ((x >> 8) & 0xffffffff)

static DEFINE_DMA_ATTRS(attrs);

/**
 * struct nvdla_private per unique FD private data
 * @pdev		pointer to platform device
 * @queue		pointer to nvhost_queue
 * @buffers		pointer to nvhost_buffer
 */

struct nvdla_private {
	struct platform_device *pdev;
	struct nvhost_queue *queue;
	struct nvhost_buffers *buffers;
};

static int nvdla_pin(struct nvdla_private *priv, void *arg)
{
	u32 *handles;
	int err = 0;
	struct nvdla_pin_unpin_args *buf_list =
			(struct nvdla_pin_unpin_args *)arg;
	u32 count = buf_list->num_buffers;
	struct platform_device *pdev = priv->pdev;

	nvdla_dbg_info(pdev, "num of buffers [%d]", count);

	handles = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
			(count * sizeof(u32)))) {
		err = -EFAULT;
		goto nvdla_buffer_cpy_err;
	}

	err = nvhost_buffer_pin(priv->buffers, handles, count);

nvdla_buffer_cpy_err:
	kfree(handles);
	return err;
}

static int nvdla_unpin(struct nvdla_private *priv, void *arg)
{
	u32 *handles;
	int err = 0;
	struct nvdla_pin_unpin_args *buf_list =
			(struct nvdla_pin_unpin_args *)arg;
	u32 count = buf_list->num_buffers;
	struct platform_device *pdev = priv->pdev;

	nvdla_dbg_info(pdev, "num of buffers [%d]", count);

	handles = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
		(count * sizeof(u32)))) {
		err = -EFAULT;
		goto nvdla_buffer_cpy_err;
	}

	nvhost_buffer_unpin(priv->buffers, handles, count);

nvdla_buffer_cpy_err:
	kfree(handles);
	return err;
}

static int nvdla_ping(struct platform_device *pdev,
			   struct nvdla_ping_args *args)
{
	DEFINE_DMA_ATTRS(ping_attrs);
	dma_addr_t ping_pa;
	u32 *ping_va;
	int err = 0;

	/* make sure that device is powered on */
	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to power on\n");
		err = -ENODEV;
		goto fail_to_on;
	}

	/* allocate ping buffer */
	ping_va = dma_alloc_attrs(&pdev->dev,
				  DEBUG_BUFFER_SIZE, &ping_pa,
				  GFP_KERNEL, &ping_attrs);
	if (!ping_va) {
		nvdla_dbg_err(pdev, "dma memory allocation failed for ping");
		err = -ENOMEM;
		goto fail_to_alloc;
	}

	/* pass ping value to falcon */
	*ping_va = args->in_challenge;

	nvdla_dbg_info(pdev, "ping challenge [%d]", *ping_va);

	/* send ping cmd */
	err = nvdla_send_cmd(pdev, DLA_CMD_PING, ALIGNED_DMA(ping_pa), true);
	if (err) {
		nvdla_dbg_err(pdev, "failed to send ping command");
		goto fail_cmd;
	}

	/* out value should have (in_challenge * 4) */
	args->out_response = *ping_va;

	nvdla_dbg_info(pdev, "ping response [%d]", *ping_va);

	if (args->out_response != args->in_challenge*4) {
		nvdla_dbg_err(pdev, "ping cmd failed. Falcon is not active");
		err = -EINVAL;
	}

fail_cmd:
	if (ping_va)
		dma_free_attrs(&pdev->dev, DEBUG_BUFFER_SIZE,
			       ping_va, ping_pa, &attrs);
fail_to_alloc:
	nvhost_module_idle(pdev);
fail_to_on:

	return err;
}

static int nvdla_submit(struct nvdla_private *priv, void *arg)
{
	struct nvdla_submit_args *args =
			(struct nvdla_submit_args *)arg;
	struct nvdla_ioctl_submit_task __user *user_tasks;
	struct nvdla_ioctl_submit_task *local_tasks;
	struct platform_device *pdev;
	struct nvhost_queue *queue;
	struct nvhost_buffers *buffers;
	u32 num_tasks;
	struct nvdla_task *task;
	int err = 0, i = 0;

	if (!args || !priv)
		return -EINVAL;

	pdev = priv->pdev;
	queue = priv->queue;
	if (!queue)
		return -EINVAL;

	buffers = priv->buffers;

	user_tasks = (struct nvdla_ioctl_submit_task __user *)
			(uintptr_t)args->tasks;
	num_tasks = args->num_tasks;

	if (num_tasks == 0 || num_tasks > MAX_TASKS_PER_SUBMIT)
		return -EINVAL;

	nvdla_dbg_info(pdev, "num of tasks [%d]", num_tasks);

	/* copy descriptors */
	local_tasks = kcalloc(num_tasks, sizeof(*local_tasks),
			  GFP_KERNEL);
	if (!local_tasks)
		return -ENOMEM;

	if (copy_from_user(local_tasks, user_tasks,
			(num_tasks * sizeof(*user_tasks)))) {
		err = -EFAULT;
		goto fail_to_copy_task;
	}

	for (i = 0; i < num_tasks; i++) {

		nvdla_dbg_info(pdev, "submit [%d]th task", i + 1);

		/* allocate per task and update fields */
		task = nvdla_task_alloc(queue, buffers, &local_tasks[i]);
		if (IS_ERR(task)) {
			err = PTR_ERR(task);
			goto fail_to_task_alloc;
		}

		/* send job to engine */
		err = nvhost_queue_submit(queue, task);
		if (err)
			goto fail_to_submit_task;

		/* send fences to user */
		err = nvdla_send_postfences(task, user_tasks[i]);
		if (err)
			goto fail_to_send_postfences;
	}

	kfree(local_tasks);
	local_tasks = NULL;

	return 0;

fail_to_send_postfences:
fail_to_submit_task:
fail_to_task_alloc:
	/*TODO: traverse list in reverse and delete jobs */
fail_to_copy_task:
	kfree(local_tasks);
	local_tasks = NULL;
	return err;
}

static long nvdla_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	struct nvdla_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;
	u8 buf[NVDLA_IOCTL_MAX_ARG_SIZE] __aligned(sizeof(u64));
	int err = 0;

	/* check for valid IOCTL cmd */
	if ((_IOC_TYPE(cmd) != NVHOST_NVDLA_IOCTL_MAGIC) ||
	    (_IOC_NR(cmd) == _IOC_NR(0)) ||
	    (_IOC_NR(cmd) > NVDLA_IOCTL_LAST) ||
	    (_IOC_SIZE(cmd) > NVDLA_IOCTL_MAX_ARG_SIZE)) {
		return -ENOIOCTLCMD;
	}

	/* copy from user for read commands */
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;

	nvdla_dbg_fn(pdev, "priv:%p cmd:%u", priv, cmd);

	/* handle IOCTL cmd */
	switch (cmd) {
	case NVDLA_IOCTL_PING:
		err = nvdla_ping(pdev, (void *)buf);
		break;
	case NVDLA_IOCTL_PIN:
		err = nvdla_pin(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_UNPIN:
		err = nvdla_unpin(priv, (void *)buf);
		break;
	case NVDLA_IOCTL_SUBMIT:
		err = nvdla_submit(priv, (void *)buf);
		break;
	default:
		err = -ENOIOCTLCMD;
		break;
	}

	/* copy to user for write commands */
	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static int nvdla_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct nvdla_private *priv;
	int err = 0;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (unlikely(priv == NULL)) {
		err = -ENOMEM;
		goto err_alloc_priv;
	}

	file->private_data = priv;
	priv->pdev = pdev;

	nvdla_dbg_fn(pdev, "priv:%p", priv);

	/* add priv to client list */
	err = nvhost_module_add_client(pdev, priv);
	if (err < 0)
		goto err_add_client;

	priv->buffers = nvhost_buffer_init(pdev);
	if (IS_ERR(priv->buffers)) {
		err = PTR_ERR(priv->buffers);
		goto err_alloc_buffer;
	}

	priv->queue = nvhost_queue_alloc(nvdla_dev->pool,
					MAX_NVDLA_TASK_COUNT);
	if (IS_ERR(priv->queue)) {
		err = PTR_ERR(priv->queue);
		goto err_alloc_queue;
	}

	return nonseekable_open(inode, file);

err_alloc_queue:
	nvhost_module_remove_client(pdev, priv);
err_alloc_buffer:
	kfree(priv->buffers);
err_add_client:
	kfree(priv);
err_alloc_priv:
	return err;
}

static int nvdla_release(struct inode *inode, struct file *file)
{
	struct nvdla_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;

	nvdla_dbg_fn(pdev, "priv:%p", priv);

	nvhost_queue_abort(priv->queue);
	nvhost_queue_put(priv->queue);
	nvhost_buffer_put(priv->buffers);
	nvhost_module_remove_client(pdev, priv);

	kfree(priv);
	return 0;
}

const struct file_operations tegra_nvdla_ctrl_ops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = nvdla_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvdla_ioctl,
#endif
	.open = nvdla_open,
	.release = nvdla_release,
};
