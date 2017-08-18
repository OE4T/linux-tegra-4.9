/*
 * PVA Ioctl Handling for T194
 *
 * Copyright (c) 2016-2017, NVIDIA Corporation.  All rights reserved.
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
#include "pva_queue.h"
#include "dev.h"
#include "nvhost_buffer.h"
#include "nvhost_acm.h"

/**
 * @brief pva_private - Per-fd specific data
 *
 * pdev		Pointer the pva device
 * queue	Pointer the struct nvhost_queue
 * buffer	Pointer to the struct nvhost_buffer
 */
struct pva_private {
	struct pva *pva;
	struct nvhost_queue *queue;
	struct nvhost_buffers *buffers;
};

/**
 * @brief	Copy a single task from userspace to kernel space
 *
 * This function copies fields from ioctl_task and performs a deep copy
 * of the task to kernel memory. At the same time, input values shall
 * be validated. This allows using all the fields without manually performing
 * copies of the structure and performing checks later.
 *
 * @param ioctl_task	Pointer to a userspace task that is copied
 *				to kernel memory
 * @param task		Pointer to a task that should be created
 * @return		0 on Success or negative error code
 *
 */
static int pva_copy_task(struct pva_ioctl_submit_task *ioctl_task,
			 struct pva_submit_task *task)
{
	int err = 0;

	if (ioctl_task->num_prefences > PVA_MAX_PREFENCES ||
	    ioctl_task->num_postfences > PVA_MAX_POSTFENCES ||
	    ioctl_task->num_input_task_status > PVA_MAX_INPUT_STATUS ||
	    ioctl_task->num_output_task_status > PVA_MAX_OUTPUT_STATUS ||
	    ioctl_task->num_input_surfaces > PVA_MAX_INPUT_SURFACES ||
	    ioctl_task->num_output_surfaces > PVA_MAX_OUTPUT_SURFACES ||
	    ioctl_task->num_pointers > PVA_MAX_POINTERS) {
		err = -EINVAL;
		goto err_out;
	}

	/*
	 * These fields are clear-text in the task descriptor. Just
	 * copy them.
	 */
	task->operation			= ioctl_task->operation;
	task->num_prefences		= ioctl_task->num_prefences;
	task->num_postfences		= ioctl_task->num_postfences;
	task->num_input_task_status	= ioctl_task->num_input_task_status;
	task->num_output_task_status	= ioctl_task->num_output_task_status;
	task->num_input_surfaces	= ioctl_task->num_input_surfaces;
	task->num_output_surfaces	= ioctl_task->num_output_surfaces;
	task->num_pointers		= ioctl_task->num_pointers;
	task->input_scalars		= ioctl_task->input_scalars;
	task->output_scalars		= ioctl_task->output_scalars;
	task->timeout			= ioctl_task->timeout;

#define COPY_FIELD(dst, src, num, type)					\
	do {								\
		if ((num) == 0) {					\
			break;						\
		}							\
		err = copy_from_user((dst),				\
				(void __user *)(src),			\
				(num) * sizeof(type));			\
		if (err < 0) {						\
			goto err_out;					\
		}							\
	} while (0)


	/* Copy the fields */
	COPY_FIELD(task->input_surfaces, ioctl_task->input_surfaces,
			task->num_input_surfaces,
			struct pva_surface);
	COPY_FIELD(task->output_surfaces, ioctl_task->output_surfaces,
			task->num_output_surfaces,
			struct pva_surface);
	COPY_FIELD(task->prefences, ioctl_task->prefences, task->num_prefences,
			struct pva_fence);
	COPY_FIELD(task->postfences, ioctl_task->postfences,
			task->num_postfences, struct pva_fence);
	COPY_FIELD(task->input_task_status, ioctl_task->input_task_status,
			task->num_input_task_status,
			struct pva_status_handle);
	COPY_FIELD(task->output_task_status, ioctl_task->output_task_status,
			task->num_output_task_status,
			struct pva_status_handle);
	COPY_FIELD(task->pointers, ioctl_task->pointers,
			task->num_pointers, struct pva_memory_handle);
#undef COPY_FIELD

err_out:
	return err;
}

/**
 * @brief	Submit a task to PVA
 *
 * This function takes the given list of tasks, converts
 * them into kernel internal representation and submits
 * them to the task queue. On success, it populates
 * the post-fence structures in userspace and returns 0.
 *
 * @param priv	PVA Private data
 * @param arg	ioctl data
 * @return	0 on Success or negative error code
 *
 */
static int pva_submit(struct pva_private *priv, void *arg)
{
	struct pva_ioctl_submit_args *ioctl_tasks_header =
		(struct pva_ioctl_submit_args *)arg;
	struct pva_ioctl_submit_task *ioctl_tasks = NULL;
	struct pva_submit_tasks tasks_header;
	struct pva_submit_task *task = NULL;
	int err = 0;
	int i;

	memset(&tasks_header, 0, sizeof(tasks_header));

	/* Sanity checks for the task heaader */
	if (ioctl_tasks_header->num_tasks > PVA_MAX_TASKS) {
		err = -EINVAL;
		goto err_check_num_tasks;
	}

	if (ioctl_tasks_header->version > 0) {
		err = -ENOSYS;
		goto err_check_version;
	}

	/* Allocate memory for the UMD representation of the tasks */
	ioctl_tasks = kcalloc(ioctl_tasks_header->num_tasks,
			sizeof(*ioctl_tasks), GFP_KERNEL);
	if (!ioctl_tasks) {
		err = -ENOMEM;
		goto err_alloc_task_mem;
	}

	/* Copy the tasks from userspace */
	err = copy_from_user(ioctl_tasks,
			(void __user *)ioctl_tasks_header->tasks,
			ioctl_tasks_header->num_tasks * sizeof(*ioctl_tasks));
	if (err < 0) {
		err = -EFAULT;
		goto err_copy_tasks;
	}

	/* Go through the tasks and make a KMD representation of them */
	for (i = 0; i < ioctl_tasks_header->num_tasks; i++) {

		struct nvhost_queue_task_mem_info task_mem_info;

		/* Allocate memory for the task and dma */
		err = nvhost_queue_alloc_task_memory(priv->queue,
							&task_mem_info);
		task = task_mem_info.kmem_addr;
		if ((err < 0) || !task)
			goto err_get_task_buffer;

		err = pva_copy_task(ioctl_tasks + i, task);
		if (err < 0)
			goto err_copy_tasks;

		task->pva = priv->pva;
		task->queue = priv->queue;
		task->buffers = priv->buffers;

		task->dma_addr = task_mem_info.dma_addr;
		task->va = task_mem_info.va;
		task->pool_index = task_mem_info.pool_index;

		tasks_header.tasks[i] = task;
		tasks_header.num_tasks += 1;
	}

	/* Populate header structure */
	tasks_header.flags = ioctl_tasks_header->flags;

	/* ..and submit them */
	err = nvhost_queue_submit(priv->queue, &tasks_header);

	if (err < 0) {
		goto err_submit_task;
	}

	/* Copy post-fences back to userspace */
	for (i = 0; i < ioctl_tasks_header->num_tasks; i++) {
		struct pva_fence __user *postfences =
				(struct pva_fence __user *)
				ioctl_tasks[i].postfences;

		task = tasks_header.tasks[i];
		err = copy_to_user(postfences,
				task->postfences, sizeof(struct pva_fence) *
				task->num_postfences);
		if (err < 0) {
			nvhost_warn(&priv->pva->pdev->dev,
					"Failed to copy fences to userspace");
		}
	}

	kfree(ioctl_tasks);
	return 0;

err_submit_task:
err_get_task_buffer:
err_copy_tasks:
	for (i = 0; i < tasks_header.num_tasks; i++) {
		task = tasks_header.tasks[i];
		/* Release memory that was allocated for the task */
		nvhost_queue_free_task_memory(task->queue, task->pool_index);
	}
err_alloc_task_mem:
	kfree(ioctl_tasks);
err_check_version:
err_check_num_tasks:
	return err;
}

/**
 * pva_queue_set_attr() - Set attribute to the queue
 *
 * @priv: PVA Private data
 * @arg: ioctl data
 *
 * This function set the attributes of the pv queue.
 */
static int pva_queue_set_attr(struct pva_private *priv, void *arg)
{
	struct pva_ioctl_queue_attr *ioctl_queue_attr =
		(struct pva_ioctl_queue_attr *)arg;
	struct pva_queue_attribute *attr;
	int err = 0;
	int id;

	/* Sanity checks for the task heaader */
	if (ioctl_queue_attr->id >= QUEUE_ATTR_MAX) {
		err = -ENOSYS;
		goto end;
	}

	id = ioctl_queue_attr->id;
	attr = priv->queue->attr;
	attr[id].pva = priv->pva;
	attr[id].id = id;
	attr[id].value = ioctl_queue_attr->val;

	/* Turn on the hardware */
	err = nvhost_module_busy(priv->pva->pdev);
	if (err)
		goto end;

	err = nvhost_queue_set_attr(priv->queue, &attr[id]);

	/* Drop PM runtime reference of PVA */
	nvhost_module_idle(priv->pva->pdev);
 end:
	return err;

}

static int pva_pin(struct pva_private *priv, void *arg)
{
	u32 *handles;
	int err = 0;
	int i = 0;
	struct dma_buf *dmabufs[PVA_MAX_PIN_BUFFERS];
	struct pva_pin_unpin_args *buf_list = (struct pva_pin_unpin_args *)arg;
	u32 count = buf_list->num_buffers;

	if (count > PVA_MAX_PIN_BUFFERS)
		return -EINVAL;

	handles = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
			(count * sizeof(u32)))) {
		err = -EFAULT;
		goto pva_buffer_cpy_err;
	}

	/* get the dmabuf pointer from the fd handle */
	for (i = 0; i < count; i++) {
		dmabufs[i] = dma_buf_get(handles[i]);
		if (IS_ERR_OR_NULL(dmabufs[i])) {
			err = -EFAULT;
			goto pva_buffer_get_err;
		}
	}

	err = nvhost_buffer_pin(priv->buffers, dmabufs, count);

pva_buffer_get_err:
	count = i;
	for (i = 0; i < count; i++)
		dma_buf_put(dmabufs[i]);

pva_buffer_cpy_err:
	kfree(handles);
	return err;
}

static int pva_unpin(struct pva_private *priv, void *arg)
{
	u32 *handles;
	int i = 0;
	int err = 0;
	struct dma_buf *dmabufs[PVA_MAX_PIN_BUFFERS];
	struct pva_pin_unpin_args *buf_list = (struct pva_pin_unpin_args *)arg;
	u32 count = buf_list->num_buffers;

	if (count > PVA_MAX_PIN_BUFFERS)
		return -EINVAL;

	handles = kcalloc(count, sizeof(u32), GFP_KERNEL);
	if (!handles)
		return -ENOMEM;

	if (copy_from_user(handles, (void __user *)buf_list->buffers,
			(count * sizeof(u32)))) {
		err = -EFAULT;
		goto pva_buffer_cpy_err;
	}

	/* get the dmabuf pointer and clean valid ones */
	for (i = 0; i < count; i++) {
		dmabufs[i] = dma_buf_get(handles[i]);
		if (IS_ERR_OR_NULL(dmabufs[i]))
			continue;
	}

	nvhost_buffer_unpin(priv->buffers, dmabufs, count);

	count = i;
	for (i = 0; i < count; i++)
		dma_buf_put(dmabufs[i]);

pva_buffer_cpy_err:
	kfree(handles);
	return err;
}

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
	pva_char.num_queues = MAX_PVA_QUEUE_COUNT;

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
	case PVA_IOCTL_PIN:
	{
		err = pva_pin(priv, buf);
		break;
	}
	case PVA_IOCTL_UNPIN:
	{
		err = pva_unpin(priv, buf);
		break;
	}
	case PVA_IOCTL_SUBMIT:
	{
		err = pva_submit(priv, buf);
		break;
	}
	case PVA_IOCTL_SET_QUEUE_ATTRIBUTES:
	{
		err = pva_queue_set_attr(priv, buf);
		break;
	}
	default:
		return -ENOIOCTLCMD;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static void pva_queue_set_default_prior_attr(struct pva_private *priv,
		struct pva_queue_attribute *attr)
{
	attr->pva = priv->pva;
	attr->id = QUEUE_ATTR_PRIORITY;
	attr->value = PVA_QUEUE_DEFAULT_PRIORITY;
}

static void pva_queue_set_default_vpu_mask_attr(struct pva_private *priv,
		struct pva_queue_attribute *attr)
{
	attr->pva = priv->pva;
	attr->id = QUEUE_ATTR_VPU;
	attr->value = PVA_QUEUE_DEFAULT_VPU_MASK;
}

static int pva_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata = container_of(inode->i_cdev,
					struct nvhost_device_data, ctrl_cdev);
	struct platform_device *pdev = pdata->pdev;
	struct pva_queue_attribute *attr;
	struct pva *pva = pdata->private_data;
	struct pva_private *priv;
	int err = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (priv == NULL) {
		err = -ENOMEM;
		goto err_alloc_priv;
	}

	attr = kzalloc(sizeof(*attr) * (QUEUE_ATTR_MAX - 1), GFP_KERNEL);
	if (!attr) {
		err = -ENOMEM;
		dev_info(&pdev->dev, "unable to allocate memory for attributes\n");
		goto err_alloc_attr;
	}

	file->private_data = priv;
	priv->pva = pva;

	/* add the pva client to nvhost */
	err = nvhost_module_add_client(pdev, priv);
	if (err < 0)
		goto err_add_client;

	priv->buffers = nvhost_buffer_init(pdev);
	if (IS_ERR(priv->buffers)) {
		err = PTR_ERR(priv->buffers);
		goto err_alloc_buffer;
	}

	priv->queue = nvhost_queue_alloc(pva->pool, MAX_PVA_TASK_COUNT,
		pva->submit_mode == PVA_SUBMIT_MODE_CHANNEL_CCQ);
	if (IS_ERR(priv->queue)) {
		err = PTR_ERR(priv->queue);
		goto err_alloc_queue;
	}


	priv->queue->attr = attr;

	pva_queue_set_default_prior_attr(priv, &attr[QUEUE_ATTR_PRIORITY]);

	pva_queue_set_default_vpu_mask_attr(priv, &attr[QUEUE_ATTR_VPU]);

	return nonseekable_open(inode, file);

err_alloc_queue:
	kfree(priv->buffers);
err_alloc_buffer:
	nvhost_module_remove_client(pdev, priv);
err_add_client:
	kfree(attr);
err_alloc_attr:
	kfree(priv);
err_alloc_priv:
	return err;
}

static int pva_release(struct inode *inode, struct file *file)
{
	struct pva_private *priv = file->private_data;

	nvhost_queue_put(priv->queue);
	nvhost_module_remove_client(priv->pva->pdev, priv);

	nvhost_buffer_release(priv->buffers);
	kfree(priv->queue->attr);
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
