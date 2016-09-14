/*
 * NVDLA queue and task management for T194
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
#include "chip_support.h"
#include "nvhost_acm.h"
#include "nvhost_queue.h"

#include "nvdla/nvdla.h"
#include "nvdla/nvdla_debug.h"
#include <linux/nvhost_nvdla_ioctl.h>
#include "dla_os_interface.h"

static DEFINE_DMA_ATTRS(attrs);

/* task management API's */
static int nvdla_get_fences(struct nvdla_ctrl_ioctl_submit_task user_task,
			struct nvdla_task *task)
{
	struct nvdla_fence __user *postfences =
		(struct nvdla_fence __user *)(uintptr_t)user_task.postfences;
	struct nvdla_fence __user *prefences =
		(struct nvdla_fence __user *)(uintptr_t)user_task.prefences;
	u32 num_postfences = user_task.num_postfences;
	u32 num_prefences = user_task.num_prefences;
	struct nvdla_fence fence;
	int err = 0;
	u32 i = 0;

	/* get pre fences */
	for (i = 0; i < num_prefences; i++, prefences++) {
		err = copy_from_user(&fence, prefences,
			sizeof(struct nvdla_fence));
		if (err)
			goto fail;

		if (fence.syncpoint_index == 0)
			goto fail;

		task->prefences[i].fence_type = fence.type;
		task->prefences[i].id = fence.syncpoint_index;
		task->prefences[i].val = fence.syncpoint_value;
	}

	/* get post fences */
	for (i = 0; i < num_postfences; i++, postfences++) {
		err = copy_from_user(&fence, postfences,
			sizeof(struct nvdla_fence));
		if (err)
			goto fail;

		if (fence.syncpoint_index == 0)
			goto fail;

		task->postfences[i].fence_type = fence.type;
	}
fail:
	return err;
}

int nvdla_send_postfences(struct nvdla_task *task,
			struct nvdla_ctrl_ioctl_submit_task usr_task)
{
	struct nvdla_fence __user *postfences =
		(struct nvdla_fence __user *)(uintptr_t)usr_task.postfences;
	u32 num_postfences = usr_task.num_postfences;
	struct nvdla_fence fence;
	int err = 0;
	int i;

	/* send post fences */
	for (i = 0; i < num_postfences; i++, postfences++) {
		fence.syncpoint_index = task->postfences[i].id;
		fence.syncpoint_value = task->postfences[i].fence;
		fence.type = task->postfences[i].fence_type;

		err = copy_to_user(postfences, &fence,
				sizeof(struct nvdla_fence));
		if (err)
			goto fail;
	}
fail:
	return err;
}

static void task_free(struct kref *ref)
{
	struct nvdla_task *task = container_of(ref, struct nvdla_task, ref);
	struct platform_device *pdev = task->queue->pool->pdev;

	nvdla_dbg_info(pdev, "freeing task[%p]", task);

	/* free allocated task desc */
	if (task->task_desc) {
		dma_free_attrs(&pdev->dev, task->buf_size,
			task->task_desc, task->task_desc_pa,
			&attrs);
		task->task_desc = NULL;
	}

	/* free operation descriptor handle */
	kfree(task->memory_handles);

	/* finally free task */
	kfree(task);
}

void nvdla_task_put(struct nvdla_task *task)
{
	kref_put(&task->ref, task_free);
}

void nvdla_task_get(struct nvdla_task *task)
{
	kref_get(&task->ref);
}

static void nvdla_task_free_locked(struct nvdla_task *task)
{
	struct nvhost_queue *queue = task->queue;
	struct platform_device *pdev = queue->pool->pdev;

	nvdla_dbg_info(pdev,
		"task[%p] completed. syncpt[%d] fence[%d]",
		task, queue->syncpt_id, task->fence);

	/* give syncpoint reference */
	nvhost_syncpt_put_ref(task->sp, queue->syncpt_id);

	/* unpin submit ref */
	nvhost_buffer_submit_unpin(task->buffers,
		task->memory_handles, task->num_handles);

	/* update takslist */
	list_del(&task->list);

	/* give taks refs */
	nvdla_task_put(task);
}

static void nvdla_task_syncpt_reset(struct nvhost_syncpt *syncpt,
			u32 id, u32 fence)
{
	atomic_set(&syncpt->min_val[id], fence);
	syncpt_op().reset(syncpt, id);
	nvhost_syncpt_update_min(syncpt, id);
}

static void nvdla_queue_update(void *priv, int nr_completed)
{
	int task_complete;
	struct nvdla_task *task, *safe;
	struct nvhost_queue *queue = priv;
	struct platform_device *pdev = queue->pool->pdev;

	mutex_lock(&queue->list_lock);

	/* check which task(s) finished */
	list_for_each_entry_safe(task, safe, &queue->tasklist, list) {

		task_complete = nvhost_syncpt_is_expired(task->sp,
					queue->syncpt_id, task->fence);

		/* clean task and remove from list */
		if (task_complete)
			nvdla_task_free_locked(task);
	}
	/* put pm refcount */
	nvhost_module_idle_mult(pdev, nr_completed);

	mutex_unlock(&queue->list_lock);
}

#if 1
/* this is HACK/unreliable way to validate sem read/write operation on parker
 * this will be used until we move to pre-silicon
 */
dma_addr_t get_semaphore_pa(struct platform_device *pdev)
{
	int *buffer_va;
	dma_addr_t buffer_pa;
	DEFINE_DMA_ATTRS(attrs);

	buffer_va = dma_alloc_attrs(&pdev->dev, 8,
				&buffer_pa, GFP_KERNEL, &attrs);


	*buffer_va = 0;

	return buffer_pa;

}
#endif

struct nvdla_task *nvdla_task_alloc(struct nvhost_queue *queue,
			struct nvhost_buffers *buffers,
			struct nvdla_ctrl_ioctl_submit_task user_task)
{
	struct platform_device *pdev = queue->pool->pdev;
	u32 num_operations = user_task.num_operations;
	u32 num_postfences = user_task.num_postfences;
	u32 num_prefences = user_task.num_prefences;
	struct dla_action_semaphore *postaction;
	struct dla_action_semaphore *preaction;
	struct dla_task_descriptor *task_desc;
	struct dla_action_list *postactionl;
	struct dla_action_list *preactionl;
	struct dla_action_opcode *opcode;
	struct nvdla_task *task = NULL;
	uint16_t postactionlist_of;
	size_t postactionlist_size;
	uint16_t preactionlist_of;
	size_t preactionlist_size;
	uint16_t postactionl_of;
	uint16_t preactionl_of;
	dma_addr_t *dma_addr;
	dma_addr_t buffer_pa;
	size_t task_size;
	size_t buf_size;
	size_t *dma_size;
	u32 *buffer_va;
	void *mem;
	int err;
	int i;

	nvdla_dbg_fn(pdev, "");

	/* allocate task resource */
	task_size = sizeof(struct nvdla_task) +
		(num_prefences + num_postfences) *
		sizeof(struct nvdla_task_fence);

	task = kzalloc(task_size, GFP_KERNEL);
	if (!task) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "task allocation failed");
		goto fail_to_alloc_task;
	}

	/* initialize task parameters */
	kref_init(&task->ref);
	task->queue = queue;
	task->sp = &nvhost_get_host(pdev)->syncpt;

	/* assign memory for local pre and post action lists */
	mem = task;
	mem += sizeof(struct nvdla_task);
	task->prefences = mem;
	mem += num_prefences * sizeof(struct nvdla_task_fence);
	task->postfences = mem;

	/* update local fences into task*/
	nvdla_get_fences(user_task, task);

	/* calculate size of task desc, actions and its list, buffers
	 * this is max possible size for updating task desc and
	 * and allocated mem size can be more than required size
	 */
	preactionlist_size =
			num_prefences * sizeof(struct dla_action_opcode) +
			num_prefences * sizeof(struct dla_action_semaphore) +
			sizeof(struct dla_action_opcode);

	postactionlist_size =
			num_postfences * sizeof(struct dla_action_opcode) +
			num_postfences * sizeof(struct dla_action_semaphore) +
			sizeof(struct dla_action_opcode);

	buf_size = sizeof(struct dla_task_descriptor) +
		(2 * MAX_NUM_ACTION_LIST * sizeof(struct dla_action_list)) +
		preactionlist_size +
		postactionlist_size;

	nvdla_dbg_info(pdev, "num of prefences[%d] num of postfences[%d]",
			num_prefences, num_postfences);
	nvdla_dbg_info(pdev, "preaction list size[%zu]",
			preactionlist_size);
	nvdla_dbg_info(pdev, "postaction list size[%zu]",
			postactionlist_size);
	nvdla_dbg_info(pdev, "Total task desc size[%zu]", buf_size);

	/* allocate task descriptor */
	buffer_va = dma_alloc_attrs(&pdev->dev, buf_size, &buffer_pa,
				GFP_KERNEL, &attrs);

	if (!buffer_va) {
		dev_err(&pdev->dev, "dma memory allocation failed for task");
		err = -ENOMEM;
		goto fail_to_dma_alloc;
	}

	task->task_desc = (struct dla_task_descriptor *)(buffer_va);
	task_desc = task->task_desc;
	task->task_desc_pa = buffer_pa;
	task->buf_size = buf_size;

	/* update task desc fields */
	task_desc->version = DLA_DESCRIPTOR_VERSION;
	task_desc->engine_id = DLA_ENGINE_ID;
	task_desc->size = buf_size;

	/* below are actual number of action lists
	 * DLA has one preaction list and one postaction list
	 */
	task_desc->num_preactions = MAX_NUM_ACTION_LIST;
	task_desc->num_postactions = MAX_NUM_ACTION_LIST;

	task_desc->queue_id = queue->id;

	nvdla_dbg_info(pdev, "Queue id[%d]", task_desc->queue_id);

	/* get pre/post action list HEAD mem offset
	 * - preactions list HEAD stored after dla_task_descriptor
	 * - postactions list HEAD followed after preaction list head offset
	 * - DLA has only one list of actions for each of pre and post
	 */
	preactionl_of = sizeof(struct dla_task_descriptor);
	postactionl_of = preactionl_of + sizeof(struct dla_action_list);

	nvdla_dbg_info(pdev, "preaction meta offset[%d]", preactionl_of);
	nvdla_dbg_info(pdev, "postaction meta offset[%d]", postactionl_of);

	/* ..and send those through descriptor */
	task_desc->preactions = preactionl_of;
	task_desc->postactions = postactionl_of;

	/* actual preaction list offset update */
	preactionlist_of = postactionl_of + sizeof(struct dla_action_list);

	/* actual postaction list offset update */
	postactionlist_of = preactionlist_of + preactionlist_size;

	nvdla_dbg_info(pdev, "preaction list offset[%d]", preactionlist_of);
	nvdla_dbg_info(pdev, "postaction list offset[%d]", postactionlist_of);

	/* actually update lists data */
	mem = (char *)task_desc + preactionl_of;
	preactionl = (struct dla_action_list *)mem;
	preactionl->offset = preactionlist_of;
	preactionl->size = preactionlist_size;

	mem = (char *)task_desc + postactionl_of;
	postactionl = (struct dla_action_list *)mem;
	postactionl->offset = postactionlist_of;
	postactionl->size = postactionlist_size;

	/* fill all preactions */
	for (i = 0; i <= user_task.num_prefences; i++) {
		void *next = NULL;

		/* get next preaction base */
		next = (char *)task_desc + preactionlist_of +
		  i * (sizeof(struct dla_action_opcode) +
			sizeof(struct dla_action_semaphore));

		/* get base opcode */
		opcode = (struct dla_action_opcode *)next;

		/* update end of action list */
		if (i == user_task.num_prefences) {
			opcode->value = ACTION_OPCODE_TERMINATE;
			break;
		}

		/* set action type */
		opcode->value = ACTION_OPCODE_READ_SEM;

		/* get actual preaction address */
		preaction = (struct dla_action_semaphore *)
			((char *)opcode + sizeof(struct dla_action_opcode));

		/* update action */
		/* TODO: remove temp hack */
		preaction->address = get_semaphore_pa(pdev);
		preaction->value = task->prefences[i].val;
	}

	/* fill all postactions */
	for (i = 0; i < user_task.num_postfences; i++, postaction++) {
		void *next = NULL;

		/* get next post action base */
		next = (char *)task_desc + postactionlist_of +
		 i * (sizeof(struct dla_action_opcode) +
			sizeof(struct dla_action_semaphore));

		/* get base opcode */
		opcode = (struct dla_action_opcode *)next;

		/* update end of list */
		if (i == user_task.num_postfences) {
			opcode->value = ACTION_OPCODE_TERMINATE;
			break;
		}

		/* set action type */
		opcode->value = ACTION_OPCODE_WRITE_SEM;

		/* get actual post action mem */
		postaction = (struct dla_action_semaphore *)
			((char *)opcode + sizeof(struct dla_action_opcode));

		/* update action */
		/* TODO: remove temp hack */
		postaction->address = get_semaphore_pa(pdev);
	}

	if (num_operations) {
		task->buffers = buffers;

		task->num_handles = 2;
		task->memory_handles = kcalloc(task->num_handles, sizeof(u32),
				GFP_KERNEL);
		if (!task->memory_handles) {
			err = -ENOMEM;
			goto fail_to_alloc_handles;
		}

		dma_addr = kcalloc(task->num_handles, sizeof(dma_addr_t),
				GFP_KERNEL);
		if (!dma_addr) {
			err = -ENOMEM;
			goto fail_to_alloc_dma_addr;
		}

		dma_size = kcalloc(task->num_handles, sizeof(u32),
				GFP_KERNEL);
		if (!dma_size) {
			err = -ENOMEM;
			goto fail_to_alloc_dma_size;
		}

		task->memory_handles[0] = user_task.operation_desc.handle;
		task->memory_handles[1] = user_task.surface_desc.handle;
		err = nvhost_buffer_submit_pin(buffers, task->memory_handles,
					task->num_handles, dma_addr, dma_size);
		if (!err) {
			task_desc->operation_desc = dma_addr[0] +
					user_task.operation_desc.offset;
			task_desc->surface_desc = dma_addr[1] +
					user_task.surface_desc.offset;
			task_desc->num_operations = num_operations;
		}

		kfree(dma_addr);
		kfree(dma_size);
	}

	nvdla_dbg_info(pdev, "task[%p] initialized", task);

	return task;

fail_to_alloc_dma_size:
	kfree(dma_addr);
fail_to_alloc_dma_addr:
	kfree(task->memory_handles);
fail_to_alloc_handles:
fail_to_dma_alloc:
	kfree(task);
fail_to_alloc_task:
	return ERR_PTR(err);
}

/* Queue management API */
static int nvdla_queue_submit(struct nvhost_queue *queue, void *in_task)
{
	struct nvdla_task *task = (struct nvdla_task *)in_task;
	struct platform_device *pdev = queue->pool->pdev;
	uint32_t method_data;
	uint32_t method_id;
	int err = 0;

	nvdla_dbg_fn(pdev, "");

	/* get pm refcount */
	if (nvhost_module_busy(pdev))
		return -EINVAL;

	/* enable slice syncpoint increment from THI
	 * required for debugging in T18x
	 * TODO: remove this hack when moved to t19x platform
	 */
	host1x_writel(pdev, 0x2c, queue->syncpt_id | (1 << 10));

	mutex_lock(&queue->list_lock);

	/* get task ref and add to list */
	nvdla_task_get(task);
	list_add_tail(&task->list, &task->queue->tasklist);

	nvdla_dbg_info(pdev, "task[%p] added to list", task);

	/* get fence from nvhost */
	task->fence = nvhost_syncpt_incr_max(task->sp, queue->syncpt_id, 1);

	nvdla_dbg_fn(pdev, "syncpt[%d] fence[%d] task[%p]", queue->syncpt_id,
				task->fence, task);

	/* get syncpoint reference */
	nvhost_syncpt_get_ref(task->sp, queue->syncpt_id);

	/* enable INT_ON_COMPLETE and INT_ON_ERROR falcon interrupts */
	method_id = (DLA_CMD_SUBMIT_TASK & DLA_METHOD_ID_CMD_MASK) |
			(1 << DLA_INT_ON_COMPLETE_SHIFT) |
			(1 << DLA_INT_ON_ERROR_SHIFT);
	method_data = ((task->task_desc_pa >> 8) & 0xffffffff);

	/* register notifier with fence */
	err = nvhost_intr_register_notifier(pdev, queue->syncpt_id,
		task->fence, nvdla_queue_update, queue);
	if (err)
		goto fail_to_register;

	/* Pass fence as through 0th postfences */
	task->postfences[0].id = queue->syncpt_id;
	task->postfences[0].fence = task->fence;

	/* submit task to engine */
	err = nvdla_send_cmd(pdev, method_id, method_data, true);
	if (err)
		nvdla_task_syncpt_reset(task->sp, queue->syncpt_id, task->fence);

fail_to_register:
	mutex_unlock(&queue->list_lock);

	return err;
}

static int nvdla_queue_abort(struct nvhost_queue *queue)
{
	/* TBD: Abort pending tasks from the queue */

	return 0;
}

struct nvhost_queue_ops nvdla_queue_ops = {
	.abort = nvdla_queue_abort,
	.submit = nvdla_queue_submit,
};
